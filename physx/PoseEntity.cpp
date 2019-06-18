//
// Created by lasagnaphil on 19. 3. 11.
//

#include "PoseEntity.h"
#include "Utils.h"
#include "InputManager.h"
#include "Ray.h"
#include "Material.h"
#include "PhongRenderer.h"
#include "GizmosRenderer.h"

#include <imgui.h>
#include <tinyxml2.h>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <queue>

using namespace physx;
using namespace tinyxml2;

glm::vec3 stringToVec3(const char* str) {
    glm::vec3 v;
    std::sscanf(str, "%f %f %f", &v.x, &v.y, &v.z);
    return v;
}

glm::mat3 stringToMat3(const char* str) {
    glm::mat3 m;
    std::sscanf(str, "%f %f %f %f %f %f %f %f %f",
           &m[0][0], &m[0][1], &m[0][2],
           &m[1][0], &m[1][1], &m[1][2],
           &m[2][0], &m[2][1], &m[2][2]);
    return m;
}

BVHArticulationMap BVHArticulationMap::fromFile(const std::string &filename) {
    BVHArticulationMap map;

    XMLDocument doc;
    doc.LoadFile(filename.c_str());


    XMLNode* root = doc.FirstChild();
    if (root == nullptr) {
        fprintf(stderr, "Error while reading XML file %s!", filename.c_str());
        exit(EXIT_FAILURE);
    }
    XMLElement* jointNode = root->FirstChildElement("Joint");
    if (jointNode == nullptr) {
        fprintf(stderr, "No Joint element found in XML file %s!", filename.c_str());
        exit(EXIT_FAILURE);
    }

    auto findAttributeVec3 = [](XMLElement* node, const char* name) {
        const XMLAttribute* attr = node->FindAttribute(name);
        if (attr) {
            return stringToVec3(attr->Value());
        }
        else {
            return glm::vec3();
        }
    };

    map.skeletonName = jointNode->FindAttribute("name")->Value();
    while (jointNode != nullptr) {
        BVHArticulationMap::Joint joint;
        std::string jointType = jointNode->FindAttribute("type")->Value();
        if (jointType == "FreeJoint") {
            joint.type = BVHArticulationMap::FreeJoint;
        }
        else if (jointType == "BallJoint") {
            joint.type = BVHArticulationMap::BallJoint;
        }

        joint.name = jointNode->FindAttribute("name")->Value();
        joint.parentName = jointNode->FindAttribute("parent_name")->Value();
        joint.size = stringToVec3(jointNode->FindAttribute("size")->Value());
        joint.mass = jointNode->FindAttribute("mass")->FloatValue();
        joint.bvhNode = jointNode->FindAttribute("bvh")->Value();
        XMLElement* bodyPosNode = jointNode->FirstChildElement("BodyPosition");
        joint.bodyTranslation = stringToVec3(bodyPosNode->FindAttribute("translation")->Value());
        XMLElement* jointPosNode = jointNode->FirstChildElement("JointPosition");
        joint.jointTranslation = stringToVec3(jointPosNode->FindAttribute("translation")->Value());
        XMLElement* capsuleNode = jointNode->FirstChildElement("Capsule");
        if (capsuleNode) {
            joint.shape.type = ShapeType::Capsule;
            joint.shape.capsule.direction = stringToVec3(capsuleNode->FindAttribute("direction")->Value());
            joint.shape.capsule.offset = findAttributeVec3(capsuleNode, "offset");
            joint.shape.capsule.radius = capsuleNode->FindAttribute("radius")->FloatValue();
            joint.shape.capsule.height = capsuleNode->FindAttribute("height")->FloatValue();
        }
        XMLElement* cylinderNode = jointNode->FirstChildElement("Cylinder");
        if (cylinderNode) {
            joint.shape.type = ShapeType::Cylinder;
            joint.shape.cylinder.direction = stringToVec3(cylinderNode->FindAttribute("direction")->Value());
            joint.shape.cylinder.offset = findAttributeVec3(cylinderNode, "offset");
            joint.shape.cylinder.radius = cylinderNode->FindAttribute("radius")->FloatValue();
            joint.shape.cylinder.height = cylinderNode->FindAttribute("height")->FloatValue();
        }
        XMLElement* sphereNode = jointNode->FirstChildElement("Sphere");
        if (sphereNode) {
            joint.shape.type = ShapeType::Box;
            joint.shape.sphere.offset = findAttributeVec3(sphereNode, "offset");
            joint.shape.sphere.radius = sphereNode->FindAttribute("radius")->FloatValue();
        }
        XMLElement* boxNode = jointNode->FirstChildElement("Box");
        if (boxNode) {
            joint.shape.type = ShapeType::Box;
            joint.shape.box.offset = findAttributeVec3(boxNode, "offset");
            joint.shape.box.size = stringToVec3(boxNode->FindAttribute("size")->Value());
        }

        map.joints.push_back(joint);
        jointNode = jointNode->NextSiblingElement("Joint");
    }

    for (uint32_t i = 0; i < map.joints.size(); i++) {
        map.jointNameMapping[map.joints[i].name] = i;
    }

    return map;
}

glm::mat4 rotMatrixBetweenTwoVectors(const glm::vec3& a, const glm::vec3& b) {
    glm::vec3 v = glm::cross(a, b);
    float s2 = glm::dot(v, v);
    if (s2 < glm::epsilon<float>()) {
        return glm::mat4(1.0f);
    }
    else {
        // Rodrigue's formula
        float c = glm::dot(a, b);
        glm::mat3 vhat;
        vhat[0][0] = vhat[1][1] = vhat[2][2] = 0;
        vhat[2][1] = v[0]; vhat[1][2] = -v[0];
        vhat[0][2] = v[1]; vhat[2][0] = -v[1];
        vhat[1][0] = v[2]; vhat[0][1] = -v[2];
        return glm::mat4(glm::mat3(1.0f) + vhat + vhat*vhat*(1 - c)/(s2));
    }
}

glm::quat quatBetweenTwoVectors(const glm::vec3& a, const glm::vec3& b) {
    float w = glm::dot(a, b) + glm::sqrt(glm::length2(a) * glm::length2(b));
    glm::vec3 xyz = glm::cross(a, b);
    return glm::normalize(glm::quat(w, xyz));
}

glm::quat eulerToQuatXYZ(glm::vec3 eulerAngleXYZ) {
    glm::vec3 c = glm::cos(eulerAngleXYZ * 0.5f);
    glm::vec3 s = glm::sin(eulerAngleXYZ * 0.5f);

    glm::quat q;
    q.w = c.x * c.y * c.z + s.x * s.y * s.z;
    q.x = s.x * c.y * c.z - c.x * s.y * s.z;
    q.y = c.x * s.y * c.z + s.x * c.y * s.z;
    q.z = c.x * c.y * s.z - s.x * s.y * c.z;
    return q;
}

// Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
glm::vec3 quatToEulerXYZ(const glm::quat& q) {
    glm::vec3 r;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    r.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        r.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        r.y = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    r.z = atan2(siny_cosp, cosy_cosp);

    return r;
}

glm::vec3 swingTwistDecompose(const glm::quat &q) {
    glm::quat twist = q.x != 0.f ? glm::normalize(glm::quat(q.w, q.x, 0, 0)) : glm::identity<glm::quat>();
    glm::quat swing = q * glm::conjugate(twist);
    float theta0 = std::atan2(twist.x, 1.f + twist.w) * 4.f;
    float theta1 = std::atan2(swing.y, 1.f + swing.w) * 4.f;
    float theta2 = std::atan2(swing.z, 1.f + swing.w) * 4.f;
    return {theta0, theta1, theta2};
}

glm::quat swingTwistToQuat(glm::vec3 v) {
    glm::quat q = {glm::cos(v.x), glm::sin(v.x), 0, 0};
    q *= glm::quat {glm::cos(v.y), 0, glm::cos(v.y), 0};
    q *= glm::quat {glm::cos(v.z), 0, 0, glm::cos(v.z)};
    return q;
}

void PoseEntity::init() {
    lineVertices.resize(poseTree.numNodes);
    initialBoneTransforms.resize(poseTree.numNodes - 1);
    boneTransforms.resize(poseTree.numNodes - 1);

    boneMesh = Mesh::makeCube(glm::vec3(1.0f));
    boneMaterial = Resources::make<Material>();
    boneMaterial->ambient = {0.1f, 0.0f, 0.0f, 1};
    boneMaterial->diffuse = {0.5f, 0.0f, 0.0f, 1};
    boneMaterial->specular = {0.8f, 0.8f, 0.8f, 1};

    calculateInitialBoneTransforms();
    updateJointPositions();
}

void PoseEntity::initPhysX(PhysicsWorld& world) {
    articulation = world.physics->createArticulationReducedCoordinate();
    articulation->setSolverIterationCounts(32);

    initArticulationDefault(world);
    // initArticulationFromCMU(world);

    std::vector<PxArticulationLink*> links(articulation->getNbLinks());
    articulation->getLinks(links.data(), links.size());

    for (auto link : links) {
        if (link) {
            link->setActorFlag(PxActorFlag::eVISUALIZATION, true);
            PxShape* shape = nullptr;
            link->getShapes(&shape, 1);
            shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
        }
    }
    PxAggregate* aggregate = world.physics->createAggregate(articulation->getNbLinks(), false);
    aggregate->addArticulation(*articulation);
    world.scene->addAggregate(*aggregate);

    // Calculate dof starting index for each link
    // From PhysX documentation on articulations
    dofStarts.resize(links.size());
    dofStarts[0] = 0; //We know that the root link does not have a joint

    for(PxU32 i = 1; i < links.size(); ++i)
    {
        PxU32 llIndex = links[i]->getLinkIndex();
        PxU32 dofs = links[i]->getInboundJointDof();
        dofStarts[llIndex] = dofs;
    }

    dofCount = 0;
    for(PxU32 i = 1; i < links.size(); ++i)
    {
        PxU32 dofs = dofStarts[i];
        dofStarts[i] = dofCount;
        dofCount += dofs;
    }

    jointPosition.resize(dofCount);

    for (auto& [nodeIdx, link] : nodeToLinkPtr) {
        nodeToLinkIdx[nodeIdx] = link->getLinkIndex();
        if (nodeIdx < poseTree.numJoints) {
            linkToNodeIdx[link->getLinkIndex()] = nodeIdx;
        }
    }
}

std::tuple<PxArticulationLink*, PxTransform> PoseEntity::createArticulationLink(
        PhysicsWorld& world, const std::string& nodeName,  PxArticulationLink* parentLink, const PxTransform& nodeTransform)
{
    uint32_t nodeIdx = poseTree.findIdx(nodeName);
    PoseTreeNode& node = poseTree[nodeIdx];
    PxVec3 geomOffset = PxVec3(glm::length(node.offset), 0, 0);
    PxQuat geomRot;
    if (node.offset.x >= 0) {
        geomRot = GLMToPx(quatBetweenTwoVectors({1, 0, 0}, glm::normalize(node.offset)));
    }
    else {
        geomRot = GLMToPx(quatBetweenTwoVectors({-1, 0, 0}, glm::normalize(node.offset)));
    }
    PxTransform geomTransform = PxTransform(geomOffset, geomRot);

    PxArticulationLink* link = articulation->createLink(parentLink, nodeTransform);
    auto geometry = PxBoxGeometry(node.boneWidthX / 2, glm::length(node.offset) / 2, node.boneWidthZ / 2);
    PxRigidActorExt::createExclusiveShape(*link, geometry, *world.material);
    PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
    nodeToLinkPtr[nodeIdx] = link;

    if (parentLink) {
        auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        joint->setJointType(PxArticulationJointType::eSPHERICAL);
        joint->setParentPose(PxTransform(nodeTransform.p / 2, nodeTransform.q));
        if (node.offset.x >= 0) {
            joint->setChildPose(PxTransform(geomTransform.p / 2, geomTransform.q));
        }
        else {
            joint->setChildPose(PxTransform(-geomTransform.p / 2, geomTransform.q));
        }
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
        joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
        joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
    }

    return {link, geomTransform};
}

void PoseEntity::createArticulationTree(
        PhysicsWorld& world, uint32_t nodeIdx, PxArticulationLink* parentLink, const PxTransform& nodeTransform) {

    std::stack<std::tuple<uint32_t, PxArticulationLink*, PxTransform>> currentJoint;
    for (uint32_t idx : poseTree[nodeIdx].childJoints) {
        currentJoint.push(std::tuple(idx, parentLink, nodeTransform));
    }

    while (!currentJoint.empty()) {
        auto [jointIdx, parentLink, parentTransform] = currentJoint.top();
        currentJoint.pop();
        PoseTreeNode& node = poseTree[jointIdx];
        auto [link, transform] = createArticulationLink(world, node.name, parentLink, parentTransform);
        for (uint32_t idx : poseTree[jointIdx].childJoints) {
            if (idx < poseTree.numJoints) {
                currentJoint.push(std::tuple(idx, link, transform));
            }
        }
    }
}

void PoseEntity::initArticulationDefault(PhysicsWorld& world) {
    bodyToLinkMap.clear();
    bodyToTransformMap.clear();

    for (BVHArticulationMap::Joint& joint : bvhArtiMap.joints) {
        uint32_t nodeIdx = poseTree.findIdx(joint.bvhNode);
        if (nodeIdx == (uint32_t) -1) {
            fprintf(stderr, "BVH Node (%s) referenced in BvhArticulationMap not found!", joint.bvhNode.c_str());
            exit(EXIT_FAILURE);
        }
        PoseTreeNode& node = poseTree[nodeIdx];

        BVHArticulationMap::Joint& parentJoint = bvhArtiMap.getJoint(joint.parentName);
        PxArticulationLink* parentLink;
        PxTransform parentTransform;
        if (joint.parentName == "None") {
            parentLink = nullptr;
            parentTransform = PxTransform(PxVec3(), PxQuat(1));
        }
        else {
            parentLink = bodyToLinkMap[joint.parentName];
            parentTransform = bodyToTransformMap[joint.parentName];
        }

        PxTransform bodyTransform {GLMToPx(joint.bodyTranslation), PxQuat(1.0)};
        PxTransform t_ParentBodyToJoint = parentTransform.getInverse() * PxTransform(GLMToPx(joint.jointTranslation));
        PxTransform t_ChildBodyToJoint = bodyTransform.getInverse() * PxTransform(GLMToPx(joint.jointTranslation));

        PxArticulationLink* link = articulation->createLink(parentLink, parentTransform);
        auto geometry = PxBoxGeometry(joint.size.x, joint.size.y, joint.size.z);
        PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.material);
        PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
        nodeToLinkPtr[nodeIdx] = link;

        if (parentLink) {
            auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
            pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
            pxJoint->setParentPose(t_ParentBodyToJoint);
            pxJoint->setChildPose(t_ChildBodyToJoint);
            pxJoint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
            pxJoint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
            pxJoint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
        }

        bodyToLinkMap[joint.name] = link;
        bodyToTransformMap[joint.name] = bodyTransform;
        linkToBodyMap[link] = joint.name;
    }
}

void PoseEntity::initArticulationFromCMU(PhysicsWorld& world) {
    auto& hip = *poseTree["Hips"];

    // PxTransform rootTransform = PxTransform(GLMToPx(hip.offset));
    PxTransform rootTransform = PxTransform(GLMToPx(hip.offset), PxQuat(M_PI, {0, 1, 0}));
    auto [spineLink, spineTransform] = createArticulationLink(world, "Spine", nullptr, rootTransform);
    auto [spine1Link, spine1Transform] = createArticulationLink(world, "Spine1", spineLink, spineTransform);
    auto [neck1Link, neck1Transform] = createArticulationLink(world, "Neck1", spine1Link, spine1Transform);
    auto [headLink, headTransform] = createArticulationLink(world, "Head", neck1Link, neck1Transform);

    auto [leftArmLink, leftArmTransform] = createArticulationLink(world, "LeftArm", spine1Link, spine1Transform);
    auto [leftForeArmLink, leftForeArmTransform] = createArticulationLink(world, "LeftForeArm", leftArmLink, leftArmTransform);
    auto [leftHandLink, leftHandTransform] = createArticulationLink(world, "LeftHand", leftForeArmLink, leftForeArmTransform);
    auto [leftHandIndex1Link, leftHandIndex1Transform] = createArticulationLink(world, "LeftHandIndex1", leftHandLink, leftHandTransform);

    auto [rightArmLink, rightArmTransform] = createArticulationLink(world, "RightArm", spine1Link, spine1Transform);
    auto [rightForeArmLink, rightForeArmTransform] = createArticulationLink(world, "RightForeArm", rightArmLink, rightArmTransform);
    auto [rightHandLink, rightHandTransform] = createArticulationLink(world, "RightHand", rightForeArmLink, rightForeArmTransform);
    auto [rightHandIndex1Link, rightHandIndex1Transform] = createArticulationLink(world, "RightHandIndex1", rightHandLink, rightHandTransform);

    auto spineRootTransform = PxTransform(rootTransform.p, spineTransform.q);
    auto [leftUpLegLink, leftUpLegTransform] = createArticulationLink(world, "LeftUpLeg", spineLink, spineRootTransform);
    auto [leftLegLink, leftLegTransform] = createArticulationLink(world, "LeftLeg", leftUpLegLink, leftUpLegTransform);
    auto [leftFootLink, leftFootTransform] = createArticulationLink(world, "LeftFoot", leftLegLink, leftLegTransform);
    auto [leftToeBaseLink, leftToeBaseTransform] = createArticulationLink(world, "LeftToeBase", leftFootLink, leftFootTransform);

    auto [rightUpLegLink, rightUpLegTransform] = createArticulationLink(world, "RightUpLeg", spineLink, spineRootTransform);
    auto [rightLegLink, rightLegTransform] = createArticulationLink(world, "RightLeg", rightUpLegLink, rightUpLegTransform);
    auto [rightFootLink, rightFootTransform] = createArticulationLink(world, "RightFoot", rightLegLink, rightLegTransform);
    auto [rightToeBaseLink, rightToeBaseTransform] = createArticulationLink(world, "RightToeBase", rightFootLink, rightFootTransform);

    auto addSecondaryJoint = [&](const std::string& nodeName, PxArticulationLink* link) {
        uint32_t idx = poseTree.findIdx(nodeName);
        nodeToLinkPtr[idx] = link;
    };

    addSecondaryJoint("LowerBack", spineLink);
    addSecondaryJoint("LHipJoint", leftUpLegLink);
    addSecondaryJoint("RHipJoint", rightUpLegLink);
    addSecondaryJoint("Neck", neck1Link);
    addSecondaryJoint("LeftShoulder", leftArmLink);
    addSecondaryJoint("RightShoulder", rightArmLink);
    addSecondaryJoint("LeftFingerBase", leftHandIndex1Link);
    addSecondaryJoint("RightFingerBase", rightHandIndex1Link);

    // articulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);
}

void PoseEntity::resetPhysX(PhysicsWorld &world) {
    PxArticulationCache* cache = articulation->createCache();
    articulation->copyInternalStateToCache(*cache, PxArticulationCache::eALL);
    PxMemZero(cache->jointPosition, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointVelocity, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointAcceleration, sizeof(PxReal) * articulation->getDofs());

    articulation->applyCache(*cache, PxArticulationCache::eALL);
}

void PoseEntity::stop(PhysicsWorld& world) {
    PxArticulationCache* cache = articulation->createCache();
    articulation->copyInternalStateToCache(*cache, PxArticulationCache::eVELOCITY | PxArticulationCache::eACCELERATION | PxArticulationCache::eFORCE);
    PxMemZero(cache->jointVelocity, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointAcceleration, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointForce, sizeof(PxReal) * articulation->getDofs());
    articulation->applyCache(*cache, PxArticulationCache::eVELOCITY | PxArticulationCache::eACCELERATION | PxArticulationCache::eFORCE);
}

void PoseEntity::saveStateToPhysX(PhysicsWorld &world) {

    std::vector<PxArticulationLink*> links(articulation->getNbLinks());
    articulation->getLinks(links.data(), links.size());

    PxArticulationCache* cache = articulation->createCache();
    PxArticulationCacheFlags flags = PxArticulationCache::eROOT | PxArticulationCache::ePOSITION |
            PxArticulationCache::eVELOCITY | PxArticulationCache::eACCELERATION;
    articulation->copyInternalStateToCache(*cache, flags);
    PxMemZero(cache->jointPosition, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointVelocity, sizeof(PxReal) * articulation->getDofs());
    PxMemZero(cache->jointAcceleration, sizeof(PxReal) * articulation->getDofs());

    cache->rootLinkData->transform = PxTransform(GLMToPx(poseState.rootPos), GLMToPx(poseState.jointRot[0]));

    PoseState poseStateGlobal = poseState;
    std::stack<uint32_t> indexStack;
    for (auto childIdx : poseTree[0].childJoints) {
        indexStack.push(childIdx);
    }
    while (!indexStack.empty()) {
        uint32_t nodeIdx = indexStack.top();
        indexStack.pop();

        uint32_t parentIdx = poseTree[nodeIdx].parent;
        glm::quat q = poseState.jointRot[nodeIdx];
        glm::quat parentQ = poseStateGlobal.jointRot[parentIdx];
        poseStateGlobal.jointRot[nodeIdx] = parentQ * q;

        for (auto childIdx : poseTree[nodeIdx].childJoints) {
            if (childIdx < poseTree.numJoints) {
                indexStack.push(childIdx);
            }
        }
    }

    for (auto& joint : bvhArtiMap.joints) {
        if (joint.parentName == "None") continue;

        BVHArticulationMap::Joint& parentJoint = bvhArtiMap.getJoint(joint.parentName);
        PxArticulationLink* link = bodyToLinkMap[joint.name];
        PxArticulationLink* parentLink = bodyToLinkMap[parentJoint.name];

        glm::quat qGlobal = poseStateGlobal.jointRot[linkToNodeIdx[link->getLinkIndex()]];
        glm::quat qParentGlobal = poseStateGlobal.jointRot[linkToNodeIdx[parentLink->getLinkIndex()]];
        glm::quat q = qGlobal * glm::conjugate(qParentGlobal);

        glm::vec3 euler = quatToEulerXYZ(q);
        // glm::extractEulerAngleZYX(glm::mat4_cast(q), euler.z, euler.y, euler.x);
        uint32_t li = link->getLinkIndex();
        cache->jointPosition[dofStarts[li]] = euler.x;
        cache->jointPosition[dofStarts[li] + 1] = euler.y;
        cache->jointPosition[dofStarts[li] + 2] = euler.z;
    }

    articulation->applyCache(*cache, flags);
}

void PoseEntity::loadStateFromPhysX(PhysicsWorld &world) {
    PxArticulationCache* cache = articulation->createCache();
    articulation->copyInternalStateToCache(*cache, PxArticulationCache::eROOT | PxArticulationCache::ePOSITION);

    std::vector<PxArticulationLink*> links(articulation->getNbLinks());
    articulation->getLinks(links.data(), links.size());

    std::vector<PxU32> dofStarts(links.size());
    dofStarts[0] = 0; //We know that the root link does not have a joint

    poseState.rootPos = PxToGLM(cache->rootLinkData->transform.p);
    poseState.jointRot[0] = PxToGLM(cache->rootLinkData->transform.q);

    for (int li = 1; li < articulation->getNbLinks(); ++li) {
        float twist = cache->jointPosition[dofStarts[li]];
        float swing1 = cache->jointPosition[dofStarts[li] + 1];
        float swing2 = cache->jointPosition[dofStarts[li] + 2];
        glm::quat q = glm::eulerAngleZYX(swing2, swing1, twist);
        poseState.jointRot[linkToNodeIdx[li]] = q;
    }

    updateJointPositions();
}

void PoseEntity::update(float dt) {

}

void PoseEntity::queueBoneRender(PhongRenderer &renderer) {
    for (int i = 0; i < poseTree.numNodes - 1; i++) {
        renderer.queueRender({
            boneMesh,
            boneMaterial,
            transform->getWorldTransform() * boneTransforms[i]
        });
    }
}

void PoseEntity::queueGizmosRender(GizmosRenderer &renderer) {
}

void PoseEntity::calculateInitialBoneTransforms() {
    for (int i = 1; i < poseTree.numNodes; i++) {
        PoseTreeNode& node = poseTree.allNodes[i];

        glm::quat initialRotation;
        if (node.offset.y >= 0) {
            initialRotation = quatBetweenTwoVectors({0, 1, 0}, glm::normalize(node.offset));
        }
        else {
            initialRotation = quatBetweenTwoVectors({0, -1, 0}, glm::normalize(node.offset));
        }
        glm::mat4 initialScale = glm::scale(glm::vec3 {node.boneWidthX, glm::length(node.offset), node.boneWidthZ});
        glm::mat4 initialTrans;
        if (node.offset.y >= 0) {
            initialTrans = glm::translate(glm::vec3{0.0f, 0.5f, 0.0f});
        }
        else {
            initialTrans = glm::translate(glm::vec3{0.0f, -0.5f, 0.0f});
        }

        initialBoneTransforms[i - 1] = glm::mat4(initialRotation) * initialScale * initialTrans;
    }
}

void PoseEntity::updateJointPositions() {
    updateJointPositionsRecursive(0, glm::translate(poseState.rootPos));
}

void PoseEntity::updateJointPositionsRecursive(uint32_t jointID, glm::mat4 curTransform) {
    auto& joint = poseTree.allNodes[jointID];
    if (!joint.isEndSite) {
        curTransform = curTransform * glm::translate(joint.offset) * glm::mat4(poseState.jointRot[jointID]);

        // Base position of joint
        lineVertices[jointID] = glm::vec3(curTransform * glm::vec4(glm::vec3(0.0f), 1.0f));

        for (auto childID : joint.childJoints) {
            boneTransforms[childID - 1] = curTransform * initialBoneTransforms[childID - 1];
            updateJointPositionsRecursive(childID, curTransform);
        }
    }
    else {
        curTransform = curTransform * glm::translate(joint.offset);
        lineVertices[jointID] = glm::vec3(curTransform * glm::vec4(glm::vec3 {}, 1.0f));
    }
}

void PoseEntity::drawImGui() {
    ImGui::Begin("PoseEntity");

    if (ImGui::TreeNode("Pose")) {
        bool edited = false;
        for (uint32_t idx = 0; idx < poseTree.numJoints; idx++) {
            PoseTreeNode& node = poseTree.allNodes[idx];
            glm::vec3 v;
            glm::extractEulerAngleZYX(glm::mat4_cast(poseState.jointRot[idx]), v.z, v.y, v.x);
            edited |= ImGui::SliderFloat3(node.name.c_str(), (float*) &v, -M_PI, M_PI);
            if (edited) {
                poseState.jointRot[idx] = glm::quat(glm::eulerAngleZYX(v.z, v.y, v.x));
            }
        }
        if (edited) {
            updateJointPositions();
        }

        ImGui::TreePop();
    }


    if (ImGui::TreeNode("Articulation")) {
        bool edited = false;
        PxArticulationCache* cache = articulation->createCache();
        articulation->copyInternalStateToCache(*cache, PxArticulationCache::eROOT | PxArticulationCache::ePOSITION);

        for (auto &joint : bvhArtiMap.joints) {
            if (joint.parentName == "None") continue;
            auto link = bodyToLinkMap[joint.name];
            uint32_t linkIdx = link->getLinkIndex();
            edited |= ImGui::SliderFloat3(joint.name.c_str(), cache->jointPosition + dofStarts[linkIdx], -M_PI, M_PI);
        }
        if (edited) {
            articulation->applyCache(*cache, PxArticulationCache::eROOT | PxArticulationCache::ePOSITION);
        }

        ImGui::TreePop();
    }


    ImGui::End();
}

void PoseEntity::drawImGuiAnimationTree(uint32_t jointID) {
    PoseTreeNode& joint = poseTree.allNodes[jointID];
    if (!joint.isEndSite) {
        glm::vec3 pos = lineVertices[jointID];
        std::string label = string_format("%s [%f, %f, %f]###%s%d",
                joint.name.c_str(), pos.x, pos.y, pos.z, joint.name.c_str(), jointID);
        if (ImGui::TreeNode(label.c_str())) {
            for (uint32_t childID : joint.childJoints) {
                drawImGuiAnimationTree(childID);
            }
            ImGui::TreePop();
        }
    }
    else {
        glm::vec3 pos = lineVertices[jointID];
        std::string label = string_format("End Site %d [%f, %f, %f]###endsite%d",
                jointID, pos.x, pos.y, pos.z, jointID);
        if (ImGui::TreeNode(label.c_str())) {
            ImGui::TreePop();
        }
    }
}


