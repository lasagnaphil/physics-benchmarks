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
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp>

#include <queue>

using namespace physx;

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

    articulationLinks.resize(poseTree.numNodes, nullptr);

    initArticulationFromCMU(world);

    for (auto link : articulationLinks) {
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
}

std::tuple<PxArticulationLink*, PxTransform> PoseEntity::createArticulationLink(
        PhysicsWorld& world, const std::string& nodeName,  PxArticulationLink* parentLink, const PxTransform& nodeTransform)
{
    uint32_t nodeIdx = poseTree.findIdx(nodeName);
    PoseTreeNode& node = poseTree[nodeIdx];
    PxVec3 geomOffset = PxVec3(0, glm::length(node.offset), 0);
    PxQuat geomRot = GLMToPx(quatBetweenTwoVectors(glm::normalize(node.offset), {0, 1, 0}));
    PxTransform geomTransform = PxTransform(geomOffset, geomRot);

    PxArticulationLink* link = articulation->createLink(parentLink, nodeTransform);
    auto geometry = PxBoxGeometry(node.boneWidthX / 2, glm::length(node.offset) / 2, node.boneWidthZ / 2);
    PxRigidActorExt::createExclusiveShape(*link, geometry, *world.material);
    PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);
    articulationLinks[nodeIdx] = link;

    if (parentLink) {
        auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        joint->setJointType(PxArticulationJointType::eSPHERICAL);
        if (node.offset.y >= 0.0f) {
            joint->setParentPose(PxTransform(nodeTransform.p / 2, nodeTransform.q));
            joint->setChildPose(PxTransform(-geomTransform.p / 2, geomTransform.q));
        }
        else {
            joint->setParentPose(PxTransform(-nodeTransform.p / 2, nodeTransform.q));
            joint->setChildPose(PxTransform(geomTransform.p / 2, geomTransform.q));
        }
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
        joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
        joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);
    }

    return {link, geomTransform};
}

void PoseEntity::initArticulationFromCMU(PhysicsWorld& world) {
    auto& hip = *poseTree["Hips"];

    PxTransform rootTransform = PxTransform(PxVec3(0.0f, 2.0f, 0.0f) + GLMToPx(hip.offset));
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

    auto [leftUpLegLink, leftUpLegTransform] = createArticulationLink(world, "LeftUpLeg", spineLink, spineTransform);
    auto [leftLegLink, leftLegTransform] = createArticulationLink(world, "LeftLeg", leftUpLegLink, leftUpLegTransform);
    auto [leftFootLink, leftFootTransform] = createArticulationLink(world, "LeftFoot", leftLegLink, leftLegTransform);
    auto [leftToeBaseLink, leftToeBaseTransform] = createArticulationLink(world, "LeftToeBase", leftFootLink, leftFootTransform);

    auto [rightUpLegLink, rightUpLegTransform] = createArticulationLink(world, "RightUpLeg", spineLink, spineTransform);
    auto [rightLegLink, rightLegTransform] = createArticulationLink(world, "RightLeg", rightUpLegLink, rightUpLegTransform);
    auto [rightFootLink, rightFootTransform] = createArticulationLink(world, "RightFoot", rightLegLink, rightLegTransform);
    auto [rightToeBaseLink, rightToeBaseTransform] = createArticulationLink(world, "RightToeBase", rightFootLink, rightFootTransform);

    auto addSecondaryJoint = [&](const std::string& nodeName, PxArticulationLink* link) {
        uint32_t idx = poseTree.findIdx(nodeName);
        articulationLinks[idx] = link;
        secondaryJoints.insert(idx);
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

void PoseEntity::saveStateToPhysX(PhysicsWorld &world) {
    articulation->teleportRootLink(PxTransform(GLMToPx(poseState.rootPos), GLMToPx(poseState.jointRot[0])), true);

    for (uint32_t idx = 1; idx < articulationLinks.size(); idx++) {
        auto link = articulationLinks[idx];
        if (link && secondaryJoints.count(idx) == 1) {
            auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
            if (joint) {
                joint->setParentPose(PxTransform(
                        joint->getChildPose().p, GLMToPx(poseState.jointRot[idx])));
            }
        }
    }

    for (uint32_t idx = 1; idx < articulationLinks.size(); idx++) {
        auto link = articulationLinks[idx];
        if (link && secondaryJoints.count(idx) == 0) {
            auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
            if (joint) {
                joint->setChildPose(PxTransform(
                        joint->getChildPose().p, GLMToPx(poseState.jointRot[idx]) * joint->getChildPose().q));
            }
        }
    }
}

void PoseEntity::loadStateFromPhysX(PhysicsWorld &world) {
    auto link = articulationLinks[poseTree.findIdx("Spine")];
    auto pose = link->getGlobalPose();
    poseState.rootPos = PxToGLM(pose.p);
    poseState.jointRot[0] = PxToGLM(pose.q);

    std::stack<uint32_t> indices;
    for (uint32_t idx : poseTree[0].childJoints) {
        indices.push(idx);
    }

    while (!indices.empty()) {
        uint32_t idx = indices.top();
        indices.pop();
        PxArticulationLink* link = articulationLinks[idx];
        if (link) {
            poseState.jointRot[idx] = PxToGLM(link->getGlobalPose().q);
        }
        else {
            poseState.jointRot[idx] = glm::identity<glm::quat>();
            // poseState.jointRot[idx] = poseState.jointRot[poseTree[idx].parent];
        }
        for (int childIdx : poseTree[idx].childJoints) {
            if (!poseTree[childIdx].isEndSite) {
                indices.push(childIdx);
            }
        }
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

        glm::mat4 initialRotation = rotMatrixBetweenTwoVectors(glm::normalize(node.offset), {0, 1, 0});
        glm::mat4 initialScale = glm::scale(glm::vec3 {node.boneWidthX, glm::length(node.offset), node.boneWidthZ});
        glm::mat4 initialTrans = glm::translate(glm::vec3{0.0f, 0.5f, 0.0f});

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
    ImGui::Begin("PoseTree Debug");

    if (ImGui::CollapsingHeader("Animation Tree##PoseMesh")) {
        drawImGuiAnimationTree(0);
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

