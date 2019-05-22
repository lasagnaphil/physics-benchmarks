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

    articulationLinks.resize(poseTree.numNodes);

    // Create the three branches of the human articulation
    uint32_t lowerBackNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LowerBack");
    uint32_t leftHipJointNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LHipJoint");
    uint32_t rightHipJointNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "RHipJoint");

    // Add each hip to the bottom of the lower back node
    uint32_t spineJointIdx = poseTree[lowerBackNodeIdx].childJoints[0];
    addPhysXBodyRecursive(world, lowerBackNodeIdx, nullptr, PxTransform(0.0f, 0.0f, 0.0f));
    addPhysXBodyRecursive(world, leftHipJointNodeIdx, articulationLinks[spineJointIdx], PxTransform(0.0f, 0.0f, 0.0f));
    addPhysXBodyRecursive(world, rightHipJointNodeIdx, articulationLinks[spineJointIdx], PxTransform(0.0f, 0.0f, 0.0f));

    for (auto link : articulationLinks) {
        if (link) {
            link->setActorFlag(PxActorFlag::eVISUALIZATION, true);
            std::vector<PxShape*> shapes(link->getNbShapes());
            link->getShapes(shapes.data(), link->getNbShapes());
            for (auto shape : shapes) {
                shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
            }
        }
    }
    world.scene->addArticulation(*articulation);
}

void PoseEntity::addPhysXBodyRecursive(PhysicsWorld &world, uint32_t parentIdx,
        PxArticulationLink* parent, const PxTransform& parentTransform) {

    PoseTreeNode& parentJoint = poseTree[parentIdx];
    if (!parentJoint.isEndSite) {
        for (auto childIdx : parentJoint.childJoints) {
            auto& childJoint = poseTree[childIdx];

            PxArticulationLink* link = nullptr;
            PxArticulationJointReducedCoordinate* pxJoint = nullptr;
            PxTransform childTransform = parentTransform;

            if (glm::length(childJoint.offset) != 0.0f) {
                link = articulation->createLink(parent, parentTransform);
                link->setActorFlag(PxActorFlag::eVISUALIZATION, true);
                auto geometry = PxBoxGeometry(parentJoint.boneWidthX/2, glm::length(childJoint.offset)/2, parentJoint.boneWidthZ/2);
                PxShape* shape = PxRigidActorExt::createExclusiveShape(*link, geometry, *world.material);
                shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
                PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

                PxVec3 geometryOffset = PxVec3(0, glm::length(childJoint.offset), 0);
                PxQuat geometryRot = GLMToPx(quatBetweenTwoVectors(glm::normalize(childJoint.offset), {0, 1, 0}));
                childTransform = PxTransform(parentTransform.p + geometryOffset, geometryRot);
                pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
                if (pxJoint) {
                    pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                    pxJoint->setParentPose(parentTransform);
                    pxJoint->setChildPose(childTransform);
                }
            }

            articulationLinks[childIdx] = link;
            if (pxJoint) {
                linkToNodeIdx[link] = childIdx;
            }
            addPhysXBodyRecursive(world, childIdx, link? link : parent, childTransform);
        }
    }
}

void PoseEntity::saveStateToPhysX(PhysicsWorld &world) {
    uint32_t lowerBackNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LowerBack");
    uint32_t spineNodeIdx = poseTree.getChildIdx(poseTree[lowerBackNodeIdx], "Spine");

    auto spineLink = articulationLinks[spineNodeIdx];
    articulation->teleportRootLink(PxTransform(GLMToPx(poseState.rootPos), GLMToPx(poseState.jointRot[0])), true);

    std::vector<PxArticulationLink*> linkStack;
    {
        // insert child links to stack
        linkStack.resize(linkStack.size() + spineLink->getNbChildren());
        spineLink->getChildren(linkStack.data() + linkStack.size() - spineLink->getNbChildren(), spineLink->getNbChildren());
    }
    while (!linkStack.empty()) {
        auto link = linkStack[linkStack.size() - 1];
        linkStack.pop_back();
        uint32_t idx = linkToNodeIdx[link];
        auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        joint->setChildPose(PxTransform(link->getGlobalPose().p, GLMToPx(poseState.jointRot[idx])));

        // insert child links to stack
        linkStack.resize(linkStack.size() + link->getNbChildren());
        link->getChildren(linkStack.data() + linkStack.size() - link->getNbChildren(), link->getNbChildren());
    }
}

void PoseEntity::loadStateFromPhysX(PhysicsWorld &world) {
    uint32_t lowerBackNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LowerBack");
    uint32_t spineNodeIdx = poseTree.getChildIdx(poseTree[lowerBackNodeIdx], "Spine");
    auto link = articulationLinks[spineNodeIdx];
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

