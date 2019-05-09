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

    articulationLinks.resize(poseTree.numJoints);

    // Create the three branches of the human articulation
    uint32_t lowerBackNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LowerBack");
    uint32_t leftHipJointNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "LHipJoint");
    uint32_t rightHipJointNodeIdx = poseTree.getChildIdx(poseTree.getRootNode(), "RHipJoint");

    // Add each hip to the bottom of the lower back node
    uint32_t spineJointIdx = poseTree[lowerBackNodeIdx].childJoints[0];
    addPhysXBodyRecursive(world, lowerBackNodeIdx, nullptr, PxTransform(0.0f, 10.0f, 0.0f));
    addPhysXBodyRecursive(world, leftHipJointNodeIdx, articulationLinks[spineJointIdx], PxTransform(0.0f, 0.0f, 0.0f));
    addPhysXBodyRecursive(world, rightHipJointNodeIdx, articulationLinks[spineJointIdx], PxTransform(0.0f, 0.0f, 0.0f));

    world.scene->addArticulation(*articulation);
}

void PoseEntity::addPhysXBodyRecursive(PhysicsWorld &world, uint32_t parentIdx,
        PxArticulationLink* parent, const PxTransform& parentTransform) {

    PoseTreeNode& parentJoint = poseTree[parentIdx];
    if (!parentJoint.isEndSite) {
        for (auto childIdx : parentJoint.childJoints) {
            auto& childJoint = poseTree[childIdx];
            PxArticulationLink* link = nullptr;
            PxTransform childTransform = parentTransform;
            if (glm::length(childJoint.offset) != 0.0f) {
                link = articulation->createLink(parent, parentTransform);
                auto geometry = PxBoxGeometry(parentJoint.boneWidthX/2, glm::length(childJoint.offset)/2, parentJoint.boneWidthZ/2);
                PxRigidActorExt::createExclusiveShape(*link, geometry, *world.material);
                PxRigidBodyExt::updateMassAndInertia(*link, 1.0f);

                childTransform = PxTransform(parentTransform.p + GLMToPx(childJoint.offset), parentTransform.q);
                auto pxJoint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
                if (pxJoint) {
                    pxJoint->setJointType(PxArticulationJointType::eSPHERICAL);
                    // pxJoint->setParentPose(parentTransform);
                    // pxJoint->setChildPose(childTransform);
                }
            }

            articulationLinks[childIdx] = link;
            addPhysXBodyRecursive(world, childIdx, link? link : parent, childTransform);
        }
    }
}

void PoseEntity::syncWithPhysX(PhysicsWorld &world) {
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
            poseState.jointRot[idx] = poseState.jointRot[poseTree[idx].parent];
        }
        for (int childIdx : poseTree[idx].childJoints) {
            indices.push(childIdx);
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
        glm::mat4 initialRotation, initialScale, initialTrans;

        PoseTreeNode& node = poseTree.allNodes[i];
        glm::vec3 offset = node.offset;
        glm::vec3 a = glm::vec3 {0, 1, 0};
        glm::vec3 b = glm::normalize(offset);
        glm::vec3 v = glm::cross(b, a);
        float s2 = glm::dot(v, v);
        if (s2 < glm::epsilon<float>()) {
            initialRotation = glm::mat4(1.0f);
        }
        else {
            // Rodrigue's formula
            float c = glm::dot(a, b);
            glm::mat3 vhat;
            vhat[0][0] = vhat[1][1] = vhat[2][2] = 0;
            vhat[2][1] = v[0]; vhat[1][2] = -v[0];
            vhat[0][2] = v[1]; vhat[2][0] = -v[1];
            vhat[1][0] = v[2]; vhat[0][1] = -v[2];
            initialRotation = glm::mat3(1.0f) + vhat + vhat*vhat*(1 - c)/(s2);
        }
        initialScale = glm::scale(glm::vec3 {node.boneWidthX, glm::length(offset), node.boneWidthZ});
        initialTrans = glm::translate(glm::vec3{0.0f, 0.5f, 0.0f});

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

