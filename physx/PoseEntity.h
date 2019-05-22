//
// Created by lasagnaphil on 2019-03-10.
//

#ifndef MOTION_EDITING_POSEMESH_H
#define MOTION_EDITING_POSEMESH_H

#include "MotionClipData.h"

#include <stack>

#include <glad/glad.h>

#include "Storage.h"
#include "Shader.h"
#include "Transform.h"
#include "Mesh.h"
#include "LineMesh.h"
#include "Camera.h"

#include "PhysicsWorld.h"

struct PhongRenderer;
struct GizmosRenderer;

struct PoseEntity {
    PoseTree poseTree;
    PoseState poseState;

    physx::PxArticulationReducedCoordinate* articulation;
    std::vector<physx::PxArticulationLink*> articulationLinks;
    std::unordered_map<physx::PxArticulationLink*, uint32_t> linkToNodeIdx;

    // used for rendering
    std::vector<glm::vec3> lineVertices;
    std::vector<glm::mat4> initialBoneTransforms;
    std::vector<glm::mat4> boneTransforms;
    Ref<Mesh> boneMesh;
    Ref<Material> boneMaterial;
    Ref<Transform> transform;
    Camera* camera = nullptr;

    PoseEntity() = default;
    PoseEntity(PoseTree tree, PoseState state, Ref<Transform> transform, Camera* camera) :
        poseTree(std::move(tree)),
        poseState(std::move(state)),
        transform(transform), camera(camera)
    {}

    void init();

    void initPhysX(PhysicsWorld& world);

    void saveStateToPhysX(PhysicsWorld &world);

    void loadStateFromPhysX(PhysicsWorld &world);

    void update(float dt);

    void queueBoneRender(PhongRenderer &renderer);

    void queueGizmosRender(GizmosRenderer &renderer);

    void drawImGui();

    void updateJointPositions();

private:

    void addPhysXBodyRecursive(PhysicsWorld& world, uint32_t parentIdx,
            physx::PxArticulationLink* parent, const physx::PxTransform& parentTransform);

    void calculateInitialBoneTransforms();

    void updateJointPositionsRecursive(uint32_t jointID, glm::mat4 curTransform);

    void drawImGuiAnimationTree(uint32_t jointID);
};


#endif //MOTION_EDITING_POSEMESH_H
