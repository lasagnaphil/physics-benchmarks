//
// Created by lasagnaphil on 2019-03-10.
//

#ifndef MOTION_EDITING_POSEMESH_H
#define MOTION_EDITING_POSEMESH_H

#include "MotionClipData.h"

#include <stack>
#include <unordered_map>
#include <unordered_set>

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

struct BVHArticulationMap {
    enum JointType {
        FreeJoint, BallJoint
    };
    enum class ShapeType {
        Capsule, Cylinder, Sphere, Box
    };
    struct CapsuleShape {
        glm::vec3 direction;
        glm::vec3 offset;
        float radius;
        float height;
    };
    struct CylinderShape {
        glm::vec3 direction;
        glm::vec3 offset;
        float radius;
        float height;
    };
    struct SphereShape {
        glm::vec3 offset;
        float radius;
    };
    struct BoxShape {
        glm::vec3 size;
        glm::vec3 offset;
    };
    struct Shape {
        ShapeType type;
        union {
            CapsuleShape capsule;
            CylinderShape cylinder;
            SphereShape sphere;
            BoxShape box;
        };
    };

    struct Joint {
        JointType type;
        std::string name;
        std::string parentName;
        glm::vec3 size;
        float mass;
        std::string bvhNode;
        glm::vec3 bodyTranslation;
        glm::vec3 jointTranslation;
        Shape shape;
    };


    std::string skeletonName;
    std::vector<Joint> joints;

    std::unordered_map<std::string, uint32_t> jointNameMapping;

    Joint& getJoint(uint32_t idx) {
        return joints[idx];
    }

    Joint& getJoint(const std::string& name) {
        return joints[jointNameMapping[name]];
    }

    static BVHArticulationMap fromFile(const std::string& filename);
};

struct PoseEntity {
    PoseTree poseTree;
    PoseState poseState;
    BVHArticulationMap bvhArtiMap;

    physx::PxArticulationReducedCoordinate* articulation;

    std::unordered_map<uint32_t, PxArticulationLink*> nodeToLinkPtr;
    std::unordered_map<uint32_t, uint32_t> nodeToLinkIdx;
    std::unordered_map<uint32_t, uint32_t> linkToNodeIdx;
    std::unordered_map<uint32_t, uint32_t> secondaryLinkToNodeIdx;

    std::unordered_map<std::string, PxArticulationLink*> bodyToLinkMap;
    std::unordered_map<std::string, PxTransform> bodyToTransformMap;
    std::unordered_map<PxArticulationLink*, std::string> linkToBodyMap;

    std::vector<float> jointPosition;

    std::vector<uint32_t> dofStarts;
    uint32_t dofCount;

    // used for rendering
    std::vector<glm::vec3> lineVertices;
    std::vector<glm::mat4> initialBoneTransforms;
    std::vector<glm::mat4> boneTransforms;
    Ref<Mesh> boneMesh;
    Ref<Material> boneMaterial;
    Ref<Transform> transform;
    Camera* camera = nullptr;

    PoseEntity() = default;
    PoseEntity(PoseTree tree, PoseState state, BVHArticulationMap bvhArtiMap,
            Ref<Transform> transform, Camera* camera) :
        poseTree(std::move(tree)),
        poseState(std::move(state)),
        bvhArtiMap(std::move(bvhArtiMap)),
        transform(transform), camera(camera)
    {}

    void init();

    void initPhysX(PhysicsWorld& world);

    void initArticulationDefault(PhysicsWorld& world);

    void resetPhysX(PhysicsWorld &world);

    void stop(PhysicsWorld &world);

    void saveStateToPhysX(PhysicsWorld &world);

    void loadStateFromPhysX(PhysicsWorld &world);

    void update(float dt);

    void queueBoneRender(PhongRenderer &renderer);

    void queueGizmosRender(GizmosRenderer &renderer);

    void drawImGui();

    void updateJointPositions();

private:

    void calculateInitialBoneTransforms();

    void updateJointPositionsRecursive(uint32_t jointID, glm::mat4 curTransform);

    void drawImGuiAnimationTree(uint32_t jointID);
};


#endif //MOTION_EDITING_POSEMESH_H
