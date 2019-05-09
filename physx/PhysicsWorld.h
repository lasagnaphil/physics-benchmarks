//
// Created by lasagnaphil on 19. 5. 7.
//

#ifndef PHYSICS_BENCHMARKS_PHYSICSWORLD_H
#define PHYSICS_BENCHMARKS_PHYSICSWORLD_H

#ifndef NDEBUG
#define _DEBUG
#endif

#include <PxPhysicsAPI.h>
#include <pvd/PxPvd.h>
#include <extensions/PxDefaultAllocator.h>
#include <extensions/PxDefaultErrorCallback.h>

#include "PhysXGLM.h"

using namespace physx;

struct PhysicsWorld {
    static PxDefaultAllocator gAllocator;
    static PxDefaultErrorCallback gErrorCallback;

    void init(uint32_t numThreads);

    bool advance(float dt);

    bool fetchResults();

    void release();

    glm::vec3 getBodyPosition(PxRigidDynamic* body) const {
        return PxToGLM(body->getGlobalPose().p);
    }

    glm::quat getBodyRotation(PxRigidDynamic* body) const {
        return PxToGLM(body->getGlobalPose().q);
    }

    glm::mat4 getBodyTransform(PxRigidDynamic* body) const {
        return PxToGLM(body->getGlobalPose());
    }

    const PxRenderBuffer& getRenderBuffer() const {
        return scene->getRenderBuffer();
    }

    PxFoundation* foundation;
    PxPvd* pvd;
    PxPhysics* physics;
    PxCudaContextManager* cudaContextManager;
    PxCooking* cooking;
    PxCpuDispatcher* cpuDispatcher;
    PxScene* scene;
    PxMaterial* material;
};

#endif //PHYSICS_BENCHMARKS_PHYSICSWORLD_H
