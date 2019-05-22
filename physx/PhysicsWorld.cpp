//
// Created by lasagnaphil on 19. 5. 7.
//

#include "PhysicsWorld.h"
#include <iostream>

PxDefaultAllocator PhysicsWorld::gAllocator = {};
PxDefaultErrorCallback PhysicsWorld::gErrorCallback = {};

void PhysicsWorld::init(uint32_t numThreads = 16) {
    foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    if (!foundation) {
        std::cerr << "PxCreateFoundation failed!" << std::endl;
        exit(EXIT_FAILURE);
    }

    PxTolerancesScale scale;
    scale.length = 1.00;
    scale.speed = 9.81;

    pvd = PxCreatePvd(*foundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation,
                              scale, true, pvd);
    if (!physics) {
        std::cerr << "PxCreatePhysics failed!" << std::endl;
        exit(EXIT_FAILURE);
    }

    cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(scale));
    if (!cooking) {
        std::cerr << "PxCreateCooking failed!" << std::endl;
        exit(EXIT_FAILURE);
    }

    PxCudaContextManagerDesc cudaContextManagerDesc;
    cudaContextManager = PxCreateCudaContextManager(*foundation, cudaContextManagerDesc, PxGetProfilerCallback());

    PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    sceneDesc.cpuDispatcher = cpuDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;

    // enable CUDA
    /*
    sceneDesc.cudaContextManager = cudaContextManager;
    sceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
    sceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
     */

    scene = physics->createScene(sceneDesc);

    PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
    if (pvdClient) {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    material = physics->createMaterial(0.5f, 0.5f, 0.6f);

    // create ground
    PxRigidStatic* groundPlane = PxCreatePlane(*physics, PxPlane(0,1,0,0), *material);
    scene->addActor(*groundPlane);
}

bool PhysicsWorld::advance(float dt) {
    static float time = 0.0f;
    constexpr float stepSize = 1.0f / 60.0f;
    time += dt;
    if (time < stepSize) {
        return false;
    }
    time -= stepSize;
    scene->simulate(stepSize);
    return true;
}

bool PhysicsWorld::fetchResults() {
    return scene->fetchResults(true);
}

void PhysicsWorld::release() {
    cooking->release();
    physics->release();
    if (pvd) {
        PxPvdTransport* transform = pvd->getTransport();
        pvd->release();
        pvd = nullptr;
        transform->release();
    }
    foundation->release();
}
