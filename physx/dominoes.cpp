//
// Created by lasagnaphil on 19. 4. 30.
//

#define _DEBUG

#include <PxPhysicsAPI.h>
#include <pvd/PxPvd.h>

#include <iostream>

using namespace physx;

PxDefaultAllocator gAllocator;
PxDefaultErrorCallback gErrorCallback;

bool recordMemoryAllocations = true;

class World {
public:
    void init() {
        foundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

        if (!foundation) {
            std::cerr << "PxCreateFoundation failed!" << std::endl;
            exit(EXIT_FAILURE);
        }

        PxTolerancesScale scale;
        scale.length = 100;
        scale.speed = 981;

        pvd = PxCreatePvd(*foundation);
        PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
        pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

        physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation,
                                  scale, recordMemoryAllocations, pvd);
        if (!physics) {
            std::cerr << "PxCreateBasePhysics failed!" << std::endl;
            exit(EXIT_FAILURE);
        }

        cooking = PxCreateCooking(PX_PHYSICS_VERSION, *foundation, PxCookingParams(scale));
        if (!cooking) {
            std::cerr << "PxCreateCooking failed!" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    void release() {
        cooking->release();
        physics->release();
        foundation->release();
    }

private:
    PxFoundation* foundation;
    PxPvd* pvd;
    PxPhysics* physics;
    PxCooking* cooking;
};

int main(int argc, char** argv) {
    World world;
    world.init();
    world.release();

    return 0;
}