//
// Created by lasagnaphil on 19. 5. 8.
//

#include <iostream>
#include "App.h"
#include "Defer.h"

#include "PhysicsWorld.h"
#include "Entities.h"
#include "MotionClipData.h"
#include "MotionClipPlayer.h"
#include "PoseEntity.h"
#include "InputManager.h"
#include "PhysXDebugRenderer.h"

class MyApp : public App {
public:
    MyApp() : App(false), pxDebugRenderer(&trackballCamera) {}

    void loadResources() override {
        Ref<Transform> cameraTransform = trackballCamera.transform;
        cameraTransform->setPosition({5.0f, 5.0f, 0.0f});
        cameraTransform->rotate(M_PI/4, {0.0f, 0.0f, 1.0f});

        world.init(2);
        world.scene->setGravity(PxVec3(0, 0, 0));

        Ref<Image> checkerImage = Resources::make<Image>("resources/textures/checker.png");
        Ref<Texture> planeTexture = Resources::make<Texture>(checkerImage);
        checkerImage.release();

        groundMesh = Mesh::makePlane(1000.0f, 100.0f);

        groundMat = Resources::make<Material>();
        groundMat->ambient = {0.1f, 0.1f, 0.1f, 1.0f};
        groundMat->specular = {0.7f, 0.7f, 0.7f, 1.0f};
        groundMat->shininess = 32.0f;
        groundMat->texDiffuse = planeTexture;
        groundMat->texSpecular = {};

        bool success = MotionClipData::loadFromFile("resources/run_merged_rtg_Take_001.bvh", poseData, 0.01f);
        if (!success) {
            std::cerr << "Failed to load pose data" << std::endl;
            exit(EXIT_FAILURE);
        }
        poseData.print();

        Ref<Transform> poseTransform = Resources::make<Transform>();
        poseTransform->setScale({1.f, 1.f, 1.f});
        Transform::addChildToParent(poseTransform, rootTransform);

        motionClipPlayer = MotionClipPlayer(&poseData);
        motionClipPlayer.setFrame(1);
        motionClipPlayer.init();

        auto bvhArtiMap = BVHArticulationMap::fromFile("resources/humanoid_complex.xml");
        poseEntity = PoseEntity(poseData.poseTree, motionClipPlayer.getPoseState(), bvhArtiMap,
                poseTransform, &trackballCamera);
        poseEntity.init();
        poseEntity.initPhysX(world);
        poseEntity.poseState = motionClipPlayer.getPoseState();

        pxDebugRenderer.init(world);

        poseEntity.saveStateToPhysX(world);
    }

    void processInput(SDL_Event &event) override {

    }

    void update(float dt) override {
        static bool start = false;
        if (start) {
            start = false;
        }

        auto inputMgr = InputManager::get();
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            enablePhysics = !enablePhysics;
        }

        motionClipPlayer.update(dt);
        if (motionClipPlayer.shouldUpdate) {
            poseEntity.poseState = motionClipPlayer.getPoseState();
            motionClipPlayer.shouldUpdate = false;

            bool advanced = world.advance(dt);
            if (advanced) {
                world.fetchResults();
            }
            // poseEntity.saveStateToPhysX(world);
            poseEntity.updateJointPositions();
        }

        if (enablePhysics) {
            bool advanced = world.advance(dt);
            if (advanced) {
                world.fetchResults();
            }
            // poseEntity.loadStateFromPhysX(world);
            poseEntity.updateJointPositions();
        }

        // if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
            // poseEntity.saveStateToPhysX(world);
            poseEntity.stop(world);
            bool advanced = world.advance(dt);
            if (advanced) {
                world.fetchResults();
            }
            // poseEntity.loadStateFromPhysX(world);
            poseEntity.updateJointPositions();
        // }
        // poseEntity.loadStateFromPhysX(world);
    }

    void render() override {
        phongRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        poseEntity.queueBoneRender(phongRenderer);
        phongRenderer.render();

        poseEntity.queueGizmosRender(gizmosRenderer);
        motionClipPlayer.queueGizmosRender(gizmosRenderer, poseEntity.transform->getWorldTransform());
        gizmosRenderer.render();

        motionClipPlayer.drawImGui();
        poseEntity.drawImGui();

        pxDebugRenderer.render(world);
    }

    void release() override {
        world.release();
    }

private:
    PhysicsWorld world;

    Ref<Mesh> groundMesh;
    Ref<Material> groundMat;

    MotionClipData poseData;
    MotionClipPlayer motionClipPlayer;
    PoseEntity poseEntity;

    bool enablePhysics = false;

    PhysXDebugRenderer pxDebugRenderer;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}