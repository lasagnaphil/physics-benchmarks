//
// Created by lasagnaphil on 19. 4. 30.
//

#include <iostream>
#include "App.h"
#include "Defer.h"

#include "PhysicsWorld.h"
#include "Entities.h"

class MyApp : public App {
public:
    MyApp() : App(false) {}

    void loadResources() override {
        Ref<Transform> cameraTransform = trackballCamera.transform;
        cameraTransform->setPosition({80.0f, 80.0f, 0.0f});
        cameraTransform->rotate(M_PI/4, {0.0f, 0.0f, 1.0f});

        world.init();

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

        boxMat = Resources::make<Material>();
        boxMat->ambient = {0.329412f, 0.223529f, 0.027451f, 1.0f};
        boxMat->diffuse = {0.780392f, 0.568627f, 0.113725f, 1.0f};
        boxMat->specular = {0.992157f, 0.941176f, 0.807843f, 1.0f};
        boxMat->shininess = 27.8974f;
        boxMat->texDiffuse = {};
        boxMat->texSpecular = {};

        sphereMat = Resources::make<Material>();
        sphereMat->ambient = {0.2125f, 0.1275f, 0.054f, 1.0f};
        sphereMat->diffuse = {0.714f, 0.4284f, 0.18144f, 1.0f};
        sphereMat->specular = {0.393548f, 0.271906f, 0.166721f, 1.0f};
        sphereMat->shininess = 25.6f;
        sphereMat->texDiffuse = {};
        sphereMat->texSpecular = {};

        cannonMat = Resources::make<Material>();
        cannonMat->ambient = {0.05375f, 0.05f, 0.06625f, 0.82f};
        cannonMat->diffuse = {0.18275f, 0.17f, 0.22525f, 0.82f};
        cannonMat->specular = {0.332741f, 0.328634f, 0.346435f, 0.82f};
        cannonMat->shininess = 38.4f;
        cannonMat->texDiffuse = {};
        cannonMat->texSpecular = {};

        for (int k = 0; k < 10; k++) {
            for (int j = 0; j < 10; j++) {
                for (int i = 0; i < 10; i++) {
                    if ((i+j+k) % 2 == 0) {
                        BoxEntity entity = BoxEntity::make(world,
                                                           {4.0f * j, 2.0f + 4.0f * i, 4.0f * k},
                                                           glm::identity<glm::quat>(),
                                                           glm::vec3(4.0));

                        entity.renderMaterial = boxMat;
                        Transform::addChildToParent(entity.transform, rootTransform);
                        boxes.push_back(entity);
                    }
                    else {
                        SphereEntity entity = SphereEntity::make(world,
                                                                 {4.0f * j, 2.0f + 4.0f * i, 4.0f * k},
                                                                 2.0);

                        entity.renderMaterial = sphereMat;
                        Transform::addChildToParent(entity.transform, rootTransform);
                        spheres.push_back(entity);
                    }
                }
            }
        }
        cannon = SphereEntity::make(world,
                                    {20.0f, 20.0f, -20.0f},
                                    5.0);
        cannon.renderMaterial = cannonMat;
        PxRigidBodyExt::updateMassAndInertia(*cannon.body, 50.0f);
        Transform::addChildToParent(cannon.transform, rootTransform);
        cannon.body->addForce({0.0f, 0.0f, 400.0f}, PxForceMode::eVELOCITY_CHANGE);
    }

    void processInput(SDL_Event &event) override {

    }

    void update(float dt) override {
        static bool start = false;
        if (start) {
            start = false;
        }

        bool advanced = world.advance(dt);
        if (advanced) {
            world.fetchResults();
            for (auto& box : boxes) {
                box.syncWithPhysics(world);
            }
            for (auto& sphere : spheres) {
                sphere.syncWithPhysics(world);
            }
            cannon.syncWithPhysics(world);
        }
    }

    void render() override {
        phongRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        for (auto& box : boxes) {
            phongRenderer.queueRender(box.renderCommand());
        }
        for (auto& sphere : spheres) {
            phongRenderer.queueRender(sphere.renderCommand());
        }
        phongRenderer.queueRender(cannon.renderCommand());
        phongRenderer.render();
    }

    void release() override {
        world.release();
    }

private:
    PhysicsWorld world;

    Ref<Mesh> groundMesh;
    Ref<Material> groundMat;

    std::vector<BoxEntity> boxes;
    Ref<Material> boxMat;

    std::vector<SphereEntity> spheres;
    Ref<Material> sphereMat;

    SphereEntity cannon;
    Ref<Material> cannonMat;
};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}