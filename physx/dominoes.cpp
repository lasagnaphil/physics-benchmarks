//
// Created by lasagnaphil on 19. 5. 13.
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

        world.init(16);

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
        }
    }

    void render() override {
        phongRenderer.queueRender({groundMesh, groundMat, rootTransform->getWorldTransform()});
        phongRenderer.render();
    }

    void release() override {
        world.release();
    }

private:
    PhysicsWorld world;

    Ref<Mesh> groundMesh;
    Ref<Material> groundMat;

    Ref<Mesh> dominoMesh;
    Ref<Material> dominoMat;
    std::vector<Ref<Transform>> dominoTransforms;

};

int main(int argc, char** argv) {
    MyApp app;
    app.start();

    return 0;
}