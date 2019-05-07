//
// Created by lasagnaphil on 19. 5. 7.
//

#ifndef PHYSICS_BENCHMARKS_ENTITIES_H
#define PHYSICS_BENCHMARKS_ENTITIES_H

#include "PhysicsWorld.h"

struct BoxEntity {
    static Ref<Mesh> mesh;

    PxRigidDynamic* body;
    Ref<Material> renderMaterial;
    Ref<Transform> transform;

    glm::vec3 size;

    static BoxEntity make(PhysicsWorld& world,
            glm::vec3 pos = glm::vec3(),
            glm::quat rot = glm::identity<glm::quat>(),
            glm::vec3 size = glm::vec3(1.0f)) {

        BoxEntity e;
        e.size = size;

        if (mesh.isNull()) {
            mesh = Mesh::makeCube();
        }

        PxShape* shape = world.physics->createShape(PxBoxGeometry(GLMToPx(size/2.f)), *world.material);
        defer {shape->release();};
        shape->setFlag(PxShapeFlag::eVISUALIZATION, true);

        PxTransform localTm(GLMToPx(pos), GLMToPx(rot));
        e.body = world.physics->createRigidDynamic(localTm);
        e.body->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*e.body, 10.0f);
        world.scene->addActor(*e.body);

        e.renderMaterial = {};
        e.transform = Resources::make<Transform>();
        e.transform->setPosition(pos);
        e.transform->setRotation(rot);
        e.transform->setScale(size);

        return e;
    }


    void syncWithPhysics(const PhysicsWorld& world) {
        transform->setPosition(world.getBodyPosition(body));
        transform->setRotation(world.getBodyRotation(body));
    }

    RenderCommand renderCommand() {
        return {mesh, renderMaterial, transform->getWorldTransform()};
    }
};

Ref<Mesh> BoxEntity::mesh = {};

struct SphereEntity {
    static Ref<Mesh> mesh;

    PxRigidDynamic* body;
    Ref<Material> renderMaterial;
    Ref<Transform> transform;

    float size;

    static SphereEntity make(PhysicsWorld& world, glm::vec3 pos = {}, float size = 1.0f) {
        SphereEntity e;
        e.size = size;

        if (mesh.isNull()) {
            mesh = Mesh::makeSphere();
        }

        PxShape* shape = world.physics->createShape(PxSphereGeometry(size), *world.material);
        defer {shape->release();};
        shape->setFlag(PxShapeFlag::eVISUALIZATION, true);

        PxTransform localTm(GLMToPx(pos));
        e.body = world.physics->createRigidDynamic(localTm);
        e.body->attachShape(*shape);
        PxRigidBodyExt::updateMassAndInertia(*e.body, 10.0f);
        world.scene->addActor(*e.body);

        e.renderMaterial = {};
        e.transform = Resources::make<Transform>();
        e.transform->setPosition(pos);
        e.transform->setScale(glm::vec3(size));

        return e;
    }

    void syncWithPhysics(const PhysicsWorld& world) {
        transform->setPosition(world.getBodyPosition(body));
        transform->setRotation(world.getBodyRotation(body));
    }

    RenderCommand renderCommand() {
        return {mesh, renderMaterial, transform->getWorldTransform()};
    }
};

Ref<Mesh> SphereEntity::mesh = {};

#endif //PHYSICS_BENCHMARKS_ENTITIES_H
