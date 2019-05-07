//
// Created by lasagnaphil on 19. 5. 7.
//

#ifndef PHYSICS_BENCHMARKS_PHYSXGLM_H
#define PHYSICS_BENCHMARKS_PHYSXGLM_H

#include <PxPhysicsAPI.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

glm::vec2 PxToGLM(const physx::PxVec2& v) {
    return glm::vec2 {v.x, v.y};
}

glm::vec3 PxToGLM(const physx::PxVec3& v) {
    return glm::vec3 {v.x, v.y, v.z};
}

glm::vec4 PxToGLM(const physx::PxVec4& v) {
    return glm::vec4 {v.x, v.y, v.z, v.w};
}

glm::quat PxToGLM(const physx::PxQuat& q) {
    return glm::quat {q.w, q.x, q.y, q.z};
}

glm::mat4 PxToGLM(const physx::PxTransform& transform) {
    return glm::mat4_cast(PxToGLM(transform.q))
        * glm::translate(glm::mat4(1.0f), PxToGLM(transform.p));
}

physx::PxVec2 GLMToPx(const glm::vec2& v) {
    return physx::PxVec2(v.x, v.y);
}

physx::PxVec3 GLMToPx(const glm::vec3& v) {
    return physx::PxVec3(v.x, v.y, v.z);
}

physx::PxVec4 GLMToPx(const glm::vec4& v) {
    return physx::PxVec4(v.x, v.y, v.z, v.w);
}

physx::PxQuat GLMToPx(const glm::quat& q) {
    return physx::PxQuat(q.x, q.y, q.z, q.w);
}

#endif //PHYSICS_BENCHMARKS_PHYSXGLM_H
