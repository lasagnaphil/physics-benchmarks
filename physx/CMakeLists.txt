# PhysX
find_package(Threads REQUIRED)
find_package(PhysX REQUIRED)

add_subdirectory(gengine)

set(PHYSICS_PHYSX_SOURCES PhysicsWorld.cpp)
add_executable(physics_physx_cannon ${PHYSICS_PHYSX_SOURCES} cannon.cpp)
add_executable(physics_physx_dominoes ${PHYSICS_PHYSX_SOURCES} dominoes.cpp)
add_executable(physics_physx_humanoid ${PHYSICS_PHYSX_SOURCES} PoseEntity.cpp humanoid.cpp)

set(PHYSX_DEMOS physics_physx_cannon physics_physx_dominoes physics_physx_humanoid)

foreach(DEMO ${PHYSX_DEMOS})
    target_include_directories(${DEMO} PUBLIC ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
    target_include_directories(${DEMO} PUBLIC gengine)

    target_link_libraries(${DEMO} ${CMAKE_DL_LIBS} Threads::Threads)
    target_link_libraries(${DEMO} gengine)
    target_link_libraries(${DEMO} debug ${PhysX_LIBRARIES_DEBUG})
    target_link_libraries(${DEMO} optimized ${PhysX_LIBRARIES_RELEASE})
endforeach()
