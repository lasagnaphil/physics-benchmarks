# PhysX
find_package(Threads REQUIRED)
find_package(PhysX REQUIRED)

add_subdirectory(gengine)

set(PHYSICS_PHYSX_SOURCES PhysicsWorld.cpp)
add_executable(physics_physx_cannon ${PHYSICS_PHYSX_SOURCES} cannon.cpp)
add_executable(physics_physx_humanoid ${PHYSICS_PHYSX_SOURCES}
        MotionClipData.cpp MotionClipPlayer.cpp PoseEntity.cpp humanoid.cpp)

target_include_directories(physics_physx_cannon PUBLIC ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
target_include_directories(physics_physx_cannon PUBLIC gengine)
target_include_directories(physics_physx_humanoid PUBLIC ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
target_include_directories(physics_physx_humanoid PUBLIC gengine)

target_link_libraries(physics_physx_cannon ${CMAKE_DL_LIBS} Threads::Threads)
target_link_libraries(physics_physx_cannon gengine)
target_link_libraries(physics_physx_cannon debug ${PhysX_LIBRARIES_DEBUG})
target_link_libraries(physics_physx_cannon optimized ${PhysX_LIBRARIES_RELEASE})
target_link_libraries(physics_physx_humanoid ${CMAKE_DL_LIBS} Threads::Threads)
target_link_libraries(physics_physx_humanoid gengine)
target_link_libraries(physics_physx_humanoid debug ${PhysX_LIBRARIES_DEBUG})
target_link_libraries(physics_physx_humanoid optimized ${PhysX_LIBRARIES_RELEASE})
