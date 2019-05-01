find_package(Threads REQUIRED)
find_package(PhysX REQUIRED)

add_executable(physics_physx dominoes.cpp ${PHYSICS_PHYSX_SOURCES})

target_link_libraries(physics_physx ${CMAKE_DL_LIBS})
target_link_libraries(physics_physx Threads::Threads)
target_link_libraries(physics_physx debug ${PhysX_LIBRARIES_DEBUG})
target_link_libraries(physics_physx optimized ${PhysX_LIBRARIES_RELEASE})

target_include_directories(physics_physx PUBLIC ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
