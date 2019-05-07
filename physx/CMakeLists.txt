# PhysX
find_package(Threads REQUIRED)
find_package(PhysX REQUIRED)

add_subdirectory(gengine)

file(GLOB PHYSICS_PHYSX_SOURCES *.cpp)
add_executable(physics_physx ${PHYSICS_PHYSX_SOURCES})

target_include_directories(physics_physx PUBLIC ${PhysX_INCLUDE_DIR} ${PxShared_INCLUDE_DIR})
target_include_directories(physics_physx PUBLIC gengine)

target_link_libraries(physics_physx ${CMAKE_DL_LIBS} Threads::Threads)
target_link_libraries(physics_physx gengine)
target_link_libraries(physics_physx debug ${PhysX_LIBRARIES_DEBUG})
target_link_libraries(physics_physx optimized ${PhysX_LIBRARIES_RELEASE})
