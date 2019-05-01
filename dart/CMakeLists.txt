find_package(DART 6 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_executable(physics_dart dominoes.cpp)

target_link_libraries(physics_dart PUBLIC dart dart-utils-urdf dart-gui)
