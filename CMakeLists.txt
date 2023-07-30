cmake_minimum_required(VERSION 3.26)

project(MeshRegistrationTool VERSION 1.0)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(SOURCES
        src/main.cpp
        src/gui/MainWindow.cpp
        src/registration/ICP.cpp
        src/registration/MeshRegistration.cpp
        src/registration/MeshRegistrationDisplay.cpp
        src/utils/mesh.cpp
        src/utils/image.cpp)

find_package(Qt6 COMPONENTS Widgets REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(VTK REQUIRED)
find_package(ITK REQUIRED)

include(${ITK_USE_FILE})

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR})

target_link_libraries(${PROJECT_NAME}
        ${VTK_LIBRARIES}
        ${ITK_LIBRARIES}
        Qt6::Widgets
        nlohmann_json::nlohmann_json
        Eigen3::Eigen)

vtk_module_autoinit(
        TARGETS ${PROJECT_NAME}
        MODULES ${VTK_LIBRARIES}
)
