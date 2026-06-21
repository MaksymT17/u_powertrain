cmake_minimum_required(VERSION 3.16)
project(EVSimulationQt VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Svg)
find_package(spdlog REQUIRED)

add_executable(EVSimulationQt
    main.cpp
    specviewer.cpp
    powertrain_simulator.cpp
    powergraph.cpp
    power_service.cpp
    assets.qrc
)

target_link_libraries(EVSimulationQt PRIVATE 
    Qt6::Widgets
    Qt6::Svg
    spdlog::spdlog
)

if(WIN32)
    set_target_properties(EVSimulationQt PROPERTIES WIN32_EXECUTABLE TRUE)
endif()