cmake_minimum_required(VERSION 3.16)

project(ProjectPoints VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SQLite3 REQUIRED)

add_library(ProjectPointsUtils
    src/utils/FileIO.cpp
    src/utils/Lines.cpp
    src/utils/Matrix.cpp
)

target_include_directories(ProjectPointsUtils
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(ProjectPointsUtils
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        SQLite::SQLite3
)

add_executable(ImagePoints
    src/ImagePoints.cpp
)

target_include_directories(ImagePoints
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(ImagePoints
    PRIVATE
        ProjectPointsUtils
        Eigen3::Eigen
        ${OpenCV_LIBS}
)
