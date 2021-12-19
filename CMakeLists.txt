cmake_minimum_required(VERSION 3.0.2)
project(trytry_vo)

if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

find_package(OpenCV 3.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${g2o_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        "/usr/include/eigen3"
)

include_directories(
        "/usr/include/suitesparse"
)

add_executable(
        ${PROJECT_NAME}_node
        src/main.cpp
        src/camera.cpp
        src/frame.cpp
        src/vo.cpp
        src/config.cpp
        src/images.cpp
        src/frontend.cpp
        src/viewer.cpp

        include/trytry_vo/camera.h
        include/trytry_vo/frame.h
        include/trytry_vo/vo.h
        include/trytry_vo/config.h
        include/trytry_vo/images.h
        include/trytry_vo/frontend.h
        include/trytry_vo/viewer.h
        include/trytry_vo/g2o_types.h
)

add_executable(
        ground_truth
        src/ground_truth.cpp
        src/camera.cpp
        src/frame.cpp
        src/vo.cpp
        src/config.cpp
        src/images.cpp
        src/frontend.cpp
        src/viewer.cpp

        include/trytry_vo/camera.h
        include/trytry_vo/frame.h
        include/trytry_vo/vo.h
        include/trytry_vo/config.h
        include/trytry_vo/images.h
        include/trytry_vo/frontend.h
        include/trytry_vo/viewer.h
)


target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBS}
        g2o_core
        g2o_stuff
        ${catkin_LIBRARIES}
        ${Sophus_LIBRARIES}
        Sophus::Sophus
        glog::glog
        ${GLOG_INCLUDE_DIRS}
        )

target_link_libraries(ground_truth
        ${OpenCV_LIBS}
        g2o_core
        g2o_stuff
        ${catkin_LIBRARIES}
        ${Sophus_LIBRARIES}
        Sophus::Sophus
        glog::glog
        ${GLOG_INCLUDE_DIRS}
        )