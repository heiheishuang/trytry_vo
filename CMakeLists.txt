cmake_minimum_required(VERSION 3.0.2)
project(trytry_vo)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
)

find_package(OpenCV 3.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES trytry_vo
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}

)

include_directories(
        "/usr/include/eigen3"
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

        include/trytry_vo/camera.h
        include/trytry_vo/frame.h
        include/trytry_vo/vo.h
        include/trytry_vo/config.h
        include/trytry_vo/images.h
        include/trytry_vo/frontend.h
)


target_link_libraries(${PROJECT_NAME}_node
        ${OpenCV_LIBS}
        g2o_core
        g2o_stuff
        ${catkin_LIBRARIES}
        ${Sophus_LIBRARIES}
        Sophus::Sophus
)