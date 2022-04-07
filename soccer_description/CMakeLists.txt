cmake_minimum_required(VERSION 3.0.2)
project(soccer_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)

execute_process(
    COMMAND
        bash -c
        "rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/meshes &&
        gdown https://drive.google.com/uc?id=1Zq_cSSgK5PmHuO_bgHujjPk-p2Vx1wxL -O ${CMAKE_CURRENT_SOURCE_DIR}/meshes.zip &&
        unzip ${CMAKE_CURRENT_SOURCE_DIR}/meshes.zip -d ${CMAKE_CURRENT_SOURCE_DIR}/meshes &&
        rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/meshes.zip"
)

xacro_add_files(
    models/soccerbot.xacro OUTPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/models/soccerbot_stl.urdf TARGET media_files
)
