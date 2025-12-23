cmake_minimum_required(VERSION 3.10)
project(webots_pathplanning C)

set(CMAKE_C_STANDARD 99)

file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/src/*.c")

file(GLOB_RECURSE MODEL_SOURCES "${CMAKE_SOURCE_DIR}/src/model/*.c")
list(APPEND SOURCES ${MODEL_SOURCES})

include_directories(
    /Applications/Webots.app/Contents/include/controller/c
    /Applications/Webots.app/Contents/include
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME} ${SOURCES})
set(CMAKE_INSTALL_RPATH "/Applications/Webots.app/Contents/lib/controller")

target_link_libraries(${PROJECT_NAME}
    /Applications/Webots.app/Contents/lib/controller/libController.dylib
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "/Applications/Webots.app/Contents/lib/controller"
    INSTALL_RPATH "/Applications/Webots.app/Contents/lib/controller"
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND install_name_tool -change
        @rpath/Contents/lib/controller/libController.dylib
        /Applications/Webots.app/Contents/lib/controller/libController.dylib
        $<TARGET_FILE:${PROJECT_NAME}>
)

