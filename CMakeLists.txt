cmake_minimum_required(VERSION 3.10)
project(MyProject)

if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
endif()

# 2. Find the raylib package
find_package(raylib REQUIRED)

add_executable(main 
    main.cpp
)

target_compile_options(main PRIVATE -O3)

target_include_directories(main PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${raylib_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE raylib)

if(APPLE)
    target_link_libraries(main PRIVATE 
        "-framework IOKit" 
        "-framework Cocoa" 
        "-framework OpenGL"
    )
endif()
