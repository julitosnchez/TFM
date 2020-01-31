cmake_minimum_required(VERSION 2.8)
project(FunctionMin)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    FunctionMinEvalOp.cpp
    FunctionMinEvalOp.h
    main.cpp)

find_package(Boost)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

find_package(PCL 1.7)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ECF REQUIRED)

add_definitions(${ECF_DEFINITIONS})

include_directories(${ECF_INCLUDE_DIRS})
add_executable(FunctionMin ${SOURCE_FILES})
target_link_libraries(FunctionMin ${ECF_LIBRARIES} ${PCL_LIBRARIES})



