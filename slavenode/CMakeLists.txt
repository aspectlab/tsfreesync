cmake_minimum_required(VERSION 2.8)

set(BOOST_REQUIRED_COMPONENTS
    system
    thread
)
find_package(Boost "1.46" REQUIRED ${BOOST_REQUIRED_COMPONENTS})
find_package(UHD "3.8.0" REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
)
link_directories(${Boost_LIBRARY_DIRS})

add_executable(slavenode slavenode.cpp)

SET(CMAKE_BUILD_TYPE "Release")

target_link_libraries(slavenode ${UHD_LIBRARIES} ${Boost_LIBRARIES})

