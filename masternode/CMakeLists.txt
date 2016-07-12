# CMakeLists.txt - HEAD directory CMakeLists file
#
# A.G.Klein / M.Overdick 9/16/15

cmake_minimum_required(VERSION 2.8)

# Some Boost components require being set here (improves compile time)
set(BOOST_REQUIRED_COMPONENTS
    system
    thread
)
find_package(Boost "1.46" REQUIRED ${BOOST_REQUIRED_COMPONENTS})
find_package(UHD "3.9.0" REQUIRED)

# Directories to be included (for header files)
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ../include/
)

link_directories(${Boost_LIBRARY_DIRS})

add_executable(masternode masternode.cpp)

SET(CMAKE_BUILD_TYPE "Release")

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --vectorize --cpu Cortex-A9 -O3 -Otime")

target_link_libraries(masternode ${UHD_LIBRARIES} ${Boost_LIBRARIES})

