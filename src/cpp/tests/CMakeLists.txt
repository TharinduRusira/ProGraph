cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0068)
  cmake_policy(SET CMP0068 NEW)
  set(CMAKE_BUILD_WITH_INSTALL_NAME_DIR ON)
endif()

if(POLICY CMP0075)
  cmake_policy(SET CMP0075 NEW)
endif()

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

project(Prographs LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(graphsim_test test.cpp)
add_executable(ngraph_test ngraph_test.cpp)

target_link_libraries(graphsim_test PRIVATE graphsim)
target_link_libraries(ngraph_test PRIVATE graphsim)





