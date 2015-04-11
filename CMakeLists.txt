cmake_minimum_required(VERSION 2.8.3)
project(robotics_toolkit)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Eigen3 REQUIRED)

find_library(LIBRT_LIBRARY NAMES rt)
add_library(librt STATIC IMPORTED)
set_property(TARGET librt PROPERTY IMPORTED_LOCATION ${LIBRT_LIBRARY})

catkin_package(
  INCLUDE_DIRS include ${ARMADILLO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${ARMADILLO_LIBRARIES}
  DEPENDS Armadillo Eigen3
)

include_directories(include ${ARMADILLO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${ARMADILLO_LIBRARY_DIRS})

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  add_definitions(-DARMA_NO_DEBUG)
endif()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY cmake/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.cmake"
)





add_executable(test_memento test/design_patterns/test_memento.cc)
target_link_libraries(test_memento
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_factory test/design_patterns/test_factory.cc)
target_link_libraries(test_factory
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_singleton test/design_patterns/test_singleton.cc)
target_link_libraries(test_singleton
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_math test/math/test_math.cc)
target_link_libraries(test_math
  ${ARMADILLO_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_linear_algebra test/linear_algebra/test_linear_algebra.cc)
target_link_libraries(test_linear_algebra
  ${ARMADILLO_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_geometry test/geometry/test_geometry.cc)
target_link_libraries(test_geometry
  ${ARMADILLO_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_equals test/geometry/test_equals.cc)
target_link_libraries(test_equals
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_so3_error test/geometry/test_so3_error.cc)
target_link_libraries(test_so3_error
  ${catkin_LIBRARIES}
  boost_unit_test_framework
)

add_executable(test_timer test/timing/test_timer.cc)
target_link_libraries(test_timer
  ${catkin_LIBRARIES}
  librt
  boost_unit_test_framework-mt
)

add_executable(test_conversions test/timing/test_conversions.cc)
target_link_libraries(test_conversions
  ${catkin_LIBRARIES}
  librt
  boost_unit_test_framework-mt
)

add_executable(test_stopwatch test/timing/test_stopwatch.cc)
target_link_libraries(test_stopwatch
  ${catkin_LIBRARIES}
  librt
  boost_unit_test_framework-mt
)
