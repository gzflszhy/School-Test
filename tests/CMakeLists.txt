cmake_minimum_required(VERSION 3.16)
project(marker_detector_host_tests LANGUAGES CXX)
enable_testing()
set(CODE_DIR "${CMAKE_CURRENT_LIST_DIR}/../code/maixcam_marker_detector/main")

add_executable(avc1_protocol_test
  avc1_protocol_test.cpp
  "${CODE_DIR}/src/avc1_protocol.cpp")
target_include_directories(avc1_protocol_test PRIVATE "${CODE_DIR}/include")
target_compile_features(avc1_protocol_test PRIVATE cxx_std_17)
add_test(NAME avc1_protocol_test COMMAND avc1_protocol_test)

find_package(OpenCV QUIET COMPONENTS core imgproc)
if(OpenCV_FOUND AND EXISTS "${CODE_DIR}/include/marker_detector.hpp")
  add_executable(marker_detector_smoke smoke_test.cpp)
  target_include_directories(marker_detector_smoke PRIVATE "${CODE_DIR}/include" ${OpenCV_INCLUDE_DIRS})
  # Keep host build free of MaixCDK camera/main translation units.
  set(CORE_SOURCES
    "${CODE_DIR}/src/marker_geometry.cpp"
    "${CODE_DIR}/src/marker_template.cpp"
    "${CODE_DIR}/src/marker_detector.cpp"
    "${CODE_DIR}/src/benchmark.cpp")
  list(FILTER CORE_SOURCES INCLUDE REGEX "^.*$")
  foreach(src IN LISTS CORE_SOURCES)
    if(NOT EXISTS "${src}")
      message(FATAL_ERROR "Expected detector core source missing: ${src}")
    endif()
  endforeach()
  target_sources(marker_detector_smoke PRIVATE ${CORE_SOURCES})
  target_link_libraries(marker_detector_smoke PRIVATE ${OpenCV_LIBS})
  target_compile_features(marker_detector_smoke PRIVATE cxx_std_17)
  add_test(NAME marker_detector_smoke COMMAND marker_detector_smoke)

  add_executable(lateral_controller_test
    lateral_controller_test.cpp
    "${CODE_DIR}/src/lateral_controller.cpp"
    "${CODE_DIR}/src/marker_geometry.cpp")
  target_include_directories(lateral_controller_test PRIVATE
    "${CODE_DIR}/include" ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(lateral_controller_test PRIVATE ${OpenCV_LIBS})
  target_compile_features(lateral_controller_test PRIVATE cxx_std_17)
  add_test(NAME lateral_controller_test COMMAND lateral_controller_test)
else()
  message(STATUS "SKIP C++ smoke: OpenCV and/or code/include/marker_detector.hpp unavailable")
endif()
