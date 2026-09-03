cmake_minimum_required(VERSION 3.16)
project(marker_detector_host_tests LANGUAGES CXX)
enable_testing()
find_package(OpenCV QUIET COMPONENTS core imgproc)
set(CODE_DIR "${CMAKE_CURRENT_LIST_DIR}/../code/maixcam_marker_detector/main")
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
else()
  message(STATUS "SKIP C++ smoke: OpenCV and/or code/include/marker_detector.hpp unavailable")
endif()
