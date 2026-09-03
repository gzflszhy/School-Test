############### Component inputs ################
list(APPEND ADD_INCLUDE "include")

# append_srcs_dir is supplied by the MaixCDK component build.  Keeping this
# list explicit makes the target deterministic and avoids compiling test/tools.
list(APPEND ADD_SRCS
    "src/main.cpp"
    "src/maix_camera_source.cpp"
    "src/marker_geometry.cpp"
    "src/marker_template.cpp"
    "src/marker_detector.cpp"
    "src/benchmark.cpp"
)

# vision provides maix_camera.hpp, maix_image.hpp and the MaixCDK OpenCV
# integration used by the detector.  No display component is requested.
list(APPEND ADD_REQUIREMENTS vision)

# MaixCDK projects do not impose a source-level C++ standard in the component
# template.  This detector uses C++17 library features, so state it locally.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

register_component()
