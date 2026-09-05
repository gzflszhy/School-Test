############### Component inputs ################
list(APPEND ADD_SRCS
    "src/main.cpp"
)

# UART and pinmap are provided by MaixCDK's peripheral component.
list(APPEND ADD_REQUIREMENTS peripheral)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

register_component()
