# lib/BMX055/CMakeLists.txt

# Minimum CMake version required
cmake_minimum_required(VERSION 3.13)

# Define the BMX055 library
add_library(BMX055 STATIC
    src/BMX055.cpp
)

# Include directories for the BMX055 library
target_include_directories(BMX055 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src  # Include the directory where BMX055.h is located
)

# Link any required libraries for BMX055
target_link_libraries(BMX055 PUBLIC
    pico_stdlib      # Standard Pico SDK libraries
    hardware_i2c     # I2C communication
)
