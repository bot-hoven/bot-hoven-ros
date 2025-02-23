cmake_minimum_required(VERSION 3.12)
project(spi_console)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Add executable with main.cpp as entry point
add_executable(spi_console
    src/main.cpp
    src/spi_console.cpp
    src/SPIPeripheral.cpp
)

# Include directories (both source and include folders)
target_include_directories(spi_console PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(spi_console PRIVATE
    pthread
)