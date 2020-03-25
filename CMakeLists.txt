# Compile your code with: (see README for additional information)
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=../ ..
# make # Compile the Coachbot library
# make coachbot_example # OPTIONAL: Compile the executable example code
# make install # Install the library (and example, if you compiled it)

cmake_minimum_required (VERSION 3.9)

project(kilosim_coachbot LANGUAGES CXX C)

# Identify the location of the kilosim library
add_subdirectory(submodules/kilosim)

# Directory containing header files
include_directories(api)

# ------------------------------------------------------------------------------

# COACHBOT LIBRARY
# Use this to be able to use the Coachbot robot class in other projects

# Be sure to list all source files
add_library(kilosim_coachbot INTERFACE)

target_include_directories(kilosim_coachbot
INTERFACE
  api
  submodules
)
target_compile_options(kilosim_coachbot
INTERFACE
  -g
  -march=native
  -ffast-math
  -Wall
  -Wextra
)
target_link_libraries(kilosim_coachbot INTERFACE kilosim)

target_compile_features(kilosim_coachbot INTERFACE cxx_std_11)

install(TARGETS kilosim_coachbot ARCHIVE DESTINATION lib)

# ------------------------------------------------------------------------------

# EXECUTABLE DEMO

add_executable(coachbot_example EXCLUDE_FROM_ALL
  src/DemoCoachbot.cpp
  src/main.cpp
)
target_link_libraries(coachbot_example PUBLIC kilosim)
target_compile_options(coachbot_example PRIVATE -g -march=native -Wall -Wextra)
target_compile_features(coachbot_example PRIVATE cxx_std_11)
install(TARGETS coachbot_example RUNTIME DESTINATION bin OPTIONAL)
