# CMakeList.txt : CMake project for emu-nes, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("emu-nes")
set(SDL2_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/SDL2-2.0.22")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/src")
include_directories("${CMAKE_SOURCE_DIR}/src/headers")
include_directories("${CMAKE_SOURCE_DIR}/src/source")

# Add source to this project's executable.
add_executable (emu-nes "src/main.cpp" "src/headers/cpu_6502.h" "src/source/cpu_6502.cpp" "src/headers/Bus.h" "src/source/Bus.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET emu-nes PROPERTY CXX_STANDARD 20)
endif()

target_link_libraries("emu-nes" ${SDL2_LIBRARIES})
