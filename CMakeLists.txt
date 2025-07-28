cmake_minimum_required(VERSION 3.16)
project(car_lateral_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -w")

set(DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(USER_HOME $ENV{HOME})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${DIR}/lib)
find_package(absl)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${SDL2_LIBRARY_DIRS})

if(absl_FOUND)
  message(STATUS "absl found")
  set(absl_libraries
      absl::algorithm
      absl::base
      absl::bind_front
      absl::bits
      absl::btree
      absl::cleanup
      absl::cord
      absl::core_headers
      absl::debugging
      absl::dynamic_annotations
      absl::flags
      absl::flat_hash_map
      absl::flat_hash_set
      absl::function_ref
      absl::hash
      absl::layout
      absl::log_severity
      absl::memory
      absl::node_hash_map
      absl::node_hash_set
      absl::optional
      absl::span
      absl::status
      absl::statusor
      absl::strings
      absl::synchronization
      absl::time
      absl::type_traits
      absl::utility
      absl::variant
      # check based on absl version maybe to comment if not found
      absl::log_internal_message
      absl::log_internal_check_op)
    foreach(iter_lib ${absl_libraries})
	string(REPLACE "absl::" "absl_" temp ${iter_lib})
	find_library(FOUND ${temp})
	if(FOUND)
	    list(APPEND protobuf_ABSL_USED_TARGETS ${iter_lib})
	endif()
	unset(FOUND CACHE)
    endforeach()
endif()
add_subdirectory(${DIR}/protobuffer)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})

# Definition of a command that must run when ${proto_files} are modified In this
# way the generation of the protobuffer is automatic
add_custom_command(
  OUTPUT ${proto_cpp} ${proto_hpp}
  COMMAND ${DIR}/protobuffer/compile.sh
  DEPENDS ${proto_files}
  VERBATIM
  COMMENT "Generating protobuffer files")

# The command above will run only if some target depends on it Definition of a
# dummy target (proto_generation) to use it as a dependency for other targets
# The target (proto_generation) has as dependencies the .pb.cc and .pb.h files
# https://stackoverflow.com/questions/12913077/cmake-add-dependency-to-add-custom-command-dynamically
add_custom_target(proto_generation DEPENDS ${proto_cpp} ${proto_hpp})
add_library(proto STATIC ${proto_cpp})
target_link_libraries(proto PUBLIC ${Protobuf_LIBRARIES})
include_directories(${DIR}/.generated ${proto_folders})

# link_directories(${DIR}/external/Clothoids/lib/lib
#                  ${DIR}/external/Clothoids/lib3rd/lib)

add_subdirectory(${DIR}/external)
# include_directories(${DIR}/external/Clothoids/lib/include
#                     ${DIR}/external/Clothoids/lib3rd/include)
add_subdirectory(${DIR}/libs)
add_subdirectory(${DIR}/core)
