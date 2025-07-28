message("[EXTERNAL]")

set(CDIR ${CMAKE_CURRENT_LIST_DIR})

set(EXTERNAL_DIRS
    ${CDIR}
    ${CIDR}/raylib
    ${CIDR}/raylib/src
    ${CDIR}/raygui

    ${CDIR}/fmt/include/

    ${CDIR}/clothoids/toolbox/src
    ${CDIR}/clothoids/toolbox/src/Utils
    ${CDIR}/clothoids/toolbox/src/Clothoidsgeographic
    ${CDIR}/lib-communication/inc
    ${CDIR}/lib-communication/external/paho/externals/paho-mqtt-c/src
    ${CDIR}/lib-communication/external/paho/include
    ${CDIR}/serializers/inc
    ${CDIR}/mqtt-topics/inc
  )

# link_directories(${CDIR}/Clothoids/lib/lib ${CDIR}/Clothoids/lib3rd/lib)
# --- CLOTHOIDS --- #
file(GLOB clothoids_toolbox_src ${CDIR}/clothoids/toolbox/src/*.cc)
file(GLOB clothoids_utils_src ${CDIR}/clothoids/toolbox/src/Utils/*.c*)
message("${clothoids_utils_src}")
add_library(clothoids
    STATIC
    ${clothoids_toolbox_src}
    #${clothoids_utils_src}
)
set_property(TARGET clothoids PROPERTY COMPILE_FLAGS "-w")

add_subdirectory(serializers)
target_link_libraries(serializers ${Protobuf_LIBRARIES} proto
    ${protobuf_ABSL_USED_TARGETS})
add_dependencies(serializers proto_generation)

add_library(geographic geographic/geographic_coordinates.cc)
target_include_directories(geographic INTERFACE ${CDIR}/geographic)

# add_subdirectory(mqtt-topics)

add_subdirectory(${CDIR}/fmt)

add_subdirectory(raylib)
add_subdirectory(lib-communication)
include_directories(
    ${communication_SOURCE_DIRS}
)
target_compile_options(communication PRIVATE -w)

include_directories(PUBLIC ${EXTERNAL_DIRS})
set(EXTERNAL_DIRS
    ${EXTERNAL_DIRS}
    PARENT_SCOPE)

    # --- MQTT TOPICS --- #
add_subdirectory(${CDIR}/mqtt-topics)
include_directories(
    ${mqtt-topics_SOURCE_DIRS}
)
target_compile_options(mqtt-topics PRIVATE -w)
