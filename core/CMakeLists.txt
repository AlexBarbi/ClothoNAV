message("[SRC]")

set(CDIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${DIR}/bin/src)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

target_compile_options(raylib PRIVATE -w)
target_compile_options(raylib PUBLIC -w)

include_directories(
    src/
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LIBS_DIR}
    ${EXTERNAL_DIRS}
)

# --- Core Utilities ---
add_library(utils
    ${CDIR}/src/utils/utils.cpp
    src/utils/app_utils.cpp
    src/utils/commands.cpp
)
target_link_libraries(utils PUBLIC raylib)

add_library(map
    ${CDIR}/src/map/map.cpp
    src/map/spline_cones.cpp
)
target_link_libraries(map PRIVATE utils raylib clothoids fmt::fmt)

add_library(parameters
    src/parameters/parameters.cpp
    src/pid/pid.cpp
    src/trajectory/trajectory.cpp
)
target_link_libraries(parameters PUBLIC utils raylib fmt::fmt clothoids)

# --- Telemetry ---
add_library(telemetry
    src/telemetry/Communication.cpp
)
target_link_libraries(telemetry PUBLIC serializers communication proto mqtt-topics parameters)


# --- UI Components ---
add_library(button src/utils/Button.cpp)
target_link_libraries(button PRIVATE raylib)

add_library(inputField src/utils/InputField.cpp)
target_link_libraries(inputField PRIVATE raylib)

add_library(checkBox src/utils/CheckBox.cpp)
target_link_libraries(checkBox PRIVATE raylib)

# --- PID, Parameters, Trajectory ---
add_library(pid src/pid/pid.cpp)
target_link_libraries(pid PUBLIC utils)


add_library(trajectory
    src/trajectory/trajectory.cpp
    src/trajectory/spline.cpp
)
target_link_libraries(trajectory PUBLIC utils Ceres::ceres clothoids fmt::fmt map
)

# --- UI Screens ---
add_library(ui
    src/ui/menu.cpp
    src/ui/longitudinal.cpp
    src/ui/circuit.cpp
)
target_link_libraries(ui
    PRIVATE utils button inputField checkBox fmt::fmt raylib map serializers trajectory map clothoids
)

add_executable(main executables/main.cpp)
target_link_libraries(main
    PRIVATE utils button inputField checkBox  clothoids fmt::fmt map telemetry
    raylib communication serializers clothoids CsvParser geographic pid trajectory
    ${SDL2_LIBRARIES} zmq Ceres::ceres parameters ui
)

add_dependencies(main proto_generation)