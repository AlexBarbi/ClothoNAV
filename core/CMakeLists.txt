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
    src/utils/app_utils.hpp
    src/utils/commands.cpp
    src/utils/commands.hpp
)
target_link_libraries(utils PUBLIC raylib)

# add_library(car
#     ${CDIR}/src/car/car.cpp
#     src/car/car.hpp
#     src/car/BaseCar.cpp
#     src/car/SimCar.cpp
#     src/car/SimCar.h
#     src/car/RealCar.cpp
#     src/car/RealCar.h
# )
# target_link_libraries(car PRIVATE utils raylib clothoids fmt::fmt)

add_library(map
    ${CDIR}/src/map/map.cpp
)
target_link_libraries(map PRIVATE utils raylib clothoids fmt::fmt)

# --- Controller ---
# add_library(controller
#     # ${CDIR}/src/controller/controller.cpp
#     # ${CDIR}/src/controller/controller.hpp
#     # ${CDIR}/src/controller/previewpoint.cpp
#     # ${CDIR}/src/controller/stanley.cpp
# )
# target_link_libraries(controller PRIVATE utils raylib clothoids fmt::fmt)

add_library(parameters
    src/parameters/parameters.cpp
    src/parameters/parameters.hpp
    # src/car/BaseCar.h
    # src/car/BaseCar.cpp
    # src/car/car.hpp
    # src/car/car.cpp
    # src/controller/basecontroller.hpp
    # src/controller/basecontroller.hpp
    # src/controller/controller.cpp
    # src/controller/controller.hpp
    # src/controller/previewpoint.cpp
    # src/controller/previewpoint.hpp
    src/pid/pid.cpp
    src/pid/pid.h
    src/trajectory/trajectory.cpp
    src/trajectory/trajectory.h
)
target_link_libraries(parameters PUBLIC utils raylib fmt::fmt clothoids)

# --- Telemetry ---
add_library(telemetry
    src/telemetry/Communication.cpp
)
target_link_libraries(telemetry PUBLIC serializers communication proto mqtt-topics parameters)


# --- UI Components ---
add_library(button src/utils/Button.cpp src/utils/Button.hpp)
target_link_libraries(button PRIVATE raylib)

add_library(inputField src/utils/InputField.cpp src/utils/InputField.hpp)
target_link_libraries(inputField PRIVATE raylib)

add_library(checkBox src/utils/CheckBox.cpp src/utils/CheckBox.hpp)
target_link_libraries(checkBox PRIVATE raylib)

# --- PID, Parameters, Trajectory ---
add_library(pid src/pid/pid.cpp src/pid/pid.h)
target_link_libraries(pid PUBLIC utils)


add_library(trajectory
    src/trajectory/trajectory.cpp
    src/trajectory/trajectory.h
    src/trajectory/spline.cpp
    src/trajectory/spline.h
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
    PRIVATE utils button inputField checkBox fmt::fmt raylib map serializers trajectory clothoids
)

# --- Executables ---
# add_executable(main executables/map/main.cpp)
# target_link_libraries(main
#     PRIVATE utils fmt::fmt car map controller telemetry raylib communication
#     serializers clothoids CsvParser geographic
# )

# add_executable(manual_control executables/manual_control/main.cpp)
# target_link_libraries(manual_control
#     PRIVATE utils telemetry raylib communication serializers
# )

# add_executable(mqtt_car executables/mqtt_car/main.cpp)
# target_link_libraries(mqtt_car
#     PRIVATE utils telemetry controller map car clothoids raylib communication
#     serializers logger
# )

add_executable(longitudinal executables/longitudinal/main.cpp)
target_link_libraries(longitudinal
    PRIVATE utils button inputField checkBox  clothoids fmt::fmt map telemetry
    raylib communication serializers clothoids CsvParser geographic pid trajectory
    ${SDL2_LIBRARIES} zmq Ceres::ceres parameters ui
)

# add_executable(stanley_controller executables/stanley/main.cpp)
# target_link_libraries(stanley_controller
#     PRIVATE utils button inputField checkBox fmt::fmt car map controller telemetry
#     raylib communication serializers clothoids CsvParser geographic pid
# )

# add_executable(gamepad
#     executables/gamepad/main.cpp
#     src/pid/pid.h
#     src/pid/pid.cpp
# )
# target_link_libraries(gamepad
#     PRIVATE ${SDL2_LIBRARIES} raylib pid telemetry serializers communication utils
# )

add_dependencies(longitudinal proto_generation)