#pragma once
#include <string>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "raylib.h"
#include "map/map.hpp"
#include "pid/pid.h"
#include "trajectory/trajectory.h"
#include "telemetry/Data.h"
#include "telemetry/Communication.h"
#include "utils/InputField.hpp"
#include "utils/Button.hpp"
#include "utils/CheckBox.hpp"
#include "Clothoids.hh"

// =====================
// UI constants
// =====================
#define SCREEN_WIDTH 1650
#define SCREEN_HEIGHT 1000
#define MOVE_OFFSET 15
#define TAB_FONT_SIZE 15
#define TAB_PADDING 10

// =====================
// Reference values & constants
// =====================
extern const double DIRECTION_FORWARD;
extern const Vector2 CENTER;
extern const std::string MAP_BUILD_FILE_PATH;
extern const std::string BASELINE_ORIGIN;


// =====================
// Enums & global state
// =====================
enum Page { MENU, SIMULATION, INSTRUCTIONS, LONGITUDINAL, LATERAL, CIRCUIT };
extern Page currentPage;
enum AvailableCars { REAL_CAR, SIM_CAR };
extern AvailableCars currentSelectedCar;
enum AvailableSimulations { START, NEUTRAL, STOP };
extern AvailableSimulations currentSimulation;
enum AvailableControllers { PREVIEW_POINT, STANLEY };
extern AvailableControllers currentSelectedController;
enum CommandStatus { ENABLED, DISABLED };
extern CommandStatus commandsStatus;
enum State { ACCELERATING, MAINTAINING, DECELERATING, STOPPED };
extern State currentState;

// =====================
// Gains & controller parameters
// =====================
extern double Ke;
extern double Kt;
extern double stanleyK;
extern double steering_angle;
extern double prevS;
extern double delta;
extern double look;
extern Vector2 lookAheadPoint;
extern double Lgain;
extern Vector2 pointOnTrack;

// =====================
// Car & controller objects
// =====================
class SimCar;
extern bool mustSwitchCar;
extern bool mustSwitchController;

// =====================
// Trajectory & map variables
// =====================
extern std::string message_lat;
extern std::vector<Eigen::Vector2d> left_cones, right_cones;
extern std::vector<ConePair> cone_pairs;
extern std::string cones_csv;
extern std::vector<CenterlinePoint> centerline;
extern ClothoidList clothoidList;
extern std::vector<Point> samplePoints;
extern std::vector<double> v_profile;
extern double x_pos;
extern double y_pos;
extern double heading;
extern double velocity_value_1;
extern int N;
extern double L;
extern double F_ymax;
extern double m;
extern int preview_idx;
extern int nearest_idx;
extern double look_ahead_distance;
extern double k_lookahead;
extern double base_lookahead;
extern Vector2 pp;

extern G2lib::ClothoidCurve center_spline;

extern ClothoidList spline_l;
extern ClothoidList spline_r;

extern G2lib::ClothoidCurve spline_direction_1;
extern G2lib::ClothoidCurve spline_direction_2;
extern G2lib::ClothoidCurve spline_direction_3;
extern G2lib::ClothoidCurve spline_direction_4;

// =====================
// PID parameters & functions
// =====================
extern PIDParams pidAt01kmh;
extern PIDParams pidAt100kmh;
PIDParams interpolatePIDParams(double speed);
PIDParams getPIDParamsForSpeed(double speed);

// =====================
// Simulation variables
// =====================
extern double desiredSpeed;
extern double currentDesiredSpeed;
extern double timeDuration;
extern double cycleNumber;
extern double throttle_power;
extern double brake_power;
extern double dt;
extern double sindt;
extern double amplitude;
extern double frequency;
extern bool issimulationrunning;
extern bool emergencystop;
extern std::chrono::high_resolution_clock::time_point start_time;
extern std::chrono::high_resolution_clock::time_point last_send_time;
extern std::chrono::high_resolution_clock::time_point timestamp;
extern double pidOutput;
extern int numberofcyclesdone;
extern double visualizepid;
extern bool useSinusoidalSpeed;
extern bool useLongitudinal;
extern bool useGamepad;
extern bool flagforstop;
extern bool pidisActivated;
extern bool useTrajectory;
extern bool resetTelemetry;
extern bool isVehicleStateReceived;
extern double cruiseSpeed;
extern bool cruiseControl;
extern bool controlStatus;
extern SimulationType mode;
extern std::string vehicleId;
extern std::string host_number;
extern std::string vehicleIdName;

// =====================
// Wheel class
// =====================
class Wheel {
  public:
    Wheel();
    void Draw();
    void Update(float delta);
    float GetDelta();
  private:
    float delta;
    float width;
    float length;
};

// Function declarations for pages
void menu(std::string &vehicleId, SimulationType &mode, bool &saveandclose);
void longitudinal(class Communication &communication, struct TelemetryData &telemetryData, std::string &vehicleId, 
  SimulationType &mode, enum State &currentState, PID &pid);
void circuit(const std::string cones_csv, const TelemetryData &telemetryData, 
              Camera2D &camera, class Communication &communication);

// Utility functions
void sinusoidalSpeed();
void switchCar();
void selectSimCar();
void selectRealCar();
void selectStartSimulation();
void selectNeutralSimulation();
void selectStopSimulation();
void selectEnabled();
void selectDisabled();
void updateValueOnFieldChange(double&, InputField);
void emergencyStop(class Communication &communication, struct TelemetryData &telemetryData);
void addTab(const std::string &name, Page page, std::vector<Button> &buttons);
void displayPages();
void displayOnlyMenu();