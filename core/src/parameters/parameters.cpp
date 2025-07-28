#include "parameters.hpp"

// =====================
// Reference values & constants
// =====================
const double DIRECTION_FORWARD = 90 * DEG2RAD;
const Vector2 CENTER = {0, 0};
const std::string MAP_BUILD_FILE_PATH = "./assets/gps_maps.json";
const std::string BASELINE_ORIGIN = "Povo@DV_1";

// =====================
// Enums & global state
// =====================
Page currentPage = LONGITUDINAL;
AvailableCars currentSelectedCar = SIM_CAR;
AvailableSimulations currentSimulation = NEUTRAL;
AvailableControllers currentSelectedController = PREVIEW_POINT;
CommandStatus commandsStatus = DISABLED;
State currentState = STOPPED;

// =====================
// Gains & controller parameters
// =====================
double Ke = 3.0;
double Kt = 0.5;
double stanleyK = 0.7;
double steering_angle = 0.0;
double prevS = 0.0;
double delta;
double look;
Vector2 lookAheadPoint;
double Lgain = 0.5;
Vector2 pointOnTrack;

// =====================
// Car & controller objects
// =====================
// SimCar car({}, 0.0, 0.0);
bool mustSwitchCar = true;
bool mustSwitchController = true;

// =====================
// Trajectory & map variables
// =====================
std::string message_lat;
std::vector<Eigen::Vector2d> left_cones, right_cones;
std::vector<ConePair> cone_pairs;
std::string cones_csv = "cones2.csv";
std::vector<CenterlinePoint> centerline;
ClothoidList clothoidList;
std::vector<Point> samplePoints;
std::vector<double> v_profile;
double x_pos = 0.0;
double y_pos = 0.0;
double heading = 0.0;
double velocity_value_1 = 0.0;
int N = 50;
double L = 1.53;
double m = 300.0; // Vehicle mass
double F_ymax = 1.5 * 9.81 * m; // Maximum lateral force
int preview_idx;
int nearest_idx;
double look_ahead_distance; // Look-ahead distance in meters
double k_lookahead = 0.5; // Tune as needed
double base_lookahead = 1.0; // Minimum lookahead distance
Vector2 pp;

// =====================
// PID parameters & functions
// =====================
PIDParams pidAt01kmh = {0.5, 0.5, 0.05};
PIDParams pidAt100kmh = {0.5, 0.5, 0.05};
PIDParams interpolatePIDParams(double speed) {
    double speedLow = 0.01 / 3.6;
    double speedHigh = 100 / 3.6;
    double t = (speed - speedLow) / (speedHigh - speedLow);
    t = std::max(0.0, std::min(1.0, t));
    PIDParams params;
    params.Kp = pidAt01kmh.Kp + t * (pidAt100kmh.Kp - pidAt01kmh.Kp);
    params.Ki = pidAt01kmh.Ki + t * (pidAt100kmh.Ki - pidAt01kmh.Ki);
    params.Kd = pidAt01kmh.Kd + t * (pidAt100kmh.Kd - pidAt01kmh.Kd);
    return params;
}
PIDParams getPIDParamsForSpeed(double speed) {
    return interpolatePIDParams(speed);
}

// =====================
// Simulation variables
// =====================
double desiredSpeed = 1;
double currentDesiredSpeed = desiredSpeed;
double timeDuration = 1000;
double cycleNumber = 5;
double throttle_power = 20;
double brake_power = 100;
double dt = 0.1;
double sindt = 10;
double amplitude = 0.25;
double frequency = 0.25;
bool issimulationrunning = false;
bool emergencystop = false;
std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point last_send_time = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point timestamp = std::chrono::high_resolution_clock::now();
double pidOutput = 0.0;
int numberofcyclesdone = 0;
double visualizepid = 0.0;
bool useSinusoidalSpeed = false;
bool useLongitudinal = false;
bool useGamepad = false;
bool flagforstop = false;
bool pidisActivated = false;
bool useTrajectory = false;
bool resetTelemetry = false;
bool isVehicleStateReceived = false;
double cruiseSpeed = 5.0 / 3.6; // 5 km/h in m/s
bool cruiseControl = false;
bool controlStatus = false;
SimulationType mode;
std::string vehicleId;
std::string host_number;
std::string vehicleIdName;

// =====================
// Wheel implementation
// =====================
Wheel::Wheel() : delta(45), width(10), length(20) {}
void Wheel::Draw() {
    DrawRectanglePro({0, 0, width, length}, {width / 2, length / 2}, -delta + 90, BLUE);
}
void Wheel::Update(float delta) { this->delta = delta; }
float Wheel::GetDelta() { return delta; }