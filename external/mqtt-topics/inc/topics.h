#ifndef MQTT_TOPICS_TOPICS_H
#define MQTT_TOPICS_TOPICS_H

#include <vector>
#include <string>

namespace MQTTTopics
{
enum class Role 
{
    role_0 = 0,
    role_1 = 1,
    role_2 = 2,
    role_3 = 3,
    role_4 = 4,
    role_128 = 5,
    role_129 = 6,
    role_130 = 7,
    role_131 = 8
};

enum class Topic
{
    vehicle_id = 0,
    device_id = 1,
    device_version = 2,
    data = 3,
    data_primary = 4,
    data_secondary = 5,
    data_bms = 6,
    data_inverter = 7,
    data_simulator = 8,
    data_gps = 9,
    data_brusa = 10,
    data_temporary = 11,
    data_camera_logger = 12,
    data_camera_logger_can = 13,
    data_camera_logger_can_imu_angular_rate = 14,
    data_camera_logger_can_imu_acceleration = 15,
    data_camera_logger_can_vehicle_position = 16,
    data_camera_logger_can_vehicle_speed = 17,
    data_camera_logger_can_front_angular_velocity = 18,
    data_camera_logger_can_rear_left_angular_velocity = 19,
    data_camera_logger_can_rear_right_angular_velocity = 20,
    data_camera_logger_can_steer_angle = 21,
    data_camera_logger_gps = 22,
    data_camera_logger_gps_hpposllh = 23,
    data_camera_logger_gps_pvt = 24,
    data_camera_logger_gps_relposned = 25,
    extra_data_to_log = 26,
    extra_tlm_data = 27,
    extra_tlm_data_vehicle_state = 28,
    extra_tlm_data_baseline = 29,
    extra_tlm_data_gps_map_origins = 30,
    data_last_update = 31,
    status = 32,
    status_info = 33,
    status_error = 34,
    status_alert = 35,
    status_can_frequencies = 36,
    status_lap_counter_status = 37,
    status_lap_counter_laps = 38,
    as = 39,
    as_commands = 40,
    as_commands_set_values = 41,
    as_commands_set_status = 42,
    file_transaction_request = 43,
    file_transaction_response = 44,
    file_transaction = 45,
    file_transaction_begin = 46,
    file_transaction_end = 47,
    file_transaction_chunk = 48,
    file_transaction_chunk_ack = 49,
    info = 50,
    info_version = 51,
    info_user = 52,
    info_telemetry_replay = 53,
    info_telemetry_replay_ready = 54,
    info_telemetry_replay_start = 55,
    info_session = 56,
    info_session_started = 57,
    info_session_stopped = 58,
    info_session_keepalive = 59,
    action = 60,
    action_telemetry_config = 61,
    action_telemetry_config_set = 62,
    action_telemetry_config_get = 63,
    action_telemetry_config_content = 64,
    action_session_config = 65,
    action_session_config_set = 66,
    action_session_config_get = 67,
    action_session_config_content = 68,
    action_car_config = 69,
    action_car_config_set = 70,
    action_car_config_get = 71,
    action_car_config_content = 72,
    action_baseline_config = 73,
    action_baseline_config_set = 74,
    action_baseline_config_get = 75,
    action_baseline_config_content = 76,
    action_handcart_settings = 77,
    action_handcart_settings_set = 78,
    action_handcart_settings_get = 79,
    action_handcart_settings_content = 80,
    raw_json_config = 81,
    raw_json_config_set = 82,
    raw_json_config_get = 83,
    raw_json_config_content = 84,
    action_lap_counter_config = 85,
    action_lap_counter_config_set = 86,
    action_lap_counter_config_get = 87,
    action_lap_counter_config_content = 88,
    action_lap_counter_track = 89,
    action_lap_counter_track_set = 90,
    action_lap_counter_track_get = 91,
    action_lap_counter_track_content = 92,
    action_kill = 93,
    action_start = 94,
    action_reset = 95,
    action_stop = 96,
    action_start_baseline = 97,
    action_stop_baseline = 98,
    action_precharge = 99,
    action_balance = 100,
    action_stop_balance = 101,
    action_charge = 102,
    action_stop_charge = 103,
    action_raw = 104,
    action_reset_lap_counter = 105,
    action_set_lap_counter_status = 106,
    simulator = 107,
    simulator_inputs = 108,
    simulator_outputs = 109,
    simulator_initial_state = 110
};

struct TopicMessage
{
    TopicMessage(std::string&& topic, int qos, bool retain);

    std::string topic;
    int qos;
    bool retain;
};

std::vector<TopicMessage> GetSubscribeTopics(Role role, const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
std::vector<TopicMessage> GetPublishTopics(Role role, const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);

bool CanSubscribe(Role role, Topic topic);
bool CanPublish(Role role, Topic topic);

TopicMessage GetTopic(Topic topic, const std::string& vehicleId = "<vehicleId>", const std::string& deviceId = "<deviceId>", const std::string& transactionId = "<transactionId>");
TopicMessage GetTopicVehicleId(const std::string& vehicleId);
TopicMessage GetTopicDeviceId(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDeviceVersion(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicData(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataPrimary(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataSecondary(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataBms(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataInverter(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataSimulator(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataGps(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataBrusa(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataTemporary(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLogger(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCan(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanImuAngularRate(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanImuAcceleration(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanVehiclePosition(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanVehicleSpeed(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanFrontAngularVelocity(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanRearLeftAngularVelocity(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanRearRightAngularVelocity(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerCanSteerAngle(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerGps(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerGpsHpposllh(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerGpsPvt(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataCameraLoggerGpsRelposned(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicExtraDataToLog(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicExtraTlmData(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicExtraTlmDataVehicleState(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicExtraTlmDataBaseline(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicExtraTlmDataGpsMapOrigins(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicDataLastUpdate(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatus(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusInfo(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusError(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusAlert(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusCanFrequencies(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusLapCounterStatus(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicStatusLapCounterLaps(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicAs(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicAsCommands(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicAsCommandsSetValues(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicAsCommandsSetStatus(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicFileTransactionRequest(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicFileTransactionResponse(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicFileTransaction(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
TopicMessage GetTopicFileTransactionBegin(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
TopicMessage GetTopicFileTransactionEnd(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
TopicMessage GetTopicFileTransactionChunk(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
TopicMessage GetTopicFileTransactionChunkAck(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId);
TopicMessage GetTopicInfo(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoVersion(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoUser(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoTelemetryReplay(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoTelemetryReplayReady(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoTelemetryReplayStart(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoSession(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoSessionStarted(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoSessionStopped(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicInfoSessionKeepalive(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicAction(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionTelemetryConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionTelemetryConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionTelemetryConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionTelemetryConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionSessionConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionSessionConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionSessionConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionSessionConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionCarConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionCarConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionCarConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionCarConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionBaselineConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionBaselineConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionBaselineConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionBaselineConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionHandcartSettings(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionHandcartSettingsSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionHandcartSettingsGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionHandcartSettingsContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicRawJsonConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicRawJsonConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicRawJsonConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicRawJsonConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterConfig(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterConfigSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterConfigGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterConfigContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterTrack(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterTrackSet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterTrackGet(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionLapCounterTrackContent(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionKill(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStart(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionReset(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStop(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStartBaseline(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStopBaseline(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionPrecharge(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionBalance(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStopBalance(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionCharge(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionStopCharge(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionRaw(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionResetLapCounter(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicActionSetLapCounterStatus(const std::string& vehicleId, const std::string& deviceId);
TopicMessage GetTopicSimulator(const std::string& vehicleId);
TopicMessage GetTopicSimulatorInputs(const std::string& vehicleId);
TopicMessage GetTopicSimulatorOutputs(const std::string& vehicleId);
TopicMessage GetTopicSimulatorInitialState(const std::string& vehicleId);
}

#endif