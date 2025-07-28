from enum import Enum, auto

class Role(Enum):
    role_0 = 0,
    role_1 = 1,
    role_2 = 2,
    role_3 = 3,
    role_4 = 4,
    role_128 = 5,
    role_129 = 6,
    role_130 = 7,
    role_131 = 8

class Topic(Enum):
    vehicle_id = auto(),
    device_id = auto(),
    device_version = auto(),
    data = auto(),
    data_primary = auto(),
    data_secondary = auto(),
    data_bms = auto(),
    data_inverter = auto(),
    data_simulator = auto(),
    data_gps = auto(),
    data_brusa = auto(),
    data_temporary = auto(),
    data_camera_logger = auto(),
    data_camera_logger_can = auto(),
    data_camera_logger_can_imu_angular_rate = auto(),
    data_camera_logger_can_imu_acceleration = auto(),
    data_camera_logger_can_vehicle_position = auto(),
    data_camera_logger_can_vehicle_speed = auto(),
    data_camera_logger_can_front_angular_velocity = auto(),
    data_camera_logger_can_rear_left_angular_velocity = auto(),
    data_camera_logger_can_rear_right_angular_velocity = auto(),
    data_camera_logger_can_steer_angle = auto(),
    data_camera_logger_gps = auto(),
    data_camera_logger_gps_hpposllh = auto(),
    data_camera_logger_gps_pvt = auto(),
    data_camera_logger_gps_relposned = auto(),
    extra_data_to_log = auto(),
    extra_tlm_data = auto(),
    extra_tlm_data_vehicle_state = auto(),
    extra_tlm_data_baseline = auto(),
    extra_tlm_data_gps_map_origins = auto(),
    data_last_update = auto(),
    status = auto(),
    status_info = auto(),
    status_error = auto(),
    status_alert = auto(),
    status_can_frequencies = auto(),
    status_lap_counter_status = auto(),
    status_lap_counter_laps = auto(),
    as = auto(),
    as_commands = auto(),
    as_commands_set_values = auto(),
    as_commands_set_status = auto(),
    file_transaction_request = auto(),
    file_transaction_response = auto(),
    file_transaction = auto(),
    file_transaction_begin = auto(),
    file_transaction_end = auto(),
    file_transaction_chunk = auto(),
    file_transaction_chunk_ack = auto(),
    info = auto(),
    info_version = auto(),
    info_user = auto(),
    info_telemetry_replay = auto(),
    info_telemetry_replay_ready = auto(),
    info_telemetry_replay_start = auto(),
    info_session = auto(),
    info_session_started = auto(),
    info_session_stopped = auto(),
    info_session_keepalive = auto(),
    action = auto(),
    action_telemetry_config = auto(),
    action_telemetry_config_set = auto(),
    action_telemetry_config_get = auto(),
    action_telemetry_config_content = auto(),
    action_session_config = auto(),
    action_session_config_set = auto(),
    action_session_config_get = auto(),
    action_session_config_content = auto(),
    action_car_config = auto(),
    action_car_config_set = auto(),
    action_car_config_get = auto(),
    action_car_config_content = auto(),
    action_baseline_config = auto(),
    action_baseline_config_set = auto(),
    action_baseline_config_get = auto(),
    action_baseline_config_content = auto(),
    action_handcart_settings = auto(),
    action_handcart_settings_set = auto(),
    action_handcart_settings_get = auto(),
    action_handcart_settings_content = auto(),
    raw_json_config = auto(),
    raw_json_config_set = auto(),
    raw_json_config_get = auto(),
    raw_json_config_content = auto(),
    action_lap_counter_config = auto(),
    action_lap_counter_config_set = auto(),
    action_lap_counter_config_get = auto(),
    action_lap_counter_config_content = auto(),
    action_lap_counter_track = auto(),
    action_lap_counter_track_set = auto(),
    action_lap_counter_track_get = auto(),
    action_lap_counter_track_content = auto(),
    action_kill = auto(),
    action_start = auto(),
    action_reset = auto(),
    action_stop = auto(),
    action_start_baseline = auto(),
    action_stop_baseline = auto(),
    action_precharge = auto(),
    action_balance = auto(),
    action_stop_balance = auto(),
    action_charge = auto(),
    action_stop_charge = auto(),
    action_raw = auto(),
    action_reset_lap_counter = auto(),
    action_set_lap_counter_status = auto(),
    simulator = auto(),
    simulator_inputs = auto(),
    simulator_outputs = auto(),
    simulator_initial_state = auto()



def GetTopicVehicleId(vehicleId):
    topic = vehicleId
    return topic

def GetTopicDeviceId(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId
    return topic

def GetTopicDeviceVersion(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "version"
    return topic

def GetTopicData(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data"
    return topic

def GetTopicDataPrimary(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/primary"
    return topic

def GetTopicDataSecondary(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/secondary"
    return topic

def GetTopicDataBms(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/bms"
    return topic

def GetTopicDataInverter(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/inverters"
    return topic

def GetTopicDataSimulator(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/simulator"
    return topic

def GetTopicDataGps(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/gps"
    return topic

def GetTopicDataBrusa(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/brusa"
    return topic

def GetTopicDataTemporary(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/temporary"
    return topic

def GetTopicDataCameraLogger(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger"
    return topic

def GetTopicDataCameraLoggerCan(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can"
    return topic

def GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/imuAngularRate"
    return topic

def GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/imuAcceleration"
    return topic

def GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/vehiclePosition"
    return topic

def GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/vehicleSpeed"
    return topic

def GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/frontAngularVelocity"
    return topic

def GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/rearLeftAngularVelocity"
    return topic

def GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/rearRightAngularVelocity"
    return topic

def GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/steerAngle"
    return topic

def GetTopicDataCameraLoggerGps(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS"
    return topic

def GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/hpposllh"
    return topic

def GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/pvt"
    return topic

def GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/relposned"
    return topic

def GetTopicExtraDataToLog(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "extra_data_to_log"
    return topic

def GetTopicExtraTlmData(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data"
    return topic

def GetTopicExtraTlmDataVehicleState(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/vehicleState"
    return topic

def GetTopicExtraTlmDataBaseline(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/baseline"
    return topic

def GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/gpsMapOrigins"
    return topic

def GetTopicDataLastUpdate(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "last_update"
    return topic

def GetTopicStatus(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status"
    return topic

def GetTopicStatusInfo(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/info"
    return topic

def GetTopicStatusError(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/error"
    return topic

def GetTopicStatusAlert(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/alert"
    return topic

def GetTopicStatusCanFrequencies(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/canFrequencies"
    return topic

def GetTopicStatusLapCounterStatus(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/lapCounterStatus"
    return topic

def GetTopicStatusLapCounterLaps(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "status/lapCounterLaps"
    return topic

def GetTopicAs(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "as"
    return topic

def GetTopicAsCommands(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "as/commands"
    return topic

def GetTopicAsCommandsSetValues(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "as/commands/setValues"
    return topic

def GetTopicAsCommandsSetStatus(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "as/commands/setStatus"
    return topic

def GetTopicFileTransactionRequest(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction/request"
    return topic

def GetTopicFileTransactionResponse(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction/response"
    return topic

def GetTopicFileTransaction(vehicleId, deviceId, transactionId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId
    return topic

def GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "begin"
    return topic

def GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "end"
    return topic

def GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "chunk"
    return topic

def GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId):
    topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "chunk_ack"
    return topic

def GetTopicInfo(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info"
    return topic

def GetTopicInfoVersion(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/version"
    return topic

def GetTopicInfoUser(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/user"
    return topic

def GetTopicInfoTelemetryReplay(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay"
    return topic

def GetTopicInfoTelemetryReplayReady(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay/ready"
    return topic

def GetTopicInfoTelemetryReplayStart(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay/start"
    return topic

def GetTopicInfoSession(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/session"
    return topic

def GetTopicInfoSessionStarted(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/session/started"
    return topic

def GetTopicInfoSessionStopped(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/session/stopped"
    return topic

def GetTopicInfoSessionKeepalive(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "info/session/logging"
    return topic

def GetTopicAction(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action"
    return topic

def GetTopicActionTelemetryConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig"
    return topic

def GetTopicActionTelemetryConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/set"
    return topic

def GetTopicActionTelemetryConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/get"
    return topic

def GetTopicActionTelemetryConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/content"
    return topic

def GetTopicActionSessionConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig"
    return topic

def GetTopicActionSessionConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/set"
    return topic

def GetTopicActionSessionConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/get"
    return topic

def GetTopicActionSessionConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/content"
    return topic

def GetTopicActionCarConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/carConfig"
    return topic

def GetTopicActionCarConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/set"
    return topic

def GetTopicActionCarConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/get"
    return topic

def GetTopicActionCarConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/content"
    return topic

def GetTopicActionBaselineConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig"
    return topic

def GetTopicActionBaselineConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/set"
    return topic

def GetTopicActionBaselineConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/get"
    return topic

def GetTopicActionBaselineConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/content"
    return topic

def GetTopicActionHandcartSettings(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings"
    return topic

def GetTopicActionHandcartSettingsSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/set"
    return topic

def GetTopicActionHandcartSettingsGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/get"
    return topic

def GetTopicActionHandcartSettingsContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/content"
    return topic

def GetTopicRawJsonConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig"
    return topic

def GetTopicRawJsonConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/set"
    return topic

def GetTopicRawJsonConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/get"
    return topic

def GetTopicRawJsonConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/content"
    return topic

def GetTopicActionLapCounterConfig(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig"
    return topic

def GetTopicActionLapCounterConfigSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/set"
    return topic

def GetTopicActionLapCounterConfigGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/get"
    return topic

def GetTopicActionLapCounterConfigContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/content"
    return topic

def GetTopicActionLapCounterTrack(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack"
    return topic

def GetTopicActionLapCounterTrackSet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/set"
    return topic

def GetTopicActionLapCounterTrackGet(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/get"
    return topic

def GetTopicActionLapCounterTrackContent(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/content"
    return topic

def GetTopicActionKill(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/kill"
    return topic

def GetTopicActionStart(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/start"
    return topic

def GetTopicActionReset(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/reset"
    return topic

def GetTopicActionStop(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/stop"
    return topic

def GetTopicActionStartBaseline(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/startBaseline"
    return topic

def GetTopicActionStopBaseline(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/stopBaseline"
    return topic

def GetTopicActionPrecharge(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/precharge"
    return topic

def GetTopicActionBalance(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/balance"
    return topic

def GetTopicActionStopBalance(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/stopBalance"
    return topic

def GetTopicActionCharge(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/charge"
    return topic

def GetTopicActionStopCharge(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/stopCharge"
    return topic

def GetTopicActionRaw(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/raw"
    return topic

def GetTopicActionResetLapCounter(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/resetLapcounter"
    return topic

def GetTopicActionSetLapCounterStatus(vehicleId, deviceId):
    topic = vehicleId + "/" + deviceId + "/" + "action/setLapcounterStatus"
    return topic

def GetTopicSimulator(vehicleId):
    topic = vehicleId + "/" + "simulator"
    return topic

def GetTopicSimulatorInputs(vehicleId):
    topic = vehicleId + "/" + "simulator/inputs"
    return topic

def GetTopicSimulatorOutputs(vehicleId):
    topic = vehicleId + "/" + "simulator/outputs"
    return topic

def GetTopicSimulatorInitialState(vehicleId):
    topic = vehicleId + "/" + "simulator/initialState"
    return topic
def GetSubscribeTopics(role, vehicleId, deviceId, transactionId):
    ret = []

    match role:
        case Role.role_0:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_1:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_2:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_3:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_4:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_128:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            return ret
            
        case Role.role_129:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            return ret
            
        case Role.role_130:
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            return ret
            
        case Role.role_131:
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            return ret

def GetPublishTopics(role, vehicleId, deviceId, transactionId):
    ret = []

    match role:
        case Role.role_0:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_1:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_2:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_3:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_4:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicExtraDataToLog(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsSet(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsGet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigSet(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigGet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackSet(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackGet(vehicleId, deviceId))
            ret.append(GetTopicActionKill(vehicleId, deviceId))
            ret.append(GetTopicActionStart(vehicleId, deviceId))
            ret.append(GetTopicActionReset(vehicleId, deviceId))
            ret.append(GetTopicActionStop(vehicleId, deviceId))
            ret.append(GetTopicActionStartBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionStopBaseline(vehicleId, deviceId))
            ret.append(GetTopicActionPrecharge(vehicleId, deviceId))
            ret.append(GetTopicActionBalance(vehicleId, deviceId))
            ret.append(GetTopicActionStopBalance(vehicleId, deviceId))
            ret.append(GetTopicActionCharge(vehicleId, deviceId))
            ret.append(GetTopicActionStopCharge(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            ret.append(GetTopicActionResetLapCounter(vehicleId, deviceId))
            ret.append(GetTopicActionSetLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicSimulator(vehicleId))
            ret.append(GetTopicSimulatorInputs(vehicleId))
            ret.append(GetTopicSimulatorOutputs(vehicleId))
            ret.append(GetTopicSimulatorInitialState(vehicleId))
            return ret
            
        case Role.role_128:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            return ret
            
        case Role.role_129:
            ret.append(GetTopicVehicleId(vehicleId))
            ret.append(GetTopicDeviceId(vehicleId, deviceId))
            ret.append(GetTopicDeviceVersion(vehicleId, deviceId))
            ret.append(GetTopicData(vehicleId, deviceId))
            ret.append(GetTopicDataPrimary(vehicleId, deviceId))
            ret.append(GetTopicDataSecondary(vehicleId, deviceId))
            ret.append(GetTopicDataBms(vehicleId, deviceId))
            ret.append(GetTopicDataInverter(vehicleId, deviceId))
            ret.append(GetTopicDataSimulator(vehicleId, deviceId))
            ret.append(GetTopicDataGps(vehicleId, deviceId))
            ret.append(GetTopicDataBrusa(vehicleId, deviceId))
            ret.append(GetTopicDataTemporary(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLogger(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCan(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGps(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId))
            ret.append(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmData(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataBaseline(vehicleId, deviceId))
            ret.append(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId))
            ret.append(GetTopicDataLastUpdate(vehicleId, deviceId))
            ret.append(GetTopicStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusInfo(vehicleId, deviceId))
            ret.append(GetTopicStatusError(vehicleId, deviceId))
            ret.append(GetTopicStatusAlert(vehicleId, deviceId))
            ret.append(GetTopicStatusCanFrequencies(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterStatus(vehicleId, deviceId))
            ret.append(GetTopicStatusLapCounterLaps(vehicleId, deviceId))
            ret.append(GetTopicAs(vehicleId, deviceId))
            ret.append(GetTopicAsCommands(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionRequest(vehicleId, deviceId))
            ret.append(GetTopicFileTransactionResponse(vehicleId, deviceId))
            ret.append(GetTopicFileTransaction(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId))
            ret.append(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId))
            ret.append(GetTopicInfo(vehicleId, deviceId))
            ret.append(GetTopicInfoVersion(vehicleId, deviceId))
            ret.append(GetTopicInfoUser(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplay(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId))
            ret.append(GetTopicInfoSession(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStarted(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionStopped(vehicleId, deviceId))
            ret.append(GetTopicInfoSessionKeepalive(vehicleId, deviceId))
            ret.append(GetTopicAction(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfig(vehicleId, deviceId))
            ret.append(GetTopicActionTelemetryConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfig(vehicleId, deviceId))
            ret.append(GetTopicActionSessionConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfig(vehicleId, deviceId))
            ret.append(GetTopicActionCarConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfig(vehicleId, deviceId))
            ret.append(GetTopicActionBaselineConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettings(vehicleId, deviceId))
            ret.append(GetTopicActionHandcartSettingsContent(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfig(vehicleId, deviceId))
            ret.append(GetTopicRawJsonConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfig(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterConfigContent(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrack(vehicleId, deviceId))
            ret.append(GetTopicActionLapCounterTrackContent(vehicleId, deviceId))
            ret.append(GetTopicActionRaw(vehicleId, deviceId))
            return ret
            
        case Role.role_130:
            ret.append(GetTopicAsCommandsSetValues(vehicleId, deviceId))
            ret.append(GetTopicAsCommandsSetStatus(vehicleId, deviceId))
            ret.append(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId))
            return ret
            
        case Role.role_131:
            return ret