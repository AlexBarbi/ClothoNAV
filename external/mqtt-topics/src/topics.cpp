#include "topics.h"

namespace MQTTTopics
{
TopicMessage::TopicMessage(std::string&& topic, int qos, bool retain) :
    topic(topic), qos(qos), retain(retain) {}

std::vector<TopicMessage> GetSubscribeTopics(Role role, const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::vector<TopicMessage> ret;
    
    switch(role) {
        case Role::role_0:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_1:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_2:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_3:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_4:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_128:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        break;
        
        case Role::role_129:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        break;
        
        case Role::role_130:
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        break;
        
        case Role::role_131:
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        break;
    }

    return ret;
}

std::vector<TopicMessage> GetPublishTopics(Role role, const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::vector<TopicMessage> ret;
    
    switch(role) {
        case Role::role_0:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_1:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_2:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_3:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_4:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraDataToLog(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackSet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackGet(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionKill(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStart(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionReset(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStop(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStartBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionPrecharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopBalance(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionStopCharge(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionResetLapCounter(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSetLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicSimulator(vehicleId));
        ret.emplace_back(GetTopicSimulatorInputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorOutputs(vehicleId));
        ret.emplace_back(GetTopicSimulatorInitialState(vehicleId));
        break;
        
        case Role::role_128:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        break;
        
        case Role::role_129:
        ret.emplace_back(GetTopicVehicleId(vehicleId));
        ret.emplace_back(GetTopicDeviceId(vehicleId, deviceId));
        ret.emplace_back(GetTopicDeviceVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicData(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataPrimary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSecondary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBms(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataInverter(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataSimulator(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataBrusa(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataTemporary(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLogger(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCan(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGps(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmData(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataVehicleState(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataBaseline(vehicleId, deviceId));
        ret.emplace_back(GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId));
        ret.emplace_back(GetTopicDataLastUpdate(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusError(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusAlert(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusCanFrequencies(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicStatusLapCounterLaps(vehicleId, deviceId));
        ret.emplace_back(GetTopicAs(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommands(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionRequest(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransactionResponse(vehicleId, deviceId));
        ret.emplace_back(GetTopicFileTransaction(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId));
        ret.emplace_back(GetTopicInfo(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoVersion(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoUser(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplay(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayReady(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSession(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStarted(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionStopped(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoSessionKeepalive(vehicleId, deviceId));
        ret.emplace_back(GetTopicAction(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionTelemetryConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionSessionConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionCarConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionBaselineConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettings(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionHandcartSettingsContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicRawJsonConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfig(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterConfigContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrack(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionLapCounterTrackContent(vehicleId, deviceId));
        ret.emplace_back(GetTopicActionRaw(vehicleId, deviceId));
        break;
        
        case Role::role_130:
        ret.emplace_back(GetTopicAsCommandsSetValues(vehicleId, deviceId));
        ret.emplace_back(GetTopicAsCommandsSetStatus(vehicleId, deviceId));
        ret.emplace_back(GetTopicInfoTelemetryReplayStart(vehicleId, deviceId));
        break;
        
        case Role::role_131:
        break;
    }
    
    return ret;
}

bool CanSubscribe(Role role, Topic topic) {
    switch(role) {
        case Role::role_0:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::extra_data_to_log:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_set_lap_counter_status:
            default:
              return false;
        }
        
        case Role::role_1:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::extra_data_to_log:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_set_lap_counter_status:
            default:
              return false;
        }
        
        case Role::role_2:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::extra_data_to_log:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_set_lap_counter_status:
            default:
              return false;
        }
        
        case Role::role_3:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::extra_data_to_log:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_set_lap_counter_status:
            default:
              return false;
        }
        
        case Role::role_4:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::extra_data_to_log:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_set_lap_counter_status:
            default:
              return false;
        }
        
        case Role::role_128:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
            default:
              return false;
        }
        
        case Role::role_129:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
            default:
              return false;
        }
        
        case Role::role_130:
        switch(topic) {
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::info_telemetry_replay_ready:
              return true;
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::extra_data_to_log:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
            default:
              return false;
        }
        
        case Role::role_131:
        switch(topic) {
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_get:
            case Topic::action_session_config_content:
            case Topic::action_car_config_get:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_get:
            case Topic::action_baseline_config_content:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_content:
            case Topic::action_stop_baseline:
            case Topic::action_set_lap_counter_status:
              return true;
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_data_to_log:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
            default:
              return false;
        }
    }

    return false;
}

bool CanPublish(Role role, Topic topic) {
    switch(role) {
        case Role::role_0:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_user:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::info_version:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
             
            default: 
              return false;
        }
        
        case Role::role_1:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_user:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_version:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
             
            default: 
              return false;
        }
        
        case Role::role_2:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_user:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_version:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
             
            default: 
              return false;
        }
        
        case Role::role_3:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_user:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_version:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
             
            default: 
              return false;
        }
        
        case Role::role_4:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::extra_data_to_log:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_user:
            case Topic::info_telemetry_replay_start:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
              return true;
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_version:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config_content:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track_content:
             
            default: 
              return false;
        }
        
        case Role::role_128:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_raw:
              return true;
            case Topic::extra_data_to_log:
            case Topic::info_telemetry_replay_start:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
             
            default: 
              return false;
        }
        
        case Role::role_129:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_content:
            case Topic::action_raw:
              return true;
            case Topic::extra_data_to_log:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_telemetry_replay_start:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
             
            default: 
              return false;
        }
        
        case Role::role_130:
        switch(topic) {
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::info_telemetry_replay_start:
              return true;
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_data_to_log:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
             
            default: 
              return false;
        }
        
        case Role::role_131:
        switch(topic) {
            case Topic::vehicle_id:
            case Topic::device_id:
            case Topic::device_version:
            case Topic::data:
            case Topic::data_primary:
            case Topic::data_secondary:
            case Topic::data_bms:
            case Topic::data_inverter:
            case Topic::data_simulator:
            case Topic::data_gps:
            case Topic::data_brusa:
            case Topic::data_temporary:
            case Topic::data_camera_logger:
            case Topic::data_camera_logger_can:
            case Topic::data_camera_logger_can_imu_angular_rate:
            case Topic::data_camera_logger_can_imu_acceleration:
            case Topic::data_camera_logger_can_vehicle_position:
            case Topic::data_camera_logger_can_vehicle_speed:
            case Topic::data_camera_logger_can_front_angular_velocity:
            case Topic::data_camera_logger_can_rear_left_angular_velocity:
            case Topic::data_camera_logger_can_rear_right_angular_velocity:
            case Topic::data_camera_logger_can_steer_angle:
            case Topic::data_camera_logger_gps:
            case Topic::data_camera_logger_gps_hpposllh:
            case Topic::data_camera_logger_gps_pvt:
            case Topic::data_camera_logger_gps_relposned:
            case Topic::extra_data_to_log:
            case Topic::extra_tlm_data:
            case Topic::extra_tlm_data_vehicle_state:
            case Topic::extra_tlm_data_baseline:
            case Topic::extra_tlm_data_gps_map_origins:
            case Topic::data_last_update:
            case Topic::status:
            case Topic::status_info:
            case Topic::status_error:
            case Topic::status_alert:
            case Topic::status_can_frequencies:
            case Topic::status_lap_counter_status:
            case Topic::status_lap_counter_laps:
            case Topic::as:
            case Topic::as_commands:
            case Topic::as_commands_set_values:
            case Topic::as_commands_set_status:
            case Topic::file_transaction_request:
            case Topic::file_transaction_response:
            case Topic::file_transaction:
            case Topic::file_transaction_begin:
            case Topic::file_transaction_end:
            case Topic::file_transaction_chunk:
            case Topic::file_transaction_chunk_ack:
            case Topic::info:
            case Topic::info_version:
            case Topic::info_user:
            case Topic::info_telemetry_replay:
            case Topic::info_telemetry_replay_ready:
            case Topic::info_telemetry_replay_start:
            case Topic::info_session:
            case Topic::info_session_started:
            case Topic::info_session_stopped:
            case Topic::info_session_keepalive:
            case Topic::action:
            case Topic::action_telemetry_config:
            case Topic::action_telemetry_config_set:
            case Topic::action_telemetry_config_get:
            case Topic::action_telemetry_config_content:
            case Topic::action_session_config:
            case Topic::action_session_config_set:
            case Topic::action_session_config_get:
            case Topic::action_session_config_content:
            case Topic::action_car_config:
            case Topic::action_car_config_set:
            case Topic::action_car_config_get:
            case Topic::action_car_config_content:
            case Topic::action_baseline_config:
            case Topic::action_baseline_config_set:
            case Topic::action_baseline_config_get:
            case Topic::action_baseline_config_content:
            case Topic::action_handcart_settings:
            case Topic::action_handcart_settings_set:
            case Topic::action_handcart_settings_get:
            case Topic::action_handcart_settings_content:
            case Topic::raw_json_config:
            case Topic::raw_json_config_set:
            case Topic::raw_json_config_get:
            case Topic::raw_json_config_content:
            case Topic::action_lap_counter_config:
            case Topic::action_lap_counter_config_set:
            case Topic::action_lap_counter_config_get:
            case Topic::action_lap_counter_config_content:
            case Topic::action_lap_counter_track:
            case Topic::action_lap_counter_track_set:
            case Topic::action_lap_counter_track_get:
            case Topic::action_lap_counter_track_content:
            case Topic::action_kill:
            case Topic::action_start:
            case Topic::action_reset:
            case Topic::action_stop:
            case Topic::action_start_baseline:
            case Topic::action_stop_baseline:
            case Topic::action_precharge:
            case Topic::action_balance:
            case Topic::action_stop_balance:
            case Topic::action_charge:
            case Topic::action_stop_charge:
            case Topic::action_raw:
            case Topic::action_reset_lap_counter:
            case Topic::action_set_lap_counter_status:
            case Topic::simulator:
            case Topic::simulator_inputs:
            case Topic::simulator_outputs:
            case Topic::simulator_initial_state:
             
            default: 
              return false;
        }
    }

    return false;
}

TopicMessage GetTopic(Topic topic, const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    switch(topic) {
        case Topic::vehicle_id: 
            return GetTopicVehicleId(vehicleId);
        case Topic::device_id: 
            return GetTopicDeviceId(vehicleId, deviceId);
        case Topic::device_version: 
            return GetTopicDeviceVersion(vehicleId, deviceId);
        case Topic::data: 
            return GetTopicData(vehicleId, deviceId);
        case Topic::data_primary: 
            return GetTopicDataPrimary(vehicleId, deviceId);
        case Topic::data_secondary: 
            return GetTopicDataSecondary(vehicleId, deviceId);
        case Topic::data_bms: 
            return GetTopicDataBms(vehicleId, deviceId);
        case Topic::data_inverter: 
            return GetTopicDataInverter(vehicleId, deviceId);
        case Topic::data_simulator: 
            return GetTopicDataSimulator(vehicleId, deviceId);
        case Topic::data_gps: 
            return GetTopicDataGps(vehicleId, deviceId);
        case Topic::data_brusa: 
            return GetTopicDataBrusa(vehicleId, deviceId);
        case Topic::data_temporary: 
            return GetTopicDataTemporary(vehicleId, deviceId);
        case Topic::data_camera_logger: 
            return GetTopicDataCameraLogger(vehicleId, deviceId);
        case Topic::data_camera_logger_can: 
            return GetTopicDataCameraLoggerCan(vehicleId, deviceId);
        case Topic::data_camera_logger_can_imu_angular_rate: 
            return GetTopicDataCameraLoggerCanImuAngularRate(vehicleId, deviceId);
        case Topic::data_camera_logger_can_imu_acceleration: 
            return GetTopicDataCameraLoggerCanImuAcceleration(vehicleId, deviceId);
        case Topic::data_camera_logger_can_vehicle_position: 
            return GetTopicDataCameraLoggerCanVehiclePosition(vehicleId, deviceId);
        case Topic::data_camera_logger_can_vehicle_speed: 
            return GetTopicDataCameraLoggerCanVehicleSpeed(vehicleId, deviceId);
        case Topic::data_camera_logger_can_front_angular_velocity: 
            return GetTopicDataCameraLoggerCanFrontAngularVelocity(vehicleId, deviceId);
        case Topic::data_camera_logger_can_rear_left_angular_velocity: 
            return GetTopicDataCameraLoggerCanRearLeftAngularVelocity(vehicleId, deviceId);
        case Topic::data_camera_logger_can_rear_right_angular_velocity: 
            return GetTopicDataCameraLoggerCanRearRightAngularVelocity(vehicleId, deviceId);
        case Topic::data_camera_logger_can_steer_angle: 
            return GetTopicDataCameraLoggerCanSteerAngle(vehicleId, deviceId);
        case Topic::data_camera_logger_gps: 
            return GetTopicDataCameraLoggerGps(vehicleId, deviceId);
        case Topic::data_camera_logger_gps_hpposllh: 
            return GetTopicDataCameraLoggerGpsHpposllh(vehicleId, deviceId);
        case Topic::data_camera_logger_gps_pvt: 
            return GetTopicDataCameraLoggerGpsPvt(vehicleId, deviceId);
        case Topic::data_camera_logger_gps_relposned: 
            return GetTopicDataCameraLoggerGpsRelposned(vehicleId, deviceId);
        case Topic::extra_data_to_log: 
            return GetTopicExtraDataToLog(vehicleId, deviceId);
        case Topic::extra_tlm_data: 
            return GetTopicExtraTlmData(vehicleId, deviceId);
        case Topic::extra_tlm_data_vehicle_state: 
            return GetTopicExtraTlmDataVehicleState(vehicleId, deviceId);
        case Topic::extra_tlm_data_baseline: 
            return GetTopicExtraTlmDataBaseline(vehicleId, deviceId);
        case Topic::extra_tlm_data_gps_map_origins: 
            return GetTopicExtraTlmDataGpsMapOrigins(vehicleId, deviceId);
        case Topic::data_last_update: 
            return GetTopicDataLastUpdate(vehicleId, deviceId);
        case Topic::status: 
            return GetTopicStatus(vehicleId, deviceId);
        case Topic::status_info: 
            return GetTopicStatusInfo(vehicleId, deviceId);
        case Topic::status_error: 
            return GetTopicStatusError(vehicleId, deviceId);
        case Topic::status_alert: 
            return GetTopicStatusAlert(vehicleId, deviceId);
        case Topic::status_can_frequencies: 
            return GetTopicStatusCanFrequencies(vehicleId, deviceId);
        case Topic::status_lap_counter_status: 
            return GetTopicStatusLapCounterStatus(vehicleId, deviceId);
        case Topic::status_lap_counter_laps: 
            return GetTopicStatusLapCounterLaps(vehicleId, deviceId);
        case Topic::as: 
            return GetTopicAs(vehicleId, deviceId);
        case Topic::as_commands: 
            return GetTopicAsCommands(vehicleId, deviceId);
        case Topic::as_commands_set_values: 
            return GetTopicAsCommandsSetValues(vehicleId, deviceId);
        case Topic::as_commands_set_status: 
            return GetTopicAsCommandsSetStatus(vehicleId, deviceId);
        case Topic::file_transaction_request: 
            return GetTopicFileTransactionRequest(vehicleId, deviceId);
        case Topic::file_transaction_response: 
            return GetTopicFileTransactionResponse(vehicleId, deviceId);
        case Topic::file_transaction: 
            return GetTopicFileTransaction(vehicleId, deviceId, transactionId);
        case Topic::file_transaction_begin: 
            return GetTopicFileTransactionBegin(vehicleId, deviceId, transactionId);
        case Topic::file_transaction_end: 
            return GetTopicFileTransactionEnd(vehicleId, deviceId, transactionId);
        case Topic::file_transaction_chunk: 
            return GetTopicFileTransactionChunk(vehicleId, deviceId, transactionId);
        case Topic::file_transaction_chunk_ack: 
            return GetTopicFileTransactionChunkAck(vehicleId, deviceId, transactionId);
        case Topic::info: 
            return GetTopicInfo(vehicleId, deviceId);
        case Topic::info_version: 
            return GetTopicInfoVersion(vehicleId, deviceId);
        case Topic::info_user: 
            return GetTopicInfoUser(vehicleId, deviceId);
        case Topic::info_telemetry_replay: 
            return GetTopicInfoTelemetryReplay(vehicleId, deviceId);
        case Topic::info_telemetry_replay_ready: 
            return GetTopicInfoTelemetryReplayReady(vehicleId, deviceId);
        case Topic::info_telemetry_replay_start: 
            return GetTopicInfoTelemetryReplayStart(vehicleId, deviceId);
        case Topic::info_session: 
            return GetTopicInfoSession(vehicleId, deviceId);
        case Topic::info_session_started: 
            return GetTopicInfoSessionStarted(vehicleId, deviceId);
        case Topic::info_session_stopped: 
            return GetTopicInfoSessionStopped(vehicleId, deviceId);
        case Topic::info_session_keepalive: 
            return GetTopicInfoSessionKeepalive(vehicleId, deviceId);
        case Topic::action: 
            return GetTopicAction(vehicleId, deviceId);
        case Topic::action_telemetry_config: 
            return GetTopicActionTelemetryConfig(vehicleId, deviceId);
        case Topic::action_telemetry_config_set: 
            return GetTopicActionTelemetryConfigSet(vehicleId, deviceId);
        case Topic::action_telemetry_config_get: 
            return GetTopicActionTelemetryConfigGet(vehicleId, deviceId);
        case Topic::action_telemetry_config_content: 
            return GetTopicActionTelemetryConfigContent(vehicleId, deviceId);
        case Topic::action_session_config: 
            return GetTopicActionSessionConfig(vehicleId, deviceId);
        case Topic::action_session_config_set: 
            return GetTopicActionSessionConfigSet(vehicleId, deviceId);
        case Topic::action_session_config_get: 
            return GetTopicActionSessionConfigGet(vehicleId, deviceId);
        case Topic::action_session_config_content: 
            return GetTopicActionSessionConfigContent(vehicleId, deviceId);
        case Topic::action_car_config: 
            return GetTopicActionCarConfig(vehicleId, deviceId);
        case Topic::action_car_config_set: 
            return GetTopicActionCarConfigSet(vehicleId, deviceId);
        case Topic::action_car_config_get: 
            return GetTopicActionCarConfigGet(vehicleId, deviceId);
        case Topic::action_car_config_content: 
            return GetTopicActionCarConfigContent(vehicleId, deviceId);
        case Topic::action_baseline_config: 
            return GetTopicActionBaselineConfig(vehicleId, deviceId);
        case Topic::action_baseline_config_set: 
            return GetTopicActionBaselineConfigSet(vehicleId, deviceId);
        case Topic::action_baseline_config_get: 
            return GetTopicActionBaselineConfigGet(vehicleId, deviceId);
        case Topic::action_baseline_config_content: 
            return GetTopicActionBaselineConfigContent(vehicleId, deviceId);
        case Topic::action_handcart_settings: 
            return GetTopicActionHandcartSettings(vehicleId, deviceId);
        case Topic::action_handcart_settings_set: 
            return GetTopicActionHandcartSettingsSet(vehicleId, deviceId);
        case Topic::action_handcart_settings_get: 
            return GetTopicActionHandcartSettingsGet(vehicleId, deviceId);
        case Topic::action_handcart_settings_content: 
            return GetTopicActionHandcartSettingsContent(vehicleId, deviceId);
        case Topic::raw_json_config: 
            return GetTopicRawJsonConfig(vehicleId, deviceId);
        case Topic::raw_json_config_set: 
            return GetTopicRawJsonConfigSet(vehicleId, deviceId);
        case Topic::raw_json_config_get: 
            return GetTopicRawJsonConfigGet(vehicleId, deviceId);
        case Topic::raw_json_config_content: 
            return GetTopicRawJsonConfigContent(vehicleId, deviceId);
        case Topic::action_lap_counter_config: 
            return GetTopicActionLapCounterConfig(vehicleId, deviceId);
        case Topic::action_lap_counter_config_set: 
            return GetTopicActionLapCounterConfigSet(vehicleId, deviceId);
        case Topic::action_lap_counter_config_get: 
            return GetTopicActionLapCounterConfigGet(vehicleId, deviceId);
        case Topic::action_lap_counter_config_content: 
            return GetTopicActionLapCounterConfigContent(vehicleId, deviceId);
        case Topic::action_lap_counter_track: 
            return GetTopicActionLapCounterTrack(vehicleId, deviceId);
        case Topic::action_lap_counter_track_set: 
            return GetTopicActionLapCounterTrackSet(vehicleId, deviceId);
        case Topic::action_lap_counter_track_get: 
            return GetTopicActionLapCounterTrackGet(vehicleId, deviceId);
        case Topic::action_lap_counter_track_content: 
            return GetTopicActionLapCounterTrackContent(vehicleId, deviceId);
        case Topic::action_kill: 
            return GetTopicActionKill(vehicleId, deviceId);
        case Topic::action_start: 
            return GetTopicActionStart(vehicleId, deviceId);
        case Topic::action_reset: 
            return GetTopicActionReset(vehicleId, deviceId);
        case Topic::action_stop: 
            return GetTopicActionStop(vehicleId, deviceId);
        case Topic::action_start_baseline: 
            return GetTopicActionStartBaseline(vehicleId, deviceId);
        case Topic::action_stop_baseline: 
            return GetTopicActionStopBaseline(vehicleId, deviceId);
        case Topic::action_precharge: 
            return GetTopicActionPrecharge(vehicleId, deviceId);
        case Topic::action_balance: 
            return GetTopicActionBalance(vehicleId, deviceId);
        case Topic::action_stop_balance: 
            return GetTopicActionStopBalance(vehicleId, deviceId);
        case Topic::action_charge: 
            return GetTopicActionCharge(vehicleId, deviceId);
        case Topic::action_stop_charge: 
            return GetTopicActionStopCharge(vehicleId, deviceId);
        case Topic::action_raw: 
            return GetTopicActionRaw(vehicleId, deviceId);
        case Topic::action_reset_lap_counter: 
            return GetTopicActionResetLapCounter(vehicleId, deviceId);
        case Topic::action_set_lap_counter_status: 
            return GetTopicActionSetLapCounterStatus(vehicleId, deviceId);
        case Topic::simulator: 
            return GetTopicSimulator(vehicleId);
        case Topic::simulator_inputs: 
            return GetTopicSimulatorInputs(vehicleId);
        case Topic::simulator_outputs: 
            return GetTopicSimulatorOutputs(vehicleId);
        case Topic::simulator_initial_state: 
            return GetTopicSimulatorInitialState(vehicleId);
    }

    return TopicMessage("", 0, false);
}

TopicMessage GetTopicVehicleId(const std::string& vehicleId) {
    std::string topic = vehicleId;
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDeviceId(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId;
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDeviceVersion(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "version";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicData(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataPrimary(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/primary";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataSecondary(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/secondary";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataBms(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/bms";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataInverter(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/inverters";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataSimulator(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/simulator";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataGps(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/gps";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataBrusa(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/brusa";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataTemporary(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/temporary";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLogger(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCan(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanImuAngularRate(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/imuAngularRate";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanImuAcceleration(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/imuAcceleration";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanVehiclePosition(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/vehiclePosition";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanVehicleSpeed(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/vehicleSpeed";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanFrontAngularVelocity(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/frontAngularVelocity";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanRearLeftAngularVelocity(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/rearLeftAngularVelocity";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanRearRightAngularVelocity(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/rearRightAngularVelocity";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerCanSteerAngle(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/can/steerAngle";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerGps(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerGpsHpposllh(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/hpposllh";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerGpsPvt(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/pvt";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataCameraLoggerGpsRelposned(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "data/cameraLogger/GPS/relposned";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicExtraDataToLog(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "extra_data_to_log";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicExtraTlmData(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicExtraTlmDataVehicleState(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/vehicleState";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicExtraTlmDataBaseline(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/baseline";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicExtraTlmDataGpsMapOrigins(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "extra_tlm_data/gpsMapOrigins";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicDataLastUpdate(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "last_update";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatus(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusInfo(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/info";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusError(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/error";
    int qos = 1;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusAlert(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/alert";
    int qos = 1;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusCanFrequencies(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/canFrequencies";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusLapCounterStatus(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/lapCounterStatus";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicStatusLapCounterLaps(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "status/lapCounterLaps";
    int qos = 1;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicAs(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "as";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicAsCommands(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "as/commands";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicAsCommandsSetValues(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "as/commands/setValues";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicAsCommandsSetStatus(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "as/commands/setStatus";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionRequest(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction/request";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionResponse(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction/response";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransaction(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId;
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionBegin(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "begin";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionEnd(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "end";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionChunk(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "chunk";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicFileTransactionChunkAck(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "fileTransaction" + "/" + transactionId + "/" + "chunk_ack";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfo(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoVersion(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/version";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoUser(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/user";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoTelemetryReplay(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoTelemetryReplayReady(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay/ready";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoTelemetryReplayStart(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/telemetryReplay/start";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoSession(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/session";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoSessionStarted(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/session/started";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoSessionStopped(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/session/stopped";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicInfoSessionKeepalive(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "info/session/logging";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicAction(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionTelemetryConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionTelemetryConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionTelemetryConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionTelemetryConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/telemetryConfig/content";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionSessionConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionSessionConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionSessionConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionSessionConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/sessionConfig/content";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionCarConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/carConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionCarConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionCarConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionCarConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/carConfig/content";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionBaselineConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionBaselineConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionBaselineConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionBaselineConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/baselineConfig/content";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionHandcartSettings(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionHandcartSettingsSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionHandcartSettingsGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionHandcartSettingsContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/handcartSettings/content";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicRawJsonConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicRawJsonConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicRawJsonConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicRawJsonConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/rawJsonConfig/content";
    int qos = 0;
    bool retain = true;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterConfig(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterConfigSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterConfigGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterConfigContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterConfig/content";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterTrack(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterTrackSet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/set";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterTrackGet(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/get";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionLapCounterTrackContent(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/lapCounterTrack/content";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionKill(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/kill";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStart(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/start";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionReset(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/reset";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStop(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/stop";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStartBaseline(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/startBaseline";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStopBaseline(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/stopBaseline";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionPrecharge(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/precharge";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionBalance(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/balance";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStopBalance(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/stopBalance";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionCharge(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/charge";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionStopCharge(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/stopCharge";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionRaw(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/raw";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionResetLapCounter(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/resetLapcounter";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicActionSetLapCounterStatus(const std::string& vehicleId, const std::string& deviceId) {
    std::string topic = vehicleId + "/" + deviceId + "/" + "action/setLapcounterStatus";
    int qos = 2;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicSimulator(const std::string& vehicleId) {
    std::string topic = vehicleId + "/" + "simulator";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicSimulatorInputs(const std::string& vehicleId) {
    std::string topic = vehicleId + "/" + "simulator/inputs";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicSimulatorOutputs(const std::string& vehicleId) {
    std::string topic = vehicleId + "/" + "simulator/outputs";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}

TopicMessage GetTopicSimulatorInitialState(const std::string& vehicleId) {
    std::string topic = vehicleId + "/" + "simulator/initialState";
    int qos = 0;
    bool retain = false;

    return TopicMessage(std::move(topic), qos, retain);
}
}