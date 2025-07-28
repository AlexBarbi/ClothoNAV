# Topics

## VehicleId
### &lt;vehicleId>
> &lt;vehicleId>
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleId
## DeviceId
### &lt;vehicleId>/&lt;deviceId>
> &lt;vehicleId>/&lt;deviceId>
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DeviceVersion
### &lt;vehicleId>/&lt;deviceId>/version
> The payload contains the current CAN version hash of the device.
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## Data
### &lt;vehicleId>/&lt;deviceId>/data
> &lt;vehicleId>/&lt;deviceId>/data
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataPrimary
### &lt;vehicleId>/&lt;deviceId>/data/primary
> Protobuffer data of primary network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataSecondary
### &lt;vehicleId>/&lt;deviceId>/data/secondary
> Protobuffer data of secondary network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataBms
### &lt;vehicleId>/&lt;deviceId>/data/bms
> Protobuffer data of bms network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataInverter
### &lt;vehicleId>/&lt;deviceId>/data/inverters
> Protobuffer data of inverter network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataSimulator
### &lt;vehicleId>/&lt;deviceId>/data/simulator
> Protobuffer data of simulator network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataGps
### &lt;vehicleId>/&lt;deviceId>/data/gps
> Protobuffer data of gps network
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataBrusa
### &lt;vehicleId>/&lt;deviceId>/data/brusa
> &lt;vehicleId>/&lt;deviceId>/data/brusa
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataTemporary
### &lt;vehicleId>/&lt;deviceId>/data/temporary
> Proto serialized with serializers, with unspecified and volatile structure.
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLogger
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCan
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanImuAngularRate
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/imuAngularRate
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/imuAngularRate
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanImuAcceleration
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/imuAcceleration
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/imuAcceleration
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanVehiclePosition
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/vehiclePosition
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/vehiclePosition
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanVehicleSpeed
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/vehicleSpeed
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/vehicleSpeed
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanFrontAngularVelocity
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/frontAngularVelocity
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/frontAngularVelocity
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanRearLeftAngularVelocity
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/rearLeftAngularVelocity
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/rearLeftAngularVelocity
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanRearRightAngularVelocity
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/rearRightAngularVelocity
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/rearRightAngularVelocity
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerCanSteerAngle
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/steerAngle
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/can/steerAngle
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerGps
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerGpsHpposllh
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/hpposllh
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/hpposllh
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerGpsPvt
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/pvt
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/pvt
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## DataCameraLoggerGpsRelposned
### &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/relposned
> &lt;vehicleId>/&lt;deviceId>/data/cameraLogger/GPS/relposned
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ExtraDataToLog
### &lt;vehicleId>/&lt;deviceId>/extra_data_to_log
> &lt;vehicleId>/&lt;deviceId>/extra_data_to_log
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ExtraTlmData
### &lt;vehicleId>/&lt;deviceId>/extra_tlm_data
> &lt;vehicleId>/&lt;deviceId>/extra_tlm_data
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ExtraTlmDataVehicleState
### &lt;vehicleId>/&lt;deviceId>/extra_tlm_data/vehicleState
> Protobuffer data of vehicle state
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ExtraTlmDataBaseline
### &lt;vehicleId>/&lt;deviceId>/extra_tlm_data/baseline
> Protobuffer data of vehicle state
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ExtraTlmDataGpsMapOrigins
### &lt;vehicleId>/&lt;deviceId>/extra_tlm_data/gpsMapOrigins
> Protobuffer data of gps map origins
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 131 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## DataLastUpdate
### &lt;vehicleId>/&lt;deviceId>/last_update
> Sends the last update timestamp of the data
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## Status
### &lt;vehicleId>/&lt;deviceId>/status
> &lt;vehicleId>/&lt;deviceId>/status
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## StatusInfo
### &lt;vehicleId>/&lt;deviceId>/status/info
> &lt;vehicleId>/&lt;deviceId>/status/info
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## StatusError
### &lt;vehicleId>/&lt;deviceId>/status/error
> &lt;vehicleId>/&lt;deviceId>/status/error
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## StatusAlert
### &lt;vehicleId>/&lt;deviceId>/status/alert
> Alert message, it contains a critical can message
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## StatusCanFrequencies
### &lt;vehicleId>/&lt;deviceId>/status/canFrequencies
> &lt;vehicleId>/&lt;deviceId>/status/canFrequencies
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## StatusLapCounterStatus
### &lt;vehicleId>/&lt;deviceId>/status/lapCounterStatus
> &lt;vehicleId>/&lt;deviceId>/status/lapCounterStatus
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 128 129 131 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## StatusLapCounterLaps
### &lt;vehicleId>/&lt;deviceId>/status/lapCounterLaps
> &lt;vehicleId>/&lt;deviceId>/status/lapCounterLaps
- **Quality of Service**: 1
- **Subscribe Roles**: 0 1 2 3 4 128 129 131 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## As
### &lt;vehicleId>/&lt;deviceId>/as
> &lt;vehicleId>/&lt;deviceId>/as
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## AsCommands
### &lt;vehicleId>/&lt;deviceId>/as/commands
> &lt;vehicleId>/&lt;deviceId>/as/commands
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## AsCommandsSetValues
### &lt;vehicleId>/&lt;deviceId>/as/commands/setValues
> &lt;vehicleId>/&lt;deviceId>/as/commands/setValues
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 128 130 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## AsCommandsSetStatus
### &lt;vehicleId>/&lt;deviceId>/as/commands/setStatus
> &lt;vehicleId>/&lt;deviceId>/as/commands/setStatus
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 128 130 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## FileTransactionRequest
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/request
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/request
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## FileTransactionResponse
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/response
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/response
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## FileTransaction
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceIdtransactionId
## FileTransactionBegin
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/begin
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/begin
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceIdtransactionId
## FileTransactionEnd
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/end
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/end
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceIdtransactionId
## FileTransactionChunk
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/chunk
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/chunk
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceIdtransactionId
## FileTransactionChunkAck
### &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/chunk_ack
> &lt;vehicleId>/&lt;deviceId>/fileTransaction/&lt;transactionId>/chunk_ack
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceIdtransactionId
## Info
### &lt;vehicleId>/&lt;deviceId>/info
> &lt;vehicleId>/&lt;deviceId>/info
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoVersion
### &lt;vehicleId>/&lt;deviceId>/info/version
> &lt;vehicleId>/&lt;deviceId>/info
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoUser
### &lt;vehicleId>/&lt;deviceId>/info/user
> &lt;vehicleId>/&lt;deviceId>/info
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoTelemetryReplay
### &lt;vehicleId>/&lt;deviceId>/info/telemetryReplay
> &lt;vehicleId>/&lt;deviceId>/info
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoTelemetryReplayReady
### &lt;vehicleId>/&lt;deviceId>/info/telemetryReplay/ready
> Sent when the telemetry replay is ready
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 130 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoTelemetryReplayStart
### &lt;vehicleId>/&lt;deviceId>/info/telemetryReplay/start
> Sent to start telemetry replay
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 130 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoSession
### &lt;vehicleId>/&lt;deviceId>/info/session
> &lt;vehicleId>/&lt;deviceId>/info
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoSessionStarted
### &lt;vehicleId>/&lt;deviceId>/info/session/started
> Sent when the session is started
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoSessionStopped
### &lt;vehicleId>/&lt;deviceId>/info/session/stopped
> Sent when the session is stopped
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## InfoSessionKeepalive
### &lt;vehicleId>/&lt;deviceId>/info/session/logging
> Repeatedly sent, like a keepalive, to indicate that the session is still active
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## Action
### &lt;vehicleId>/&lt;deviceId>/action
> &lt;vehicleId>/&lt;deviceId>/action
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionTelemetryConfig
### &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig
> &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 131 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionTelemetryConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionTelemetryConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionTelemetryConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/telemetryConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ActionSessionConfig
### &lt;vehicleId>/&lt;deviceId>/action/sessionConfig
> &lt;vehicleId>/&lt;deviceId>/action/sessionConfig
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionSessionConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionSessionConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionSessionConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/sessionConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 131 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ActionCarConfig
### &lt;vehicleId>/&lt;deviceId>/action/carConfig
> &lt;vehicleId>/&lt;deviceId>/action/actionCar
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionCarConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/carConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/carConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionCarConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/carConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/carConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionCarConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/carConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/carConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 131 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ActionBaselineConfig
### &lt;vehicleId>/&lt;deviceId>/action/baselineConfig
> &lt;vehicleId>/&lt;deviceId>/action/actionBaseline
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionBaselineConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionBaselineConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionBaselineConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/baselineConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 131 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ActionHandcartSettings
### &lt;vehicleId>/&lt;deviceId>/action/handcartSettings
> &lt;vehicleId>/&lt;deviceId>/action/handcartSettings
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionHandcartSettingsSet
### &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/set
> &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionHandcartSettingsGet
### &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/get
> &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionHandcartSettingsContent
### &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/content
> &lt;vehicleId>/&lt;deviceId>/action/handcartSettings/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## RawJsonConfig
### &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig
> &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## RawJsonConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## RawJsonConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## RawJsonConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/rawJsonConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: Yes
- **Variables**:
vehicleIddeviceId
## ActionLapCounterConfig
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterConfigSet
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/set
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterConfigGet
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/get
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterConfigContent
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/content
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterConfig/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterTrack
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterTrackSet
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/set
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/set
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterTrackGet
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/get
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/get
- **Quality of Service**: 0
- **Subscribe Roles**: 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionLapCounterTrackContent
### &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/content
> &lt;vehicleId>/&lt;deviceId>/action/lapCounterTrack/content
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 131 - **Publish Roles**: 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionKill
### &lt;vehicleId>/&lt;deviceId>/action/kill
> &lt;vehicleId>/&lt;deviceId>/action/kill
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStart
### &lt;vehicleId>/&lt;deviceId>/action/start
> &lt;vehicleId>/&lt;deviceId>/action/start
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionReset
### &lt;vehicleId>/&lt;deviceId>/action/reset
> &lt;vehicleId>/&lt;deviceId>/action/reset
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStop
### &lt;vehicleId>/&lt;deviceId>/action/stop
> &lt;vehicleId>/&lt;deviceId>/action/stop
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStartBaseline
### &lt;vehicleId>/&lt;deviceId>/action/startBaseline
> &lt;vehicleId>/&lt;deviceId>/action/start_baseline
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStopBaseline
### &lt;vehicleId>/&lt;deviceId>/action/stopBaseline
> &lt;vehicleId>/&lt;deviceId>/action/stop_baseline
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionPrecharge
### &lt;vehicleId>/&lt;deviceId>/action/precharge
> &lt;vehicleId>/&lt;deviceId>/action/precharge
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionBalance
### &lt;vehicleId>/&lt;deviceId>/action/balance
> &lt;vehicleId>/&lt;deviceId>/action/balance
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStopBalance
### &lt;vehicleId>/&lt;deviceId>/action/stopBalance
> &lt;vehicleId>/&lt;deviceId>/action/stopBalance
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionCharge
### &lt;vehicleId>/&lt;deviceId>/action/charge
> &lt;vehicleId>/&lt;deviceId>/action/charge
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionStopCharge
### &lt;vehicleId>/&lt;deviceId>/action/stopCharge
> &lt;vehicleId>/&lt;deviceId>/action/stopCharge
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionRaw
### &lt;vehicleId>/&lt;deviceId>/action/raw
> &lt;vehicleId>/&lt;deviceId>/action/raw
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 128 129 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionResetLapCounter
### &lt;vehicleId>/&lt;deviceId>/action/resetLapcounter
> &lt;vehicleId>/&lt;deviceId>/action/resetLapcounter
- **Quality of Service**: 2
- **Subscribe Roles**: 0 1 2 3 4 128 129 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## ActionSetLapCounterStatus
### &lt;vehicleId>/&lt;deviceId>/action/setLapcounterStatus
> &lt;vehicleId>/&lt;deviceId>/action/setLapcounterStatus
- **Quality of Service**: 2
- **Subscribe Roles**: 128 129 131 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleIddeviceId
## Simulator
### &lt;vehicleId>/simulator
> data from and to the simulator
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleId
## SimulatorInputs
### &lt;vehicleId>/simulator/inputs
> 
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleId
## SimulatorOutputs
### &lt;vehicleId>/simulator/outputs
> 
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleId
## SimulatorInitialState
### &lt;vehicleId>/simulator/initialState
> 
- **Quality of Service**: 0
- **Subscribe Roles**: 0 1 2 3 4 - **Publish Roles**: 0 1 2 3 4 
- **Retain**: No
- **Variables**:
vehicleId
