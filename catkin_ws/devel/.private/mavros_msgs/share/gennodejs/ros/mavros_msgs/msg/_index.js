
"use strict";

let ExtendedState = require('./ExtendedState.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let HilControls = require('./HilControls.js');
let GimbalManagerStatus = require('./GimbalManagerStatus.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let Waypoint = require('./Waypoint.js');
let GPSRAW = require('./GPSRAW.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let Trajectory = require('./Trajectory.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let LogEntry = require('./LogEntry.js');
let LogData = require('./LogData.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let HilSensor = require('./HilSensor.js');
let RTCM = require('./RTCM.js');
let ESCStatus = require('./ESCStatus.js');
let PositionTarget = require('./PositionTarget.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let GimbalDeviceSetAttitude = require('./GimbalDeviceSetAttitude.js');
let TerrainReport = require('./TerrainReport.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let RCOut = require('./RCOut.js');
let WaypointReached = require('./WaypointReached.js');
let Tunnel = require('./Tunnel.js');
let FileEntry = require('./FileEntry.js');
let StatusText = require('./StatusText.js');
let Thrust = require('./Thrust.js');
let RCIn = require('./RCIn.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let GimbalManagerSetAttitude = require('./GimbalManagerSetAttitude.js');
let GPSRTK = require('./GPSRTK.js');
let GPSINPUT = require('./GPSINPUT.js');
let GimbalManagerInformation = require('./GimbalManagerInformation.js');
let GimbalDeviceAttitudeStatus = require('./GimbalDeviceAttitudeStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let DebugValue = require('./DebugValue.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HomePosition = require('./HomePosition.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let Altitude = require('./Altitude.js');
let WaypointList = require('./WaypointList.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let VFR_HUD = require('./VFR_HUD.js');
let VehicleInfo = require('./VehicleInfo.js');
let CellularStatus = require('./CellularStatus.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let RTKBaseline = require('./RTKBaseline.js');
let ESCInfo = require('./ESCInfo.js');
let ActuatorControl = require('./ActuatorControl.js');
let MountControl = require('./MountControl.js');
let HilGPS = require('./HilGPS.js');
let ManualControl = require('./ManualControl.js');
let State = require('./State.js');
let BatteryStatus = require('./BatteryStatus.js');
let RadioStatus = require('./RadioStatus.js');
let CommandCode = require('./CommandCode.js');
let GimbalDeviceInformation = require('./GimbalDeviceInformation.js');
let Mavlink = require('./Mavlink.js');
let GimbalManagerSetPitchyaw = require('./GimbalManagerSetPitchyaw.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let Vibration = require('./Vibration.js');
let Param = require('./Param.js');
let LandingTarget = require('./LandingTarget.js');
let ParamValue = require('./ParamValue.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');

module.exports = {
  ExtendedState: ExtendedState,
  TimesyncStatus: TimesyncStatus,
  HilControls: HilControls,
  GimbalManagerStatus: GimbalManagerStatus,
  OverrideRCIn: OverrideRCIn,
  Waypoint: Waypoint,
  GPSRAW: GPSRAW,
  ESCTelemetryItem: ESCTelemetryItem,
  Trajectory: Trajectory,
  EstimatorStatus: EstimatorStatus,
  LogEntry: LogEntry,
  LogData: LogData,
  WheelOdomStamped: WheelOdomStamped,
  GlobalPositionTarget: GlobalPositionTarget,
  ESCInfoItem: ESCInfoItem,
  ESCTelemetry: ESCTelemetry,
  HilSensor: HilSensor,
  RTCM: RTCM,
  ESCStatus: ESCStatus,
  PositionTarget: PositionTarget,
  AttitudeTarget: AttitudeTarget,
  GimbalDeviceSetAttitude: GimbalDeviceSetAttitude,
  TerrainReport: TerrainReport,
  PlayTuneV2: PlayTuneV2,
  RCOut: RCOut,
  WaypointReached: WaypointReached,
  Tunnel: Tunnel,
  FileEntry: FileEntry,
  StatusText: StatusText,
  Thrust: Thrust,
  RCIn: RCIn,
  HilActuatorControls: HilActuatorControls,
  GimbalManagerSetAttitude: GimbalManagerSetAttitude,
  GPSRTK: GPSRTK,
  GPSINPUT: GPSINPUT,
  GimbalManagerInformation: GimbalManagerInformation,
  GimbalDeviceAttitudeStatus: GimbalDeviceAttitudeStatus,
  OpticalFlowRad: OpticalFlowRad,
  DebugValue: DebugValue,
  HilStateQuaternion: HilStateQuaternion,
  HomePosition: HomePosition,
  ESCStatusItem: ESCStatusItem,
  OnboardComputerStatus: OnboardComputerStatus,
  Altitude: Altitude,
  WaypointList: WaypointList,
  ADSBVehicle: ADSBVehicle,
  NavControllerOutput: NavControllerOutput,
  CamIMUStamp: CamIMUStamp,
  VFR_HUD: VFR_HUD,
  VehicleInfo: VehicleInfo,
  CellularStatus: CellularStatus,
  MagnetometerReporter: MagnetometerReporter,
  RTKBaseline: RTKBaseline,
  ESCInfo: ESCInfo,
  ActuatorControl: ActuatorControl,
  MountControl: MountControl,
  HilGPS: HilGPS,
  ManualControl: ManualControl,
  State: State,
  BatteryStatus: BatteryStatus,
  RadioStatus: RadioStatus,
  CommandCode: CommandCode,
  GimbalDeviceInformation: GimbalDeviceInformation,
  Mavlink: Mavlink,
  GimbalManagerSetPitchyaw: GimbalManagerSetPitchyaw,
  CameraImageCaptured: CameraImageCaptured,
  Vibration: Vibration,
  Param: Param,
  LandingTarget: LandingTarget,
  ParamValue: ParamValue,
  CompanionProcessStatus: CompanionProcessStatus,
};
