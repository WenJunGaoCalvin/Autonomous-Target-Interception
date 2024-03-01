
"use strict";

let VehicleInfoGet = require('./VehicleInfoGet.js')
let CommandBool = require('./CommandBool.js')
let GimbalManagerSetRoi = require('./GimbalManagerSetRoi.js')
let GimbalGetInformation = require('./GimbalGetInformation.js')
let CommandAck = require('./CommandAck.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let MessageInterval = require('./MessageInterval.js')
let StreamRate = require('./StreamRate.js')
let ParamPull = require('./ParamPull.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let ParamPush = require('./ParamPush.js')
let ParamSet = require('./ParamSet.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let WaypointClear = require('./WaypointClear.js')
let FileRename = require('./FileRename.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let LogRequestList = require('./LogRequestList.js')
let GimbalManagerPitchyaw = require('./GimbalManagerPitchyaw.js')
let LogRequestData = require('./LogRequestData.js')
let GimbalManagerConfigure = require('./GimbalManagerConfigure.js')
let WaypointPush = require('./WaypointPush.js')
let MountConfigure = require('./MountConfigure.js')
let CommandLong = require('./CommandLong.js')
let CommandTOL = require('./CommandTOL.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let FileChecksum = require('./FileChecksum.js')
let FileOpen = require('./FileOpen.js')
let FileTruncate = require('./FileTruncate.js')
let FileClose = require('./FileClose.js')
let GimbalManagerCameraTrack = require('./GimbalManagerCameraTrack.js')
let FileWrite = require('./FileWrite.js')
let SetMode = require('./SetMode.js')
let FileRead = require('./FileRead.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileMakeDir = require('./FileMakeDir.js')
let FileRemove = require('./FileRemove.js')
let ParamGet = require('./ParamGet.js')
let SetMavFrame = require('./SetMavFrame.js')
let WaypointPull = require('./WaypointPull.js')
let CommandHome = require('./CommandHome.js')
let CommandInt = require('./CommandInt.js')
let FileList = require('./FileList.js')

module.exports = {
  VehicleInfoGet: VehicleInfoGet,
  CommandBool: CommandBool,
  GimbalManagerSetRoi: GimbalManagerSetRoi,
  GimbalGetInformation: GimbalGetInformation,
  CommandAck: CommandAck,
  CommandTriggerInterval: CommandTriggerInterval,
  MessageInterval: MessageInterval,
  StreamRate: StreamRate,
  ParamPull: ParamPull,
  WaypointSetCurrent: WaypointSetCurrent,
  ParamPush: ParamPush,
  ParamSet: ParamSet,
  FileRemoveDir: FileRemoveDir,
  WaypointClear: WaypointClear,
  FileRename: FileRename,
  CommandTriggerControl: CommandTriggerControl,
  LogRequestList: LogRequestList,
  GimbalManagerPitchyaw: GimbalManagerPitchyaw,
  LogRequestData: LogRequestData,
  GimbalManagerConfigure: GimbalManagerConfigure,
  WaypointPush: WaypointPush,
  MountConfigure: MountConfigure,
  CommandLong: CommandLong,
  CommandTOL: CommandTOL,
  LogRequestEnd: LogRequestEnd,
  FileChecksum: FileChecksum,
  FileOpen: FileOpen,
  FileTruncate: FileTruncate,
  FileClose: FileClose,
  GimbalManagerCameraTrack: GimbalManagerCameraTrack,
  FileWrite: FileWrite,
  SetMode: SetMode,
  FileRead: FileRead,
  CommandVtolTransition: CommandVtolTransition,
  FileMakeDir: FileMakeDir,
  FileRemove: FileRemove,
  ParamGet: ParamGet,
  SetMavFrame: SetMavFrame,
  WaypointPull: WaypointPull,
  CommandHome: CommandHome,
  CommandInt: CommandInt,
  FileList: FileList,
};
