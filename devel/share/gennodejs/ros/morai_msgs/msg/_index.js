
"use strict";

let Lamps = require('./Lamps.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let TrafficLight = require('./TrafficLight.js');
let ERP42Info = require('./ERP42Info.js');
let CtrlCmd = require('./CtrlCmd.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let IntersectionControl = require('./IntersectionControl.js');
let IntscnTL = require('./IntscnTL.js');
let PREvent = require('./PREvent.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let WaitForTick = require('./WaitForTick.js');
let VehicleSpec = require('./VehicleSpec.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let GhostMessage = require('./GhostMessage.js');
let ObjectStatus = require('./ObjectStatus.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let CollisionData = require('./CollisionData.js');
let SensorPosControl = require('./SensorPosControl.js');
let PRStatus = require('./PRStatus.js');
let VehicleCollision = require('./VehicleCollision.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let GPSMessage = require('./GPSMessage.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let ReplayInfo = require('./ReplayInfo.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let MapSpec = require('./MapSpec.js');
let RadarDetections = require('./RadarDetections.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let EventInfo = require('./EventInfo.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let SaveSensorData = require('./SaveSensorData.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let RadarDetection = require('./RadarDetection.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let MapSpecIndex = require('./MapSpecIndex.js');

module.exports = {
  Lamps: Lamps,
  MoraiTLIndex: MoraiTLIndex,
  MoraiSimProcStatus: MoraiSimProcStatus,
  TrafficLight: TrafficLight,
  ERP42Info: ERP42Info,
  CtrlCmd: CtrlCmd,
  SyncModeAddObject: SyncModeAddObject,
  MoraiSimProcHandle: MoraiSimProcHandle,
  VehicleCollisionData: VehicleCollisionData,
  SetTrafficLight: SetTrafficLight,
  GetTrafficLightStatus: GetTrafficLightStatus,
  PRCtrlCmd: PRCtrlCmd,
  IntersectionControl: IntersectionControl,
  IntscnTL: IntscnTL,
  PREvent: PREvent,
  VehicleSpecIndex: VehicleSpecIndex,
  MoraiTLInfo: MoraiTLInfo,
  WaitForTick: WaitForTick,
  VehicleSpec: VehicleSpec,
  SyncModeCmd: SyncModeCmd,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  GhostMessage: GhostMessage,
  ObjectStatus: ObjectStatus,
  DdCtrlCmd: DdCtrlCmd,
  SyncModeCmdResponse: SyncModeCmdResponse,
  CollisionData: CollisionData,
  SensorPosControl: SensorPosControl,
  PRStatus: PRStatus,
  VehicleCollision: VehicleCollision,
  ScenarioLoad: ScenarioLoad,
  NpcGhostCmd: NpcGhostCmd,
  GPSMessage: GPSMessage,
  SyncModeResultResponse: SyncModeResultResponse,
  ReplayInfo: ReplayInfo,
  NpcGhostInfo: NpcGhostInfo,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  MapSpec: MapSpec,
  RadarDetections: RadarDetections,
  ObjectStatusList: ObjectStatusList,
  EventInfo: EventInfo,
  WaitForTickResponse: WaitForTickResponse,
  EgoVehicleStatus: EgoVehicleStatus,
  SaveSensorData: SaveSensorData,
  SyncModeSetGear: SyncModeSetGear,
  RadarDetection: RadarDetection,
  IntersectionStatus: IntersectionStatus,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  SyncModeRemoveObject: SyncModeRemoveObject,
  SyncModeInfo: SyncModeInfo,
  MultiEgoSetting: MultiEgoSetting,
  MoraiSrvResponse: MoraiSrvResponse,
  MapSpecIndex: MapSpecIndex,
};
