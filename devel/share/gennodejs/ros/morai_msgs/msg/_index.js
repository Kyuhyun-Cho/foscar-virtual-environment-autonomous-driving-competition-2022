
"use strict";

let PREvent = require('./PREvent.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let WaitForTick = require('./WaitForTick.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let CollisionData = require('./CollisionData.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let IntscnTL = require('./IntscnTL.js');
let GPSMessage = require('./GPSMessage.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let ObjectStatus = require('./ObjectStatus.js');
let RadarDetection = require('./RadarDetection.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let PRStatus = require('./PRStatus.js');
let Lamps = require('./Lamps.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let SensorPosControl = require('./SensorPosControl.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let IntersectionControl = require('./IntersectionControl.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let ERP42Info = require('./ERP42Info.js');
let EventInfo = require('./EventInfo.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let VehicleSpec = require('./VehicleSpec.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let CtrlCmd = require('./CtrlCmd.js');
let TrafficLight = require('./TrafficLight.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let ReplayInfo = require('./ReplayInfo.js');
let RadarDetections = require('./RadarDetections.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let MapSpec = require('./MapSpec.js');
let VehicleCollision = require('./VehicleCollision.js');
let GhostMessage = require('./GhostMessage.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let SaveSensorData = require('./SaveSensorData.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');

module.exports = {
  PREvent: PREvent,
  PRCtrlCmd: PRCtrlCmd,
  WaitForTick: WaitForTick,
  SetTrafficLight: SetTrafficLight,
  MoraiSimProcStatus: MoraiSimProcStatus,
  CollisionData: CollisionData,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  IntscnTL: IntscnTL,
  GPSMessage: GPSMessage,
  VehicleSpecIndex: VehicleSpecIndex,
  ObjectStatus: ObjectStatus,
  RadarDetection: RadarDetection,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  PRStatus: PRStatus,
  Lamps: Lamps,
  SyncModeInfo: SyncModeInfo,
  GetTrafficLightStatus: GetTrafficLightStatus,
  NpcGhostInfo: NpcGhostInfo,
  SensorPosControl: SensorPosControl,
  ObjectStatusList: ObjectStatusList,
  MoraiSrvResponse: MoraiSrvResponse,
  IntersectionControl: IntersectionControl,
  MapSpecIndex: MapSpecIndex,
  ERP42Info: ERP42Info,
  EventInfo: EventInfo,
  MoraiTLInfo: MoraiTLInfo,
  SyncModeCmdResponse: SyncModeCmdResponse,
  MultiEgoSetting: MultiEgoSetting,
  EgoVehicleStatus: EgoVehicleStatus,
  VehicleSpec: VehicleSpec,
  ScenarioLoad: ScenarioLoad,
  CtrlCmd: CtrlCmd,
  TrafficLight: TrafficLight,
  SyncModeAddObject: SyncModeAddObject,
  ReplayInfo: ReplayInfo,
  RadarDetections: RadarDetections,
  MoraiSimProcHandle: MoraiSimProcHandle,
  SyncModeRemoveObject: SyncModeRemoveObject,
  VehicleCollisionData: VehicleCollisionData,
  MoraiTLIndex: MoraiTLIndex,
  MapSpec: MapSpec,
  VehicleCollision: VehicleCollision,
  GhostMessage: GhostMessage,
  SyncModeResultResponse: SyncModeResultResponse,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  SyncModeCmd: SyncModeCmd,
  DdCtrlCmd: DdCtrlCmd,
  IntersectionStatus: IntersectionStatus,
  NpcGhostCmd: NpcGhostCmd,
  SaveSensorData: SaveSensorData,
  WaitForTickResponse: WaitForTickResponse,
  SyncModeSetGear: SyncModeSetGear,
};
