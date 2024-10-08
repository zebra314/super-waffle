cmake_minimum_required(VERSION 2.8.3)
project(pal_navigation_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs sensor_msgs actionlib actionlib_msgs geometry_msgs)

include_directories(include)

add_action_files(
  DIRECTORY action
  FILES
    ExecuteParking.action
    JoyPriority.action
    JoyTurbo.action
    GoToPOI.action
    GoTo.action
    VisualTraining.action)

add_message_files(
  DIRECTORY msg
  FILES
    AvailableMaps.msg
    Emergency.msg
    EulerAngles.msg
    EulerAnglesStamped.msg
    MapConfiguration.msg
    NavigationStatus.msg
    NiceMapTransformation.msg
    POI.msg
    POIGroup.msg
    PolarReading.msg
    PolarReadingScan.msg
    ServiceStatus.msg
    TabletPOI.msg
    VisualLocDB.msg
    Highways.msg
    LaserImage.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    Acknowledgment.srv
    ChangeBuilding.srv
    ChangeMap.srv
    ChangeSyncMap.srv
    DisableEmergency.srv
    FinalApproachPose.srv
    ListMaps.srv
    GetMapConfiguration.srv
    GetNodes.srv
    GetPOI.srv
    GetSubMap.srv
    RegisterSync.srv
    SafetyZone.srv
    SaveMap.srv
    SetMapConfiguration.srv
    SetPOI.srv
    SetSubMapFloor.srv
    VisualLocRecognize.srv
    RenameMap.srv
)

generate_messages(DEPENDENCIES nav_msgs actionlib_msgs sensor_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime nav_msgs actionlib_msgs sensor_msgs geometry_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
