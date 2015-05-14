package com.automodality.message;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class MAVLink {
	
	private static final int PROTOCOL_VERSION = 3;
	private static final short PACKET_START_SIGN = 0xFE; //v1.0: 0xFE; v0.9: 0x55
	private static final int HEADER_LENGTH = 8; // Overhead size
	
	// Mavlink constants
	
	/*
	 * Content Types for data transmission handshake
	 */
	public static final int DATA_TYPE_JPEG_IMAGE = 1; // $value.description
	public static final int DATA_TYPE_RAW_IMAGE = 2; // $value.description
	public static final int DATA_TYPE_KINECT = 3; // $value.description
 
	/*
	 * Disable fenced mode
	 */
	public static final int FENCE_ACTION_NONE = 0; // Disable fenced mode
	public static final int FENCE_ACTION_GUIDED = 1; // Switched to guided mode to return point (fence point 0)
	public static final int FENCE_ACTION_REPORT = 2; // Report fence breach, but don't take action
	public static final int FENCE_ACTION_GUIDED_THR_PASS = 3; // Switched to guided mode to return point (fence point 0) with manual throttle control
 
	/*
	 * No last fence breach
	 */
	public static final int FENCE_BREACH_NONE = 0; // No last fence breach
	public static final int FENCE_BREACH_MINALT = 1; // Breached minimum altitude
	public static final int FENCE_BREACH_MAXALT = 2; // Breached maximum altitude
	public static final int FENCE_BREACH_BOUNDARY = 3; // Breached fence boundary
 
	/*
	 * 
	 */
	public static final int MAVLINK_DATA_STREAM_IMG_JPEG = 0; // 
	public static final int MAVLINK_DATA_STREAM_IMG_BMP = 1; // 
	public static final int MAVLINK_DATA_STREAM_IMG_RAW8U = 2; // 
	public static final int MAVLINK_DATA_STREAM_IMG_RAW32U = 3; // 
	public static final int MAVLINK_DATA_STREAM_IMG_PGM = 4; // 
	public static final int MAVLINK_DATA_STREAM_IMG_PNG = 5; // 
 
	/*
	 * Micro air vehicle / autopilot classes. This identifies the individual model.
	 */
	public static final int MAV_AUTOPILOT_GENERIC = 0; // Generic autopilot, full support for everything
	public static final int MAV_AUTOPILOT_PIXHAWK = 1; // PIXHAWK autopilot, http://pixhawk.ethz.ch
	public static final int MAV_AUTOPILOT_SLUGS = 2; // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	public static final int MAV_AUTOPILOT_ARDUPILOTMEGA = 3; // ArduPilotMega / ArduCopter, http://diydrones.com
	public static final int MAV_AUTOPILOT_OPENPILOT = 4; // OpenPilot, http://openpilot.org
	public static final int MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5; // Generic autopilot only supporting simple waypoints
	public static final int MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6; // Generic autopilot supporting waypoints and other simple navigation commands
	public static final int MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7; // Generic autopilot supporting the full mission command set
	public static final int MAV_AUTOPILOT_INVALID = 8; // No valid autopilot, e.g. a GCS or other MAVLink component
	public static final int MAV_AUTOPILOT_PPZ = 9; // PPZ UAV - http://nongnu.org/paparazzi
	public static final int MAV_AUTOPILOT_UDB = 10; // UAV Dev Board
	public static final int MAV_AUTOPILOT_FP = 11; // FlexiPilot
	public static final int MAV_AUTOPILOT_PX4 = 12; // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
	public static final int MAV_AUTOPILOT_SMACCMPILOT = 13; // SMACCMPilot - http://smaccmpilot.org
	public static final int MAV_AUTOPILOT_AUTOQUAD = 14; // AutoQuad -- http://autoquad.org
	public static final int MAV_AUTOPILOT_ARMAZILA = 15; // Armazila -- http://armazila.com
	public static final int MAV_AUTOPILOT_AEROB = 16; // Aerob -- http://aerob.ru
	public static final int MAV_AUTOPILOT_ASLUAV = 17; // ASLUAV autopilot -- http://www.asl.ethz.ch
 
	/*
	 * Enumeration of battery functions
	 */
	public static final int MAV_BATTERY_FUNCTION_UNKNOWN = 0; // Lithium polymere battery
	public static final int MAV_BATTERY_FUNCTION_ALL = 1; // Battery supports all flight systems
	public static final int MAV_BATTERY_FUNCTION_PROPULSION = 2; // Battery for the propulsion system
	public static final int MAV_BATTERY_FUNCTION_AVIONICS = 3; // Avionics battery
	public static final int MAV_BATTERY_TYPE_PAYLOAD = 4; // Payload battery
 
	/*
	 * Enumeration of battery types
	 */
	public static final int MAV_BATTERY_TYPE_UNKNOWN = 0; // Not specified.
	public static final int MAV_BATTERY_TYPE_LIPO = 1; // Lithium polymere battery
	public static final int MAV_BATTERY_TYPE_LIFE = 2; // Lithium ferrite battery
	public static final int MAV_BATTERY_TYPE_LION = 3; // Lithium-ION battery
	public static final int MAV_BATTERY_TYPE_NIMH = 4; // Nickel metal hydride battery
 
	/*
	 * Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
	 */
	public static final int MAV_CMD_NAV_WAYPOINT = 16; // Navigate to MISSION.
	public static final int MAV_CMD_NAV_LOITER_UNLIM = 17; // Loiter around this MISSION an unlimited amount of time
	public static final int MAV_CMD_NAV_LOITER_TURNS = 18; // Loiter around this MISSION for X turns
	public static final int MAV_CMD_NAV_LOITER_TIME = 19; // Loiter around this MISSION for X seconds
	public static final int MAV_CMD_NAV_RETURN_TO_LAUNCH = 20; // Return to launch location
	public static final int MAV_CMD_NAV_LAND = 21; // Land at location
	public static final int MAV_CMD_NAV_TAKEOFF = 22; // Takeoff from ground / hand
	public static final int MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30; // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
	public static final int MAV_CMD_NAV_ROI = 80; // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	public static final int MAV_CMD_NAV_PATHPLANNING = 81; // Control autonomous path planning on the MAV.
	public static final int MAV_CMD_NAV_SPLINE_WAYPOINT = 82; // Navigate to MISSION using a spline path.
	public static final int MAV_CMD_NAV_GUIDED_ENABLE = 92; // hand control over to an external controller
	public static final int MAV_CMD_NAV_LAST = 95; // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	public static final int MAV_CMD_CONDITION_DELAY = 112; // Delay mission state machine.
	public static final int MAV_CMD_CONDITION_CHANGE_ALT = 113; // Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	public static final int MAV_CMD_CONDITION_DISTANCE = 114; // Delay mission state machine until within desired distance of next NAV point.
	public static final int MAV_CMD_CONDITION_YAW = 115; // Reach a certain target angle.
	public static final int MAV_CMD_CONDITION_LAST = 159; // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	public static final int MAV_CMD_DO_SET_MODE = 176; // Set system mode.
	public static final int MAV_CMD_DO_JUMP = 177; // Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	public static final int MAV_CMD_DO_CHANGE_SPEED = 178; // Change speed and/or throttle set points.
	public static final int MAV_CMD_DO_SET_HOME = 179; // Changes the home location either to the current location or a specified location.
	public static final int MAV_CMD_DO_SET_PARAMETER = 180; // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	public static final int MAV_CMD_DO_SET_RELAY = 181; // Set a relay to a condition.
	public static final int MAV_CMD_DO_REPEAT_RELAY = 182; // Cycle a relay on and off for a desired number of cyles with a desired period.
	public static final int MAV_CMD_DO_SET_SERVO = 183; // Set a servo to a desired PWM value.
	public static final int MAV_CMD_DO_REPEAT_SERVO = 184; // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	public static final int MAV_CMD_DO_FLIGHTTERMINATION = 185; // Terminate flight immediately
	public static final int MAV_CMD_DO_LAND_START = 189; // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence.
	public static final int MAV_CMD_DO_RALLY_LAND = 190; // Mission command to perform a landing from a rally point.
	public static final int MAV_CMD_DO_GO_AROUND = 191; // Mission command to safely abort an autonmous landing.
	public static final int MAV_CMD_DO_CONTROL_VIDEO = 200; // Control onboard camera system.
	public static final int MAV_CMD_DO_SET_ROI = 201; // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	public static final int MAV_CMD_DO_DIGICAM_CONFIGURE = 202; // Mission command to configure an on-board camera controller system.
	public static final int MAV_CMD_DO_DIGICAM_CONTROL = 203; // Mission command to control an on-board camera controller system.
	public static final int MAV_CMD_DO_MOUNT_CONFIGURE = 204; // Mission command to configure a camera or antenna mount
	public static final int MAV_CMD_DO_MOUNT_CONTROL = 205; // Mission command to control a camera or antenna mount
	public static final int MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206; // Mission command to set CAM_TRIGG_DIST for this flight
	public static final int MAV_CMD_DO_FENCE_ENABLE = 207; // Mission command to enable the geofence
	public static final int MAV_CMD_DO_PARACHUTE = 208; // Mission command to trigger a parachute
	public static final int MAV_CMD_DO_INVERTED_FLIGHT = 210; // Change to/from inverted flight
	public static final int MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220; // Mission command to control a camera or antenna mount, using a quaternion as reference.
	public static final int MAV_CMD_DO_GUIDED_MASTER = 221; // set id of master controller
	public static final int MAV_CMD_DO_GUIDED_LIMITS = 222; // set limits for external control
	public static final int MAV_CMD_DO_LAST = 240; // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	public static final int MAV_CMD_PREFLIGHT_CALIBRATION = 241; // Trigger calibration. This command will be only accepted if in pre-flight mode.
	public static final int MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242; // Set sensor offsets. This command will be only accepted if in pre-flight mode.
	public static final int MAV_CMD_PREFLIGHT_STORAGE = 245; // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	public static final int MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246; // Request the reboot or shutdown of system components.
	public static final int MAV_CMD_OVERRIDE_GOTO = 252; // Hold / continue the current action
	public static final int MAV_CMD_MISSION_START = 300; // start running a mission
	public static final int MAV_CMD_COMPONENT_ARM_DISARM = 400; // Arms / Disarms a component
	public static final int MAV_CMD_START_RX_PAIR = 500; // Starts receiver pairing
	public static final int MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520; // Request autopilot capabilities
	public static final int MAV_CMD_IMAGE_START_CAPTURE = 2000; // Start image capture sequence
	public static final int MAV_CMD_IMAGE_STOP_CAPTURE = 2001; // Stop image capture sequence
	public static final int MAV_CMD_DO_TRIGGER_CONTROL = 2003; // Enable or disable on-board camera triggering system.
	public static final int MAV_CMD_VIDEO_START_CAPTURE = 2500; // Starts video capture
	public static final int MAV_CMD_VIDEO_STOP_CAPTURE = 2501; // Stop the current video capture
	public static final int MAV_CMD_PANORAMA_CREATE = 2800; // Create a panorama at the current position
	public static final int MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001; // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	public static final int MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002; // Control the payload deployment.
 
	/*
	 * Starts a search
	 */
	public static final int MAV_CMD_DO_START_SEARCH = 10001; // Starts a search
	public static final int MAV_CMD_DO_FINISH_SEARCH = 10002; // Starts a search
	public static final int MAV_CMD_NAV_SWEEP = 10003; // Starts a search
 
	/*
	 * ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
	 */
	public static final int MAV_CMD_ACK_OK = 0; // Command / mission item is ok.
	public static final int MAV_CMD_ACK_ERR_FAIL = 1; // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	public static final int MAV_CMD_ACK_ERR_ACCESS_DENIED = 2; // The system is refusing to accept this command from this source / communication partner.
	public static final int MAV_CMD_ACK_ERR_NOT_SUPPORTED = 3; // Command or mission item is not supported, other commands would be accepted.
	public static final int MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4; // The coordinate frame of this command / mission item is not supported.
	public static final int MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE = 5; // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	public static final int MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE = 6; // The X or latitude value is out of range.
	public static final int MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE = 7; // The Y or longitude value is out of range.
	public static final int MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE = 8; // The Z or altitude value is out of range.
 
	/*
	 * 
	 */
	public static final int MAV_COMP_ID_ALL = 0; // 
	public static final int MAV_COMP_ID_GPS = 220; // 
	public static final int MAV_COMP_ID_MISSIONPLANNER = 190; // 
	public static final int MAV_COMP_ID_PATHPLANNER = 195; // 
	public static final int MAV_COMP_ID_MAPPER = 180; // 
	public static final int MAV_COMP_ID_CAMERA = 100; // 
	public static final int MAV_COMP_ID_IMU = 200; // 
	public static final int MAV_COMP_ID_IMU_2 = 201; // 
	public static final int MAV_COMP_ID_IMU_3 = 202; // 
	public static final int MAV_COMP_ID_UDP_BRIDGE = 240; // 
	public static final int MAV_COMP_ID_UART_BRIDGE = 241; // 
	public static final int MAV_COMP_ID_SYSTEM_CONTROL = 250; // 
	public static final int MAV_COMP_ID_SERVO1 = 140; // 
	public static final int MAV_COMP_ID_SERVO2 = 141; // 
	public static final int MAV_COMP_ID_SERVO3 = 142; // 
	public static final int MAV_COMP_ID_SERVO4 = 143; // 
	public static final int MAV_COMP_ID_SERVO5 = 144; // 
	public static final int MAV_COMP_ID_SERVO6 = 145; // 
	public static final int MAV_COMP_ID_SERVO7 = 146; // 
	public static final int MAV_COMP_ID_SERVO8 = 147; // 
	public static final int MAV_COMP_ID_SERVO9 = 148; // 
	public static final int MAV_COMP_ID_SERVO10 = 149; // 
	public static final int MAV_COMP_ID_SERVO11 = 150; // 
	public static final int MAV_COMP_ID_SERVO12 = 151; // 
	public static final int MAV_COMP_ID_SERVO13 = 152; // 
	public static final int MAV_COMP_ID_SERVO14 = 153; // 
	public static final int MAV_COMP_ID_GIMBAL = 154; // 
 
	/*
	 * Data stream IDs. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages.
	 */
	public static final int MAV_DATA_STREAM_ALL = 0; // Enable all data streams
	public static final int MAV_DATA_STREAM_RAW_SENSORS = 1; // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	public static final int MAV_DATA_STREAM_EXTENDED_STATUS = 2; // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	public static final int MAV_DATA_STREAM_RC_CHANNELS = 3; // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	public static final int MAV_DATA_STREAM_RAW_CONTROLLER = 4; // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	public static final int MAV_DATA_STREAM_POSITION = 6; // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	public static final int MAV_DATA_STREAM_EXTRA1 = 10; // Dependent on the autopilot
	public static final int MAV_DATA_STREAM_EXTRA2 = 11; // Dependent on the autopilot
	public static final int MAV_DATA_STREAM_EXTRA3 = 12; // Dependent on the autopilot
 
	/*
	 * Enumeration of distance sensor types
	 */
	public static final int MAV_DISTANCE_SENSOR_LASER = 0; // Laser altimeter, e.g. LightWare SF02/F or PulsedLight units
	public static final int MAV_DISTANCE_SENSOR_ULTRASOUND = 1; // Ultrasound altimeter, e.g. MaxBotix units
 
	/*
	 * Enumeration of estimator types
	 */
	public static final int MAV_ESTIMATOR_TYPE_NAIVE = 1; // This is a naive estimator without any real covariance feedback.
	public static final int MAV_ESTIMATOR_TYPE_VISION = 2; // Computer vision based estimate. Might be up to scale.
	public static final int MAV_ESTIMATOR_TYPE_VIO = 3; // Visual-inertial estimate.
	public static final int MAV_ESTIMATOR_TYPE_GPS = 4; // Plain GPS estimate.
	public static final int MAV_ESTIMATOR_TYPE_GPS_INS = 5; // Estimator integrating GPS and inertial sensing.
 
	/*
	 * Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	 */
	public static final int MAV_FRAME_GLOBAL = 0; // Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	public static final int MAV_FRAME_LOCAL_NED = 1; // Local coordinate frame, Z-up (x: north, y: east, z: down).
	public static final int MAV_FRAME_MISSION = 2; // NOT a coordinate frame, indicates a mission command.
	public static final int MAV_FRAME_GLOBAL_RELATIVE_ALT = 3; // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	public static final int MAV_FRAME_LOCAL_ENU = 4; // Local coordinate frame, Z-down (x: east, y: north, z: up)
	public static final int MAV_FRAME_GLOBAL_INT = 5; // Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
	public static final int MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6; // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
	public static final int MAV_FRAME_LOCAL_OFFSET_NED = 7; // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
	public static final int MAV_FRAME_BODY_NED = 8; // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
	public static final int MAV_FRAME_BODY_OFFSET_NED = 9; // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
	public static final int MAV_FRAME_GLOBAL_TERRAIN_ALT = 10; // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
	public static final int MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11; // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
 
	/*
	 * Override command, pauses current mission execution and moves immediately to a position
	 */
	public static final int MAV_GOTO_DO_HOLD = 0; // Hold at the current position.
	public static final int MAV_GOTO_DO_CONTINUE = 1; // Continue with the next item in mission execution.
	public static final int MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2; // Hold at the current position of the system
	public static final int MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3; // Hold at the position specified in the parameters of the DO_HOLD action
 
	/*
	 * result in a mavlink mission ack
	 */
	public static final int MAV_MISSION_ACCEPTED = 0; // mission accepted OK
	public static final int MAV_MISSION_ERROR = 1; // generic error / not accepting mission commands at all right now
	public static final int MAV_MISSION_UNSUPPORTED_FRAME = 2; // coordinate frame is not supported
	public static final int MAV_MISSION_UNSUPPORTED = 3; // command is not supported
	public static final int MAV_MISSION_NO_SPACE = 4; // mission item exceeds storage space
	public static final int MAV_MISSION_INVALID = 5; // one of the parameters has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM1 = 6; // param1 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM2 = 7; // param2 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM3 = 8; // param3 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM4 = 9; // param4 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM5_X = 10; // x/param5 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM6_Y = 11; // y/param6 has an invalid value
	public static final int MAV_MISSION_INVALID_PARAM7 = 12; // param7 has an invalid value
	public static final int MAV_MISSION_INVALID_SEQUENCE = 13; // received waypoint out of sequence
	public static final int MAV_MISSION_DENIED = 14; // not accepting any mission commands from this communication partner
 
	/*
	 * These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
	 */
	public static final int MAV_MODE_PREFLIGHT = 0; // System is not ready to fly, booting, calibrating, etc. No flag is set.
	public static final int MAV_MODE_STABILIZE_DISARMED = 80; // System is allowed to be active, under assisted RC control.
	public static final int MAV_MODE_STABILIZE_ARMED = 208; // System is allowed to be active, under assisted RC control.
	public static final int MAV_MODE_MANUAL_DISARMED = 64; // System is allowed to be active, under manual (RC) control, no stabilization
	public static final int MAV_MODE_MANUAL_ARMED = 192; // System is allowed to be active, under manual (RC) control, no stabilization
	public static final int MAV_MODE_GUIDED_DISARMED = 88; // System is allowed to be active, under autonomous control, manual setpoint
	public static final int MAV_MODE_GUIDED_ARMED = 216; // System is allowed to be active, under autonomous control, manual setpoint
	public static final int MAV_MODE_AUTO_DISARMED = 92; // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	public static final int MAV_MODE_AUTO_ARMED = 220; // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	public static final int MAV_MODE_TEST_DISARMED = 66; // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	public static final int MAV_MODE_TEST_ARMED = 194; // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
 
	/*
	 * These flags encode the MAV mode.
	 */
	public static final int MAV_MODE_FLAG_SAFETY_ARMED = 128; // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	public static final int MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64; // 0b01000000 remote control input is enabled.
	public static final int MAV_MODE_FLAG_HIL_ENABLED = 32; // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	public static final int MAV_MODE_FLAG_STABILIZE_ENABLED = 16; // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	public static final int MAV_MODE_FLAG_GUIDED_ENABLED = 8; // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	public static final int MAV_MODE_FLAG_AUTO_ENABLED = 4; // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	public static final int MAV_MODE_FLAG_TEST_ENABLED = 2; // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	public static final int MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1; // 0b00000001 Reserved for future use.
 
	/*
	 * These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
	 */
	public static final int MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128; // First bit:  10000000
	public static final int MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64; // Second bit: 01000000
	public static final int MAV_MODE_FLAG_DECODE_POSITION_HIL = 32; // Third bit:  00100000
	public static final int MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16; // Fourth bit: 00010000
	public static final int MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8; // Fifth bit:  00001000
	public static final int MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4; // Sixt bit:   00000100
	public static final int MAV_MODE_FLAG_DECODE_POSITION_TEST = 2; // Seventh bit: 00000010
	public static final int MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1; // Eighth bit: 00000001
 
	/*
	 * Enumeration of possible mount operation modes
	 */
	public static final int MAV_MOUNT_MODE_RETRACT = 0; // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	public static final int MAV_MOUNT_MODE_NEUTRAL = 1; // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	public static final int MAV_MOUNT_MODE_MAVLINK_TARGETING = 2; // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	public static final int MAV_MOUNT_MODE_RC_TARGETING = 3; // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	public static final int MAV_MOUNT_MODE_GPS_POINT = 4; // Load neutral position and start to point to Lat,Lon,Alt
 
	/*
	 * Specifies the datatype of a MAVLink parameter.
	 */
	public static final int MAV_PARAM_TYPE_UINT8 = 1; // 8-bit unsigned integer
	public static final int MAV_PARAM_TYPE_INT8 = 2; // 8-bit signed integer
	public static final int MAV_PARAM_TYPE_UINT16 = 3; // 16-bit unsigned integer
	public static final int MAV_PARAM_TYPE_INT16 = 4; // 16-bit signed integer
	public static final int MAV_PARAM_TYPE_UINT32 = 5; // 32-bit unsigned integer
	public static final int MAV_PARAM_TYPE_INT32 = 6; // 32-bit signed integer
	public static final int MAV_PARAM_TYPE_UINT64 = 7; // 64-bit unsigned integer
	public static final int MAV_PARAM_TYPE_INT64 = 8; // 64-bit signed integer
	public static final int MAV_PARAM_TYPE_REAL32 = 9; // 32-bit floating-point
	public static final int MAV_PARAM_TYPE_REAL64 = 10; // 64-bit floating-point
 
	/*
	 * Power supply status flags (bitmask)
	 */
	public static final int MAV_POWER_STATUS_BRICK_VALID = 1; // main brick power supply valid
	public static final int MAV_POWER_STATUS_SERVO_VALID = 2; // main servo power supply valid for FMU
	public static final int MAV_POWER_STATUS_USB_CONNECTED = 4; // USB power is connected
	public static final int MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8; // peripheral supply is in over-current state
	public static final int MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16; // hi-power peripheral supply is in over-current state
	public static final int MAV_POWER_STATUS_CHANGED = 32; // Power status has changed since boot
 
	/*
	 * Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
	 */
	public static final int MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1; // Autopilot supports MISSION float message type.
	public static final int MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2; // Autopilot supports the new param float message type.
	public static final int MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4; // Autopilot supports MISSION_INT scaled integer message type.
	public static final int MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8; // Autopilot supports COMMAND_INT scaled integer message type.
	public static final int MAV_PROTOCOL_CAPABILITY_PARAM_UNION = 16; // Autopilot supports the new param union message type.
	public static final int MAV_PROTOCOL_CAPABILITY_FTP = 32; // Autopilot supports the new param union message type.
	public static final int MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64; // Autopilot supports commanding attitude offboard.
	public static final int MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128; // Autopilot supports commanding position and velocity targets in local NED frame.
	public static final int MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256; // Autopilot supports commanding position and velocity targets in global scaled integers.
	public static final int MAV_PROTOCOL_CAPABILITY_TERRAIN = 512; // Autopilot supports terrain protocol / data handling.
	public static final int MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET = 1024; // Autopilot supports direct actuator control.
 
	/*
	 * result from a mavlink command
	 */
	public static final int MAV_RESULT_ACCEPTED = 0; // Command ACCEPTED and EXECUTED
	public static final int MAV_RESULT_TEMPORARILY_REJECTED = 1; // Command TEMPORARY REJECTED/DENIED
	public static final int MAV_RESULT_DENIED = 2; // Command PERMANENTLY DENIED
	public static final int MAV_RESULT_UNSUPPORTED = 3; // Command UNKNOWN/UNSUPPORTED
	public static final int MAV_RESULT_FAILED = 4; // Command executed, but failed
 
	/*
	 *  The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI).
	 */
	public static final int MAV_ROI_NONE = 0; // No region of interest.
	public static final int MAV_ROI_WPNEXT = 1; // Point toward next MISSION.
	public static final int MAV_ROI_WPINDEX = 2; // Point toward given MISSION.
	public static final int MAV_ROI_LOCATION = 3; // Point toward fixed location.
	public static final int MAV_ROI_TARGET = 4; // Point toward of given id.
 
	/*
	 * Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
	 */
	public static final int MAV_SEVERITY_EMERGENCY = 0; // System is unusable. This is a "panic" condition.
	public static final int MAV_SEVERITY_ALERT = 1; // Action should be taken immediately. Indicates error in non-critical systems.
	public static final int MAV_SEVERITY_CRITICAL = 2; // Action must be taken immediately. Indicates failure in a primary system.
	public static final int MAV_SEVERITY_ERROR = 3; // Indicates an error in secondary/redundant systems.
	public static final int MAV_SEVERITY_WARNING = 4; // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	public static final int MAV_SEVERITY_NOTICE = 5; // An unusual event has occured, though not an error condition. This should be investigated for the root cause.
	public static final int MAV_SEVERITY_INFO = 6; // Normal operational messages. Useful for logging. No action is required for these messages.
	public static final int MAV_SEVERITY_DEBUG = 7; // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
 
	/*
	 * Uninitialized system, state is unknown.
	 */
	public static final int MAV_STATE_UNINIT = 0; // Uninitialized system, state is unknown.
	public static final int MAV_STATE_BOOT = 1; // System is booting up.
	public static final int MAV_STATE_CALIBRATING = 2; // System is calibrating and not flight-ready.
	public static final int MAV_STATE_STANDBY = 3; // System is grounded and on standby. It can be launched any time.
	public static final int MAV_STATE_ACTIVE = 4; // System is active and might be already airborne. Motors are engaged.
	public static final int MAV_STATE_CRITICAL = 5; // System is in a non-normal flight mode. It can however still navigate.
	public static final int MAV_STATE_EMERGENCY = 6; // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	public static final int MAV_STATE_POWEROFF = 7; // System just initialized its power-down sequence, will shut down now.
 
	/*
	 * These encode the sensors whose status is sent as part of the SYS_STATUS message.
	 */
	public static final int MAV_SYS_STATUS_SENSOR_3D_GYRO = 1; // 0x01 3D gyro
	public static final int MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2; // 0x02 3D accelerometer
	public static final int MAV_SYS_STATUS_SENSOR_3D_MAG = 4; // 0x04 3D magnetometer
	public static final int MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8; // 0x08 absolute pressure
	public static final int MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16; // 0x10 differential pressure
	public static final int MAV_SYS_STATUS_SENSOR_GPS = 32; // 0x20 GPS
	public static final int MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64; // 0x40 optical flow
	public static final int MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128; // 0x80 computer vision position
	public static final int MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256; // 0x100 laser based position
	public static final int MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512; // 0x200 external ground truth (Vicon or Leica)
	public static final int MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024; // 0x400 3D angular rate control
	public static final int MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048; // 0x800 attitude stabilization
	public static final int MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096; // 0x1000 yaw position
	public static final int MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192; // 0x2000 z/altitude control
	public static final int MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384; // 0x4000 x/y position control
	public static final int MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768; // 0x8000 motor outputs / control
	public static final int MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536; // 0x10000 rc receiver
	public static final int MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072; // 0x20000 2nd 3D gyro
	public static final int MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144; // 0x40000 2nd 3D accelerometer
	public static final int MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288; // 0x80000 2nd 3D magnetometer
	public static final int MAV_SYS_STATUS_GEOFENCE = 1048576; // 0x100000 geofence
	public static final int MAV_SYS_STATUS_AHRS = 2097152; // 0x200000 AHRS subsystem health
	public static final int MAV_SYS_STATUS_TERRAIN = 4194304; // 0x400000 Terrain subsystem health
 
	/*
	 * Generic micro air vehicle.
	 */
	public static final int MAV_TYPE_GENERIC = 0; // Generic micro air vehicle.
	public static final int MAV_TYPE_FIXED_WING = 1; // Fixed wing aircraft.
	public static final int MAV_TYPE_QUADROTOR = 2; // Quadrotor
	public static final int MAV_TYPE_COAXIAL = 3; // Coaxial helicopter
	public static final int MAV_TYPE_HELICOPTER = 4; // Normal helicopter with tail rotor.
	public static final int MAV_TYPE_ANTENNA_TRACKER = 5; // Ground installation
	public static final int MAV_TYPE_GCS = 6; // Operator control unit / ground control station
	public static final int MAV_TYPE_AIRSHIP = 7; // Airship, controlled
	public static final int MAV_TYPE_FREE_BALLOON = 8; // Free balloon, uncontrolled
	public static final int MAV_TYPE_ROCKET = 9; // Rocket
	public static final int MAV_TYPE_GROUND_ROVER = 10; // Ground rover
	public static final int MAV_TYPE_SURFACE_BOAT = 11; // Surface vessel, boat, ship
	public static final int MAV_TYPE_SUBMARINE = 12; // Submarine
	public static final int MAV_TYPE_HEXAROTOR = 13; // Hexarotor
	public static final int MAV_TYPE_OCTOROTOR = 14; // Octorotor
	public static final int MAV_TYPE_TRICOPTER = 15; // Octorotor
	public static final int MAV_TYPE_FLAPPING_WING = 16; // Flapping wing
	public static final int MAV_TYPE_KITE = 17; // Flapping wing
	public static final int MAV_TYPE_ONBOARD_CONTROLLER = 18; // Onboard companion controller
	public static final int MAV_TYPE_VTOL_DUOROTOR = 19; // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	public static final int MAV_TYPE_VTOL_QUADROTOR = 20; // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
	public static final int MAV_TYPE_VTOL_RESERVED1 = 21; // VTOL reserved 1
	public static final int MAV_TYPE_VTOL_RESERVED2 = 22; // VTOL reserved 2
	public static final int MAV_TYPE_VTOL_RESERVED3 = 23; // VTOL reserved 3
	public static final int MAV_TYPE_VTOL_RESERVED4 = 24; // VTOL reserved 4
	public static final int MAV_TYPE_VTOL_RESERVED5 = 25; // VTOL reserved 5
	public static final int MAV_TYPE_GIMBAL = 26; // Onboard gimbal
 
	/*
	 * SERIAL_CONTROL device types
	 */
	public static final int SERIAL_CONTROL_DEV_TELEM1 = 0; // First telemetry port
	public static final int SERIAL_CONTROL_DEV_TELEM2 = 1; // Second telemetry port
	public static final int SERIAL_CONTROL_DEV_GPS1 = 2; // First GPS port
	public static final int SERIAL_CONTROL_DEV_GPS2 = 3; // Second GPS port
 
	/*
	 * SERIAL_CONTROL flags (bitmask)
	 */
	public static final int SERIAL_CONTROL_FLAG_REPLY = 1; // Set if this is a reply
	public static final int SERIAL_CONTROL_FLAG_RESPOND = 2; // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	public static final int SERIAL_CONTROL_FLAG_EXCLUSIVE = 4; // Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	public static final int SERIAL_CONTROL_FLAG_BLOCKING = 8; // Block on writes to the serial port
	public static final int SERIAL_CONTROL_FLAG_MULTI = 16; // Send multiple replies until port is drained
 
	/*
	 * Message IDs
	 */
	public static final short MSG_ID_ACTUATOR_CONTROL_TARGET = 140;
	public static final short MSG_ID_ATTITUDE = 30;
	public static final short MSG_ID_ATTITUDE_CONTROL = 200;
	public static final short MSG_ID_ATTITUDE_QUATERNION = 31;
	public static final short MSG_ID_ATTITUDE_QUATERNION_COV = 61;
	public static final short MSG_ID_ATTITUDE_TARGET = 83;
	public static final short MSG_ID_ATT_POS_MOCAP = 138;
	public static final short MSG_ID_AUTH_KEY = 7;
	public static final short MSG_ID_AUTOPILOT_VERSION = 148;
	public static final short MSG_ID_BATTERY_STATUS = 147;
	public static final short MSG_ID_BRIEF_FEATURE = 195;
	public static final short MSG_ID_CAMERA_TRIGGER = 112;
	public static final short MSG_ID_CHANGE_OPERATOR_CONTROL = 5;
	public static final short MSG_ID_CHANGE_OPERATOR_CONTROL_ACK = 6;
	public static final short MSG_ID_COMMAND_ACK = 77;
	public static final short MSG_ID_COMMAND_INT = 75;
	public static final short MSG_ID_COMMAND_LONG = 76;
	public static final short MSG_ID_DATA_STREAM = 67;
	public static final short MSG_ID_DATA_TRANSMISSION_HANDSHAKE = 130;
	public static final short MSG_ID_DEBUG = 254;
	public static final short MSG_ID_DEBUG_VECT = 250;
	public static final short MSG_ID_DETECTION_STATS = 205;
	public static final short MSG_ID_DISTANCE_SENSOR = 132;
	public static final short MSG_ID_ENCAPSULATED_DATA = 131;
	public static final short MSG_ID_FILE_TRANSFER_PROTOCOL = 110;
	public static final short MSG_ID_GLOBAL_POSITION_INT = 33;
	public static final short MSG_ID_GLOBAL_POSITION_INT_COV = 63;
	public static final short MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE = 101;
	public static final short MSG_ID_GPS2_RAW = 124;
	public static final short MSG_ID_GPS2_RTK = 128;
	public static final short MSG_ID_GPS_GLOBAL_ORIGIN = 49;
	public static final short MSG_ID_GPS_INJECT_DATA = 123;
	public static final short MSG_ID_GPS_RAW_INT = 24;
	public static final short MSG_ID_GPS_RTK = 127;
	public static final short MSG_ID_GPS_STATUS = 25;
	public static final short MSG_ID_HEARTBEAT = 0;
	public static final short MSG_ID_HIGHRES_IMU = 105;
	public static final short MSG_ID_HIL_CONTROLS = 91;
	public static final short MSG_ID_HIL_GPS = 113;
	public static final short MSG_ID_HIL_OPTICAL_FLOW = 114;
	public static final short MSG_ID_HIL_RC_INPUTS_RAW = 92;
	public static final short MSG_ID_HIL_SENSOR = 107;
	public static final short MSG_ID_HIL_STATE = 90;
	public static final short MSG_ID_HIL_STATE_QUATERNION = 115;
	public static final short MSG_ID_IMAGE_AVAILABLE = 154;
	public static final short MSG_ID_IMAGE_TRIGGERED = 152;
	public static final short MSG_ID_IMAGE_TRIGGER_CONTROL = 153;
	public static final short MSG_ID_LOCAL_POSITION_NED = 32;
	public static final short MSG_ID_LOCAL_POSITION_NED_COV = 64;
	public static final short MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89;
	public static final short MSG_ID_LOG_DATA = 120;
	public static final short MSG_ID_LOG_ENTRY = 118;
	public static final short MSG_ID_LOG_ERASE = 121;
	public static final short MSG_ID_LOG_REQUEST_DATA = 119;
	public static final short MSG_ID_LOG_REQUEST_END = 122;
	public static final short MSG_ID_LOG_REQUEST_LIST = 117;
	public static final short MSG_ID_MANUAL_CONTROL = 69;
	public static final short MSG_ID_MANUAL_SETPOINT = 81;
	public static final short MSG_ID_MARKER = 171;
	public static final short MSG_ID_MEMORY_VECT = 249;
	public static final short MSG_ID_MISSION_ACK = 47;
	public static final short MSG_ID_MISSION_CLEAR_ALL = 45;
	public static final short MSG_ID_MISSION_COUNT = 44;
	public static final short MSG_ID_MISSION_CURRENT = 42;
	public static final short MSG_ID_MISSION_ITEM = 39;
	public static final short MSG_ID_MISSION_ITEM_INT = 73;
	public static final short MSG_ID_MISSION_ITEM_REACHED = 46;
	public static final short MSG_ID_MISSION_REQUEST = 40;
	public static final short MSG_ID_MISSION_REQUEST_LIST = 43;
	public static final short MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37;
	public static final short MSG_ID_MISSION_SET_CURRENT = 41;
	public static final short MSG_ID_MISSION_WRITE_PARTIAL_LIST = 38;
	public static final short MSG_ID_NAMED_VALUE_FLOAT = 251;
	public static final short MSG_ID_NAMED_VALUE_INT = 252;
	public static final short MSG_ID_NAV_CONTROLLER_OUTPUT = 62;
	public static final short MSG_ID_ONBOARD_HEALTH = 206;
	public static final short MSG_ID_OPTICAL_FLOW = 100;
	public static final short MSG_ID_OPTICAL_FLOW_RAD = 106;
	public static final short MSG_ID_PARAM_MAP_RC = 50;
	public static final short MSG_ID_PARAM_REQUEST_LIST = 21;
	public static final short MSG_ID_PARAM_REQUEST_READ = 20;
	public static final short MSG_ID_PARAM_SET = 23;
	public static final short MSG_ID_PARAM_VALUE = 22;
	public static final short MSG_ID_PATTERN_DETECTED = 190;
	public static final short MSG_ID_PING = 4;
	public static final short MSG_ID_POINT_OF_INTEREST = 191;
	public static final short MSG_ID_POINT_OF_INTEREST_CONNECTION = 192;
	public static final short MSG_ID_POSITION_CONTROL_SETPOINT = 170;
	public static final short MSG_ID_POSITION_TARGET_GLOBAL_INT = 87;
	public static final short MSG_ID_POSITION_TARGET_LOCAL_NED = 85;
	public static final short MSG_ID_POWER_STATUS = 125;
	public static final short MSG_ID_RADIO_STATUS = 109;
	public static final short MSG_ID_RAW_AUX = 172;
	public static final short MSG_ID_RAW_IMU = 27;
	public static final short MSG_ID_RAW_PRESSURE = 28;
	public static final short MSG_ID_RC_CHANNELS = 65;
	public static final short MSG_ID_RC_CHANNELS_OVERRIDE = 70;
	public static final short MSG_ID_RC_CHANNELS_RAW = 35;
	public static final short MSG_ID_RC_CHANNELS_SCALED = 34;
	public static final short MSG_ID_REQUEST_DATA_STREAM = 66;
	public static final short MSG_ID_SAFETY_ALLOWED_AREA = 55;
	public static final short MSG_ID_SAFETY_SET_ALLOWED_AREA = 54;
	public static final short MSG_ID_SCALED_IMU = 26;
	public static final short MSG_ID_SCALED_IMU2 = 116;
	public static final short MSG_ID_SCALED_IMU3 = 129;
	public static final short MSG_ID_SCALED_PRESSURE = 29;
	public static final short MSG_ID_SCALED_PRESSURE2 = 137;
	public static final short MSG_ID_SERIAL_CONTROL = 126;
	public static final short MSG_ID_SERVO_OUTPUT_RAW = 36;
	public static final short MSG_ID_SET_ACTUATOR_CONTROL_TARGET = 139;
	public static final short MSG_ID_SET_ATTITUDE_TARGET = 82;
	public static final short MSG_ID_SET_CAM_SHUTTER = 151;
	public static final short MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48;
	public static final short MSG_ID_SET_MODE = 11;
	public static final short MSG_ID_SET_POSITION_CONTROL_OFFSET = 160;
	public static final short MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86;
	public static final short MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84;
	public static final short MSG_ID_SIM_STATE = 108;
	public static final short MSG_ID_STATUSTEXT = 253;
	public static final short MSG_ID_SYSTEM_TIME = 2;
	public static final short MSG_ID_SYS_STATUS = 1;
	public static final short MSG_ID_TERRAIN_CHECK = 135;
	public static final short MSG_ID_TERRAIN_DATA = 134;
	public static final short MSG_ID_TERRAIN_REPORT = 136;
	public static final short MSG_ID_TERRAIN_REQUEST = 133;
	public static final short MSG_ID_TIMESYNC = 111;
	public static final short MSG_ID_V2_EXTENSION = 248;
	public static final short MSG_ID_VFR_HUD = 74;
	public static final short MSG_ID_VICON_POSITION_ESTIMATE = 104;
	public static final short MSG_ID_VISION_POSITION_ESTIMATE = 102;
	public static final short MSG_ID_VISION_SPEED_ESTIMATE = 103;
	public static final short MSG_ID_WATCHDOG_COMMAND = 183;
	public static final short MSG_ID_WATCHDOG_HEARTBEAT = 180;
	public static final short MSG_ID_WATCHDOG_PROCESS_INFO = 181;
	public static final short MSG_ID_WATCHDOG_PROCESS_STATUS = 182;

	public static abstract class Message extends AMMAVLink {
		
		protected String msgName;
		
		public static Message decodeMessage(byte[] bytes) {
			short msg = (short)(bytes[5] & 0xFF);
		
			switch(msg) {
					case MSG_ID_ACTUATOR_CONTROL_TARGET: 
						return new MSG_ACTUATOR_CONTROL_TARGET ().decode(bytes);
					case MSG_ID_ATTITUDE: 
						return new MSG_ATTITUDE ().decode(bytes);
					case MSG_ID_ATTITUDE_CONTROL: 
						return new MSG_ATTITUDE_CONTROL ().decode(bytes);
					case MSG_ID_ATTITUDE_QUATERNION: 
						return new MSG_ATTITUDE_QUATERNION ().decode(bytes);
					case MSG_ID_ATTITUDE_QUATERNION_COV: 
						return new MSG_ATTITUDE_QUATERNION_COV ().decode(bytes);
					case MSG_ID_ATTITUDE_TARGET: 
						return new MSG_ATTITUDE_TARGET ().decode(bytes);
					case MSG_ID_ATT_POS_MOCAP: 
						return new MSG_ATT_POS_MOCAP ().decode(bytes);
					case MSG_ID_AUTH_KEY: 
						return new MSG_AUTH_KEY ().decode(bytes);
					case MSG_ID_AUTOPILOT_VERSION: 
						return new MSG_AUTOPILOT_VERSION ().decode(bytes);
					case MSG_ID_BATTERY_STATUS: 
						return new MSG_BATTERY_STATUS ().decode(bytes);
					case MSG_ID_BRIEF_FEATURE: 
						return new MSG_BRIEF_FEATURE ().decode(bytes);
					case MSG_ID_CAMERA_TRIGGER: 
						return new MSG_CAMERA_TRIGGER ().decode(bytes);
					case MSG_ID_CHANGE_OPERATOR_CONTROL: 
						return new MSG_CHANGE_OPERATOR_CONTROL ().decode(bytes);
					case MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: 
						return new MSG_CHANGE_OPERATOR_CONTROL_ACK ().decode(bytes);
					case MSG_ID_COMMAND_ACK: 
						return new MSG_COMMAND_ACK ().decode(bytes);
					case MSG_ID_COMMAND_INT: 
						return new MSG_COMMAND_INT ().decode(bytes);
					case MSG_ID_COMMAND_LONG: 
						return new MSG_COMMAND_LONG ().decode(bytes);
					case MSG_ID_DATA_STREAM: 
						return new MSG_DATA_STREAM ().decode(bytes);
					case MSG_ID_DATA_TRANSMISSION_HANDSHAKE: 
						return new MSG_DATA_TRANSMISSION_HANDSHAKE ().decode(bytes);
					case MSG_ID_DEBUG: 
						return new MSG_DEBUG ().decode(bytes);
					case MSG_ID_DEBUG_VECT: 
						return new MSG_DEBUG_VECT ().decode(bytes);
					case MSG_ID_DETECTION_STATS: 
						return new MSG_DETECTION_STATS ().decode(bytes);
					case MSG_ID_DISTANCE_SENSOR: 
						return new MSG_DISTANCE_SENSOR ().decode(bytes);
					case MSG_ID_ENCAPSULATED_DATA: 
						return new MSG_ENCAPSULATED_DATA ().decode(bytes);
					case MSG_ID_FILE_TRANSFER_PROTOCOL: 
						return new MSG_FILE_TRANSFER_PROTOCOL ().decode(bytes);
					case MSG_ID_GLOBAL_POSITION_INT: 
						return new MSG_GLOBAL_POSITION_INT ().decode(bytes);
					case MSG_ID_GLOBAL_POSITION_INT_COV: 
						return new MSG_GLOBAL_POSITION_INT_COV ().decode(bytes);
					case MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: 
						return new MSG_GLOBAL_VISION_POSITION_ESTIMATE ().decode(bytes);
					case MSG_ID_GPS2_RAW: 
						return new MSG_GPS2_RAW ().decode(bytes);
					case MSG_ID_GPS2_RTK: 
						return new MSG_GPS2_RTK ().decode(bytes);
					case MSG_ID_GPS_GLOBAL_ORIGIN: 
						return new MSG_GPS_GLOBAL_ORIGIN ().decode(bytes);
					case MSG_ID_GPS_INJECT_DATA: 
						return new MSG_GPS_INJECT_DATA ().decode(bytes);
					case MSG_ID_GPS_RAW_INT: 
						return new MSG_GPS_RAW_INT ().decode(bytes);
					case MSG_ID_GPS_RTK: 
						return new MSG_GPS_RTK ().decode(bytes);
					case MSG_ID_GPS_STATUS: 
						return new MSG_GPS_STATUS ().decode(bytes);
					case MSG_ID_HEARTBEAT: 
						return new MSG_HEARTBEAT ().decode(bytes);
					case MSG_ID_HIGHRES_IMU: 
						return new MSG_HIGHRES_IMU ().decode(bytes);
					case MSG_ID_HIL_CONTROLS: 
						return new MSG_HIL_CONTROLS ().decode(bytes);
					case MSG_ID_HIL_GPS: 
						return new MSG_HIL_GPS ().decode(bytes);
					case MSG_ID_HIL_OPTICAL_FLOW: 
						return new MSG_HIL_OPTICAL_FLOW ().decode(bytes);
					case MSG_ID_HIL_RC_INPUTS_RAW: 
						return new MSG_HIL_RC_INPUTS_RAW ().decode(bytes);
					case MSG_ID_HIL_SENSOR: 
						return new MSG_HIL_SENSOR ().decode(bytes);
					case MSG_ID_HIL_STATE: 
						return new MSG_HIL_STATE ().decode(bytes);
					case MSG_ID_HIL_STATE_QUATERNION: 
						return new MSG_HIL_STATE_QUATERNION ().decode(bytes);
					case MSG_ID_IMAGE_AVAILABLE: 
						return new MSG_IMAGE_AVAILABLE ().decode(bytes);
					case MSG_ID_IMAGE_TRIGGERED: 
						return new MSG_IMAGE_TRIGGERED ().decode(bytes);
					case MSG_ID_IMAGE_TRIGGER_CONTROL: 
						return new MSG_IMAGE_TRIGGER_CONTROL ().decode(bytes);
					case MSG_ID_LOCAL_POSITION_NED: 
						return new MSG_LOCAL_POSITION_NED ().decode(bytes);
					case MSG_ID_LOCAL_POSITION_NED_COV: 
						return new MSG_LOCAL_POSITION_NED_COV ().decode(bytes);
					case MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: 
						return new MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET ().decode(bytes);
					case MSG_ID_LOG_DATA: 
						return new MSG_LOG_DATA ().decode(bytes);
					case MSG_ID_LOG_ENTRY: 
						return new MSG_LOG_ENTRY ().decode(bytes);
					case MSG_ID_LOG_ERASE: 
						return new MSG_LOG_ERASE ().decode(bytes);
					case MSG_ID_LOG_REQUEST_DATA: 
						return new MSG_LOG_REQUEST_DATA ().decode(bytes);
					case MSG_ID_LOG_REQUEST_END: 
						return new MSG_LOG_REQUEST_END ().decode(bytes);
					case MSG_ID_LOG_REQUEST_LIST: 
						return new MSG_LOG_REQUEST_LIST ().decode(bytes);
					case MSG_ID_MANUAL_CONTROL: 
						return new MSG_MANUAL_CONTROL ().decode(bytes);
					case MSG_ID_MANUAL_SETPOINT: 
						return new MSG_MANUAL_SETPOINT ().decode(bytes);
					case MSG_ID_MARKER: 
						return new MSG_MARKER ().decode(bytes);
					case MSG_ID_MEMORY_VECT: 
						return new MSG_MEMORY_VECT ().decode(bytes);
					case MSG_ID_MISSION_ACK: 
						return new MSG_MISSION_ACK ().decode(bytes);
					case MSG_ID_MISSION_CLEAR_ALL: 
						return new MSG_MISSION_CLEAR_ALL ().decode(bytes);
					case MSG_ID_MISSION_COUNT: 
						return new MSG_MISSION_COUNT ().decode(bytes);
					case MSG_ID_MISSION_CURRENT: 
						return new MSG_MISSION_CURRENT ().decode(bytes);
					case MSG_ID_MISSION_ITEM: 
						return new MSG_MISSION_ITEM ().decode(bytes);
					case MSG_ID_MISSION_ITEM_INT: 
						return new MSG_MISSION_ITEM_INT ().decode(bytes);
					case MSG_ID_MISSION_ITEM_REACHED: 
						return new MSG_MISSION_ITEM_REACHED ().decode(bytes);
					case MSG_ID_MISSION_REQUEST: 
						return new MSG_MISSION_REQUEST ().decode(bytes);
					case MSG_ID_MISSION_REQUEST_LIST: 
						return new MSG_MISSION_REQUEST_LIST ().decode(bytes);
					case MSG_ID_MISSION_REQUEST_PARTIAL_LIST: 
						return new MSG_MISSION_REQUEST_PARTIAL_LIST ().decode(bytes);
					case MSG_ID_MISSION_SET_CURRENT: 
						return new MSG_MISSION_SET_CURRENT ().decode(bytes);
					case MSG_ID_MISSION_WRITE_PARTIAL_LIST: 
						return new MSG_MISSION_WRITE_PARTIAL_LIST ().decode(bytes);
					case MSG_ID_NAMED_VALUE_FLOAT: 
						return new MSG_NAMED_VALUE_FLOAT ().decode(bytes);
					case MSG_ID_NAMED_VALUE_INT: 
						return new MSG_NAMED_VALUE_INT ().decode(bytes);
					case MSG_ID_NAV_CONTROLLER_OUTPUT: 
						return new MSG_NAV_CONTROLLER_OUTPUT ().decode(bytes);
					case MSG_ID_ONBOARD_HEALTH: 
						return new MSG_ONBOARD_HEALTH ().decode(bytes);
					case MSG_ID_OPTICAL_FLOW: 
						return new MSG_OPTICAL_FLOW ().decode(bytes);
					case MSG_ID_OPTICAL_FLOW_RAD: 
						return new MSG_OPTICAL_FLOW_RAD ().decode(bytes);
					case MSG_ID_PARAM_MAP_RC: 
						return new MSG_PARAM_MAP_RC ().decode(bytes);
					case MSG_ID_PARAM_REQUEST_LIST: 
						return new MSG_PARAM_REQUEST_LIST ().decode(bytes);
					case MSG_ID_PARAM_REQUEST_READ: 
						return new MSG_PARAM_REQUEST_READ ().decode(bytes);
					case MSG_ID_PARAM_SET: 
						return new MSG_PARAM_SET ().decode(bytes);
					case MSG_ID_PARAM_VALUE: 
						return new MSG_PARAM_VALUE ().decode(bytes);
					case MSG_ID_PATTERN_DETECTED: 
						return new MSG_PATTERN_DETECTED ().decode(bytes);
					case MSG_ID_PING: 
						return new MSG_PING ().decode(bytes);
					case MSG_ID_POINT_OF_INTEREST: 
						return new MSG_POINT_OF_INTEREST ().decode(bytes);
					case MSG_ID_POINT_OF_INTEREST_CONNECTION: 
						return new MSG_POINT_OF_INTEREST_CONNECTION ().decode(bytes);
					case MSG_ID_POSITION_CONTROL_SETPOINT: 
						return new MSG_POSITION_CONTROL_SETPOINT ().decode(bytes);
					case MSG_ID_POSITION_TARGET_GLOBAL_INT: 
						return new MSG_POSITION_TARGET_GLOBAL_INT ().decode(bytes);
					case MSG_ID_POSITION_TARGET_LOCAL_NED: 
						return new MSG_POSITION_TARGET_LOCAL_NED ().decode(bytes);
					case MSG_ID_POWER_STATUS: 
						return new MSG_POWER_STATUS ().decode(bytes);
					case MSG_ID_RADIO_STATUS: 
						return new MSG_RADIO_STATUS ().decode(bytes);
					case MSG_ID_RAW_AUX: 
						return new MSG_RAW_AUX ().decode(bytes);
					case MSG_ID_RAW_IMU: 
						return new MSG_RAW_IMU ().decode(bytes);
					case MSG_ID_RAW_PRESSURE: 
						return new MSG_RAW_PRESSURE ().decode(bytes);
					case MSG_ID_RC_CHANNELS: 
						return new MSG_RC_CHANNELS ().decode(bytes);
					case MSG_ID_RC_CHANNELS_OVERRIDE: 
						return new MSG_RC_CHANNELS_OVERRIDE ().decode(bytes);
					case MSG_ID_RC_CHANNELS_RAW: 
						return new MSG_RC_CHANNELS_RAW ().decode(bytes);
					case MSG_ID_RC_CHANNELS_SCALED: 
						return new MSG_RC_CHANNELS_SCALED ().decode(bytes);
					case MSG_ID_REQUEST_DATA_STREAM: 
						return new MSG_REQUEST_DATA_STREAM ().decode(bytes);
					case MSG_ID_SAFETY_ALLOWED_AREA: 
						return new MSG_SAFETY_ALLOWED_AREA ().decode(bytes);
					case MSG_ID_SAFETY_SET_ALLOWED_AREA: 
						return new MSG_SAFETY_SET_ALLOWED_AREA ().decode(bytes);
					case MSG_ID_SCALED_IMU: 
						return new MSG_SCALED_IMU ().decode(bytes);
					case MSG_ID_SCALED_IMU2: 
						return new MSG_SCALED_IMU2 ().decode(bytes);
					case MSG_ID_SCALED_IMU3: 
						return new MSG_SCALED_IMU3 ().decode(bytes);
					case MSG_ID_SCALED_PRESSURE: 
						return new MSG_SCALED_PRESSURE ().decode(bytes);
					case MSG_ID_SCALED_PRESSURE2: 
						return new MSG_SCALED_PRESSURE2 ().decode(bytes);
					case MSG_ID_SERIAL_CONTROL: 
						return new MSG_SERIAL_CONTROL ().decode(bytes);
					case MSG_ID_SERVO_OUTPUT_RAW: 
						return new MSG_SERVO_OUTPUT_RAW ().decode(bytes);
					case MSG_ID_SET_ACTUATOR_CONTROL_TARGET: 
						return new MSG_SET_ACTUATOR_CONTROL_TARGET ().decode(bytes);
					case MSG_ID_SET_ATTITUDE_TARGET: 
						return new MSG_SET_ATTITUDE_TARGET ().decode(bytes);
					case MSG_ID_SET_CAM_SHUTTER: 
						return new MSG_SET_CAM_SHUTTER ().decode(bytes);
					case MSG_ID_SET_GPS_GLOBAL_ORIGIN: 
						return new MSG_SET_GPS_GLOBAL_ORIGIN ().decode(bytes);
					case MSG_ID_SET_MODE: 
						return new MSG_SET_MODE ().decode(bytes);
					case MSG_ID_SET_POSITION_CONTROL_OFFSET: 
						return new MSG_SET_POSITION_CONTROL_OFFSET ().decode(bytes);
					case MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: 
						return new MSG_SET_POSITION_TARGET_GLOBAL_INT ().decode(bytes);
					case MSG_ID_SET_POSITION_TARGET_LOCAL_NED: 
						return new MSG_SET_POSITION_TARGET_LOCAL_NED ().decode(bytes);
					case MSG_ID_SIM_STATE: 
						return new MSG_SIM_STATE ().decode(bytes);
					case MSG_ID_STATUSTEXT: 
						return new MSG_STATUSTEXT ().decode(bytes);
					case MSG_ID_SYSTEM_TIME: 
						return new MSG_SYSTEM_TIME ().decode(bytes);
					case MSG_ID_SYS_STATUS: 
						return new MSG_SYS_STATUS ().decode(bytes);
					case MSG_ID_TERRAIN_CHECK: 
						return new MSG_TERRAIN_CHECK ().decode(bytes);
					case MSG_ID_TERRAIN_DATA: 
						return new MSG_TERRAIN_DATA ().decode(bytes);
					case MSG_ID_TERRAIN_REPORT: 
						return new MSG_TERRAIN_REPORT ().decode(bytes);
					case MSG_ID_TERRAIN_REQUEST: 
						return new MSG_TERRAIN_REQUEST ().decode(bytes);
					case MSG_ID_TIMESYNC: 
						return new MSG_TIMESYNC ().decode(bytes);
					case MSG_ID_V2_EXTENSION: 
						return new MSG_V2_EXTENSION ().decode(bytes);
					case MSG_ID_VFR_HUD: 
						return new MSG_VFR_HUD ().decode(bytes);
					case MSG_ID_VICON_POSITION_ESTIMATE: 
						return new MSG_VICON_POSITION_ESTIMATE ().decode(bytes);
					case MSG_ID_VISION_POSITION_ESTIMATE: 
						return new MSG_VISION_POSITION_ESTIMATE ().decode(bytes);
					case MSG_ID_VISION_SPEED_ESTIMATE: 
						return new MSG_VISION_SPEED_ESTIMATE ().decode(bytes);
					case MSG_ID_WATCHDOG_COMMAND: 
						return new MSG_WATCHDOG_COMMAND ().decode(bytes);
					case MSG_ID_WATCHDOG_HEARTBEAT: 
						return new MSG_WATCHDOG_HEARTBEAT ().decode(bytes);
					case MSG_ID_WATCHDOG_PROCESS_INFO: 
						return new MSG_WATCHDOG_PROCESS_INFO ().decode(bytes);
					case MSG_ID_WATCHDOG_PROCESS_STATUS: 
						return new MSG_WATCHDOG_PROCESS_STATUS ().decode(bytes);
					default: throw new IllegalArgumentException("Message with id=" + msg + " is not supported.");
			}
		}
	
		protected Message(short msg, short lngth, String msgName)
		{
			this.msg = msg;			
			this.lngth = lngth;
			this.msgName = msgName;
		}
		
		protected Message(byte[] bytes, short msg, short lngth, String msgName) {
			this.msg = msg;
			this.lngth = lngth;
			this.msgName = msgName;
			decode(bytes);
		}
		
		protected Message(short sys, short comp, short lngth, String msgName) {
			this.sys = sys;
			this.lngth = lngth;
			this.msgName = msgName;
			this.comp = Component.getNameValueOf(comp);
		}
	
		public abstract int getCRCExtra();
		
		public String getMsgName()
		{
			return msgName;
		}
		
		protected abstract ByteBuffer decodePayload(ByteBuffer buffer);
		
		protected Message decode(byte[] bytes)
		{
			ByteBuffer buffer = ByteBuffer.wrap(bytes);
			buffer.order(ByteOrder.LITTLE_ENDIAN);
			short startSign = (short)(buffer.get() & 0xFF);
			
			if(startSign != PACKET_START_SIGN) {
				throw new IllegalStateException("Unsupported protocol. Excepted: " + 
					PACKET_START_SIGN + " got: " + startSign);
			}
			
			this.lngth = (short)(buffer.get() & 0xFF);
			// TODO: validate against known length for given msgID
			
			this.sqnc = (short)(buffer.get() & 0xFF);
			this.sys = (short)(buffer.get() & 0xFF);
			this.comp = Component.getNameValueOf(buffer.get() & 0xFF);
			
			short tmp = (short)(buffer.get() & 0xFF); // messageID not used
			if(tmp != getMsg()) {
				throw new IllegalArgumentException("Invalid message id. Expected: " + 
					getMsg() + " got: " + msg);
			 }
			
			decodePayload(buffer);
			// TODO: CRC		
			
			return this;	
		}

		public abstract ByteBuffer encodePayload(ByteBuffer buffer);
		
		public byte[] encode() {
			ByteBuffer buffer = ByteBuffer.allocate(HEADER_LENGTH+getLngth());
			buffer.order(ByteOrder.LITTLE_ENDIAN);
			
			this.setSqnc((short)0x4E); // TMP
			
			buffer.put((byte)(PACKET_START_SIGN & 0xFF));
			buffer.put((byte)(getLngth() & 0xFF));
			buffer.put((byte)(getSqnc() & 0xFF));//sequence
			buffer.put((byte)(getSys() & 0xFF));
			buffer.put((byte)(Component.getIDValueOf(getComp()) & 0xFF));
			buffer.put((byte)(getMsg() & 0xFF));
			encodePayload(buffer);
			
			// Calculate CRC
			buffer.put((byte)(getCRCExtra() & 0xFF));
			
			byte[] bytes = new byte[HEADER_LENGTH+getLngth()];
			buffer.rewind();
			buffer.get(bytes);
			
			int crc = getCRC(bytes, 1, bytes.length-1);
			bytes[bytes.length-2] = (byte)((crc>>8)&0xFF);
			bytes[bytes.length-1] = (byte)((crc)&0xFF);

			return bytes;
		}
		
		public static byte[] encode(short lngth, short sqnc, short sys, short comp, short msg, byte[] payload, int crcExtra) {
			ByteBuffer buffer = ByteBuffer.allocate(HEADER_LENGTH+lngth);
			buffer.order(ByteOrder.LITTLE_ENDIAN);
			
			buffer.put((byte)(PACKET_START_SIGN & 0xFF));
			buffer.put((byte)(lngth & 0xFF));
			buffer.put((byte)(sqnc & 0xFF));//sequence
			buffer.put((byte)(sys & 0xFF));
			buffer.put((byte)(comp & 0xFF));
			buffer.put((byte)(msg & 0xFF));
			for(byte b: payload)
			{
				buffer.put(b);
			}	
			
			// Calculate CRC
			buffer.put((byte)(crcExtra & 0xFF));
			byte[] bytes = new byte[HEADER_LENGTH+lngth];
			buffer.rewind();
			buffer.get(bytes);
			int crc = getCRC(bytes, 1, bytes.length-1);
			bytes[bytes.length-2] = (byte)((crc>>8)&0xFF);
			bytes[bytes.length-1] = (byte)((crc)&0xFF);

			return bytes;
		}

		private static int getCRC(byte[] bytes, int offset, int length) {
			int crc = 0xffff;
			
			for(int c=offset; c<length; ++c) {
				int tmp;
				int data = bytes[c] & 0xff;	//cast because we want an unsigned type
				tmp = data ^ (crc & 0xff);
				tmp ^= (tmp << 4) & 0xff;
				crc = ((crc >> 8) & 0xff) ^ (tmp << 8) ^ (tmp << 3)
						^ ((tmp >> 4) & 0xf);
			}

			return (crc&0xFF) << 8 | (crc>>8&0x00FF);
		}
	}
	
	/*
	 * Set the vehicle attitude and body angular rates.
	 */
	public static class MSG_ACTUATOR_CONTROL_TARGET extends Message {
	
		private long time_usec; // Timestamp (micros since boot or Unix epoch)
		private float[] controls = new float[8]; // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
		private int group_mlx; // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	
		public MSG_ACTUATOR_CONTROL_TARGET () {
			super(MSG_ID_ACTUATOR_CONTROL_TARGET, (short)69, "ACTUATOR_CONTROL_TARGET");
		}

		public MSG_ACTUATOR_CONTROL_TARGET (byte[] bytes) {
			super(bytes, MSG_ID_ACTUATOR_CONTROL_TARGET, (short)69, "ACTUATOR_CONTROL_TARGET");
		}
	
		public MSG_ACTUATOR_CONTROL_TARGET (short sys, short comp, long time_usec  , float controls [] , int group_mlx  ) {
			super(sys, comp, (short)69, "ACTUATOR_CONTROL_TARGET");
			this.time_usec = time_usec;
			this.controls = controls;
			this.group_mlx = group_mlx;
		}

		@Override
		public int getCRCExtra() {
			return 181;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float[] getControls() {
			return controls;
		}
		
		public void setControls(float controls[]) {
			this.controls = controls;
		}
		
		public int getGroup_mlx() {
			return group_mlx;
		}
		
		public void setGroup_mlx(int group_mlx) {
			this.group_mlx = group_mlx;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ACTUATOR_CONTROL_TARGET";
			
			time_usec = buffer.getLong(); // uint64_t
  			for(int c=0; c<8; ++c) {
				controls [c] = buffer.getFloat(); // float[8]
 			}
			
 			group_mlx = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			for(int c=0; c<8; ++c) {
				buffer.putFloat(controls [c]); // float[8]
 			}
			
 			buffer.put((byte)(group_mlx & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ACTUATOR_CONTROL_TARGET { " + 
			"time_usec = " + time_usec + ", " + 
			"controls = " + controls + ", " + 
			"group_mlx = " + group_mlx + ",  }";
		}
	}
	/*
	 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
	 */
	public static class MSG_ATTITUDE extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float roll; // Roll angle (rad, -pi..+pi)
		private float pitch; // Pitch angle (rad, -pi..+pi)
		private float yaw; // Yaw angle (rad, -pi..+pi)
		private float rollspeed; // Roll angular speed (rad/s)
		private float pitchspeed; // Pitch angular speed (rad/s)
		private float yawspeed; // Yaw angular speed (rad/s)
	
		public MSG_ATTITUDE () {
			super(MSG_ID_ATTITUDE, (short)28, "ATTITUDE");
		}

		public MSG_ATTITUDE (byte[] bytes) {
			super(bytes, MSG_ID_ATTITUDE, (short)28, "ATTITUDE");
		}
	
		public MSG_ATTITUDE (short sys, short comp, long time_boot_ms  , float roll  , float pitch  , float yaw  , float rollspeed  , float pitchspeed  , float yawspeed  ) {
			super(sys, comp, (short)28, "ATTITUDE");
			this.time_boot_ms = time_boot_ms;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.rollspeed = rollspeed;
			this.pitchspeed = pitchspeed;
			this.yawspeed = yawspeed;
		}

		@Override
		public int getCRCExtra() {
			return 39;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getRollspeed() {
			return rollspeed;
		}
		
		public void setRollspeed(float rollspeed) {
			this.rollspeed = rollspeed;
		}
		
		public float getPitchspeed() {
			return pitchspeed;
		}
		
		public void setPitchspeed(float pitchspeed) {
			this.pitchspeed = pitchspeed;
		}
		
		public float getYawspeed() {
			return yawspeed;
		}
		
		public void setYawspeed(float yawspeed) {
			this.yawspeed = yawspeed;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATTITUDE";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			rollspeed = buffer.getFloat(); // float
  			pitchspeed = buffer.getFloat(); // float
  			yawspeed = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(rollspeed); // float
  			buffer.putFloat(pitchspeed); // float
  			buffer.putFloat(yawspeed); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATTITUDE { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"rollspeed = " + rollspeed + ", " + 
			"pitchspeed = " + pitchspeed + ", " + 
			"yawspeed = " + yawspeed + ",  }";
		}
	}
	public static class MSG_ATTITUDE_CONTROL extends Message {
	
		private float roll; // roll
		private float pitch; // pitch
		private float yaw; // yaw
		private float thrust; // thrust
		private int target; // The system to be controlled
		private int roll_manual; // roll control enabled auto:0, manual:1
		private int pitch_manual; // pitch auto:0, manual:1
		private int yaw_manual; // yaw auto:0, manual:1
		private int thrust_manual; // thrust auto:0, manual:1
	
		public MSG_ATTITUDE_CONTROL () {
			super(MSG_ID_ATTITUDE_CONTROL, (short)21, "ATTITUDE_CONTROL");
		}

		public MSG_ATTITUDE_CONTROL (byte[] bytes) {
			super(bytes, MSG_ID_ATTITUDE_CONTROL, (short)21, "ATTITUDE_CONTROL");
		}
	
		public MSG_ATTITUDE_CONTROL (short sys, short comp, float roll  , float pitch  , float yaw  , float thrust  , int target  , int roll_manual  , int pitch_manual  , int yaw_manual  , int thrust_manual  ) {
			super(sys, comp, (short)21, "ATTITUDE_CONTROL");
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.thrust = thrust;
			this.target = target;
			this.roll_manual = roll_manual;
			this.pitch_manual = pitch_manual;
			this.yaw_manual = yaw_manual;
			this.thrust_manual = thrust_manual;
		}

		@Override
		public int getCRCExtra() {
			return 254;
		}
	
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getThrust() {
			return thrust;
		}
		
		public void setThrust(float thrust) {
			this.thrust = thrust;
		}
		
		public int getTarget() {
			return target;
		}
		
		public void setTarget(int target) {
			this.target = target;
		}
		
		public int getRoll_manual() {
			return roll_manual;
		}
		
		public void setRoll_manual(int roll_manual) {
			this.roll_manual = roll_manual;
		}
		
		public int getPitch_manual() {
			return pitch_manual;
		}
		
		public void setPitch_manual(int pitch_manual) {
			this.pitch_manual = pitch_manual;
		}
		
		public int getYaw_manual() {
			return yaw_manual;
		}
		
		public void setYaw_manual(int yaw_manual) {
			this.yaw_manual = yaw_manual;
		}
		
		public int getThrust_manual() {
			return thrust_manual;
		}
		
		public void setThrust_manual(int thrust_manual) {
			this.thrust_manual = thrust_manual;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATTITUDE_CONTROL";
			
			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			thrust = buffer.getFloat(); // float
  			target = (int)buffer.get() & 0xff; // uint8_t
  			roll_manual = (int)buffer.get() & 0xff; // uint8_t
  			pitch_manual = (int)buffer.get() & 0xff; // uint8_t
  			yaw_manual = (int)buffer.get() & 0xff; // uint8_t
  			thrust_manual = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(thrust); // float
  			buffer.put((byte)(target & 0xff)); // uint8_t
  			buffer.put((byte)(roll_manual & 0xff)); // uint8_t
  			buffer.put((byte)(pitch_manual & 0xff)); // uint8_t
  			buffer.put((byte)(yaw_manual & 0xff)); // uint8_t
  			buffer.put((byte)(thrust_manual & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATTITUDE_CONTROL { " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"thrust = " + thrust + ", " + 
			"target = " + target + ", " + 
			"roll_manual = " + roll_manual + ", " + 
			"pitch_manual = " + pitch_manual + ", " + 
			"yaw_manual = " + yaw_manual + ", " + 
			"thrust_manual = " + thrust_manual + ",  }";
		}
	}
	/*
	 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
	 */
	public static class MSG_ATTITUDE_QUATERNION extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float q1; // Quaternion component 1, w (1 in null-rotation)
		private float q2; // Quaternion component 2, x (0 in null-rotation)
		private float q3; // Quaternion component 3, y (0 in null-rotation)
		private float q4; // Quaternion component 4, z (0 in null-rotation)
		private float rollspeed; // Roll angular speed (rad/s)
		private float pitchspeed; // Pitch angular speed (rad/s)
		private float yawspeed; // Yaw angular speed (rad/s)
	
		public MSG_ATTITUDE_QUATERNION () {
			super(MSG_ID_ATTITUDE_QUATERNION, (short)32, "ATTITUDE_QUATERNION");
		}

		public MSG_ATTITUDE_QUATERNION (byte[] bytes) {
			super(bytes, MSG_ID_ATTITUDE_QUATERNION, (short)32, "ATTITUDE_QUATERNION");
		}
	
		public MSG_ATTITUDE_QUATERNION (short sys, short comp, long time_boot_ms  , float q1  , float q2  , float q3  , float q4  , float rollspeed  , float pitchspeed  , float yawspeed  ) {
			super(sys, comp, (short)32, "ATTITUDE_QUATERNION");
			this.time_boot_ms = time_boot_ms;
			this.q1 = q1;
			this.q2 = q2;
			this.q3 = q3;
			this.q4 = q4;
			this.rollspeed = rollspeed;
			this.pitchspeed = pitchspeed;
			this.yawspeed = yawspeed;
		}

		@Override
		public int getCRCExtra() {
			return 246;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getQ1() {
			return q1;
		}
		
		public void setQ1(float q1) {
			this.q1 = q1;
		}
		
		public float getQ2() {
			return q2;
		}
		
		public void setQ2(float q2) {
			this.q2 = q2;
		}
		
		public float getQ3() {
			return q3;
		}
		
		public void setQ3(float q3) {
			this.q3 = q3;
		}
		
		public float getQ4() {
			return q4;
		}
		
		public void setQ4(float q4) {
			this.q4 = q4;
		}
		
		public float getRollspeed() {
			return rollspeed;
		}
		
		public void setRollspeed(float rollspeed) {
			this.rollspeed = rollspeed;
		}
		
		public float getPitchspeed() {
			return pitchspeed;
		}
		
		public void setPitchspeed(float pitchspeed) {
			this.pitchspeed = pitchspeed;
		}
		
		public float getYawspeed() {
			return yawspeed;
		}
		
		public void setYawspeed(float yawspeed) {
			this.yawspeed = yawspeed;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATTITUDE_QUATERNION";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			q1 = buffer.getFloat(); // float
  			q2 = buffer.getFloat(); // float
  			q3 = buffer.getFloat(); // float
  			q4 = buffer.getFloat(); // float
  			rollspeed = buffer.getFloat(); // float
  			pitchspeed = buffer.getFloat(); // float
  			yawspeed = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(q1); // float
  			buffer.putFloat(q2); // float
  			buffer.putFloat(q3); // float
  			buffer.putFloat(q4); // float
  			buffer.putFloat(rollspeed); // float
  			buffer.putFloat(pitchspeed); // float
  			buffer.putFloat(yawspeed); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATTITUDE_QUATERNION { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"q1 = " + q1 + ", " + 
			"q2 = " + q2 + ", " + 
			"q3 = " + q3 + ", " + 
			"q4 = " + q4 + ", " + 
			"rollspeed = " + rollspeed + ", " + 
			"pitchspeed = " + pitchspeed + ", " + 
			"yawspeed = " + yawspeed + ",  }";
		}
	}
	/*
	 * The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
	 */
	public static class MSG_ATTITUDE_QUATERNION_COV extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float[] q = new float[4]; // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
		private float rollspeed; // Roll angular speed (rad/s)
		private float pitchspeed; // Pitch angular speed (rad/s)
		private float yawspeed; // Yaw angular speed (rad/s)
		private float[] covariance = new float[9]; // Attitude covariance
	
		public MSG_ATTITUDE_QUATERNION_COV () {
			super(MSG_ID_ATTITUDE_QUATERNION_COV, (short)24, "ATTITUDE_QUATERNION_COV");
		}

		public MSG_ATTITUDE_QUATERNION_COV (byte[] bytes) {
			super(bytes, MSG_ID_ATTITUDE_QUATERNION_COV, (short)24, "ATTITUDE_QUATERNION_COV");
		}
	
		public MSG_ATTITUDE_QUATERNION_COV (short sys, short comp, long time_boot_ms  , float q [] , float rollspeed  , float pitchspeed  , float yawspeed  , float covariance [] ) {
			super(sys, comp, (short)24, "ATTITUDE_QUATERNION_COV");
			this.time_boot_ms = time_boot_ms;
			this.q = q;
			this.rollspeed = rollspeed;
			this.pitchspeed = pitchspeed;
			this.yawspeed = yawspeed;
			this.covariance = covariance;
		}

		@Override
		public int getCRCExtra() {
			return 153;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float[] getQ() {
			return q;
		}
		
		public void setQ(float q[]) {
			this.q = q;
		}
		
		public float getRollspeed() {
			return rollspeed;
		}
		
		public void setRollspeed(float rollspeed) {
			this.rollspeed = rollspeed;
		}
		
		public float getPitchspeed() {
			return pitchspeed;
		}
		
		public void setPitchspeed(float pitchspeed) {
			this.pitchspeed = pitchspeed;
		}
		
		public float getYawspeed() {
			return yawspeed;
		}
		
		public void setYawspeed(float yawspeed) {
			this.yawspeed = yawspeed;
		}
		
		public float[] getCovariance() {
			return covariance;
		}
		
		public void setCovariance(float covariance[]) {
			this.covariance = covariance;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATTITUDE_QUATERNION_COV";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			for(int c=0; c<4; ++c) {
				q [c] = buffer.getFloat(); // float[4]
 			}
			
 			rollspeed = buffer.getFloat(); // float
  			pitchspeed = buffer.getFloat(); // float
  			yawspeed = buffer.getFloat(); // float
  			for(int c=0; c<9; ++c) {
				covariance [c] = buffer.getFloat(); // float[9]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			for(int c=0; c<4; ++c) {
				buffer.putFloat(q [c]); // float[4]
 			}
			
 			buffer.putFloat(rollspeed); // float
  			buffer.putFloat(pitchspeed); // float
  			buffer.putFloat(yawspeed); // float
  			for(int c=0; c<9; ++c) {
				buffer.putFloat(covariance [c]); // float[9]
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATTITUDE_QUATERNION_COV { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"q = " + q + ", " + 
			"rollspeed = " + rollspeed + ", " + 
			"pitchspeed = " + pitchspeed + ", " + 
			"yawspeed = " + yawspeed + ", " + 
			"covariance = " + covariance + ",  }";
		}
	}
	/*
	 * Set the vehicle attitude and body angular rates.
	 */
	public static class MSG_ATTITUDE_TARGET extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot
		private float[] q = new float[4]; // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		private float body_roll_rate; // Body roll rate in radians per second
		private float body_pitch_rate; // Body roll rate in radians per second
		private float body_yaw_rate; // Body roll rate in radians per second
		private float thrust; // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		private int type_mask; // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
	
		public MSG_ATTITUDE_TARGET () {
			super(MSG_ID_ATTITUDE_TARGET, (short)25, "ATTITUDE_TARGET");
		}

		public MSG_ATTITUDE_TARGET (byte[] bytes) {
			super(bytes, MSG_ID_ATTITUDE_TARGET, (short)25, "ATTITUDE_TARGET");
		}
	
		public MSG_ATTITUDE_TARGET (short sys, short comp, long time_boot_ms  , float q [] , float body_roll_rate  , float body_pitch_rate  , float body_yaw_rate  , float thrust  , int type_mask  ) {
			super(sys, comp, (short)25, "ATTITUDE_TARGET");
			this.time_boot_ms = time_boot_ms;
			this.q = q;
			this.body_roll_rate = body_roll_rate;
			this.body_pitch_rate = body_pitch_rate;
			this.body_yaw_rate = body_yaw_rate;
			this.thrust = thrust;
			this.type_mask = type_mask;
		}

		@Override
		public int getCRCExtra() {
			return 22;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float[] getQ() {
			return q;
		}
		
		public void setQ(float q[]) {
			this.q = q;
		}
		
		public float getBody_roll_rate() {
			return body_roll_rate;
		}
		
		public void setBody_roll_rate(float body_roll_rate) {
			this.body_roll_rate = body_roll_rate;
		}
		
		public float getBody_pitch_rate() {
			return body_pitch_rate;
		}
		
		public void setBody_pitch_rate(float body_pitch_rate) {
			this.body_pitch_rate = body_pitch_rate;
		}
		
		public float getBody_yaw_rate() {
			return body_yaw_rate;
		}
		
		public void setBody_yaw_rate(float body_yaw_rate) {
			this.body_yaw_rate = body_yaw_rate;
		}
		
		public float getThrust() {
			return thrust;
		}
		
		public void setThrust(float thrust) {
			this.thrust = thrust;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATTITUDE_TARGET";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			for(int c=0; c<4; ++c) {
				q [c] = buffer.getFloat(); // float[4]
 			}
			
 			body_roll_rate = buffer.getFloat(); // float
  			body_pitch_rate = buffer.getFloat(); // float
  			body_yaw_rate = buffer.getFloat(); // float
  			thrust = buffer.getFloat(); // float
  			type_mask = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			for(int c=0; c<4; ++c) {
				buffer.putFloat(q [c]); // float[4]
 			}
			
 			buffer.putFloat(body_roll_rate); // float
  			buffer.putFloat(body_pitch_rate); // float
  			buffer.putFloat(body_yaw_rate); // float
  			buffer.putFloat(thrust); // float
  			buffer.put((byte)(type_mask & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATTITUDE_TARGET { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"q = " + q + ", " + 
			"body_roll_rate = " + body_roll_rate + ", " + 
			"body_pitch_rate = " + body_pitch_rate + ", " + 
			"body_yaw_rate = " + body_yaw_rate + ", " + 
			"thrust = " + thrust + ", " + 
			"type_mask = " + type_mask + ",  }";
		}
	}
	/*
	 * Motion capture attitude and position
	 */
	public static class MSG_ATT_POS_MOCAP extends Message {
	
		private long time_usec; // Timestamp (micros since boot or Unix epoch)
		private float[] q = new float[4]; // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		private float x; // X position in meters (NED)
		private float y; // Y position in meters (NED)
		private float z; // Z position in meters (NED)
	
		public MSG_ATT_POS_MOCAP () {
			super(MSG_ID_ATT_POS_MOCAP, (short)80, "ATT_POS_MOCAP");
		}

		public MSG_ATT_POS_MOCAP (byte[] bytes) {
			super(bytes, MSG_ID_ATT_POS_MOCAP, (short)80, "ATT_POS_MOCAP");
		}
	
		public MSG_ATT_POS_MOCAP (short sys, short comp, long time_usec  , float q [] , float x  , float y  , float z  ) {
			super(sys, comp, (short)80, "ATT_POS_MOCAP");
			this.time_usec = time_usec;
			this.q = q;
			this.x = x;
			this.y = y;
			this.z = z;
		}

		@Override
		public int getCRCExtra() {
			return 109;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float[] getQ() {
			return q;
		}
		
		public void setQ(float q[]) {
			this.q = q;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ATT_POS_MOCAP";
			
			time_usec = buffer.getLong(); // uint64_t
  			for(int c=0; c<4; ++c) {
				q [c] = buffer.getFloat(); // float[4]
 			}
			
 			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			for(int c=0; c<4; ++c) {
				buffer.putFloat(q [c]); // float[4]
 			}
			
 			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ATT_POS_MOCAP { " + 
			"time_usec = " + time_usec + ", " + 
			"q = " + q + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ",  }";
		}
	}
	/*
	 * Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
	 */
	public static class MSG_AUTH_KEY extends Message {
	
		private char[] key = new char[32]; // key
	
		public MSG_AUTH_KEY () {
			super(MSG_ID_AUTH_KEY, (short)1, "AUTH_KEY");
		}

		public MSG_AUTH_KEY (byte[] bytes) {
			super(bytes, MSG_ID_AUTH_KEY, (short)1, "AUTH_KEY");
		}
	
		public MSG_AUTH_KEY (short sys, short comp, char key [] ) {
			super(sys, comp, (short)1, "AUTH_KEY");
			this.key = key;
		}

		@Override
		public int getCRCExtra() {
			return 119;
		}
	
		public char[] getKey() {
			return key;
		}
		
		public void setKey(char key[]) {
			this.key = key;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "AUTH_KEY";
			
			for(int c=0; c<32; ++c) {
				key [c] =  (char)buffer.get(); // char[32]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			for(int c=0; c<32; ++c) {
				buffer.put((byte)(key [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_AUTH_KEY { " + 
			"key = " + key + ",  }";
		}
	}
	/*
	 * Version and capability of autopilot software
	 */
	public static class MSG_AUTOPILOT_VERSION extends Message {
	
		private long capabilities; // bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
		private long uid; // UID if provided by hardware
		private long flight_sw_version; // Firmware version number
		private long middleware_sw_version; // Middleware version number
		private long os_sw_version; // Operating system version number
		private long board_version; // HW / board version (last 8 bytes should be silicon ID, if any)
		private int vendor_id; // ID of the board vendor
		private int product_id; // ID of the product
		private int[] flight_custom_version = new int[8]; // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
		private int[] middleware_custom_version = new int[8]; // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
		private int[] os_custom_version = new int[8]; // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	
		public MSG_AUTOPILOT_VERSION () {
			super(MSG_ID_AUTOPILOT_VERSION, (short)151, "AUTOPILOT_VERSION");
		}

		public MSG_AUTOPILOT_VERSION (byte[] bytes) {
			super(bytes, MSG_ID_AUTOPILOT_VERSION, (short)151, "AUTOPILOT_VERSION");
		}
	
		public MSG_AUTOPILOT_VERSION (short sys, short comp, long capabilities  , long uid  , long flight_sw_version  , long middleware_sw_version  , long os_sw_version  , long board_version  , int vendor_id  , int product_id  , int flight_custom_version [] , int middleware_custom_version [] , int os_custom_version [] ) {
			super(sys, comp, (short)151, "AUTOPILOT_VERSION");
			this.capabilities = capabilities;
			this.uid = uid;
			this.flight_sw_version = flight_sw_version;
			this.middleware_sw_version = middleware_sw_version;
			this.os_sw_version = os_sw_version;
			this.board_version = board_version;
			this.vendor_id = vendor_id;
			this.product_id = product_id;
			this.flight_custom_version = flight_custom_version;
			this.middleware_custom_version = middleware_custom_version;
			this.os_custom_version = os_custom_version;
		}

		@Override
		public int getCRCExtra() {
			return 178;
		}
	
		public long getCapabilities() {
			return capabilities;
		}
		
		public void setCapabilities(long capabilities) {
			this.capabilities = capabilities;
		}
		
		public long getUid() {
			return uid;
		}
		
		public void setUid(long uid) {
			this.uid = uid;
		}
		
		public long getFlight_sw_version() {
			return flight_sw_version;
		}
		
		public void setFlight_sw_version(long flight_sw_version) {
			this.flight_sw_version = flight_sw_version;
		}
		
		public long getMiddleware_sw_version() {
			return middleware_sw_version;
		}
		
		public void setMiddleware_sw_version(long middleware_sw_version) {
			this.middleware_sw_version = middleware_sw_version;
		}
		
		public long getOs_sw_version() {
			return os_sw_version;
		}
		
		public void setOs_sw_version(long os_sw_version) {
			this.os_sw_version = os_sw_version;
		}
		
		public long getBoard_version() {
			return board_version;
		}
		
		public void setBoard_version(long board_version) {
			this.board_version = board_version;
		}
		
		public int getVendor_id() {
			return vendor_id;
		}
		
		public void setVendor_id(int vendor_id) {
			this.vendor_id = vendor_id;
		}
		
		public int getProduct_id() {
			return product_id;
		}
		
		public void setProduct_id(int product_id) {
			this.product_id = product_id;
		}
		
		public int[] getFlight_custom_version() {
			return flight_custom_version;
		}
		
		public void setFlight_custom_version(int flight_custom_version[]) {
			this.flight_custom_version = flight_custom_version;
		}
		
		public int[] getMiddleware_custom_version() {
			return middleware_custom_version;
		}
		
		public void setMiddleware_custom_version(int middleware_custom_version[]) {
			this.middleware_custom_version = middleware_custom_version;
		}
		
		public int[] getOs_custom_version() {
			return os_custom_version;
		}
		
		public void setOs_custom_version(int os_custom_version[]) {
			this.os_custom_version = os_custom_version;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "AUTOPILOT_VERSION";
			
			capabilities = buffer.getLong(); // uint64_t
  			uid = buffer.getLong(); // uint64_t
  			flight_sw_version = buffer.getInt() & 0xffffffff; // uint32_t
  			middleware_sw_version = buffer.getInt() & 0xffffffff; // uint32_t
  			os_sw_version = buffer.getInt() & 0xffffffff; // uint32_t
  			board_version = buffer.getInt() & 0xffffffff; // uint32_t
  			vendor_id = buffer.getShort() & 0xffff; // uint16_t
  			product_id = buffer.getShort() & 0xffff; // uint16_t
  			for(int c=0; c<8; ++c) {
				flight_custom_version [c] =  (int)buffer.get() & 0xff; // uint8_t[8]
 			}
			
 			for(int c=0; c<8; ++c) {
				middleware_custom_version [c] =  (int)buffer.get() & 0xff; // uint8_t[8]
 			}
			
 			for(int c=0; c<8; ++c) {
				os_custom_version [c] =  (int)buffer.get() & 0xff; // uint8_t[8]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(capabilities); // uint64_t
  			buffer.putLong(uid); // uint64_t
  			buffer.putInt((int)(flight_sw_version & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(middleware_sw_version & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(os_sw_version & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(board_version & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(vendor_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(product_id & 0xffff)); // uint16_t
  			for(int c=0; c<8; ++c) {
				buffer.put((byte)(flight_custom_version [c] & 0xff));
 			}
			
 			for(int c=0; c<8; ++c) {
				buffer.put((byte)(middleware_custom_version [c] & 0xff));
 			}
			
 			for(int c=0; c<8; ++c) {
				buffer.put((byte)(os_custom_version [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_AUTOPILOT_VERSION { " + 
			"capabilities = " + capabilities + ", " + 
			"uid = " + uid + ", " + 
			"flight_sw_version = " + flight_sw_version + ", " + 
			"middleware_sw_version = " + middleware_sw_version + ", " + 
			"os_sw_version = " + os_sw_version + ", " + 
			"board_version = " + board_version + ", " + 
			"vendor_id = " + vendor_id + ", " + 
			"product_id = " + product_id + ", " + 
			"flight_custom_version = " + flight_custom_version + ", " + 
			"middleware_custom_version = " + middleware_custom_version + ", " + 
			"os_custom_version = " + os_custom_version + ",  }";
		}
	}
	/*
	 * Battery information
	 */
	public static class MSG_BATTERY_STATUS extends Message {
	
		private int current_consumed; // Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
		private int energy_consumed; // Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
		private int temperature; // Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
		private int[] voltages = new int[10]; // Battery voltage of cells, in millivolts (1 = 1 millivolt)
		private int current_battery; // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
		private int id; // Battery ID
		private int battery_function; // Function of the battery
		private int type; // Type (chemistry) of the battery
		private int battery_remaining; // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
	
		public MSG_BATTERY_STATUS () {
			super(MSG_ID_BATTERY_STATUS, (short)18, "BATTERY_STATUS");
		}

		public MSG_BATTERY_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_BATTERY_STATUS, (short)18, "BATTERY_STATUS");
		}
	
		public MSG_BATTERY_STATUS (short sys, short comp, int current_consumed  , int energy_consumed  , int temperature  , int voltages [] , int current_battery  , int id  , int battery_function  , int type  , int battery_remaining  ) {
			super(sys, comp, (short)18, "BATTERY_STATUS");
			this.current_consumed = current_consumed;
			this.energy_consumed = energy_consumed;
			this.temperature = temperature;
			this.voltages = voltages;
			this.current_battery = current_battery;
			this.id = id;
			this.battery_function = battery_function;
			this.type = type;
			this.battery_remaining = battery_remaining;
		}

		@Override
		public int getCRCExtra() {
			return 154;
		}
	
		public int getCurrent_consumed() {
			return current_consumed;
		}
		
		public void setCurrent_consumed(int current_consumed) {
			this.current_consumed = current_consumed;
		}
		
		public int getEnergy_consumed() {
			return energy_consumed;
		}
		
		public void setEnergy_consumed(int energy_consumed) {
			this.energy_consumed = energy_consumed;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
		public int[] getVoltages() {
			return voltages;
		}
		
		public void setVoltages(int voltages[]) {
			this.voltages = voltages;
		}
		
		public int getCurrent_battery() {
			return current_battery;
		}
		
		public void setCurrent_battery(int current_battery) {
			this.current_battery = current_battery;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
		public int getBattery_function() {
			return battery_function;
		}
		
		public void setBattery_function(int battery_function) {
			this.battery_function = battery_function;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getBattery_remaining() {
			return battery_remaining;
		}
		
		public void setBattery_remaining(int battery_remaining) {
			this.battery_remaining = battery_remaining;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "BATTERY_STATUS";
			
			current_consumed = buffer.getInt(); // int32_t
  			energy_consumed = buffer.getInt(); // int32_t
  			temperature = buffer.getShort(); // int16_t
  			for(int c=0; c<10; ++c) {
				voltages [c] = buffer.getShort() & 0xffff; // uint16_t[10]
 			}
			
 			current_battery = buffer.getShort(); // int16_t
  			id = (int)buffer.get() & 0xff; // uint8_t
  			battery_function = (int)buffer.get() & 0xff; // uint8_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			battery_remaining = (int)buffer.get(); // int8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(current_consumed)); // int32_t
  			buffer.putInt((int)(energy_consumed)); // int32_t
  			buffer.putShort((short)(temperature)); // int16_t
  			for(int c=0; c<10; ++c) {
				buffer.putShort((short)(voltages [c] & 0xffff));
 			}
			
 			buffer.putShort((short)(current_battery)); // int16_t
  			buffer.put((byte)(id & 0xff)); // uint8_t
  			buffer.put((byte)(battery_function & 0xff)); // uint8_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(battery_remaining)); // int8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_BATTERY_STATUS { " + 
			"current_consumed = " + current_consumed + ", " + 
			"energy_consumed = " + energy_consumed + ", " + 
			"temperature = " + temperature + ", " + 
			"voltages = " + voltages + ", " + 
			"current_battery = " + current_battery + ", " + 
			"id = " + id + ", " + 
			"battery_function = " + battery_function + ", " + 
			"type = " + type + ", " + 
			"battery_remaining = " + battery_remaining + ",  }";
		}
	}
	public static class MSG_BRIEF_FEATURE extends Message {
	
		private float x; // x position in m
		private float y; // y position in m
		private float z; // z position in m
		private float response; // Harris operator response at this location
		private int size; // Size in pixels
		private int orientation; // Orientation
		private int orientation_assignment; // Orientation assignment 0: false, 1:true
		private int[] descriptor = new int[32]; // Descriptor
	
		public MSG_BRIEF_FEATURE () {
			super(MSG_ID_BRIEF_FEATURE, (short)22, "BRIEF_FEATURE");
		}

		public MSG_BRIEF_FEATURE (byte[] bytes) {
			super(bytes, MSG_ID_BRIEF_FEATURE, (short)22, "BRIEF_FEATURE");
		}
	
		public MSG_BRIEF_FEATURE (short sys, short comp, float x  , float y  , float z  , float response  , int size  , int orientation  , int orientation_assignment  , int descriptor [] ) {
			super(sys, comp, (short)22, "BRIEF_FEATURE");
			this.x = x;
			this.y = y;
			this.z = z;
			this.response = response;
			this.size = size;
			this.orientation = orientation;
			this.orientation_assignment = orientation_assignment;
			this.descriptor = descriptor;
		}

		@Override
		public int getCRCExtra() {
			return 88;
		}
	
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getResponse() {
			return response;
		}
		
		public void setResponse(float response) {
			this.response = response;
		}
		
		public int getSize() {
			return size;
		}
		
		public void setSize(int size) {
			this.size = size;
		}
		
		public int getOrientation() {
			return orientation;
		}
		
		public void setOrientation(int orientation) {
			this.orientation = orientation;
		}
		
		public int getOrientation_assignment() {
			return orientation_assignment;
		}
		
		public void setOrientation_assignment(int orientation_assignment) {
			this.orientation_assignment = orientation_assignment;
		}
		
		public int[] getDescriptor() {
			return descriptor;
		}
		
		public void setDescriptor(int descriptor[]) {
			this.descriptor = descriptor;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "BRIEF_FEATURE";
			
			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			response = buffer.getFloat(); // float
  			size = buffer.getShort() & 0xffff; // uint16_t
  			orientation = buffer.getShort() & 0xffff; // uint16_t
  			orientation_assignment = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<32; ++c) {
				descriptor [c] =  (int)buffer.get() & 0xff; // uint8_t[32]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(response); // float
  			buffer.putShort((short)(size & 0xffff)); // uint16_t
  			buffer.putShort((short)(orientation & 0xffff)); // uint16_t
  			buffer.put((byte)(orientation_assignment & 0xff)); // uint8_t
  			for(int c=0; c<32; ++c) {
				buffer.put((byte)(descriptor [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_BRIEF_FEATURE { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"response = " + response + ", " + 
			"size = " + size + ", " + 
			"orientation = " + orientation + ", " + 
			"orientation_assignment = " + orientation_assignment + ", " + 
			"descriptor = " + descriptor + ",  }";
		}
	}
	/*
	 * Camera-IMU triggering and synchronisation message.
	 */
	public static class MSG_CAMERA_TRIGGER extends Message {
	
		private long time_usec; // Timestamp for the image frame in microseconds
		private long seq; // Image frame sequence
	
		public MSG_CAMERA_TRIGGER () {
			super(MSG_ID_CAMERA_TRIGGER, (short)68, "CAMERA_TRIGGER");
		}

		public MSG_CAMERA_TRIGGER (byte[] bytes) {
			super(bytes, MSG_ID_CAMERA_TRIGGER, (short)68, "CAMERA_TRIGGER");
		}
	
		public MSG_CAMERA_TRIGGER (short sys, short comp, long time_usec  , long seq  ) {
			super(sys, comp, (short)68, "CAMERA_TRIGGER");
			this.time_usec = time_usec;
			this.seq = seq;
		}

		@Override
		public int getCRCExtra() {
			return 174;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public long getSeq() {
			return seq;
		}
		
		public void setSeq(long seq) {
			this.seq = seq;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "CAMERA_TRIGGER";
			
			time_usec = buffer.getLong(); // uint64_t
  			seq = buffer.getInt() & 0xffffffff; // uint32_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(seq & 0xffffffff)); // uint32_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_CAMERA_TRIGGER { " + 
			"time_usec = " + time_usec + ", " + 
			"seq = " + seq + ",  }";
		}
	}
	/*
	 * Request to control this MAV
	 */
	public static class MSG_CHANGE_OPERATOR_CONTROL extends Message {
	
		private int target_system; // System the GCS requests control for
		private int control_request; // 0: request control of this MAV, 1: Release control of this MAV
		private int version; // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
		private char[] passkey = new char[25]; // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
	
		public MSG_CHANGE_OPERATOR_CONTROL () {
			super(MSG_ID_CHANGE_OPERATOR_CONTROL, (short)4, "CHANGE_OPERATOR_CONTROL");
		}

		public MSG_CHANGE_OPERATOR_CONTROL (byte[] bytes) {
			super(bytes, MSG_ID_CHANGE_OPERATOR_CONTROL, (short)4, "CHANGE_OPERATOR_CONTROL");
		}
	
		public MSG_CHANGE_OPERATOR_CONTROL (short sys, short comp, int target_system  , int control_request  , int version  , char passkey [] ) {
			super(sys, comp, (short)4, "CHANGE_OPERATOR_CONTROL");
			this.target_system = target_system;
			this.control_request = control_request;
			this.version = version;
			this.passkey = passkey;
		}

		@Override
		public int getCRCExtra() {
			return 217;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getControl_request() {
			return control_request;
		}
		
		public void setControl_request(int control_request) {
			this.control_request = control_request;
		}
		
		public int getVersion() {
			return version;
		}
		
		public void setVersion(int version) {
			this.version = version;
		}
		
		public char[] getPasskey() {
			return passkey;
		}
		
		public void setPasskey(char passkey[]) {
			this.passkey = passkey;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "CHANGE_OPERATOR_CONTROL";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			control_request = (int)buffer.get() & 0xff; // uint8_t
  			version = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<25; ++c) {
				passkey [c] =  (char)buffer.get(); // char[25]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(control_request & 0xff)); // uint8_t
  			buffer.put((byte)(version & 0xff)); // uint8_t
  			for(int c=0; c<25; ++c) {
				buffer.put((byte)(passkey [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_CHANGE_OPERATOR_CONTROL { " + 
			"target_system = " + target_system + ", " + 
			"control_request = " + control_request + ", " + 
			"version = " + version + ", " + 
			"passkey = " + passkey + ",  }";
		}
	}
	/*
	 * Accept / deny control of this MAV
	 */
	public static class MSG_CHANGE_OPERATOR_CONTROL_ACK extends Message {
	
		private int gcs_system_id; // ID of the GCS this message 
		private int control_request; // 0: request control of this MAV, 1: Release control of this MAV
		private int ack; // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
	
		public MSG_CHANGE_OPERATOR_CONTROL_ACK () {
			super(MSG_ID_CHANGE_OPERATOR_CONTROL_ACK, (short)3, "CHANGE_OPERATOR_CONTROL_ACK");
		}

		public MSG_CHANGE_OPERATOR_CONTROL_ACK (byte[] bytes) {
			super(bytes, MSG_ID_CHANGE_OPERATOR_CONTROL_ACK, (short)3, "CHANGE_OPERATOR_CONTROL_ACK");
		}
	
		public MSG_CHANGE_OPERATOR_CONTROL_ACK (short sys, short comp, int gcs_system_id  , int control_request  , int ack  ) {
			super(sys, comp, (short)3, "CHANGE_OPERATOR_CONTROL_ACK");
			this.gcs_system_id = gcs_system_id;
			this.control_request = control_request;
			this.ack = ack;
		}

		@Override
		public int getCRCExtra() {
			return 104;
		}
	
		public int getGcs_system_id() {
			return gcs_system_id;
		}
		
		public void setGcs_system_id(int gcs_system_id) {
			this.gcs_system_id = gcs_system_id;
		}
		
		public int getControl_request() {
			return control_request;
		}
		
		public void setControl_request(int control_request) {
			this.control_request = control_request;
		}
		
		public int getAck() {
			return ack;
		}
		
		public void setAck(int ack) {
			this.ack = ack;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "CHANGE_OPERATOR_CONTROL_ACK";
			
			gcs_system_id = (int)buffer.get() & 0xff; // uint8_t
  			control_request = (int)buffer.get() & 0xff; // uint8_t
  			ack = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(gcs_system_id & 0xff)); // uint8_t
  			buffer.put((byte)(control_request & 0xff)); // uint8_t
  			buffer.put((byte)(ack & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_CHANGE_OPERATOR_CONTROL_ACK { " + 
			"gcs_system_id = " + gcs_system_id + ", " + 
			"control_request = " + control_request + ", " + 
			"ack = " + ack + ",  }";
		}
	}
	/*
	 * Report status of a command. Includes feedback wether the command was executed.
	 */
	public static class MSG_COMMAND_ACK extends Message {
	
		private int command; // Command ID, as defined by MAV_CMD enum.
		private int result; // See MAV_RESULT enum
	
		public MSG_COMMAND_ACK () {
			super(MSG_ID_COMMAND_ACK, (short)3, "COMMAND_ACK");
		}

		public MSG_COMMAND_ACK (byte[] bytes) {
			super(bytes, MSG_ID_COMMAND_ACK, (short)3, "COMMAND_ACK");
		}
	
		public MSG_COMMAND_ACK (short sys, short comp, int command  , int result  ) {
			super(sys, comp, (short)3, "COMMAND_ACK");
			this.command = command;
			this.result = result;
		}

		@Override
		public int getCRCExtra() {
			return 143;
		}
	
		public int getCommand() {
			return command;
		}
		
		public void setCommand(int command) {
			this.command = command;
		}
		
		public int getResult() {
			return result;
		}
		
		public void setResult(int result) {
			this.result = result;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "COMMAND_ACK";
			
			command = buffer.getShort() & 0xffff; // uint16_t
  			result = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(command & 0xffff)); // uint16_t
  			buffer.put((byte)(result & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_COMMAND_ACK { " + 
			"command = " + command + ", " + 
			"result = " + result + ",  }";
		}
	}
	/*
	 * Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
	 */
	public static class MSG_COMMAND_INT extends Message {
	
		private float param1; // PARAM1, see MAV_CMD enum
		private float param2; // PARAM2, see MAV_CMD enum
		private float param3; // PARAM3, see MAV_CMD enum
		private float param4; // PARAM4, see MAV_CMD enum
		private int x; // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
		private int y; // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
		private float z; // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
		private int command; // The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
		private int target_system; // System ID
		private int target_component; // Component ID
		private int frame; // The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
		private int current; // false:0, true:1
		private int autocontinue; // autocontinue to next wp
	
		public MSG_COMMAND_INT () {
			super(MSG_ID_COMMAND_INT, (short)35, "COMMAND_INT");
		}

		public MSG_COMMAND_INT (byte[] bytes) {
			super(bytes, MSG_ID_COMMAND_INT, (short)35, "COMMAND_INT");
		}
	
		public MSG_COMMAND_INT (short sys, short comp, float param1  , float param2  , float param3  , float param4  , int x  , int y  , float z  , int command  , int target_system  , int target_component  , int frame  , int current  , int autocontinue  ) {
			super(sys, comp, (short)35, "COMMAND_INT");
			this.param1 = param1;
			this.param2 = param2;
			this.param3 = param3;
			this.param4 = param4;
			this.x = x;
			this.y = y;
			this.z = z;
			this.command = command;
			this.target_system = target_system;
			this.target_component = target_component;
			this.frame = frame;
			this.current = current;
			this.autocontinue = autocontinue;
		}

		@Override
		public int getCRCExtra() {
			return 158;
		}
	
		public float getParam1() {
			return param1;
		}
		
		public void setParam1(float param1) {
			this.param1 = param1;
		}
		
		public float getParam2() {
			return param2;
		}
		
		public void setParam2(float param2) {
			this.param2 = param2;
		}
		
		public float getParam3() {
			return param3;
		}
		
		public void setParam3(float param3) {
			this.param3 = param3;
		}
		
		public float getParam4() {
			return param4;
		}
		
		public void setParam4(float param4) {
			this.param4 = param4;
		}
		
		public int getX() {
			return x;
		}
		
		public void setX(int x) {
			this.x = x;
		}
		
		public int getY() {
			return y;
		}
		
		public void setY(int y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public int getCommand() {
			return command;
		}
		
		public void setCommand(int command) {
			this.command = command;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getFrame() {
			return frame;
		}
		
		public void setFrame(int frame) {
			this.frame = frame;
		}
		
		public int getCurrent() {
			return current;
		}
		
		public void setCurrent(int current) {
			this.current = current;
		}
		
		public int getAutocontinue() {
			return autocontinue;
		}
		
		public void setAutocontinue(int autocontinue) {
			this.autocontinue = autocontinue;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "COMMAND_INT";
			
			param1 = buffer.getFloat(); // float
  			param2 = buffer.getFloat(); // float
  			param3 = buffer.getFloat(); // float
  			param4 = buffer.getFloat(); // float
  			x = buffer.getInt(); // int32_t
  			y = buffer.getInt(); // int32_t
  			z = buffer.getFloat(); // float
  			command = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			frame = (int)buffer.get() & 0xff; // uint8_t
  			current = (int)buffer.get() & 0xff; // uint8_t
  			autocontinue = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param1); // float
  			buffer.putFloat(param2); // float
  			buffer.putFloat(param3); // float
  			buffer.putFloat(param4); // float
  			buffer.putInt((int)(x)); // int32_t
  			buffer.putInt((int)(y)); // int32_t
  			buffer.putFloat(z); // float
  			buffer.putShort((short)(command & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(frame & 0xff)); // uint8_t
  			buffer.put((byte)(current & 0xff)); // uint8_t
  			buffer.put((byte)(autocontinue & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_COMMAND_INT { " + 
			"param1 = " + param1 + ", " + 
			"param2 = " + param2 + ", " + 
			"param3 = " + param3 + ", " + 
			"param4 = " + param4 + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"command = " + command + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"frame = " + frame + ", " + 
			"current = " + current + ", " + 
			"autocontinue = " + autocontinue + ",  }";
		}
	}
	/*
	 * Send a command with up to seven parameters to the MAV
	 */
	public static class MSG_COMMAND_LONG extends Message {
	
		private float param1; // Parameter 1, as defined by MAV_CMD enum.
		private float param2; // Parameter 2, as defined by MAV_CMD enum.
		private float param3; // Parameter 3, as defined by MAV_CMD enum.
		private float param4; // Parameter 4, as defined by MAV_CMD enum.
		private float param5; // Parameter 5, as defined by MAV_CMD enum.
		private float param6; // Parameter 6, as defined by MAV_CMD enum.
		private float param7; // Parameter 7, as defined by MAV_CMD enum.
		private int command; // Command ID, as defined by MAV_CMD enum.
		private int target_system; // System which should execute the command
		private int target_component; // Component which should execute the command, 0 for all components
		private int confirmation; // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	
		public MSG_COMMAND_LONG () {
			super(MSG_ID_COMMAND_LONG, (short)33, "COMMAND_LONG");
		}

		public MSG_COMMAND_LONG (byte[] bytes) {
			super(bytes, MSG_ID_COMMAND_LONG, (short)33, "COMMAND_LONG");
		}
	
		public MSG_COMMAND_LONG (short sys, short comp, float param1  , float param2  , float param3  , float param4  , float param5  , float param6  , float param7  , int command  , int target_system  , int target_component  , int confirmation  ) {
			super(sys, comp, (short)33, "COMMAND_LONG");
			this.param1 = param1;
			this.param2 = param2;
			this.param3 = param3;
			this.param4 = param4;
			this.param5 = param5;
			this.param6 = param6;
			this.param7 = param7;
			this.command = command;
			this.target_system = target_system;
			this.target_component = target_component;
			this.confirmation = confirmation;
		}

		@Override
		public int getCRCExtra() {
			return 152;
		}
	
		public float getParam1() {
			return param1;
		}
		
		public void setParam1(float param1) {
			this.param1 = param1;
		}
		
		public float getParam2() {
			return param2;
		}
		
		public void setParam2(float param2) {
			this.param2 = param2;
		}
		
		public float getParam3() {
			return param3;
		}
		
		public void setParam3(float param3) {
			this.param3 = param3;
		}
		
		public float getParam4() {
			return param4;
		}
		
		public void setParam4(float param4) {
			this.param4 = param4;
		}
		
		public float getParam5() {
			return param5;
		}
		
		public void setParam5(float param5) {
			this.param5 = param5;
		}
		
		public float getParam6() {
			return param6;
		}
		
		public void setParam6(float param6) {
			this.param6 = param6;
		}
		
		public float getParam7() {
			return param7;
		}
		
		public void setParam7(float param7) {
			this.param7 = param7;
		}
		
		public int getCommand() {
			return command;
		}
		
		public void setCommand(int command) {
			this.command = command;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getConfirmation() {
			return confirmation;
		}
		
		public void setConfirmation(int confirmation) {
			this.confirmation = confirmation;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "COMMAND_LONG";
			
			param1 = buffer.getFloat(); // float
  			param2 = buffer.getFloat(); // float
  			param3 = buffer.getFloat(); // float
  			param4 = buffer.getFloat(); // float
  			param5 = buffer.getFloat(); // float
  			param6 = buffer.getFloat(); // float
  			param7 = buffer.getFloat(); // float
  			command = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			confirmation = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param1); // float
  			buffer.putFloat(param2); // float
  			buffer.putFloat(param3); // float
  			buffer.putFloat(param4); // float
  			buffer.putFloat(param5); // float
  			buffer.putFloat(param6); // float
  			buffer.putFloat(param7); // float
  			buffer.putShort((short)(command & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(confirmation & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_COMMAND_LONG { " + 
			"param1 = " + param1 + ", " + 
			"param2 = " + param2 + ", " + 
			"param3 = " + param3 + ", " + 
			"param4 = " + param4 + ", " + 
			"param5 = " + param5 + ", " + 
			"param6 = " + param6 + ", " + 
			"param7 = " + param7 + ", " + 
			"command = " + command + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"confirmation = " + confirmation + ",  }";
		}
	}
	public static class MSG_DATA_STREAM extends Message {
	
		private int message_rate; // The requested interval between two messages of this type
		private int stream_id; // The ID of the requested data stream
		private int on_off; // 1 stream is enabled, 0 stream is stopped.
	
		public MSG_DATA_STREAM () {
			super(MSG_ID_DATA_STREAM, (short)4, "DATA_STREAM");
		}

		public MSG_DATA_STREAM (byte[] bytes) {
			super(bytes, MSG_ID_DATA_STREAM, (short)4, "DATA_STREAM");
		}
	
		public MSG_DATA_STREAM (short sys, short comp, int message_rate  , int stream_id  , int on_off  ) {
			super(sys, comp, (short)4, "DATA_STREAM");
			this.message_rate = message_rate;
			this.stream_id = stream_id;
			this.on_off = on_off;
		}

		@Override
		public int getCRCExtra() {
			return 21;
		}
	
		public int getMessage_rate() {
			return message_rate;
		}
		
		public void setMessage_rate(int message_rate) {
			this.message_rate = message_rate;
		}
		
		public int getStream_id() {
			return stream_id;
		}
		
		public void setStream_id(int stream_id) {
			this.stream_id = stream_id;
		}
		
		public int getOn_off() {
			return on_off;
		}
		
		public void setOn_off(int on_off) {
			this.on_off = on_off;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DATA_STREAM";
			
			message_rate = buffer.getShort() & 0xffff; // uint16_t
  			stream_id = (int)buffer.get() & 0xff; // uint8_t
  			on_off = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(message_rate & 0xffff)); // uint16_t
  			buffer.put((byte)(stream_id & 0xff)); // uint8_t
  			buffer.put((byte)(on_off & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DATA_STREAM { " + 
			"message_rate = " + message_rate + ", " + 
			"stream_id = " + stream_id + ", " + 
			"on_off = " + on_off + ",  }";
		}
	}
	public static class MSG_DATA_TRANSMISSION_HANDSHAKE extends Message {
	
		private long size; // total data size in bytes (set on ACK only)
		private int width; // Width of a matrix or image
		private int height; // Height of a matrix or image
		private int packets; // number of packets beeing sent (set on ACK only)
		private int type; // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
		private int payload; // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
		private int jpg_quality; // JPEG quality out of [1,100]
	
		public MSG_DATA_TRANSMISSION_HANDSHAKE () {
			super(MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (short)13, "DATA_TRANSMISSION_HANDSHAKE");
		}

		public MSG_DATA_TRANSMISSION_HANDSHAKE (byte[] bytes) {
			super(bytes, MSG_ID_DATA_TRANSMISSION_HANDSHAKE, (short)13, "DATA_TRANSMISSION_HANDSHAKE");
		}
	
		public MSG_DATA_TRANSMISSION_HANDSHAKE (short sys, short comp, long size  , int width  , int height  , int packets  , int type  , int payload  , int jpg_quality  ) {
			super(sys, comp, (short)13, "DATA_TRANSMISSION_HANDSHAKE");
			this.size = size;
			this.width = width;
			this.height = height;
			this.packets = packets;
			this.type = type;
			this.payload = payload;
			this.jpg_quality = jpg_quality;
		}

		@Override
		public int getCRCExtra() {
			return 29;
		}
	
		public long getSize() {
			return size;
		}
		
		public void setSize(long size) {
			this.size = size;
		}
		
		public int getWidth() {
			return width;
		}
		
		public void setWidth(int width) {
			this.width = width;
		}
		
		public int getHeight() {
			return height;
		}
		
		public void setHeight(int height) {
			this.height = height;
		}
		
		public int getPackets() {
			return packets;
		}
		
		public void setPackets(int packets) {
			this.packets = packets;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getPayload() {
			return payload;
		}
		
		public void setPayload(int payload) {
			this.payload = payload;
		}
		
		public int getJpg_quality() {
			return jpg_quality;
		}
		
		public void setJpg_quality(int jpg_quality) {
			this.jpg_quality = jpg_quality;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DATA_TRANSMISSION_HANDSHAKE";
			
			size = buffer.getInt() & 0xffffffff; // uint32_t
  			width = buffer.getShort() & 0xffff; // uint16_t
  			height = buffer.getShort() & 0xffff; // uint16_t
  			packets = buffer.getShort() & 0xffff; // uint16_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			payload = (int)buffer.get() & 0xff; // uint8_t
  			jpg_quality = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(size & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(width & 0xffff)); // uint16_t
  			buffer.putShort((short)(height & 0xffff)); // uint16_t
  			buffer.putShort((short)(packets & 0xffff)); // uint16_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(payload & 0xff)); // uint8_t
  			buffer.put((byte)(jpg_quality & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DATA_TRANSMISSION_HANDSHAKE { " + 
			"size = " + size + ", " + 
			"width = " + width + ", " + 
			"height = " + height + ", " + 
			"packets = " + packets + ", " + 
			"type = " + type + ", " + 
			"payload = " + payload + ", " + 
			"jpg_quality = " + jpg_quality + ",  }";
		}
	}
	/*
	 * Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
	 */
	public static class MSG_DEBUG extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float value; // DEBUG value
		private int ind; // index of debug variable
	
		public MSG_DEBUG () {
			super(MSG_ID_DEBUG, (short)9, "DEBUG");
		}

		public MSG_DEBUG (byte[] bytes) {
			super(bytes, MSG_ID_DEBUG, (short)9, "DEBUG");
		}
	
		public MSG_DEBUG (short sys, short comp, long time_boot_ms  , float value  , int ind  ) {
			super(sys, comp, (short)9, "DEBUG");
			this.time_boot_ms = time_boot_ms;
			this.value = value;
			this.ind = ind;
		}

		@Override
		public int getCRCExtra() {
			return 46;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getValue() {
			return value;
		}
		
		public void setValue(float value) {
			this.value = value;
		}
		
		public int getInd() {
			return ind;
		}
		
		public void setInd(int ind) {
			this.ind = ind;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DEBUG";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			value = buffer.getFloat(); // float
  			ind = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(value); // float
  			buffer.put((byte)(ind & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DEBUG { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"value = " + value + ", " + 
			"ind = " + ind + ",  }";
		}
	}
	public static class MSG_DEBUG_VECT extends Message {
	
		private long time_usec; // Timestamp
		private float x; // x
		private float y; // y
		private float z; // z
		private char[] name = new char[10]; // Name
	
		public MSG_DEBUG_VECT () {
			super(MSG_ID_DEBUG_VECT, (short)77, "DEBUG_VECT");
		}

		public MSG_DEBUG_VECT (byte[] bytes) {
			super(bytes, MSG_ID_DEBUG_VECT, (short)77, "DEBUG_VECT");
		}
	
		public MSG_DEBUG_VECT (short sys, short comp, long time_usec  , float x  , float y  , float z  , char name [] ) {
			super(sys, comp, (short)77, "DEBUG_VECT");
			this.time_usec = time_usec;
			this.x = x;
			this.y = y;
			this.z = z;
			this.name = name;
		}

		@Override
		public int getCRCExtra() {
			return 49;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DEBUG_VECT";
			
			time_usec = buffer.getLong(); // uint64_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			for(int c=0; c<10; ++c) {
				name [c] =  (char)buffer.get(); // char[10]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			for(int c=0; c<10; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DEBUG_VECT { " + 
			"time_usec = " + time_usec + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"name = " + name + ",  }";
		}
	}
	public static class MSG_DETECTION_STATS extends Message {
	
		private long detections; // Number of detections
		private long cluster_iters; // Number of cluster iterations
		private float best_score; // Best score
		private int best_lat; // Latitude of the best detection * 1E7
		private int best_lon; // Longitude of the best detection * 1E7
		private int best_alt; // Altitude of the best detection * 1E3
		private long best_detection_id; // Best detection ID
		private long best_cluster_id; // Best cluster ID
		private long best_cluster_iter_id; // Best cluster ID
		private long images_done; // Number of images already processed
		private long images_todo; // Number of images still to process
		private float fps; // Average images per seconds processed
	
		public MSG_DETECTION_STATS () {
			super(MSG_ID_DETECTION_STATS, (short)48, "DETECTION_STATS");
		}

		public MSG_DETECTION_STATS (byte[] bytes) {
			super(bytes, MSG_ID_DETECTION_STATS, (short)48, "DETECTION_STATS");
		}
	
		public MSG_DETECTION_STATS (short sys, short comp, long detections  , long cluster_iters  , float best_score  , int best_lat  , int best_lon  , int best_alt  , long best_detection_id  , long best_cluster_id  , long best_cluster_iter_id  , long images_done  , long images_todo  , float fps  ) {
			super(sys, comp, (short)48, "DETECTION_STATS");
			this.detections = detections;
			this.cluster_iters = cluster_iters;
			this.best_score = best_score;
			this.best_lat = best_lat;
			this.best_lon = best_lon;
			this.best_alt = best_alt;
			this.best_detection_id = best_detection_id;
			this.best_cluster_id = best_cluster_id;
			this.best_cluster_iter_id = best_cluster_iter_id;
			this.images_done = images_done;
			this.images_todo = images_todo;
			this.fps = fps;
		}

		@Override
		public int getCRCExtra() {
			return 87;
		}
	
		public long getDetections() {
			return detections;
		}
		
		public void setDetections(long detections) {
			this.detections = detections;
		}
		
		public long getCluster_iters() {
			return cluster_iters;
		}
		
		public void setCluster_iters(long cluster_iters) {
			this.cluster_iters = cluster_iters;
		}
		
		public float getBest_score() {
			return best_score;
		}
		
		public void setBest_score(float best_score) {
			this.best_score = best_score;
		}
		
		public int getBest_lat() {
			return best_lat;
		}
		
		public void setBest_lat(int best_lat) {
			this.best_lat = best_lat;
		}
		
		public int getBest_lon() {
			return best_lon;
		}
		
		public void setBest_lon(int best_lon) {
			this.best_lon = best_lon;
		}
		
		public int getBest_alt() {
			return best_alt;
		}
		
		public void setBest_alt(int best_alt) {
			this.best_alt = best_alt;
		}
		
		public long getBest_detection_id() {
			return best_detection_id;
		}
		
		public void setBest_detection_id(long best_detection_id) {
			this.best_detection_id = best_detection_id;
		}
		
		public long getBest_cluster_id() {
			return best_cluster_id;
		}
		
		public void setBest_cluster_id(long best_cluster_id) {
			this.best_cluster_id = best_cluster_id;
		}
		
		public long getBest_cluster_iter_id() {
			return best_cluster_iter_id;
		}
		
		public void setBest_cluster_iter_id(long best_cluster_iter_id) {
			this.best_cluster_iter_id = best_cluster_iter_id;
		}
		
		public long getImages_done() {
			return images_done;
		}
		
		public void setImages_done(long images_done) {
			this.images_done = images_done;
		}
		
		public long getImages_todo() {
			return images_todo;
		}
		
		public void setImages_todo(long images_todo) {
			this.images_todo = images_todo;
		}
		
		public float getFps() {
			return fps;
		}
		
		public void setFps(float fps) {
			this.fps = fps;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DETECTION_STATS";
			
			detections = buffer.getInt() & 0xffffffff; // uint32_t
  			cluster_iters = buffer.getInt() & 0xffffffff; // uint32_t
  			best_score = buffer.getFloat(); // float
  			best_lat = buffer.getInt(); // int32_t
  			best_lon = buffer.getInt(); // int32_t
  			best_alt = buffer.getInt(); // int32_t
  			best_detection_id = buffer.getInt() & 0xffffffff; // uint32_t
  			best_cluster_id = buffer.getInt() & 0xffffffff; // uint32_t
  			best_cluster_iter_id = buffer.getInt() & 0xffffffff; // uint32_t
  			images_done = buffer.getInt() & 0xffffffff; // uint32_t
  			images_todo = buffer.getInt() & 0xffffffff; // uint32_t
  			fps = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(detections & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(cluster_iters & 0xffffffff)); // uint32_t
  			buffer.putFloat(best_score); // float
  			buffer.putInt((int)(best_lat)); // int32_t
  			buffer.putInt((int)(best_lon)); // int32_t
  			buffer.putInt((int)(best_alt)); // int32_t
  			buffer.putInt((int)(best_detection_id & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(best_cluster_id & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(best_cluster_iter_id & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(images_done & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(images_todo & 0xffffffff)); // uint32_t
  			buffer.putFloat(fps); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DETECTION_STATS { " + 
			"detections = " + detections + ", " + 
			"cluster_iters = " + cluster_iters + ", " + 
			"best_score = " + best_score + ", " + 
			"best_lat = " + best_lat + ", " + 
			"best_lon = " + best_lon + ", " + 
			"best_alt = " + best_alt + ", " + 
			"best_detection_id = " + best_detection_id + ", " + 
			"best_cluster_id = " + best_cluster_id + ", " + 
			"best_cluster_iter_id = " + best_cluster_iter_id + ", " + 
			"images_done = " + images_done + ", " + 
			"images_todo = " + images_todo + ", " + 
			"fps = " + fps + ",  }";
		}
	}
	public static class MSG_DISTANCE_SENSOR extends Message {
	
		private long time_boot_ms; // Time since system boot
		private int min_distance; // Minimum distance the sensor can measure in centimeters
		private int max_distance; // Maximum distance the sensor can measure in centimeters
		private int current_distance; // Current distance reading
		private int type; // Type from MAV_DISTANCE_SENSOR enum.
		private int id; // Onboard ID of the sensor
		private int orientation; // Direction the sensor faces from FIXME enum.
		private int covariance; // Measurement covariance in centimeters, 0 for unknown / invalid readings
	
		public MSG_DISTANCE_SENSOR () {
			super(MSG_ID_DISTANCE_SENSOR, (short)14, "DISTANCE_SENSOR");
		}

		public MSG_DISTANCE_SENSOR (byte[] bytes) {
			super(bytes, MSG_ID_DISTANCE_SENSOR, (short)14, "DISTANCE_SENSOR");
		}
	
		public MSG_DISTANCE_SENSOR (short sys, short comp, long time_boot_ms  , int min_distance  , int max_distance  , int current_distance  , int type  , int id  , int orientation  , int covariance  ) {
			super(sys, comp, (short)14, "DISTANCE_SENSOR");
			this.time_boot_ms = time_boot_ms;
			this.min_distance = min_distance;
			this.max_distance = max_distance;
			this.current_distance = current_distance;
			this.type = type;
			this.id = id;
			this.orientation = orientation;
			this.covariance = covariance;
		}

		@Override
		public int getCRCExtra() {
			return 85;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getMin_distance() {
			return min_distance;
		}
		
		public void setMin_distance(int min_distance) {
			this.min_distance = min_distance;
		}
		
		public int getMax_distance() {
			return max_distance;
		}
		
		public void setMax_distance(int max_distance) {
			this.max_distance = max_distance;
		}
		
		public int getCurrent_distance() {
			return current_distance;
		}
		
		public void setCurrent_distance(int current_distance) {
			this.current_distance = current_distance;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
		public int getOrientation() {
			return orientation;
		}
		
		public void setOrientation(int orientation) {
			this.orientation = orientation;
		}
		
		public int getCovariance() {
			return covariance;
		}
		
		public void setCovariance(int covariance) {
			this.covariance = covariance;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "DISTANCE_SENSOR";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			min_distance = buffer.getShort() & 0xffff; // uint16_t
  			max_distance = buffer.getShort() & 0xffff; // uint16_t
  			current_distance = buffer.getShort() & 0xffff; // uint16_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			id = (int)buffer.get() & 0xff; // uint8_t
  			orientation = (int)buffer.get() & 0xff; // uint8_t
  			covariance = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(min_distance & 0xffff)); // uint16_t
  			buffer.putShort((short)(max_distance & 0xffff)); // uint16_t
  			buffer.putShort((short)(current_distance & 0xffff)); // uint16_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(id & 0xff)); // uint8_t
  			buffer.put((byte)(orientation & 0xff)); // uint8_t
  			buffer.put((byte)(covariance & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_DISTANCE_SENSOR { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"min_distance = " + min_distance + ", " + 
			"max_distance = " + max_distance + ", " + 
			"current_distance = " + current_distance + ", " + 
			"type = " + type + ", " + 
			"id = " + id + ", " + 
			"orientation = " + orientation + ", " + 
			"covariance = " + covariance + ",  }";
		}
	}
	public static class MSG_ENCAPSULATED_DATA extends Message {
	
		private int seqnr; // sequence number (starting with 0 on every transmission)
		private int[] data = new int[253]; // image data bytes
	
		public MSG_ENCAPSULATED_DATA () {
			super(MSG_ID_ENCAPSULATED_DATA, (short)3, "ENCAPSULATED_DATA");
		}

		public MSG_ENCAPSULATED_DATA (byte[] bytes) {
			super(bytes, MSG_ID_ENCAPSULATED_DATA, (short)3, "ENCAPSULATED_DATA");
		}
	
		public MSG_ENCAPSULATED_DATA (short sys, short comp, int seqnr  , int data [] ) {
			super(sys, comp, (short)3, "ENCAPSULATED_DATA");
			this.seqnr = seqnr;
			this.data = data;
		}

		@Override
		public int getCRCExtra() {
			return 172;
		}
	
		public int getSeqnr() {
			return seqnr;
		}
		
		public void setSeqnr(int seqnr) {
			this.seqnr = seqnr;
		}
		
		public int[] getData() {
			return data;
		}
		
		public void setData(int data[]) {
			this.data = data;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ENCAPSULATED_DATA";
			
			seqnr = buffer.getShort() & 0xffff; // uint16_t
  			for(int c=0; c<253; ++c) {
				data [c] =  (int)buffer.get() & 0xff; // uint8_t[253]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(seqnr & 0xffff)); // uint16_t
  			for(int c=0; c<253; ++c) {
				buffer.put((byte)(data [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ENCAPSULATED_DATA { " + 
			"seqnr = " + seqnr + ", " + 
			"data = " + data + ",  }";
		}
	}
	/*
	 * File transfer message
	 */
	public static class MSG_FILE_TRANSFER_PROTOCOL extends Message {
	
		private int target_network; // Network ID (0 for broadcast)
		private int target_system; // System ID (0 for broadcast)
		private int target_component; // Component ID (0 for broadcast)
		private int[] payload = new int[251]; // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
	
		public MSG_FILE_TRANSFER_PROTOCOL () {
			super(MSG_ID_FILE_TRANSFER_PROTOCOL, (short)4, "FILE_TRANSFER_PROTOCOL");
		}

		public MSG_FILE_TRANSFER_PROTOCOL (byte[] bytes) {
			super(bytes, MSG_ID_FILE_TRANSFER_PROTOCOL, (short)4, "FILE_TRANSFER_PROTOCOL");
		}
	
		public MSG_FILE_TRANSFER_PROTOCOL (short sys, short comp, int target_network  , int target_system  , int target_component  , int payload [] ) {
			super(sys, comp, (short)4, "FILE_TRANSFER_PROTOCOL");
			this.target_network = target_network;
			this.target_system = target_system;
			this.target_component = target_component;
			this.payload = payload;
		}

		@Override
		public int getCRCExtra() {
			return 23;
		}
	
		public int getTarget_network() {
			return target_network;
		}
		
		public void setTarget_network(int target_network) {
			this.target_network = target_network;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int[] getPayload() {
			return payload;
		}
		
		public void setPayload(int payload[]) {
			this.payload = payload;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "FILE_TRANSFER_PROTOCOL";
			
			target_network = (int)buffer.get() & 0xff; // uint8_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<251; ++c) {
				payload [c] =  (int)buffer.get() & 0xff; // uint8_t[251]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_network & 0xff)); // uint8_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			for(int c=0; c<251; ++c) {
				buffer.put((byte)(payload [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_FILE_TRANSFER_PROTOCOL { " + 
			"target_network = " + target_network + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"payload = " + payload + ",  }";
		}
	}
	/*
	 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
               is designed as scaled integer message since the resolution of float is not sufficient.
	 */
	public static class MSG_GLOBAL_POSITION_INT extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int lat; // Latitude, expressed as * 1E7
		private int lon; // Longitude, expressed as * 1E7
		private int alt; // Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
		private int relative_alt; // Altitude above ground in meters, expressed as * 1000 (millimeters)
		private int vx; // Ground X Speed (Latitude), expressed as m/s * 100
		private int vy; // Ground Y Speed (Longitude), expressed as m/s * 100
		private int vz; // Ground Z Speed (Altitude), expressed as m/s * 100
		private int hdg; // Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	
		public MSG_GLOBAL_POSITION_INT () {
			super(MSG_ID_GLOBAL_POSITION_INT, (short)28, "GLOBAL_POSITION_INT");
		}

		public MSG_GLOBAL_POSITION_INT (byte[] bytes) {
			super(bytes, MSG_ID_GLOBAL_POSITION_INT, (short)28, "GLOBAL_POSITION_INT");
		}
	
		public MSG_GLOBAL_POSITION_INT (short sys, short comp, long time_boot_ms  , int lat  , int lon  , int alt  , int relative_alt  , int vx  , int vy  , int vz  , int hdg  ) {
			super(sys, comp, (short)28, "GLOBAL_POSITION_INT");
			this.time_boot_ms = time_boot_ms;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.relative_alt = relative_alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.hdg = hdg;
		}

		@Override
		public int getCRCExtra() {
			return 104;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getRelative_alt() {
			return relative_alt;
		}
		
		public void setRelative_alt(int relative_alt) {
			this.relative_alt = relative_alt;
		}
		
		public int getVx() {
			return vx;
		}
		
		public void setVx(int vx) {
			this.vx = vx;
		}
		
		public int getVy() {
			return vy;
		}
		
		public void setVy(int vy) {
			this.vy = vy;
		}
		
		public int getVz() {
			return vz;
		}
		
		public void setVz(int vz) {
			this.vz = vz;
		}
		
		public int getHdg() {
			return hdg;
		}
		
		public void setHdg(int hdg) {
			this.hdg = hdg;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GLOBAL_POSITION_INT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			relative_alt = buffer.getInt(); // int32_t
  			vx = buffer.getShort(); // int16_t
  			vy = buffer.getShort(); // int16_t
  			vz = buffer.getShort(); // int16_t
  			hdg = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putInt((int)(relative_alt)); // int32_t
  			buffer.putShort((short)(vx)); // int16_t
  			buffer.putShort((short)(vy)); // int16_t
  			buffer.putShort((short)(vz)); // int16_t
  			buffer.putShort((short)(hdg & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GLOBAL_POSITION_INT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"relative_alt = " + relative_alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"hdg = " + hdg + ",  }";
		}
	}
	/*
	 * The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
	 */
	public static class MSG_GLOBAL_POSITION_INT_COV extends Message {
	
		private long time_utc; // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int lat; // Latitude, expressed as degrees * 1E7
		private int lon; // Longitude, expressed as degrees * 1E7
		private int alt; // Altitude in meters, expressed as * 1000 (millimeters), above MSL
		private int relative_alt; // Altitude above ground in meters, expressed as * 1000 (millimeters)
		private float vx; // Ground X Speed (Latitude), expressed as m/s
		private float vy; // Ground Y Speed (Longitude), expressed as m/s
		private float vz; // Ground Z Speed (Altitude), expressed as m/s
		private float[] covariance = new float[36]; // Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
		private int estimator_type; // Class id of the estimator this estimate originated from.
	
		public MSG_GLOBAL_POSITION_INT_COV () {
			super(MSG_ID_GLOBAL_POSITION_INT_COV, (short)101, "GLOBAL_POSITION_INT_COV");
		}

		public MSG_GLOBAL_POSITION_INT_COV (byte[] bytes) {
			super(bytes, MSG_ID_GLOBAL_POSITION_INT_COV, (short)101, "GLOBAL_POSITION_INT_COV");
		}
	
		public MSG_GLOBAL_POSITION_INT_COV (short sys, short comp, long time_utc  , long time_boot_ms  , int lat  , int lon  , int alt  , int relative_alt  , float vx  , float vy  , float vz  , float covariance [] , int estimator_type  ) {
			super(sys, comp, (short)101, "GLOBAL_POSITION_INT_COV");
			this.time_utc = time_utc;
			this.time_boot_ms = time_boot_ms;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.relative_alt = relative_alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.covariance = covariance;
			this.estimator_type = estimator_type;
		}

		@Override
		public int getCRCExtra() {
			return 51;
		}
	
		public long getTime_utc() {
			return time_utc;
		}
		
		public void setTime_utc(long time_utc) {
			this.time_utc = time_utc;
		}
		
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getRelative_alt() {
			return relative_alt;
		}
		
		public void setRelative_alt(int relative_alt) {
			this.relative_alt = relative_alt;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float[] getCovariance() {
			return covariance;
		}
		
		public void setCovariance(float covariance[]) {
			this.covariance = covariance;
		}
		
		public int getEstimator_type() {
			return estimator_type;
		}
		
		public void setEstimator_type(int estimator_type) {
			this.estimator_type = estimator_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GLOBAL_POSITION_INT_COV";
			
			time_utc = buffer.getLong(); // uint64_t
  			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			relative_alt = buffer.getInt(); // int32_t
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			for(int c=0; c<36; ++c) {
				covariance [c] = buffer.getFloat(); // float[36]
 			}
			
 			estimator_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_utc); // uint64_t
  			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putInt((int)(relative_alt)); // int32_t
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			for(int c=0; c<36; ++c) {
				buffer.putFloat(covariance [c]); // float[36]
 			}
			
 			buffer.put((byte)(estimator_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GLOBAL_POSITION_INT_COV { " + 
			"time_utc = " + time_utc + ", " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"relative_alt = " + relative_alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"covariance = " + covariance + ", " + 
			"estimator_type = " + estimator_type + ",  }";
		}
	}
	public static class MSG_GLOBAL_VISION_POSITION_ESTIMATE extends Message {
	
		private long usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float x; // Global X position
		private float y; // Global Y position
		private float z; // Global Z position
		private float roll; // Roll angle in rad
		private float pitch; // Pitch angle in rad
		private float yaw; // Yaw angle in rad
	
		public MSG_GLOBAL_VISION_POSITION_ESTIMATE () {
			super(MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE, (short)88, "GLOBAL_VISION_POSITION_ESTIMATE");
		}

		public MSG_GLOBAL_VISION_POSITION_ESTIMATE (byte[] bytes) {
			super(bytes, MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE, (short)88, "GLOBAL_VISION_POSITION_ESTIMATE");
		}
	
		public MSG_GLOBAL_VISION_POSITION_ESTIMATE (short sys, short comp, long usec  , float x  , float y  , float z  , float roll  , float pitch  , float yaw  ) {
			super(sys, comp, (short)88, "GLOBAL_VISION_POSITION_ESTIMATE");
			this.usec = usec;
			this.x = x;
			this.y = y;
			this.z = z;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
		}

		@Override
		public int getCRCExtra() {
			return 102;
		}
	
		public long getUsec() {
			return usec;
		}
		
		public void setUsec(long usec) {
			this.usec = usec;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GLOBAL_VISION_POSITION_ESTIMATE";
			
			usec = buffer.getLong(); // uint64_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(usec); // uint64_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GLOBAL_VISION_POSITION_ESTIMATE { " + 
			"usec = " + usec + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ",  }";
		}
	}
	/*
	 * Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
	 */
	public static class MSG_GPS2_RAW extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int lat; // Latitude (WGS84), in degrees * 1E7
		private int lon; // Longitude (WGS84), in degrees * 1E7
		private int alt; // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
		private long dgps_age; // Age of DGPS info
		private int eph; // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
		private int epv; // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
		private int vel; // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
		private int cog; // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
		private int fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS fix, 5: RTK Fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		private int satellites_visible; // Number of satellites visible. If unknown, set to 255
		private int dgps_numch; // Number of DGPS satellites
	
		public MSG_GPS2_RAW () {
			super(MSG_ID_GPS2_RAW, (short)91, "GPS2_RAW");
		}

		public MSG_GPS2_RAW (byte[] bytes) {
			super(bytes, MSG_ID_GPS2_RAW, (short)91, "GPS2_RAW");
		}
	
		public MSG_GPS2_RAW (short sys, short comp, long time_usec  , int lat  , int lon  , int alt  , long dgps_age  , int eph  , int epv  , int vel  , int cog  , int fix_type  , int satellites_visible  , int dgps_numch  ) {
			super(sys, comp, (short)91, "GPS2_RAW");
			this.time_usec = time_usec;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.dgps_age = dgps_age;
			this.eph = eph;
			this.epv = epv;
			this.vel = vel;
			this.cog = cog;
			this.fix_type = fix_type;
			this.satellites_visible = satellites_visible;
			this.dgps_numch = dgps_numch;
		}

		@Override
		public int getCRCExtra() {
			return 87;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public long getDgps_age() {
			return dgps_age;
		}
		
		public void setDgps_age(long dgps_age) {
			this.dgps_age = dgps_age;
		}
		
		public int getEph() {
			return eph;
		}
		
		public void setEph(int eph) {
			this.eph = eph;
		}
		
		public int getEpv() {
			return epv;
		}
		
		public void setEpv(int epv) {
			this.epv = epv;
		}
		
		public int getVel() {
			return vel;
		}
		
		public void setVel(int vel) {
			this.vel = vel;
		}
		
		public int getCog() {
			return cog;
		}
		
		public void setCog(int cog) {
			this.cog = cog;
		}
		
		public int getFix_type() {
			return fix_type;
		}
		
		public void setFix_type(int fix_type) {
			this.fix_type = fix_type;
		}
		
		public int getSatellites_visible() {
			return satellites_visible;
		}
		
		public void setSatellites_visible(int satellites_visible) {
			this.satellites_visible = satellites_visible;
		}
		
		public int getDgps_numch() {
			return dgps_numch;
		}
		
		public void setDgps_numch(int dgps_numch) {
			this.dgps_numch = dgps_numch;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS2_RAW";
			
			time_usec = buffer.getLong(); // uint64_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			dgps_age = buffer.getInt() & 0xffffffff; // uint32_t
  			eph = buffer.getShort() & 0xffff; // uint16_t
  			epv = buffer.getShort() & 0xffff; // uint16_t
  			vel = buffer.getShort() & 0xffff; // uint16_t
  			cog = buffer.getShort() & 0xffff; // uint16_t
  			fix_type = (int)buffer.get() & 0xff; // uint8_t
  			satellites_visible = (int)buffer.get() & 0xff; // uint8_t
  			dgps_numch = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putInt((int)(dgps_age & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(eph & 0xffff)); // uint16_t
  			buffer.putShort((short)(epv & 0xffff)); // uint16_t
  			buffer.putShort((short)(vel & 0xffff)); // uint16_t
  			buffer.putShort((short)(cog & 0xffff)); // uint16_t
  			buffer.put((byte)(fix_type & 0xff)); // uint8_t
  			buffer.put((byte)(satellites_visible & 0xff)); // uint8_t
  			buffer.put((byte)(dgps_numch & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS2_RAW { " + 
			"time_usec = " + time_usec + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"dgps_age = " + dgps_age + ", " + 
			"eph = " + eph + ", " + 
			"epv = " + epv + ", " + 
			"vel = " + vel + ", " + 
			"cog = " + cog + ", " + 
			"fix_type = " + fix_type + ", " + 
			"satellites_visible = " + satellites_visible + ", " + 
			"dgps_numch = " + dgps_numch + ",  }";
		}
	}
	/*
	 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
	 */
	public static class MSG_GPS2_RTK extends Message {
	
		private long time_last_baseline_ms; // Time since boot of last baseline message received in ms.
		private long tow; // GPS Time of Week of last baseline
		private int baseline_a_mm; // Current baseline in ECEF x or NED north component in mm.
		private int baseline_b_mm; // Current baseline in ECEF y or NED east component in mm.
		private int baseline_c_mm; // Current baseline in ECEF z or NED down component in mm.
		private long accuracy; // Current estimate of baseline accuracy.
		private int iar_num_hypotheses; // Current number of integer ambiguity hypotheses.
		private int wn; // GPS Week Number of last baseline
		private int rtk_receiver_id; // Identification of connected RTK receiver.
		private int rtk_health; // GPS-specific health report for RTK data.
		private int rtk_rate; // Rate of baseline messages being received by GPS, in HZ
		private int nsats; // Current number of sats used for RTK calculation.
		private int baseline_coords_type; // Coordinate system of baseline. 0 == ECEF, 1 == NED
	
		public MSG_GPS2_RTK () {
			super(MSG_ID_GPS2_RTK, (short)35, "GPS2_RTK");
		}

		public MSG_GPS2_RTK (byte[] bytes) {
			super(bytes, MSG_ID_GPS2_RTK, (short)35, "GPS2_RTK");
		}
	
		public MSG_GPS2_RTK (short sys, short comp, long time_last_baseline_ms  , long tow  , int baseline_a_mm  , int baseline_b_mm  , int baseline_c_mm  , long accuracy  , int iar_num_hypotheses  , int wn  , int rtk_receiver_id  , int rtk_health  , int rtk_rate  , int nsats  , int baseline_coords_type  ) {
			super(sys, comp, (short)35, "GPS2_RTK");
			this.time_last_baseline_ms = time_last_baseline_ms;
			this.tow = tow;
			this.baseline_a_mm = baseline_a_mm;
			this.baseline_b_mm = baseline_b_mm;
			this.baseline_c_mm = baseline_c_mm;
			this.accuracy = accuracy;
			this.iar_num_hypotheses = iar_num_hypotheses;
			this.wn = wn;
			this.rtk_receiver_id = rtk_receiver_id;
			this.rtk_health = rtk_health;
			this.rtk_rate = rtk_rate;
			this.nsats = nsats;
			this.baseline_coords_type = baseline_coords_type;
		}

		@Override
		public int getCRCExtra() {
			return 226;
		}
	
		public long getTime_last_baseline_ms() {
			return time_last_baseline_ms;
		}
		
		public void setTime_last_baseline_ms(long time_last_baseline_ms) {
			this.time_last_baseline_ms = time_last_baseline_ms;
		}
		
		public long getTow() {
			return tow;
		}
		
		public void setTow(long tow) {
			this.tow = tow;
		}
		
		public int getBaseline_a_mm() {
			return baseline_a_mm;
		}
		
		public void setBaseline_a_mm(int baseline_a_mm) {
			this.baseline_a_mm = baseline_a_mm;
		}
		
		public int getBaseline_b_mm() {
			return baseline_b_mm;
		}
		
		public void setBaseline_b_mm(int baseline_b_mm) {
			this.baseline_b_mm = baseline_b_mm;
		}
		
		public int getBaseline_c_mm() {
			return baseline_c_mm;
		}
		
		public void setBaseline_c_mm(int baseline_c_mm) {
			this.baseline_c_mm = baseline_c_mm;
		}
		
		public long getAccuracy() {
			return accuracy;
		}
		
		public void setAccuracy(long accuracy) {
			this.accuracy = accuracy;
		}
		
		public int getIar_num_hypotheses() {
			return iar_num_hypotheses;
		}
		
		public void setIar_num_hypotheses(int iar_num_hypotheses) {
			this.iar_num_hypotheses = iar_num_hypotheses;
		}
		
		public int getWn() {
			return wn;
		}
		
		public void setWn(int wn) {
			this.wn = wn;
		}
		
		public int getRtk_receiver_id() {
			return rtk_receiver_id;
		}
		
		public void setRtk_receiver_id(int rtk_receiver_id) {
			this.rtk_receiver_id = rtk_receiver_id;
		}
		
		public int getRtk_health() {
			return rtk_health;
		}
		
		public void setRtk_health(int rtk_health) {
			this.rtk_health = rtk_health;
		}
		
		public int getRtk_rate() {
			return rtk_rate;
		}
		
		public void setRtk_rate(int rtk_rate) {
			this.rtk_rate = rtk_rate;
		}
		
		public int getNsats() {
			return nsats;
		}
		
		public void setNsats(int nsats) {
			this.nsats = nsats;
		}
		
		public int getBaseline_coords_type() {
			return baseline_coords_type;
		}
		
		public void setBaseline_coords_type(int baseline_coords_type) {
			this.baseline_coords_type = baseline_coords_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS2_RTK";
			
			time_last_baseline_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			tow = buffer.getInt() & 0xffffffff; // uint32_t
  			baseline_a_mm = buffer.getInt(); // int32_t
  			baseline_b_mm = buffer.getInt(); // int32_t
  			baseline_c_mm = buffer.getInt(); // int32_t
  			accuracy = buffer.getInt() & 0xffffffff; // uint32_t
  			iar_num_hypotheses = buffer.getInt(); // int32_t
  			wn = buffer.getShort() & 0xffff; // uint16_t
  			rtk_receiver_id = (int)buffer.get() & 0xff; // uint8_t
  			rtk_health = (int)buffer.get() & 0xff; // uint8_t
  			rtk_rate = (int)buffer.get() & 0xff; // uint8_t
  			nsats = (int)buffer.get() & 0xff; // uint8_t
  			baseline_coords_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_last_baseline_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(tow & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(baseline_a_mm)); // int32_t
  			buffer.putInt((int)(baseline_b_mm)); // int32_t
  			buffer.putInt((int)(baseline_c_mm)); // int32_t
  			buffer.putInt((int)(accuracy & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(iar_num_hypotheses)); // int32_t
  			buffer.putShort((short)(wn & 0xffff)); // uint16_t
  			buffer.put((byte)(rtk_receiver_id & 0xff)); // uint8_t
  			buffer.put((byte)(rtk_health & 0xff)); // uint8_t
  			buffer.put((byte)(rtk_rate & 0xff)); // uint8_t
  			buffer.put((byte)(nsats & 0xff)); // uint8_t
  			buffer.put((byte)(baseline_coords_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS2_RTK { " + 
			"time_last_baseline_ms = " + time_last_baseline_ms + ", " + 
			"tow = " + tow + ", " + 
			"baseline_a_mm = " + baseline_a_mm + ", " + 
			"baseline_b_mm = " + baseline_b_mm + ", " + 
			"baseline_c_mm = " + baseline_c_mm + ", " + 
			"accuracy = " + accuracy + ", " + 
			"iar_num_hypotheses = " + iar_num_hypotheses + ", " + 
			"wn = " + wn + ", " + 
			"rtk_receiver_id = " + rtk_receiver_id + ", " + 
			"rtk_health = " + rtk_health + ", " + 
			"rtk_rate = " + rtk_rate + ", " + 
			"nsats = " + nsats + ", " + 
			"baseline_coords_type = " + baseline_coords_type + ",  }";
		}
	}
	/*
	 * Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
	 */
	public static class MSG_GPS_GLOBAL_ORIGIN extends Message {
	
		private int latitude; // Latitude (WGS84), in degrees * 1E7
		private int longitude; // Longitude (WGS84), in degrees * 1E7
		private int altitude; // Altitude (AMSL), in meters * 1000 (positive for up)
	
		public MSG_GPS_GLOBAL_ORIGIN () {
			super(MSG_ID_GPS_GLOBAL_ORIGIN, (short)12, "GPS_GLOBAL_ORIGIN");
		}

		public MSG_GPS_GLOBAL_ORIGIN (byte[] bytes) {
			super(bytes, MSG_ID_GPS_GLOBAL_ORIGIN, (short)12, "GPS_GLOBAL_ORIGIN");
		}
	
		public MSG_GPS_GLOBAL_ORIGIN (short sys, short comp, int latitude  , int longitude  , int altitude  ) {
			super(sys, comp, (short)12, "GPS_GLOBAL_ORIGIN");
			this.latitude = latitude;
			this.longitude = longitude;
			this.altitude = altitude;
		}

		@Override
		public int getCRCExtra() {
			return 39;
		}
	
		public int getLatitude() {
			return latitude;
		}
		
		public void setLatitude(int latitude) {
			this.latitude = latitude;
		}
		
		public int getLongitude() {
			return longitude;
		}
		
		public void setLongitude(int longitude) {
			this.longitude = longitude;
		}
		
		public int getAltitude() {
			return altitude;
		}
		
		public void setAltitude(int altitude) {
			this.altitude = altitude;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS_GLOBAL_ORIGIN";
			
			latitude = buffer.getInt(); // int32_t
  			longitude = buffer.getInt(); // int32_t
  			altitude = buffer.getInt(); // int32_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(latitude)); // int32_t
  			buffer.putInt((int)(longitude)); // int32_t
  			buffer.putInt((int)(altitude)); // int32_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS_GLOBAL_ORIGIN { " + 
			"latitude = " + latitude + ", " + 
			"longitude = " + longitude + ", " + 
			"altitude = " + altitude + ",  }";
		}
	}
	/*
	 * data for injecting into the onboard GPS (used for DGPS)
	 */
	public static class MSG_GPS_INJECT_DATA extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
		private int len; // data length
		private int[] data = new int[110]; // raw data (110 is enough for 12 satellites of RTCMv2)
	
		public MSG_GPS_INJECT_DATA () {
			super(MSG_ID_GPS_INJECT_DATA, (short)4, "GPS_INJECT_DATA");
		}

		public MSG_GPS_INJECT_DATA (byte[] bytes) {
			super(bytes, MSG_ID_GPS_INJECT_DATA, (short)4, "GPS_INJECT_DATA");
		}
	
		public MSG_GPS_INJECT_DATA (short sys, short comp, int target_system  , int target_component  , int len  , int data [] ) {
			super(sys, comp, (short)4, "GPS_INJECT_DATA");
			this.target_system = target_system;
			this.target_component = target_component;
			this.len = len;
			this.data = data;
		}

		@Override
		public int getCRCExtra() {
			return 250;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getLen() {
			return len;
		}
		
		public void setLen(int len) {
			this.len = len;
		}
		
		public int[] getData() {
			return data;
		}
		
		public void setData(int data[]) {
			this.data = data;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS_INJECT_DATA";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			len = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<110; ++c) {
				data [c] =  (int)buffer.get() & 0xff; // uint8_t[110]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(len & 0xff)); // uint8_t
  			for(int c=0; c<110; ++c) {
				buffer.put((byte)(data [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS_INJECT_DATA { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"len = " + len + ", " + 
			"data = " + data + ",  }";
		}
	}
	/*
	 * The global position, as returned by the Global Positioning System (GPS). This is
                NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
	 */
	public static class MSG_GPS_RAW_INT extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int lat; // Latitude (WGS84), in degrees * 1E7
		private int lon; // Longitude (WGS84), in degrees * 1E7
		private int alt; // Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
		private int eph; // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
		private int epv; // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
		private int vel; // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
		private int cog; // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
		private int fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		private int satellites_visible; // Number of satellites visible. If unknown, set to 255
	
		public MSG_GPS_RAW_INT () {
			super(MSG_ID_GPS_RAW_INT, (short)86, "GPS_RAW_INT");
		}

		public MSG_GPS_RAW_INT (byte[] bytes) {
			super(bytes, MSG_ID_GPS_RAW_INT, (short)86, "GPS_RAW_INT");
		}
	
		public MSG_GPS_RAW_INT (short sys, short comp, long time_usec  , int lat  , int lon  , int alt  , int eph  , int epv  , int vel  , int cog  , int fix_type  , int satellites_visible  ) {
			super(sys, comp, (short)86, "GPS_RAW_INT");
			this.time_usec = time_usec;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.eph = eph;
			this.epv = epv;
			this.vel = vel;
			this.cog = cog;
			this.fix_type = fix_type;
			this.satellites_visible = satellites_visible;
		}

		@Override
		public int getCRCExtra() {
			return 24;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getEph() {
			return eph;
		}
		
		public void setEph(int eph) {
			this.eph = eph;
		}
		
		public int getEpv() {
			return epv;
		}
		
		public void setEpv(int epv) {
			this.epv = epv;
		}
		
		public int getVel() {
			return vel;
		}
		
		public void setVel(int vel) {
			this.vel = vel;
		}
		
		public int getCog() {
			return cog;
		}
		
		public void setCog(int cog) {
			this.cog = cog;
		}
		
		public int getFix_type() {
			return fix_type;
		}
		
		public void setFix_type(int fix_type) {
			this.fix_type = fix_type;
		}
		
		public int getSatellites_visible() {
			return satellites_visible;
		}
		
		public void setSatellites_visible(int satellites_visible) {
			this.satellites_visible = satellites_visible;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS_RAW_INT";
			
			time_usec = buffer.getLong(); // uint64_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			eph = buffer.getShort() & 0xffff; // uint16_t
  			epv = buffer.getShort() & 0xffff; // uint16_t
  			vel = buffer.getShort() & 0xffff; // uint16_t
  			cog = buffer.getShort() & 0xffff; // uint16_t
  			fix_type = (int)buffer.get() & 0xff; // uint8_t
  			satellites_visible = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putShort((short)(eph & 0xffff)); // uint16_t
  			buffer.putShort((short)(epv & 0xffff)); // uint16_t
  			buffer.putShort((short)(vel & 0xffff)); // uint16_t
  			buffer.putShort((short)(cog & 0xffff)); // uint16_t
  			buffer.put((byte)(fix_type & 0xff)); // uint8_t
  			buffer.put((byte)(satellites_visible & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS_RAW_INT { " + 
			"time_usec = " + time_usec + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"eph = " + eph + ", " + 
			"epv = " + epv + ", " + 
			"vel = " + vel + ", " + 
			"cog = " + cog + ", " + 
			"fix_type = " + fix_type + ", " + 
			"satellites_visible = " + satellites_visible + ",  }";
		}
	}
	/*
	 * RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
	 */
	public static class MSG_GPS_RTK extends Message {
	
		private long time_last_baseline_ms; // Time since boot of last baseline message received in ms.
		private long tow; // GPS Time of Week of last baseline
		private int baseline_a_mm; // Current baseline in ECEF x or NED north component in mm.
		private int baseline_b_mm; // Current baseline in ECEF y or NED east component in mm.
		private int baseline_c_mm; // Current baseline in ECEF z or NED down component in mm.
		private long accuracy; // Current estimate of baseline accuracy.
		private int iar_num_hypotheses; // Current number of integer ambiguity hypotheses.
		private int wn; // GPS Week Number of last baseline
		private int rtk_receiver_id; // Identification of connected RTK receiver.
		private int rtk_health; // GPS-specific health report for RTK data.
		private int rtk_rate; // Rate of baseline messages being received by GPS, in HZ
		private int nsats; // Current number of sats used for RTK calculation.
		private int baseline_coords_type; // Coordinate system of baseline. 0 == ECEF, 1 == NED
	
		public MSG_GPS_RTK () {
			super(MSG_ID_GPS_RTK, (short)35, "GPS_RTK");
		}

		public MSG_GPS_RTK (byte[] bytes) {
			super(bytes, MSG_ID_GPS_RTK, (short)35, "GPS_RTK");
		}
	
		public MSG_GPS_RTK (short sys, short comp, long time_last_baseline_ms  , long tow  , int baseline_a_mm  , int baseline_b_mm  , int baseline_c_mm  , long accuracy  , int iar_num_hypotheses  , int wn  , int rtk_receiver_id  , int rtk_health  , int rtk_rate  , int nsats  , int baseline_coords_type  ) {
			super(sys, comp, (short)35, "GPS_RTK");
			this.time_last_baseline_ms = time_last_baseline_ms;
			this.tow = tow;
			this.baseline_a_mm = baseline_a_mm;
			this.baseline_b_mm = baseline_b_mm;
			this.baseline_c_mm = baseline_c_mm;
			this.accuracy = accuracy;
			this.iar_num_hypotheses = iar_num_hypotheses;
			this.wn = wn;
			this.rtk_receiver_id = rtk_receiver_id;
			this.rtk_health = rtk_health;
			this.rtk_rate = rtk_rate;
			this.nsats = nsats;
			this.baseline_coords_type = baseline_coords_type;
		}

		@Override
		public int getCRCExtra() {
			return 25;
		}
	
		public long getTime_last_baseline_ms() {
			return time_last_baseline_ms;
		}
		
		public void setTime_last_baseline_ms(long time_last_baseline_ms) {
			this.time_last_baseline_ms = time_last_baseline_ms;
		}
		
		public long getTow() {
			return tow;
		}
		
		public void setTow(long tow) {
			this.tow = tow;
		}
		
		public int getBaseline_a_mm() {
			return baseline_a_mm;
		}
		
		public void setBaseline_a_mm(int baseline_a_mm) {
			this.baseline_a_mm = baseline_a_mm;
		}
		
		public int getBaseline_b_mm() {
			return baseline_b_mm;
		}
		
		public void setBaseline_b_mm(int baseline_b_mm) {
			this.baseline_b_mm = baseline_b_mm;
		}
		
		public int getBaseline_c_mm() {
			return baseline_c_mm;
		}
		
		public void setBaseline_c_mm(int baseline_c_mm) {
			this.baseline_c_mm = baseline_c_mm;
		}
		
		public long getAccuracy() {
			return accuracy;
		}
		
		public void setAccuracy(long accuracy) {
			this.accuracy = accuracy;
		}
		
		public int getIar_num_hypotheses() {
			return iar_num_hypotheses;
		}
		
		public void setIar_num_hypotheses(int iar_num_hypotheses) {
			this.iar_num_hypotheses = iar_num_hypotheses;
		}
		
		public int getWn() {
			return wn;
		}
		
		public void setWn(int wn) {
			this.wn = wn;
		}
		
		public int getRtk_receiver_id() {
			return rtk_receiver_id;
		}
		
		public void setRtk_receiver_id(int rtk_receiver_id) {
			this.rtk_receiver_id = rtk_receiver_id;
		}
		
		public int getRtk_health() {
			return rtk_health;
		}
		
		public void setRtk_health(int rtk_health) {
			this.rtk_health = rtk_health;
		}
		
		public int getRtk_rate() {
			return rtk_rate;
		}
		
		public void setRtk_rate(int rtk_rate) {
			this.rtk_rate = rtk_rate;
		}
		
		public int getNsats() {
			return nsats;
		}
		
		public void setNsats(int nsats) {
			this.nsats = nsats;
		}
		
		public int getBaseline_coords_type() {
			return baseline_coords_type;
		}
		
		public void setBaseline_coords_type(int baseline_coords_type) {
			this.baseline_coords_type = baseline_coords_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS_RTK";
			
			time_last_baseline_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			tow = buffer.getInt() & 0xffffffff; // uint32_t
  			baseline_a_mm = buffer.getInt(); // int32_t
  			baseline_b_mm = buffer.getInt(); // int32_t
  			baseline_c_mm = buffer.getInt(); // int32_t
  			accuracy = buffer.getInt() & 0xffffffff; // uint32_t
  			iar_num_hypotheses = buffer.getInt(); // int32_t
  			wn = buffer.getShort() & 0xffff; // uint16_t
  			rtk_receiver_id = (int)buffer.get() & 0xff; // uint8_t
  			rtk_health = (int)buffer.get() & 0xff; // uint8_t
  			rtk_rate = (int)buffer.get() & 0xff; // uint8_t
  			nsats = (int)buffer.get() & 0xff; // uint8_t
  			baseline_coords_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_last_baseline_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(tow & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(baseline_a_mm)); // int32_t
  			buffer.putInt((int)(baseline_b_mm)); // int32_t
  			buffer.putInt((int)(baseline_c_mm)); // int32_t
  			buffer.putInt((int)(accuracy & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(iar_num_hypotheses)); // int32_t
  			buffer.putShort((short)(wn & 0xffff)); // uint16_t
  			buffer.put((byte)(rtk_receiver_id & 0xff)); // uint8_t
  			buffer.put((byte)(rtk_health & 0xff)); // uint8_t
  			buffer.put((byte)(rtk_rate & 0xff)); // uint8_t
  			buffer.put((byte)(nsats & 0xff)); // uint8_t
  			buffer.put((byte)(baseline_coords_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS_RTK { " + 
			"time_last_baseline_ms = " + time_last_baseline_ms + ", " + 
			"tow = " + tow + ", " + 
			"baseline_a_mm = " + baseline_a_mm + ", " + 
			"baseline_b_mm = " + baseline_b_mm + ", " + 
			"baseline_c_mm = " + baseline_c_mm + ", " + 
			"accuracy = " + accuracy + ", " + 
			"iar_num_hypotheses = " + iar_num_hypotheses + ", " + 
			"wn = " + wn + ", " + 
			"rtk_receiver_id = " + rtk_receiver_id + ", " + 
			"rtk_health = " + rtk_health + ", " + 
			"rtk_rate = " + rtk_rate + ", " + 
			"nsats = " + nsats + ", " + 
			"baseline_coords_type = " + baseline_coords_type + ",  }";
		}
	}
	/*
	 * The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
	 */
	public static class MSG_GPS_STATUS extends Message {
	
		private int satellites_visible; // Number of satellites visible
		private int[] satellite_prn = new int[20]; // Global satellite ID
		private int[] satellite_used = new int[20]; // 0: Satellite not used, 1: used for localization
		private int[] satellite_elevation = new int[20]; // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
		private int[] satellite_azimuth = new int[20]; // Direction of satellite, 0: 0 deg, 255: 360 deg.
		private int[] satellite_snr = new int[20]; // Signal to noise ratio of satellite
	
		public MSG_GPS_STATUS () {
			super(MSG_ID_GPS_STATUS, (short)6, "GPS_STATUS");
		}

		public MSG_GPS_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_GPS_STATUS, (short)6, "GPS_STATUS");
		}
	
		public MSG_GPS_STATUS (short sys, short comp, int satellites_visible  , int satellite_prn [] , int satellite_used [] , int satellite_elevation [] , int satellite_azimuth [] , int satellite_snr [] ) {
			super(sys, comp, (short)6, "GPS_STATUS");
			this.satellites_visible = satellites_visible;
			this.satellite_prn = satellite_prn;
			this.satellite_used = satellite_used;
			this.satellite_elevation = satellite_elevation;
			this.satellite_azimuth = satellite_azimuth;
			this.satellite_snr = satellite_snr;
		}

		@Override
		public int getCRCExtra() {
			return 23;
		}
	
		public int getSatellites_visible() {
			return satellites_visible;
		}
		
		public void setSatellites_visible(int satellites_visible) {
			this.satellites_visible = satellites_visible;
		}
		
		public int[] getSatellite_prn() {
			return satellite_prn;
		}
		
		public void setSatellite_prn(int satellite_prn[]) {
			this.satellite_prn = satellite_prn;
		}
		
		public int[] getSatellite_used() {
			return satellite_used;
		}
		
		public void setSatellite_used(int satellite_used[]) {
			this.satellite_used = satellite_used;
		}
		
		public int[] getSatellite_elevation() {
			return satellite_elevation;
		}
		
		public void setSatellite_elevation(int satellite_elevation[]) {
			this.satellite_elevation = satellite_elevation;
		}
		
		public int[] getSatellite_azimuth() {
			return satellite_azimuth;
		}
		
		public void setSatellite_azimuth(int satellite_azimuth[]) {
			this.satellite_azimuth = satellite_azimuth;
		}
		
		public int[] getSatellite_snr() {
			return satellite_snr;
		}
		
		public void setSatellite_snr(int satellite_snr[]) {
			this.satellite_snr = satellite_snr;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "GPS_STATUS";
			
			satellites_visible = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<20; ++c) {
				satellite_prn [c] =  (int)buffer.get() & 0xff; // uint8_t[20]
 			}
			
 			for(int c=0; c<20; ++c) {
				satellite_used [c] =  (int)buffer.get() & 0xff; // uint8_t[20]
 			}
			
 			for(int c=0; c<20; ++c) {
				satellite_elevation [c] =  (int)buffer.get() & 0xff; // uint8_t[20]
 			}
			
 			for(int c=0; c<20; ++c) {
				satellite_azimuth [c] =  (int)buffer.get() & 0xff; // uint8_t[20]
 			}
			
 			for(int c=0; c<20; ++c) {
				satellite_snr [c] =  (int)buffer.get() & 0xff; // uint8_t[20]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(satellites_visible & 0xff)); // uint8_t
  			for(int c=0; c<20; ++c) {
				buffer.put((byte)(satellite_prn [c] & 0xff));
 			}
			
 			for(int c=0; c<20; ++c) {
				buffer.put((byte)(satellite_used [c] & 0xff));
 			}
			
 			for(int c=0; c<20; ++c) {
				buffer.put((byte)(satellite_elevation [c] & 0xff));
 			}
			
 			for(int c=0; c<20; ++c) {
				buffer.put((byte)(satellite_azimuth [c] & 0xff));
 			}
			
 			for(int c=0; c<20; ++c) {
				buffer.put((byte)(satellite_snr [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_GPS_STATUS { " + 
			"satellites_visible = " + satellites_visible + ", " + 
			"satellite_prn = " + satellite_prn + ", " + 
			"satellite_used = " + satellite_used + ", " + 
			"satellite_elevation = " + satellite_elevation + ", " + 
			"satellite_azimuth = " + satellite_azimuth + ", " + 
			"satellite_snr = " + satellite_snr + ",  }";
		}
	}
	/*
	 * The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
	 */
	public static class MSG_HEARTBEAT extends Message {
	
		private long custom_mode; // A bitfield for use for autopilot-specific flags.
		private int type; // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
		private int autopilot; // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
		private int base_mode; // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
		private int system_status; // System status flag, see MAV_STATE ENUM
		private int mavlink_version; // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
	
		public MSG_HEARTBEAT () {
			super(MSG_ID_HEARTBEAT, (short)9, "HEARTBEAT");
		}

		public MSG_HEARTBEAT (byte[] bytes) {
			super(bytes, MSG_ID_HEARTBEAT, (short)9, "HEARTBEAT");
		}
	
		public MSG_HEARTBEAT (short sys, short comp, long custom_mode  , int type  , int autopilot  , int base_mode  , int system_status  , int mavlink_version  ) {
			super(sys, comp, (short)9, "HEARTBEAT");
			this.custom_mode = custom_mode;
			this.type = type;
			this.autopilot = autopilot;
			this.base_mode = base_mode;
			this.system_status = system_status;
			this.mavlink_version = mavlink_version;
		}

		@Override
		public int getCRCExtra() {
			return 50;
		}
	
		public long getCustom_mode() {
			return custom_mode;
		}
		
		public void setCustom_mode(long custom_mode) {
			this.custom_mode = custom_mode;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getAutopilot() {
			return autopilot;
		}
		
		public void setAutopilot(int autopilot) {
			this.autopilot = autopilot;
		}
		
		public int getBase_mode() {
			return base_mode;
		}
		
		public void setBase_mode(int base_mode) {
			this.base_mode = base_mode;
		}
		
		public int getSystem_status() {
			return system_status;
		}
		
		public void setSystem_status(int system_status) {
			this.system_status = system_status;
		}
		
		public int getMavlink_version() {
			return mavlink_version;
		}
		
		public void setMavlink_version(int mavlink_version) {
			this.mavlink_version = mavlink_version;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HEARTBEAT";
			
			custom_mode = buffer.getInt() & 0xffffffff; // uint32_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			autopilot = (int)buffer.get() & 0xff; // uint8_t
  			base_mode = (int)buffer.get() & 0xff; // uint8_t
  			system_status = (int)buffer.get() & 0xff; // uint8_t
  			mavlink_version = (int)buffer.get() & 0xff; // uint8_t_mavlink_version
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(custom_mode & 0xffffffff)); // uint32_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(autopilot & 0xff)); // uint8_t
  			buffer.put((byte)(base_mode & 0xff)); // uint8_t
  			buffer.put((byte)(system_status & 0xff)); // uint8_t
  			buffer.put((byte)(mavlink_version & 0xff)); // uint8_t_mavlink_version
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HEARTBEAT { " + 
			"custom_mode = " + custom_mode + ", " + 
			"type = " + type + ", " + 
			"autopilot = " + autopilot + ", " + 
			"base_mode = " + base_mode + ", " + 
			"system_status = " + system_status + ", " + 
			"mavlink_version = " + mavlink_version + ",  }";
		}
	}
	/*
	 * The IMU readings in SI units in NED body frame
	 */
	public static class MSG_HIGHRES_IMU extends Message {
	
		private long time_usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float xacc; // X acceleration (m/s^2)
		private float yacc; // Y acceleration (m/s^2)
		private float zacc; // Z acceleration (m/s^2)
		private float xgyro; // Angular speed around X axis (rad / sec)
		private float ygyro; // Angular speed around Y axis (rad / sec)
		private float zgyro; // Angular speed around Z axis (rad / sec)
		private float xmag; // X Magnetic field (Gauss)
		private float ymag; // Y Magnetic field (Gauss)
		private float zmag; // Z Magnetic field (Gauss)
		private float abs_pressure; // Absolute pressure in millibar
		private float diff_pressure; // Differential pressure in millibar
		private float pressure_alt; // Altitude calculated from pressure
		private float temperature; // Temperature in degrees celsius
		private int fields_updated; // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
	
		public MSG_HIGHRES_IMU () {
			super(MSG_ID_HIGHRES_IMU, (short)118, "HIGHRES_IMU");
		}

		public MSG_HIGHRES_IMU (byte[] bytes) {
			super(bytes, MSG_ID_HIGHRES_IMU, (short)118, "HIGHRES_IMU");
		}
	
		public MSG_HIGHRES_IMU (short sys, short comp, long time_usec  , float xacc  , float yacc  , float zacc  , float xgyro  , float ygyro  , float zgyro  , float xmag  , float ymag  , float zmag  , float abs_pressure  , float diff_pressure  , float pressure_alt  , float temperature  , int fields_updated  ) {
			super(sys, comp, (short)118, "HIGHRES_IMU");
			this.time_usec = time_usec;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
			this.abs_pressure = abs_pressure;
			this.diff_pressure = diff_pressure;
			this.pressure_alt = pressure_alt;
			this.temperature = temperature;
			this.fields_updated = fields_updated;
		}

		@Override
		public int getCRCExtra() {
			return 93;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getXacc() {
			return xacc;
		}
		
		public void setXacc(float xacc) {
			this.xacc = xacc;
		}
		
		public float getYacc() {
			return yacc;
		}
		
		public void setYacc(float yacc) {
			this.yacc = yacc;
		}
		
		public float getZacc() {
			return zacc;
		}
		
		public void setZacc(float zacc) {
			this.zacc = zacc;
		}
		
		public float getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(float xgyro) {
			this.xgyro = xgyro;
		}
		
		public float getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(float ygyro) {
			this.ygyro = ygyro;
		}
		
		public float getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(float zgyro) {
			this.zgyro = zgyro;
		}
		
		public float getXmag() {
			return xmag;
		}
		
		public void setXmag(float xmag) {
			this.xmag = xmag;
		}
		
		public float getYmag() {
			return ymag;
		}
		
		public void setYmag(float ymag) {
			this.ymag = ymag;
		}
		
		public float getZmag() {
			return zmag;
		}
		
		public void setZmag(float zmag) {
			this.zmag = zmag;
		}
		
		public float getAbs_pressure() {
			return abs_pressure;
		}
		
		public void setAbs_pressure(float abs_pressure) {
			this.abs_pressure = abs_pressure;
		}
		
		public float getDiff_pressure() {
			return diff_pressure;
		}
		
		public void setDiff_pressure(float diff_pressure) {
			this.diff_pressure = diff_pressure;
		}
		
		public float getPressure_alt() {
			return pressure_alt;
		}
		
		public void setPressure_alt(float pressure_alt) {
			this.pressure_alt = pressure_alt;
		}
		
		public float getTemperature() {
			return temperature;
		}
		
		public void setTemperature(float temperature) {
			this.temperature = temperature;
		}
		
		public int getFields_updated() {
			return fields_updated;
		}
		
		public void setFields_updated(int fields_updated) {
			this.fields_updated = fields_updated;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIGHRES_IMU";
			
			time_usec = buffer.getLong(); // uint64_t
  			xacc = buffer.getFloat(); // float
  			yacc = buffer.getFloat(); // float
  			zacc = buffer.getFloat(); // float
  			xgyro = buffer.getFloat(); // float
  			ygyro = buffer.getFloat(); // float
  			zgyro = buffer.getFloat(); // float
  			xmag = buffer.getFloat(); // float
  			ymag = buffer.getFloat(); // float
  			zmag = buffer.getFloat(); // float
  			abs_pressure = buffer.getFloat(); // float
  			diff_pressure = buffer.getFloat(); // float
  			pressure_alt = buffer.getFloat(); // float
  			temperature = buffer.getFloat(); // float
  			fields_updated = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(xacc); // float
  			buffer.putFloat(yacc); // float
  			buffer.putFloat(zacc); // float
  			buffer.putFloat(xgyro); // float
  			buffer.putFloat(ygyro); // float
  			buffer.putFloat(zgyro); // float
  			buffer.putFloat(xmag); // float
  			buffer.putFloat(ymag); // float
  			buffer.putFloat(zmag); // float
  			buffer.putFloat(abs_pressure); // float
  			buffer.putFloat(diff_pressure); // float
  			buffer.putFloat(pressure_alt); // float
  			buffer.putFloat(temperature); // float
  			buffer.putShort((short)(fields_updated & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIGHRES_IMU { " + 
			"time_usec = " + time_usec + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ", " + 
			"abs_pressure = " + abs_pressure + ", " + 
			"diff_pressure = " + diff_pressure + ", " + 
			"pressure_alt = " + pressure_alt + ", " + 
			"temperature = " + temperature + ", " + 
			"fields_updated = " + fields_updated + ",  }";
		}
	}
	/*
	 * Sent from autopilot to simulation. Hardware in the loop control outputs
	 */
	public static class MSG_HIL_CONTROLS extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private float roll_ailerons; // Control output -1 .. 1
		private float pitch_elevator; // Control output -1 .. 1
		private float yaw_rudder; // Control output -1 .. 1
		private float throttle; // Throttle 0 .. 1
		private float aux1; // Aux 1, -1 .. 1
		private float aux2; // Aux 2, -1 .. 1
		private float aux3; // Aux 3, -1 .. 1
		private float aux4; // Aux 4, -1 .. 1
		private int mode; // System mode (MAV_MODE)
		private int nav_mode; // Navigation mode (MAV_NAV_MODE)
	
		public MSG_HIL_CONTROLS () {
			super(MSG_ID_HIL_CONTROLS, (short)98, "HIL_CONTROLS");
		}

		public MSG_HIL_CONTROLS (byte[] bytes) {
			super(bytes, MSG_ID_HIL_CONTROLS, (short)98, "HIL_CONTROLS");
		}
	
		public MSG_HIL_CONTROLS (short sys, short comp, long time_usec  , float roll_ailerons  , float pitch_elevator  , float yaw_rudder  , float throttle  , float aux1  , float aux2  , float aux3  , float aux4  , int mode  , int nav_mode  ) {
			super(sys, comp, (short)98, "HIL_CONTROLS");
			this.time_usec = time_usec;
			this.roll_ailerons = roll_ailerons;
			this.pitch_elevator = pitch_elevator;
			this.yaw_rudder = yaw_rudder;
			this.throttle = throttle;
			this.aux1 = aux1;
			this.aux2 = aux2;
			this.aux3 = aux3;
			this.aux4 = aux4;
			this.mode = mode;
			this.nav_mode = nav_mode;
		}

		@Override
		public int getCRCExtra() {
			return 63;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getRoll_ailerons() {
			return roll_ailerons;
		}
		
		public void setRoll_ailerons(float roll_ailerons) {
			this.roll_ailerons = roll_ailerons;
		}
		
		public float getPitch_elevator() {
			return pitch_elevator;
		}
		
		public void setPitch_elevator(float pitch_elevator) {
			this.pitch_elevator = pitch_elevator;
		}
		
		public float getYaw_rudder() {
			return yaw_rudder;
		}
		
		public void setYaw_rudder(float yaw_rudder) {
			this.yaw_rudder = yaw_rudder;
		}
		
		public float getThrottle() {
			return throttle;
		}
		
		public void setThrottle(float throttle) {
			this.throttle = throttle;
		}
		
		public float getAux1() {
			return aux1;
		}
		
		public void setAux1(float aux1) {
			this.aux1 = aux1;
		}
		
		public float getAux2() {
			return aux2;
		}
		
		public void setAux2(float aux2) {
			this.aux2 = aux2;
		}
		
		public float getAux3() {
			return aux3;
		}
		
		public void setAux3(float aux3) {
			this.aux3 = aux3;
		}
		
		public float getAux4() {
			return aux4;
		}
		
		public void setAux4(float aux4) {
			this.aux4 = aux4;
		}
		
		public int getMode() {
			return mode;
		}
		
		public void setMode(int mode) {
			this.mode = mode;
		}
		
		public int getNav_mode() {
			return nav_mode;
		}
		
		public void setNav_mode(int nav_mode) {
			this.nav_mode = nav_mode;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_CONTROLS";
			
			time_usec = buffer.getLong(); // uint64_t
  			roll_ailerons = buffer.getFloat(); // float
  			pitch_elevator = buffer.getFloat(); // float
  			yaw_rudder = buffer.getFloat(); // float
  			throttle = buffer.getFloat(); // float
  			aux1 = buffer.getFloat(); // float
  			aux2 = buffer.getFloat(); // float
  			aux3 = buffer.getFloat(); // float
  			aux4 = buffer.getFloat(); // float
  			mode = (int)buffer.get() & 0xff; // uint8_t
  			nav_mode = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(roll_ailerons); // float
  			buffer.putFloat(pitch_elevator); // float
  			buffer.putFloat(yaw_rudder); // float
  			buffer.putFloat(throttle); // float
  			buffer.putFloat(aux1); // float
  			buffer.putFloat(aux2); // float
  			buffer.putFloat(aux3); // float
  			buffer.putFloat(aux4); // float
  			buffer.put((byte)(mode & 0xff)); // uint8_t
  			buffer.put((byte)(nav_mode & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_CONTROLS { " + 
			"time_usec = " + time_usec + ", " + 
			"roll_ailerons = " + roll_ailerons + ", " + 
			"pitch_elevator = " + pitch_elevator + ", " + 
			"yaw_rudder = " + yaw_rudder + ", " + 
			"throttle = " + throttle + ", " + 
			"aux1 = " + aux1 + ", " + 
			"aux2 = " + aux2 + ", " + 
			"aux3 = " + aux3 + ", " + 
			"aux4 = " + aux4 + ", " + 
			"mode = " + mode + ", " + 
			"nav_mode = " + nav_mode + ",  }";
		}
	}
	/*
	 * The global position, as returned by the Global Positioning System (GPS). This is
                 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
	 */
	public static class MSG_HIL_GPS extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int lat; // Latitude (WGS84), in degrees * 1E7
		private int lon; // Longitude (WGS84), in degrees * 1E7
		private int alt; // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
		private int eph; // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
		private int epv; // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
		private int vel; // GPS ground speed (m/s * 100). If unknown, set to: 65535
		private int vn; // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
		private int ve; // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
		private int vd; // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
		private int cog; // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
		private int fix_type; // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
		private int satellites_visible; // Number of satellites visible. If unknown, set to 255
	
		public MSG_HIL_GPS () {
			super(MSG_ID_HIL_GPS, (short)92, "HIL_GPS");
		}

		public MSG_HIL_GPS (byte[] bytes) {
			super(bytes, MSG_ID_HIL_GPS, (short)92, "HIL_GPS");
		}
	
		public MSG_HIL_GPS (short sys, short comp, long time_usec  , int lat  , int lon  , int alt  , int eph  , int epv  , int vel  , int vn  , int ve  , int vd  , int cog  , int fix_type  , int satellites_visible  ) {
			super(sys, comp, (short)92, "HIL_GPS");
			this.time_usec = time_usec;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.eph = eph;
			this.epv = epv;
			this.vel = vel;
			this.vn = vn;
			this.ve = ve;
			this.vd = vd;
			this.cog = cog;
			this.fix_type = fix_type;
			this.satellites_visible = satellites_visible;
		}

		@Override
		public int getCRCExtra() {
			return 124;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getEph() {
			return eph;
		}
		
		public void setEph(int eph) {
			this.eph = eph;
		}
		
		public int getEpv() {
			return epv;
		}
		
		public void setEpv(int epv) {
			this.epv = epv;
		}
		
		public int getVel() {
			return vel;
		}
		
		public void setVel(int vel) {
			this.vel = vel;
		}
		
		public int getVn() {
			return vn;
		}
		
		public void setVn(int vn) {
			this.vn = vn;
		}
		
		public int getVe() {
			return ve;
		}
		
		public void setVe(int ve) {
			this.ve = ve;
		}
		
		public int getVd() {
			return vd;
		}
		
		public void setVd(int vd) {
			this.vd = vd;
		}
		
		public int getCog() {
			return cog;
		}
		
		public void setCog(int cog) {
			this.cog = cog;
		}
		
		public int getFix_type() {
			return fix_type;
		}
		
		public void setFix_type(int fix_type) {
			this.fix_type = fix_type;
		}
		
		public int getSatellites_visible() {
			return satellites_visible;
		}
		
		public void setSatellites_visible(int satellites_visible) {
			this.satellites_visible = satellites_visible;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_GPS";
			
			time_usec = buffer.getLong(); // uint64_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			eph = buffer.getShort() & 0xffff; // uint16_t
  			epv = buffer.getShort() & 0xffff; // uint16_t
  			vel = buffer.getShort() & 0xffff; // uint16_t
  			vn = buffer.getShort(); // int16_t
  			ve = buffer.getShort(); // int16_t
  			vd = buffer.getShort(); // int16_t
  			cog = buffer.getShort() & 0xffff; // uint16_t
  			fix_type = (int)buffer.get() & 0xff; // uint8_t
  			satellites_visible = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putShort((short)(eph & 0xffff)); // uint16_t
  			buffer.putShort((short)(epv & 0xffff)); // uint16_t
  			buffer.putShort((short)(vel & 0xffff)); // uint16_t
  			buffer.putShort((short)(vn)); // int16_t
  			buffer.putShort((short)(ve)); // int16_t
  			buffer.putShort((short)(vd)); // int16_t
  			buffer.putShort((short)(cog & 0xffff)); // uint16_t
  			buffer.put((byte)(fix_type & 0xff)); // uint8_t
  			buffer.put((byte)(satellites_visible & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_GPS { " + 
			"time_usec = " + time_usec + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"eph = " + eph + ", " + 
			"epv = " + epv + ", " + 
			"vel = " + vel + ", " + 
			"vn = " + vn + ", " + 
			"ve = " + ve + ", " + 
			"vd = " + vd + ", " + 
			"cog = " + cog + ", " + 
			"fix_type = " + fix_type + ", " + 
			"satellites_visible = " + satellites_visible + ",  }";
		}
	}
	/*
	 * Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
	 */
	public static class MSG_HIL_OPTICAL_FLOW extends Message {
	
		private long time_usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private long integration_time_us; // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
		private float integrated_x; // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
		private float integrated_y; // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
		private float integrated_xgyro; // RH rotation around X axis (rad)
		private float integrated_ygyro; // RH rotation around Y axis (rad)
		private float integrated_zgyro; // RH rotation around Z axis (rad)
		private long time_delta_distance_us; // Time in microseconds since the distance was sampled.
		private float distance; // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
		private int temperature; // Temperature * 100 in centi-degrees Celsius
		private int sensor_id; // Sensor ID
		private int quality; // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
		public MSG_HIL_OPTICAL_FLOW () {
			super(MSG_ID_HIL_OPTICAL_FLOW, (short)100, "HIL_OPTICAL_FLOW");
		}

		public MSG_HIL_OPTICAL_FLOW (byte[] bytes) {
			super(bytes, MSG_ID_HIL_OPTICAL_FLOW, (short)100, "HIL_OPTICAL_FLOW");
		}
	
		public MSG_HIL_OPTICAL_FLOW (short sys, short comp, long time_usec  , long integration_time_us  , float integrated_x  , float integrated_y  , float integrated_xgyro  , float integrated_ygyro  , float integrated_zgyro  , long time_delta_distance_us  , float distance  , int temperature  , int sensor_id  , int quality  ) {
			super(sys, comp, (short)100, "HIL_OPTICAL_FLOW");
			this.time_usec = time_usec;
			this.integration_time_us = integration_time_us;
			this.integrated_x = integrated_x;
			this.integrated_y = integrated_y;
			this.integrated_xgyro = integrated_xgyro;
			this.integrated_ygyro = integrated_ygyro;
			this.integrated_zgyro = integrated_zgyro;
			this.time_delta_distance_us = time_delta_distance_us;
			this.distance = distance;
			this.temperature = temperature;
			this.sensor_id = sensor_id;
			this.quality = quality;
		}

		@Override
		public int getCRCExtra() {
			return 237;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public long getIntegration_time_us() {
			return integration_time_us;
		}
		
		public void setIntegration_time_us(long integration_time_us) {
			this.integration_time_us = integration_time_us;
		}
		
		public float getIntegrated_x() {
			return integrated_x;
		}
		
		public void setIntegrated_x(float integrated_x) {
			this.integrated_x = integrated_x;
		}
		
		public float getIntegrated_y() {
			return integrated_y;
		}
		
		public void setIntegrated_y(float integrated_y) {
			this.integrated_y = integrated_y;
		}
		
		public float getIntegrated_xgyro() {
			return integrated_xgyro;
		}
		
		public void setIntegrated_xgyro(float integrated_xgyro) {
			this.integrated_xgyro = integrated_xgyro;
		}
		
		public float getIntegrated_ygyro() {
			return integrated_ygyro;
		}
		
		public void setIntegrated_ygyro(float integrated_ygyro) {
			this.integrated_ygyro = integrated_ygyro;
		}
		
		public float getIntegrated_zgyro() {
			return integrated_zgyro;
		}
		
		public void setIntegrated_zgyro(float integrated_zgyro) {
			this.integrated_zgyro = integrated_zgyro;
		}
		
		public long getTime_delta_distance_us() {
			return time_delta_distance_us;
		}
		
		public void setTime_delta_distance_us(long time_delta_distance_us) {
			this.time_delta_distance_us = time_delta_distance_us;
		}
		
		public float getDistance() {
			return distance;
		}
		
		public void setDistance(float distance) {
			this.distance = distance;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
		public int getSensor_id() {
			return sensor_id;
		}
		
		public void setSensor_id(int sensor_id) {
			this.sensor_id = sensor_id;
		}
		
		public int getQuality() {
			return quality;
		}
		
		public void setQuality(int quality) {
			this.quality = quality;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_OPTICAL_FLOW";
			
			time_usec = buffer.getLong(); // uint64_t
  			integration_time_us = buffer.getInt() & 0xffffffff; // uint32_t
  			integrated_x = buffer.getFloat(); // float
  			integrated_y = buffer.getFloat(); // float
  			integrated_xgyro = buffer.getFloat(); // float
  			integrated_ygyro = buffer.getFloat(); // float
  			integrated_zgyro = buffer.getFloat(); // float
  			time_delta_distance_us = buffer.getInt() & 0xffffffff; // uint32_t
  			distance = buffer.getFloat(); // float
  			temperature = buffer.getShort(); // int16_t
  			sensor_id = (int)buffer.get() & 0xff; // uint8_t
  			quality = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(integration_time_us & 0xffffffff)); // uint32_t
  			buffer.putFloat(integrated_x); // float
  			buffer.putFloat(integrated_y); // float
  			buffer.putFloat(integrated_xgyro); // float
  			buffer.putFloat(integrated_ygyro); // float
  			buffer.putFloat(integrated_zgyro); // float
  			buffer.putInt((int)(time_delta_distance_us & 0xffffffff)); // uint32_t
  			buffer.putFloat(distance); // float
  			buffer.putShort((short)(temperature)); // int16_t
  			buffer.put((byte)(sensor_id & 0xff)); // uint8_t
  			buffer.put((byte)(quality & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_OPTICAL_FLOW { " + 
			"time_usec = " + time_usec + ", " + 
			"integration_time_us = " + integration_time_us + ", " + 
			"integrated_x = " + integrated_x + ", " + 
			"integrated_y = " + integrated_y + ", " + 
			"integrated_xgyro = " + integrated_xgyro + ", " + 
			"integrated_ygyro = " + integrated_ygyro + ", " + 
			"integrated_zgyro = " + integrated_zgyro + ", " + 
			"time_delta_distance_us = " + time_delta_distance_us + ", " + 
			"distance = " + distance + ", " + 
			"temperature = " + temperature + ", " + 
			"sensor_id = " + sensor_id + ", " + 
			"quality = " + quality + ",  }";
		}
	}
	/*
	 * Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
	 */
	public static class MSG_HIL_RC_INPUTS_RAW extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int chan1_raw; // RC channel 1 value, in microseconds
		private int chan2_raw; // RC channel 2 value, in microseconds
		private int chan3_raw; // RC channel 3 value, in microseconds
		private int chan4_raw; // RC channel 4 value, in microseconds
		private int chan5_raw; // RC channel 5 value, in microseconds
		private int chan6_raw; // RC channel 6 value, in microseconds
		private int chan7_raw; // RC channel 7 value, in microseconds
		private int chan8_raw; // RC channel 8 value, in microseconds
		private int chan9_raw; // RC channel 9 value, in microseconds
		private int chan10_raw; // RC channel 10 value, in microseconds
		private int chan11_raw; // RC channel 11 value, in microseconds
		private int chan12_raw; // RC channel 12 value, in microseconds
		private int rssi; // Receive signal strength indicator, 0: 0%, 255: 100%
	
		public MSG_HIL_RC_INPUTS_RAW () {
			super(MSG_ID_HIL_RC_INPUTS_RAW, (short)89, "HIL_RC_INPUTS_RAW");
		}

		public MSG_HIL_RC_INPUTS_RAW (byte[] bytes) {
			super(bytes, MSG_ID_HIL_RC_INPUTS_RAW, (short)89, "HIL_RC_INPUTS_RAW");
		}
	
		public MSG_HIL_RC_INPUTS_RAW (short sys, short comp, long time_usec  , int chan1_raw  , int chan2_raw  , int chan3_raw  , int chan4_raw  , int chan5_raw  , int chan6_raw  , int chan7_raw  , int chan8_raw  , int chan9_raw  , int chan10_raw  , int chan11_raw  , int chan12_raw  , int rssi  ) {
			super(sys, comp, (short)89, "HIL_RC_INPUTS_RAW");
			this.time_usec = time_usec;
			this.chan1_raw = chan1_raw;
			this.chan2_raw = chan2_raw;
			this.chan3_raw = chan3_raw;
			this.chan4_raw = chan4_raw;
			this.chan5_raw = chan5_raw;
			this.chan6_raw = chan6_raw;
			this.chan7_raw = chan7_raw;
			this.chan8_raw = chan8_raw;
			this.chan9_raw = chan9_raw;
			this.chan10_raw = chan10_raw;
			this.chan11_raw = chan11_raw;
			this.chan12_raw = chan12_raw;
			this.rssi = rssi;
		}

		@Override
		public int getCRCExtra() {
			return 54;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getChan1_raw() {
			return chan1_raw;
		}
		
		public void setChan1_raw(int chan1_raw) {
			this.chan1_raw = chan1_raw;
		}
		
		public int getChan2_raw() {
			return chan2_raw;
		}
		
		public void setChan2_raw(int chan2_raw) {
			this.chan2_raw = chan2_raw;
		}
		
		public int getChan3_raw() {
			return chan3_raw;
		}
		
		public void setChan3_raw(int chan3_raw) {
			this.chan3_raw = chan3_raw;
		}
		
		public int getChan4_raw() {
			return chan4_raw;
		}
		
		public void setChan4_raw(int chan4_raw) {
			this.chan4_raw = chan4_raw;
		}
		
		public int getChan5_raw() {
			return chan5_raw;
		}
		
		public void setChan5_raw(int chan5_raw) {
			this.chan5_raw = chan5_raw;
		}
		
		public int getChan6_raw() {
			return chan6_raw;
		}
		
		public void setChan6_raw(int chan6_raw) {
			this.chan6_raw = chan6_raw;
		}
		
		public int getChan7_raw() {
			return chan7_raw;
		}
		
		public void setChan7_raw(int chan7_raw) {
			this.chan7_raw = chan7_raw;
		}
		
		public int getChan8_raw() {
			return chan8_raw;
		}
		
		public void setChan8_raw(int chan8_raw) {
			this.chan8_raw = chan8_raw;
		}
		
		public int getChan9_raw() {
			return chan9_raw;
		}
		
		public void setChan9_raw(int chan9_raw) {
			this.chan9_raw = chan9_raw;
		}
		
		public int getChan10_raw() {
			return chan10_raw;
		}
		
		public void setChan10_raw(int chan10_raw) {
			this.chan10_raw = chan10_raw;
		}
		
		public int getChan11_raw() {
			return chan11_raw;
		}
		
		public void setChan11_raw(int chan11_raw) {
			this.chan11_raw = chan11_raw;
		}
		
		public int getChan12_raw() {
			return chan12_raw;
		}
		
		public void setChan12_raw(int chan12_raw) {
			this.chan12_raw = chan12_raw;
		}
		
		public int getRssi() {
			return rssi;
		}
		
		public void setRssi(int rssi) {
			this.rssi = rssi;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_RC_INPUTS_RAW";
			
			time_usec = buffer.getLong(); // uint64_t
  			chan1_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan2_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan3_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan4_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan5_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan6_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan7_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan8_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan9_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan10_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan11_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan12_raw = buffer.getShort() & 0xffff; // uint16_t
  			rssi = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putShort((short)(chan1_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan2_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan3_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan4_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan5_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan6_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan7_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan8_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan9_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan10_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan11_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan12_raw & 0xffff)); // uint16_t
  			buffer.put((byte)(rssi & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_RC_INPUTS_RAW { " + 
			"time_usec = " + time_usec + ", " + 
			"chan1_raw = " + chan1_raw + ", " + 
			"chan2_raw = " + chan2_raw + ", " + 
			"chan3_raw = " + chan3_raw + ", " + 
			"chan4_raw = " + chan4_raw + ", " + 
			"chan5_raw = " + chan5_raw + ", " + 
			"chan6_raw = " + chan6_raw + ", " + 
			"chan7_raw = " + chan7_raw + ", " + 
			"chan8_raw = " + chan8_raw + ", " + 
			"chan9_raw = " + chan9_raw + ", " + 
			"chan10_raw = " + chan10_raw + ", " + 
			"chan11_raw = " + chan11_raw + ", " + 
			"chan12_raw = " + chan12_raw + ", " + 
			"rssi = " + rssi + ",  }";
		}
	}
	/*
	 * The IMU readings in SI units in NED body frame
	 */
	public static class MSG_HIL_SENSOR extends Message {
	
		private long time_usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float xacc; // X acceleration (m/s^2)
		private float yacc; // Y acceleration (m/s^2)
		private float zacc; // Z acceleration (m/s^2)
		private float xgyro; // Angular speed around X axis in body frame (rad / sec)
		private float ygyro; // Angular speed around Y axis in body frame (rad / sec)
		private float zgyro; // Angular speed around Z axis in body frame (rad / sec)
		private float xmag; // X Magnetic field (Gauss)
		private float ymag; // Y Magnetic field (Gauss)
		private float zmag; // Z Magnetic field (Gauss)
		private float abs_pressure; // Absolute pressure in millibar
		private float diff_pressure; // Differential pressure (airspeed) in millibar
		private float pressure_alt; // Altitude calculated from pressure
		private float temperature; // Temperature in degrees celsius
		private long fields_updated; // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
	
		public MSG_HIL_SENSOR () {
			super(MSG_ID_HIL_SENSOR, (short)120, "HIL_SENSOR");
		}

		public MSG_HIL_SENSOR (byte[] bytes) {
			super(bytes, MSG_ID_HIL_SENSOR, (short)120, "HIL_SENSOR");
		}
	
		public MSG_HIL_SENSOR (short sys, short comp, long time_usec  , float xacc  , float yacc  , float zacc  , float xgyro  , float ygyro  , float zgyro  , float xmag  , float ymag  , float zmag  , float abs_pressure  , float diff_pressure  , float pressure_alt  , float temperature  , long fields_updated  ) {
			super(sys, comp, (short)120, "HIL_SENSOR");
			this.time_usec = time_usec;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
			this.abs_pressure = abs_pressure;
			this.diff_pressure = diff_pressure;
			this.pressure_alt = pressure_alt;
			this.temperature = temperature;
			this.fields_updated = fields_updated;
		}

		@Override
		public int getCRCExtra() {
			return 108;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getXacc() {
			return xacc;
		}
		
		public void setXacc(float xacc) {
			this.xacc = xacc;
		}
		
		public float getYacc() {
			return yacc;
		}
		
		public void setYacc(float yacc) {
			this.yacc = yacc;
		}
		
		public float getZacc() {
			return zacc;
		}
		
		public void setZacc(float zacc) {
			this.zacc = zacc;
		}
		
		public float getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(float xgyro) {
			this.xgyro = xgyro;
		}
		
		public float getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(float ygyro) {
			this.ygyro = ygyro;
		}
		
		public float getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(float zgyro) {
			this.zgyro = zgyro;
		}
		
		public float getXmag() {
			return xmag;
		}
		
		public void setXmag(float xmag) {
			this.xmag = xmag;
		}
		
		public float getYmag() {
			return ymag;
		}
		
		public void setYmag(float ymag) {
			this.ymag = ymag;
		}
		
		public float getZmag() {
			return zmag;
		}
		
		public void setZmag(float zmag) {
			this.zmag = zmag;
		}
		
		public float getAbs_pressure() {
			return abs_pressure;
		}
		
		public void setAbs_pressure(float abs_pressure) {
			this.abs_pressure = abs_pressure;
		}
		
		public float getDiff_pressure() {
			return diff_pressure;
		}
		
		public void setDiff_pressure(float diff_pressure) {
			this.diff_pressure = diff_pressure;
		}
		
		public float getPressure_alt() {
			return pressure_alt;
		}
		
		public void setPressure_alt(float pressure_alt) {
			this.pressure_alt = pressure_alt;
		}
		
		public float getTemperature() {
			return temperature;
		}
		
		public void setTemperature(float temperature) {
			this.temperature = temperature;
		}
		
		public long getFields_updated() {
			return fields_updated;
		}
		
		public void setFields_updated(long fields_updated) {
			this.fields_updated = fields_updated;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_SENSOR";
			
			time_usec = buffer.getLong(); // uint64_t
  			xacc = buffer.getFloat(); // float
  			yacc = buffer.getFloat(); // float
  			zacc = buffer.getFloat(); // float
  			xgyro = buffer.getFloat(); // float
  			ygyro = buffer.getFloat(); // float
  			zgyro = buffer.getFloat(); // float
  			xmag = buffer.getFloat(); // float
  			ymag = buffer.getFloat(); // float
  			zmag = buffer.getFloat(); // float
  			abs_pressure = buffer.getFloat(); // float
  			diff_pressure = buffer.getFloat(); // float
  			pressure_alt = buffer.getFloat(); // float
  			temperature = buffer.getFloat(); // float
  			fields_updated = buffer.getInt() & 0xffffffff; // uint32_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(xacc); // float
  			buffer.putFloat(yacc); // float
  			buffer.putFloat(zacc); // float
  			buffer.putFloat(xgyro); // float
  			buffer.putFloat(ygyro); // float
  			buffer.putFloat(zgyro); // float
  			buffer.putFloat(xmag); // float
  			buffer.putFloat(ymag); // float
  			buffer.putFloat(zmag); // float
  			buffer.putFloat(abs_pressure); // float
  			buffer.putFloat(diff_pressure); // float
  			buffer.putFloat(pressure_alt); // float
  			buffer.putFloat(temperature); // float
  			buffer.putInt((int)(fields_updated & 0xffffffff)); // uint32_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_SENSOR { " + 
			"time_usec = " + time_usec + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ", " + 
			"abs_pressure = " + abs_pressure + ", " + 
			"diff_pressure = " + diff_pressure + ", " + 
			"pressure_alt = " + pressure_alt + ", " + 
			"temperature = " + temperature + ", " + 
			"fields_updated = " + fields_updated + ",  }";
		}
	}
	/*
	 * DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
	 */
	public static class MSG_HIL_STATE extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private float roll; // Roll angle (rad)
		private float pitch; // Pitch angle (rad)
		private float yaw; // Yaw angle (rad)
		private float rollspeed; // Body frame roll / phi angular speed (rad/s)
		private float pitchspeed; // Body frame pitch / theta angular speed (rad/s)
		private float yawspeed; // Body frame yaw / psi angular speed (rad/s)
		private int lat; // Latitude, expressed as * 1E7
		private int lon; // Longitude, expressed as * 1E7
		private int alt; // Altitude in meters, expressed as * 1000 (millimeters)
		private int vx; // Ground X Speed (Latitude), expressed as m/s * 100
		private int vy; // Ground Y Speed (Longitude), expressed as m/s * 100
		private int vz; // Ground Z Speed (Altitude), expressed as m/s * 100
		private int xacc; // X acceleration (mg)
		private int yacc; // Y acceleration (mg)
		private int zacc; // Z acceleration (mg)
	
		public MSG_HIL_STATE () {
			super(MSG_ID_HIL_STATE, (short)112, "HIL_STATE");
		}

		public MSG_HIL_STATE (byte[] bytes) {
			super(bytes, MSG_ID_HIL_STATE, (short)112, "HIL_STATE");
		}
	
		public MSG_HIL_STATE (short sys, short comp, long time_usec  , float roll  , float pitch  , float yaw  , float rollspeed  , float pitchspeed  , float yawspeed  , int lat  , int lon  , int alt  , int vx  , int vy  , int vz  , int xacc  , int yacc  , int zacc  ) {
			super(sys, comp, (short)112, "HIL_STATE");
			this.time_usec = time_usec;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.rollspeed = rollspeed;
			this.pitchspeed = pitchspeed;
			this.yawspeed = yawspeed;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
		}

		@Override
		public int getCRCExtra() {
			return 183;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getRollspeed() {
			return rollspeed;
		}
		
		public void setRollspeed(float rollspeed) {
			this.rollspeed = rollspeed;
		}
		
		public float getPitchspeed() {
			return pitchspeed;
		}
		
		public void setPitchspeed(float pitchspeed) {
			this.pitchspeed = pitchspeed;
		}
		
		public float getYawspeed() {
			return yawspeed;
		}
		
		public void setYawspeed(float yawspeed) {
			this.yawspeed = yawspeed;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getVx() {
			return vx;
		}
		
		public void setVx(int vx) {
			this.vx = vx;
		}
		
		public int getVy() {
			return vy;
		}
		
		public void setVy(int vy) {
			this.vy = vy;
		}
		
		public int getVz() {
			return vz;
		}
		
		public void setVz(int vz) {
			this.vz = vz;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_STATE";
			
			time_usec = buffer.getLong(); // uint64_t
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			rollspeed = buffer.getFloat(); // float
  			pitchspeed = buffer.getFloat(); // float
  			yawspeed = buffer.getFloat(); // float
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			vx = buffer.getShort(); // int16_t
  			vy = buffer.getShort(); // int16_t
  			vz = buffer.getShort(); // int16_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(rollspeed); // float
  			buffer.putFloat(pitchspeed); // float
  			buffer.putFloat(yawspeed); // float
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putShort((short)(vx)); // int16_t
  			buffer.putShort((short)(vy)); // int16_t
  			buffer.putShort((short)(vz)); // int16_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_STATE { " + 
			"time_usec = " + time_usec + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"rollspeed = " + rollspeed + ", " + 
			"pitchspeed = " + pitchspeed + ", " + 
			"yawspeed = " + yawspeed + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ",  }";
		}
	}
	/*
	 * Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
	 */
	public static class MSG_HIL_STATE_QUATERNION extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private float[] attitude_quaternion = new float[4]; // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
		private float rollspeed; // Body frame roll / phi angular speed (rad/s)
		private float pitchspeed; // Body frame pitch / theta angular speed (rad/s)
		private float yawspeed; // Body frame yaw / psi angular speed (rad/s)
		private int lat; // Latitude, expressed as * 1E7
		private int lon; // Longitude, expressed as * 1E7
		private int alt; // Altitude in meters, expressed as * 1000 (millimeters)
		private int vx; // Ground X Speed (Latitude), expressed as m/s * 100
		private int vy; // Ground Y Speed (Longitude), expressed as m/s * 100
		private int vz; // Ground Z Speed (Altitude), expressed as m/s * 100
		private int ind_airspeed; // Indicated airspeed, expressed as m/s * 100
		private int true_airspeed; // True airspeed, expressed as m/s * 100
		private int xacc; // X acceleration (mg)
		private int yacc; // Y acceleration (mg)
		private int zacc; // Z acceleration (mg)
	
		public MSG_HIL_STATE_QUATERNION () {
			super(MSG_ID_HIL_STATE_QUATERNION, (short)108, "HIL_STATE_QUATERNION");
		}

		public MSG_HIL_STATE_QUATERNION (byte[] bytes) {
			super(bytes, MSG_ID_HIL_STATE_QUATERNION, (short)108, "HIL_STATE_QUATERNION");
		}
	
		public MSG_HIL_STATE_QUATERNION (short sys, short comp, long time_usec  , float attitude_quaternion [] , float rollspeed  , float pitchspeed  , float yawspeed  , int lat  , int lon  , int alt  , int vx  , int vy  , int vz  , int ind_airspeed  , int true_airspeed  , int xacc  , int yacc  , int zacc  ) {
			super(sys, comp, (short)108, "HIL_STATE_QUATERNION");
			this.time_usec = time_usec;
			this.attitude_quaternion = attitude_quaternion;
			this.rollspeed = rollspeed;
			this.pitchspeed = pitchspeed;
			this.yawspeed = yawspeed;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.ind_airspeed = ind_airspeed;
			this.true_airspeed = true_airspeed;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
		}

		@Override
		public int getCRCExtra() {
			return 4;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float[] getAttitude_quaternion() {
			return attitude_quaternion;
		}
		
		public void setAttitude_quaternion(float attitude_quaternion[]) {
			this.attitude_quaternion = attitude_quaternion;
		}
		
		public float getRollspeed() {
			return rollspeed;
		}
		
		public void setRollspeed(float rollspeed) {
			this.rollspeed = rollspeed;
		}
		
		public float getPitchspeed() {
			return pitchspeed;
		}
		
		public void setPitchspeed(float pitchspeed) {
			this.pitchspeed = pitchspeed;
		}
		
		public float getYawspeed() {
			return yawspeed;
		}
		
		public void setYawspeed(float yawspeed) {
			this.yawspeed = yawspeed;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getAlt() {
			return alt;
		}
		
		public void setAlt(int alt) {
			this.alt = alt;
		}
		
		public int getVx() {
			return vx;
		}
		
		public void setVx(int vx) {
			this.vx = vx;
		}
		
		public int getVy() {
			return vy;
		}
		
		public void setVy(int vy) {
			this.vy = vy;
		}
		
		public int getVz() {
			return vz;
		}
		
		public void setVz(int vz) {
			this.vz = vz;
		}
		
		public int getInd_airspeed() {
			return ind_airspeed;
		}
		
		public void setInd_airspeed(int ind_airspeed) {
			this.ind_airspeed = ind_airspeed;
		}
		
		public int getTrue_airspeed() {
			return true_airspeed;
		}
		
		public void setTrue_airspeed(int true_airspeed) {
			this.true_airspeed = true_airspeed;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "HIL_STATE_QUATERNION";
			
			time_usec = buffer.getLong(); // uint64_t
  			for(int c=0; c<4; ++c) {
				attitude_quaternion [c] = buffer.getFloat(); // float[4]
 			}
			
 			rollspeed = buffer.getFloat(); // float
  			pitchspeed = buffer.getFloat(); // float
  			yawspeed = buffer.getFloat(); // float
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			alt = buffer.getInt(); // int32_t
  			vx = buffer.getShort(); // int16_t
  			vy = buffer.getShort(); // int16_t
  			vz = buffer.getShort(); // int16_t
  			ind_airspeed = buffer.getShort() & 0xffff; // uint16_t
  			true_airspeed = buffer.getShort() & 0xffff; // uint16_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			for(int c=0; c<4; ++c) {
				buffer.putFloat(attitude_quaternion [c]); // float[4]
 			}
			
 			buffer.putFloat(rollspeed); // float
  			buffer.putFloat(pitchspeed); // float
  			buffer.putFloat(yawspeed); // float
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putInt((int)(alt)); // int32_t
  			buffer.putShort((short)(vx)); // int16_t
  			buffer.putShort((short)(vy)); // int16_t
  			buffer.putShort((short)(vz)); // int16_t
  			buffer.putShort((short)(ind_airspeed & 0xffff)); // uint16_t
  			buffer.putShort((short)(true_airspeed & 0xffff)); // uint16_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_HIL_STATE_QUATERNION { " + 
			"time_usec = " + time_usec + ", " + 
			"attitude_quaternion = " + attitude_quaternion + ", " + 
			"rollspeed = " + rollspeed + ", " + 
			"pitchspeed = " + pitchspeed + ", " + 
			"yawspeed = " + yawspeed + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"ind_airspeed = " + ind_airspeed + ", " + 
			"true_airspeed = " + true_airspeed + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ",  }";
		}
	}
	public static class MSG_IMAGE_AVAILABLE extends Message {
	
		private long cam_id; // Camera id
		private long timestamp; // Timestamp
		private long valid_until; // Until which timestamp this buffer will stay valid
		private long img_seq; // The image sequence number
		private long img_buf_index; // Position of the image in the buffer, starts with 0
		private long key; // Shared memory area key
		private long exposure; // Exposure time, in microseconds
		private float gain; // Camera gain
		private float roll; // Roll angle in rad
		private float pitch; // Pitch angle in rad
		private float yaw; // Yaw angle in rad
		private float local_z; // Local frame Z coordinate (height over ground)
		private float lat; // GPS X coordinate
		private float lon; // GPS Y coordinate
		private float alt; // Global frame altitude
		private float ground_x; // Ground truth X
		private float ground_y; // Ground truth Y
		private float ground_z; // Ground truth Z
		private int width; // Image width
		private int height; // Image height
		private int depth; // Image depth
		private int cam_no; // Camera # (starts with 0)
		private int channels; // Image channels
	
		public MSG_IMAGE_AVAILABLE () {
			super(MSG_ID_IMAGE_AVAILABLE, (short)260, "IMAGE_AVAILABLE");
		}

		public MSG_IMAGE_AVAILABLE (byte[] bytes) {
			super(bytes, MSG_ID_IMAGE_AVAILABLE, (short)260, "IMAGE_AVAILABLE");
		}
	
		public MSG_IMAGE_AVAILABLE (short sys, short comp, long cam_id  , long timestamp  , long valid_until  , long img_seq  , long img_buf_index  , long key  , long exposure  , float gain  , float roll  , float pitch  , float yaw  , float local_z  , float lat  , float lon  , float alt  , float ground_x  , float ground_y  , float ground_z  , int width  , int height  , int depth  , int cam_no  , int channels  ) {
			super(sys, comp, (short)260, "IMAGE_AVAILABLE");
			this.cam_id = cam_id;
			this.timestamp = timestamp;
			this.valid_until = valid_until;
			this.img_seq = img_seq;
			this.img_buf_index = img_buf_index;
			this.key = key;
			this.exposure = exposure;
			this.gain = gain;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.local_z = local_z;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.ground_x = ground_x;
			this.ground_y = ground_y;
			this.ground_z = ground_z;
			this.width = width;
			this.height = height;
			this.depth = depth;
			this.cam_no = cam_no;
			this.channels = channels;
		}

		@Override
		public int getCRCExtra() {
			return 224;
		}
	
		public long getCam_id() {
			return cam_id;
		}
		
		public void setCam_id(long cam_id) {
			this.cam_id = cam_id;
		}
		
		public long getTimestamp() {
			return timestamp;
		}
		
		public void setTimestamp(long timestamp) {
			this.timestamp = timestamp;
		}
		
		public long getValid_until() {
			return valid_until;
		}
		
		public void setValid_until(long valid_until) {
			this.valid_until = valid_until;
		}
		
		public long getImg_seq() {
			return img_seq;
		}
		
		public void setImg_seq(long img_seq) {
			this.img_seq = img_seq;
		}
		
		public long getImg_buf_index() {
			return img_buf_index;
		}
		
		public void setImg_buf_index(long img_buf_index) {
			this.img_buf_index = img_buf_index;
		}
		
		public long getKey() {
			return key;
		}
		
		public void setKey(long key) {
			this.key = key;
		}
		
		public long getExposure() {
			return exposure;
		}
		
		public void setExposure(long exposure) {
			this.exposure = exposure;
		}
		
		public float getGain() {
			return gain;
		}
		
		public void setGain(float gain) {
			this.gain = gain;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getLocal_z() {
			return local_z;
		}
		
		public void setLocal_z(float local_z) {
			this.local_z = local_z;
		}
		
		public float getLat() {
			return lat;
		}
		
		public void setLat(float lat) {
			this.lat = lat;
		}
		
		public float getLon() {
			return lon;
		}
		
		public void setLon(float lon) {
			this.lon = lon;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getGround_x() {
			return ground_x;
		}
		
		public void setGround_x(float ground_x) {
			this.ground_x = ground_x;
		}
		
		public float getGround_y() {
			return ground_y;
		}
		
		public void setGround_y(float ground_y) {
			this.ground_y = ground_y;
		}
		
		public float getGround_z() {
			return ground_z;
		}
		
		public void setGround_z(float ground_z) {
			this.ground_z = ground_z;
		}
		
		public int getWidth() {
			return width;
		}
		
		public void setWidth(int width) {
			this.width = width;
		}
		
		public int getHeight() {
			return height;
		}
		
		public void setHeight(int height) {
			this.height = height;
		}
		
		public int getDepth() {
			return depth;
		}
		
		public void setDepth(int depth) {
			this.depth = depth;
		}
		
		public int getCam_no() {
			return cam_no;
		}
		
		public void setCam_no(int cam_no) {
			this.cam_no = cam_no;
		}
		
		public int getChannels() {
			return channels;
		}
		
		public void setChannels(int channels) {
			this.channels = channels;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "IMAGE_AVAILABLE";
			
			cam_id = buffer.getLong(); // uint64_t
  			timestamp = buffer.getLong(); // uint64_t
  			valid_until = buffer.getLong(); // uint64_t
  			img_seq = buffer.getInt() & 0xffffffff; // uint32_t
  			img_buf_index = buffer.getInt() & 0xffffffff; // uint32_t
  			key = buffer.getInt() & 0xffffffff; // uint32_t
  			exposure = buffer.getInt() & 0xffffffff; // uint32_t
  			gain = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			local_z = buffer.getFloat(); // float
  			lat = buffer.getFloat(); // float
  			lon = buffer.getFloat(); // float
  			alt = buffer.getFloat(); // float
  			ground_x = buffer.getFloat(); // float
  			ground_y = buffer.getFloat(); // float
  			ground_z = buffer.getFloat(); // float
  			width = buffer.getShort() & 0xffff; // uint16_t
  			height = buffer.getShort() & 0xffff; // uint16_t
  			depth = buffer.getShort() & 0xffff; // uint16_t
  			cam_no = (int)buffer.get() & 0xff; // uint8_t
  			channels = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(cam_id); // uint64_t
  			buffer.putLong(timestamp); // uint64_t
  			buffer.putLong(valid_until); // uint64_t
  			buffer.putInt((int)(img_seq & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(img_buf_index & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(key & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(exposure & 0xffffffff)); // uint32_t
  			buffer.putFloat(gain); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(local_z); // float
  			buffer.putFloat(lat); // float
  			buffer.putFloat(lon); // float
  			buffer.putFloat(alt); // float
  			buffer.putFloat(ground_x); // float
  			buffer.putFloat(ground_y); // float
  			buffer.putFloat(ground_z); // float
  			buffer.putShort((short)(width & 0xffff)); // uint16_t
  			buffer.putShort((short)(height & 0xffff)); // uint16_t
  			buffer.putShort((short)(depth & 0xffff)); // uint16_t
  			buffer.put((byte)(cam_no & 0xff)); // uint8_t
  			buffer.put((byte)(channels & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_IMAGE_AVAILABLE { " + 
			"cam_id = " + cam_id + ", " + 
			"timestamp = " + timestamp + ", " + 
			"valid_until = " + valid_until + ", " + 
			"img_seq = " + img_seq + ", " + 
			"img_buf_index = " + img_buf_index + ", " + 
			"key = " + key + ", " + 
			"exposure = " + exposure + ", " + 
			"gain = " + gain + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"local_z = " + local_z + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"ground_x = " + ground_x + ", " + 
			"ground_y = " + ground_y + ", " + 
			"ground_z = " + ground_z + ", " + 
			"width = " + width + ", " + 
			"height = " + height + ", " + 
			"depth = " + depth + ", " + 
			"cam_no = " + cam_no + ", " + 
			"channels = " + channels + ",  }";
		}
	}
	public static class MSG_IMAGE_TRIGGERED extends Message {
	
		private long timestamp; // Timestamp
		private long seq; // IMU seq
		private float roll; // Roll angle in rad
		private float pitch; // Pitch angle in rad
		private float yaw; // Yaw angle in rad
		private float local_z; // Local frame Z coordinate (height over ground)
		private float lat; // GPS X coordinate
		private float lon; // GPS Y coordinate
		private float alt; // Global frame altitude
		private float ground_x; // Ground truth X
		private float ground_y; // Ground truth Y
		private float ground_z; // Ground truth Z
	
		public MSG_IMAGE_TRIGGERED () {
			super(MSG_ID_IMAGE_TRIGGERED, (short)108, "IMAGE_TRIGGERED");
		}

		public MSG_IMAGE_TRIGGERED (byte[] bytes) {
			super(bytes, MSG_ID_IMAGE_TRIGGERED, (short)108, "IMAGE_TRIGGERED");
		}
	
		public MSG_IMAGE_TRIGGERED (short sys, short comp, long timestamp  , long seq  , float roll  , float pitch  , float yaw  , float local_z  , float lat  , float lon  , float alt  , float ground_x  , float ground_y  , float ground_z  ) {
			super(sys, comp, (short)108, "IMAGE_TRIGGERED");
			this.timestamp = timestamp;
			this.seq = seq;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.local_z = local_z;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.ground_x = ground_x;
			this.ground_y = ground_y;
			this.ground_z = ground_z;
		}

		@Override
		public int getCRCExtra() {
			return 86;
		}
	
		public long getTimestamp() {
			return timestamp;
		}
		
		public void setTimestamp(long timestamp) {
			this.timestamp = timestamp;
		}
		
		public long getSeq() {
			return seq;
		}
		
		public void setSeq(long seq) {
			this.seq = seq;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getLocal_z() {
			return local_z;
		}
		
		public void setLocal_z(float local_z) {
			this.local_z = local_z;
		}
		
		public float getLat() {
			return lat;
		}
		
		public void setLat(float lat) {
			this.lat = lat;
		}
		
		public float getLon() {
			return lon;
		}
		
		public void setLon(float lon) {
			this.lon = lon;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getGround_x() {
			return ground_x;
		}
		
		public void setGround_x(float ground_x) {
			this.ground_x = ground_x;
		}
		
		public float getGround_y() {
			return ground_y;
		}
		
		public void setGround_y(float ground_y) {
			this.ground_y = ground_y;
		}
		
		public float getGround_z() {
			return ground_z;
		}
		
		public void setGround_z(float ground_z) {
			this.ground_z = ground_z;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "IMAGE_TRIGGERED";
			
			timestamp = buffer.getLong(); // uint64_t
  			seq = buffer.getInt() & 0xffffffff; // uint32_t
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			local_z = buffer.getFloat(); // float
  			lat = buffer.getFloat(); // float
  			lon = buffer.getFloat(); // float
  			alt = buffer.getFloat(); // float
  			ground_x = buffer.getFloat(); // float
  			ground_y = buffer.getFloat(); // float
  			ground_z = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(timestamp); // uint64_t
  			buffer.putInt((int)(seq & 0xffffffff)); // uint32_t
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(local_z); // float
  			buffer.putFloat(lat); // float
  			buffer.putFloat(lon); // float
  			buffer.putFloat(alt); // float
  			buffer.putFloat(ground_x); // float
  			buffer.putFloat(ground_y); // float
  			buffer.putFloat(ground_z); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_IMAGE_TRIGGERED { " + 
			"timestamp = " + timestamp + ", " + 
			"seq = " + seq + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"local_z = " + local_z + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"ground_x = " + ground_x + ", " + 
			"ground_y = " + ground_y + ", " + 
			"ground_z = " + ground_z + ",  }";
		}
	}
	public static class MSG_IMAGE_TRIGGER_CONTROL extends Message {
	
		private int enable; // 0 to disable, 1 to enable
	
		public MSG_IMAGE_TRIGGER_CONTROL () {
			super(MSG_ID_IMAGE_TRIGGER_CONTROL, (short)1, "IMAGE_TRIGGER_CONTROL");
		}

		public MSG_IMAGE_TRIGGER_CONTROL (byte[] bytes) {
			super(bytes, MSG_ID_IMAGE_TRIGGER_CONTROL, (short)1, "IMAGE_TRIGGER_CONTROL");
		}
	
		public MSG_IMAGE_TRIGGER_CONTROL (short sys, short comp, int enable  ) {
			super(sys, comp, (short)1, "IMAGE_TRIGGER_CONTROL");
			this.enable = enable;
		}

		@Override
		public int getCRCExtra() {
			return 95;
		}
	
		public int getEnable() {
			return enable;
		}
		
		public void setEnable(int enable) {
			this.enable = enable;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "IMAGE_TRIGGER_CONTROL";
			
			enable = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(enable & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_IMAGE_TRIGGER_CONTROL { " + 
			"enable = " + enable + ",  }";
		}
	}
	/*
	 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
	 */
	public static class MSG_LOCAL_POSITION_NED extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float x; // X Position
		private float y; // Y Position
		private float z; // Z Position
		private float vx; // X Speed
		private float vy; // Y Speed
		private float vz; // Z Speed
	
		public MSG_LOCAL_POSITION_NED () {
			super(MSG_ID_LOCAL_POSITION_NED, (short)28, "LOCAL_POSITION_NED");
		}

		public MSG_LOCAL_POSITION_NED (byte[] bytes) {
			super(bytes, MSG_ID_LOCAL_POSITION_NED, (short)28, "LOCAL_POSITION_NED");
		}
	
		public MSG_LOCAL_POSITION_NED (short sys, short comp, long time_boot_ms  , float x  , float y  , float z  , float vx  , float vy  , float vz  ) {
			super(sys, comp, (short)28, "LOCAL_POSITION_NED");
			this.time_boot_ms = time_boot_ms;
			this.x = x;
			this.y = y;
			this.z = z;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
		}

		@Override
		public int getCRCExtra() {
			return 185;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOCAL_POSITION_NED";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOCAL_POSITION_NED { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ",  }";
		}
	}
	/*
	 * The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
	 */
	public static class MSG_LOCAL_POSITION_NED_COV extends Message {
	
		private long time_utc; // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
		private long time_boot_ms; // Timestamp (milliseconds since system boot). 0 for system without monotonic timestamp
		private float x; // X Position
		private float y; // Y Position
		private float z; // Z Position
		private float vx; // X Speed (m/s)
		private float vy; // Y Speed (m/s)
		private float vz; // Z Speed (m/s)
		private float ax; // X Acceleration (m/s^2)
		private float ay; // Y Acceleration (m/s^2)
		private float az; // Z Acceleration (m/s^2)
		private float[] covariance = new float[45]; // Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
		private int estimator_type; // Class id of the estimator this estimate originated from.
	
		public MSG_LOCAL_POSITION_NED_COV () {
			super(MSG_ID_LOCAL_POSITION_NED_COV, (short)109, "LOCAL_POSITION_NED_COV");
		}

		public MSG_LOCAL_POSITION_NED_COV (byte[] bytes) {
			super(bytes, MSG_ID_LOCAL_POSITION_NED_COV, (short)109, "LOCAL_POSITION_NED_COV");
		}
	
		public MSG_LOCAL_POSITION_NED_COV (short sys, short comp, long time_utc  , long time_boot_ms  , float x  , float y  , float z  , float vx  , float vy  , float vz  , float ax  , float ay  , float az  , float covariance [] , int estimator_type  ) {
			super(sys, comp, (short)109, "LOCAL_POSITION_NED_COV");
			this.time_utc = time_utc;
			this.time_boot_ms = time_boot_ms;
			this.x = x;
			this.y = y;
			this.z = z;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.ax = ax;
			this.ay = ay;
			this.az = az;
			this.covariance = covariance;
			this.estimator_type = estimator_type;
		}

		@Override
		public int getCRCExtra() {
			return 59;
		}
	
		public long getTime_utc() {
			return time_utc;
		}
		
		public void setTime_utc(long time_utc) {
			this.time_utc = time_utc;
		}
		
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float getAx() {
			return ax;
		}
		
		public void setAx(float ax) {
			this.ax = ax;
		}
		
		public float getAy() {
			return ay;
		}
		
		public void setAy(float ay) {
			this.ay = ay;
		}
		
		public float getAz() {
			return az;
		}
		
		public void setAz(float az) {
			this.az = az;
		}
		
		public float[] getCovariance() {
			return covariance;
		}
		
		public void setCovariance(float covariance[]) {
			this.covariance = covariance;
		}
		
		public int getEstimator_type() {
			return estimator_type;
		}
		
		public void setEstimator_type(int estimator_type) {
			this.estimator_type = estimator_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOCAL_POSITION_NED_COV";
			
			time_utc = buffer.getLong(); // uint64_t
  			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			ax = buffer.getFloat(); // float
  			ay = buffer.getFloat(); // float
  			az = buffer.getFloat(); // float
  			for(int c=0; c<45; ++c) {
				covariance [c] = buffer.getFloat(); // float[45]
 			}
			
 			estimator_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_utc); // uint64_t
  			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			buffer.putFloat(ax); // float
  			buffer.putFloat(ay); // float
  			buffer.putFloat(az); // float
  			for(int c=0; c<45; ++c) {
				buffer.putFloat(covariance [c]); // float[45]
 			}
			
 			buffer.put((byte)(estimator_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOCAL_POSITION_NED_COV { " + 
			"time_utc = " + time_utc + ", " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"ax = " + ax + ", " + 
			"ay = " + ay + ", " + 
			"az = " + az + ", " + 
			"covariance = " + covariance + ", " + 
			"estimator_type = " + estimator_type + ",  }";
		}
	}
	/*
	 * The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
	 */
	public static class MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float x; // X Position
		private float y; // Y Position
		private float z; // Z Position
		private float roll; // Roll
		private float pitch; // Pitch
		private float yaw; // Yaw
	
		public MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET () {
			super(MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, (short)28, "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET");
		}

		public MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET (byte[] bytes) {
			super(bytes, MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, (short)28, "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET");
		}
	
		public MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET (short sys, short comp, long time_boot_ms  , float x  , float y  , float z  , float roll  , float pitch  , float yaw  ) {
			super(sys, comp, (short)28, "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET");
			this.time_boot_ms = time_boot_ms;
			this.x = x;
			this.y = y;
			this.z = z;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
		}

		@Override
		public int getCRCExtra() {
			return 231;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ",  }";
		}
	}
	/*
	 * Reply to LOG_REQUEST_DATA
	 */
	public static class MSG_LOG_DATA extends Message {
	
		private long ofs; // Offset into the log
		private int id; // Log id (from LOG_ENTRY reply)
		private int count; // Number of bytes (zero for end of log)
		private int[] data = new int[90]; // log data
	
		public MSG_LOG_DATA () {
			super(MSG_ID_LOG_DATA, (short)8, "LOG_DATA");
		}

		public MSG_LOG_DATA (byte[] bytes) {
			super(bytes, MSG_ID_LOG_DATA, (short)8, "LOG_DATA");
		}
	
		public MSG_LOG_DATA (short sys, short comp, long ofs  , int id  , int count  , int data [] ) {
			super(sys, comp, (short)8, "LOG_DATA");
			this.ofs = ofs;
			this.id = id;
			this.count = count;
			this.data = data;
		}

		@Override
		public int getCRCExtra() {
			return 134;
		}
	
		public long getOfs() {
			return ofs;
		}
		
		public void setOfs(long ofs) {
			this.ofs = ofs;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
		public int getCount() {
			return count;
		}
		
		public void setCount(int count) {
			this.count = count;
		}
		
		public int[] getData() {
			return data;
		}
		
		public void setData(int data[]) {
			this.data = data;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_DATA";
			
			ofs = buffer.getInt() & 0xffffffff; // uint32_t
  			id = buffer.getShort() & 0xffff; // uint16_t
  			count = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<90; ++c) {
				data [c] =  (int)buffer.get() & 0xff; // uint8_t[90]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(ofs & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(id & 0xffff)); // uint16_t
  			buffer.put((byte)(count & 0xff)); // uint8_t
  			for(int c=0; c<90; ++c) {
				buffer.put((byte)(data [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_DATA { " + 
			"ofs = " + ofs + ", " + 
			"id = " + id + ", " + 
			"count = " + count + ", " + 
			"data = " + data + ",  }";
		}
	}
	/*
	 * Reply to LOG_REQUEST_LIST
	 */
	public static class MSG_LOG_ENTRY extends Message {
	
		private long time_utc; // UTC timestamp of log in seconds since 1970, or 0 if not available
		private long size; // Size of the log (may be approximate) in bytes
		private int id; // Log id
		private int num_logs; // Total number of logs
		private int last_log_num; // High log number
	
		public MSG_LOG_ENTRY () {
			super(MSG_ID_LOG_ENTRY, (short)14, "LOG_ENTRY");
		}

		public MSG_LOG_ENTRY (byte[] bytes) {
			super(bytes, MSG_ID_LOG_ENTRY, (short)14, "LOG_ENTRY");
		}
	
		public MSG_LOG_ENTRY (short sys, short comp, long time_utc  , long size  , int id  , int num_logs  , int last_log_num  ) {
			super(sys, comp, (short)14, "LOG_ENTRY");
			this.time_utc = time_utc;
			this.size = size;
			this.id = id;
			this.num_logs = num_logs;
			this.last_log_num = last_log_num;
		}

		@Override
		public int getCRCExtra() {
			return 56;
		}
	
		public long getTime_utc() {
			return time_utc;
		}
		
		public void setTime_utc(long time_utc) {
			this.time_utc = time_utc;
		}
		
		public long getSize() {
			return size;
		}
		
		public void setSize(long size) {
			this.size = size;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
		public int getNum_logs() {
			return num_logs;
		}
		
		public void setNum_logs(int num_logs) {
			this.num_logs = num_logs;
		}
		
		public int getLast_log_num() {
			return last_log_num;
		}
		
		public void setLast_log_num(int last_log_num) {
			this.last_log_num = last_log_num;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_ENTRY";
			
			time_utc = buffer.getInt() & 0xffffffff; // uint32_t
  			size = buffer.getInt() & 0xffffffff; // uint32_t
  			id = buffer.getShort() & 0xffff; // uint16_t
  			num_logs = buffer.getShort() & 0xffff; // uint16_t
  			last_log_num = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_utc & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(size & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(id & 0xffff)); // uint16_t
  			buffer.putShort((short)(num_logs & 0xffff)); // uint16_t
  			buffer.putShort((short)(last_log_num & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_ENTRY { " + 
			"time_utc = " + time_utc + ", " + 
			"size = " + size + ", " + 
			"id = " + id + ", " + 
			"num_logs = " + num_logs + ", " + 
			"last_log_num = " + last_log_num + ",  }";
		}
	}
	/*
	 * Erase all logs
	 */
	public static class MSG_LOG_ERASE extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_LOG_ERASE () {
			super(MSG_ID_LOG_ERASE, (short)2, "LOG_ERASE");
		}

		public MSG_LOG_ERASE (byte[] bytes) {
			super(bytes, MSG_ID_LOG_ERASE, (short)2, "LOG_ERASE");
		}
	
		public MSG_LOG_ERASE (short sys, short comp, int target_system  , int target_component  ) {
			super(sys, comp, (short)2, "LOG_ERASE");
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 237;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_ERASE";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_ERASE { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Request a chunk of a log
	 */
	public static class MSG_LOG_REQUEST_DATA extends Message {
	
		private long ofs; // Offset into the log
		private long count; // Number of bytes
		private int id; // Log id (from LOG_ENTRY reply)
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_LOG_REQUEST_DATA () {
			super(MSG_ID_LOG_REQUEST_DATA, (short)12, "LOG_REQUEST_DATA");
		}

		public MSG_LOG_REQUEST_DATA (byte[] bytes) {
			super(bytes, MSG_ID_LOG_REQUEST_DATA, (short)12, "LOG_REQUEST_DATA");
		}
	
		public MSG_LOG_REQUEST_DATA (short sys, short comp, long ofs  , long count  , int id  , int target_system  , int target_component  ) {
			super(sys, comp, (short)12, "LOG_REQUEST_DATA");
			this.ofs = ofs;
			this.count = count;
			this.id = id;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 116;
		}
	
		public long getOfs() {
			return ofs;
		}
		
		public void setOfs(long ofs) {
			this.ofs = ofs;
		}
		
		public long getCount() {
			return count;
		}
		
		public void setCount(long count) {
			this.count = count;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_REQUEST_DATA";
			
			ofs = buffer.getInt() & 0xffffffff; // uint32_t
  			count = buffer.getInt() & 0xffffffff; // uint32_t
  			id = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(ofs & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(count & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(id & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_REQUEST_DATA { " + 
			"ofs = " + ofs + ", " + 
			"count = " + count + ", " + 
			"id = " + id + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Stop log transfer and resume normal logging
	 */
	public static class MSG_LOG_REQUEST_END extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_LOG_REQUEST_END () {
			super(MSG_ID_LOG_REQUEST_END, (short)2, "LOG_REQUEST_END");
		}

		public MSG_LOG_REQUEST_END (byte[] bytes) {
			super(bytes, MSG_ID_LOG_REQUEST_END, (short)2, "LOG_REQUEST_END");
		}
	
		public MSG_LOG_REQUEST_END (short sys, short comp, int target_system  , int target_component  ) {
			super(sys, comp, (short)2, "LOG_REQUEST_END");
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 203;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_REQUEST_END";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_REQUEST_END { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
	 */
	public static class MSG_LOG_REQUEST_LIST extends Message {
	
		private int start; // First log id (0 for first available)
		private int end; // Last log id (0xffff for last available)
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_LOG_REQUEST_LIST () {
			super(MSG_ID_LOG_REQUEST_LIST, (short)6, "LOG_REQUEST_LIST");
		}

		public MSG_LOG_REQUEST_LIST (byte[] bytes) {
			super(bytes, MSG_ID_LOG_REQUEST_LIST, (short)6, "LOG_REQUEST_LIST");
		}
	
		public MSG_LOG_REQUEST_LIST (short sys, short comp, int start  , int end  , int target_system  , int target_component  ) {
			super(sys, comp, (short)6, "LOG_REQUEST_LIST");
			this.start = start;
			this.end = end;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 128;
		}
	
		public int getStart() {
			return start;
		}
		
		public void setStart(int start) {
			this.start = start;
		}
		
		public int getEnd() {
			return end;
		}
		
		public void setEnd(int end) {
			this.end = end;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "LOG_REQUEST_LIST";
			
			start = buffer.getShort() & 0xffff; // uint16_t
  			end = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(start & 0xffff)); // uint16_t
  			buffer.putShort((short)(end & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_LOG_REQUEST_LIST { " + 
			"start = " + start + ", " + 
			"end = " + end + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their 
	 */
	public static class MSG_MANUAL_CONTROL extends Message {
	
		private int x; // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
		private int y; // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
		private int z; // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
		private int r; // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
		private int buttons; // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
		private int target; // The system to be controlled.
	
		public MSG_MANUAL_CONTROL () {
			super(MSG_ID_MANUAL_CONTROL, (short)11, "MANUAL_CONTROL");
		}

		public MSG_MANUAL_CONTROL (byte[] bytes) {
			super(bytes, MSG_ID_MANUAL_CONTROL, (short)11, "MANUAL_CONTROL");
		}
	
		public MSG_MANUAL_CONTROL (short sys, short comp, int x  , int y  , int z  , int r  , int buttons  , int target  ) {
			super(sys, comp, (short)11, "MANUAL_CONTROL");
			this.x = x;
			this.y = y;
			this.z = z;
			this.r = r;
			this.buttons = buttons;
			this.target = target;
		}

		@Override
		public int getCRCExtra() {
			return 243;
		}
	
		public int getX() {
			return x;
		}
		
		public void setX(int x) {
			this.x = x;
		}
		
		public int getY() {
			return y;
		}
		
		public void setY(int y) {
			this.y = y;
		}
		
		public int getZ() {
			return z;
		}
		
		public void setZ(int z) {
			this.z = z;
		}
		
		public int getR() {
			return r;
		}
		
		public void setR(int r) {
			this.r = r;
		}
		
		public int getButtons() {
			return buttons;
		}
		
		public void setButtons(int buttons) {
			this.buttons = buttons;
		}
		
		public int getTarget() {
			return target;
		}
		
		public void setTarget(int target) {
			this.target = target;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MANUAL_CONTROL";
			
			x = buffer.getShort(); // int16_t
  			y = buffer.getShort(); // int16_t
  			z = buffer.getShort(); // int16_t
  			r = buffer.getShort(); // int16_t
  			buttons = buffer.getShort() & 0xffff; // uint16_t
  			target = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(x)); // int16_t
  			buffer.putShort((short)(y)); // int16_t
  			buffer.putShort((short)(z)); // int16_t
  			buffer.putShort((short)(r)); // int16_t
  			buffer.putShort((short)(buttons & 0xffff)); // uint16_t
  			buffer.put((byte)(target & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MANUAL_CONTROL { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"r = " + r + ", " + 
			"buttons = " + buttons + ", " + 
			"target = " + target + ",  }";
		}
	}
	/*
	 * Setpoint in roll, pitch, yaw and thrust from the operator
	 */
	public static class MSG_MANUAL_SETPOINT extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot
		private float roll; // Desired roll rate in radians per second
		private float pitch; // Desired pitch rate in radians per second
		private float yaw; // Desired yaw rate in radians per second
		private float thrust; // Collective thrust, normalized to 0 .. 1
		private int mode_switch; // Flight mode switch position, 0.. 255
		private int manual_override_switch; // Override mode switch position, 0.. 255
	
		public MSG_MANUAL_SETPOINT () {
			super(MSG_ID_MANUAL_SETPOINT, (short)22, "MANUAL_SETPOINT");
		}

		public MSG_MANUAL_SETPOINT (byte[] bytes) {
			super(bytes, MSG_ID_MANUAL_SETPOINT, (short)22, "MANUAL_SETPOINT");
		}
	
		public MSG_MANUAL_SETPOINT (short sys, short comp, long time_boot_ms  , float roll  , float pitch  , float yaw  , float thrust  , int mode_switch  , int manual_override_switch  ) {
			super(sys, comp, (short)22, "MANUAL_SETPOINT");
			this.time_boot_ms = time_boot_ms;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.thrust = thrust;
			this.mode_switch = mode_switch;
			this.manual_override_switch = manual_override_switch;
		}

		@Override
		public int getCRCExtra() {
			return 106;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getThrust() {
			return thrust;
		}
		
		public void setThrust(float thrust) {
			this.thrust = thrust;
		}
		
		public int getMode_switch() {
			return mode_switch;
		}
		
		public void setMode_switch(int mode_switch) {
			this.mode_switch = mode_switch;
		}
		
		public int getManual_override_switch() {
			return manual_override_switch;
		}
		
		public void setManual_override_switch(int manual_override_switch) {
			this.manual_override_switch = manual_override_switch;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MANUAL_SETPOINT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			thrust = buffer.getFloat(); // float
  			mode_switch = (int)buffer.get() & 0xff; // uint8_t
  			manual_override_switch = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(thrust); // float
  			buffer.put((byte)(mode_switch & 0xff)); // uint8_t
  			buffer.put((byte)(manual_override_switch & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MANUAL_SETPOINT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"thrust = " + thrust + ", " + 
			"mode_switch = " + mode_switch + ", " + 
			"manual_override_switch = " + manual_override_switch + ",  }";
		}
	}
	public static class MSG_MARKER extends Message {
	
		private float x; // x position
		private float y; // y position
		private float z; // z position
		private float roll; // roll orientation
		private float pitch; // pitch orientation
		private float yaw; // yaw orientation
		private int id; // ID
	
		public MSG_MARKER () {
			super(MSG_ID_MARKER, (short)26, "MARKER");
		}

		public MSG_MARKER (byte[] bytes) {
			super(bytes, MSG_ID_MARKER, (short)26, "MARKER");
		}
	
		public MSG_MARKER (short sys, short comp, float x  , float y  , float z  , float roll  , float pitch  , float yaw  , int id  ) {
			super(sys, comp, (short)26, "MARKER");
			this.x = x;
			this.y = y;
			this.z = z;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.id = id;
		}

		@Override
		public int getCRCExtra() {
			return 249;
		}
	
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MARKER";
			
			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			id = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putShort((short)(id & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MARKER { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"id = " + id + ",  }";
		}
	}
	/*
	 * Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
	 */
	public static class MSG_MEMORY_VECT extends Message {
	
		private int address; // Starting address of the debug variables
		private int ver; // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
		private int type; // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
		private int[] value = new int[32]; // Memory contents at specified address
	
		public MSG_MEMORY_VECT () {
			super(MSG_ID_MEMORY_VECT, (short)5, "MEMORY_VECT");
		}

		public MSG_MEMORY_VECT (byte[] bytes) {
			super(bytes, MSG_ID_MEMORY_VECT, (short)5, "MEMORY_VECT");
		}
	
		public MSG_MEMORY_VECT (short sys, short comp, int address  , int ver  , int type  , int value [] ) {
			super(sys, comp, (short)5, "MEMORY_VECT");
			this.address = address;
			this.ver = ver;
			this.type = type;
			this.value = value;
		}

		@Override
		public int getCRCExtra() {
			return 204;
		}
	
		public int getAddress() {
			return address;
		}
		
		public void setAddress(int address) {
			this.address = address;
		}
		
		public int getVer() {
			return ver;
		}
		
		public void setVer(int ver) {
			this.ver = ver;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int[] getValue() {
			return value;
		}
		
		public void setValue(int value[]) {
			this.value = value;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MEMORY_VECT";
			
			address = buffer.getShort() & 0xffff; // uint16_t
  			ver = (int)buffer.get() & 0xff; // uint8_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<32; ++c) {
				value [c] =  (int)buffer.get(); // int8_t[32]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(address & 0xffff)); // uint16_t
  			buffer.put((byte)(ver & 0xff)); // uint8_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			for(int c=0; c<32; ++c) {
				buffer.put((byte)(value [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MEMORY_VECT { " + 
			"address = " + address + ", " + 
			"ver = " + ver + ", " + 
			"type = " + type + ", " + 
			"value = " + value + ",  }";
		}
	}
	/*
	 * Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
	 */
	public static class MSG_MISSION_ACK extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
		private int type; // See MAV_MISSION_RESULT enum
	
		public MSG_MISSION_ACK () {
			super(MSG_ID_MISSION_ACK, (short)3, "MISSION_ACK");
		}

		public MSG_MISSION_ACK (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_ACK, (short)3, "MISSION_ACK");
		}
	
		public MSG_MISSION_ACK (short sys, short comp, int target_system  , int target_component  , int type  ) {
			super(sys, comp, (short)3, "MISSION_ACK");
			this.target_system = target_system;
			this.target_component = target_component;
			this.type = type;
		}

		@Override
		public int getCRCExtra() {
			return 153;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_ACK";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_ACK { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"type = " + type + ",  }";
		}
	}
	/*
	 * Delete all mission items at once.
	 */
	public static class MSG_MISSION_CLEAR_ALL extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_CLEAR_ALL () {
			super(MSG_ID_MISSION_CLEAR_ALL, (short)2, "MISSION_CLEAR_ALL");
		}

		public MSG_MISSION_CLEAR_ALL (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_CLEAR_ALL, (short)2, "MISSION_CLEAR_ALL");
		}
	
		public MSG_MISSION_CLEAR_ALL (short sys, short comp, int target_system  , int target_component  ) {
			super(sys, comp, (short)2, "MISSION_CLEAR_ALL");
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 232;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_CLEAR_ALL";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_CLEAR_ALL { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
	 */
	public static class MSG_MISSION_COUNT extends Message {
	
		private int count; // Number of mission items in the sequence
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_COUNT () {
			super(MSG_ID_MISSION_COUNT, (short)4, "MISSION_COUNT");
		}

		public MSG_MISSION_COUNT (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_COUNT, (short)4, "MISSION_COUNT");
		}
	
		public MSG_MISSION_COUNT (short sys, short comp, int count  , int target_system  , int target_component  ) {
			super(sys, comp, (short)4, "MISSION_COUNT");
			this.count = count;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 221;
		}
	
		public int getCount() {
			return count;
		}
		
		public void setCount(int count) {
			this.count = count;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_COUNT";
			
			count = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(count & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_COUNT { " + 
			"count = " + count + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
	 */
	public static class MSG_MISSION_CURRENT extends Message {
	
		private int seq; // Sequence
	
		public MSG_MISSION_CURRENT () {
			super(MSG_ID_MISSION_CURRENT, (short)2, "MISSION_CURRENT");
		}

		public MSG_MISSION_CURRENT (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_CURRENT, (short)2, "MISSION_CURRENT");
		}
	
		public MSG_MISSION_CURRENT (short sys, short comp, int seq  ) {
			super(sys, comp, (short)2, "MISSION_CURRENT");
			this.seq = seq;
		}

		@Override
		public int getCRCExtra() {
			return 28;
		}
	
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_CURRENT";
			
			seq = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(seq & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_CURRENT { " + 
			"seq = " + seq + ",  }";
		}
	}
	/*
	 * Message encoding a mission item. This message is emitted to announce
                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
	 */
	public static class MSG_MISSION_ITEM extends Message {
	
		private float param1; // PARAM1, see MAV_CMD enum
		private float param2; // PARAM2, see MAV_CMD enum
		private float param3; // PARAM3, see MAV_CMD enum
		private float param4; // PARAM4, see MAV_CMD enum
		private float x; // PARAM5 / local: x position, global: latitude
		private float y; // PARAM6 / y position: global: longitude
		private float z; // PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
		private int seq; // Sequence
		private int command; // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		private int target_system; // System ID
		private int target_component; // Component ID
		private int frame; // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		private int current; // false:0, true:1
		private int autocontinue; // autocontinue to next wp
	
		public MSG_MISSION_ITEM () {
			super(MSG_ID_MISSION_ITEM, (short)37, "MISSION_ITEM");
		}

		public MSG_MISSION_ITEM (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_ITEM, (short)37, "MISSION_ITEM");
		}
	
		public MSG_MISSION_ITEM (short sys, short comp, float param1  , float param2  , float param3  , float param4  , float x  , float y  , float z  , int seq  , int command  , int target_system  , int target_component  , int frame  , int current  , int autocontinue  ) {
			super(sys, comp, (short)37, "MISSION_ITEM");
			this.param1 = param1;
			this.param2 = param2;
			this.param3 = param3;
			this.param4 = param4;
			this.x = x;
			this.y = y;
			this.z = z;
			this.seq = seq;
			this.command = command;
			this.target_system = target_system;
			this.target_component = target_component;
			this.frame = frame;
			this.current = current;
			this.autocontinue = autocontinue;
		}

		@Override
		public int getCRCExtra() {
			return 254;
		}
	
		public float getParam1() {
			return param1;
		}
		
		public void setParam1(float param1) {
			this.param1 = param1;
		}
		
		public float getParam2() {
			return param2;
		}
		
		public void setParam2(float param2) {
			this.param2 = param2;
		}
		
		public float getParam3() {
			return param3;
		}
		
		public void setParam3(float param3) {
			this.param3 = param3;
		}
		
		public float getParam4() {
			return param4;
		}
		
		public void setParam4(float param4) {
			this.param4 = param4;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
		public int getCommand() {
			return command;
		}
		
		public void setCommand(int command) {
			this.command = command;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getFrame() {
			return frame;
		}
		
		public void setFrame(int frame) {
			this.frame = frame;
		}
		
		public int getCurrent() {
			return current;
		}
		
		public void setCurrent(int current) {
			this.current = current;
		}
		
		public int getAutocontinue() {
			return autocontinue;
		}
		
		public void setAutocontinue(int autocontinue) {
			this.autocontinue = autocontinue;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_ITEM";
			
			param1 = buffer.getFloat(); // float
  			param2 = buffer.getFloat(); // float
  			param3 = buffer.getFloat(); // float
  			param4 = buffer.getFloat(); // float
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			seq = buffer.getShort() & 0xffff; // uint16_t
  			command = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			frame = (int)buffer.get() & 0xff; // uint8_t
  			current = (int)buffer.get() & 0xff; // uint8_t
  			autocontinue = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param1); // float
  			buffer.putFloat(param2); // float
  			buffer.putFloat(param3); // float
  			buffer.putFloat(param4); // float
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putShort((short)(seq & 0xffff)); // uint16_t
  			buffer.putShort((short)(command & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(frame & 0xff)); // uint8_t
  			buffer.put((byte)(current & 0xff)); // uint8_t
  			buffer.put((byte)(autocontinue & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_ITEM { " + 
			"param1 = " + param1 + ", " + 
			"param2 = " + param2 + ", " + 
			"param3 = " + param3 + ", " + 
			"param4 = " + param4 + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"seq = " + seq + ", " + 
			"command = " + command + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"frame = " + frame + ", " + 
			"current = " + current + ", " + 
			"autocontinue = " + autocontinue + ",  }";
		}
	}
	/*
	 * Message encoding a mission item. This message is emitted to announce
                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
	 */
	public static class MSG_MISSION_ITEM_INT extends Message {
	
		private float param1; // PARAM1, see MAV_CMD enum
		private float param2; // PARAM2, see MAV_CMD enum
		private float param3; // PARAM3, see MAV_CMD enum
		private float param4; // PARAM4, see MAV_CMD enum
		private int x; // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
		private int y; // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
		private float z; // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
		private int seq; // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
		private int command; // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
		private int target_system; // System ID
		private int target_component; // Component ID
		private int frame; // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
		private int current; // false:0, true:1
		private int autocontinue; // autocontinue to next wp
	
		public MSG_MISSION_ITEM_INT () {
			super(MSG_ID_MISSION_ITEM_INT, (short)37, "MISSION_ITEM_INT");
		}

		public MSG_MISSION_ITEM_INT (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_ITEM_INT, (short)37, "MISSION_ITEM_INT");
		}
	
		public MSG_MISSION_ITEM_INT (short sys, short comp, float param1  , float param2  , float param3  , float param4  , int x  , int y  , float z  , int seq  , int command  , int target_system  , int target_component  , int frame  , int current  , int autocontinue  ) {
			super(sys, comp, (short)37, "MISSION_ITEM_INT");
			this.param1 = param1;
			this.param2 = param2;
			this.param3 = param3;
			this.param4 = param4;
			this.x = x;
			this.y = y;
			this.z = z;
			this.seq = seq;
			this.command = command;
			this.target_system = target_system;
			this.target_component = target_component;
			this.frame = frame;
			this.current = current;
			this.autocontinue = autocontinue;
		}

		@Override
		public int getCRCExtra() {
			return 38;
		}
	
		public float getParam1() {
			return param1;
		}
		
		public void setParam1(float param1) {
			this.param1 = param1;
		}
		
		public float getParam2() {
			return param2;
		}
		
		public void setParam2(float param2) {
			this.param2 = param2;
		}
		
		public float getParam3() {
			return param3;
		}
		
		public void setParam3(float param3) {
			this.param3 = param3;
		}
		
		public float getParam4() {
			return param4;
		}
		
		public void setParam4(float param4) {
			this.param4 = param4;
		}
		
		public int getX() {
			return x;
		}
		
		public void setX(int x) {
			this.x = x;
		}
		
		public int getY() {
			return y;
		}
		
		public void setY(int y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
		public int getCommand() {
			return command;
		}
		
		public void setCommand(int command) {
			this.command = command;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getFrame() {
			return frame;
		}
		
		public void setFrame(int frame) {
			this.frame = frame;
		}
		
		public int getCurrent() {
			return current;
		}
		
		public void setCurrent(int current) {
			this.current = current;
		}
		
		public int getAutocontinue() {
			return autocontinue;
		}
		
		public void setAutocontinue(int autocontinue) {
			this.autocontinue = autocontinue;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_ITEM_INT";
			
			param1 = buffer.getFloat(); // float
  			param2 = buffer.getFloat(); // float
  			param3 = buffer.getFloat(); // float
  			param4 = buffer.getFloat(); // float
  			x = buffer.getInt(); // int32_t
  			y = buffer.getInt(); // int32_t
  			z = buffer.getFloat(); // float
  			seq = buffer.getShort() & 0xffff; // uint16_t
  			command = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			frame = (int)buffer.get() & 0xff; // uint8_t
  			current = (int)buffer.get() & 0xff; // uint8_t
  			autocontinue = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param1); // float
  			buffer.putFloat(param2); // float
  			buffer.putFloat(param3); // float
  			buffer.putFloat(param4); // float
  			buffer.putInt((int)(x)); // int32_t
  			buffer.putInt((int)(y)); // int32_t
  			buffer.putFloat(z); // float
  			buffer.putShort((short)(seq & 0xffff)); // uint16_t
  			buffer.putShort((short)(command & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(frame & 0xff)); // uint8_t
  			buffer.put((byte)(current & 0xff)); // uint8_t
  			buffer.put((byte)(autocontinue & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_ITEM_INT { " + 
			"param1 = " + param1 + ", " + 
			"param2 = " + param2 + ", " + 
			"param3 = " + param3 + ", " + 
			"param4 = " + param4 + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"seq = " + seq + ", " + 
			"command = " + command + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"frame = " + frame + ", " + 
			"current = " + current + ", " + 
			"autocontinue = " + autocontinue + ",  }";
		}
	}
	/*
	 * A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
	 */
	public static class MSG_MISSION_ITEM_REACHED extends Message {
	
		private int seq; // Sequence
	
		public MSG_MISSION_ITEM_REACHED () {
			super(MSG_ID_MISSION_ITEM_REACHED, (short)2, "MISSION_ITEM_REACHED");
		}

		public MSG_MISSION_ITEM_REACHED (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_ITEM_REACHED, (short)2, "MISSION_ITEM_REACHED");
		}
	
		public MSG_MISSION_ITEM_REACHED (short sys, short comp, int seq  ) {
			super(sys, comp, (short)2, "MISSION_ITEM_REACHED");
			this.seq = seq;
		}

		@Override
		public int getCRCExtra() {
			return 11;
		}
	
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_ITEM_REACHED";
			
			seq = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(seq & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_ITEM_REACHED { " + 
			"seq = " + seq + ",  }";
		}
	}
	/*
	 * Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
	 */
	public static class MSG_MISSION_REQUEST extends Message {
	
		private int seq; // Sequence
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_REQUEST () {
			super(MSG_ID_MISSION_REQUEST, (short)4, "MISSION_REQUEST");
		}

		public MSG_MISSION_REQUEST (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_REQUEST, (short)4, "MISSION_REQUEST");
		}
	
		public MSG_MISSION_REQUEST (short sys, short comp, int seq  , int target_system  , int target_component  ) {
			super(sys, comp, (short)4, "MISSION_REQUEST");
			this.seq = seq;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 230;
		}
	
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_REQUEST";
			
			seq = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(seq & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_REQUEST { " + 
			"seq = " + seq + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Request the overall list of mission items from the system/component.
	 */
	public static class MSG_MISSION_REQUEST_LIST extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_REQUEST_LIST () {
			super(MSG_ID_MISSION_REQUEST_LIST, (short)2, "MISSION_REQUEST_LIST");
		}

		public MSG_MISSION_REQUEST_LIST (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_REQUEST_LIST, (short)2, "MISSION_REQUEST_LIST");
		}
	
		public MSG_MISSION_REQUEST_LIST (short sys, short comp, int target_system  , int target_component  ) {
			super(sys, comp, (short)2, "MISSION_REQUEST_LIST");
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 132;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_REQUEST_LIST";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_REQUEST_LIST { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
	 */
	public static class MSG_MISSION_REQUEST_PARTIAL_LIST extends Message {
	
		private int start_index; // Start index, 0 by default
		private int end_index; // End index, -1 by default (-1: send list to end). Else a valid index of the list
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_REQUEST_PARTIAL_LIST () {
			super(MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (short)6, "MISSION_REQUEST_PARTIAL_LIST");
		}

		public MSG_MISSION_REQUEST_PARTIAL_LIST (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_REQUEST_PARTIAL_LIST, (short)6, "MISSION_REQUEST_PARTIAL_LIST");
		}
	
		public MSG_MISSION_REQUEST_PARTIAL_LIST (short sys, short comp, int start_index  , int end_index  , int target_system  , int target_component  ) {
			super(sys, comp, (short)6, "MISSION_REQUEST_PARTIAL_LIST");
			this.start_index = start_index;
			this.end_index = end_index;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 212;
		}
	
		public int getStart_index() {
			return start_index;
		}
		
		public void setStart_index(int start_index) {
			this.start_index = start_index;
		}
		
		public int getEnd_index() {
			return end_index;
		}
		
		public void setEnd_index(int end_index) {
			this.end_index = end_index;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_REQUEST_PARTIAL_LIST";
			
			start_index = buffer.getShort(); // int16_t
  			end_index = buffer.getShort(); // int16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(start_index)); // int16_t
  			buffer.putShort((short)(end_index)); // int16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_REQUEST_PARTIAL_LIST { " + 
			"start_index = " + start_index + ", " + 
			"end_index = " + end_index + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
	 */
	public static class MSG_MISSION_SET_CURRENT extends Message {
	
		private int seq; // Sequence
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_SET_CURRENT () {
			super(MSG_ID_MISSION_SET_CURRENT, (short)4, "MISSION_SET_CURRENT");
		}

		public MSG_MISSION_SET_CURRENT (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_SET_CURRENT, (short)4, "MISSION_SET_CURRENT");
		}
	
		public MSG_MISSION_SET_CURRENT (short sys, short comp, int seq  , int target_system  , int target_component  ) {
			super(sys, comp, (short)4, "MISSION_SET_CURRENT");
			this.seq = seq;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 28;
		}
	
		public int getSeq() {
			return seq;
		}
		
		public void setSeq(int seq) {
			this.seq = seq;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_SET_CURRENT";
			
			seq = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(seq & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_SET_CURRENT { " + 
			"seq = " + seq + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
	 */
	public static class MSG_MISSION_WRITE_PARTIAL_LIST extends Message {
	
		private int start_index; // Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
		private int end_index; // End index, equal or greater than start index.
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_MISSION_WRITE_PARTIAL_LIST () {
			super(MSG_ID_MISSION_WRITE_PARTIAL_LIST, (short)6, "MISSION_WRITE_PARTIAL_LIST");
		}

		public MSG_MISSION_WRITE_PARTIAL_LIST (byte[] bytes) {
			super(bytes, MSG_ID_MISSION_WRITE_PARTIAL_LIST, (short)6, "MISSION_WRITE_PARTIAL_LIST");
		}
	
		public MSG_MISSION_WRITE_PARTIAL_LIST (short sys, short comp, int start_index  , int end_index  , int target_system  , int target_component  ) {
			super(sys, comp, (short)6, "MISSION_WRITE_PARTIAL_LIST");
			this.start_index = start_index;
			this.end_index = end_index;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 9;
		}
	
		public int getStart_index() {
			return start_index;
		}
		
		public void setStart_index(int start_index) {
			this.start_index = start_index;
		}
		
		public int getEnd_index() {
			return end_index;
		}
		
		public void setEnd_index(int end_index) {
			this.end_index = end_index;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "MISSION_WRITE_PARTIAL_LIST";
			
			start_index = buffer.getShort(); // int16_t
  			end_index = buffer.getShort(); // int16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(start_index)); // int16_t
  			buffer.putShort((short)(end_index)); // int16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_MISSION_WRITE_PARTIAL_LIST { " + 
			"start_index = " + start_index + ", " + 
			"end_index = " + end_index + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
	 */
	public static class MSG_NAMED_VALUE_FLOAT extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float value; // Floating point value
		private char[] name = new char[10]; // Name of the debug variable
	
		public MSG_NAMED_VALUE_FLOAT () {
			super(MSG_ID_NAMED_VALUE_FLOAT, (short)9, "NAMED_VALUE_FLOAT");
		}

		public MSG_NAMED_VALUE_FLOAT (byte[] bytes) {
			super(bytes, MSG_ID_NAMED_VALUE_FLOAT, (short)9, "NAMED_VALUE_FLOAT");
		}
	
		public MSG_NAMED_VALUE_FLOAT (short sys, short comp, long time_boot_ms  , float value  , char name [] ) {
			super(sys, comp, (short)9, "NAMED_VALUE_FLOAT");
			this.time_boot_ms = time_boot_ms;
			this.value = value;
			this.name = name;
		}

		@Override
		public int getCRCExtra() {
			return 170;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getValue() {
			return value;
		}
		
		public void setValue(float value) {
			this.value = value;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "NAMED_VALUE_FLOAT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			value = buffer.getFloat(); // float
  			for(int c=0; c<10; ++c) {
				name [c] =  (char)buffer.get(); // char[10]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(value); // float
  			for(int c=0; c<10; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_NAMED_VALUE_FLOAT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"value = " + value + ", " + 
			"name = " + name + ",  }";
		}
	}
	/*
	 * Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
	 */
	public static class MSG_NAMED_VALUE_INT extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int value; // Signed integer value
		private char[] name = new char[10]; // Name of the debug variable
	
		public MSG_NAMED_VALUE_INT () {
			super(MSG_ID_NAMED_VALUE_INT, (short)9, "NAMED_VALUE_INT");
		}

		public MSG_NAMED_VALUE_INT (byte[] bytes) {
			super(bytes, MSG_ID_NAMED_VALUE_INT, (short)9, "NAMED_VALUE_INT");
		}
	
		public MSG_NAMED_VALUE_INT (short sys, short comp, long time_boot_ms  , int value  , char name [] ) {
			super(sys, comp, (short)9, "NAMED_VALUE_INT");
			this.time_boot_ms = time_boot_ms;
			this.value = value;
			this.name = name;
		}

		@Override
		public int getCRCExtra() {
			return 44;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getValue() {
			return value;
		}
		
		public void setValue(int value) {
			this.value = value;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "NAMED_VALUE_INT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			value = buffer.getInt(); // int32_t
  			for(int c=0; c<10; ++c) {
				name [c] =  (char)buffer.get(); // char[10]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(value)); // int32_t
  			for(int c=0; c<10; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_NAMED_VALUE_INT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"value = " + value + ", " + 
			"name = " + name + ",  }";
		}
	}
	/*
	 * Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
	 */
	public static class MSG_NAV_CONTROLLER_OUTPUT extends Message {
	
		private float nav_roll; // Current desired roll in degrees
		private float nav_pitch; // Current desired pitch in degrees
		private float alt_error; // Current altitude error in meters
		private float aspd_error; // Current airspeed error in meters/second
		private float xtrack_error; // Current crosstrack error on x-y plane in meters
		private int nav_bearing; // Current desired heading in degrees
		private int target_bearing; // Bearing to current MISSION/target in degrees
		private int wp_dist; // Distance to active MISSION in meters
	
		public MSG_NAV_CONTROLLER_OUTPUT () {
			super(MSG_ID_NAV_CONTROLLER_OUTPUT, (short)26, "NAV_CONTROLLER_OUTPUT");
		}

		public MSG_NAV_CONTROLLER_OUTPUT (byte[] bytes) {
			super(bytes, MSG_ID_NAV_CONTROLLER_OUTPUT, (short)26, "NAV_CONTROLLER_OUTPUT");
		}
	
		public MSG_NAV_CONTROLLER_OUTPUT (short sys, short comp, float nav_roll  , float nav_pitch  , float alt_error  , float aspd_error  , float xtrack_error  , int nav_bearing  , int target_bearing  , int wp_dist  ) {
			super(sys, comp, (short)26, "NAV_CONTROLLER_OUTPUT");
			this.nav_roll = nav_roll;
			this.nav_pitch = nav_pitch;
			this.alt_error = alt_error;
			this.aspd_error = aspd_error;
			this.xtrack_error = xtrack_error;
			this.nav_bearing = nav_bearing;
			this.target_bearing = target_bearing;
			this.wp_dist = wp_dist;
		}

		@Override
		public int getCRCExtra() {
			return 183;
		}
	
		public float getNav_roll() {
			return nav_roll;
		}
		
		public void setNav_roll(float nav_roll) {
			this.nav_roll = nav_roll;
		}
		
		public float getNav_pitch() {
			return nav_pitch;
		}
		
		public void setNav_pitch(float nav_pitch) {
			this.nav_pitch = nav_pitch;
		}
		
		public float getAlt_error() {
			return alt_error;
		}
		
		public void setAlt_error(float alt_error) {
			this.alt_error = alt_error;
		}
		
		public float getAspd_error() {
			return aspd_error;
		}
		
		public void setAspd_error(float aspd_error) {
			this.aspd_error = aspd_error;
		}
		
		public float getXtrack_error() {
			return xtrack_error;
		}
		
		public void setXtrack_error(float xtrack_error) {
			this.xtrack_error = xtrack_error;
		}
		
		public int getNav_bearing() {
			return nav_bearing;
		}
		
		public void setNav_bearing(int nav_bearing) {
			this.nav_bearing = nav_bearing;
		}
		
		public int getTarget_bearing() {
			return target_bearing;
		}
		
		public void setTarget_bearing(int target_bearing) {
			this.target_bearing = target_bearing;
		}
		
		public int getWp_dist() {
			return wp_dist;
		}
		
		public void setWp_dist(int wp_dist) {
			this.wp_dist = wp_dist;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "NAV_CONTROLLER_OUTPUT";
			
			nav_roll = buffer.getFloat(); // float
  			nav_pitch = buffer.getFloat(); // float
  			alt_error = buffer.getFloat(); // float
  			aspd_error = buffer.getFloat(); // float
  			xtrack_error = buffer.getFloat(); // float
  			nav_bearing = buffer.getShort(); // int16_t
  			target_bearing = buffer.getShort(); // int16_t
  			wp_dist = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(nav_roll); // float
  			buffer.putFloat(nav_pitch); // float
  			buffer.putFloat(alt_error); // float
  			buffer.putFloat(aspd_error); // float
  			buffer.putFloat(xtrack_error); // float
  			buffer.putShort((short)(nav_bearing)); // int16_t
  			buffer.putShort((short)(target_bearing)); // int16_t
  			buffer.putShort((short)(wp_dist & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_NAV_CONTROLLER_OUTPUT { " + 
			"nav_roll = " + nav_roll + ", " + 
			"nav_pitch = " + nav_pitch + ", " + 
			"alt_error = " + alt_error + ", " + 
			"aspd_error = " + aspd_error + ", " + 
			"xtrack_error = " + xtrack_error + ", " + 
			"nav_bearing = " + nav_bearing + ", " + 
			"target_bearing = " + target_bearing + ", " + 
			"wp_dist = " + wp_dist + ",  }";
		}
	}
	public static class MSG_ONBOARD_HEALTH extends Message {
	
		private long uptime; // Uptime of system
		private float ram_total; // RAM size in GiB
		private float swap_total; // Swap size in GiB
		private float disk_total; // Disk total in GiB
		private float temp; // Temperature
		private float voltage; // Supply voltage V
		private float network_load_in; // Network load inbound KiB/s
		private float network_load_out; // Network load outbound in KiB/s 
		private int cpu_freq; // CPU frequency
		private int cpu_load; // CPU load in percent
		private int ram_usage; // RAM usage in percent
		private int swap_usage; // Swap usage in percent
		private int disk_health; // Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
		private int disk_usage; // Disk usage in percent
	
		public MSG_ONBOARD_HEALTH () {
			super(MSG_ID_ONBOARD_HEALTH, (short)39, "ONBOARD_HEALTH");
		}

		public MSG_ONBOARD_HEALTH (byte[] bytes) {
			super(bytes, MSG_ID_ONBOARD_HEALTH, (short)39, "ONBOARD_HEALTH");
		}
	
		public MSG_ONBOARD_HEALTH (short sys, short comp, long uptime  , float ram_total  , float swap_total  , float disk_total  , float temp  , float voltage  , float network_load_in  , float network_load_out  , int cpu_freq  , int cpu_load  , int ram_usage  , int swap_usage  , int disk_health  , int disk_usage  ) {
			super(sys, comp, (short)39, "ONBOARD_HEALTH");
			this.uptime = uptime;
			this.ram_total = ram_total;
			this.swap_total = swap_total;
			this.disk_total = disk_total;
			this.temp = temp;
			this.voltage = voltage;
			this.network_load_in = network_load_in;
			this.network_load_out = network_load_out;
			this.cpu_freq = cpu_freq;
			this.cpu_load = cpu_load;
			this.ram_usage = ram_usage;
			this.swap_usage = swap_usage;
			this.disk_health = disk_health;
			this.disk_usage = disk_usage;
		}

		@Override
		public int getCRCExtra() {
			return 19;
		}
	
		public long getUptime() {
			return uptime;
		}
		
		public void setUptime(long uptime) {
			this.uptime = uptime;
		}
		
		public float getRam_total() {
			return ram_total;
		}
		
		public void setRam_total(float ram_total) {
			this.ram_total = ram_total;
		}
		
		public float getSwap_total() {
			return swap_total;
		}
		
		public void setSwap_total(float swap_total) {
			this.swap_total = swap_total;
		}
		
		public float getDisk_total() {
			return disk_total;
		}
		
		public void setDisk_total(float disk_total) {
			this.disk_total = disk_total;
		}
		
		public float getTemp() {
			return temp;
		}
		
		public void setTemp(float temp) {
			this.temp = temp;
		}
		
		public float getVoltage() {
			return voltage;
		}
		
		public void setVoltage(float voltage) {
			this.voltage = voltage;
		}
		
		public float getNetwork_load_in() {
			return network_load_in;
		}
		
		public void setNetwork_load_in(float network_load_in) {
			this.network_load_in = network_load_in;
		}
		
		public float getNetwork_load_out() {
			return network_load_out;
		}
		
		public void setNetwork_load_out(float network_load_out) {
			this.network_load_out = network_load_out;
		}
		
		public int getCpu_freq() {
			return cpu_freq;
		}
		
		public void setCpu_freq(int cpu_freq) {
			this.cpu_freq = cpu_freq;
		}
		
		public int getCpu_load() {
			return cpu_load;
		}
		
		public void setCpu_load(int cpu_load) {
			this.cpu_load = cpu_load;
		}
		
		public int getRam_usage() {
			return ram_usage;
		}
		
		public void setRam_usage(int ram_usage) {
			this.ram_usage = ram_usage;
		}
		
		public int getSwap_usage() {
			return swap_usage;
		}
		
		public void setSwap_usage(int swap_usage) {
			this.swap_usage = swap_usage;
		}
		
		public int getDisk_health() {
			return disk_health;
		}
		
		public void setDisk_health(int disk_health) {
			this.disk_health = disk_health;
		}
		
		public int getDisk_usage() {
			return disk_usage;
		}
		
		public void setDisk_usage(int disk_usage) {
			this.disk_usage = disk_usage;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "ONBOARD_HEALTH";
			
			uptime = buffer.getInt() & 0xffffffff; // uint32_t
  			ram_total = buffer.getFloat(); // float
  			swap_total = buffer.getFloat(); // float
  			disk_total = buffer.getFloat(); // float
  			temp = buffer.getFloat(); // float
  			voltage = buffer.getFloat(); // float
  			network_load_in = buffer.getFloat(); // float
  			network_load_out = buffer.getFloat(); // float
  			cpu_freq = buffer.getShort() & 0xffff; // uint16_t
  			cpu_load = (int)buffer.get() & 0xff; // uint8_t
  			ram_usage = (int)buffer.get() & 0xff; // uint8_t
  			swap_usage = (int)buffer.get() & 0xff; // uint8_t
  			disk_health = (int)buffer.get(); // int8_t
  			disk_usage = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(uptime & 0xffffffff)); // uint32_t
  			buffer.putFloat(ram_total); // float
  			buffer.putFloat(swap_total); // float
  			buffer.putFloat(disk_total); // float
  			buffer.putFloat(temp); // float
  			buffer.putFloat(voltage); // float
  			buffer.putFloat(network_load_in); // float
  			buffer.putFloat(network_load_out); // float
  			buffer.putShort((short)(cpu_freq & 0xffff)); // uint16_t
  			buffer.put((byte)(cpu_load & 0xff)); // uint8_t
  			buffer.put((byte)(ram_usage & 0xff)); // uint8_t
  			buffer.put((byte)(swap_usage & 0xff)); // uint8_t
  			buffer.put((byte)(disk_health)); // int8_t
  			buffer.put((byte)(disk_usage & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_ONBOARD_HEALTH { " + 
			"uptime = " + uptime + ", " + 
			"ram_total = " + ram_total + ", " + 
			"swap_total = " + swap_total + ", " + 
			"disk_total = " + disk_total + ", " + 
			"temp = " + temp + ", " + 
			"voltage = " + voltage + ", " + 
			"network_load_in = " + network_load_in + ", " + 
			"network_load_out = " + network_load_out + ", " + 
			"cpu_freq = " + cpu_freq + ", " + 
			"cpu_load = " + cpu_load + ", " + 
			"ram_usage = " + ram_usage + ", " + 
			"swap_usage = " + swap_usage + ", " + 
			"disk_health = " + disk_health + ", " + 
			"disk_usage = " + disk_usage + ",  }";
		}
	}
	/*
	 * Optical flow from a flow sensor (e.g. optical mouse sensor)
	 */
	public static class MSG_OPTICAL_FLOW extends Message {
	
		private long time_usec; // Timestamp (UNIX)
		private float flow_comp_m_x; // Flow in meters in x-sensor direction, angular-speed compensated
		private float flow_comp_m_y; // Flow in meters in y-sensor direction, angular-speed compensated
		private float ground_distance; // Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
		private int flow_x; // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
		private int flow_y; // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
		private int sensor_id; // Sensor ID
		private int quality; // Optical flow quality / confidence. 0: bad, 255: maximum quality
	
		public MSG_OPTICAL_FLOW () {
			super(MSG_ID_OPTICAL_FLOW, (short)82, "OPTICAL_FLOW");
		}

		public MSG_OPTICAL_FLOW (byte[] bytes) {
			super(bytes, MSG_ID_OPTICAL_FLOW, (short)82, "OPTICAL_FLOW");
		}
	
		public MSG_OPTICAL_FLOW (short sys, short comp, long time_usec  , float flow_comp_m_x  , float flow_comp_m_y  , float ground_distance  , int flow_x  , int flow_y  , int sensor_id  , int quality  ) {
			super(sys, comp, (short)82, "OPTICAL_FLOW");
			this.time_usec = time_usec;
			this.flow_comp_m_x = flow_comp_m_x;
			this.flow_comp_m_y = flow_comp_m_y;
			this.ground_distance = ground_distance;
			this.flow_x = flow_x;
			this.flow_y = flow_y;
			this.sensor_id = sensor_id;
			this.quality = quality;
		}

		@Override
		public int getCRCExtra() {
			return 175;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float getFlow_comp_m_x() {
			return flow_comp_m_x;
		}
		
		public void setFlow_comp_m_x(float flow_comp_m_x) {
			this.flow_comp_m_x = flow_comp_m_x;
		}
		
		public float getFlow_comp_m_y() {
			return flow_comp_m_y;
		}
		
		public void setFlow_comp_m_y(float flow_comp_m_y) {
			this.flow_comp_m_y = flow_comp_m_y;
		}
		
		public float getGround_distance() {
			return ground_distance;
		}
		
		public void setGround_distance(float ground_distance) {
			this.ground_distance = ground_distance;
		}
		
		public int getFlow_x() {
			return flow_x;
		}
		
		public void setFlow_x(int flow_x) {
			this.flow_x = flow_x;
		}
		
		public int getFlow_y() {
			return flow_y;
		}
		
		public void setFlow_y(int flow_y) {
			this.flow_y = flow_y;
		}
		
		public int getSensor_id() {
			return sensor_id;
		}
		
		public void setSensor_id(int sensor_id) {
			this.sensor_id = sensor_id;
		}
		
		public int getQuality() {
			return quality;
		}
		
		public void setQuality(int quality) {
			this.quality = quality;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "OPTICAL_FLOW";
			
			time_usec = buffer.getLong(); // uint64_t
  			flow_comp_m_x = buffer.getFloat(); // float
  			flow_comp_m_y = buffer.getFloat(); // float
  			ground_distance = buffer.getFloat(); // float
  			flow_x = buffer.getShort(); // int16_t
  			flow_y = buffer.getShort(); // int16_t
  			sensor_id = (int)buffer.get() & 0xff; // uint8_t
  			quality = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putFloat(flow_comp_m_x); // float
  			buffer.putFloat(flow_comp_m_y); // float
  			buffer.putFloat(ground_distance); // float
  			buffer.putShort((short)(flow_x)); // int16_t
  			buffer.putShort((short)(flow_y)); // int16_t
  			buffer.put((byte)(sensor_id & 0xff)); // uint8_t
  			buffer.put((byte)(quality & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_OPTICAL_FLOW { " + 
			"time_usec = " + time_usec + ", " + 
			"flow_comp_m_x = " + flow_comp_m_x + ", " + 
			"flow_comp_m_y = " + flow_comp_m_y + ", " + 
			"ground_distance = " + ground_distance + ", " + 
			"flow_x = " + flow_x + ", " + 
			"flow_y = " + flow_y + ", " + 
			"sensor_id = " + sensor_id + ", " + 
			"quality = " + quality + ",  }";
		}
	}
	/*
	 * Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
	 */
	public static class MSG_OPTICAL_FLOW_RAD extends Message {
	
		private long time_usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private long integration_time_us; // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
		private float integrated_x; // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
		private float integrated_y; // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
		private float integrated_xgyro; // RH rotation around X axis (rad)
		private float integrated_ygyro; // RH rotation around Y axis (rad)
		private float integrated_zgyro; // RH rotation around Z axis (rad)
		private long time_delta_distance_us; // Time in microseconds since the distance was sampled.
		private float distance; // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
		private int temperature; // Temperature * 100 in centi-degrees Celsius
		private int sensor_id; // Sensor ID
		private int quality; // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	
		public MSG_OPTICAL_FLOW_RAD () {
			super(MSG_ID_OPTICAL_FLOW_RAD, (short)100, "OPTICAL_FLOW_RAD");
		}

		public MSG_OPTICAL_FLOW_RAD (byte[] bytes) {
			super(bytes, MSG_ID_OPTICAL_FLOW_RAD, (short)100, "OPTICAL_FLOW_RAD");
		}
	
		public MSG_OPTICAL_FLOW_RAD (short sys, short comp, long time_usec  , long integration_time_us  , float integrated_x  , float integrated_y  , float integrated_xgyro  , float integrated_ygyro  , float integrated_zgyro  , long time_delta_distance_us  , float distance  , int temperature  , int sensor_id  , int quality  ) {
			super(sys, comp, (short)100, "OPTICAL_FLOW_RAD");
			this.time_usec = time_usec;
			this.integration_time_us = integration_time_us;
			this.integrated_x = integrated_x;
			this.integrated_y = integrated_y;
			this.integrated_xgyro = integrated_xgyro;
			this.integrated_ygyro = integrated_ygyro;
			this.integrated_zgyro = integrated_zgyro;
			this.time_delta_distance_us = time_delta_distance_us;
			this.distance = distance;
			this.temperature = temperature;
			this.sensor_id = sensor_id;
			this.quality = quality;
		}

		@Override
		public int getCRCExtra() {
			return 138;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public long getIntegration_time_us() {
			return integration_time_us;
		}
		
		public void setIntegration_time_us(long integration_time_us) {
			this.integration_time_us = integration_time_us;
		}
		
		public float getIntegrated_x() {
			return integrated_x;
		}
		
		public void setIntegrated_x(float integrated_x) {
			this.integrated_x = integrated_x;
		}
		
		public float getIntegrated_y() {
			return integrated_y;
		}
		
		public void setIntegrated_y(float integrated_y) {
			this.integrated_y = integrated_y;
		}
		
		public float getIntegrated_xgyro() {
			return integrated_xgyro;
		}
		
		public void setIntegrated_xgyro(float integrated_xgyro) {
			this.integrated_xgyro = integrated_xgyro;
		}
		
		public float getIntegrated_ygyro() {
			return integrated_ygyro;
		}
		
		public void setIntegrated_ygyro(float integrated_ygyro) {
			this.integrated_ygyro = integrated_ygyro;
		}
		
		public float getIntegrated_zgyro() {
			return integrated_zgyro;
		}
		
		public void setIntegrated_zgyro(float integrated_zgyro) {
			this.integrated_zgyro = integrated_zgyro;
		}
		
		public long getTime_delta_distance_us() {
			return time_delta_distance_us;
		}
		
		public void setTime_delta_distance_us(long time_delta_distance_us) {
			this.time_delta_distance_us = time_delta_distance_us;
		}
		
		public float getDistance() {
			return distance;
		}
		
		public void setDistance(float distance) {
			this.distance = distance;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
		public int getSensor_id() {
			return sensor_id;
		}
		
		public void setSensor_id(int sensor_id) {
			this.sensor_id = sensor_id;
		}
		
		public int getQuality() {
			return quality;
		}
		
		public void setQuality(int quality) {
			this.quality = quality;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "OPTICAL_FLOW_RAD";
			
			time_usec = buffer.getLong(); // uint64_t
  			integration_time_us = buffer.getInt() & 0xffffffff; // uint32_t
  			integrated_x = buffer.getFloat(); // float
  			integrated_y = buffer.getFloat(); // float
  			integrated_xgyro = buffer.getFloat(); // float
  			integrated_ygyro = buffer.getFloat(); // float
  			integrated_zgyro = buffer.getFloat(); // float
  			time_delta_distance_us = buffer.getInt() & 0xffffffff; // uint32_t
  			distance = buffer.getFloat(); // float
  			temperature = buffer.getShort(); // int16_t
  			sensor_id = (int)buffer.get() & 0xff; // uint8_t
  			quality = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(integration_time_us & 0xffffffff)); // uint32_t
  			buffer.putFloat(integrated_x); // float
  			buffer.putFloat(integrated_y); // float
  			buffer.putFloat(integrated_xgyro); // float
  			buffer.putFloat(integrated_ygyro); // float
  			buffer.putFloat(integrated_zgyro); // float
  			buffer.putInt((int)(time_delta_distance_us & 0xffffffff)); // uint32_t
  			buffer.putFloat(distance); // float
  			buffer.putShort((short)(temperature)); // int16_t
  			buffer.put((byte)(sensor_id & 0xff)); // uint8_t
  			buffer.put((byte)(quality & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_OPTICAL_FLOW_RAD { " + 
			"time_usec = " + time_usec + ", " + 
			"integration_time_us = " + integration_time_us + ", " + 
			"integrated_x = " + integrated_x + ", " + 
			"integrated_y = " + integrated_y + ", " + 
			"integrated_xgyro = " + integrated_xgyro + ", " + 
			"integrated_ygyro = " + integrated_ygyro + ", " + 
			"integrated_zgyro = " + integrated_zgyro + ", " + 
			"time_delta_distance_us = " + time_delta_distance_us + ", " + 
			"distance = " + distance + ", " + 
			"temperature = " + temperature + ", " + 
			"sensor_id = " + sensor_id + ", " + 
			"quality = " + quality + ",  }";
		}
	}
	/*
	 * Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
	 */
	public static class MSG_PARAM_MAP_RC extends Message {
	
		private float param_value0; // Initial parameter value
		private float scale; // Scale, maps the RC range [-1, 1] to a parameter value
		private float param_value_min; // Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
		private float param_value_max; // Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
		private int param_index; // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
		private int target_system; // System ID
		private int target_component; // Component ID
		private char[] param_id = new char[16]; // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
		private int parameter_rc_channel_index; // Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC.
	
		public MSG_PARAM_MAP_RC () {
			super(MSG_ID_PARAM_MAP_RC, (short)22, "PARAM_MAP_RC");
		}

		public MSG_PARAM_MAP_RC (byte[] bytes) {
			super(bytes, MSG_ID_PARAM_MAP_RC, (short)22, "PARAM_MAP_RC");
		}
	
		public MSG_PARAM_MAP_RC (short sys, short comp, float param_value0  , float scale  , float param_value_min  , float param_value_max  , int param_index  , int target_system  , int target_component  , char param_id [] , int parameter_rc_channel_index  ) {
			super(sys, comp, (short)22, "PARAM_MAP_RC");
			this.param_value0 = param_value0;
			this.scale = scale;
			this.param_value_min = param_value_min;
			this.param_value_max = param_value_max;
			this.param_index = param_index;
			this.target_system = target_system;
			this.target_component = target_component;
			this.param_id = param_id;
			this.parameter_rc_channel_index = parameter_rc_channel_index;
		}

		@Override
		public int getCRCExtra() {
			return 78;
		}
	
		public float getParam_value0() {
			return param_value0;
		}
		
		public void setParam_value0(float param_value0) {
			this.param_value0 = param_value0;
		}
		
		public float getScale() {
			return scale;
		}
		
		public void setScale(float scale) {
			this.scale = scale;
		}
		
		public float getParam_value_min() {
			return param_value_min;
		}
		
		public void setParam_value_min(float param_value_min) {
			this.param_value_min = param_value_min;
		}
		
		public float getParam_value_max() {
			return param_value_max;
		}
		
		public void setParam_value_max(float param_value_max) {
			this.param_value_max = param_value_max;
		}
		
		public int getParam_index() {
			return param_index;
		}
		
		public void setParam_index(int param_index) {
			this.param_index = param_index;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public char[] getParam_id() {
			return param_id;
		}
		
		public void setParam_id(char param_id[]) {
			this.param_id = param_id;
		}
		
		public int getParameter_rc_channel_index() {
			return parameter_rc_channel_index;
		}
		
		public void setParameter_rc_channel_index(int parameter_rc_channel_index) {
			this.parameter_rc_channel_index = parameter_rc_channel_index;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PARAM_MAP_RC";
			
			param_value0 = buffer.getFloat(); // float
  			scale = buffer.getFloat(); // float
  			param_value_min = buffer.getFloat(); // float
  			param_value_max = buffer.getFloat(); // float
  			param_index = buffer.getShort(); // int16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<16; ++c) {
				param_id [c] =  (char)buffer.get(); // char[16]
 			}
			
 			parameter_rc_channel_index = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param_value0); // float
  			buffer.putFloat(scale); // float
  			buffer.putFloat(param_value_min); // float
  			buffer.putFloat(param_value_max); // float
  			buffer.putShort((short)(param_index)); // int16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			for(int c=0; c<16; ++c) {
				buffer.put((byte)(param_id [c]));
 			}
			
 			buffer.put((byte)(parameter_rc_channel_index & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PARAM_MAP_RC { " + 
			"param_value0 = " + param_value0 + ", " + 
			"scale = " + scale + ", " + 
			"param_value_min = " + param_value_min + ", " + 
			"param_value_max = " + param_value_max + ", " + 
			"param_index = " + param_index + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"param_id = " + param_id + ", " + 
			"parameter_rc_channel_index = " + parameter_rc_channel_index + ",  }";
		}
	}
	/*
	 * Request all parameters of this component. After his request, all parameters are emitted.
	 */
	public static class MSG_PARAM_REQUEST_LIST extends Message {
	
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_PARAM_REQUEST_LIST () {
			super(MSG_ID_PARAM_REQUEST_LIST, (short)2, "PARAM_REQUEST_LIST");
		}

		public MSG_PARAM_REQUEST_LIST (byte[] bytes) {
			super(bytes, MSG_ID_PARAM_REQUEST_LIST, (short)2, "PARAM_REQUEST_LIST");
		}
	
		public MSG_PARAM_REQUEST_LIST (short sys, short comp, int target_system  , int target_component  ) {
			super(sys, comp, (short)2, "PARAM_REQUEST_LIST");
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 159;
		}
	
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PARAM_REQUEST_LIST";
			
			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PARAM_REQUEST_LIST { " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
	 */
	public static class MSG_PARAM_REQUEST_READ extends Message {
	
		private int param_index; // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
		private int target_system; // System ID
		private int target_component; // Component ID
		private char[] param_id = new char[16]; // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	
		public MSG_PARAM_REQUEST_READ () {
			super(MSG_ID_PARAM_REQUEST_READ, (short)5, "PARAM_REQUEST_READ");
		}

		public MSG_PARAM_REQUEST_READ (byte[] bytes) {
			super(bytes, MSG_ID_PARAM_REQUEST_READ, (short)5, "PARAM_REQUEST_READ");
		}
	
		public MSG_PARAM_REQUEST_READ (short sys, short comp, int param_index  , int target_system  , int target_component  , char param_id [] ) {
			super(sys, comp, (short)5, "PARAM_REQUEST_READ");
			this.param_index = param_index;
			this.target_system = target_system;
			this.target_component = target_component;
			this.param_id = param_id;
		}

		@Override
		public int getCRCExtra() {
			return 214;
		}
	
		public int getParam_index() {
			return param_index;
		}
		
		public void setParam_index(int param_index) {
			this.param_index = param_index;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public char[] getParam_id() {
			return param_id;
		}
		
		public void setParam_id(char param_id[]) {
			this.param_id = param_id;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PARAM_REQUEST_READ";
			
			param_index = buffer.getShort(); // int16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<16; ++c) {
				param_id [c] =  (char)buffer.get(); // char[16]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(param_index)); // int16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			for(int c=0; c<16; ++c) {
				buffer.put((byte)(param_id [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PARAM_REQUEST_READ { " + 
			"param_index = " + param_index + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"param_id = " + param_id + ",  }";
		}
	}
	/*
	 * Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
	 */
	public static class MSG_PARAM_SET extends Message {
	
		private float param_value; // Onboard parameter value
		private int target_system; // System ID
		private int target_component; // Component ID
		private char[] param_id = new char[16]; // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
		private int param_type; // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
	
		public MSG_PARAM_SET () {
			super(MSG_ID_PARAM_SET, (short)8, "PARAM_SET");
		}

		public MSG_PARAM_SET (byte[] bytes) {
			super(bytes, MSG_ID_PARAM_SET, (short)8, "PARAM_SET");
		}
	
		public MSG_PARAM_SET (short sys, short comp, float param_value  , int target_system  , int target_component  , char param_id [] , int param_type  ) {
			super(sys, comp, (short)8, "PARAM_SET");
			this.param_value = param_value;
			this.target_system = target_system;
			this.target_component = target_component;
			this.param_id = param_id;
			this.param_type = param_type;
		}

		@Override
		public int getCRCExtra() {
			return 168;
		}
	
		public float getParam_value() {
			return param_value;
		}
		
		public void setParam_value(float param_value) {
			this.param_value = param_value;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public char[] getParam_id() {
			return param_id;
		}
		
		public void setParam_id(char param_id[]) {
			this.param_id = param_id;
		}
		
		public int getParam_type() {
			return param_type;
		}
		
		public void setParam_type(int param_type) {
			this.param_type = param_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PARAM_SET";
			
			param_value = buffer.getFloat(); // float
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<16; ++c) {
				param_id [c] =  (char)buffer.get(); // char[16]
 			}
			
 			param_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param_value); // float
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			for(int c=0; c<16; ++c) {
				buffer.put((byte)(param_id [c]));
 			}
			
 			buffer.put((byte)(param_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PARAM_SET { " + 
			"param_value = " + param_value + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"param_id = " + param_id + ", " + 
			"param_type = " + param_type + ",  }";
		}
	}
	/*
	 * Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
	 */
	public static class MSG_PARAM_VALUE extends Message {
	
		private float param_value; // Onboard parameter value
		private int param_count; // Total number of onboard parameters
		private int param_index; // Index of this onboard parameter
		private char[] param_id = new char[16]; // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
		private int param_type; // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
	
		public MSG_PARAM_VALUE () {
			super(MSG_ID_PARAM_VALUE, (short)10, "PARAM_VALUE");
		}

		public MSG_PARAM_VALUE (byte[] bytes) {
			super(bytes, MSG_ID_PARAM_VALUE, (short)10, "PARAM_VALUE");
		}
	
		public MSG_PARAM_VALUE (short sys, short comp, float param_value  , int param_count  , int param_index  , char param_id [] , int param_type  ) {
			super(sys, comp, (short)10, "PARAM_VALUE");
			this.param_value = param_value;
			this.param_count = param_count;
			this.param_index = param_index;
			this.param_id = param_id;
			this.param_type = param_type;
		}

		@Override
		public int getCRCExtra() {
			return 220;
		}
	
		public float getParam_value() {
			return param_value;
		}
		
		public void setParam_value(float param_value) {
			this.param_value = param_value;
		}
		
		public int getParam_count() {
			return param_count;
		}
		
		public void setParam_count(int param_count) {
			this.param_count = param_count;
		}
		
		public int getParam_index() {
			return param_index;
		}
		
		public void setParam_index(int param_index) {
			this.param_index = param_index;
		}
		
		public char[] getParam_id() {
			return param_id;
		}
		
		public void setParam_id(char param_id[]) {
			this.param_id = param_id;
		}
		
		public int getParam_type() {
			return param_type;
		}
		
		public void setParam_type(int param_type) {
			this.param_type = param_type;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PARAM_VALUE";
			
			param_value = buffer.getFloat(); // float
  			param_count = buffer.getShort() & 0xffff; // uint16_t
  			param_index = buffer.getShort() & 0xffff; // uint16_t
  			for(int c=0; c<16; ++c) {
				param_id [c] =  (char)buffer.get(); // char[16]
 			}
			
 			param_type = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(param_value); // float
  			buffer.putShort((short)(param_count & 0xffff)); // uint16_t
  			buffer.putShort((short)(param_index & 0xffff)); // uint16_t
  			for(int c=0; c<16; ++c) {
				buffer.put((byte)(param_id [c]));
 			}
			
 			buffer.put((byte)(param_type & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PARAM_VALUE { " + 
			"param_value = " + param_value + ", " + 
			"param_count = " + param_count + ", " + 
			"param_index = " + param_index + ", " + 
			"param_id = " + param_id + ", " + 
			"param_type = " + param_type + ",  }";
		}
	}
	public static class MSG_PATTERN_DETECTED extends Message {
	
		private float confidence; // Confidence of detection
		private int type; // 0: Pattern, 1: Letter
		private char[] file = new char[100]; // Pattern file name
		private int detected; // Accepted as true detection, 0 no, 1 yes
	
		public MSG_PATTERN_DETECTED () {
			super(MSG_ID_PATTERN_DETECTED, (short)7, "PATTERN_DETECTED");
		}

		public MSG_PATTERN_DETECTED (byte[] bytes) {
			super(bytes, MSG_ID_PATTERN_DETECTED, (short)7, "PATTERN_DETECTED");
		}
	
		public MSG_PATTERN_DETECTED (short sys, short comp, float confidence  , int type  , char file [] , int detected  ) {
			super(sys, comp, (short)7, "PATTERN_DETECTED");
			this.confidence = confidence;
			this.type = type;
			this.file = file;
			this.detected = detected;
		}

		@Override
		public int getCRCExtra() {
			return 90;
		}
	
		public float getConfidence() {
			return confidence;
		}
		
		public void setConfidence(float confidence) {
			this.confidence = confidence;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public char[] getFile() {
			return file;
		}
		
		public void setFile(char file[]) {
			this.file = file;
		}
		
		public int getDetected() {
			return detected;
		}
		
		public void setDetected(int detected) {
			this.detected = detected;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PATTERN_DETECTED";
			
			confidence = buffer.getFloat(); // float
  			type = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<100; ++c) {
				file [c] =  (char)buffer.get(); // char[100]
 			}
			
 			detected = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(confidence); // float
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			for(int c=0; c<100; ++c) {
				buffer.put((byte)(file [c]));
 			}
			
 			buffer.put((byte)(detected & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PATTERN_DETECTED { " + 
			"confidence = " + confidence + ", " + 
			"type = " + type + ", " + 
			"file = " + file + ", " + 
			"detected = " + detected + ",  }";
		}
	}
	/*
	 * A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
	 */
	public static class MSG_PING extends Message {
	
		private long time_usec; // Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
		private long seq; // PING sequence
		private int target_system; // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
		private int target_component; // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
	
		public MSG_PING () {
			super(MSG_ID_PING, (short)70, "PING");
		}

		public MSG_PING (byte[] bytes) {
			super(bytes, MSG_ID_PING, (short)70, "PING");
		}
	
		public MSG_PING (short sys, short comp, long time_usec  , long seq  , int target_system  , int target_component  ) {
			super(sys, comp, (short)70, "PING");
			this.time_usec = time_usec;
			this.seq = seq;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 237;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public long getSeq() {
			return seq;
		}
		
		public void setSeq(long seq) {
			this.seq = seq;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "PING";
			
			time_usec = buffer.getLong(); // uint64_t
  			seq = buffer.getInt() & 0xffffffff; // uint32_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putInt((int)(seq & 0xffffffff)); // uint32_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_PING { " + 
			"time_usec = " + time_usec + ", " + 
			"seq = " + seq + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Notifies the operator about a point of interest (POI). This can be anything detected by the
                system. This generic message is intented to help interfacing to generic visualizations and to display
                the POI on a map.
            
	 */
	public static class MSG_POINT_OF_INTEREST extends Message {
	
		private float x; // X Position
		private float y; // Y Position
		private float z; // Z Position
		private int timeout; // 0: no timeout, >1: timeout in seconds
		private int type; // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
		private int color; // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
		private int coordinate_system; // 0: global, 1:local
		private char[] name = new char[26]; // POI name
	
		public MSG_POINT_OF_INTEREST () {
			super(MSG_ID_POINT_OF_INTEREST, (short)18, "POINT_OF_INTEREST");
		}

		public MSG_POINT_OF_INTEREST (byte[] bytes) {
			super(bytes, MSG_ID_POINT_OF_INTEREST, (short)18, "POINT_OF_INTEREST");
		}
	
		public MSG_POINT_OF_INTEREST (short sys, short comp, float x  , float y  , float z  , int timeout  , int type  , int color  , int coordinate_system  , char name [] ) {
			super(sys, comp, (short)18, "POINT_OF_INTEREST");
			this.x = x;
			this.y = y;
			this.z = z;
			this.timeout = timeout;
			this.type = type;
			this.color = color;
			this.coordinate_system = coordinate_system;
			this.name = name;
		}

		@Override
		public int getCRCExtra() {
			return 95;
		}
	
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public int getTimeout() {
			return timeout;
		}
		
		public void setTimeout(int timeout) {
			this.timeout = timeout;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getColor() {
			return color;
		}
		
		public void setColor(int color) {
			this.color = color;
		}
		
		public int getCoordinate_system() {
			return coordinate_system;
		}
		
		public void setCoordinate_system(int coordinate_system) {
			this.coordinate_system = coordinate_system;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POINT_OF_INTEREST";
			
			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			timeout = buffer.getShort() & 0xffff; // uint16_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			color = (int)buffer.get() & 0xff; // uint8_t
  			coordinate_system = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<26; ++c) {
				name [c] =  (char)buffer.get(); // char[26]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putShort((short)(timeout & 0xffff)); // uint16_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(color & 0xff)); // uint8_t
  			buffer.put((byte)(coordinate_system & 0xff)); // uint8_t
  			for(int c=0; c<26; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POINT_OF_INTEREST { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"timeout = " + timeout + ", " + 
			"type = " + type + ", " + 
			"color = " + color + ", " + 
			"coordinate_system = " + coordinate_system + ", " + 
			"name = " + name + ",  }";
		}
	}
	/*
	 * Notifies the operator about the connection of two point of interests (POI). This can be anything detected by the
                system. This generic message is intented to help interfacing to generic visualizations and to display
                the POI on a map.
            
	 */
	public static class MSG_POINT_OF_INTEREST_CONNECTION extends Message {
	
		private float xp1; // X1 Position
		private float yp1; // Y1 Position
		private float zp1; // Z1 Position
		private float xp2; // X2 Position
		private float yp2; // Y2 Position
		private float zp2; // Z2 Position
		private int timeout; // 0: no timeout, >1: timeout in seconds
		private int type; // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
		private int color; // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
		private int coordinate_system; // 0: global, 1:local
		private char[] name = new char[26]; // POI connection name
	
		public MSG_POINT_OF_INTEREST_CONNECTION () {
			super(MSG_ID_POINT_OF_INTEREST_CONNECTION, (short)30, "POINT_OF_INTEREST_CONNECTION");
		}

		public MSG_POINT_OF_INTEREST_CONNECTION (byte[] bytes) {
			super(bytes, MSG_ID_POINT_OF_INTEREST_CONNECTION, (short)30, "POINT_OF_INTEREST_CONNECTION");
		}
	
		public MSG_POINT_OF_INTEREST_CONNECTION (short sys, short comp, float xp1  , float yp1  , float zp1  , float xp2  , float yp2  , float zp2  , int timeout  , int type  , int color  , int coordinate_system  , char name [] ) {
			super(sys, comp, (short)30, "POINT_OF_INTEREST_CONNECTION");
			this.xp1 = xp1;
			this.yp1 = yp1;
			this.zp1 = zp1;
			this.xp2 = xp2;
			this.yp2 = yp2;
			this.zp2 = zp2;
			this.timeout = timeout;
			this.type = type;
			this.color = color;
			this.coordinate_system = coordinate_system;
			this.name = name;
		}

		@Override
		public int getCRCExtra() {
			return 36;
		}
	
		public float getXp1() {
			return xp1;
		}
		
		public void setXp1(float xp1) {
			this.xp1 = xp1;
		}
		
		public float getYp1() {
			return yp1;
		}
		
		public void setYp1(float yp1) {
			this.yp1 = yp1;
		}
		
		public float getZp1() {
			return zp1;
		}
		
		public void setZp1(float zp1) {
			this.zp1 = zp1;
		}
		
		public float getXp2() {
			return xp2;
		}
		
		public void setXp2(float xp2) {
			this.xp2 = xp2;
		}
		
		public float getYp2() {
			return yp2;
		}
		
		public void setYp2(float yp2) {
			this.yp2 = yp2;
		}
		
		public float getZp2() {
			return zp2;
		}
		
		public void setZp2(float zp2) {
			this.zp2 = zp2;
		}
		
		public int getTimeout() {
			return timeout;
		}
		
		public void setTimeout(int timeout) {
			this.timeout = timeout;
		}
		
		public int getType() {
			return type;
		}
		
		public void setType(int type) {
			this.type = type;
		}
		
		public int getColor() {
			return color;
		}
		
		public void setColor(int color) {
			this.color = color;
		}
		
		public int getCoordinate_system() {
			return coordinate_system;
		}
		
		public void setCoordinate_system(int coordinate_system) {
			this.coordinate_system = coordinate_system;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POINT_OF_INTEREST_CONNECTION";
			
			xp1 = buffer.getFloat(); // float
  			yp1 = buffer.getFloat(); // float
  			zp1 = buffer.getFloat(); // float
  			xp2 = buffer.getFloat(); // float
  			yp2 = buffer.getFloat(); // float
  			zp2 = buffer.getFloat(); // float
  			timeout = buffer.getShort() & 0xffff; // uint16_t
  			type = (int)buffer.get() & 0xff; // uint8_t
  			color = (int)buffer.get() & 0xff; // uint8_t
  			coordinate_system = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<26; ++c) {
				name [c] =  (char)buffer.get(); // char[26]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(xp1); // float
  			buffer.putFloat(yp1); // float
  			buffer.putFloat(zp1); // float
  			buffer.putFloat(xp2); // float
  			buffer.putFloat(yp2); // float
  			buffer.putFloat(zp2); // float
  			buffer.putShort((short)(timeout & 0xffff)); // uint16_t
  			buffer.put((byte)(type & 0xff)); // uint8_t
  			buffer.put((byte)(color & 0xff)); // uint8_t
  			buffer.put((byte)(coordinate_system & 0xff)); // uint8_t
  			for(int c=0; c<26; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POINT_OF_INTEREST_CONNECTION { " + 
			"xp1 = " + xp1 + ", " + 
			"yp1 = " + yp1 + ", " + 
			"zp1 = " + zp1 + ", " + 
			"xp2 = " + xp2 + ", " + 
			"yp2 = " + yp2 + ", " + 
			"zp2 = " + zp2 + ", " + 
			"timeout = " + timeout + ", " + 
			"type = " + type + ", " + 
			"color = " + color + ", " + 
			"coordinate_system = " + coordinate_system + ", " + 
			"name = " + name + ",  }";
		}
	}
	public static class MSG_POSITION_CONTROL_SETPOINT extends Message {
	
		private float x; // x position
		private float y; // y position
		private float z; // z position
		private float yaw; // yaw orientation in radians, 0 = NORTH
		private int id; // ID of waypoint, 0 for plain position
	
		public MSG_POSITION_CONTROL_SETPOINT () {
			super(MSG_ID_POSITION_CONTROL_SETPOINT, (short)18, "POSITION_CONTROL_SETPOINT");
		}

		public MSG_POSITION_CONTROL_SETPOINT (byte[] bytes) {
			super(bytes, MSG_ID_POSITION_CONTROL_SETPOINT, (short)18, "POSITION_CONTROL_SETPOINT");
		}
	
		public MSG_POSITION_CONTROL_SETPOINT (short sys, short comp, float x  , float y  , float z  , float yaw  , int id  ) {
			super(sys, comp, (short)18, "POSITION_CONTROL_SETPOINT");
			this.x = x;
			this.y = y;
			this.z = z;
			this.yaw = yaw;
			this.id = id;
		}

		@Override
		public int getCRCExtra() {
			return 28;
		}
	
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public int getId() {
			return id;
		}
		
		public void setId(int id) {
			this.id = id;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POSITION_CONTROL_SETPOINT";
			
			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			id = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(yaw); // float
  			buffer.putShort((short)(id & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POSITION_CONTROL_SETPOINT { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"yaw = " + yaw + ", " + 
			"id = " + id + ",  }";
		}
	}
	/*
	 * Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
	 */
	public static class MSG_POSITION_TARGET_GLOBAL_INT extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
		private int lat_int; // X Position in WGS84 frame in 1e7 * meters
		private int lon_int; // Y Position in WGS84 frame in 1e7 * meters
		private float alt; // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
		private float vx; // X velocity in NED frame in meter / s
		private float vy; // Y velocity in NED frame in meter / s
		private float vz; // Z velocity in NED frame in meter / s
		private float afx; // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afy; // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afz; // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float yaw; // yaw setpoint in rad
		private float yaw_rate; // yaw rate setpoint in rad/s
		private int type_mask; // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
		private int coordinate_frame; // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
	
		public MSG_POSITION_TARGET_GLOBAL_INT () {
			super(MSG_ID_POSITION_TARGET_GLOBAL_INT, (short)51, "POSITION_TARGET_GLOBAL_INT");
		}

		public MSG_POSITION_TARGET_GLOBAL_INT (byte[] bytes) {
			super(bytes, MSG_ID_POSITION_TARGET_GLOBAL_INT, (short)51, "POSITION_TARGET_GLOBAL_INT");
		}
	
		public MSG_POSITION_TARGET_GLOBAL_INT (short sys, short comp, long time_boot_ms  , int lat_int  , int lon_int  , float alt  , float vx  , float vy  , float vz  , float afx  , float afy  , float afz  , float yaw  , float yaw_rate  , int type_mask  , int coordinate_frame  ) {
			super(sys, comp, (short)51, "POSITION_TARGET_GLOBAL_INT");
			this.time_boot_ms = time_boot_ms;
			this.lat_int = lat_int;
			this.lon_int = lon_int;
			this.alt = alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.afx = afx;
			this.afy = afy;
			this.afz = afz;
			this.yaw = yaw;
			this.yaw_rate = yaw_rate;
			this.type_mask = type_mask;
			this.coordinate_frame = coordinate_frame;
		}

		@Override
		public int getCRCExtra() {
			return 150;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getLat_int() {
			return lat_int;
		}
		
		public void setLat_int(int lat_int) {
			this.lat_int = lat_int;
		}
		
		public int getLon_int() {
			return lon_int;
		}
		
		public void setLon_int(int lon_int) {
			this.lon_int = lon_int;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float getAfx() {
			return afx;
		}
		
		public void setAfx(float afx) {
			this.afx = afx;
		}
		
		public float getAfy() {
			return afy;
		}
		
		public void setAfy(float afy) {
			this.afy = afy;
		}
		
		public float getAfz() {
			return afz;
		}
		
		public void setAfz(float afz) {
			this.afz = afz;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getYaw_rate() {
			return yaw_rate;
		}
		
		public void setYaw_rate(float yaw_rate) {
			this.yaw_rate = yaw_rate;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
		public int getCoordinate_frame() {
			return coordinate_frame;
		}
		
		public void setCoordinate_frame(int coordinate_frame) {
			this.coordinate_frame = coordinate_frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POSITION_TARGET_GLOBAL_INT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			lat_int = buffer.getInt(); // int32_t
  			lon_int = buffer.getInt(); // int32_t
  			alt = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			afx = buffer.getFloat(); // float
  			afy = buffer.getFloat(); // float
  			afz = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			yaw_rate = buffer.getFloat(); // float
  			type_mask = buffer.getShort() & 0xffff; // uint16_t
  			coordinate_frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(lat_int)); // int32_t
  			buffer.putInt((int)(lon_int)); // int32_t
  			buffer.putFloat(alt); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			buffer.putFloat(afx); // float
  			buffer.putFloat(afy); // float
  			buffer.putFloat(afz); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(yaw_rate); // float
  			buffer.putShort((short)(type_mask & 0xffff)); // uint16_t
  			buffer.put((byte)(coordinate_frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POSITION_TARGET_GLOBAL_INT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"lat_int = " + lat_int + ", " + 
			"lon_int = " + lon_int + ", " + 
			"alt = " + alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"afx = " + afx + ", " + 
			"afy = " + afy + ", " + 
			"afz = " + afz + ", " + 
			"yaw = " + yaw + ", " + 
			"yaw_rate = " + yaw_rate + ", " + 
			"type_mask = " + type_mask + ", " + 
			"coordinate_frame = " + coordinate_frame + ",  }";
		}
	}
	/*
	 * Set vehicle position, velocity and acceleration setpoint in local frame.
	 */
	public static class MSG_POSITION_TARGET_LOCAL_NED extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot
		private float x; // X Position in NED frame in meters
		private float y; // Y Position in NED frame in meters
		private float z; // Z Position in NED frame in meters (note, altitude is negative in NED)
		private float vx; // X velocity in NED frame in meter / s
		private float vy; // Y velocity in NED frame in meter / s
		private float vz; // Z velocity in NED frame in meter / s
		private float afx; // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afy; // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afz; // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float yaw; // yaw setpoint in rad
		private float yaw_rate; // yaw rate setpoint in rad/s
		private int type_mask; // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
		private int coordinate_frame; // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
	
		public MSG_POSITION_TARGET_LOCAL_NED () {
			super(MSG_ID_POSITION_TARGET_LOCAL_NED, (short)51, "POSITION_TARGET_LOCAL_NED");
		}

		public MSG_POSITION_TARGET_LOCAL_NED (byte[] bytes) {
			super(bytes, MSG_ID_POSITION_TARGET_LOCAL_NED, (short)51, "POSITION_TARGET_LOCAL_NED");
		}
	
		public MSG_POSITION_TARGET_LOCAL_NED (short sys, short comp, long time_boot_ms  , float x  , float y  , float z  , float vx  , float vy  , float vz  , float afx  , float afy  , float afz  , float yaw  , float yaw_rate  , int type_mask  , int coordinate_frame  ) {
			super(sys, comp, (short)51, "POSITION_TARGET_LOCAL_NED");
			this.time_boot_ms = time_boot_ms;
			this.x = x;
			this.y = y;
			this.z = z;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.afx = afx;
			this.afy = afy;
			this.afz = afz;
			this.yaw = yaw;
			this.yaw_rate = yaw_rate;
			this.type_mask = type_mask;
			this.coordinate_frame = coordinate_frame;
		}

		@Override
		public int getCRCExtra() {
			return 140;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float getAfx() {
			return afx;
		}
		
		public void setAfx(float afx) {
			this.afx = afx;
		}
		
		public float getAfy() {
			return afy;
		}
		
		public void setAfy(float afy) {
			this.afy = afy;
		}
		
		public float getAfz() {
			return afz;
		}
		
		public void setAfz(float afz) {
			this.afz = afz;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getYaw_rate() {
			return yaw_rate;
		}
		
		public void setYaw_rate(float yaw_rate) {
			this.yaw_rate = yaw_rate;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
		public int getCoordinate_frame() {
			return coordinate_frame;
		}
		
		public void setCoordinate_frame(int coordinate_frame) {
			this.coordinate_frame = coordinate_frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POSITION_TARGET_LOCAL_NED";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			afx = buffer.getFloat(); // float
  			afy = buffer.getFloat(); // float
  			afz = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			yaw_rate = buffer.getFloat(); // float
  			type_mask = buffer.getShort() & 0xffff; // uint16_t
  			coordinate_frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			buffer.putFloat(afx); // float
  			buffer.putFloat(afy); // float
  			buffer.putFloat(afz); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(yaw_rate); // float
  			buffer.putShort((short)(type_mask & 0xffff)); // uint16_t
  			buffer.put((byte)(coordinate_frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POSITION_TARGET_LOCAL_NED { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"afx = " + afx + ", " + 
			"afy = " + afy + ", " + 
			"afz = " + afz + ", " + 
			"yaw = " + yaw + ", " + 
			"yaw_rate = " + yaw_rate + ", " + 
			"type_mask = " + type_mask + ", " + 
			"coordinate_frame = " + coordinate_frame + ",  }";
		}
	}
	/*
	 * Power supply status
	 */
	public static class MSG_POWER_STATUS extends Message {
	
		private int Vcc; // 5V rail voltage in millivolts
		private int Vservo; // servo rail voltage in millivolts
		private int flags; // power supply status flags (see MAV_POWER_STATUS enum)
	
		public MSG_POWER_STATUS () {
			super(MSG_ID_POWER_STATUS, (short)6, "POWER_STATUS");
		}

		public MSG_POWER_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_POWER_STATUS, (short)6, "POWER_STATUS");
		}
	
		public MSG_POWER_STATUS (short sys, short comp, int Vcc  , int Vservo  , int flags  ) {
			super(sys, comp, (short)6, "POWER_STATUS");
			this.Vcc = Vcc;
			this.Vservo = Vservo;
			this.flags = flags;
		}

		@Override
		public int getCRCExtra() {
			return 203;
		}
	
		public int getVcc() {
			return Vcc;
		}
		
		public void setVcc(int Vcc) {
			this.Vcc = Vcc;
		}
		
		public int getVservo() {
			return Vservo;
		}
		
		public void setVservo(int Vservo) {
			this.Vservo = Vservo;
		}
		
		public int getFlags() {
			return flags;
		}
		
		public void setFlags(int flags) {
			this.flags = flags;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "POWER_STATUS";
			
			Vcc = buffer.getShort() & 0xffff; // uint16_t
  			Vservo = buffer.getShort() & 0xffff; // uint16_t
  			flags = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(Vcc & 0xffff)); // uint16_t
  			buffer.putShort((short)(Vservo & 0xffff)); // uint16_t
  			buffer.putShort((short)(flags & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_POWER_STATUS { " + 
			"Vcc = " + Vcc + ", " + 
			"Vservo = " + Vservo + ", " + 
			"flags = " + flags + ",  }";
		}
	}
	/*
	 * Status generated by radio and injected into MAVLink stream.
	 */
	public static class MSG_RADIO_STATUS extends Message {
	
		private int rxerrors; // Receive errors
		private int fixed; // Count of error corrected packets
		private int rssi; // Local signal strength
		private int remrssi; // Remote signal strength
		private int txbuf; // Remaining free buffer space in percent.
		private int noise; // Background noise level
		private int remnoise; // Remote background noise level
	
		public MSG_RADIO_STATUS () {
			super(MSG_ID_RADIO_STATUS, (short)9, "RADIO_STATUS");
		}

		public MSG_RADIO_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_RADIO_STATUS, (short)9, "RADIO_STATUS");
		}
	
		public MSG_RADIO_STATUS (short sys, short comp, int rxerrors  , int fixed  , int rssi  , int remrssi  , int txbuf  , int noise  , int remnoise  ) {
			super(sys, comp, (short)9, "RADIO_STATUS");
			this.rxerrors = rxerrors;
			this.fixed = fixed;
			this.rssi = rssi;
			this.remrssi = remrssi;
			this.txbuf = txbuf;
			this.noise = noise;
			this.remnoise = remnoise;
		}

		@Override
		public int getCRCExtra() {
			return 185;
		}
	
		public int getRxerrors() {
			return rxerrors;
		}
		
		public void setRxerrors(int rxerrors) {
			this.rxerrors = rxerrors;
		}
		
		public int getFixed() {
			return fixed;
		}
		
		public void setFixed(int fixed) {
			this.fixed = fixed;
		}
		
		public int getRssi() {
			return rssi;
		}
		
		public void setRssi(int rssi) {
			this.rssi = rssi;
		}
		
		public int getRemrssi() {
			return remrssi;
		}
		
		public void setRemrssi(int remrssi) {
			this.remrssi = remrssi;
		}
		
		public int getTxbuf() {
			return txbuf;
		}
		
		public void setTxbuf(int txbuf) {
			this.txbuf = txbuf;
		}
		
		public int getNoise() {
			return noise;
		}
		
		public void setNoise(int noise) {
			this.noise = noise;
		}
		
		public int getRemnoise() {
			return remnoise;
		}
		
		public void setRemnoise(int remnoise) {
			this.remnoise = remnoise;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RADIO_STATUS";
			
			rxerrors = buffer.getShort() & 0xffff; // uint16_t
  			fixed = buffer.getShort() & 0xffff; // uint16_t
  			rssi = (int)buffer.get() & 0xff; // uint8_t
  			remrssi = (int)buffer.get() & 0xff; // uint8_t
  			txbuf = (int)buffer.get() & 0xff; // uint8_t
  			noise = (int)buffer.get() & 0xff; // uint8_t
  			remnoise = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(rxerrors & 0xffff)); // uint16_t
  			buffer.putShort((short)(fixed & 0xffff)); // uint16_t
  			buffer.put((byte)(rssi & 0xff)); // uint8_t
  			buffer.put((byte)(remrssi & 0xff)); // uint8_t
  			buffer.put((byte)(txbuf & 0xff)); // uint8_t
  			buffer.put((byte)(noise & 0xff)); // uint8_t
  			buffer.put((byte)(remnoise & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RADIO_STATUS { " + 
			"rxerrors = " + rxerrors + ", " + 
			"fixed = " + fixed + ", " + 
			"rssi = " + rssi + ", " + 
			"remrssi = " + remrssi + ", " + 
			"txbuf = " + txbuf + ", " + 
			"noise = " + noise + ", " + 
			"remnoise = " + remnoise + ",  }";
		}
	}
	public static class MSG_RAW_AUX extends Message {
	
		private int baro; // Barometric pressure (hecto Pascal)
		private int adc1; // ADC1 (J405 ADC3, LPC2148 AD0.6)
		private int adc2; // ADC2 (J405 ADC5, LPC2148 AD0.2)
		private int adc3; // ADC3 (J405 ADC6, LPC2148 AD0.1)
		private int adc4; // ADC4 (J405 ADC7, LPC2148 AD1.3)
		private int vbat; // Battery voltage
		private int temp; // Temperature (degrees celcius)
	
		public MSG_RAW_AUX () {
			super(MSG_ID_RAW_AUX, (short)16, "RAW_AUX");
		}

		public MSG_RAW_AUX (byte[] bytes) {
			super(bytes, MSG_ID_RAW_AUX, (short)16, "RAW_AUX");
		}
	
		public MSG_RAW_AUX (short sys, short comp, int baro  , int adc1  , int adc2  , int adc3  , int adc4  , int vbat  , int temp  ) {
			super(sys, comp, (short)16, "RAW_AUX");
			this.baro = baro;
			this.adc1 = adc1;
			this.adc2 = adc2;
			this.adc3 = adc3;
			this.adc4 = adc4;
			this.vbat = vbat;
			this.temp = temp;
		}

		@Override
		public int getCRCExtra() {
			return 182;
		}
	
		public int getBaro() {
			return baro;
		}
		
		public void setBaro(int baro) {
			this.baro = baro;
		}
		
		public int getAdc1() {
			return adc1;
		}
		
		public void setAdc1(int adc1) {
			this.adc1 = adc1;
		}
		
		public int getAdc2() {
			return adc2;
		}
		
		public void setAdc2(int adc2) {
			this.adc2 = adc2;
		}
		
		public int getAdc3() {
			return adc3;
		}
		
		public void setAdc3(int adc3) {
			this.adc3 = adc3;
		}
		
		public int getAdc4() {
			return adc4;
		}
		
		public void setAdc4(int adc4) {
			this.adc4 = adc4;
		}
		
		public int getVbat() {
			return vbat;
		}
		
		public void setVbat(int vbat) {
			this.vbat = vbat;
		}
		
		public int getTemp() {
			return temp;
		}
		
		public void setTemp(int temp) {
			this.temp = temp;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RAW_AUX";
			
			baro = buffer.getInt(); // int32_t
  			adc1 = buffer.getShort() & 0xffff; // uint16_t
  			adc2 = buffer.getShort() & 0xffff; // uint16_t
  			adc3 = buffer.getShort() & 0xffff; // uint16_t
  			adc4 = buffer.getShort() & 0xffff; // uint16_t
  			vbat = buffer.getShort() & 0xffff; // uint16_t
  			temp = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(baro)); // int32_t
  			buffer.putShort((short)(adc1 & 0xffff)); // uint16_t
  			buffer.putShort((short)(adc2 & 0xffff)); // uint16_t
  			buffer.putShort((short)(adc3 & 0xffff)); // uint16_t
  			buffer.putShort((short)(adc4 & 0xffff)); // uint16_t
  			buffer.putShort((short)(vbat & 0xffff)); // uint16_t
  			buffer.putShort((short)(temp)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RAW_AUX { " + 
			"baro = " + baro + ", " + 
			"adc1 = " + adc1 + ", " + 
			"adc2 = " + adc2 + ", " + 
			"adc3 = " + adc3 + ", " + 
			"adc4 = " + adc4 + ", " + 
			"vbat = " + vbat + ", " + 
			"temp = " + temp + ",  }";
		}
	}
	/*
	 * The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
	 */
	public static class MSG_RAW_IMU extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int xacc; // X acceleration (raw)
		private int yacc; // Y acceleration (raw)
		private int zacc; // Z acceleration (raw)
		private int xgyro; // Angular speed around X axis (raw)
		private int ygyro; // Angular speed around Y axis (raw)
		private int zgyro; // Angular speed around Z axis (raw)
		private int xmag; // X Magnetic field (raw)
		private int ymag; // Y Magnetic field (raw)
		private int zmag; // Z Magnetic field (raw)
	
		public MSG_RAW_IMU () {
			super(MSG_ID_RAW_IMU, (short)82, "RAW_IMU");
		}

		public MSG_RAW_IMU (byte[] bytes) {
			super(bytes, MSG_ID_RAW_IMU, (short)82, "RAW_IMU");
		}
	
		public MSG_RAW_IMU (short sys, short comp, long time_usec  , int xacc  , int yacc  , int zacc  , int xgyro  , int ygyro  , int zgyro  , int xmag  , int ymag  , int zmag  ) {
			super(sys, comp, (short)82, "RAW_IMU");
			this.time_usec = time_usec;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
		}

		@Override
		public int getCRCExtra() {
			return 144;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
		public int getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(int xgyro) {
			this.xgyro = xgyro;
		}
		
		public int getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(int ygyro) {
			this.ygyro = ygyro;
		}
		
		public int getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(int zgyro) {
			this.zgyro = zgyro;
		}
		
		public int getXmag() {
			return xmag;
		}
		
		public void setXmag(int xmag) {
			this.xmag = xmag;
		}
		
		public int getYmag() {
			return ymag;
		}
		
		public void setYmag(int ymag) {
			this.ymag = ymag;
		}
		
		public int getZmag() {
			return zmag;
		}
		
		public void setZmag(int zmag) {
			this.zmag = zmag;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RAW_IMU";
			
			time_usec = buffer.getLong(); // uint64_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
  			xgyro = buffer.getShort(); // int16_t
  			ygyro = buffer.getShort(); // int16_t
  			zgyro = buffer.getShort(); // int16_t
  			xmag = buffer.getShort(); // int16_t
  			ymag = buffer.getShort(); // int16_t
  			zmag = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
  			buffer.putShort((short)(xgyro)); // int16_t
  			buffer.putShort((short)(ygyro)); // int16_t
  			buffer.putShort((short)(zgyro)); // int16_t
  			buffer.putShort((short)(xmag)); // int16_t
  			buffer.putShort((short)(ymag)); // int16_t
  			buffer.putShort((short)(zmag)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RAW_IMU { " + 
			"time_usec = " + time_usec + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ",  }";
		}
	}
	/*
	 * The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
	 */
	public static class MSG_RAW_PRESSURE extends Message {
	
		private long time_usec; // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
		private int press_abs; // Absolute pressure (raw)
		private int press_diff1; // Differential pressure 1 (raw)
		private int press_diff2; // Differential pressure 2 (raw)
		private int temperature; // Raw Temperature measurement (raw)
	
		public MSG_RAW_PRESSURE () {
			super(MSG_ID_RAW_PRESSURE, (short)72, "RAW_PRESSURE");
		}

		public MSG_RAW_PRESSURE (byte[] bytes) {
			super(bytes, MSG_ID_RAW_PRESSURE, (short)72, "RAW_PRESSURE");
		}
	
		public MSG_RAW_PRESSURE (short sys, short comp, long time_usec  , int press_abs  , int press_diff1  , int press_diff2  , int temperature  ) {
			super(sys, comp, (short)72, "RAW_PRESSURE");
			this.time_usec = time_usec;
			this.press_abs = press_abs;
			this.press_diff1 = press_diff1;
			this.press_diff2 = press_diff2;
			this.temperature = temperature;
		}

		@Override
		public int getCRCExtra() {
			return 67;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getPress_abs() {
			return press_abs;
		}
		
		public void setPress_abs(int press_abs) {
			this.press_abs = press_abs;
		}
		
		public int getPress_diff1() {
			return press_diff1;
		}
		
		public void setPress_diff1(int press_diff1) {
			this.press_diff1 = press_diff1;
		}
		
		public int getPress_diff2() {
			return press_diff2;
		}
		
		public void setPress_diff2(int press_diff2) {
			this.press_diff2 = press_diff2;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RAW_PRESSURE";
			
			time_usec = buffer.getLong(); // uint64_t
  			press_abs = buffer.getShort(); // int16_t
  			press_diff1 = buffer.getShort(); // int16_t
  			press_diff2 = buffer.getShort(); // int16_t
  			temperature = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			buffer.putShort((short)(press_abs)); // int16_t
  			buffer.putShort((short)(press_diff1)); // int16_t
  			buffer.putShort((short)(press_diff2)); // int16_t
  			buffer.putShort((short)(temperature)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RAW_PRESSURE { " + 
			"time_usec = " + time_usec + ", " + 
			"press_abs = " + press_abs + ", " + 
			"press_diff1 = " + press_diff1 + ", " + 
			"press_diff2 = " + press_diff2 + ", " + 
			"temperature = " + temperature + ",  }";
		}
	}
	/*
	 * The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
	 */
	public static class MSG_RC_CHANNELS extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int chan1_raw; // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan2_raw; // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan3_raw; // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan4_raw; // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan5_raw; // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan6_raw; // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan7_raw; // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan8_raw; // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan9_raw; // RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan10_raw; // RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan11_raw; // RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan12_raw; // RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan13_raw; // RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan14_raw; // RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan15_raw; // RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan16_raw; // RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan17_raw; // RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan18_raw; // RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chancount; // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
		private int rssi; // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
		public MSG_RC_CHANNELS () {
			super(MSG_ID_RC_CHANNELS, (short)42, "RC_CHANNELS");
		}

		public MSG_RC_CHANNELS (byte[] bytes) {
			super(bytes, MSG_ID_RC_CHANNELS, (short)42, "RC_CHANNELS");
		}
	
		public MSG_RC_CHANNELS (short sys, short comp, long time_boot_ms  , int chan1_raw  , int chan2_raw  , int chan3_raw  , int chan4_raw  , int chan5_raw  , int chan6_raw  , int chan7_raw  , int chan8_raw  , int chan9_raw  , int chan10_raw  , int chan11_raw  , int chan12_raw  , int chan13_raw  , int chan14_raw  , int chan15_raw  , int chan16_raw  , int chan17_raw  , int chan18_raw  , int chancount  , int rssi  ) {
			super(sys, comp, (short)42, "RC_CHANNELS");
			this.time_boot_ms = time_boot_ms;
			this.chan1_raw = chan1_raw;
			this.chan2_raw = chan2_raw;
			this.chan3_raw = chan3_raw;
			this.chan4_raw = chan4_raw;
			this.chan5_raw = chan5_raw;
			this.chan6_raw = chan6_raw;
			this.chan7_raw = chan7_raw;
			this.chan8_raw = chan8_raw;
			this.chan9_raw = chan9_raw;
			this.chan10_raw = chan10_raw;
			this.chan11_raw = chan11_raw;
			this.chan12_raw = chan12_raw;
			this.chan13_raw = chan13_raw;
			this.chan14_raw = chan14_raw;
			this.chan15_raw = chan15_raw;
			this.chan16_raw = chan16_raw;
			this.chan17_raw = chan17_raw;
			this.chan18_raw = chan18_raw;
			this.chancount = chancount;
			this.rssi = rssi;
		}

		@Override
		public int getCRCExtra() {
			return 118;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getChan1_raw() {
			return chan1_raw;
		}
		
		public void setChan1_raw(int chan1_raw) {
			this.chan1_raw = chan1_raw;
		}
		
		public int getChan2_raw() {
			return chan2_raw;
		}
		
		public void setChan2_raw(int chan2_raw) {
			this.chan2_raw = chan2_raw;
		}
		
		public int getChan3_raw() {
			return chan3_raw;
		}
		
		public void setChan3_raw(int chan3_raw) {
			this.chan3_raw = chan3_raw;
		}
		
		public int getChan4_raw() {
			return chan4_raw;
		}
		
		public void setChan4_raw(int chan4_raw) {
			this.chan4_raw = chan4_raw;
		}
		
		public int getChan5_raw() {
			return chan5_raw;
		}
		
		public void setChan5_raw(int chan5_raw) {
			this.chan5_raw = chan5_raw;
		}
		
		public int getChan6_raw() {
			return chan6_raw;
		}
		
		public void setChan6_raw(int chan6_raw) {
			this.chan6_raw = chan6_raw;
		}
		
		public int getChan7_raw() {
			return chan7_raw;
		}
		
		public void setChan7_raw(int chan7_raw) {
			this.chan7_raw = chan7_raw;
		}
		
		public int getChan8_raw() {
			return chan8_raw;
		}
		
		public void setChan8_raw(int chan8_raw) {
			this.chan8_raw = chan8_raw;
		}
		
		public int getChan9_raw() {
			return chan9_raw;
		}
		
		public void setChan9_raw(int chan9_raw) {
			this.chan9_raw = chan9_raw;
		}
		
		public int getChan10_raw() {
			return chan10_raw;
		}
		
		public void setChan10_raw(int chan10_raw) {
			this.chan10_raw = chan10_raw;
		}
		
		public int getChan11_raw() {
			return chan11_raw;
		}
		
		public void setChan11_raw(int chan11_raw) {
			this.chan11_raw = chan11_raw;
		}
		
		public int getChan12_raw() {
			return chan12_raw;
		}
		
		public void setChan12_raw(int chan12_raw) {
			this.chan12_raw = chan12_raw;
		}
		
		public int getChan13_raw() {
			return chan13_raw;
		}
		
		public void setChan13_raw(int chan13_raw) {
			this.chan13_raw = chan13_raw;
		}
		
		public int getChan14_raw() {
			return chan14_raw;
		}
		
		public void setChan14_raw(int chan14_raw) {
			this.chan14_raw = chan14_raw;
		}
		
		public int getChan15_raw() {
			return chan15_raw;
		}
		
		public void setChan15_raw(int chan15_raw) {
			this.chan15_raw = chan15_raw;
		}
		
		public int getChan16_raw() {
			return chan16_raw;
		}
		
		public void setChan16_raw(int chan16_raw) {
			this.chan16_raw = chan16_raw;
		}
		
		public int getChan17_raw() {
			return chan17_raw;
		}
		
		public void setChan17_raw(int chan17_raw) {
			this.chan17_raw = chan17_raw;
		}
		
		public int getChan18_raw() {
			return chan18_raw;
		}
		
		public void setChan18_raw(int chan18_raw) {
			this.chan18_raw = chan18_raw;
		}
		
		public int getChancount() {
			return chancount;
		}
		
		public void setChancount(int chancount) {
			this.chancount = chancount;
		}
		
		public int getRssi() {
			return rssi;
		}
		
		public void setRssi(int rssi) {
			this.rssi = rssi;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RC_CHANNELS";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			chan1_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan2_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan3_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan4_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan5_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan6_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan7_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan8_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan9_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan10_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan11_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan12_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan13_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan14_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan15_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan16_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan17_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan18_raw = buffer.getShort() & 0xffff; // uint16_t
  			chancount = (int)buffer.get() & 0xff; // uint8_t
  			rssi = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(chan1_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan2_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan3_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan4_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan5_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan6_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan7_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan8_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan9_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan10_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan11_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan12_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan13_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan14_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan15_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan16_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan17_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan18_raw & 0xffff)); // uint16_t
  			buffer.put((byte)(chancount & 0xff)); // uint8_t
  			buffer.put((byte)(rssi & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RC_CHANNELS { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"chan1_raw = " + chan1_raw + ", " + 
			"chan2_raw = " + chan2_raw + ", " + 
			"chan3_raw = " + chan3_raw + ", " + 
			"chan4_raw = " + chan4_raw + ", " + 
			"chan5_raw = " + chan5_raw + ", " + 
			"chan6_raw = " + chan6_raw + ", " + 
			"chan7_raw = " + chan7_raw + ", " + 
			"chan8_raw = " + chan8_raw + ", " + 
			"chan9_raw = " + chan9_raw + ", " + 
			"chan10_raw = " + chan10_raw + ", " + 
			"chan11_raw = " + chan11_raw + ", " + 
			"chan12_raw = " + chan12_raw + ", " + 
			"chan13_raw = " + chan13_raw + ", " + 
			"chan14_raw = " + chan14_raw + ", " + 
			"chan15_raw = " + chan15_raw + ", " + 
			"chan16_raw = " + chan16_raw + ", " + 
			"chan17_raw = " + chan17_raw + ", " + 
			"chan18_raw = " + chan18_raw + ", " + 
			"chancount = " + chancount + ", " + 
			"rssi = " + rssi + ",  }";
		}
	}
	/*
	 * The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
	 */
	public static class MSG_RC_CHANNELS_OVERRIDE extends Message {
	
		private int chan1_raw; // RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan2_raw; // RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan3_raw; // RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan4_raw; // RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan5_raw; // RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan6_raw; // RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan7_raw; // RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int chan8_raw; // RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_RC_CHANNELS_OVERRIDE () {
			super(MSG_ID_RC_CHANNELS_OVERRIDE, (short)18, "RC_CHANNELS_OVERRIDE");
		}

		public MSG_RC_CHANNELS_OVERRIDE (byte[] bytes) {
			super(bytes, MSG_ID_RC_CHANNELS_OVERRIDE, (short)18, "RC_CHANNELS_OVERRIDE");
		}
	
		public MSG_RC_CHANNELS_OVERRIDE (short sys, short comp, int chan1_raw  , int chan2_raw  , int chan3_raw  , int chan4_raw  , int chan5_raw  , int chan6_raw  , int chan7_raw  , int chan8_raw  , int target_system  , int target_component  ) {
			super(sys, comp, (short)18, "RC_CHANNELS_OVERRIDE");
			this.chan1_raw = chan1_raw;
			this.chan2_raw = chan2_raw;
			this.chan3_raw = chan3_raw;
			this.chan4_raw = chan4_raw;
			this.chan5_raw = chan5_raw;
			this.chan6_raw = chan6_raw;
			this.chan7_raw = chan7_raw;
			this.chan8_raw = chan8_raw;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 124;
		}
	
		public int getChan1_raw() {
			return chan1_raw;
		}
		
		public void setChan1_raw(int chan1_raw) {
			this.chan1_raw = chan1_raw;
		}
		
		public int getChan2_raw() {
			return chan2_raw;
		}
		
		public void setChan2_raw(int chan2_raw) {
			this.chan2_raw = chan2_raw;
		}
		
		public int getChan3_raw() {
			return chan3_raw;
		}
		
		public void setChan3_raw(int chan3_raw) {
			this.chan3_raw = chan3_raw;
		}
		
		public int getChan4_raw() {
			return chan4_raw;
		}
		
		public void setChan4_raw(int chan4_raw) {
			this.chan4_raw = chan4_raw;
		}
		
		public int getChan5_raw() {
			return chan5_raw;
		}
		
		public void setChan5_raw(int chan5_raw) {
			this.chan5_raw = chan5_raw;
		}
		
		public int getChan6_raw() {
			return chan6_raw;
		}
		
		public void setChan6_raw(int chan6_raw) {
			this.chan6_raw = chan6_raw;
		}
		
		public int getChan7_raw() {
			return chan7_raw;
		}
		
		public void setChan7_raw(int chan7_raw) {
			this.chan7_raw = chan7_raw;
		}
		
		public int getChan8_raw() {
			return chan8_raw;
		}
		
		public void setChan8_raw(int chan8_raw) {
			this.chan8_raw = chan8_raw;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RC_CHANNELS_OVERRIDE";
			
			chan1_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan2_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan3_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan4_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan5_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan6_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan7_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan8_raw = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(chan1_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan2_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan3_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan4_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan5_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan6_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan7_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan8_raw & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RC_CHANNELS_OVERRIDE { " + 
			"chan1_raw = " + chan1_raw + ", " + 
			"chan2_raw = " + chan2_raw + ", " + 
			"chan3_raw = " + chan3_raw + ", " + 
			"chan4_raw = " + chan4_raw + ", " + 
			"chan5_raw = " + chan5_raw + ", " + 
			"chan6_raw = " + chan6_raw + ", " + 
			"chan7_raw = " + chan7_raw + ", " + 
			"chan8_raw = " + chan8_raw + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
	 */
	public static class MSG_RC_CHANNELS_RAW extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int chan1_raw; // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan2_raw; // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan3_raw; // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan4_raw; // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan5_raw; // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan6_raw; // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan7_raw; // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int chan8_raw; // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
		private int port; // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
		private int rssi; // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
		public MSG_RC_CHANNELS_RAW () {
			super(MSG_ID_RC_CHANNELS_RAW, (short)22, "RC_CHANNELS_RAW");
		}

		public MSG_RC_CHANNELS_RAW (byte[] bytes) {
			super(bytes, MSG_ID_RC_CHANNELS_RAW, (short)22, "RC_CHANNELS_RAW");
		}
	
		public MSG_RC_CHANNELS_RAW (short sys, short comp, long time_boot_ms  , int chan1_raw  , int chan2_raw  , int chan3_raw  , int chan4_raw  , int chan5_raw  , int chan6_raw  , int chan7_raw  , int chan8_raw  , int port  , int rssi  ) {
			super(sys, comp, (short)22, "RC_CHANNELS_RAW");
			this.time_boot_ms = time_boot_ms;
			this.chan1_raw = chan1_raw;
			this.chan2_raw = chan2_raw;
			this.chan3_raw = chan3_raw;
			this.chan4_raw = chan4_raw;
			this.chan5_raw = chan5_raw;
			this.chan6_raw = chan6_raw;
			this.chan7_raw = chan7_raw;
			this.chan8_raw = chan8_raw;
			this.port = port;
			this.rssi = rssi;
		}

		@Override
		public int getCRCExtra() {
			return 244;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getChan1_raw() {
			return chan1_raw;
		}
		
		public void setChan1_raw(int chan1_raw) {
			this.chan1_raw = chan1_raw;
		}
		
		public int getChan2_raw() {
			return chan2_raw;
		}
		
		public void setChan2_raw(int chan2_raw) {
			this.chan2_raw = chan2_raw;
		}
		
		public int getChan3_raw() {
			return chan3_raw;
		}
		
		public void setChan3_raw(int chan3_raw) {
			this.chan3_raw = chan3_raw;
		}
		
		public int getChan4_raw() {
			return chan4_raw;
		}
		
		public void setChan4_raw(int chan4_raw) {
			this.chan4_raw = chan4_raw;
		}
		
		public int getChan5_raw() {
			return chan5_raw;
		}
		
		public void setChan5_raw(int chan5_raw) {
			this.chan5_raw = chan5_raw;
		}
		
		public int getChan6_raw() {
			return chan6_raw;
		}
		
		public void setChan6_raw(int chan6_raw) {
			this.chan6_raw = chan6_raw;
		}
		
		public int getChan7_raw() {
			return chan7_raw;
		}
		
		public void setChan7_raw(int chan7_raw) {
			this.chan7_raw = chan7_raw;
		}
		
		public int getChan8_raw() {
			return chan8_raw;
		}
		
		public void setChan8_raw(int chan8_raw) {
			this.chan8_raw = chan8_raw;
		}
		
		public int getPort() {
			return port;
		}
		
		public void setPort(int port) {
			this.port = port;
		}
		
		public int getRssi() {
			return rssi;
		}
		
		public void setRssi(int rssi) {
			this.rssi = rssi;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RC_CHANNELS_RAW";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			chan1_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan2_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan3_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan4_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan5_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan6_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan7_raw = buffer.getShort() & 0xffff; // uint16_t
  			chan8_raw = buffer.getShort() & 0xffff; // uint16_t
  			port = (int)buffer.get() & 0xff; // uint8_t
  			rssi = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(chan1_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan2_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan3_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan4_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan5_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan6_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan7_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(chan8_raw & 0xffff)); // uint16_t
  			buffer.put((byte)(port & 0xff)); // uint8_t
  			buffer.put((byte)(rssi & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RC_CHANNELS_RAW { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"chan1_raw = " + chan1_raw + ", " + 
			"chan2_raw = " + chan2_raw + ", " + 
			"chan3_raw = " + chan3_raw + ", " + 
			"chan4_raw = " + chan4_raw + ", " + 
			"chan5_raw = " + chan5_raw + ", " + 
			"chan6_raw = " + chan6_raw + ", " + 
			"chan7_raw = " + chan7_raw + ", " + 
			"chan8_raw = " + chan8_raw + ", " + 
			"port = " + port + ", " + 
			"rssi = " + rssi + ",  }";
		}
	}
	/*
	 * The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
	 */
	public static class MSG_RC_CHANNELS_SCALED extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int chan1_scaled; // RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan2_scaled; // RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan3_scaled; // RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan4_scaled; // RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan5_scaled; // RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan6_scaled; // RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan7_scaled; // RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int chan8_scaled; // RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
		private int port; // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
		private int rssi; // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	
		public MSG_RC_CHANNELS_SCALED () {
			super(MSG_ID_RC_CHANNELS_SCALED, (short)22, "RC_CHANNELS_SCALED");
		}

		public MSG_RC_CHANNELS_SCALED (byte[] bytes) {
			super(bytes, MSG_ID_RC_CHANNELS_SCALED, (short)22, "RC_CHANNELS_SCALED");
		}
	
		public MSG_RC_CHANNELS_SCALED (short sys, short comp, long time_boot_ms  , int chan1_scaled  , int chan2_scaled  , int chan3_scaled  , int chan4_scaled  , int chan5_scaled  , int chan6_scaled  , int chan7_scaled  , int chan8_scaled  , int port  , int rssi  ) {
			super(sys, comp, (short)22, "RC_CHANNELS_SCALED");
			this.time_boot_ms = time_boot_ms;
			this.chan1_scaled = chan1_scaled;
			this.chan2_scaled = chan2_scaled;
			this.chan3_scaled = chan3_scaled;
			this.chan4_scaled = chan4_scaled;
			this.chan5_scaled = chan5_scaled;
			this.chan6_scaled = chan6_scaled;
			this.chan7_scaled = chan7_scaled;
			this.chan8_scaled = chan8_scaled;
			this.port = port;
			this.rssi = rssi;
		}

		@Override
		public int getCRCExtra() {
			return 237;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getChan1_scaled() {
			return chan1_scaled;
		}
		
		public void setChan1_scaled(int chan1_scaled) {
			this.chan1_scaled = chan1_scaled;
		}
		
		public int getChan2_scaled() {
			return chan2_scaled;
		}
		
		public void setChan2_scaled(int chan2_scaled) {
			this.chan2_scaled = chan2_scaled;
		}
		
		public int getChan3_scaled() {
			return chan3_scaled;
		}
		
		public void setChan3_scaled(int chan3_scaled) {
			this.chan3_scaled = chan3_scaled;
		}
		
		public int getChan4_scaled() {
			return chan4_scaled;
		}
		
		public void setChan4_scaled(int chan4_scaled) {
			this.chan4_scaled = chan4_scaled;
		}
		
		public int getChan5_scaled() {
			return chan5_scaled;
		}
		
		public void setChan5_scaled(int chan5_scaled) {
			this.chan5_scaled = chan5_scaled;
		}
		
		public int getChan6_scaled() {
			return chan6_scaled;
		}
		
		public void setChan6_scaled(int chan6_scaled) {
			this.chan6_scaled = chan6_scaled;
		}
		
		public int getChan7_scaled() {
			return chan7_scaled;
		}
		
		public void setChan7_scaled(int chan7_scaled) {
			this.chan7_scaled = chan7_scaled;
		}
		
		public int getChan8_scaled() {
			return chan8_scaled;
		}
		
		public void setChan8_scaled(int chan8_scaled) {
			this.chan8_scaled = chan8_scaled;
		}
		
		public int getPort() {
			return port;
		}
		
		public void setPort(int port) {
			this.port = port;
		}
		
		public int getRssi() {
			return rssi;
		}
		
		public void setRssi(int rssi) {
			this.rssi = rssi;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "RC_CHANNELS_SCALED";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			chan1_scaled = buffer.getShort(); // int16_t
  			chan2_scaled = buffer.getShort(); // int16_t
  			chan3_scaled = buffer.getShort(); // int16_t
  			chan4_scaled = buffer.getShort(); // int16_t
  			chan5_scaled = buffer.getShort(); // int16_t
  			chan6_scaled = buffer.getShort(); // int16_t
  			chan7_scaled = buffer.getShort(); // int16_t
  			chan8_scaled = buffer.getShort(); // int16_t
  			port = (int)buffer.get() & 0xff; // uint8_t
  			rssi = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(chan1_scaled)); // int16_t
  			buffer.putShort((short)(chan2_scaled)); // int16_t
  			buffer.putShort((short)(chan3_scaled)); // int16_t
  			buffer.putShort((short)(chan4_scaled)); // int16_t
  			buffer.putShort((short)(chan5_scaled)); // int16_t
  			buffer.putShort((short)(chan6_scaled)); // int16_t
  			buffer.putShort((short)(chan7_scaled)); // int16_t
  			buffer.putShort((short)(chan8_scaled)); // int16_t
  			buffer.put((byte)(port & 0xff)); // uint8_t
  			buffer.put((byte)(rssi & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_RC_CHANNELS_SCALED { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"chan1_scaled = " + chan1_scaled + ", " + 
			"chan2_scaled = " + chan2_scaled + ", " + 
			"chan3_scaled = " + chan3_scaled + ", " + 
			"chan4_scaled = " + chan4_scaled + ", " + 
			"chan5_scaled = " + chan5_scaled + ", " + 
			"chan6_scaled = " + chan6_scaled + ", " + 
			"chan7_scaled = " + chan7_scaled + ", " + 
			"chan8_scaled = " + chan8_scaled + ", " + 
			"port = " + port + ", " + 
			"rssi = " + rssi + ",  }";
		}
	}
	public static class MSG_REQUEST_DATA_STREAM extends Message {
	
		private int req_message_rate; // The requested interval between two messages of this type
		private int target_system; // The target requested to send the message stream.
		private int target_component; // The target requested to send the message stream.
		private int req_stream_id; // The ID of the requested data stream
		private int start_stop; // 1 to start sending, 0 to stop sending.
	
		public MSG_REQUEST_DATA_STREAM () {
			super(MSG_ID_REQUEST_DATA_STREAM, (short)6, "REQUEST_DATA_STREAM");
		}

		public MSG_REQUEST_DATA_STREAM (byte[] bytes) {
			super(bytes, MSG_ID_REQUEST_DATA_STREAM, (short)6, "REQUEST_DATA_STREAM");
		}
	
		public MSG_REQUEST_DATA_STREAM (short sys, short comp, int req_message_rate  , int target_system  , int target_component  , int req_stream_id  , int start_stop  ) {
			super(sys, comp, (short)6, "REQUEST_DATA_STREAM");
			this.req_message_rate = req_message_rate;
			this.target_system = target_system;
			this.target_component = target_component;
			this.req_stream_id = req_stream_id;
			this.start_stop = start_stop;
		}

		@Override
		public int getCRCExtra() {
			return 148;
		}
	
		public int getReq_message_rate() {
			return req_message_rate;
		}
		
		public void setReq_message_rate(int req_message_rate) {
			this.req_message_rate = req_message_rate;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getReq_stream_id() {
			return req_stream_id;
		}
		
		public void setReq_stream_id(int req_stream_id) {
			this.req_stream_id = req_stream_id;
		}
		
		public int getStart_stop() {
			return start_stop;
		}
		
		public void setStart_stop(int start_stop) {
			this.start_stop = start_stop;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "REQUEST_DATA_STREAM";
			
			req_message_rate = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			req_stream_id = (int)buffer.get() & 0xff; // uint8_t
  			start_stop = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(req_message_rate & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(req_stream_id & 0xff)); // uint8_t
  			buffer.put((byte)(start_stop & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_REQUEST_DATA_STREAM { " + 
			"req_message_rate = " + req_message_rate + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"req_stream_id = " + req_stream_id + ", " + 
			"start_stop = " + start_stop + ",  }";
		}
	}
	/*
	 * Read out the safety zone the MAV currently assumes.
	 */
	public static class MSG_SAFETY_ALLOWED_AREA extends Message {
	
		private float p1x; // x position 1 / Latitude 1
		private float p1y; // y position 1 / Longitude 1
		private float p1z; // z position 1 / Altitude 1
		private float p2x; // x position 2 / Latitude 2
		private float p2y; // y position 2 / Longitude 2
		private float p2z; // z position 2 / Altitude 2
		private int frame; // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	
		public MSG_SAFETY_ALLOWED_AREA () {
			super(MSG_ID_SAFETY_ALLOWED_AREA, (short)25, "SAFETY_ALLOWED_AREA");
		}

		public MSG_SAFETY_ALLOWED_AREA (byte[] bytes) {
			super(bytes, MSG_ID_SAFETY_ALLOWED_AREA, (short)25, "SAFETY_ALLOWED_AREA");
		}
	
		public MSG_SAFETY_ALLOWED_AREA (short sys, short comp, float p1x  , float p1y  , float p1z  , float p2x  , float p2y  , float p2z  , int frame  ) {
			super(sys, comp, (short)25, "SAFETY_ALLOWED_AREA");
			this.p1x = p1x;
			this.p1y = p1y;
			this.p1z = p1z;
			this.p2x = p2x;
			this.p2y = p2y;
			this.p2z = p2z;
			this.frame = frame;
		}

		@Override
		public int getCRCExtra() {
			return 3;
		}
	
		public float getP1x() {
			return p1x;
		}
		
		public void setP1x(float p1x) {
			this.p1x = p1x;
		}
		
		public float getP1y() {
			return p1y;
		}
		
		public void setP1y(float p1y) {
			this.p1y = p1y;
		}
		
		public float getP1z() {
			return p1z;
		}
		
		public void setP1z(float p1z) {
			this.p1z = p1z;
		}
		
		public float getP2x() {
			return p2x;
		}
		
		public void setP2x(float p2x) {
			this.p2x = p2x;
		}
		
		public float getP2y() {
			return p2y;
		}
		
		public void setP2y(float p2y) {
			this.p2y = p2y;
		}
		
		public float getP2z() {
			return p2z;
		}
		
		public void setP2z(float p2z) {
			this.p2z = p2z;
		}
		
		public int getFrame() {
			return frame;
		}
		
		public void setFrame(int frame) {
			this.frame = frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SAFETY_ALLOWED_AREA";
			
			p1x = buffer.getFloat(); // float
  			p1y = buffer.getFloat(); // float
  			p1z = buffer.getFloat(); // float
  			p2x = buffer.getFloat(); // float
  			p2y = buffer.getFloat(); // float
  			p2z = buffer.getFloat(); // float
  			frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(p1x); // float
  			buffer.putFloat(p1y); // float
  			buffer.putFloat(p1z); // float
  			buffer.putFloat(p2x); // float
  			buffer.putFloat(p2y); // float
  			buffer.putFloat(p2z); // float
  			buffer.put((byte)(frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SAFETY_ALLOWED_AREA { " + 
			"p1x = " + p1x + ", " + 
			"p1y = " + p1y + ", " + 
			"p1z = " + p1z + ", " + 
			"p2x = " + p2x + ", " + 
			"p2y = " + p2y + ", " + 
			"p2z = " + p2z + ", " + 
			"frame = " + frame + ",  }";
		}
	}
	/*
	 * Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
	 */
	public static class MSG_SAFETY_SET_ALLOWED_AREA extends Message {
	
		private float p1x; // x position 1 / Latitude 1
		private float p1y; // y position 1 / Longitude 1
		private float p1z; // z position 1 / Altitude 1
		private float p2x; // x position 2 / Latitude 2
		private float p2y; // y position 2 / Longitude 2
		private float p2z; // z position 2 / Altitude 2
		private int target_system; // System ID
		private int target_component; // Component ID
		private int frame; // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	
		public MSG_SAFETY_SET_ALLOWED_AREA () {
			super(MSG_ID_SAFETY_SET_ALLOWED_AREA, (short)27, "SAFETY_SET_ALLOWED_AREA");
		}

		public MSG_SAFETY_SET_ALLOWED_AREA (byte[] bytes) {
			super(bytes, MSG_ID_SAFETY_SET_ALLOWED_AREA, (short)27, "SAFETY_SET_ALLOWED_AREA");
		}
	
		public MSG_SAFETY_SET_ALLOWED_AREA (short sys, short comp, float p1x  , float p1y  , float p1z  , float p2x  , float p2y  , float p2z  , int target_system  , int target_component  , int frame  ) {
			super(sys, comp, (short)27, "SAFETY_SET_ALLOWED_AREA");
			this.p1x = p1x;
			this.p1y = p1y;
			this.p1z = p1z;
			this.p2x = p2x;
			this.p2y = p2y;
			this.p2z = p2z;
			this.target_system = target_system;
			this.target_component = target_component;
			this.frame = frame;
		}

		@Override
		public int getCRCExtra() {
			return 15;
		}
	
		public float getP1x() {
			return p1x;
		}
		
		public void setP1x(float p1x) {
			this.p1x = p1x;
		}
		
		public float getP1y() {
			return p1y;
		}
		
		public void setP1y(float p1y) {
			this.p1y = p1y;
		}
		
		public float getP1z() {
			return p1z;
		}
		
		public void setP1z(float p1z) {
			this.p1z = p1z;
		}
		
		public float getP2x() {
			return p2x;
		}
		
		public void setP2x(float p2x) {
			this.p2x = p2x;
		}
		
		public float getP2y() {
			return p2y;
		}
		
		public void setP2y(float p2y) {
			this.p2y = p2y;
		}
		
		public float getP2z() {
			return p2z;
		}
		
		public void setP2z(float p2z) {
			this.p2z = p2z;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getFrame() {
			return frame;
		}
		
		public void setFrame(int frame) {
			this.frame = frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SAFETY_SET_ALLOWED_AREA";
			
			p1x = buffer.getFloat(); // float
  			p1y = buffer.getFloat(); // float
  			p1z = buffer.getFloat(); // float
  			p2x = buffer.getFloat(); // float
  			p2y = buffer.getFloat(); // float
  			p2z = buffer.getFloat(); // float
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(p1x); // float
  			buffer.putFloat(p1y); // float
  			buffer.putFloat(p1z); // float
  			buffer.putFloat(p2x); // float
  			buffer.putFloat(p2y); // float
  			buffer.putFloat(p2z); // float
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SAFETY_SET_ALLOWED_AREA { " + 
			"p1x = " + p1x + ", " + 
			"p1y = " + p1y + ", " + 
			"p1z = " + p1z + ", " + 
			"p2x = " + p2x + ", " + 
			"p2y = " + p2y + ", " + 
			"p2z = " + p2z + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"frame = " + frame + ",  }";
		}
	}
	/*
	 * The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
	 */
	public static class MSG_SCALED_IMU extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int xacc; // X acceleration (mg)
		private int yacc; // Y acceleration (mg)
		private int zacc; // Z acceleration (mg)
		private int xgyro; // Angular speed around X axis (millirad /sec)
		private int ygyro; // Angular speed around Y axis (millirad /sec)
		private int zgyro; // Angular speed around Z axis (millirad /sec)
		private int xmag; // X Magnetic field (milli tesla)
		private int ymag; // Y Magnetic field (milli tesla)
		private int zmag; // Z Magnetic field (milli tesla)
	
		public MSG_SCALED_IMU () {
			super(MSG_ID_SCALED_IMU, (short)22, "SCALED_IMU");
		}

		public MSG_SCALED_IMU (byte[] bytes) {
			super(bytes, MSG_ID_SCALED_IMU, (short)22, "SCALED_IMU");
		}
	
		public MSG_SCALED_IMU (short sys, short comp, long time_boot_ms  , int xacc  , int yacc  , int zacc  , int xgyro  , int ygyro  , int zgyro  , int xmag  , int ymag  , int zmag  ) {
			super(sys, comp, (short)22, "SCALED_IMU");
			this.time_boot_ms = time_boot_ms;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
		}

		@Override
		public int getCRCExtra() {
			return 170;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
		public int getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(int xgyro) {
			this.xgyro = xgyro;
		}
		
		public int getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(int ygyro) {
			this.ygyro = ygyro;
		}
		
		public int getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(int zgyro) {
			this.zgyro = zgyro;
		}
		
		public int getXmag() {
			return xmag;
		}
		
		public void setXmag(int xmag) {
			this.xmag = xmag;
		}
		
		public int getYmag() {
			return ymag;
		}
		
		public void setYmag(int ymag) {
			this.ymag = ymag;
		}
		
		public int getZmag() {
			return zmag;
		}
		
		public void setZmag(int zmag) {
			this.zmag = zmag;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SCALED_IMU";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
  			xgyro = buffer.getShort(); // int16_t
  			ygyro = buffer.getShort(); // int16_t
  			zgyro = buffer.getShort(); // int16_t
  			xmag = buffer.getShort(); // int16_t
  			ymag = buffer.getShort(); // int16_t
  			zmag = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
  			buffer.putShort((short)(xgyro)); // int16_t
  			buffer.putShort((short)(ygyro)); // int16_t
  			buffer.putShort((short)(zgyro)); // int16_t
  			buffer.putShort((short)(xmag)); // int16_t
  			buffer.putShort((short)(ymag)); // int16_t
  			buffer.putShort((short)(zmag)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SCALED_IMU { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ",  }";
		}
	}
	/*
	 * The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
	 */
	public static class MSG_SCALED_IMU2 extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int xacc; // X acceleration (mg)
		private int yacc; // Y acceleration (mg)
		private int zacc; // Z acceleration (mg)
		private int xgyro; // Angular speed around X axis (millirad /sec)
		private int ygyro; // Angular speed around Y axis (millirad /sec)
		private int zgyro; // Angular speed around Z axis (millirad /sec)
		private int xmag; // X Magnetic field (milli tesla)
		private int ymag; // Y Magnetic field (milli tesla)
		private int zmag; // Z Magnetic field (milli tesla)
	
		public MSG_SCALED_IMU2 () {
			super(MSG_ID_SCALED_IMU2, (short)22, "SCALED_IMU2");
		}

		public MSG_SCALED_IMU2 (byte[] bytes) {
			super(bytes, MSG_ID_SCALED_IMU2, (short)22, "SCALED_IMU2");
		}
	
		public MSG_SCALED_IMU2 (short sys, short comp, long time_boot_ms  , int xacc  , int yacc  , int zacc  , int xgyro  , int ygyro  , int zgyro  , int xmag  , int ymag  , int zmag  ) {
			super(sys, comp, (short)22, "SCALED_IMU2");
			this.time_boot_ms = time_boot_ms;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
		}

		@Override
		public int getCRCExtra() {
			return 76;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
		public int getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(int xgyro) {
			this.xgyro = xgyro;
		}
		
		public int getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(int ygyro) {
			this.ygyro = ygyro;
		}
		
		public int getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(int zgyro) {
			this.zgyro = zgyro;
		}
		
		public int getXmag() {
			return xmag;
		}
		
		public void setXmag(int xmag) {
			this.xmag = xmag;
		}
		
		public int getYmag() {
			return ymag;
		}
		
		public void setYmag(int ymag) {
			this.ymag = ymag;
		}
		
		public int getZmag() {
			return zmag;
		}
		
		public void setZmag(int zmag) {
			this.zmag = zmag;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SCALED_IMU2";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
  			xgyro = buffer.getShort(); // int16_t
  			ygyro = buffer.getShort(); // int16_t
  			zgyro = buffer.getShort(); // int16_t
  			xmag = buffer.getShort(); // int16_t
  			ymag = buffer.getShort(); // int16_t
  			zmag = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
  			buffer.putShort((short)(xgyro)); // int16_t
  			buffer.putShort((short)(ygyro)); // int16_t
  			buffer.putShort((short)(zgyro)); // int16_t
  			buffer.putShort((short)(xmag)); // int16_t
  			buffer.putShort((short)(ymag)); // int16_t
  			buffer.putShort((short)(zmag)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SCALED_IMU2 { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ",  }";
		}
	}
	/*
	 * The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
	 */
	public static class MSG_SCALED_IMU3 extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private int xacc; // X acceleration (mg)
		private int yacc; // Y acceleration (mg)
		private int zacc; // Z acceleration (mg)
		private int xgyro; // Angular speed around X axis (millirad /sec)
		private int ygyro; // Angular speed around Y axis (millirad /sec)
		private int zgyro; // Angular speed around Z axis (millirad /sec)
		private int xmag; // X Magnetic field (milli tesla)
		private int ymag; // Y Magnetic field (milli tesla)
		private int zmag; // Z Magnetic field (milli tesla)
	
		public MSG_SCALED_IMU3 () {
			super(MSG_ID_SCALED_IMU3, (short)22, "SCALED_IMU3");
		}

		public MSG_SCALED_IMU3 (byte[] bytes) {
			super(bytes, MSG_ID_SCALED_IMU3, (short)22, "SCALED_IMU3");
		}
	
		public MSG_SCALED_IMU3 (short sys, short comp, long time_boot_ms  , int xacc  , int yacc  , int zacc  , int xgyro  , int ygyro  , int zgyro  , int xmag  , int ymag  , int zmag  ) {
			super(sys, comp, (short)22, "SCALED_IMU3");
			this.time_boot_ms = time_boot_ms;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.xmag = xmag;
			this.ymag = ymag;
			this.zmag = zmag;
		}

		@Override
		public int getCRCExtra() {
			return 46;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getXacc() {
			return xacc;
		}
		
		public void setXacc(int xacc) {
			this.xacc = xacc;
		}
		
		public int getYacc() {
			return yacc;
		}
		
		public void setYacc(int yacc) {
			this.yacc = yacc;
		}
		
		public int getZacc() {
			return zacc;
		}
		
		public void setZacc(int zacc) {
			this.zacc = zacc;
		}
		
		public int getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(int xgyro) {
			this.xgyro = xgyro;
		}
		
		public int getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(int ygyro) {
			this.ygyro = ygyro;
		}
		
		public int getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(int zgyro) {
			this.zgyro = zgyro;
		}
		
		public int getXmag() {
			return xmag;
		}
		
		public void setXmag(int xmag) {
			this.xmag = xmag;
		}
		
		public int getYmag() {
			return ymag;
		}
		
		public void setYmag(int ymag) {
			this.ymag = ymag;
		}
		
		public int getZmag() {
			return zmag;
		}
		
		public void setZmag(int zmag) {
			this.zmag = zmag;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SCALED_IMU3";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			xacc = buffer.getShort(); // int16_t
  			yacc = buffer.getShort(); // int16_t
  			zacc = buffer.getShort(); // int16_t
  			xgyro = buffer.getShort(); // int16_t
  			ygyro = buffer.getShort(); // int16_t
  			zgyro = buffer.getShort(); // int16_t
  			xmag = buffer.getShort(); // int16_t
  			ymag = buffer.getShort(); // int16_t
  			zmag = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(xacc)); // int16_t
  			buffer.putShort((short)(yacc)); // int16_t
  			buffer.putShort((short)(zacc)); // int16_t
  			buffer.putShort((short)(xgyro)); // int16_t
  			buffer.putShort((short)(ygyro)); // int16_t
  			buffer.putShort((short)(zgyro)); // int16_t
  			buffer.putShort((short)(xmag)); // int16_t
  			buffer.putShort((short)(ymag)); // int16_t
  			buffer.putShort((short)(zmag)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SCALED_IMU3 { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"xmag = " + xmag + ", " + 
			"ymag = " + ymag + ", " + 
			"zmag = " + zmag + ",  }";
		}
	}
	/*
	 * The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
	 */
	public static class MSG_SCALED_PRESSURE extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float press_abs; // Absolute pressure (hectopascal)
		private float press_diff; // Differential pressure 1 (hectopascal)
		private int temperature; // Temperature measurement (0.01 degrees celsius)
	
		public MSG_SCALED_PRESSURE () {
			super(MSG_ID_SCALED_PRESSURE, (short)14, "SCALED_PRESSURE");
		}

		public MSG_SCALED_PRESSURE (byte[] bytes) {
			super(bytes, MSG_ID_SCALED_PRESSURE, (short)14, "SCALED_PRESSURE");
		}
	
		public MSG_SCALED_PRESSURE (short sys, short comp, long time_boot_ms  , float press_abs  , float press_diff  , int temperature  ) {
			super(sys, comp, (short)14, "SCALED_PRESSURE");
			this.time_boot_ms = time_boot_ms;
			this.press_abs = press_abs;
			this.press_diff = press_diff;
			this.temperature = temperature;
		}

		@Override
		public int getCRCExtra() {
			return 115;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getPress_abs() {
			return press_abs;
		}
		
		public void setPress_abs(float press_abs) {
			this.press_abs = press_abs;
		}
		
		public float getPress_diff() {
			return press_diff;
		}
		
		public void setPress_diff(float press_diff) {
			this.press_diff = press_diff;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SCALED_PRESSURE";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			press_abs = buffer.getFloat(); // float
  			press_diff = buffer.getFloat(); // float
  			temperature = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(press_abs); // float
  			buffer.putFloat(press_diff); // float
  			buffer.putShort((short)(temperature)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SCALED_PRESSURE { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"press_abs = " + press_abs + ", " + 
			"press_diff = " + press_diff + ", " + 
			"temperature = " + temperature + ",  }";
		}
	}
	/*
	 * Barometer readings for 2nd barometer
	 */
	public static class MSG_SCALED_PRESSURE2 extends Message {
	
		private long time_boot_ms; // Timestamp (milliseconds since system boot)
		private float press_abs; // Absolute pressure (hectopascal)
		private float press_diff; // Differential pressure 1 (hectopascal)
		private int temperature; // Temperature measurement (0.01 degrees celsius)
	
		public MSG_SCALED_PRESSURE2 () {
			super(MSG_ID_SCALED_PRESSURE2, (short)14, "SCALED_PRESSURE2");
		}

		public MSG_SCALED_PRESSURE2 (byte[] bytes) {
			super(bytes, MSG_ID_SCALED_PRESSURE2, (short)14, "SCALED_PRESSURE2");
		}
	
		public MSG_SCALED_PRESSURE2 (short sys, short comp, long time_boot_ms  , float press_abs  , float press_diff  , int temperature  ) {
			super(sys, comp, (short)14, "SCALED_PRESSURE2");
			this.time_boot_ms = time_boot_ms;
			this.press_abs = press_abs;
			this.press_diff = press_diff;
			this.temperature = temperature;
		}

		@Override
		public int getCRCExtra() {
			return 195;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getPress_abs() {
			return press_abs;
		}
		
		public void setPress_abs(float press_abs) {
			this.press_abs = press_abs;
		}
		
		public float getPress_diff() {
			return press_diff;
		}
		
		public void setPress_diff(float press_diff) {
			this.press_diff = press_diff;
		}
		
		public int getTemperature() {
			return temperature;
		}
		
		public void setTemperature(int temperature) {
			this.temperature = temperature;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SCALED_PRESSURE2";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			press_abs = buffer.getFloat(); // float
  			press_diff = buffer.getFloat(); // float
  			temperature = buffer.getShort(); // int16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(press_abs); // float
  			buffer.putFloat(press_diff); // float
  			buffer.putShort((short)(temperature)); // int16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SCALED_PRESSURE2 { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"press_abs = " + press_abs + ", " + 
			"press_diff = " + press_diff + ", " + 
			"temperature = " + temperature + ",  }";
		}
	}
	/*
	 * Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
	 */
	public static class MSG_SERIAL_CONTROL extends Message {
	
		private long baudrate; // Baudrate of transfer. Zero means no change.
		private int timeout; // Timeout for reply data in milliseconds
		private int device; // See SERIAL_CONTROL_DEV enum
		private int flags; // See SERIAL_CONTROL_FLAG enum
		private int count; // how many bytes in this transfer
		private int[] data = new int[70]; // serial data
	
		public MSG_SERIAL_CONTROL () {
			super(MSG_ID_SERIAL_CONTROL, (short)10, "SERIAL_CONTROL");
		}

		public MSG_SERIAL_CONTROL (byte[] bytes) {
			super(bytes, MSG_ID_SERIAL_CONTROL, (short)10, "SERIAL_CONTROL");
		}
	
		public MSG_SERIAL_CONTROL (short sys, short comp, long baudrate  , int timeout  , int device  , int flags  , int count  , int data [] ) {
			super(sys, comp, (short)10, "SERIAL_CONTROL");
			this.baudrate = baudrate;
			this.timeout = timeout;
			this.device = device;
			this.flags = flags;
			this.count = count;
			this.data = data;
		}

		@Override
		public int getCRCExtra() {
			return 220;
		}
	
		public long getBaudrate() {
			return baudrate;
		}
		
		public void setBaudrate(long baudrate) {
			this.baudrate = baudrate;
		}
		
		public int getTimeout() {
			return timeout;
		}
		
		public void setTimeout(int timeout) {
			this.timeout = timeout;
		}
		
		public int getDevice() {
			return device;
		}
		
		public void setDevice(int device) {
			this.device = device;
		}
		
		public int getFlags() {
			return flags;
		}
		
		public void setFlags(int flags) {
			this.flags = flags;
		}
		
		public int getCount() {
			return count;
		}
		
		public void setCount(int count) {
			this.count = count;
		}
		
		public int[] getData() {
			return data;
		}
		
		public void setData(int data[]) {
			this.data = data;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SERIAL_CONTROL";
			
			baudrate = buffer.getInt() & 0xffffffff; // uint32_t
  			timeout = buffer.getShort() & 0xffff; // uint16_t
  			device = (int)buffer.get() & 0xff; // uint8_t
  			flags = (int)buffer.get() & 0xff; // uint8_t
  			count = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<70; ++c) {
				data [c] =  (int)buffer.get() & 0xff; // uint8_t[70]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(baudrate & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(timeout & 0xffff)); // uint16_t
  			buffer.put((byte)(device & 0xff)); // uint8_t
  			buffer.put((byte)(flags & 0xff)); // uint8_t
  			buffer.put((byte)(count & 0xff)); // uint8_t
  			for(int c=0; c<70; ++c) {
				buffer.put((byte)(data [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SERIAL_CONTROL { " + 
			"baudrate = " + baudrate + ", " + 
			"timeout = " + timeout + ", " + 
			"device = " + device + ", " + 
			"flags = " + flags + ", " + 
			"count = " + count + ", " + 
			"data = " + data + ",  }";
		}
	}
	/*
	 * The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
	 */
	public static class MSG_SERVO_OUTPUT_RAW extends Message {
	
		private long time_usec; // Timestamp (microseconds since system boot)
		private int servo1_raw; // Servo output 1 value, in microseconds
		private int servo2_raw; // Servo output 2 value, in microseconds
		private int servo3_raw; // Servo output 3 value, in microseconds
		private int servo4_raw; // Servo output 4 value, in microseconds
		private int servo5_raw; // Servo output 5 value, in microseconds
		private int servo6_raw; // Servo output 6 value, in microseconds
		private int servo7_raw; // Servo output 7 value, in microseconds
		private int servo8_raw; // Servo output 8 value, in microseconds
		private int port; // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
	
		public MSG_SERVO_OUTPUT_RAW () {
			super(MSG_ID_SERVO_OUTPUT_RAW, (short)21, "SERVO_OUTPUT_RAW");
		}

		public MSG_SERVO_OUTPUT_RAW (byte[] bytes) {
			super(bytes, MSG_ID_SERVO_OUTPUT_RAW, (short)21, "SERVO_OUTPUT_RAW");
		}
	
		public MSG_SERVO_OUTPUT_RAW (short sys, short comp, long time_usec  , int servo1_raw  , int servo2_raw  , int servo3_raw  , int servo4_raw  , int servo5_raw  , int servo6_raw  , int servo7_raw  , int servo8_raw  , int port  ) {
			super(sys, comp, (short)21, "SERVO_OUTPUT_RAW");
			this.time_usec = time_usec;
			this.servo1_raw = servo1_raw;
			this.servo2_raw = servo2_raw;
			this.servo3_raw = servo3_raw;
			this.servo4_raw = servo4_raw;
			this.servo5_raw = servo5_raw;
			this.servo6_raw = servo6_raw;
			this.servo7_raw = servo7_raw;
			this.servo8_raw = servo8_raw;
			this.port = port;
		}

		@Override
		public int getCRCExtra() {
			return 222;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public int getServo1_raw() {
			return servo1_raw;
		}
		
		public void setServo1_raw(int servo1_raw) {
			this.servo1_raw = servo1_raw;
		}
		
		public int getServo2_raw() {
			return servo2_raw;
		}
		
		public void setServo2_raw(int servo2_raw) {
			this.servo2_raw = servo2_raw;
		}
		
		public int getServo3_raw() {
			return servo3_raw;
		}
		
		public void setServo3_raw(int servo3_raw) {
			this.servo3_raw = servo3_raw;
		}
		
		public int getServo4_raw() {
			return servo4_raw;
		}
		
		public void setServo4_raw(int servo4_raw) {
			this.servo4_raw = servo4_raw;
		}
		
		public int getServo5_raw() {
			return servo5_raw;
		}
		
		public void setServo5_raw(int servo5_raw) {
			this.servo5_raw = servo5_raw;
		}
		
		public int getServo6_raw() {
			return servo6_raw;
		}
		
		public void setServo6_raw(int servo6_raw) {
			this.servo6_raw = servo6_raw;
		}
		
		public int getServo7_raw() {
			return servo7_raw;
		}
		
		public void setServo7_raw(int servo7_raw) {
			this.servo7_raw = servo7_raw;
		}
		
		public int getServo8_raw() {
			return servo8_raw;
		}
		
		public void setServo8_raw(int servo8_raw) {
			this.servo8_raw = servo8_raw;
		}
		
		public int getPort() {
			return port;
		}
		
		public void setPort(int port) {
			this.port = port;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SERVO_OUTPUT_RAW";
			
			time_usec = buffer.getInt() & 0xffffffff; // uint32_t
  			servo1_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo2_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo3_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo4_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo5_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo6_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo7_raw = buffer.getShort() & 0xffff; // uint16_t
  			servo8_raw = buffer.getShort() & 0xffff; // uint16_t
  			port = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_usec & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(servo1_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo2_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo3_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo4_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo5_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo6_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo7_raw & 0xffff)); // uint16_t
  			buffer.putShort((short)(servo8_raw & 0xffff)); // uint16_t
  			buffer.put((byte)(port & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SERVO_OUTPUT_RAW { " + 
			"time_usec = " + time_usec + ", " + 
			"servo1_raw = " + servo1_raw + ", " + 
			"servo2_raw = " + servo2_raw + ", " + 
			"servo3_raw = " + servo3_raw + ", " + 
			"servo4_raw = " + servo4_raw + ", " + 
			"servo5_raw = " + servo5_raw + ", " + 
			"servo6_raw = " + servo6_raw + ", " + 
			"servo7_raw = " + servo7_raw + ", " + 
			"servo8_raw = " + servo8_raw + ", " + 
			"port = " + port + ",  }";
		}
	}
	/*
	 * Set the vehicle attitude and body angular rates.
	 */
	public static class MSG_SET_ACTUATOR_CONTROL_TARGET extends Message {
	
		private long time_usec; // Timestamp (micros since boot or Unix epoch)
		private float[] controls = new float[8]; // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
		private int group_mlx; // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_SET_ACTUATOR_CONTROL_TARGET () {
			super(MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (short)71, "SET_ACTUATOR_CONTROL_TARGET");
		}

		public MSG_SET_ACTUATOR_CONTROL_TARGET (byte[] bytes) {
			super(bytes, MSG_ID_SET_ACTUATOR_CONTROL_TARGET, (short)71, "SET_ACTUATOR_CONTROL_TARGET");
		}
	
		public MSG_SET_ACTUATOR_CONTROL_TARGET (short sys, short comp, long time_usec  , float controls [] , int group_mlx  , int target_system  , int target_component  ) {
			super(sys, comp, (short)71, "SET_ACTUATOR_CONTROL_TARGET");
			this.time_usec = time_usec;
			this.controls = controls;
			this.group_mlx = group_mlx;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 168;
		}
	
		public long getTime_usec() {
			return time_usec;
		}
		
		public void setTime_usec(long time_usec) {
			this.time_usec = time_usec;
		}
		
		public float[] getControls() {
			return controls;
		}
		
		public void setControls(float controls[]) {
			this.controls = controls;
		}
		
		public int getGroup_mlx() {
			return group_mlx;
		}
		
		public void setGroup_mlx(int group_mlx) {
			this.group_mlx = group_mlx;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_ACTUATOR_CONTROL_TARGET";
			
			time_usec = buffer.getLong(); // uint64_t
  			for(int c=0; c<8; ++c) {
				controls [c] = buffer.getFloat(); // float[8]
 			}
			
 			group_mlx = (int)buffer.get() & 0xff; // uint8_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_usec); // uint64_t
  			for(int c=0; c<8; ++c) {
				buffer.putFloat(controls [c]); // float[8]
 			}
			
 			buffer.put((byte)(group_mlx & 0xff)); // uint8_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_ACTUATOR_CONTROL_TARGET { " + 
			"time_usec = " + time_usec + ", " + 
			"controls = " + controls + ", " + 
			"group_mlx = " + group_mlx + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Set the vehicle attitude and body angular rates.
	 */
	public static class MSG_SET_ATTITUDE_TARGET extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot
		private float[] q = new float[4]; // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		private float body_roll_rate; // Body roll rate in radians per second
		private float body_pitch_rate; // Body roll rate in radians per second
		private float body_yaw_rate; // Body roll rate in radians per second
		private float thrust; // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
		private int target_system; // System ID
		private int target_component; // Component ID
		private int type_mask; // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
	
		public MSG_SET_ATTITUDE_TARGET () {
			super(MSG_ID_SET_ATTITUDE_TARGET, (short)27, "SET_ATTITUDE_TARGET");
		}

		public MSG_SET_ATTITUDE_TARGET (byte[] bytes) {
			super(bytes, MSG_ID_SET_ATTITUDE_TARGET, (short)27, "SET_ATTITUDE_TARGET");
		}
	
		public MSG_SET_ATTITUDE_TARGET (short sys, short comp, long time_boot_ms  , float q [] , float body_roll_rate  , float body_pitch_rate  , float body_yaw_rate  , float thrust  , int target_system  , int target_component  , int type_mask  ) {
			super(sys, comp, (short)27, "SET_ATTITUDE_TARGET");
			this.time_boot_ms = time_boot_ms;
			this.q = q;
			this.body_roll_rate = body_roll_rate;
			this.body_pitch_rate = body_pitch_rate;
			this.body_yaw_rate = body_yaw_rate;
			this.thrust = thrust;
			this.target_system = target_system;
			this.target_component = target_component;
			this.type_mask = type_mask;
		}

		@Override
		public int getCRCExtra() {
			return 49;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float[] getQ() {
			return q;
		}
		
		public void setQ(float q[]) {
			this.q = q;
		}
		
		public float getBody_roll_rate() {
			return body_roll_rate;
		}
		
		public void setBody_roll_rate(float body_roll_rate) {
			this.body_roll_rate = body_roll_rate;
		}
		
		public float getBody_pitch_rate() {
			return body_pitch_rate;
		}
		
		public void setBody_pitch_rate(float body_pitch_rate) {
			this.body_pitch_rate = body_pitch_rate;
		}
		
		public float getBody_yaw_rate() {
			return body_yaw_rate;
		}
		
		public void setBody_yaw_rate(float body_yaw_rate) {
			this.body_yaw_rate = body_yaw_rate;
		}
		
		public float getThrust() {
			return thrust;
		}
		
		public void setThrust(float thrust) {
			this.thrust = thrust;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_ATTITUDE_TARGET";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			for(int c=0; c<4; ++c) {
				q [c] = buffer.getFloat(); // float[4]
 			}
			
 			body_roll_rate = buffer.getFloat(); // float
  			body_pitch_rate = buffer.getFloat(); // float
  			body_yaw_rate = buffer.getFloat(); // float
  			thrust = buffer.getFloat(); // float
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			type_mask = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			for(int c=0; c<4; ++c) {
				buffer.putFloat(q [c]); // float[4]
 			}
			
 			buffer.putFloat(body_roll_rate); // float
  			buffer.putFloat(body_pitch_rate); // float
  			buffer.putFloat(body_yaw_rate); // float
  			buffer.putFloat(thrust); // float
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(type_mask & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_ATTITUDE_TARGET { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"q = " + q + ", " + 
			"body_roll_rate = " + body_roll_rate + ", " + 
			"body_pitch_rate = " + body_pitch_rate + ", " + 
			"body_yaw_rate = " + body_yaw_rate + ", " + 
			"thrust = " + thrust + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"type_mask = " + type_mask + ",  }";
		}
	}
	public static class MSG_SET_CAM_SHUTTER extends Message {
	
		private float gain; // Camera gain
		private int interval; // Shutter interval, in microseconds
		private int exposure; // Exposure time, in microseconds
		private int cam_no; // Camera id
		private int cam_mode; // Camera mode: 0 = auto, 1 = manual
		private int trigger_pin; // Trigger pin, 0-3 for PtGrey FireFly
	
		public MSG_SET_CAM_SHUTTER () {
			super(MSG_ID_SET_CAM_SHUTTER, (short)11, "SET_CAM_SHUTTER");
		}

		public MSG_SET_CAM_SHUTTER (byte[] bytes) {
			super(bytes, MSG_ID_SET_CAM_SHUTTER, (short)11, "SET_CAM_SHUTTER");
		}
	
		public MSG_SET_CAM_SHUTTER (short sys, short comp, float gain  , int interval  , int exposure  , int cam_no  , int cam_mode  , int trigger_pin  ) {
			super(sys, comp, (short)11, "SET_CAM_SHUTTER");
			this.gain = gain;
			this.interval = interval;
			this.exposure = exposure;
			this.cam_no = cam_no;
			this.cam_mode = cam_mode;
			this.trigger_pin = trigger_pin;
		}

		@Override
		public int getCRCExtra() {
			return 108;
		}
	
		public float getGain() {
			return gain;
		}
		
		public void setGain(float gain) {
			this.gain = gain;
		}
		
		public int getInterval() {
			return interval;
		}
		
		public void setInterval(int interval) {
			this.interval = interval;
		}
		
		public int getExposure() {
			return exposure;
		}
		
		public void setExposure(int exposure) {
			this.exposure = exposure;
		}
		
		public int getCam_no() {
			return cam_no;
		}
		
		public void setCam_no(int cam_no) {
			this.cam_no = cam_no;
		}
		
		public int getCam_mode() {
			return cam_mode;
		}
		
		public void setCam_mode(int cam_mode) {
			this.cam_mode = cam_mode;
		}
		
		public int getTrigger_pin() {
			return trigger_pin;
		}
		
		public void setTrigger_pin(int trigger_pin) {
			this.trigger_pin = trigger_pin;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_CAM_SHUTTER";
			
			gain = buffer.getFloat(); // float
  			interval = buffer.getShort() & 0xffff; // uint16_t
  			exposure = buffer.getShort() & 0xffff; // uint16_t
  			cam_no = (int)buffer.get() & 0xff; // uint8_t
  			cam_mode = (int)buffer.get() & 0xff; // uint8_t
  			trigger_pin = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(gain); // float
  			buffer.putShort((short)(interval & 0xffff)); // uint16_t
  			buffer.putShort((short)(exposure & 0xffff)); // uint16_t
  			buffer.put((byte)(cam_no & 0xff)); // uint8_t
  			buffer.put((byte)(cam_mode & 0xff)); // uint8_t
  			buffer.put((byte)(trigger_pin & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_CAM_SHUTTER { " + 
			"gain = " + gain + ", " + 
			"interval = " + interval + ", " + 
			"exposure = " + exposure + ", " + 
			"cam_no = " + cam_no + ", " + 
			"cam_mode = " + cam_mode + ", " + 
			"trigger_pin = " + trigger_pin + ",  }";
		}
	}
	/*
	 * As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
	 */
	public static class MSG_SET_GPS_GLOBAL_ORIGIN extends Message {
	
		private int latitude; // Latitude (WGS84), in degrees * 1E7
		private int longitude; // Longitude (WGS84, in degrees * 1E7
		private int altitude; // Altitude (AMSL), in meters * 1000 (positive for up)
		private int target_system; // System ID
	
		public MSG_SET_GPS_GLOBAL_ORIGIN () {
			super(MSG_ID_SET_GPS_GLOBAL_ORIGIN, (short)13, "SET_GPS_GLOBAL_ORIGIN");
		}

		public MSG_SET_GPS_GLOBAL_ORIGIN (byte[] bytes) {
			super(bytes, MSG_ID_SET_GPS_GLOBAL_ORIGIN, (short)13, "SET_GPS_GLOBAL_ORIGIN");
		}
	
		public MSG_SET_GPS_GLOBAL_ORIGIN (short sys, short comp, int latitude  , int longitude  , int altitude  , int target_system  ) {
			super(sys, comp, (short)13, "SET_GPS_GLOBAL_ORIGIN");
			this.latitude = latitude;
			this.longitude = longitude;
			this.altitude = altitude;
			this.target_system = target_system;
		}

		@Override
		public int getCRCExtra() {
			return 41;
		}
	
		public int getLatitude() {
			return latitude;
		}
		
		public void setLatitude(int latitude) {
			this.latitude = latitude;
		}
		
		public int getLongitude() {
			return longitude;
		}
		
		public void setLongitude(int longitude) {
			this.longitude = longitude;
		}
		
		public int getAltitude() {
			return altitude;
		}
		
		public void setAltitude(int altitude) {
			this.altitude = altitude;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_GPS_GLOBAL_ORIGIN";
			
			latitude = buffer.getInt(); // int32_t
  			longitude = buffer.getInt(); // int32_t
  			altitude = buffer.getInt(); // int32_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(latitude)); // int32_t
  			buffer.putInt((int)(longitude)); // int32_t
  			buffer.putInt((int)(altitude)); // int32_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_GPS_GLOBAL_ORIGIN { " + 
			"latitude = " + latitude + ", " + 
			"longitude = " + longitude + ", " + 
			"altitude = " + altitude + ", " + 
			"target_system = " + target_system + ",  }";
		}
	}
	/*
	 * Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
	 */
	public static class MSG_SET_MODE extends Message {
	
		private long custom_mode; // The new autopilot-specific mode. This field can be ignored by an autopilot.
		private int target_system; // The system setting the mode
		private int base_mode; // The new base mode
	
		public MSG_SET_MODE () {
			super(MSG_ID_SET_MODE, (short)6, "SET_MODE");
		}

		public MSG_SET_MODE (byte[] bytes) {
			super(bytes, MSG_ID_SET_MODE, (short)6, "SET_MODE");
		}
	
		public MSG_SET_MODE (short sys, short comp, long custom_mode  , int target_system  , int base_mode  ) {
			super(sys, comp, (short)6, "SET_MODE");
			this.custom_mode = custom_mode;
			this.target_system = target_system;
			this.base_mode = base_mode;
		}

		@Override
		public int getCRCExtra() {
			return 89;
		}
	
		public long getCustom_mode() {
			return custom_mode;
		}
		
		public void setCustom_mode(long custom_mode) {
			this.custom_mode = custom_mode;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getBase_mode() {
			return base_mode;
		}
		
		public void setBase_mode(int base_mode) {
			this.base_mode = base_mode;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_MODE";
			
			custom_mode = buffer.getInt() & 0xffffffff; // uint32_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			base_mode = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(custom_mode & 0xffffffff)); // uint32_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(base_mode & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_MODE { " + 
			"custom_mode = " + custom_mode + ", " + 
			"target_system = " + target_system + ", " + 
			"base_mode = " + base_mode + ",  }";
		}
	}
	/*
	 * Message sent to the MAV to set a new offset from the currently controlled position
	 */
	public static class MSG_SET_POSITION_CONTROL_OFFSET extends Message {
	
		private float x; // x position offset
		private float y; // y position offset
		private float z; // z position offset
		private float yaw; // yaw orientation offset in radians, 0 = NORTH
		private int target_system; // System ID
		private int target_component; // Component ID
	
		public MSG_SET_POSITION_CONTROL_OFFSET () {
			super(MSG_ID_SET_POSITION_CONTROL_OFFSET, (short)18, "SET_POSITION_CONTROL_OFFSET");
		}

		public MSG_SET_POSITION_CONTROL_OFFSET (byte[] bytes) {
			super(bytes, MSG_ID_SET_POSITION_CONTROL_OFFSET, (short)18, "SET_POSITION_CONTROL_OFFSET");
		}
	
		public MSG_SET_POSITION_CONTROL_OFFSET (short sys, short comp, float x  , float y  , float z  , float yaw  , int target_system  , int target_component  ) {
			super(sys, comp, (short)18, "SET_POSITION_CONTROL_OFFSET");
			this.x = x;
			this.y = y;
			this.z = z;
			this.yaw = yaw;
			this.target_system = target_system;
			this.target_component = target_component;
		}

		@Override
		public int getCRCExtra() {
			return 22;
		}
	
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_POSITION_CONTROL_OFFSET";
			
			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(yaw); // float
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_POSITION_CONTROL_OFFSET { " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"yaw = " + yaw + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ",  }";
		}
	}
	/*
	 * Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
	 */
	public static class MSG_SET_POSITION_TARGET_GLOBAL_INT extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
		private int lat_int; // X Position in WGS84 frame in 1e7 * meters
		private int lon_int; // Y Position in WGS84 frame in 1e7 * meters
		private float alt; // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
		private float vx; // X velocity in NED frame in meter / s
		private float vy; // Y velocity in NED frame in meter / s
		private float vz; // Z velocity in NED frame in meter / s
		private float afx; // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afy; // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afz; // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float yaw; // yaw setpoint in rad
		private float yaw_rate; // yaw rate setpoint in rad/s
		private int type_mask; // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
		private int target_system; // System ID
		private int target_component; // Component ID
		private int coordinate_frame; // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
	
		public MSG_SET_POSITION_TARGET_GLOBAL_INT () {
			super(MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, (short)53, "SET_POSITION_TARGET_GLOBAL_INT");
		}

		public MSG_SET_POSITION_TARGET_GLOBAL_INT (byte[] bytes) {
			super(bytes, MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, (short)53, "SET_POSITION_TARGET_GLOBAL_INT");
		}
	
		public MSG_SET_POSITION_TARGET_GLOBAL_INT (short sys, short comp, long time_boot_ms  , int lat_int  , int lon_int  , float alt  , float vx  , float vy  , float vz  , float afx  , float afy  , float afz  , float yaw  , float yaw_rate  , int type_mask  , int target_system  , int target_component  , int coordinate_frame  ) {
			super(sys, comp, (short)53, "SET_POSITION_TARGET_GLOBAL_INT");
			this.time_boot_ms = time_boot_ms;
			this.lat_int = lat_int;
			this.lon_int = lon_int;
			this.alt = alt;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.afx = afx;
			this.afy = afy;
			this.afz = afz;
			this.yaw = yaw;
			this.yaw_rate = yaw_rate;
			this.type_mask = type_mask;
			this.target_system = target_system;
			this.target_component = target_component;
			this.coordinate_frame = coordinate_frame;
		}

		@Override
		public int getCRCExtra() {
			return 5;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public int getLat_int() {
			return lat_int;
		}
		
		public void setLat_int(int lat_int) {
			this.lat_int = lat_int;
		}
		
		public int getLon_int() {
			return lon_int;
		}
		
		public void setLon_int(int lon_int) {
			this.lon_int = lon_int;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float getAfx() {
			return afx;
		}
		
		public void setAfx(float afx) {
			this.afx = afx;
		}
		
		public float getAfy() {
			return afy;
		}
		
		public void setAfy(float afy) {
			this.afy = afy;
		}
		
		public float getAfz() {
			return afz;
		}
		
		public void setAfz(float afz) {
			this.afz = afz;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getYaw_rate() {
			return yaw_rate;
		}
		
		public void setYaw_rate(float yaw_rate) {
			this.yaw_rate = yaw_rate;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getCoordinate_frame() {
			return coordinate_frame;
		}
		
		public void setCoordinate_frame(int coordinate_frame) {
			this.coordinate_frame = coordinate_frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_POSITION_TARGET_GLOBAL_INT";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			lat_int = buffer.getInt(); // int32_t
  			lon_int = buffer.getInt(); // int32_t
  			alt = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			afx = buffer.getFloat(); // float
  			afy = buffer.getFloat(); // float
  			afz = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			yaw_rate = buffer.getFloat(); // float
  			type_mask = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			coordinate_frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(lat_int)); // int32_t
  			buffer.putInt((int)(lon_int)); // int32_t
  			buffer.putFloat(alt); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			buffer.putFloat(afx); // float
  			buffer.putFloat(afy); // float
  			buffer.putFloat(afz); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(yaw_rate); // float
  			buffer.putShort((short)(type_mask & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(coordinate_frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_POSITION_TARGET_GLOBAL_INT { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"lat_int = " + lat_int + ", " + 
			"lon_int = " + lon_int + ", " + 
			"alt = " + alt + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"afx = " + afx + ", " + 
			"afy = " + afy + ", " + 
			"afz = " + afz + ", " + 
			"yaw = " + yaw + ", " + 
			"yaw_rate = " + yaw_rate + ", " + 
			"type_mask = " + type_mask + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"coordinate_frame = " + coordinate_frame + ",  }";
		}
	}
	/*
	 * Set vehicle position, velocity and acceleration setpoint in local frame.
	 */
	public static class MSG_SET_POSITION_TARGET_LOCAL_NED extends Message {
	
		private long time_boot_ms; // Timestamp in milliseconds since system boot
		private float x; // X Position in NED frame in meters
		private float y; // Y Position in NED frame in meters
		private float z; // Z Position in NED frame in meters (note, altitude is negative in NED)
		private float vx; // X velocity in NED frame in meter / s
		private float vy; // Y velocity in NED frame in meter / s
		private float vz; // Z velocity in NED frame in meter / s
		private float afx; // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afy; // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float afz; // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
		private float yaw; // yaw setpoint in rad
		private float yaw_rate; // yaw rate setpoint in rad/s
		private int type_mask; // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
		private int target_system; // System ID
		private int target_component; // Component ID
		private int coordinate_frame; // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
	
		public MSG_SET_POSITION_TARGET_LOCAL_NED () {
			super(MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (short)53, "SET_POSITION_TARGET_LOCAL_NED");
		}

		public MSG_SET_POSITION_TARGET_LOCAL_NED (byte[] bytes) {
			super(bytes, MSG_ID_SET_POSITION_TARGET_LOCAL_NED, (short)53, "SET_POSITION_TARGET_LOCAL_NED");
		}
	
		public MSG_SET_POSITION_TARGET_LOCAL_NED (short sys, short comp, long time_boot_ms  , float x  , float y  , float z  , float vx  , float vy  , float vz  , float afx  , float afy  , float afz  , float yaw  , float yaw_rate  , int type_mask  , int target_system  , int target_component  , int coordinate_frame  ) {
			super(sys, comp, (short)53, "SET_POSITION_TARGET_LOCAL_NED");
			this.time_boot_ms = time_boot_ms;
			this.x = x;
			this.y = y;
			this.z = z;
			this.vx = vx;
			this.vy = vy;
			this.vz = vz;
			this.afx = afx;
			this.afy = afy;
			this.afz = afz;
			this.yaw = yaw;
			this.yaw_rate = yaw_rate;
			this.type_mask = type_mask;
			this.target_system = target_system;
			this.target_component = target_component;
			this.coordinate_frame = coordinate_frame;
		}

		@Override
		public int getCRCExtra() {
			return 143;
		}
	
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getVx() {
			return vx;
		}
		
		public void setVx(float vx) {
			this.vx = vx;
		}
		
		public float getVy() {
			return vy;
		}
		
		public void setVy(float vy) {
			this.vy = vy;
		}
		
		public float getVz() {
			return vz;
		}
		
		public void setVz(float vz) {
			this.vz = vz;
		}
		
		public float getAfx() {
			return afx;
		}
		
		public void setAfx(float afx) {
			this.afx = afx;
		}
		
		public float getAfy() {
			return afy;
		}
		
		public void setAfy(float afy) {
			this.afy = afy;
		}
		
		public float getAfz() {
			return afz;
		}
		
		public void setAfz(float afz) {
			this.afz = afz;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getYaw_rate() {
			return yaw_rate;
		}
		
		public void setYaw_rate(float yaw_rate) {
			this.yaw_rate = yaw_rate;
		}
		
		public int getType_mask() {
			return type_mask;
		}
		
		public void setType_mask(int type_mask) {
			this.type_mask = type_mask;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int getCoordinate_frame() {
			return coordinate_frame;
		}
		
		public void setCoordinate_frame(int coordinate_frame) {
			this.coordinate_frame = coordinate_frame;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SET_POSITION_TARGET_LOCAL_NED";
			
			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			vx = buffer.getFloat(); // float
  			vy = buffer.getFloat(); // float
  			vz = buffer.getFloat(); // float
  			afx = buffer.getFloat(); // float
  			afy = buffer.getFloat(); // float
  			afz = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			yaw_rate = buffer.getFloat(); // float
  			type_mask = buffer.getShort() & 0xffff; // uint16_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			coordinate_frame = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(vx); // float
  			buffer.putFloat(vy); // float
  			buffer.putFloat(vz); // float
  			buffer.putFloat(afx); // float
  			buffer.putFloat(afy); // float
  			buffer.putFloat(afz); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(yaw_rate); // float
  			buffer.putShort((short)(type_mask & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			buffer.put((byte)(coordinate_frame & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SET_POSITION_TARGET_LOCAL_NED { " + 
			"time_boot_ms = " + time_boot_ms + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"vx = " + vx + ", " + 
			"vy = " + vy + ", " + 
			"vz = " + vz + ", " + 
			"afx = " + afx + ", " + 
			"afy = " + afy + ", " + 
			"afz = " + afz + ", " + 
			"yaw = " + yaw + ", " + 
			"yaw_rate = " + yaw_rate + ", " + 
			"type_mask = " + type_mask + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"coordinate_frame = " + coordinate_frame + ",  }";
		}
	}
	/*
	 * Status of simulation environment, if used
	 */
	public static class MSG_SIM_STATE extends Message {
	
		private float q1; // True attitude quaternion component 1, w (1 in null-rotation)
		private float q2; // True attitude quaternion component 2, x (0 in null-rotation)
		private float q3; // True attitude quaternion component 3, y (0 in null-rotation)
		private float q4; // True attitude quaternion component 4, z (0 in null-rotation)
		private float roll; // Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
		private float pitch; // Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
		private float yaw; // Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
		private float xacc; // X acceleration m/s/s
		private float yacc; // Y acceleration m/s/s
		private float zacc; // Z acceleration m/s/s
		private float xgyro; // Angular speed around X axis rad/s
		private float ygyro; // Angular speed around Y axis rad/s
		private float zgyro; // Angular speed around Z axis rad/s
		private float lat; // Latitude in degrees
		private float lon; // Longitude in degrees
		private float alt; // Altitude in meters
		private float std_dev_horz; // Horizontal position standard deviation
		private float std_dev_vert; // Vertical position standard deviation
		private float vn; // True velocity in m/s in NORTH direction in earth-fixed NED frame
		private float ve; // True velocity in m/s in EAST direction in earth-fixed NED frame
		private float vd; // True velocity in m/s in DOWN direction in earth-fixed NED frame
	
		public MSG_SIM_STATE () {
			super(MSG_ID_SIM_STATE, (short)84, "SIM_STATE");
		}

		public MSG_SIM_STATE (byte[] bytes) {
			super(bytes, MSG_ID_SIM_STATE, (short)84, "SIM_STATE");
		}
	
		public MSG_SIM_STATE (short sys, short comp, float q1  , float q2  , float q3  , float q4  , float roll  , float pitch  , float yaw  , float xacc  , float yacc  , float zacc  , float xgyro  , float ygyro  , float zgyro  , float lat  , float lon  , float alt  , float std_dev_horz  , float std_dev_vert  , float vn  , float ve  , float vd  ) {
			super(sys, comp, (short)84, "SIM_STATE");
			this.q1 = q1;
			this.q2 = q2;
			this.q3 = q3;
			this.q4 = q4;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
			this.xacc = xacc;
			this.yacc = yacc;
			this.zacc = zacc;
			this.xgyro = xgyro;
			this.ygyro = ygyro;
			this.zgyro = zgyro;
			this.lat = lat;
			this.lon = lon;
			this.alt = alt;
			this.std_dev_horz = std_dev_horz;
			this.std_dev_vert = std_dev_vert;
			this.vn = vn;
			this.ve = ve;
			this.vd = vd;
		}

		@Override
		public int getCRCExtra() {
			return 32;
		}
	
		public float getQ1() {
			return q1;
		}
		
		public void setQ1(float q1) {
			this.q1 = q1;
		}
		
		public float getQ2() {
			return q2;
		}
		
		public void setQ2(float q2) {
			this.q2 = q2;
		}
		
		public float getQ3() {
			return q3;
		}
		
		public void setQ3(float q3) {
			this.q3 = q3;
		}
		
		public float getQ4() {
			return q4;
		}
		
		public void setQ4(float q4) {
			this.q4 = q4;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
		public float getXacc() {
			return xacc;
		}
		
		public void setXacc(float xacc) {
			this.xacc = xacc;
		}
		
		public float getYacc() {
			return yacc;
		}
		
		public void setYacc(float yacc) {
			this.yacc = yacc;
		}
		
		public float getZacc() {
			return zacc;
		}
		
		public void setZacc(float zacc) {
			this.zacc = zacc;
		}
		
		public float getXgyro() {
			return xgyro;
		}
		
		public void setXgyro(float xgyro) {
			this.xgyro = xgyro;
		}
		
		public float getYgyro() {
			return ygyro;
		}
		
		public void setYgyro(float ygyro) {
			this.ygyro = ygyro;
		}
		
		public float getZgyro() {
			return zgyro;
		}
		
		public void setZgyro(float zgyro) {
			this.zgyro = zgyro;
		}
		
		public float getLat() {
			return lat;
		}
		
		public void setLat(float lat) {
			this.lat = lat;
		}
		
		public float getLon() {
			return lon;
		}
		
		public void setLon(float lon) {
			this.lon = lon;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getStd_dev_horz() {
			return std_dev_horz;
		}
		
		public void setStd_dev_horz(float std_dev_horz) {
			this.std_dev_horz = std_dev_horz;
		}
		
		public float getStd_dev_vert() {
			return std_dev_vert;
		}
		
		public void setStd_dev_vert(float std_dev_vert) {
			this.std_dev_vert = std_dev_vert;
		}
		
		public float getVn() {
			return vn;
		}
		
		public void setVn(float vn) {
			this.vn = vn;
		}
		
		public float getVe() {
			return ve;
		}
		
		public void setVe(float ve) {
			this.ve = ve;
		}
		
		public float getVd() {
			return vd;
		}
		
		public void setVd(float vd) {
			this.vd = vd;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SIM_STATE";
			
			q1 = buffer.getFloat(); // float
  			q2 = buffer.getFloat(); // float
  			q3 = buffer.getFloat(); // float
  			q4 = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
  			xacc = buffer.getFloat(); // float
  			yacc = buffer.getFloat(); // float
  			zacc = buffer.getFloat(); // float
  			xgyro = buffer.getFloat(); // float
  			ygyro = buffer.getFloat(); // float
  			zgyro = buffer.getFloat(); // float
  			lat = buffer.getFloat(); // float
  			lon = buffer.getFloat(); // float
  			alt = buffer.getFloat(); // float
  			std_dev_horz = buffer.getFloat(); // float
  			std_dev_vert = buffer.getFloat(); // float
  			vn = buffer.getFloat(); // float
  			ve = buffer.getFloat(); // float
  			vd = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(q1); // float
  			buffer.putFloat(q2); // float
  			buffer.putFloat(q3); // float
  			buffer.putFloat(q4); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
  			buffer.putFloat(xacc); // float
  			buffer.putFloat(yacc); // float
  			buffer.putFloat(zacc); // float
  			buffer.putFloat(xgyro); // float
  			buffer.putFloat(ygyro); // float
  			buffer.putFloat(zgyro); // float
  			buffer.putFloat(lat); // float
  			buffer.putFloat(lon); // float
  			buffer.putFloat(alt); // float
  			buffer.putFloat(std_dev_horz); // float
  			buffer.putFloat(std_dev_vert); // float
  			buffer.putFloat(vn); // float
  			buffer.putFloat(ve); // float
  			buffer.putFloat(vd); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SIM_STATE { " + 
			"q1 = " + q1 + ", " + 
			"q2 = " + q2 + ", " + 
			"q3 = " + q3 + ", " + 
			"q4 = " + q4 + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ", " + 
			"xacc = " + xacc + ", " + 
			"yacc = " + yacc + ", " + 
			"zacc = " + zacc + ", " + 
			"xgyro = " + xgyro + ", " + 
			"ygyro = " + ygyro + ", " + 
			"zgyro = " + zgyro + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"alt = " + alt + ", " + 
			"std_dev_horz = " + std_dev_horz + ", " + 
			"std_dev_vert = " + std_dev_vert + ", " + 
			"vn = " + vn + ", " + 
			"ve = " + ve + ", " + 
			"vd = " + vd + ",  }";
		}
	}
	/*
	 * Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
	 */
	public static class MSG_STATUSTEXT extends Message {
	
		private int severity; // Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
		private char[] text = new char[50]; // Status text message, without null termination character
	
		public MSG_STATUSTEXT () {
			super(MSG_ID_STATUSTEXT, (short)2, "STATUSTEXT");
		}

		public MSG_STATUSTEXT (byte[] bytes) {
			super(bytes, MSG_ID_STATUSTEXT, (short)2, "STATUSTEXT");
		}
	
		public MSG_STATUSTEXT (short sys, short comp, int severity  , char text [] ) {
			super(sys, comp, (short)2, "STATUSTEXT");
			this.severity = severity;
			this.text = text;
		}

		@Override
		public int getCRCExtra() {
			return 83;
		}
	
		public int getSeverity() {
			return severity;
		}
		
		public void setSeverity(int severity) {
			this.severity = severity;
		}
		
		public char[] getText() {
			return text;
		}
		
		public void setText(char text[]) {
			this.text = text;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "STATUSTEXT";
			
			severity = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<50; ++c) {
				text [c] =  (char)buffer.get(); // char[50]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.put((byte)(severity & 0xff)); // uint8_t
  			for(int c=0; c<50; ++c) {
				buffer.put((byte)(text [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_STATUSTEXT { " + 
			"severity = " + severity + ", " + 
			"text = " + text + ",  }";
		}
	}
	/*
	 * The system time is the time of the master clock, typically the computer clock of the main onboard computer.
	 */
	public static class MSG_SYSTEM_TIME extends Message {
	
		private long time_unix_usec; // Timestamp of the master clock in microseconds since UNIX epoch.
		private long time_boot_ms; // Timestamp of the component clock since boot time in milliseconds.
	
		public MSG_SYSTEM_TIME () {
			super(MSG_ID_SYSTEM_TIME, (short)68, "SYSTEM_TIME");
		}

		public MSG_SYSTEM_TIME (byte[] bytes) {
			super(bytes, MSG_ID_SYSTEM_TIME, (short)68, "SYSTEM_TIME");
		}
	
		public MSG_SYSTEM_TIME (short sys, short comp, long time_unix_usec  , long time_boot_ms  ) {
			super(sys, comp, (short)68, "SYSTEM_TIME");
			this.time_unix_usec = time_unix_usec;
			this.time_boot_ms = time_boot_ms;
		}

		@Override
		public int getCRCExtra() {
			return 137;
		}
	
		public long getTime_unix_usec() {
			return time_unix_usec;
		}
		
		public void setTime_unix_usec(long time_unix_usec) {
			this.time_unix_usec = time_unix_usec;
		}
		
		public long getTime_boot_ms() {
			return time_boot_ms;
		}
		
		public void setTime_boot_ms(long time_boot_ms) {
			this.time_boot_ms = time_boot_ms;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SYSTEM_TIME";
			
			time_unix_usec = buffer.getLong(); // uint64_t
  			time_boot_ms = buffer.getInt() & 0xffffffff; // uint32_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(time_unix_usec); // uint64_t
  			buffer.putInt((int)(time_boot_ms & 0xffffffff)); // uint32_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SYSTEM_TIME { " + 
			"time_unix_usec = " + time_unix_usec + ", " + 
			"time_boot_ms = " + time_boot_ms + ",  }";
		}
	}
	/*
	 * The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
	 */
	public static class MSG_SYS_STATUS extends Message {
	
		private long onboard_control_sensors_present; // Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
		private long onboard_control_sensors_enabled; // Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
		private long onboard_control_sensors_health; // Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
		private int load; // Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
		private int voltage_battery; // Battery voltage, in millivolts (1 = 1 millivolt)
		private int current_battery; // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
		private int drop_rate_comm; // Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
		private int errors_comm; // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
		private int errors_count1; // Autopilot-specific errors
		private int errors_count2; // Autopilot-specific errors
		private int errors_count3; // Autopilot-specific errors
		private int errors_count4; // Autopilot-specific errors
		private int battery_remaining; // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
	
		public MSG_SYS_STATUS () {
			super(MSG_ID_SYS_STATUS, (short)31, "SYS_STATUS");
		}

		public MSG_SYS_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_SYS_STATUS, (short)31, "SYS_STATUS");
		}
	
		public MSG_SYS_STATUS (short sys, short comp, long onboard_control_sensors_present  , long onboard_control_sensors_enabled  , long onboard_control_sensors_health  , int load  , int voltage_battery  , int current_battery  , int drop_rate_comm  , int errors_comm  , int errors_count1  , int errors_count2  , int errors_count3  , int errors_count4  , int battery_remaining  ) {
			super(sys, comp, (short)31, "SYS_STATUS");
			this.onboard_control_sensors_present = onboard_control_sensors_present;
			this.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
			this.onboard_control_sensors_health = onboard_control_sensors_health;
			this.load = load;
			this.voltage_battery = voltage_battery;
			this.current_battery = current_battery;
			this.drop_rate_comm = drop_rate_comm;
			this.errors_comm = errors_comm;
			this.errors_count1 = errors_count1;
			this.errors_count2 = errors_count2;
			this.errors_count3 = errors_count3;
			this.errors_count4 = errors_count4;
			this.battery_remaining = battery_remaining;
		}

		@Override
		public int getCRCExtra() {
			return 124;
		}
	
		public long getOnboard_control_sensors_present() {
			return onboard_control_sensors_present;
		}
		
		public void setOnboard_control_sensors_present(long onboard_control_sensors_present) {
			this.onboard_control_sensors_present = onboard_control_sensors_present;
		}
		
		public long getOnboard_control_sensors_enabled() {
			return onboard_control_sensors_enabled;
		}
		
		public void setOnboard_control_sensors_enabled(long onboard_control_sensors_enabled) {
			this.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
		}
		
		public long getOnboard_control_sensors_health() {
			return onboard_control_sensors_health;
		}
		
		public void setOnboard_control_sensors_health(long onboard_control_sensors_health) {
			this.onboard_control_sensors_health = onboard_control_sensors_health;
		}
		
		public int getLoad() {
			return load;
		}
		
		public void setLoad(int load) {
			this.load = load;
		}
		
		public int getVoltage_battery() {
			return voltage_battery;
		}
		
		public void setVoltage_battery(int voltage_battery) {
			this.voltage_battery = voltage_battery;
		}
		
		public int getCurrent_battery() {
			return current_battery;
		}
		
		public void setCurrent_battery(int current_battery) {
			this.current_battery = current_battery;
		}
		
		public int getDrop_rate_comm() {
			return drop_rate_comm;
		}
		
		public void setDrop_rate_comm(int drop_rate_comm) {
			this.drop_rate_comm = drop_rate_comm;
		}
		
		public int getErrors_comm() {
			return errors_comm;
		}
		
		public void setErrors_comm(int errors_comm) {
			this.errors_comm = errors_comm;
		}
		
		public int getErrors_count1() {
			return errors_count1;
		}
		
		public void setErrors_count1(int errors_count1) {
			this.errors_count1 = errors_count1;
		}
		
		public int getErrors_count2() {
			return errors_count2;
		}
		
		public void setErrors_count2(int errors_count2) {
			this.errors_count2 = errors_count2;
		}
		
		public int getErrors_count3() {
			return errors_count3;
		}
		
		public void setErrors_count3(int errors_count3) {
			this.errors_count3 = errors_count3;
		}
		
		public int getErrors_count4() {
			return errors_count4;
		}
		
		public void setErrors_count4(int errors_count4) {
			this.errors_count4 = errors_count4;
		}
		
		public int getBattery_remaining() {
			return battery_remaining;
		}
		
		public void setBattery_remaining(int battery_remaining) {
			this.battery_remaining = battery_remaining;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "SYS_STATUS";
			
			onboard_control_sensors_present = buffer.getInt() & 0xffffffff; // uint32_t
  			onboard_control_sensors_enabled = buffer.getInt() & 0xffffffff; // uint32_t
  			onboard_control_sensors_health = buffer.getInt() & 0xffffffff; // uint32_t
  			load = buffer.getShort() & 0xffff; // uint16_t
  			voltage_battery = buffer.getShort() & 0xffff; // uint16_t
  			current_battery = buffer.getShort(); // int16_t
  			drop_rate_comm = buffer.getShort() & 0xffff; // uint16_t
  			errors_comm = buffer.getShort() & 0xffff; // uint16_t
  			errors_count1 = buffer.getShort() & 0xffff; // uint16_t
  			errors_count2 = buffer.getShort() & 0xffff; // uint16_t
  			errors_count3 = buffer.getShort() & 0xffff; // uint16_t
  			errors_count4 = buffer.getShort() & 0xffff; // uint16_t
  			battery_remaining = (int)buffer.get(); // int8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(onboard_control_sensors_present & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(onboard_control_sensors_enabled & 0xffffffff)); // uint32_t
  			buffer.putInt((int)(onboard_control_sensors_health & 0xffffffff)); // uint32_t
  			buffer.putShort((short)(load & 0xffff)); // uint16_t
  			buffer.putShort((short)(voltage_battery & 0xffff)); // uint16_t
  			buffer.putShort((short)(current_battery)); // int16_t
  			buffer.putShort((short)(drop_rate_comm & 0xffff)); // uint16_t
  			buffer.putShort((short)(errors_comm & 0xffff)); // uint16_t
  			buffer.putShort((short)(errors_count1 & 0xffff)); // uint16_t
  			buffer.putShort((short)(errors_count2 & 0xffff)); // uint16_t
  			buffer.putShort((short)(errors_count3 & 0xffff)); // uint16_t
  			buffer.putShort((short)(errors_count4 & 0xffff)); // uint16_t
  			buffer.put((byte)(battery_remaining)); // int8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_SYS_STATUS { " + 
			"onboard_control_sensors_present = " + onboard_control_sensors_present + ", " + 
			"onboard_control_sensors_enabled = " + onboard_control_sensors_enabled + ", " + 
			"onboard_control_sensors_health = " + onboard_control_sensors_health + ", " + 
			"load = " + load + ", " + 
			"voltage_battery = " + voltage_battery + ", " + 
			"current_battery = " + current_battery + ", " + 
			"drop_rate_comm = " + drop_rate_comm + ", " + 
			"errors_comm = " + errors_comm + ", " + 
			"errors_count1 = " + errors_count1 + ", " + 
			"errors_count2 = " + errors_count2 + ", " + 
			"errors_count3 = " + errors_count3 + ", " + 
			"errors_count4 = " + errors_count4 + ", " + 
			"battery_remaining = " + battery_remaining + ",  }";
		}
	}
	/*
	 * Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
	 */
	public static class MSG_TERRAIN_CHECK extends Message {
	
		private int lat; // Latitude (degrees *10^7)
		private int lon; // Longitude (degrees *10^7)
	
		public MSG_TERRAIN_CHECK () {
			super(MSG_ID_TERRAIN_CHECK, (short)8, "TERRAIN_CHECK");
		}

		public MSG_TERRAIN_CHECK (byte[] bytes) {
			super(bytes, MSG_ID_TERRAIN_CHECK, (short)8, "TERRAIN_CHECK");
		}
	
		public MSG_TERRAIN_CHECK (short sys, short comp, int lat  , int lon  ) {
			super(sys, comp, (short)8, "TERRAIN_CHECK");
			this.lat = lat;
			this.lon = lon;
		}

		@Override
		public int getCRCExtra() {
			return 203;
		}
	
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "TERRAIN_CHECK";
			
			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_TERRAIN_CHECK { " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ",  }";
		}
	}
	/*
	 * Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
	 */
	public static class MSG_TERRAIN_DATA extends Message {
	
		private int lat; // Latitude of SW corner of first grid (degrees *10^7)
		private int lon; // Longitude of SW corner of first grid (in degrees *10^7)
		private int grid_spacing; // Grid spacing in meters
		private int[] data = new int[16]; // Terrain data in meters AMSL
		private int gridbit; // bit within the terrain request mask
	
		public MSG_TERRAIN_DATA () {
			super(MSG_ID_TERRAIN_DATA, (short)13, "TERRAIN_DATA");
		}

		public MSG_TERRAIN_DATA (byte[] bytes) {
			super(bytes, MSG_ID_TERRAIN_DATA, (short)13, "TERRAIN_DATA");
		}
	
		public MSG_TERRAIN_DATA (short sys, short comp, int lat  , int lon  , int grid_spacing  , int data [] , int gridbit  ) {
			super(sys, comp, (short)13, "TERRAIN_DATA");
			this.lat = lat;
			this.lon = lon;
			this.grid_spacing = grid_spacing;
			this.data = data;
			this.gridbit = gridbit;
		}

		@Override
		public int getCRCExtra() {
			return 229;
		}
	
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getGrid_spacing() {
			return grid_spacing;
		}
		
		public void setGrid_spacing(int grid_spacing) {
			this.grid_spacing = grid_spacing;
		}
		
		public int[] getData() {
			return data;
		}
		
		public void setData(int data[]) {
			this.data = data;
		}
		
		public int getGridbit() {
			return gridbit;
		}
		
		public void setGridbit(int gridbit) {
			this.gridbit = gridbit;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "TERRAIN_DATA";
			
			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			grid_spacing = buffer.getShort() & 0xffff; // uint16_t
  			for(int c=0; c<16; ++c) {
				data [c] = buffer.getShort(); // int16_t[16]
 			}
			
 			gridbit = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putShort((short)(grid_spacing & 0xffff)); // uint16_t
  			for(int c=0; c<16; ++c) {
				buffer.putShort((short)(data [c]));
 			}
			
 			buffer.put((byte)(gridbit & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_TERRAIN_DATA { " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"grid_spacing = " + grid_spacing + ", " + 
			"data = " + data + ", " + 
			"gridbit = " + gridbit + ",  }";
		}
	}
	/*
	 * Response from a TERRAIN_CHECK request
	 */
	public static class MSG_TERRAIN_REPORT extends Message {
	
		private int lat; // Latitude (degrees *10^7)
		private int lon; // Longitude (degrees *10^7)
		private float terrain_height; // Terrain height in meters AMSL
		private float current_height; // Current vehicle height above lat/lon terrain height (meters)
		private int spacing; // grid spacing (zero if terrain at this location unavailable)
		private int pending; // Number of 4x4 terrain blocks waiting to be received or read from disk
		private int loaded; // Number of 4x4 terrain blocks in memory
	
		public MSG_TERRAIN_REPORT () {
			super(MSG_ID_TERRAIN_REPORT, (short)22, "TERRAIN_REPORT");
		}

		public MSG_TERRAIN_REPORT (byte[] bytes) {
			super(bytes, MSG_ID_TERRAIN_REPORT, (short)22, "TERRAIN_REPORT");
		}
	
		public MSG_TERRAIN_REPORT (short sys, short comp, int lat  , int lon  , float terrain_height  , float current_height  , int spacing  , int pending  , int loaded  ) {
			super(sys, comp, (short)22, "TERRAIN_REPORT");
			this.lat = lat;
			this.lon = lon;
			this.terrain_height = terrain_height;
			this.current_height = current_height;
			this.spacing = spacing;
			this.pending = pending;
			this.loaded = loaded;
		}

		@Override
		public int getCRCExtra() {
			return 1;
		}
	
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public float getTerrain_height() {
			return terrain_height;
		}
		
		public void setTerrain_height(float terrain_height) {
			this.terrain_height = terrain_height;
		}
		
		public float getCurrent_height() {
			return current_height;
		}
		
		public void setCurrent_height(float current_height) {
			this.current_height = current_height;
		}
		
		public int getSpacing() {
			return spacing;
		}
		
		public void setSpacing(int spacing) {
			this.spacing = spacing;
		}
		
		public int getPending() {
			return pending;
		}
		
		public void setPending(int pending) {
			this.pending = pending;
		}
		
		public int getLoaded() {
			return loaded;
		}
		
		public void setLoaded(int loaded) {
			this.loaded = loaded;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "TERRAIN_REPORT";
			
			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			terrain_height = buffer.getFloat(); // float
  			current_height = buffer.getFloat(); // float
  			spacing = buffer.getShort() & 0xffff; // uint16_t
  			pending = buffer.getShort() & 0xffff; // uint16_t
  			loaded = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putFloat(terrain_height); // float
  			buffer.putFloat(current_height); // float
  			buffer.putShort((short)(spacing & 0xffff)); // uint16_t
  			buffer.putShort((short)(pending & 0xffff)); // uint16_t
  			buffer.putShort((short)(loaded & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_TERRAIN_REPORT { " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"terrain_height = " + terrain_height + ", " + 
			"current_height = " + current_height + ", " + 
			"spacing = " + spacing + ", " + 
			"pending = " + pending + ", " + 
			"loaded = " + loaded + ",  }";
		}
	}
	/*
	 * Request for terrain data and terrain status
	 */
	public static class MSG_TERRAIN_REQUEST extends Message {
	
		private long mask; // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
		private int lat; // Latitude of SW corner of first grid (degrees *10^7)
		private int lon; // Longitude of SW corner of first grid (in degrees *10^7)
		private int grid_spacing; // Grid spacing in meters
	
		public MSG_TERRAIN_REQUEST () {
			super(MSG_ID_TERRAIN_REQUEST, (short)74, "TERRAIN_REQUEST");
		}

		public MSG_TERRAIN_REQUEST (byte[] bytes) {
			super(bytes, MSG_ID_TERRAIN_REQUEST, (short)74, "TERRAIN_REQUEST");
		}
	
		public MSG_TERRAIN_REQUEST (short sys, short comp, long mask  , int lat  , int lon  , int grid_spacing  ) {
			super(sys, comp, (short)74, "TERRAIN_REQUEST");
			this.mask = mask;
			this.lat = lat;
			this.lon = lon;
			this.grid_spacing = grid_spacing;
		}

		@Override
		public int getCRCExtra() {
			return 6;
		}
	
		public long getMask() {
			return mask;
		}
		
		public void setMask(long mask) {
			this.mask = mask;
		}
		
		public int getLat() {
			return lat;
		}
		
		public void setLat(int lat) {
			this.lat = lat;
		}
		
		public int getLon() {
			return lon;
		}
		
		public void setLon(int lon) {
			this.lon = lon;
		}
		
		public int getGrid_spacing() {
			return grid_spacing;
		}
		
		public void setGrid_spacing(int grid_spacing) {
			this.grid_spacing = grid_spacing;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "TERRAIN_REQUEST";
			
			mask = buffer.getLong(); // uint64_t
  			lat = buffer.getInt(); // int32_t
  			lon = buffer.getInt(); // int32_t
  			grid_spacing = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(mask); // uint64_t
  			buffer.putInt((int)(lat)); // int32_t
  			buffer.putInt((int)(lon)); // int32_t
  			buffer.putShort((short)(grid_spacing & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_TERRAIN_REQUEST { " + 
			"mask = " + mask + ", " + 
			"lat = " + lat + ", " + 
			"lon = " + lon + ", " + 
			"grid_spacing = " + grid_spacing + ",  }";
		}
	}
	/*
	 * Time synchronization message.
	 */
	public static class MSG_TIMESYNC extends Message {
	
		private long tc1; // Time sync timestamp 1
		private long ts1; // Time sync timestamp 2
	
		public MSG_TIMESYNC () {
			super(MSG_ID_TIMESYNC, (short)128, "TIMESYNC");
		}

		public MSG_TIMESYNC (byte[] bytes) {
			super(bytes, MSG_ID_TIMESYNC, (short)128, "TIMESYNC");
		}
	
		public MSG_TIMESYNC (short sys, short comp, long tc1  , long ts1  ) {
			super(sys, comp, (short)128, "TIMESYNC");
			this.tc1 = tc1;
			this.ts1 = ts1;
		}

		@Override
		public int getCRCExtra() {
			return 34;
		}
	
		public long getTc1() {
			return tc1;
		}
		
		public void setTc1(long tc1) {
			this.tc1 = tc1;
		}
		
		public long getTs1() {
			return ts1;
		}
		
		public void setTs1(long ts1) {
			this.ts1 = ts1;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "TIMESYNC";
			
			tc1 = buffer.getLong(); // int64_t
  			ts1 = buffer.getLong(); // int64_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(tc1); // int64_t
  			buffer.putLong(ts1); // int64_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_TIMESYNC { " + 
			"tc1 = " + tc1 + ", " + 
			"ts1 = " + ts1 + ",  }";
		}
	}
	/*
	 * Message implementing parts of the V2 payload specs in V1 frames for transitional support.
	 */
	public static class MSG_V2_EXTENSION extends Message {
	
		private int message_type; // A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
		private int target_network; // Network ID (0 for broadcast)
		private int target_system; // System ID (0 for broadcast)
		private int target_component; // Component ID (0 for broadcast)
		private int[] payload = new int[249]; // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
	
		public MSG_V2_EXTENSION () {
			super(MSG_ID_V2_EXTENSION, (short)6, "V2_EXTENSION");
		}

		public MSG_V2_EXTENSION (byte[] bytes) {
			super(bytes, MSG_ID_V2_EXTENSION, (short)6, "V2_EXTENSION");
		}
	
		public MSG_V2_EXTENSION (short sys, short comp, int message_type  , int target_network  , int target_system  , int target_component  , int payload [] ) {
			super(sys, comp, (short)6, "V2_EXTENSION");
			this.message_type = message_type;
			this.target_network = target_network;
			this.target_system = target_system;
			this.target_component = target_component;
			this.payload = payload;
		}

		@Override
		public int getCRCExtra() {
			return 48;
		}
	
		public int getMessage_type() {
			return message_type;
		}
		
		public void setMessage_type(int message_type) {
			this.message_type = message_type;
		}
		
		public int getTarget_network() {
			return target_network;
		}
		
		public void setTarget_network(int target_network) {
			this.target_network = target_network;
		}
		
		public int getTarget_system() {
			return target_system;
		}
		
		public void setTarget_system(int target_system) {
			this.target_system = target_system;
		}
		
		public int getTarget_component() {
			return target_component;
		}
		
		public void setTarget_component(int target_component) {
			this.target_component = target_component;
		}
		
		public int[] getPayload() {
			return payload;
		}
		
		public void setPayload(int payload[]) {
			this.payload = payload;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "V2_EXTENSION";
			
			message_type = buffer.getShort() & 0xffff; // uint16_t
  			target_network = (int)buffer.get() & 0xff; // uint8_t
  			target_system = (int)buffer.get() & 0xff; // uint8_t
  			target_component = (int)buffer.get() & 0xff; // uint8_t
  			for(int c=0; c<249; ++c) {
				payload [c] =  (int)buffer.get() & 0xff; // uint8_t[249]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(message_type & 0xffff)); // uint16_t
  			buffer.put((byte)(target_network & 0xff)); // uint8_t
  			buffer.put((byte)(target_system & 0xff)); // uint8_t
  			buffer.put((byte)(target_component & 0xff)); // uint8_t
  			for(int c=0; c<249; ++c) {
				buffer.put((byte)(payload [c] & 0xff));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_V2_EXTENSION { " + 
			"message_type = " + message_type + ", " + 
			"target_network = " + target_network + ", " + 
			"target_system = " + target_system + ", " + 
			"target_component = " + target_component + ", " + 
			"payload = " + payload + ",  }";
		}
	}
	/*
	 * Metrics typically displayed on a HUD for fixed wing aircraft
	 */
	public static class MSG_VFR_HUD extends Message {
	
		private float airspeed; // Current airspeed in m/s
		private float groundspeed; // Current ground speed in m/s
		private float alt; // Current altitude (MSL), in meters
		private float climb; // Current climb rate in meters/second
		private int heading; // Current heading in degrees, in compass units (0..360, 0=north)
		private int throttle; // Current throttle setting in integer percent, 0 to 100
	
		public MSG_VFR_HUD () {
			super(MSG_ID_VFR_HUD, (short)20, "VFR_HUD");
		}

		public MSG_VFR_HUD (byte[] bytes) {
			super(bytes, MSG_ID_VFR_HUD, (short)20, "VFR_HUD");
		}
	
		public MSG_VFR_HUD (short sys, short comp, float airspeed  , float groundspeed  , float alt  , float climb  , int heading  , int throttle  ) {
			super(sys, comp, (short)20, "VFR_HUD");
			this.airspeed = airspeed;
			this.groundspeed = groundspeed;
			this.alt = alt;
			this.climb = climb;
			this.heading = heading;
			this.throttle = throttle;
		}

		@Override
		public int getCRCExtra() {
			return 20;
		}
	
		public float getAirspeed() {
			return airspeed;
		}
		
		public void setAirspeed(float airspeed) {
			this.airspeed = airspeed;
		}
		
		public float getGroundspeed() {
			return groundspeed;
		}
		
		public void setGroundspeed(float groundspeed) {
			this.groundspeed = groundspeed;
		}
		
		public float getAlt() {
			return alt;
		}
		
		public void setAlt(float alt) {
			this.alt = alt;
		}
		
		public float getClimb() {
			return climb;
		}
		
		public void setClimb(float climb) {
			this.climb = climb;
		}
		
		public int getHeading() {
			return heading;
		}
		
		public void setHeading(int heading) {
			this.heading = heading;
		}
		
		public int getThrottle() {
			return throttle;
		}
		
		public void setThrottle(int throttle) {
			this.throttle = throttle;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "VFR_HUD";
			
			airspeed = buffer.getFloat(); // float
  			groundspeed = buffer.getFloat(); // float
  			alt = buffer.getFloat(); // float
  			climb = buffer.getFloat(); // float
  			heading = buffer.getShort(); // int16_t
  			throttle = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putFloat(airspeed); // float
  			buffer.putFloat(groundspeed); // float
  			buffer.putFloat(alt); // float
  			buffer.putFloat(climb); // float
  			buffer.putShort((short)(heading)); // int16_t
  			buffer.putShort((short)(throttle & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_VFR_HUD { " + 
			"airspeed = " + airspeed + ", " + 
			"groundspeed = " + groundspeed + ", " + 
			"alt = " + alt + ", " + 
			"climb = " + climb + ", " + 
			"heading = " + heading + ", " + 
			"throttle = " + throttle + ",  }";
		}
	}
	public static class MSG_VICON_POSITION_ESTIMATE extends Message {
	
		private long usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float x; // Global X position
		private float y; // Global Y position
		private float z; // Global Z position
		private float roll; // Roll angle in rad
		private float pitch; // Pitch angle in rad
		private float yaw; // Yaw angle in rad
	
		public MSG_VICON_POSITION_ESTIMATE () {
			super(MSG_ID_VICON_POSITION_ESTIMATE, (short)88, "VICON_POSITION_ESTIMATE");
		}

		public MSG_VICON_POSITION_ESTIMATE (byte[] bytes) {
			super(bytes, MSG_ID_VICON_POSITION_ESTIMATE, (short)88, "VICON_POSITION_ESTIMATE");
		}
	
		public MSG_VICON_POSITION_ESTIMATE (short sys, short comp, long usec  , float x  , float y  , float z  , float roll  , float pitch  , float yaw  ) {
			super(sys, comp, (short)88, "VICON_POSITION_ESTIMATE");
			this.usec = usec;
			this.x = x;
			this.y = y;
			this.z = z;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
		}

		@Override
		public int getCRCExtra() {
			return 56;
		}
	
		public long getUsec() {
			return usec;
		}
		
		public void setUsec(long usec) {
			this.usec = usec;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "VICON_POSITION_ESTIMATE";
			
			usec = buffer.getLong(); // uint64_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(usec); // uint64_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_VICON_POSITION_ESTIMATE { " + 
			"usec = " + usec + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ",  }";
		}
	}
	public static class MSG_VISION_POSITION_ESTIMATE extends Message {
	
		private long usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float x; // Global X position
		private float y; // Global Y position
		private float z; // Global Z position
		private float roll; // Roll angle in rad
		private float pitch; // Pitch angle in rad
		private float yaw; // Yaw angle in rad
	
		public MSG_VISION_POSITION_ESTIMATE () {
			super(MSG_ID_VISION_POSITION_ESTIMATE, (short)88, "VISION_POSITION_ESTIMATE");
		}

		public MSG_VISION_POSITION_ESTIMATE (byte[] bytes) {
			super(bytes, MSG_ID_VISION_POSITION_ESTIMATE, (short)88, "VISION_POSITION_ESTIMATE");
		}
	
		public MSG_VISION_POSITION_ESTIMATE (short sys, short comp, long usec  , float x  , float y  , float z  , float roll  , float pitch  , float yaw  ) {
			super(sys, comp, (short)88, "VISION_POSITION_ESTIMATE");
			this.usec = usec;
			this.x = x;
			this.y = y;
			this.z = z;
			this.roll = roll;
			this.pitch = pitch;
			this.yaw = yaw;
		}

		@Override
		public int getCRCExtra() {
			return 158;
		}
	
		public long getUsec() {
			return usec;
		}
		
		public void setUsec(long usec) {
			this.usec = usec;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
		public float getRoll() {
			return roll;
		}
		
		public void setRoll(float roll) {
			this.roll = roll;
		}
		
		public float getPitch() {
			return pitch;
		}
		
		public void setPitch(float pitch) {
			this.pitch = pitch;
		}
		
		public float getYaw() {
			return yaw;
		}
		
		public void setYaw(float yaw) {
			this.yaw = yaw;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "VISION_POSITION_ESTIMATE";
			
			usec = buffer.getLong(); // uint64_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
  			roll = buffer.getFloat(); // float
  			pitch = buffer.getFloat(); // float
  			yaw = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(usec); // uint64_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
  			buffer.putFloat(roll); // float
  			buffer.putFloat(pitch); // float
  			buffer.putFloat(yaw); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_VISION_POSITION_ESTIMATE { " + 
			"usec = " + usec + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ", " + 
			"roll = " + roll + ", " + 
			"pitch = " + pitch + ", " + 
			"yaw = " + yaw + ",  }";
		}
	}
	public static class MSG_VISION_SPEED_ESTIMATE extends Message {
	
		private long usec; // Timestamp (microseconds, synced to UNIX time or since system boot)
		private float x; // Global X speed
		private float y; // Global Y speed
		private float z; // Global Z speed
	
		public MSG_VISION_SPEED_ESTIMATE () {
			super(MSG_ID_VISION_SPEED_ESTIMATE, (short)76, "VISION_SPEED_ESTIMATE");
		}

		public MSG_VISION_SPEED_ESTIMATE (byte[] bytes) {
			super(bytes, MSG_ID_VISION_SPEED_ESTIMATE, (short)76, "VISION_SPEED_ESTIMATE");
		}
	
		public MSG_VISION_SPEED_ESTIMATE (short sys, short comp, long usec  , float x  , float y  , float z  ) {
			super(sys, comp, (short)76, "VISION_SPEED_ESTIMATE");
			this.usec = usec;
			this.x = x;
			this.y = y;
			this.z = z;
		}

		@Override
		public int getCRCExtra() {
			return 208;
		}
	
		public long getUsec() {
			return usec;
		}
		
		public void setUsec(long usec) {
			this.usec = usec;
		}
		
		public float getX() {
			return x;
		}
		
		public void setX(float x) {
			this.x = x;
		}
		
		public float getY() {
			return y;
		}
		
		public void setY(float y) {
			this.y = y;
		}
		
		public float getZ() {
			return z;
		}
		
		public void setZ(float z) {
			this.z = z;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "VISION_SPEED_ESTIMATE";
			
			usec = buffer.getLong(); // uint64_t
  			x = buffer.getFloat(); // float
  			y = buffer.getFloat(); // float
  			z = buffer.getFloat(); // float
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putLong(usec); // uint64_t
  			buffer.putFloat(x); // float
  			buffer.putFloat(y); // float
  			buffer.putFloat(z); // float
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_VISION_SPEED_ESTIMATE { " + 
			"usec = " + usec + ", " + 
			"x = " + x + ", " + 
			"y = " + y + ", " + 
			"z = " + z + ",  }";
		}
	}
	public static class MSG_WATCHDOG_COMMAND extends Message {
	
		private int watchdog_id; // Watchdog ID
		private int process_id; // Process ID
		private int target_system_id; // Target system ID
		private int command_id; // Command ID
	
		public MSG_WATCHDOG_COMMAND () {
			super(MSG_ID_WATCHDOG_COMMAND, (short)6, "WATCHDOG_COMMAND");
		}

		public MSG_WATCHDOG_COMMAND (byte[] bytes) {
			super(bytes, MSG_ID_WATCHDOG_COMMAND, (short)6, "WATCHDOG_COMMAND");
		}
	
		public MSG_WATCHDOG_COMMAND (short sys, short comp, int watchdog_id  , int process_id  , int target_system_id  , int command_id  ) {
			super(sys, comp, (short)6, "WATCHDOG_COMMAND");
			this.watchdog_id = watchdog_id;
			this.process_id = process_id;
			this.target_system_id = target_system_id;
			this.command_id = command_id;
		}

		@Override
		public int getCRCExtra() {
			return 162;
		}
	
		public int getWatchdog_id() {
			return watchdog_id;
		}
		
		public void setWatchdog_id(int watchdog_id) {
			this.watchdog_id = watchdog_id;
		}
		
		public int getProcess_id() {
			return process_id;
		}
		
		public void setProcess_id(int process_id) {
			this.process_id = process_id;
		}
		
		public int getTarget_system_id() {
			return target_system_id;
		}
		
		public void setTarget_system_id(int target_system_id) {
			this.target_system_id = target_system_id;
		}
		
		public int getCommand_id() {
			return command_id;
		}
		
		public void setCommand_id(int command_id) {
			this.command_id = command_id;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "WATCHDOG_COMMAND";
			
			watchdog_id = buffer.getShort() & 0xffff; // uint16_t
  			process_id = buffer.getShort() & 0xffff; // uint16_t
  			target_system_id = (int)buffer.get() & 0xff; // uint8_t
  			command_id = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(watchdog_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(process_id & 0xffff)); // uint16_t
  			buffer.put((byte)(target_system_id & 0xff)); // uint8_t
  			buffer.put((byte)(command_id & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_WATCHDOG_COMMAND { " + 
			"watchdog_id = " + watchdog_id + ", " + 
			"process_id = " + process_id + ", " + 
			"target_system_id = " + target_system_id + ", " + 
			"command_id = " + command_id + ",  }";
		}
	}
	public static class MSG_WATCHDOG_HEARTBEAT extends Message {
	
		private int watchdog_id; // Watchdog ID
		private int process_count; // Number of processes
	
		public MSG_WATCHDOG_HEARTBEAT () {
			super(MSG_ID_WATCHDOG_HEARTBEAT, (short)4, "WATCHDOG_HEARTBEAT");
		}

		public MSG_WATCHDOG_HEARTBEAT (byte[] bytes) {
			super(bytes, MSG_ID_WATCHDOG_HEARTBEAT, (short)4, "WATCHDOG_HEARTBEAT");
		}
	
		public MSG_WATCHDOG_HEARTBEAT (short sys, short comp, int watchdog_id  , int process_count  ) {
			super(sys, comp, (short)4, "WATCHDOG_HEARTBEAT");
			this.watchdog_id = watchdog_id;
			this.process_count = process_count;
		}

		@Override
		public int getCRCExtra() {
			return 153;
		}
	
		public int getWatchdog_id() {
			return watchdog_id;
		}
		
		public void setWatchdog_id(int watchdog_id) {
			this.watchdog_id = watchdog_id;
		}
		
		public int getProcess_count() {
			return process_count;
		}
		
		public void setProcess_count(int process_count) {
			this.process_count = process_count;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "WATCHDOG_HEARTBEAT";
			
			watchdog_id = buffer.getShort() & 0xffff; // uint16_t
  			process_count = buffer.getShort() & 0xffff; // uint16_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putShort((short)(watchdog_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(process_count & 0xffff)); // uint16_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_WATCHDOG_HEARTBEAT { " + 
			"watchdog_id = " + watchdog_id + ", " + 
			"process_count = " + process_count + ",  }";
		}
	}
	public static class MSG_WATCHDOG_PROCESS_INFO extends Message {
	
		private int timeout; // Timeout (seconds)
		private int watchdog_id; // Watchdog ID
		private int process_id; // Process ID
		private char[] name = new char[100]; // Process name
		private char[] arguments = new char[147]; // Process arguments
	
		public MSG_WATCHDOG_PROCESS_INFO () {
			super(MSG_ID_WATCHDOG_PROCESS_INFO, (short)10, "WATCHDOG_PROCESS_INFO");
		}

		public MSG_WATCHDOG_PROCESS_INFO (byte[] bytes) {
			super(bytes, MSG_ID_WATCHDOG_PROCESS_INFO, (short)10, "WATCHDOG_PROCESS_INFO");
		}
	
		public MSG_WATCHDOG_PROCESS_INFO (short sys, short comp, int timeout  , int watchdog_id  , int process_id  , char name [] , char arguments [] ) {
			super(sys, comp, (short)10, "WATCHDOG_PROCESS_INFO");
			this.timeout = timeout;
			this.watchdog_id = watchdog_id;
			this.process_id = process_id;
			this.name = name;
			this.arguments = arguments;
		}

		@Override
		public int getCRCExtra() {
			return 113;
		}
	
		public int getTimeout() {
			return timeout;
		}
		
		public void setTimeout(int timeout) {
			this.timeout = timeout;
		}
		
		public int getWatchdog_id() {
			return watchdog_id;
		}
		
		public void setWatchdog_id(int watchdog_id) {
			this.watchdog_id = watchdog_id;
		}
		
		public int getProcess_id() {
			return process_id;
		}
		
		public void setProcess_id(int process_id) {
			this.process_id = process_id;
		}
		
		public char[] getName() {
			return name;
		}
		
		public void setName(char name[]) {
			this.name = name;
		}
		
		public char[] getArguments() {
			return arguments;
		}
		
		public void setArguments(char arguments[]) {
			this.arguments = arguments;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "WATCHDOG_PROCESS_INFO";
			
			timeout = buffer.getInt(); // int32_t
  			watchdog_id = buffer.getShort() & 0xffff; // uint16_t
  			process_id = buffer.getShort() & 0xffff; // uint16_t
  			for(int c=0; c<100; ++c) {
				name [c] =  (char)buffer.get(); // char[100]
 			}
			
 			for(int c=0; c<147; ++c) {
				arguments [c] =  (char)buffer.get(); // char[147]
 			}
			
  			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(timeout)); // int32_t
  			buffer.putShort((short)(watchdog_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(process_id & 0xffff)); // uint16_t
  			for(int c=0; c<100; ++c) {
				buffer.put((byte)(name [c]));
 			}
			
 			for(int c=0; c<147; ++c) {
				buffer.put((byte)(arguments [c]));
 			}
			
  			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_WATCHDOG_PROCESS_INFO { " + 
			"timeout = " + timeout + ", " + 
			"watchdog_id = " + watchdog_id + ", " + 
			"process_id = " + process_id + ", " + 
			"name = " + name + ", " + 
			"arguments = " + arguments + ",  }";
		}
	}
	public static class MSG_WATCHDOG_PROCESS_STATUS extends Message {
	
		private int pid; // PID
		private int watchdog_id; // Watchdog ID
		private int process_id; // Process ID
		private int crashes; // Number of crashes
		private int state; // Is running / finished / suspended / crashed
		private int muted; // Is muted
	
		public MSG_WATCHDOG_PROCESS_STATUS () {
			super(MSG_ID_WATCHDOG_PROCESS_STATUS, (short)12, "WATCHDOG_PROCESS_STATUS");
		}

		public MSG_WATCHDOG_PROCESS_STATUS (byte[] bytes) {
			super(bytes, MSG_ID_WATCHDOG_PROCESS_STATUS, (short)12, "WATCHDOG_PROCESS_STATUS");
		}
	
		public MSG_WATCHDOG_PROCESS_STATUS (short sys, short comp, int pid  , int watchdog_id  , int process_id  , int crashes  , int state  , int muted  ) {
			super(sys, comp, (short)12, "WATCHDOG_PROCESS_STATUS");
			this.pid = pid;
			this.watchdog_id = watchdog_id;
			this.process_id = process_id;
			this.crashes = crashes;
			this.state = state;
			this.muted = muted;
		}

		@Override
		public int getCRCExtra() {
			return 29;
		}
	
		public int getPid() {
			return pid;
		}
		
		public void setPid(int pid) {
			this.pid = pid;
		}
		
		public int getWatchdog_id() {
			return watchdog_id;
		}
		
		public void setWatchdog_id(int watchdog_id) {
			this.watchdog_id = watchdog_id;
		}
		
		public int getProcess_id() {
			return process_id;
		}
		
		public void setProcess_id(int process_id) {
			this.process_id = process_id;
		}
		
		public int getCrashes() {
			return crashes;
		}
		
		public void setCrashes(int crashes) {
			this.crashes = crashes;
		}
		
		public int getState() {
			return state;
		}
		
		public void setState(int state) {
			this.state = state;
		}
		
		public int getMuted() {
			return muted;
		}
		
		public void setMuted(int muted) {
			this.muted = muted;
		}
		
			
		protected ByteBuffer decodePayload(ByteBuffer buffer) {
			msgName = "WATCHDOG_PROCESS_STATUS";
			
			pid = buffer.getInt(); // int32_t
  			watchdog_id = buffer.getShort() & 0xffff; // uint16_t
  			process_id = buffer.getShort() & 0xffff; // uint16_t
  			crashes = buffer.getShort() & 0xffff; // uint16_t
  			state = (int)buffer.get() & 0xff; // uint8_t
  			muted = (int)buffer.get() & 0xff; // uint8_t
   			return buffer;
		}
			
		public ByteBuffer encodePayload(ByteBuffer buffer) {
			
			buffer.putInt((int)(pid)); // int32_t
  			buffer.putShort((short)(watchdog_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(process_id & 0xffff)); // uint16_t
  			buffer.putShort((short)(crashes & 0xffff)); // uint16_t
  			buffer.put((byte)(state & 0xff)); // uint8_t
  			buffer.put((byte)(muted & 0xff)); // uint8_t
   			return buffer;
		}
		
		@Override
		public String toString() {
			return "MSG_WATCHDOG_PROCESS_STATUS { " + 
			"pid = " + pid + ", " + 
			"watchdog_id = " + watchdog_id + ", " + 
			"process_id = " + process_id + ", " + 
			"crashes = " + crashes + ", " + 
			"state = " + state + ", " + 
			"muted = " + muted + ",  }";
		}
	}

}