# VisionPoseStatus.msg
# State of vision_pose_node, the node that gives PX4 its pose (mocap or onboard VIO).

# Standard ROS Header (stamp: time of publication)
std_msgs/Header header

# What PX4 is fed
uint8 SOURCE_NONE=0
uint8 SOURCE_MOCAP=1
uint8 SOURCE_VIO=2

uint8 source

bool use_vio            # parameter: PX4 may be fed the VIO
bool vio_alive          # a VIO pose arrived within vio_timeout_s
bool vio_ready          # the VIO is settled and placed in the room
bool mocap_valid        # the latest mocap sample is valid and less than 0.1 s old
bool armed              # from the latest vehicle status (false until one is received)
bool fallback_latched   # the VIO was given up; holds until the node restarts

# Why PX4 is not fed the VIO (REASON_NONE while it is)
uint8 REASON_NONE=0
uint8 REASON_VIO_NOT_READY_BEFORE_ARMING=1
uint8 REASON_VIO_TIMEOUT=2
uint8 REASON_DISTANCE_LIMIT=3
uint8 REASON_NO_MOUNT=4
uint8 REASON_NO_VEHICLE_STATUS=5
uint8 REASON_OPERATOR=6
uint8 REASON_VIO_NOT_READY=7
uint8 REASON_NO_VEHICLE_ODOMETRY=8   # PX4's own position is not arriving: the distance rule cannot run

uint8 reason

# reset_counter of the messages sent to PX4
uint8 reset_counter

float32 heading_deg       # VIO world -> room heading in use (NaN until placed)
float32 heading_trim_deg  # part of it added by the heading trim
float32 heading_fit_deg   # rotation about the pin that maps the untrimmed placed VIO onto the mocap,
                          # i.e. the position-based estimate of the heading error (NaN until available)
float32 body_tilt_deg     # tilt of the rigid body -> IMU body rotation identified at this start (NaN if none)

float32 vio_mocap_dist_m  # placed VIO to mocap at the same stamp (NaN if none)
float32 px4_mocap_dist_m  # PX4 position to mocap at the same stamp (NaN if none)
float32 vio_age_s         # age of the latest VIO pose when it arrived (NaN if none)

uint32 vio_poses          # VIO poses accepted
uint32 px4_sent           # messages sent to PX4
uint32 fallbacks          # VIO -> mocap fallbacks
uint32 dist_exceed_count  # crossings of fallback_dist_m (PX4 to mocap on the VIO, placed VIO to mocap otherwise)
