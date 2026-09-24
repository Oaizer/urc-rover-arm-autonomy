# Pixel corners in detector order; pose is camera optical <- marker, metres.
int32 id
geometry_msgs/Point32[4] corners
# Consumers MUST ignore pose unless pose_valid is true.
bool pose_valid
geometry_msgs/Pose pose
float64 reprojection_error_px
bool ambiguous
