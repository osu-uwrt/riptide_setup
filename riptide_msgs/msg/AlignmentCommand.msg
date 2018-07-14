std_msgs/Header header

string object_name
int8 alignment_plane

# Relative position in pixels
int32 bbox_dim
int8 bbox_control # 0 for width, 1 for height
geometry_msgs/Point target_pos

geometry_msgs/Vector3 euler_rpy
riptide_msgs/DepthCommand depth
