std_msgs/Header header

int8 YZ=0 # Forward cameras
int8 XY=1 # Downward cameras

# Plane in which we are aligning
int8 alignment_plane

# Relative position in pixels
geometry_msgs/Point target_pos

# Bounding Box width in pixels. Useful for controlling
# an approximate distance.
int32 bbox_width
