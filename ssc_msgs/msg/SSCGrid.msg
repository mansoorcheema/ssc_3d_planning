# origin of occupancy grid
int32 origin_x
int32 origin_y
int32 origin_z

# reference frame of the grid
string frame

uint32 width
uint32 height
uint32 depth

# Voxel data packed in 4-byte chunks to better mirror protobuf serialization.
uint32[] data
