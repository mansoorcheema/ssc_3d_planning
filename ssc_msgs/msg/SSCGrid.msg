# origin of occupancy grid
float32 origin_x
float32 origin_y
float32 origin_z

# reference frame of the grid
string frame

uint32 width
uint32 height
uint32 depth

# Voxel data packed in 4-byte chunks to better mirror protobuf serialization.
float32[] data
