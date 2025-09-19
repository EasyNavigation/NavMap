# Per-NavCel layer (one scalar per triangle)
std_msgs/Header header

uint8 U8 = 0
uint8 F32 = 1
uint8 F64 = 2

string name
uint8 type         # 0=U8, 1=F32, 2=F64

# Exactly one of the data_* arrays must be non-empty, and its length must be
# equal to the number of triangles (NavCels) in the NavMap message.
uint8[]   data_u8
float32[] data_f32
float64[] data_f64
