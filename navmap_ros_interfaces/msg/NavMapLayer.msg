# Per-NavCel layer (one scalar per triangle)
std_msgs/Header header

uint8 U8 = 0
uint8 F32 = 1
uint8 F64 = 2
uint8 I16 = 3

string name
uint8 type         # 0=U8, 1=F32, 2=F64, 3=I16

# Metadata
string description
string unit
bool has_unknown        # true: unknown_value means "not observed" (NaN allowed)
float64 unknown_value

# Geometry this layer belongs to (NavMap::geometry_id(); 0 = unspecified)
uint64 geometry_id

# Sparse update: if non-empty, the data_* array holds one value per listed NavCel and the
# rest of the layer is unchanged. If empty, the data_* array covers every NavCel.
uint32[] indices

# Exactly one of the data_* arrays must be non-empty, and its length must be
# equal to the number of triangles (NavCels) in the NavMap message (or to indices).
uint8[]   data_u8
float32[] data_f32
float64[] data_f64
int16[]   data_i16
