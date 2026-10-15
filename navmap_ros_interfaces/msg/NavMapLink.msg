# Link between two NavCels that do not share an edge (stairs, elevator, door...).
# The map only records it: whether a robot can take it is up to the planner.

uint8 GENERIC = 0
uint8 RAMP = 1
uint8 STAIRS = 2
uint8 ELEVATOR = 3
uint8 DOOR = 4

uint32 from_navcel
uint32 to_navcel
uint8 type
float32 length        # travel length (m)
bool bidirectional    # also from to_navcel to from_navcel
