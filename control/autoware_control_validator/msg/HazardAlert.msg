std_msgs/Header header  # stamp = evaluation time, frame_id = base_link

# Sector indices (same order as ProximityHazardObjects)
uint8 FRONT=0
uint8 FRONT_RIGHT=1
uint8 RIGHT=2
uint8 REAR_RIGHT=3
uint8 REAR=4
uint8 REAR_LEFT=5
uint8 LEFT=6
uint8 FRONT_LEFT=7

bool[8] has_object                  # collision latch of the sector is ON
float32[8] ttc_s                    # minimum TTC among candidates raised this cycle [s]; NaN while holding
uint8[8] level                      # hazard level: number of ttc_thresholds the TTC is below (0 = none)
bool is_boundary_departure          # road_border departure latch is ON
float32 boundary_lateral_distance_m # minimum lateral distance to an uncrossable boundary [m]
