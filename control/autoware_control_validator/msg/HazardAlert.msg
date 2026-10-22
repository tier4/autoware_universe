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

HazardSector[8] sectors  # index = sector constant above
