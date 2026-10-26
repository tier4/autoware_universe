# For debugging. No compatibility guarantee; removed when the development ends.
builtin_interfaces/Time source_stamp  # header.stamp of the PredictedObjects / PointCloud2 the candidate came from

uint8 KIND_OBJECT=0
uint8 KIND_POINTCLOUD=1

uint8 kind
uint8 label               # ObjectClassification label; 0 for a point
float32 ttc_s             # time to collision [s]; NaN when the candidate does not collide
float32 cos_collision     # cosine of the collision angle; NaN when the candidate does not collide
uint8 level               # hazard level of this candidate (risk_level, 0 to 4; 0 = not raised)
