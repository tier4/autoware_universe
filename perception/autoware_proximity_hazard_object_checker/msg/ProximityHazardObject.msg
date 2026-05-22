bool has_object                                             # true if a hazard is present in this sector
float32 distance_m                                          # Polygon-to-polygon distance [m]; valid iff has_object
autoware_perception_msgs/PredictedObject predicted_object   # Source perception object, propagated unchanged; valid iff has_object
