# Note : There is a lot of redundancy built in to the following fields.
# Potentially prune out the fields that are not useful in the future.

string source             # Observation Source (TF Frame)
time stamp                # Last Observation Time
iarc_msgs/Identifier obj_id
iarc_msgs/Box box         # Pixel-Space Bounding Box
float32[9] covariance     # 3DoF Position Uncertainty  (in observation frame)
float32[3] volume         # Estimated Occupancy Volume (in observation frame)
float32[2] distance_range # Estimated Distance From Source
geometry_msgs/PointStamped centroid     # Estimated Object Centroid
