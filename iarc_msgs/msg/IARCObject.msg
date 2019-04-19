uint8 OBJ_NULL=0   # Null (class-agnostic)
uint8 OBJ_PERSON=1 # Person (operator)
uint8 OBJ_ENEMY=2  # Enemy drone
uint8 OBJ_DRONE=3  # Friendly drone
uint8 OBJ_BIN=4    # Bin
uint8 OBJ_QR=5     # QR Code

# Note : There is a lot of redundancy built in to the following fields.
# Potentially prune out the fields that are not useful in the future.

string source             # Observation Source (TF Frame)
uint8 obj_id              # Object ID
iarc_msgs/Box box         # Pixel-Space Bounding Box
float32[9] covariance     # 3DoF Position Uncertainty  (in observation frame)
float32[3] volume         # Estimated Occupancy Volume (in observation frame)
float32[2] distance_range # Estimated Distance From Source
geometry_msgs/PointStamped centroid     # Estimated Object Centroid
