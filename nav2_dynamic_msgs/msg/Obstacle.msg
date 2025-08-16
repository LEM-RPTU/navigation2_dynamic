unique_identifier_msgs/UUID uuid         # For static obstacles, fill with an invalid value (all zeros)
int64 id                            # Unique identifier for the obstacle, used for tracking.
float32 score                           # Detection confidence
geometry_msgs/Point[] position          # First entry is current centroid, others are VAR predictions.
geometry_msgs/Vector3 velocity          # Zero for static obstacles.
geometry_msgs/Vector3 heading           # Zero for static obstacles.
geometry_msgs/Polygon polygon           # Convex hull of the obstacle.
float64[4] position_covariance          # Position covariance (pad VAR MSE to 9 values)
float64[4] velocity_covariance          # Velocity covariance (if dynamic, nonzero)