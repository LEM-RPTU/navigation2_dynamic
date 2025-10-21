std_msgs/Header header
unique_identifier_msgs/UUID uuid             # For static obstacles, fill with an invalid value (all zeros)
int64 id                                     # Unique identifier for the obstacle, used for tracking
#
# time difference between positions/velocities relative to current header/time stamp, i.e.
# position[0] would be position at header/time stamp
# position[1] would be position at header/time stamp + delta_time 
# position[2] would be position at header/time stamp + 2*delta_time ...
builtin_interfaces/Duration delta_time    
# 
geometry_msgs/PoseWithCovariance[] position  
geometry_msgs/TwistWithCovariance[] velocity 
geometry_msgs/Polygon polygon                # Convex hull of the obstacle.
