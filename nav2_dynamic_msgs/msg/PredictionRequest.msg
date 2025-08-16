# PredictionRequest.msg
std_msgs/Header header
int32 obstacle_id
geometry_msgs/Point[] history  # Position history
int32 prediction_steps         # How many steps to predict (t+1 for tracking, t+N for final output)