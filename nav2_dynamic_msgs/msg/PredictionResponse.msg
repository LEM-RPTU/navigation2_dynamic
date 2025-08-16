# PredictionResponse.msg
std_msgs/Header header
int32 obstacle_id
geometry_msgs/Point[] predicted_positions
float64[] prediction_confidence  # Optional confidence scores