std_msgs/Header header

# The frame ID of the image that the apriltags were detected on
int64 frameId

# The frame timestamp (nanoseconds) of the image that the apriltags were detected on
int64 timestamp

# Success flag to indicate whether for the apriltag algorithm ran successfully
uint8 success

# The number of apriltags that were detected
uint32 numDetections

# The collection of apriltag detections
AprilTagDetection[] detections
