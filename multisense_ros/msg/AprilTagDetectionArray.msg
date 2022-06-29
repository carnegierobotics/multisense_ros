std_msgs/Header header

# The frame timestamp (nanoseconds) of the image that the apriltags were detected on
int64 timestamp

# The image source that the apriltags were detected on
string imageSource

# Success flag to indicate whether for the apriltag algorithm ran successfully
uint8 success

# The number of apriltags that were detected
uint32 numDetections

# The collection of apriltag detections
AprilTagDetection[] detections
