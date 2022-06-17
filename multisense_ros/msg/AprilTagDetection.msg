# The family of the tag
uint64 family

# The ID of the tag
uint32 id

# The hamming distance between the detection and the real code
uint8 hamming

# The quality/confidence of the binary decoding process
# average difference between intensity of data bit vs decision thresh.
# Higher is better. Only useful for small tags
float32 decisionMargin

# The 3x3 homography matrix describing the projection from an
# "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
# -1)) to pixels in the image
std_msgs/Float64MultiArray tagToImageHomography

# The 2D position of the origin of the tag in the image
float64[2] center

# The 4 tag corner pixel locations in the order:
# point 0: [-squareLength / 2, squareLength / 2]
# point 1: [ squareLength / 2, squareLength / 2]
# point 2: [ squareLength / 2, -squareLength / 2]
# point 3: [-squareLength / 2, -squareLength / 2]
AprilTagCornerPoint[4] corners
