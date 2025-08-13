R_MTR = 0
L_MTR = 1
FWD = 1
RWD = 0

MIN_SPEED = 128
MAX_SPEED = 255

# Dedicated speeds for in-place rotations (tune on hardware)
TURN_MIN_SPEED = 95   # Slightly higher minimum for better control
TURN_MAX_SPEED = 110  # Much lower maximum to reduce overshoot


WHEEL_DIAMETER = 68 # mm
TRACK_WIDTH = 220 # mm (center to center of each wheel)

# Per-wheel encoder calibration (measured PPR)
PULSES_PER_ROTATION_LEFT = 2466
PULSES_PER_ROTATION_RIGHT = 2339

# Encoder sign normalization: multiply raw counts by these so forward is positive
ENCODER_LEFT_SIGN = -1
ENCODER_RIGHT_SIGN = 1

OLLAMA_HOST = "http://vengeance.matthewtwo.com:11434"
TEMPERATURE = 1