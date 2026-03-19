class Config:
    # Camera
    CAM_INDEX = 0
    CAM_W = 640
    CAM_H = 480
    CAM_FPS = 30.0

    # Serial Communication
    BAUD = 115200
    SERIAL_TIMEOUT = 0.1
    SERIAL_HANDSHAKE = True
    SERIAL_WAIT_TIMEOUT = 15.0
    SERIAL_PORT = 'dev/ttyACM0' # port on Raspberry PI, fwt and I'll cut you

    # Main Loop
    LOOP_HZ = 20.0
    RUN_TIME_S = 20.0

    # Color Thresholds
    RED_LOWER1 = (0, 100, 100)
    RED_UPPER1 = (10, 255, 255)
    RED_LOWER2 = (160, 100, 100)
    RED_UPPER2 = (180, 255, 255)

    # Image Processing
    MIN_MASK_AREA = 300
    CANNY1, CANNY2 = 50, 150
    HOUGH_THRESH = 30
    MIN_LINE_LEN = 25
    MAX_LINE_GAP = 10
    MORPH_K = 5
    YREF_FRAC = 0.85
    MAX_ABS_DEG_FROM_VERTICAL = 60.0

    # Debug
    DEBUG_SHOW = False
    DEBUG_DRAW = False

    # Robot Geometry
    r = 0.04
    L = 0.20
    vmax = 0.2
    wmax = vmax / r  # 6.25 rad/s is based off latency constraint, not firm

    # Encoder
    ENCODER_CPR = 2797

    # Timing
    DT_OUTER = 1.0 / CAM_FPS # ~0.033s
    INNER_HZ = 150.0
    DT_INNER = 1.0 / INNER_HZ
    MAX_RUN_S = 10.0
    LINE_LOST_TIMEOUT = 1.5

    # Control - Velocity
    V_SLEW_UP = 0.25 
    V_SLEW_DOWN = 0.75 
    V_MIN = 0.01
    KV = 0.0 # Speed reduction factor when turning, might remove

    # Control - Yaw
    YAW_SLEW = 0.0
    WHEEL_OMEGA_LIMIT = 10
    OFFSET_TO_ANGLE_GAIN = 0.001
    KP_THETA = 2.6
    KD_THETA = 0.45
    U_YAW_LIMIT = 1.0
