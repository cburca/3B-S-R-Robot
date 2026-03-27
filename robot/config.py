class Config:
    # Camera
    CAM_INDEX = 0
    CAM_W = 640
    CAM_H = 480
    CAM_FPS = 30.0

    # Serial
    BAUD = 115200
    SERIAL_TIMEOUT = 0.1
    SERIAL_HANDSHAKE = True
    SERIAL_WAIT_TIMEOUT = 15.0
    SERIAL_PORT = "/dev/ttyACM0"

    # Main Loop
    LOOP_HZ = 20.0
    RUN_TIME_S = 20.0

    # Color Thresholds
    RED_LOWER1 = (0, 100, 100)
    RED_UPPER1 = (10, 255, 255)
    RED_LOWER2 = (160, 100, 100)
    RED_UPPER2 = (180, 255, 255)
    BLUE_LOWER = (100, 90, 80)
    BLUE_UPPER = (120, 255, 255)
    GREEN_LOWER = (64, 75, 61)
    GREEN_UPPER = (141, 225, 255)
    
    KERNEL_SIZE = 5
    MORPH_K = 5

    MIN_CIRCLE_AREA = 150
    MIN_BLUE_AREA = 1200
    MIN_MASK_AREA = 600

    # Image Processing
    CANNY1 = 50
    CANNY2 = 150
    HOUGH_THRESH = 30
    MIN_LINE_LEN = 35
    MAX_LINE_GAP = 8
    YREF_FRAC = 0.85
    MAX_ABS_DEG_FROM_VERTICAL = 40.0

    # Bullseye
    BULLSEYE_ANGLE_TOL_DEG = 5.0
    BULLSEYE_PICKUP_Y = 300

    # Safezone
    SAFEZONE_Y_MIN = 0.05
    SAFEZONE_Y_MAX = 0.25
    SAFEZONE_MIN_PIXELS = 300
    SAFEZONE_MAX_CENTER_PIXELS = 100

    # Debug
    DEBUG_SHOW = True
    DEBUG_DRAW = True

    # Robot Geometry
    r = 0.04
    L = 0.20
    vmax = 0.15
    wmax = vmax / r  # 6.25 rad/s is based off latency constraint, not firm

    # Encoder
    ENCODER_CPR = 2797

    # Timing
    DT_OUTER = 1.0 / LOOP_HZ
    INNER_HZ = 150.0
    DT_INNER = 1.0 / INNER_HZ
    MAX_RUN_S = 1000
    LINE_LOST_TIMEOUT = 0.90
    # State Machine
    START_DELAY_S = 1.0
    BULLSEYE_LOST_TIMEOUT = 0.4
    BULLSEYE_ALIGN_KP = 0.01
    BULLSEYE_DY_KP = 0.01
    BULLSEYE_FORWARD_ANGLE_GATE_DEG = 8.0
    BULLSEYE_CREEP_MIN = 0.00
    BULLSEYE_CREEP_MAX = 0.06
    
    # Control - Velocity
    V_SLEW_UP = 0.25
    V_SLEW_DOWN = 0.75
    V_MIN = 0.01
    KV = 0.01 # Speed reduction factor when turning, might remove
    MIN_TURN_ANGLE = 30 # if line_theta_deg > this, go at reduced speed
    TURN_SPEED = 0.1

    # Control - Yaw
    YAW_SLEW = 3.0
    WHEEL_OMEGA_LIMIT = 10
    OFFSET_TO_ANGLE_GAIN = 0.001
    KP_THETA = 2.4
    KD_THETA = 0.005
    U_YAW_LIMIT = 1.25
    
    # Control - Bullseye Pickup
    BULLSEYE_ANGLE_TOL_DEG = 5.0
    BULLSEYE_PICKUP_Y = 300
    POST_PICK_SETTLE_S = 0.0
