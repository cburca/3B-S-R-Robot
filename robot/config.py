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
    RED_LOWER1 = (0, 90, 90)
    RED_UPPER1 = (20, 255, 255)
    RED_LOWER2 = (160, 90, 90)
    RED_UPPER2 = (180, 255, 255)
    BLUE_LOWER = (100, 90, 80)
    BLUE_UPPER = (120, 255, 255)
    GREEN_LOWER = (40, 70, 70)
    GREEN_UPPER = (85, 255, 255)
    
    KERNEL_SIZE = 5
    MIN_CIRCLE_AREA = 100
    MIN_BLUE_AREA = 1000

    # Image Processing
    MIN_MASK_AREA = 400
    CANNY1, CANNY2 = 50, 150
    HOUGH_THRESH = 30
    MIN_LINE_LEN = 25
    MAX_LINE_GAP = 10
    MORPH_K = 5
    YREF_FRAC = 0.85
    MAX_ABS_DEG_FROM_VERTICAL = 60.0
    BULLSEYE_ANGLE_TOL_DEG = 10.0
    BULLSEYE_PICKUP_Y = 340

    # Safezone stuff
    SAFEZONE_Y_MIN = 0.05      # top of image
    SAFEZONE_Y_MAX = 0.25      # thickness of detection band

    SAFEZONE_MIN_PIXELS = 300   # left/right threshold
    SAFEZONE_MAX_CENTER_PIXELS = 100  # must stay low (no green in center)

    # Debug
    DEBUG_SHOW = True
    DEBUG_DRAW = True

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
    MAX_RUN_S = 1000
    LINE_LOST_TIMEOUT = 1.5
    
    # Control - State Machine
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
    KV = 0.0 # Speed reduction factor when turning, might remove

    # Control - Yaw
    YAW_SLEW = 0.0
    WHEEL_OMEGA_LIMIT = 10
    OFFSET_TO_ANGLE_GAIN = 0.001
    KP_THETA = 2.2
    KD_THETA = 0.005
    U_YAW_LIMIT = 1.0
    
    # Control - Bullseye Pickup
    BULLSEYE_ANGLE_TOL_DEG = 5.0
    BULLSEYE_PICKUP_Y = 300
    POST_PICK_SETTLE_S = 2.0
