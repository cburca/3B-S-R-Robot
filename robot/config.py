class Config:
    CAM_INDEX = 0
    CAM_W = 640
    CAM_H = 480
    CAM_FPS = 30.0

    SERIAL_PORT = "/dev/ttyACM0"
    BAUD = 115200
    SERIAL_TIMEOUT = 0.1
    SERIAL_HANDSHAKE = True

    SERIAL_WAIT_TIMEOUT = 15.0
    SERIAL_CONNECT_RETRIES = 10
    SERIAL_RETRY_DELAY = 1.0
    ARDUINO_RESET_DELAY = 2.0

    CAM_CONNECT_RETRIES = 10
    CAM_RETRY_DELAY = 1.0

    MAIN_RETRY_DELAY = 2.0

    LOOP_HZ = 20.0
    RUN_TIME_S = 10.0

    KP_HEADING = 0.0
    KD_HEADING = 0.0
    YAW_RATE_MAX = 2.0

    V_MAX = 0.25
    V_MIN = 0.0
    KV = 0.25

    WHEEL_RADIUS_M = 0.03
    TRACK_WIDTH_M = 0.14
    ENC_CPR = 2797
    MAX_TPS = 2000.0

    SHOW_DEBUG = True

    RED_LOWER1 = (0, 100, 100)
    RED_UPPER1 = (10, 255, 255)
    RED_LOWER2 = (160, 100, 100)
    RED_UPPER2 = (180, 255, 255)

    MIN_MASK_AREA = 800
    CANNY1, CANNY2 = 50, 150
    HOUGH_THRESH = 30
    MIN_LINE_LEN = 30
    MAX_LINE_GAP = 10
    MORPH_K = 5
    YREF_FRAC = 0.85
    MAX_ABS_DEG_FROM_VERTICAL = 90.0 

    DEBUG_SHOW = True
    DEBUG_DRAW = True

    # Robot Geometry - TO BE UPDATED
    r = 0.04
    L = 0.20
    vmax = 0.05
    wmax = vmax / r  # 6.25 rad/s 

    # Timing Rates
    DT_OUTER = 1.0 / CAM_FPS # ~0.033s
    INNER_HZ = 150.0
    DT_INNER = 1.0 / INNER_HZ

    # Outer PD Gains - TO BE UPDATED
    KP_THETA = 299.30
    KD_THETA = 15.998
    U_YAW_LIMIT = 1.0 # normalized yaw actuation command

    # inner PI gains - TO BE UPDATED
    KP_W = 0.160
    KI_W = 11.97
    U_PWM_LIMIT = 1.0 # normalized PWM

    KV = 0.2  # TO BE TUNED: v_cmd = vmax (1 - KV*|yaw_cmd|)
    V_MIN = 0.01

    # encoder counts per revolution
    ENCODER_CPR = 2800

    # USB Serial Params
    SERIAL_PORT = 'dev/ttyACM0'  # update when applicable
    BAUD_RATE = 115200 # update to real value
