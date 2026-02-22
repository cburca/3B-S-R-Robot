class Config:
    # Camera params
    CAM_INDEX = 0
    CAM_W = 640
    CAM_H = 480
    CAM_FPS = 30.0
    VISION_W = 480
    VISION_H = 480

    # Vision Thresholds
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

    DEBUG_SHOW = False
    DEBUG_DRAW = True

    # Robot Geometry - TO BE UPDATED
    r = 0.04
    L = 0.20
    vmax = 0.25
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

    KV = 0.6  # tune: v_cmd = vmax (1 - KV*|yaw_cmd|)  (your report form) :contentReference[oaicite:8]{index=8}
    V_MIN = 0.05

    # encoder counts per revolution
    ENCODER_CPR = 360 

    # I2C Communication
    I2C_BUS = 1
    ARDUINO_ADDR = 0x08
    REG_PWM = 0x10
    REG_ENC = 0x20

    # PWM command scaling to int16 sent to Arduino - CHANGE BASED ON ENCODER COUNTS PER REVOLUTION AND DESIRED SPEED RANGE
    CMD_SCALE = 1000