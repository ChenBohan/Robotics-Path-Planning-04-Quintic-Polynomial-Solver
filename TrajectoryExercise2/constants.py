N_SAMPLES = 10
SIGMA_S = [10.0, 4.0, 2.0] # s, s_dot, s_double_dot
SIGMA_D = [1.0, 1.0, 1.0]
SIGMA_T = 2.0

MAX_JERK = 10 # m/s/s/s
MAX_ACCEL= 10 # m/s/s

EXPECTED_JERK_IN_ONE_SEC = 2 # m/s/s
EXPECTED_ACC_IN_ONE_SEC = 1 # m/s

SPEED_LIMIT = 30
VEHICLE_RADIUS = 1.5 # model vehicle as circle to simplify collision detection