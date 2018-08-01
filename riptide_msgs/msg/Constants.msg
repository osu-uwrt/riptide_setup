# Colors Codes
uint8 COLOR_BLACK = 0
uint8 COLOR_RED = 1


# Safety Measure Codes
uint8 SAFETY_BAT_VOLT_LOW = 0
uint8 SAFETY_BAT_CUR_SPIKE = 1
uint8 SAFETY_CVTR_VOLT_LOW = 2
uint8 SAFETY_CVTR_CUR_SPIKE = 3

# Status Codes
uint8 STATUS_INIT = 0
uint8 STATUS_ACTIVATED = 1
uint8 STATUS_DEACTIVATED = 2
uint8 STATUS_MISSION_ATTEMPTED = 3
uint8 STATUS_MISSION_NOT_ATTEMPTED = 4
uint8 STATUS_SAFE = 5
uint8 STATUS_EMERGENCY = 6

# Alignment plane
uint8 PLANE_YZ = 0
uint8 PLANE_XY = 1

# Controls
uint8 CONTROL_BBOX_WIDTH = 0
uint8 CONTROL_BBOX_HEIGHT = 1

# Quadrants
uint8 QUAD_A = 0
uint8 QUAD_B = 1
uint8 QUAD_C = 2
uint8 QUAD_D = 3

# Mission IDs
uint8 MISSION_A_BLACK = 0
uint8 MISSION_B_BLACK = 1
uint8 MISSION_C_BLACK = 2
uint8 MISSION_D_BLACK = 3
uint8 MISSION_TEST = 4
uint8 MISSION_NONE = 255

# Competition Round IDs
uint8 COMPETITION_SEMIS = 0 # Practice and semis are the same
uint8 COMPETITION_FINALS = 1

# Task and Task Map file paths
string FILE_TASKS = ../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/tasks.yaml
string FILE_TASKS_SIM = ../osu-uwrt/riptide_software/src/riptide_vision/cfg/tasks_sim.yaml
string FILE_MAP_SEMIS = ../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/task_map_semis.yaml
string FILE_MAP_FINALS = ../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/task_map_finals.yaml

# Execute Codes
uint8 EXECUTE_STANDBY = 0
uint8 EXECUTE_TSLAM = 1
uint8 EXECUTE_MISSION = 2

# Competition Task IDs
uint8 TASK_CASINO_GATE = 0
uint8 TASK_PATH_MARKER1 = 1
uint8 TASK_DICE = 2
uint8 TASK_PATH_MARKER2 = 3
uint8 TASK_SLOTS = 4
uint8 TASK_BUY_GOLD_CHIP1 = 5
uint8 TASK_ROULETTE = 6
uint8 TASK_BUY_GOLD_CHIP2 = 7
uint8 TASK_CASH_IN = 8
