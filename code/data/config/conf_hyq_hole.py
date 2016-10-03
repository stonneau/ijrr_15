from math import sqrt
import numpy as np

''' *********************** USER-PARAMETERS *********************** '''
SOLVER_ID       = 0;    # classic TSID formulation
SOLVER_TO_INTEGRATE         = [SOLVER_ID];
DATA_FILE_NAME              = 'data';
TEXT_FILE_NAME              = 'results.txt';
SAVE_DATA                   = True;

''' INITIAL STATE PARAMETERS '''
SMOOTH_FILTER_WINDOW_LENGTH = 101;
MAX_TEST_DURATION           = 2*1000;
dt                          = 1e-3;
INPUT_FILE_NAME             = '../data/hyq/hole_hyq_t_var_04f_andrea';
model_path                  = ["/home/adelpret/repos/20151101_contact_planner_steve/code/data/hyq"];
urdfFileName                = model_path[0] + "/hyq_description/urdf/hyq.urdf";
freeFlyer                   = True;

''' CONTROLLER CONFIGURATION '''
ENABLE_CAPTURE_POINT_LIMITS     = False;
ENABLE_TORQUE_LIMITS            = True;
ENABLE_FORCE_LIMITS             = True;
ENABLE_JOINT_LIMITS             = False;
IMPOSE_POSITION_BOUNDS          = False;
IMPOSE_VELOCITY_BOUNDS          = False;
IMPOSE_VIABILITY_BOUNDS         = False;
IMPOSE_ACCELERATION_BOUNDS      = True;
JOINT_POS_PREVIEW               = 1.5; # preview window to convert joint pos limits into joint acc limits
JOINT_VEL_PREVIEW               = 1;   # preview window to convert joint vel limits into joint acc limits
MAX_JOINT_ACC                   = 20.0;
MAX_MIN_JOINT_ACC               = 20.0;
USE_JOINT_VELOCITY_ESTIMATOR    = False;
ACCOUNT_FOR_ROTOR_INERTIAS      = True;

# CONTROLLER GAINS
kp_posture  = 30.0; #1.0;   # proportional gain of postural task
kd_posture  = 2*sqrt(kp_posture);
kp_constr   = 200.0;   # constraint proportional feedback gain
kd_constr   = 2*sqrt(kp_constr);   # constraint derivative feedback gain
kp_com      = 30.0;
kd_com      = 2*sqrt(kp_com);
kp_ee       = 10.0;
kd_ee       = 2*sqrt(kp_ee);
constraint_mask = np.array([True, True, True, False, False, False]).T;
ee_mask         = np.array([True, True, True, False, False, False]).T;

# CONTROLLER WEIGTHS
w_com           = 1;
w_posture       = 1e-1;  #1e-5;  # weight of postural task

# QP SOLVER PARAMETERS
maxIter = 300;      # max number of iterations
maxTime = 0.8;      # max computation time for the solver in seconds
verb=0;             # verbosity level (0, 1, or 2)

# CONTACT PARAMETERS
USE_INPUT_CONTACT_POINTS = False;
DEFAULT_CONTACT_POINTS  = np.matrix([0, 0, 0]).T    # contact points in local reference frame
DEFAULT_CONTACT_NORMALS = np.matrix([0, 0, 1]).T    # contact normals in local reference frame
mu  = np.array([0.4, 0.1]);          # force and moment friction coefficient
fMin = 0.0;					     # minimum normal force

# SIMULATOR PARAMETERS
FORCE_TORQUE_LIMITS            = False;
FORCE_JOINT_LIMITS             = True;
USE_LCP_SOLVER                 = False

''' STOPPING CRITERIA THRESHOLDS '''
MAX_COM_VELOCITY            = 5;

''' VIEWER PARAMETERS '''
ENABLE_VIEWER               = True;
PLAY_MOTION_WHILE_COMPUTING = True;
PLAY_REFERENCE_MOTION       = True;
PLAY_MOTION_AT_THE_END      = True;
DT_VIEWER                   = 10*dt;   # timestep used to display motion with viewer

''' FIGURE PARAMETERS '''
SHOW_FIGURES     = False;
PLOT_JOINT_TRAJ  = False;
PLOT_COM_TRAJ    = True;
PLOT_EE_TRAJ     = False;
SAVE_FIGURES     = False;
SHOW_LEGENDS     = True;
LINE_ALPHA       = 0.7;
LINE_WIDTH_RED   = 2;
LINE_WIDTH_MIN   = 2;
BUTTON_PRESS_TIMEOUT        = 100.0;
