from math import sqrt
import numpy as np

''' *********************** USER-PARAMETERS *********************** '''
SOLVER_ID       = 0;    # classic TSID formulation
SOLVER_TO_INTEGRATE         = [SOLVER_ID];
DATA_FILE_NAME              = 'data';
TEXT_FILE_NAME              = 'results.txt';
SAVE_DATA                   = True;

MAX_TEST_DURATION           = 1000;
dt                          = 2e-3;
model_path   = "/home/adelpret/devel/sot_hydro/install/share";
urdfFileName = model_path + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf"
freeFlyer = True;

''' CONTROLLER CONFIGURATION '''
ENABLE_CAPTURE_POINT_LIMITS     = False;
ENABLE_TORQUE_LIMITS            = True;
ENABLE_FORCE_LIMITS             = True;
ENABLE_JOINT_LIMITS             = True;
IMPOSE_POSITION_BOUNDS          = True;
IMPOSE_VELOCITY_BOUNDS          = True;
IMPOSE_VIABILITY_BOUNDS         = True;
IMPOSE_ACCELERATION_BOUNDS      = True;
JOINT_POS_PREVIEW               = 1.5; # preview window to convert joint pos limits into joint acc limits
JOINT_VEL_PREVIEW               = 1;   # preview window to convert joint vel limits into joint acc limits
MAX_JOINT_ACC                   = 10.0;
MAX_MIN_JOINT_ACC               = 10.0;
USE_JOINT_VELOCITY_ESTIMATOR    = False;
ACCOUNT_FOR_ROTOR_INERTIAS      = True;

# CONTROLLER GAINS
kp_posture  = 30; #1.0;   # proportional gain of postural task
kd_posture  = 2*sqrt(kp_posture);
kc_p        = 1000.0;   # constraint proportional feedback gain
kc_d        = 2*sqrt(kc_p);   # constraint derivative feedback gain
kp_com      = 10;
kd_com      = 2*sqrt(kp_com);
x_com_des   = np.array([ 0.01,  0.  ,  0.78]);

# CONTROLLER WEIGTHS
w_com           = 1;
w_posture       = 1e-3;  #1e-5;  # weight of postural task
w_force_reg     = 1e-5; # weight of force regularization task
w_foot          = np.array([1, 1, 1e-3, 2, 2, 2]); # weights of force components in force regularization task
w_f             = np.append(w_foot, w_foot);

# QP SOLVER PARAMETERS
maxIter = 300;      # max number of iterations
maxTime = 0.8;      # max computation time for the solver in seconds
verb=0;             # verbosity level (0, 1, or 2)

# CONTACT PARAMETERS
mu  = np.array([0.3, 0.1]);          # force and moment friction coefficient
fMin = 1e-3;					     # minimum normal force

# SIMULATOR PARAMETERS
FORCE_TORQUE_LIMITS            = False;
FORCE_JOINT_LIMITS             = True;
USE_LCP_SOLVER = False

''' STOPPING CRITERIA THRESHOLDS '''
ZERO_JOINT_VEL_THR          = 1e-2;
ZERO_COM_VEL_THR            = 1e-3;
ZERO_ANG_MOM_THR            = 1e-2;
MAX_COM_VELOCITY            = 5;

''' INITIAL STATE PARAMETERS '''
INITIAL_CONFIG_ID               = 0;
MAX_INITIAL_JOINT_VEL		    = 1.0;
ZERO_INITIAL_VERTICAL_COM_VEL   = True;
ZERO_INITIAL_ANGULAR_MOMENTUM   = True;
INITIAL_CONFIG_FILENAME         = '../data/20160906_hrp2_coplanar';
#Q_INITIAL = np.array([ 0.  ,  0.  ,  0.62,  0.  ,  0.  ,  0.  ,  0.  ,  0.  , -0.15,
#                        0.87, -0.72,  0.  ,  0.  ,  0.  , -0.75,  0.87, -0.12,  0.  ,
#                        0.  ,  0.  ,  0.  ,  0.  ,  0.26, -0.17,  0.  , -0.52,  0.  ,
#                        0.  ,  0.1 ,  0.26,  0.17,  0.  , -0.52,  0.  ,  0.  ,  0.1 ]);

''' VIEWER PARAMETERS '''
ENABLE_VIEWER               = True;
PLAY_MOTION_WHILE_COMPUTING = True;
PLAY_REAL_TIME_MOTION       = False;
DT_VIEWER                   = dt;   # timestep used to display motion with viewer

''' FIGURE PARAMETERS '''
SAVE_FIGURES     = False;
SHOW_FIGURES     = False;
SHOW_LEGENDS     = True;
LINE_ALPHA       = 0.7;
LINE_WIDTH_RED   = 2;
LINE_WIDTH_MIN   = 2;
BUTTON_PRESS_TIMEOUT        = 100.0;
