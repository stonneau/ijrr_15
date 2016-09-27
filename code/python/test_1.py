'''
To do.
'''
import sys
import os

cwd = os.getcwd()[:-7]; # current path without "python"
if cwd+'/data/config' not in sys.path:
    sys.path += [cwd+'/data/config',];

import test1_planar as conf
from standard_qp_solver import StandardQpSolver
#from multi_contact_stability_criterion_utils import can_I_stop
from simulator import Simulator
import viewer_utils
from inv_dyn_formulation_util import InvDynFormulation
from constraint_violations import ConstraintViolationType
from sot_utils import compute_nullspace_projector, qpOasesSolverMsg, solveWithNullSpace, solveLeastSquare
from sot_utils import pinocchio_2_sot
from acc_bounds_util import isStateViable
from multi_contact_stability_criterion_utils import compute_GIWC, compute_com_acceleration_polytope
from geom_utils import crossMatrix, plot_inequalities
from plot_utils import plotNdQuantity
from plot_utils import plotNdQuantityPerSolver
from plot_utils import plotQuantityPerSolver
import plot_utils
#from plot_utils import create_empty_figure

import cProfile
import pickle
import numpy as np
import numpy.matlib
from numpy.linalg import norm
import matplotlib.pyplot as plt
from datetime import datetime
#import matplotlib.pyplot as plt
from time import sleep
from math import sqrt
from tasks import SE3Task, CoMTask, PosturalTask, AngularMomentumTask
from trajectories import ConstantSE3Trajectory, Constant3dTrajectory
from sot_utils import RIGHT_FOOT_SIZES, LEFT_FOOT_SIZES

EPS = 1e-4;

def createListOfMatrices(listSize, matrixSize):
    l = listSize*[None,];
    for i in range(listSize):
        l[i] = np.matlib.zeros(matrixSize);
    return l;

def createInvDynFormUtil(q, dq):
    invDynForm = InvDynFormulation('hrp2_inv_dyn'+datetime.now().strftime('%m%d_%H%M%S'), 
                                   q, dq, conf.dt, conf.model_path, conf.urdfFileName, conf.freeFlyer);
    invDynForm.USE_COM_TRAJECTORY_GENERATOR = False;
    invDynForm.enableCapturePointLimits(conf.ENABLE_CAPTURE_POINT_LIMITS);
    invDynForm.enableTorqueLimits(conf.ENABLE_TORQUE_LIMITS);
    invDynForm.enableForceLimits(conf.ENABLE_FORCE_LIMITS);
    invDynForm.enableJointLimits(conf.ENABLE_JOINT_LIMITS, conf.IMPOSE_POSITION_BOUNDS, conf.IMPOSE_VELOCITY_BOUNDS, 
                                 conf.IMPOSE_VIABILITY_BOUNDS, conf.IMPOSE_ACCELERATION_BOUNDS);
    invDynForm.JOINT_POS_PREVIEW            = conf.JOINT_POS_PREVIEW;
    invDynForm.JOINT_VEL_PREVIEW            = conf.JOINT_VEL_PREVIEW;
    invDynForm.MAX_JOINT_ACC                = conf.MAX_JOINT_ACC;
    invDynForm.MAX_MIN_JOINT_ACC            = conf.MAX_MIN_JOINT_ACC;
    invDynForm.USE_JOINT_VELOCITY_ESTIMATOR = conf.USE_JOINT_VELOCITY_ESTIMATOR;
    invDynForm.ACCOUNT_FOR_ROTOR_INERTIAS   = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    #print invDynForm.r.model
    
    rf_id = invDynForm.getJointId("RLEG_JOINT5");
    rf_ref_traj = ConstantSE3Trajectory("rf_traj", invDynForm.r.position(q, rf_id));
    rf_constr = SE3Task(invDynForm.r, rf_id, rf_ref_traj, "right_foot");
    rf_constr.kp = conf.kc_p;
    rf_constr.kv = conf.kc_d;
    p = np.array([[ RIGHT_FOOT_SIZES[0],  RIGHT_FOOT_SIZES[2], 0],
                  [ RIGHT_FOOT_SIZES[0], -RIGHT_FOOT_SIZES[3], 0],
                  [-RIGHT_FOOT_SIZES[1], -RIGHT_FOOT_SIZES[3], 0],
                  [-RIGHT_FOOT_SIZES[1],  RIGHT_FOOT_SIZES[2], 0]]);
    N = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1]]);
    invDynForm.addUnilateralContactConstraint(rf_constr, p, N, conf.fMin, conf.mu);
    
    lf_id = invDynForm.getJointId("LLEG_JOINT5");
    lf_ref_traj = ConstantSE3Trajectory("lf_traj", invDynForm.r.position(q, lf_id));
    lf_constr = SE3Task(invDynForm.r, lf_id, lf_ref_traj, "left_foot");
    lf_constr.kp = conf.kc_p;
    lf_constr.kv = conf.kc_d;
    p = np.array([[ LEFT_FOOT_SIZES[0],  LEFT_FOOT_SIZES[2], 0],
                  [ LEFT_FOOT_SIZES[0], -LEFT_FOOT_SIZES[3], 0],
                  [-LEFT_FOOT_SIZES[1], -LEFT_FOOT_SIZES[3], 0],
                  [-LEFT_FOOT_SIZES[1],  LEFT_FOOT_SIZES[2], 0]]);
    invDynForm.addUnilateralContactConstraint(lf_constr, p, N, conf.fMin, conf.mu);
    
    com_traj = Constant3dTrajectory("com_traj", invDynForm.r.com(q)+np.matrix([[0.0, 0.1, 0.0]]).T);
    com_task = CoMTask(invDynForm.r, com_traj);
    com_task.kp = conf.kp_com;
    com_task.kv = conf.kd_com;
    invDynForm.addTask(com_task, conf.w_com);
    
    posture_task = PosturalTask(invDynForm.r);
    q_des = np.matrix.copy(q);
    q_des[11] += 2.7;
    posture_task.setPosture(q_des);
    posture_task.kp = conf.kp_posture;
    posture_task.kv = conf.kd_posture;
    invDynForm.addTask(posture_task, conf.w_posture);
    
    return invDynForm;
    
    
def createSimulator(q0, v0):
    simulator  = Simulator('hrp2_sim'+datetime.now().strftime('%Y%m%d_%H%M%S')+str(np.random.random()), 
                                   q0, v0, conf.fMin, conf.mu, conf.dt, conf.model_path, conf.urdfFileName);
    simulator.viewer.CAMERA_FOLLOW_ROBOT = False;
#    simulator.USE_LCP_SOLVER = conf.USE_LCP_SOLVER;
    simulator.ENABLE_TORQUE_LIMITS = conf.FORCE_TORQUE_LIMITS;
    simulator.ENABLE_FORCE_LIMITS = conf.ENABLE_FORCE_LIMITS;
    simulator.ENABLE_JOINT_LIMITS = conf.FORCE_JOINT_LIMITS;
    simulator.ACCOUNT_FOR_ROTOR_INERTIAS = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    simulator.verb=0;
    return simulator;
        
        
def startSimulation(q0, v0, solverId):
    j = solverId
    print '\nGONNA INTEGRATE CONTROLLER %d' % j;
    
    # make a copy of the initial state to make sure it is not modified during the simulation
    q0 = np.matrix.copy(q0);
    v0 = np.matrix.copy(v0);
    q[j][:,0]  = q0;
    dq[j][:,0] = v0;
    t = 1;
    t0 = t+1;
    
    constrViol          = np.empty(conf.MAX_TEST_DURATION).tolist(); #list of lists of lists
    constrViolString    = '';    

    simulator.reset(t, q0, v0, conf.dt);
    solvers[j].changeInequalityNumber(invDynForm.m_in);
    
    for i in range(conf.MAX_TEST_DURATION):
        t = t+1;
        
        invDynForm.setNewSensorData(t, simulator.q, simulator.v);
        (G,glb,gub,lb,ub) = invDynForm.createInequalityConstraints(t);
        m_in = glb.size;
        
        (D,d)       = invDynForm.computeCostFunction(t);

        q[j][:,i]         = np.matrix.copy(invDynForm.q);
        dq[j][:,i]        = np.matrix.copy(invDynForm.v);
#        x_com[i,j,:]      = invDynForm.x_com;       # from the solver view-point
#        dx_com[i,j,:]     = invDynForm.dx_com;      # from the solver view-point
#        cp[i,j,:]         = invDynForm.cp;          # from the solver view-point
#        ang_mom[i,j]      = norm(invDynForm.getAngularMomentum())

        if(i%10==0):
            print "Time %.3f... i %d" % ((t-t0)*conf.dt, i), "Max joint vel", np.max(np.abs(dq[j][:,i]));
        
        if(i==conf.MAX_TEST_DURATION-1):
            print "MAX TIME REACHED \n";
            print "Max joint vel", np.max(np.abs(dq[j][:,i]));
            final_time[j] = (t-t0)*conf.dt;
            final_time_step[j]    = i;
            return True;
        
        ''' tell the solvers that if the QP is unfeasible they can relax the joint-acc inequality constraints '''
        #solvers[j].setSoftInequalityIndexes(invDynForm.ind_acc_in);
        
#        D_ext = np.vstack((conf.w_com*D, conf.w_posture*D_q, conf.w_force_reg*D_f));
#        d_ext = np.hstack((conf.w_com*d, conf.w_posture*d_q, conf.w_force_reg*d_f));
            
        (tau[i,j,:],solver_imode[i,j])    = solvers[j].solve(D.A, d.A, G.A, glb.A, gub.A, lb.A, ub.A, tau[i-1,j,:], maxTime=conf.maxTime);

        torques                     = np.matrix.copy(tau[i,j,:]).reshape((na,1));
        y                           = np.dot(invDynForm.C, torques) + invDynForm.c;
        dv[j][:,i]                  = y[:nv];
#        if('c_lf' in invDynForm.rigidContactConstraints[0].name):
#            fc[i,j,:6]             = y[nv:nv+6];  # left
#            fc[i,j,6:]             = y[nv+6:nv+12]; #right
#        else:
#            fc[i,j,:6]             = y[nv+6:nv+12];
#            fc[i,j,6:]             = y[nv:nv+6];
#        zmp[i,j,:]                  = invDynForm.getZmp(fc[i,j,:6], fc[i,j,6:]);
#        ddx_com[i,j,:]              = np.dot(invDynForm.J_com, dv[i,j,:]) + invDynForm.dJcom_dq;
#        n_active_ineq[i,j]          = solvers[j].nActiveInequalities;   # from the solver view-point
#        n_violated_ineq[i,j]        = solvers[j].nViolatedInequalities; # from the solver view-point
#        ineq[i,j,:m_in]             = np.dot(G, tau[i,j,:]) - glb; # from the solver view-point
#
#        if(np.isnan(tau[i,j,:]).any() or np.isinf(tau[i,j,:]).any()):
#            no_sol_count[j] += 1;
        
        constrViol[i] = simulator.integrateAcc((t-t0)*conf.dt, conf.dt, dv[j][:,i], fc[i,j,:], tau[i,j,:], conf.PLAY_MOTION_WHILE_COMPUTING);
        
        for cv in constrViol[i]:
            cv.time = (t-t0)*conf.dt;
            print cv.toString();
            constrViolString += cv.toString()+'\n';
            
        # Check whether robot is falling
        if(np.sum(n_violated_ineq[:,j]) > 10 or norm(dx_com[i,j,:])>conf.MAX_COM_VELOCITY):
            print "Com velocity", np.linalg.norm(dx_com[i,j,:]);
            print "Solver violated %d inequalities" % solvers[j].nViolatedInequalities, "max inequality violation", np.min(ineq[i,j,:m_in]);
            print "ROBOT FELL AFTER %.1f s\n" % ((t-t0)*conf.dt);
            final_time[j] = (t-t0)*conf.dt;
            final_time_step[j] = i;
            for index in range(i+1,conf.MAX_TEST_DURATION):
                q[index,j,:] = q[i,j,:];
            return False;


''' *********************** BEGINNING OF MAIN SCRIPT *********************** '''
READ_INITIAL_STATE_FROM_FILE = '../results/test_1/20160919_170307_batch_1/20160919_232443/data.pkl';

''' Input parameter may be either an integer value that is the id of the initial joint configuration,
    or a string being the path to a pickle file containing initial joint pos (q0) and vel (v0)
'''
if(len(sys.argv)==2):
    if(isinstance(sys.argv[1], ( int, long ) )):
        conf.INITIAL_CONFIG_ID = int(sys.argv[1]);
    else:
        READ_INITIAL_STATE_FROM_FILE = str(sys.argv[1]);

    
np.set_printoptions(precision=2, suppress=True);
date_time = datetime.now().strftime('%Y%m%d_%H%M%S');
viewer_utils.ENABLE_VIEWER = conf.ENABLE_VIEWER
plot_utils.FIGURE_PATH      = '../results/test_1/'+date_time+'/'; #'_'+str(conf.SOLVER_TO_INTEGRATE).replace(' ', '_')+'/';
plot_utils.SAVE_FIGURES     = conf.SAVE_FIGURES;
plot_utils.SHOW_FIGURES     = conf.SHOW_FIGURES;
plot_utils.SHOW_LEGENDS     = conf.SHOW_LEGENDS;
plot_utils.LINE_ALPHA       = conf.LINE_ALPHA;
plot_utils.LINE_WIDTH_RED   = conf.LINE_WIDTH_RED;
plot_utils.LINE_WIDTH_MIN   = conf.LINE_WIDTH_MIN;
SHOW_CONTACT_POINTS_IN_VIEWER = True;

t = 1;
q0 = np.matrix( [
        0.0, 0.0, 0.648702, 0.0, 0.0 , 0.0, 1.0,                             # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ] ).T
v0 = np.matlib.zeros((36,1));
invDynForm = createInvDynFormUtil(q0, v0);
simulator = createSimulator(q0, v0);


(G,glb,gub,lb,ub) = invDynForm.createInequalityConstraints(t);
m_in = glb.size;
nq = invDynForm.nq;    # number of joints
nv = invDynForm.nv;    # number of joints
na = invDynForm.na;    # number of joints
k = invDynForm.k;    # number of constraints
mass = invDynForm.M[0,0];

''' Store data to compute stats about initial state later'''
B_ch = np.copy(invDynForm.B_conv_hull);
b_ch = np.copy(invDynForm.b_conv_hull);
p = np.copy(invDynForm.contact_points);
N = np.zeros((8,3)); N[:,2] = 1;     # contact normals are all [0 0 1]
# compute centroidal cone
#(H,h) = compute_GIWC(p, N, conf.mu[0]);

''' Create the controllers '''
solver_id       = StandardQpSolver(na, m_in, "qpoases", maxIter=conf.maxIter, verb=conf.verb);
solvers = [solver_id];
N_SOLVERS = len(solvers);
solver_names = [s.name for s in solvers];
    
q                    = createListOfMatrices(N_SOLVERS, (nq, conf.MAX_TEST_DURATION)); #np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, nq));
dq                   = createListOfMatrices(N_SOLVERS, (nv, conf.MAX_TEST_DURATION)); #np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, nv));
fc                   = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, k));
tau                  = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, na));
dv                   = createListOfMatrices(N_SOLVERS, (nv, conf.MAX_TEST_DURATION)); #np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, nv));
ineq                 = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, m_in));
n_active_ineq        = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
n_violated_ineq      = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
no_sol_count         = np.zeros(N_SOLVERS, dtype=np.int);
solver_imode         = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
x_com                = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, 3));
dx_com               = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, 3));
ddx_com              = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, 3));
cp                   = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, 2));   # capture point
zmp                  = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS, 2));   # zmp
ang_mom              = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS));      # angular momentum
final_time           = np.zeros(N_SOLVERS);
final_time_step      = np.zeros(N_SOLVERS, np.int);
controller_balance   = N_SOLVERS*[False,];

for s in conf.SOLVER_TO_INTEGRATE:
    controller_balance[s] = startSimulation(q0, v0, s);
#    cProfile.run('startSimulation(q0, v0, s);');

#''' compute distance of com from border in direction of capture point '''
#a = np.dot(B_ch, dx_com[0,s,:2] / norm(dx_com[0,s,:2]));
#b = np.dot(B_ch, x_com[0,s,:2]) + b_ch;
#for i in range(a.shape[0]):
#    b[i] = b[i]/abs(a[i]) if abs(a[i])>0.0 else b[i];
#print "Distance of com from support polygon border in capture point direction %.3f"%np.min(b[np.where(a<0.0)[0]])
#
#''' Compute stability criteria '''
#s = conf.SOLVER_TO_INTEGRATE[0];
#com_ineq = np.dot(B_ch, x_com[0,s,:2]) + b_ch;
#cp_ineq = np.dot(B_ch, cp[0,s,:]) + b_ch;
#
#info = "\n*************************************** RESULTS ********************************************\n"
#info += "Configuration %d:"%conf.INITIAL_CONFIG_ID + "\n";
#in_or_out = "outside" if (com_ineq<0.0).any() else "inside";
#info += "initial com pos "+str(x_com[0,s,:])+" is "+in_or_out+" support polygon, margin: %.3f\n"%(np.min(com_ineq));
#info += "initial com vel "+str(dx_com[0,s,:])+" (norm %.3f)"%np.linalg.norm(dx_com[0,s,:])+"\n";
#cp_out = (cp_ineq<0.0).any();
#in_or_out = "outside" if cp_out else "inside";
#info +="initial capture point "+str(cp[0,s,:])+" is "+in_or_out+" support polygon, margin: %.3f\n"%np.min(cp_ineq);
#info += "Initial max joint vel %.3f\n"%(np.max(np.abs(dq[0,s,:])));
#for j in conf.SOLVER_TO_INTEGRATE:
#    info += "Final com pos solver %d: "%j + str(x_com[final_time_step[j],j,:]) + "\n";
#    info += "Final com vel solver %d: "%j + str(dx_com[final_time_step[j],j,:]);
#    info += " (norm %.3f)\n" % norm(dx_com[final_time_step[j],j,:]);
#    cp_ineq = np.min(np.dot(B_ch, cp[final_time_step[j],j,:]) + b_ch);
#    in_or_out = "outside" if (cp_ineq<0.0) else "inside";
#    info += "Final capture point solver %d "%j +str(cp[final_time_step[j],j,:])+" is "+in_or_out+" support polygon %.3f\n"%(cp_ineq);
#    info += "Final max joint vel solver %d: %.3f\n"%(j, np.max(np.abs(dq[final_time_step[j],j,:])));
#info += "***********************************************************************************\n"
#print info
#
#imax = np.max(final_time_step);
#
#if(conf.PLAY_REAL_TIME_MOTION):
#    raw_input("Press Enter to play motion in real time...");    
#    for s in conf.SOLVER_TO_INTEGRATE:
#        simulator.viewer.play(q[:final_time_step[s],s,:], conf.dt);
#
#time = conf.dt*np.array(range(conf.MAX_TEST_DURATION));
#
#if(SHOW_CONTACT_POINTS_IN_VIEWER):
#    q0[2] -= 0.005
#    simulator.viewer.updateRobotConfig(q0);
#    p[:,2] -= 0.005;
#    for j in range(p.shape[0]):
#        simulator.viewer.addSphere("contact_point"+str(j), 0.005, p[j,:], (0,0,0), (1, 1, 1, 1));
#        simulator.viewer.updateObjectConfigRpy("contact_point"+str(j), p[j,:]); 
#
#
#if(plot_utils.SHOW_FIGURES or plot_utils.SAVE_FIGURES):
#    line_styles     =["k-", "b--", "r-", "c-", "g-"];
#    line_styles_no_marker =["k x", "b x", "r x", "c x", "g x"];
#
#    if(np.sum(n_violated_ineq)>0):
#        plotQuantityPerSolver(n_violated_ineq[:imax], 'Number of violated inequalities', solver_names, line_styles);
#    
#    f, ax = plot_utils.create_empty_figure();
#    for s in range(N_SOLVERS):
#        ax.plot(dx_com[:final_time_step[s],s,0], dx_com[:final_time_step[s],s,1], line_styles[s]);
#    ax.set_ylabel('com vel y');
#    ax.set_xlabel('com vel x');
#
#    f, ax = plot_utils.create_empty_figure();
#    for j in range(p.shape[0]):
#        ax.scatter(p[j,0], p[j,1], c='k', s=100);
#    for s in range(N_SOLVERS):
#        ax.plot(x_com[:final_time_step[s],s,0], x_com[:final_time_step[s],s,1], line_styles[s]);
#    ax.set_ylabel('com pos y');
#    ax.set_xlabel('com pos x');
#    
#    f, ax = plot_utils.create_empty_figure();
#    for j in range(p.shape[0]):
#        ax.scatter(p[j,0], p[j,1], c='k', s=100);
#    for s in range(N_SOLVERS):
#        ax.plot(cp[:final_time_step[s],s,0], cp[:final_time_step[s],s,1], line_styles[s]);
#    ax.set_ylabel('cp y');
#    ax.set_xlabel('cp x');
#    
#    f, ax = plot_utils.create_empty_figure();
#    for j in range(p.shape[0]):
#        ax.scatter(p[j,0], p[j,1], c='k', s=100);
#    for s in range(N_SOLVERS):
#        ax.plot(zmp[:final_time_step[s],s,0], zmp[:final_time_step[s],s,1], line_styles_no_marker[s]);
#    ax.set_ylabel('zmp y');
#    ax.set_xlabel('zmp x');
#        
##    ax = plotQuantityPerSolver(ang_mom[:imax,:], 'Angular momentum', solver_names, line_styles);
#    ax = plotNdQuantityPerSolver(3, 1, x_com[:imax,:,:], 'Center of mass', solver_names, line_styles);
#    ax = plotNdQuantityPerSolver(3, 1, dx_com[:imax,:,:], 'Center of mass velocity', solver_names, line_styles);
##    plotNdQuantityPerSolver(2, 1, cp,  "Capture point",  solver_names, line_styles, boundUp=invDynForm.cp_max, boundLow=invDynForm.cp_min);
#        
#    dx_com_int = np.zeros((imax,N_SOLVERS,3));
#    dx_com_int[0,:,:] = dx_com[0,:,:];
#    for s in conf.SOLVER_TO_INTEGRATE:
#        for i in range(1,imax):
#            dx_com_int[i,s,:] = dx_com_int[i-1,s,:] + conf.dt*ddx_com[i-1,s,:];
#        ax = plotNdQuantity(3, 1, dx_com[:imax,s,:], 'COM vel real(k) VS integrated'+str(s), linestyle='k--');
#        ax = plotNdQuantity(3, 1, dx_com_int[:,s,:], 'COM vel real(k) VS integrated(r)'+str(s), linestyle='r--', ax=ax);
#            
#    if(plot_utils.SHOW_FIGURES):
#        plt.show();
