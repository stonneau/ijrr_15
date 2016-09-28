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
#from sot_utils import compute_nullspace_projector, qpOasesSolverMsg, solveWithNullSpace, solveLeastSquare
#from sot_utils import pinocchio_2_sot
#from acc_bounds_util import isStateViable
#from multi_contact_stability_criterion_utils import compute_GIWC, compute_com_acceleration_polytope
#from geom_utils import crossMatrix, plot_inequalities
#from plot_utils import plotNdQuantity
#from plot_utils import plotNdQuantityPerSolver
#from plot_utils import plotQuantityPerSolver
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
from tasks import SE3Task, CoMTask, JointPostureTask
from trajectories import ConstantSE3Trajectory, ConstantNdTrajectory, SmoothedNdTrajectory
#from sot_utils import RIGHT_FOOT_SIZES, LEFT_FOOT_SIZES

EPS = 1e-4;

def createListOfMatrices(listSize, matrixSize):
    l = listSize*[None,];
    for i in range(listSize):
        l[i] = np.matlib.zeros(matrixSize);
    return l;

def createInvDynFormUtil(q, v):
    invDynForm = InvDynFormulation('hrp2_inv_dyn'+datetime.now().strftime('%m%d_%H%M%S'), 
                                   q, v, conf.dt, conf.model_path, conf.urdfFileName, conf.freeFlyer);
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
    
    return invDynForm;
    
def updateConstraints(invDynForm, contactJointNames, P, N):
    for active_constr in invDynForm.rigidContactConstraints:
        if(active_constr.name not in contactJointNames):
            invDynForm.removeUnilateralContactConstraint(active_constr.name);
            print "Removing constraint", active_constr.name;

    for (i,name) in enumerate(contactJointNames):
        if(invDynForm.existUnilateralContactConstraint(name)):
            continue;
#        if("ARM" in name):
#            continue;
        jid = invDynForm.getJointId(name);
        oMi = invDynForm.r.position(invDynForm.q, jid);
        ref_traj = ConstantSE3Trajectory(name, oMi);
        constr = SE3Task(invDynForm.r, jid, ref_traj, name);
        constr.kp = conf.kc_p;
        constr.kv = conf.kc_d;

        print "Adding constraint", name, invDynForm.r.velocity(invDynForm.q, invDynForm.v, jid);        
        
        Pi = P[:,i*4:i*4+4];
        Ni = N[:,i*4:i*4+4];
        for j in range(4):
            Pi[:,j] = oMi.actInv_point(Pi[:,j]);
            Ni[:,j] = oMi.rotation.T * Ni[:,j];
        invDynForm.addUnilateralContactConstraint(constr, Pi, Ni, conf.fMin, conf.mu);
    
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
    q[j][:,0] = q0;
    v[j][:,0] = v0;
    
    constrViol          = np.empty(conf.MAX_TEST_DURATION).tolist(); #list of lists of lists
    constrViolString    = '';
    torques = np.zeros(na);

    t = 0;
    simulator.reset(t, q0, v0, conf.dt);
    for i in range(conf.MAX_TEST_DURATION):        
        updateConstraints(invDynForm, data[i]['contacts'], np.matrix(data[i]['P']).T, np.matrix(data[i]['N']).T);
        invDynForm.setNewSensorData(t, simulator.q, simulator.v);
        (G,glb,gub,lb,ub) = invDynForm.createInequalityConstraints();
        m_in = glb.size;
        
        (D,d)       = invDynForm.computeCostFunction(t);

        q[j][:,i]         = np.matrix.copy(invDynForm.q);
        v[j][:,i]         = np.matrix.copy(invDynForm.v);
        x_com[j][:,i]     = np.matrix.copy(invDynForm.x_com);       # from the solver view-point
        dx_com[j][:,i]    = np.matrix.copy(invDynForm.dx_com);      # from the solver view-point

        if(i%100==0):
            print "Time %.3f... i %d" % (t, i), "Max joint vel", np.max(np.abs(v[j][:,i]));
        
        if(i==conf.MAX_TEST_DURATION-1):
            print "MAX TIME REACHED \n";
            print "Max joint vel", np.max(np.abs(v[j][:,i]));
            final_time[j]       = t;
            final_time_step[j]  = i;
            return True;
        
        ''' tell the solvers that if the QP is unfeasible they can relax the joint-acc inequality constraints '''
        solvers[j].setSoftInequalityIndexes(invDynForm.ind_acc_in);
        solvers[j].changeInequalityNumber(m_in);
        (torques, solver_imode[i,j])    = solvers[j].solve(D.A, d.A, G.A, glb.A, gub.A, lb.A, ub.A, torques, maxTime=conf.maxTime);

        tau[j][:,i]                 = np.matrix.copy(torques).reshape((na,1));
        y                           = invDynForm.C * tau[j][:,i] + invDynForm.c;
        dv[j][:,i]                  = y[:nv];
#        if('c_lf' in invDynForm.rigidContactConstraints[0].name):
#            fc[i,j,:6]             = y[nv:nv+6];  # left
#            fc[i,j,6:]             = y[nv+6:nv+12]; #right
#        else:
#            fc[i,j,:6]             = y[nv+6:nv+12];
#            fc[i,j,6:]             = y[nv:nv+6];
        ddx_com[j][:,i]             = invDynForm.J_com * dv[j][:,i] + invDynForm.dJcom_dq;
        n_active_ineq[i,j]          = solvers[j].nActiveInequalities;   # from the solver view-point
        n_violated_ineq[i,j]        = solvers[j].nViolatedInequalities; # from the solver view-point
#        ineq[i,j,:m_in]             = np.dot(G, tau[i,j,:]) - glb; # from the solver view-point

        if(np.isnan(torques).any() or np.isinf(torques).any()):
            no_sol_count[j] += 1;
            
        cost = norm(D*tau[j][:,i]-d);
        if(cost>1e-1):
            print "Cost ", cost
        
        constrViol[i] = simulator.integrateAcc(t, dt, dv[j][:,i], fc[j][:,i], tau[j][:,i], conf.PLAY_MOTION_WHILE_COMPUTING);
        
        for cv in constrViol[i]:
            cv.time = t;
            print cv.toString();
            constrViolString += cv.toString()+'\n';
            
        # Check whether robot is falling
        if(np.sum(n_violated_ineq[:,j]) > 10 or norm(dx_com[j][:,i])>conf.MAX_COM_VELOCITY):
            print "Com velocity", np.linalg.norm(dx_com[j][:,i]);
            print "Solver violated %d inequalities" % solvers[j].nViolatedInequalities, "max inequality violation", np.min(ineq[i,j,:m_in]);
            print "ROBOT FELL AFTER %.1f s\n" % (t);
            final_time[j] = t;
            final_time_step[j] = i;
            for index in range(i+1,conf.MAX_TEST_DURATION):
                q[j][:,index] = q[j][:,i];
            return False;
        t += dt;
        
    return True;


''' *********************** BEGINNING OF MAIN SCRIPT *********************** '''
    
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

''' LOAD INPUT DATA '''
f = open(conf.INPUT_FILE_NAME, 'rb');
data = pickle.load(f);
if(len(data)>conf.MAX_TEST_DURATION):
    data = data[:conf.MAX_TEST_DURATION];
T = len(data);

q0 = np.matrix(data[0]['q']).T;
nq = q0.shape[0];
nv = nq-1 if conf.freeFlyer else nq;
v0 = np.matlib.zeros((nv,1));
invDynForm = createInvDynFormUtil(q0, v0);
updateConstraints(invDynForm, data[0]['contacts'], np.matrix(data[0]['P']).T, np.matrix(data[0]['N']).T);
simulator = createSimulator(q0, v0);
robot = invDynForm.r;
dt = conf.dt;

q_ref = np.matlib.empty((nq,T));
com_ref = np.matlib.empty((3,T));
ee_names = ['LARM_JOINT5'];
ee_indexes = [robot.model.getJointId(e) for e in ee_names];
ee_ref = createListOfMatrices(len(ee_names), (3, conf.MAX_TEST_DURATION));
for t in range(T):
    q_ref[:,t]   = np.matrix(data[t]['q']).T;
    com_ref[:,t] = robot.com(q_ref[:,t]);
    for (i,ee) in enumerate(ee_indexes):
        ee_ref[i][:,t] = robot.position(q_ref[:,t], ee).translation;

for (i,ee) in enumerate(ee_indexes):
    f, ax = plot_utils.create_empty_figure(3, 1);
    ax[0].plot(ee_ref[i][0, :].A.squeeze());
    ax[1].plot(ee_ref[i][1, :].A.squeeze());
    ax[2].plot(ee_ref[i][2, :].A.squeeze());
    ax[0].set_title("ee "+str(i));
    plt.show();
    
posture_traj = SmoothedNdTrajectory("posture_traj", q_ref[7:,:], conf.dt, conf.SMOOTH_FILTER_WINDOW_LENGTH);
posture_task = JointPostureTask(invDynForm.r, posture_traj);
posture_task.kp = conf.kp_posture;
posture_task.kv = conf.kd_posture;
invDynForm.addTask(posture_task, conf.w_posture);

com_traj  = SmoothedNdTrajectory("com_traj", com_ref, conf.dt, conf.SMOOTH_FILTER_WINDOW_LENGTH);
com_task = CoMTask(invDynForm.r, com_traj);
com_task.kp = conf.kp_com;
com_task.kv = conf.kd_com;
invDynForm.addTask(com_task, conf.w_com);

if(conf.PLAY_REFERENCE_MOTION):
    print "Gonna play reference motion";
    sleep(1);
    simulator.viewer.play(q_ref, dt, 1.0);
    print "Reference motion finished";
    sleep(1);
    

if(conf.SHOW_FIGURES):
    for j in range(3):
        f, ax = plot_utils.create_empty_figure(3, 1);
        ax[0].plot(com_ref[j, :].A.squeeze(), 'r--');
        ax[0].plot(com_traj._x_ref[j, :].A.squeeze());
        ax[1].plot(com_traj._v_ref[j, :].A.squeeze());
        ax[2].plot(com_traj._a_ref[j, :].A.squeeze());
        ax[0].set_title("Com " + str(j));
        plt.show();

    for j in range(nq-7):
        f, ax = plot_utils.create_empty_figure(3,1);
        ax[0].plot(q_ref[7+j,:].A.squeeze(), 'r--');
        ax[0].plot(posture_traj._x_ref[j,:].A.squeeze());
        ax[1].plot(posture_traj._v_ref[j,:].A.squeeze());
        ax[2].plot(posture_traj._a_ref[j,:].A.squeeze());
        ax[0].set_title("Joint "+str(j));
        plt.show();

(G,glb,gub,lb,ub) = invDynForm.createInequalityConstraints();
m_in = glb.size;
na = invDynForm.na;    # number of joints
k = invDynForm.k;    # number of constraints
mass = invDynForm.M[0,0];

''' Create the qp solver '''
solver_id       = StandardQpSolver(na, m_in, "qpoases", maxIter=conf.maxIter, verb=conf.verb);
solvers = [solver_id];
N_SOLVERS = len(solvers);
solver_names = [s.name for s in solvers];
    
q                    = createListOfMatrices(N_SOLVERS, (nq, conf.MAX_TEST_DURATION));
v                    = createListOfMatrices(N_SOLVERS, (nv, conf.MAX_TEST_DURATION));
fc                   = createListOfMatrices(N_SOLVERS, (k, conf.MAX_TEST_DURATION));
tau                  = createListOfMatrices(N_SOLVERS, (na, conf.MAX_TEST_DURATION));
dv                   = createListOfMatrices(N_SOLVERS, (nv, conf.MAX_TEST_DURATION));
ineq                 = createListOfMatrices(N_SOLVERS, (m_in, conf.MAX_TEST_DURATION));
x_com                = createListOfMatrices(N_SOLVERS, (3, conf.MAX_TEST_DURATION));
dx_com               = createListOfMatrices(N_SOLVERS, (3, conf.MAX_TEST_DURATION));
ddx_com              = createListOfMatrices(N_SOLVERS, (3, conf.MAX_TEST_DURATION));
n_active_ineq        = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
n_violated_ineq      = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
no_sol_count         = np.zeros(N_SOLVERS, dtype=np.int);
solver_imode         = np.zeros((conf.MAX_TEST_DURATION, N_SOLVERS), dtype=np.int);
final_time           = np.zeros(N_SOLVERS);
final_time_step      = np.zeros(N_SOLVERS, np.int);
controller_balance   = N_SOLVERS*[False,];

for s in conf.SOLVER_TO_INTEGRATE:
    controller_balance[s] = startSimulation(q0, v0, s);
#    cProfile.run('startSimulation(q0, v0, s);');

if(conf.PLAY_MOTION_AT_THE_END):
    print "Gonna play computed motion";
    sleep(1);
    simulator.viewer.play(q[s], dt, 1.0);
    print "Computed motion finished";

#off = 0;
#f, ax = plot_utils.create_empty_figure(3, 1);
#for j in range(3):    
#    ax[j].plot(posture_traj._x_ref[off+j, :].A.squeeze(), 'r--');
#    ax[j].plot(q[s][off+7+j, :].A.squeeze(), 'b:');
#    ax[j].set_title("Joint " + str(j));
#    
#f, ax = plot_utils.create_empty_figure(3, 1);
#for j in range(3):    
#    ax[j].plot(posture_traj._v_ref[off+j, :].A.squeeze(), 'r--');
#    ax[j].plot(v[s][off+6+j, :].A.squeeze(), 'b:');
#    ax[j].set_title("Joint vel " + str(j));
#    
#f, ax = plot_utils.create_empty_figure(3, 1);
#for j in range(3):    
#    ax[j].plot(posture_traj._a_ref[off+j, :].A.squeeze(), 'r--');
#    ax[j].plot(dv[s][off+6+j, :].A.squeeze(), 'b:');
#    ax[j].set_title("Joint acc " + str(j));
#    
#plt.show();
    
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
