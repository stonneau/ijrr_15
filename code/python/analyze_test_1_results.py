# -*- coding: utf-8 -*-
"""
Created on Tue Oct  6 13:36:34 2015

@author: adelpret
"""
import numpy as np
import plot_utils
from plot_utils import plotQuantityPerSolver
#from constraint_violations import ConstraintViolationType
import matplotlib.pyplot as plt
import os
import pickle
#from viewer_utils import Viewer
#import viewer_utils
from time import sleep
import math

            
np.set_printoptions(precision=2, suppress=True);
DATA_PATH = '../results/test_1/20160919_170307_batch_1/';

print 'Analyze data in folder', DATA_PATH;

DATA_FILE_NAME              = 'data';
REPEAT_TESTS_WHERE_CAPTURE_POINT_DISAGREES_WITH_STABILITY_CRITERION = True;
SAVE_RESULTS                = False;
PLAY_SALIENT_MOMENTS        = False;
DUMP_IMAGE_FILES            = False;
VERBOSE                     = False;
N_DIR = 0;
if(N_DIR<=0):
    N_DIR = len(os.listdir(DATA_PATH));
N_SOLVERS = 2;
TEXT_FILE_NAME = 'stats.txt';
dt = 0.002;
MAX_TIME = 4.0-dt;
SALIENT_MOMENT_DURATION = MAX_TIME;
SALIENT_MOMENT_SLOWDOWN_FACTOR = 1.0;

line_styles     =["k-", "b--", "r:", "c-", "g-"];
solver_names    = ['ID','CP'];

time_to_fall        = np.zeros((N_DIR,N_SOLVERS));
i_to_fall           = np.zeros((N_DIR,N_SOLVERS), np.int);
scores = np.zeros((N_DIR,N_SOLVERS), np.int);
falls = np.zeros(N_SOLVERS, np.int);
if(PLAY_SALIENT_MOMENTS):
    viewer_utils.ENABLE_VIEWER = True;
    viewer = Viewer('viewer');
    viewer.CAMERA_FOLLOW_ROBOT = False;
#    viewer.activateMouseCamera();

i = 0;
info_tail = '';
capture_point_true = 0;
multi_contact_true = 0;
controller_true = np.zeros(N_SOLVERS, np.int);
capture_point_multi_contact_disagree = 0;
counter_tests_per_config = np.zeros(1000, np.int);
MAX_INITIAL_CONFIG_ID = 0;
for dirname in os.listdir(DATA_PATH):
    path = DATA_PATH+dirname+'/';
    if(not os.path.exists(path)):
        continue;
    if(VERBOSE):
        print "Open", path,
    f = open(path+DATA_FILE_NAME+'.pkl', 'r');
    data = pickle.load(f); 
    if(VERBOSE):
        if('INITIAL_CONFIG_ID' in data.keys()):
            print "Ïnitial config %3d"% (data['INITIAL_CONFIG_ID']),
        else:
            print "Initial config ???",
        print "Capture point", data['capture_point_stability'],
        print "Multi-contact", data['multi_contact_stability'],
        print "Control", data['controller_balance'];
    
    if('INITIAL_CONFIG_ID' in data.keys()):
        counter_tests_per_config[data['INITIAL_CONFIG_ID']] += 1;
        if(data['INITIAL_CONFIG_ID']>MAX_INITIAL_CONFIG_ID):
            MAX_INITIAL_CONFIG_ID = data['INITIAL_CONFIG_ID'];
            
    if(data['capture_point_stability']):
        capture_point_true += 1;
    if(data['multi_contact_stability']):
        multi_contact_true += 1;
    for s in range(N_SOLVERS):
        if(data['controller_balance'][s]):
            controller_true[s] += 1;
    if(data['capture_point_stability']!=data['multi_contact_stability']):
        capture_point_multi_contact_disagree += 1;
        print "WARNING: %s: Config %d, capture point and multi-contact stability criterion disagreed"%(path,data['INITIAL_CONFIG_ID']);
        if(REPEAT_TESTS_WHERE_CAPTURE_POINT_DISAGREES_WITH_STABILITY_CRITERION):
            os.system("python test_1.py %s" % path+DATA_FILE_NAME+'.pkl');
        
    if(PLAY_SALIENT_MOMENTS):
        for s in range(N_SOLVERS):
            end = data['i_to_fall'][s];
            if(DUMP_IMAGE_FILES or time_to_fall[i,s]<=MAX_TIME and end>0):
                start = max(0, end - int(SALIENT_MOMENT_DURATION/dt));
                print "Play test %d, solver %d from %.2f s to %.2f s" % (i, s, start*dt, end*dt);
                sleep(1.0);
                viewer.updateRobotConfig(data['q'][start,s,:]);
                if(DUMP_IMAGE_FILES):
                    viewer.startCapture('solver_'+str(s));
                viewer.play(data['q'][start:end,s,:], dt, SALIENT_MOMENT_SLOWDOWN_FACTOR);
                if(DUMP_IMAGE_FILES):
                    viewer.stopCapture();  

    i += 1;
    if(i>=N_DIR):
        break;
counter_tests_per_config = counter_tests_per_config[:MAX_INITIAL_CONFIG_ID];

info = "Number of directories analyzed: "+str(i)+"\n";
for j in range(MAX_INITIAL_CONFIG_ID):
    if(counter_tests_per_config[j]>0):
        info += "Configuration %4d, number of tests %2d\n" % (j, counter_tests_per_config[j]);
info += "Number of times capture point was in/out:  "+str(capture_point_true)+" / "+str(i-capture_point_true)+"\n";
info += "Number of times multi-contact stability criterion was true/false:  "+str(multi_contact_true)+" / "+str(i-multi_contact_true)+"\n";
for s in range(N_SOLVERS):
    info += "Number of times solver "+str(s)+" balanced/did not balance:  "+str(controller_true[s])+" / "+str(i-controller_true[s])+"\n";
info += "Number of times capture point and multi-contact stability criterion disagreed %d\n"%capture_point_multi_contact_disagree;

print info_tail
print '\n'+info

if(SAVE_RESULTS):
    np.savez(DATA_PATH+'results', time_to_fall=time_to_fall);    
    tfile = open(DATA_PATH+TEXT_FILE_NAME, "w")
    tfile.write(info);
    tfile.write('\n'+info_tail);
    tfile.close();
