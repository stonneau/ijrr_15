#!/usr/bin/env python

#from tools import *
import pinocchio as se3
from pinocchio.utils import *
from pinocchio import SE3
from pinocchio import Motion
import numpy as np
import time
from math import *

def errorInSE3 (M, Mdes):
	error = se3.log(Mdes.inverse () * M)
	return error

# Define a generic Task
class Task:
  def __init__ (self, robot, name = "Task"):
    self.name = name
    self.robot = robot
    self._kp = 1.
    self._kv = 1.
    self._coeff = 1.

  def setCoeff(self, value):
    self._coeff = value

  @property
  def kp(self):
    return self._kp

  @kp.setter
  def kp(self, value):
    self._kp = value

  @property
  def kv(self):
    return self._kv

  @kv.setter
  def kv(self, kv):
    self._kv = kv

  def error_kin(self, t, q):
    error = np.matrix ([]).reshape (0, 0)
    r_dot = np.matrix ([]).reshape (0, 0)
    return error, r_dot

  def error_dyn(self, t, q, qdot):
    error = np.matrix ([]).reshape (0, 0)
    error_dot = np.matrix ([]).reshape (0, 0)
    r_ddot = np.matrix ([]).reshape (0, 0)
    return error, error_dot, r_ddot

  def dyn_value(self, t, q, qdot, update_geometry = False):
    a_des = np.matrix ([]).reshape (0, 0)
    J = np.matrix ([]).reshape (0, 0)
    drift = np.matrix ([]).reshape (0, 0)
    return J, drift, a_des

  def rhs(self, *args):
    num_args = len(args)
    assert num_args <= 3 and num_args > 1, "rhs(t, q, (qdot))" 

    if num_args == 2:
      t = args[0]
      q = args[1]
      error, r_dot = self.error_kin(t, q) 
      #print error
      rhs_value = -self.kp * error + r_dot
      #print self._kp * error
    else:
      t = args[0]
      q = args[1]
      v = args[2]
      error, error_dot, r_ddot = self.error_dyn(t, q, v) 
      #if error[0] > 1e-2:
        #print self.kp
        #print error
      rhs_value = -self.kp * error - self.kv * error_dot + r_ddot

    return rhs_value

  def jacobian (self):
    jacobian = matrix ([]).reshape (0, robot.nq)
    return jacobian
  
  @property
  def dim(self):
    return 0

# Define SE3 Task
class SE3Task(Task):

  def __init__ (self, robot, link_id, ref_trajectory, name = "SE3 Task"):
    Task.__init__ (self, robot, name)
    self._link_id = link_id
    self._ref_trajectory = ref_trajectory

    # set default value to M_ref
    self._M_ref = SE3.Identity () 
  
    # mask over the desired euclidian axis
    self._mask = (np.ones(6)).astype(bool)

  @property
  def dim(self):
    return self._mask.sum ()

  def mask(self, mask):
    assert len(mask) == 6, "The mask must have 6 elemets"
    self._mask = mask.astype(bool)

  @property
  def refTrajectory(self):
    return self._ref_trajectory

  def refConfiguration (self, M_ref):
    assert isinstance(M_ref, SE3), "M_ref is not an element of class SE3"
    self._M_ref = M_ref

  def error_kin(self, t, q, update_geometry = True):
    # Get the current configuration of the link
    self._M = self.robot.position(q, self._link_id, update_geometry)
    M_ref, v_ref, _  = self._ref_trajectory(t)

    # Compute error
    self.__error_value = errorInSE3(self._M, M_ref).vector();

    return self.__error_value[self._mask], v_ref.vector()[self._mask]

  def error_dyn(self, t, q, v, update_geometry = False):
    # Get the current configuration of the link
    oMi = self.robot.data.oMi[self._link_id]

    # Get the reference trajectory
    M_ref, v_ref, a_ref  = self._ref_trajectory(t)

    # Transformation from local to world
    gMl = SE3.Identity()
    gMl.rotation = oMi.rotation
    v_frame = self.robot.velocity(q,v,self._link_id, update_geometry)

    # Compute error
    self.__error_value = errorInSE3(oMi, M_ref).vector;
    v_error = v_frame - gMl.actInv(v_ref)
    a_frame = self.robot.acceleration(q,v,0*v,self._link_id, update_geometry)
    a_coriolis = Motion.Zero()
    a_coriolis.linear = a_frame.linear
    a_coriolis.angular = a_frame.angular
    a_coriolis.linear += np.cross(v_frame.angular.T, v_frame.linear.T).T

    a_tot = gMl.actInv(a_ref) - a_coriolis

    return self.__error_value[self._mask], v_error.vector()[self._mask], a_tot.vector()[self._mask]

  def dyn_value(self, t, q, v, update_geometry = False):
    # Get the current configuration of the link
    oMi = self.robot.data.oMi[self._link_id]

    # Get the reference trajectory
    M_ref, v_ref, a_ref  = self._ref_trajectory(t)

    # Transformation from local to world
    gMl = SE3.Identity()
    gMl.rotation = oMi.rotation
    v_frame = self.robot.velocity(q,v,self._link_id, update_geometry)

    # Compute error acceleration desired
    p_error= errorInSE3(oMi, M_ref);
    v_error = v_frame - gMl.actInv(v_ref)
    a_frame = self.robot.acceleration(q,v,0*v,self._link_id, update_geometry)
    a_coriolis = Motion.Zero()
    a_coriolis.linear = a_frame.linear
    a_coriolis.angular = a_frame.angular
    a_coriolis.linear += np.cross(v_frame.angular.T, v_frame.linear.T).T
    
    drift = a_coriolis.vector
    a_des = -self.kp * p_error.vector -self.kv * v_error.vector + gMl.actInv(a_ref).vector

    # Compute jacobian
    J = self.robot.jacobian(q, self._link_id, update_geometry)

    return J[self._mask,:], drift[self._mask], a_des[self._mask]

  def jacobian(self, q, update_geometry = False):
    self.__jacobian_value = self.robot.jacobian(q, self._link_id, update_geometry)
    
    return self.__jacobian_value[self._mask,:] 


# Define CoM Task
class CoMTask(Task):

  def __init__ (self, robot, ref_trajectory, name = "CoM Task"):
    assert ref_trajectory.dim == 3
    Task.__init__ (self, robot, name)
    self._ref_trajectory = ref_trajectory

    # mask over the desired euclidian axis
    self._mask = (np.ones(3)).astype(bool)

    # desired CoM position
    self.CoM_ref = np.matrix([0., 0., 0.6]).T

  #@kp.setter
  #def kp(self, *args):
  #  num_args = len(args)
  #  self._kp = args
  #  #if num_args == 1:
  #  #  if isinstance(args, np.matrix):
  #  #    self._kp = np.diag(args)
  #  #  else:
  #  #    self._kp = args
  #  #else:
  #  #  assert num_args == 3
  #  #  self._kp = np.diag(args)

  @property
  def dim(self):
    return self._mask.sum()

  @property
  def RefTrajectory(self):
    return self._ref_trajectory

  def mask(self, mask):
    assert len(mask) == 3, "The mask must have 3 elements"
    self._mask = mask.astype(bool)

  def error_kin(self, t, q, update_geometry = True):
    # Get the current CoM position
    self.CoM = self.robot.com(q)

    p_ref, v_ref, _ = self._ref_trajectory(t)

    # Compute error
    self.__error_value = self.CoM - p_ref

    return self.__error_value[self._mask], v_ref[self._mask]
  
  def error_dyn(self, t, q, v, update_geometry = True):
    # Get the current CoM position
    p_com, v_com, a_com = self.robot.com(q,v,0*v)
    self.CoM = p_com
    p_ref, v_ref, a_ref = self._ref_trajectory(t)

    # Compute error
    self.__error_value = p_com - p_ref
    v_error = v_com - v_ref 

    a_tot = a_ref - a_com 

    return self.__error_value[self._mask], v_error[self._mask], a_tot[self._mask]
    
  def dyn_value(self, t, q, v, update_geometry = False):
    # Get the current CoM position, velocity and acceleration
    p_com, v_com, a_com = self.robot.com(q,v,0*v)

    # Get reference CoM trajectory
    p_ref, v_ref, a_ref = self._ref_trajectory(t)

    # Compute errors
    p_error = p_com - p_ref
    v_error = v_com - v_ref 

    drift = a_com # Coriolis acceleration
    a_des = -(self.kp * p_error + self.kv * v_error) + a_ref

    # Compute jacobian
    J = self.robot.Jcom(q)

    return J[self._mask,:], drift[self._mask], a_des[self._mask]

  def jacobian(self, q, update_geometry = True):
    
    self.__jacobian_value = self.robot.Jcom(q) # TODO - add update geometry option
    
    return self.__jacobian_value[self._mask,:] 

# Define Postural Task
class PosturalTask(Task):

  def __init__ (self, robot, name = "Postural Task"):
    Task.__init__ (self, robot, name)

    # mask over the desired euclidian axis
    self._mask = (np.ones(robot.nv)).astype(bool)

    # desired postural configuration
    self.q_posture_des = zero(robot.nq)

    # Init
    self.__error_value = np.matrix(np.empty([robot.nv,1]))
    self.__jacobian_value = np.matrix(np.identity(robot.nv))
    self.__gain_vector = np.matrix(np.ones([1.,robot.nv]))

  @property
  def dim(self):
    return self._mask.sum ()

  def setPosture(self, q_posture):
    self.q_posture_des = q_posture

  def setGain(self, gain_vector):
    assert gain_vector.shape == (1, self.robot.nv) 
    self.__gain_vector = np.matrix(gain_vector)

  def getGain(self):
    return self.__gain_vector

  def mask(self, mask):
    assert len(mask) == self.robot.nv, "The mask must have {} elements".format(self.robot.nq)
    self._mask = mask.astype(bool)
  
  def error_kin(self, t, q):
    nv = self.robot.nv
    M_ff = XYZQUATToSe3(q[:7])
    M_ff_des = XYZQUATToSe3(self.q_posture_des[:7])

    error_ff = errorInSE3(M_ff, M_ff_des).vector() 
    
    # Compute error
    error_value = self.__error_value
    error_value[:6,0] = error_ff
    error_value[6:,0] = q[7:,0] - self.q_posture_des[7:,0]

    return self.__error_value[self._mask], 0.

  def error_dyn(self, t, q, v):
    M_ff = XYZQUATToSe3(q[:7])
    M_ff_des = XYZQUATToSe3(self.q_posture_des[:7])
    #self.robot.mass(q)

    error_ff = errorInSE3(M_ff, M_ff_des).vector() 
    
    # Compute error
    error_value = self.__error_value
    error_value[:6,0] = error_ff
    error_value[6:,0] = q[7:,0] - self.q_posture_des[7:,0]
    
    #print error_value
    #diag = np.matrix(self.robot.data.M.diagonal()) 
    #print diag
    
    #M = self.robot.data.M
    #P = np.diag(np.diag(M.A)) 
    #print P.shape 
    #print error_value.shape 
    #error_value_pond = np.matrix(P * error_value)
    return error_value[self._mask], v[self._mask], 0.
    #return error_value_pond[self._mask], v[self._mask], 0.

  def dyn_value(self, t, q, v, update_geometry = False):
    M_ff = XYZQUATToSe3(q[:7])
    M_ff_des = XYZQUATToSe3(self.q_posture_des[:7])
    error_ff = errorInSE3(M_ff, M_ff_des).vector
    
    # Compute error
    error_value = self.__error_value
    error_value[:6,0] = error_ff
    error_value[6:,0] = q[7:,0] - self.q_posture_des[7:,0]
    
    self.J = np.diag(self.__gain_vector.A.squeeze())
    self.a_des = -(self.kp * error_value + self.kv * v)
    self.drift = 0*self.a_des
    
    return self.J[self._mask,:], self.drift[self._mask], self.a_des[self._mask]

  def jacobian(self, q):
    self.__jacobian_value = np.diag(self.__gain_vector.A.squeeze())
    return self.__jacobian_value[self._mask,:] 

# Define Angular Momentum Task
class AngularMomentumTask(Task):

  def __init__ (self, robot, name = "Angular Momentum Task"):
    Task.__init__ (self, robot, name)

    # mask over the desired euclidian axis
    self._mask = (np.ones(robot.nv)).astype(bool)

  @property
  def dim(self):
    return self._mask.sum ()

  def mask(self, mask):
    assert len(mask) == 3, "The mask must have {} elements".format(3)
    self._mask = mask.astype(bool)

  def setTrajectory(self, traj):
    self._ref_traj = traj
  
  def error_dyn(self, t, q, v):
    g = self.robot.biais(q,0*v)
    b = self.robot.biais(q,v)
    b -= g;
    M = self.robot.mass(q)

    com_p = self.robot.com(q)
    cXi = SE3.Identity()
    oXi = self.robot.data.oMi[1]
    cXi.rotation = oXi.rotation
    cXi.translation = oXi.translation - com_p
    b_com = cXi.inverse().np.T * b[:6,0]
    b_angular = -b_com[3:,:]

    M_com = cXi.inverse().np.T * M[:6,:]
    L = M_com[3:,:] * v

    L_des, Ldot_des = self._ref_traj(t)
    L_error = L - L_des

    acc = Ldot_des - b_com[3:,:]
    
    # Compute error
    #error_value = self.__error_value
    #error_value[:6,0] = error_ff
    #error_value[6:,0] = q[7:,0] - self.q_posture_des[7:,0]
    
    #print error_value
    #diag = np.matrix(self.robot.data.M.diagonal()) 
    #print diag
    
    #M = self.robot.data.M
    #P = np.diag(np.diag(M.A)) 
    #print P.shape 
    #print error_value.shape 
    #error_value_pond = np.matrix(P * error_value)
    #print b_angular[self._mask,0]
    #print L
    #L -= 10.
    #wXc  = SE3(eye(3),self.robot.position(q,1).inverse()*self.robot.com(q))
    #Jang = wXc.action.T[3:,:]*self.robot.mass(q)[:6,:]
    #b_com = wXc.action.T[3:,:]*b[:6]
    #b_angular = -0*b_com
    #bang = Jang*v
    #return L[self._mask], 0., b_angular[self._mask,0]
    return self._coeff * L_error[self._mask], 0., self._coeff * acc[self._mask,0]
    #return bang[self._mask], 0., b_angular[self._mask,0]


  def jacobian(self, q):
    self.robot.mass(q)
    com_p = self.robot.com(q)
    cXi= SE3.Identity()
    oXi = self.robot.data.oMi[1]
    cXi.rotation = oXi.rotation
    cXi.translation = oXi.translation - com_p
    M_ff = self.robot.data.M[:6,:]
    M_com = cXi.inverse().np.T * M_ff
    L_dot = M_com[3:,:]
    wXc  = SE3(eye(3),self.robot.position(q,1).inverse()*self.robot.com(q))
    Jang = wXc.action.T[3:,:]*self.robot.mass(q)[:6,:]
    return self._coeff * L_dot[self._mask,:] 
    #return Jang[self._mask,:] 

class ConfigTask(Task):

  def __init__ (self, robot, dof, ref_traj, name = "Config Task"):
    Task.__init__ (self, robot, name)

    # mask over the desired euclidian axis
    self.dim = len(dof)
    self._mask = (np.ones(self.dim)).astype(bool)
    self.dof = dof

    # desired postural configuration
    self.ref_traj = ref_traj

    # Init
    self.__error_value = np.matrix(np.empty([robot.nv,1]))
    self.__jacobian_value = np.matrix(np.identity(robot.nv))[np.array(dof)-1,:]
    self.__gain_vector = np.matrix(np.ones([1.,robot.nv]))

  @property
  def dim(self):
    return self._mask.sum ()

  def setTraj(self, ref_traj):
    self.ref_traj = ref_traj

  def setGain(self, gain_vector):
    assert gain_vector.shape == (1, self.robot.nv) 
    self.__gain_vector = np.matrix(gain_vector)

  def getGain(self):
    return self.__gain_vector

  def mask(self, mask):
    assert len(mask) == self.dim, "The mask must have {} elements".format(self.dim)
    self._mask = mask.astype(bool)
  
  def error_kin(self, t, q):
    nv = self.robot.nv
    M_ff = XYZQUATToSe3(q[:7])
    M_ff_des = XYZQUATToSe3(self.q_posture_des[:7])

    error_ff = errorInSE3(M_ff, M_ff_des).vector() 
    
    # Compute error
    error_value = self.__error_value
    error_value[:6,0] = error_ff
    error_value[6:,0] = q[7:,0] - self.q_posture_des[7:,0]

    return self.__error_value[self._mask], 0.

  def error_dyn(self, t, q, v):
    q_ref, v_ref, a_ref = self.ref_traj(t) 
    
    # Compute error
    error_value = q[self.dof,0] - q_ref
    v_err = v[np.array(self.dof)-1,0] - v_ref
    a_tot = a_ref
    
    return error_value[self._mask,0], v_err[self._mask,0], a_tot[self._mask,0]
  
  def dyn_value(self, t, q, v, update_geometry = False):
    q_ref, v_ref, a_ref = self.ref_traj(t) 
    
    # Compute error
    error_value = q[self.dof,0] - q_ref
    v_err = v[np.array(self.dof)-1,0] - v_ref
    
    J = self.__jacobian_value[self._mask,:]
    a_des = -(self.kp * error_value + self.kv * v_err) + a_ref
    drift = 0*a_des
    
    return J[self._mask,:], drift[self._mask], a_des[self._mask]

  def jacobian(self, q):
    #self.robot.mass(q)
    #M = self.robot.data.M
    #P = np.diag(np.diag(M.A)) 
    #self.__jacobian_value = np.diag(self.__gain_vector.A.squeeze())
    return self.__jacobian_value[self._mask,:] 
    #return P[self._mask,:]

