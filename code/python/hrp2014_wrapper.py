from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import pinocchio as se3

class Hrp2014Wrapper(RobotWrapper):

    def __init__(self,*args):
        filename = args[0]
        if len(args) >= 2:
          mesh_dir = args[1]
          RobotWrapper.__init__(self,filename,mesh_dir,se3.JointModelFreeFlyer())
        else:
          RobotWrapper.__init__(self,filename,se3.JointModelFreeFlyer())
#        RobotWrapper.__init__(self,filename, root_joint='ff')
        self.q0 = np.matrix( [
        0.0, 0.0, 0.648702, 0.0, 0.0 , 0.0, 1.0,                             # Free flyer 0-6
        0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
        0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # LARM       11-17
        0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # RARM       18-24
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
        0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
        ] ).T

        self.ff = range(7)
        self.chest = range(7,9)
        self.head = range(9,11)
        self.l_arm = range(11,18)
        self.r_arm = range(18,25)
        self.l_leg = range(25,31)
        self.r_leg = range(31,37)

        self.opCorrespondances = { "lh": "LARM_JOINT5",
                                   "rh": "RARM_JOINT5",
                                   "lf": "LLEG_JOINT5",
                                   "rf": "RLEG_JOINT5",
                                   }

        for op,name in self.opCorrespondances.items():
            idx = self.__dict__[op] = self.index(name)
            #self.__dict__['_M'+op] = types.MethodType(lambda s,q: s.position(q,idx),self)

    # --- SHORTCUTS ---
    def Mrh(self,q):
        return self.position(q,self.rh)
    def Jrh(self,q):
        return self.jacobian(q,self.rh)
    def wJrh(self,q):
        return se3.jacobian(self.model,self.data,self.rh,q,False)
    def vrh(self,q,v):
        return self.velocity(q,v,self.rh)

    def Jlh(self,q):
        return self.jacobian(q,self.lh)
    def Mlh(self,q):
        return self.position(q,self.lh)

    def Jlf(self,q):
        return self.jacobian(q,self.lf)
    def Mlf(self,q):
        return self.position(q,self.lf)

    def Jrf(self,q):
        return self.jacobian(q,self.rf)
    def Mrf(self,q):
        return self.position(q,self.rf)


__all__ = [ 'Hrp2014Wrapper' ]
