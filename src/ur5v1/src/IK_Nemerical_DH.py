import numpy as np
from utilities import *
from tqdm import *

# Example Reference:
# https://github.com/George-Delagrammatikas/DukeMEMS-MotionPlanning/

def GetGradient(joints, DHTable):
    delta = 0.0001
    jacobian = np.zeros((16, joints.shape[0]))
    for i, joint in enumerate(joints):
        q_plus_delta = joints.copy()
        q_minus_delta = joints.copy()

        q_plus_delta[i] = q_plus_delta[i] - delta
        q_minus_delta[i] = q_minus_delta[i] + delta

        EF_plus_delta ,_ = FKinDHParam(q_plus_delta, DHTable)
        EF_minus_delta ,_ = FKinDHParam(q_minus_delta, DHTable)
        jacobian[:, i] = (EF_minus_delta - EF_plus_delta).flatten() / (2 * delta)
    return jacobian

def ik(DHTable, T_desired, theta_seed, tolerance = 1e-17):
    step = 0.5
    joints = theta_seed.copy()
    for i in tqdm(range(10000)):
        T_cur,_ = FKinDHParam(joints, DHTable)  
        deltaError = (T_desired - T_cur).flatten()
        error = np.linalg.norm(deltaError)
        if error < tolerance:
            return joints, True
        jac = GetGradient(joints, DHTable)
        inv_jac = np.linalg.pinv(jac)

        #Newton-Raphson method
        deltaq = inv_jac @ deltaError

        #Damped Least squared test - commented due to high no of iterations required to compute IK
        #More accurate - if req un-comment the below line:
        #deltaq = jac.T@np.linalg.pinv((jac@jac.T)+0.9*np.eye(16)) @ deltaT

        joints = joints + step * deltaq
    return joints, False