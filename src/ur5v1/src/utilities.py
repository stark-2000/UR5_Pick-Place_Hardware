import numpy as np
from numpy import linalg as LA
import math
from sympy import Function, MatrixSymbol, Matrix, cos, sin, pprint, eye

from robot_params import *

def FKinDHParam(config, DHParam):
    TF_base_list = [ ]
    TF_link_list = [ ]

    TF_FixedToEndEffector = np.eye(4)

    #Perform Forward Kinematics to find the Transformations and 
    # end effector location
    for i in range(1,len(DHParam)+1):
        Joint = 'J'+str(i)
        
        #Compute transformation matrix from i-1 to ith link
        if DHParam[Joint].type == 'R':

            d = DHParam[Joint].d
            a = DHParam[Joint].a
            alpha = DHParam[Joint].alpha
            theta =config[i-1]

            TF = np.array([
                [cos(theta),   -1*sin(theta)*cos(alpha),    sin(theta)*sin(alpha),  a*cos(theta)],
                [sin(theta)  ,    cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha),  a*sin(theta)],
                [0           ,    sin(alpha),               cos(alpha),             d           ],
                [0,               0,                        0,                      1           ]
            ], dtype='f')

        # Find Each links transformation with respect to the fixed frame
        TF_FixedToEndEffector = TF_FixedToEndEffector@TF
        TF_link_list.append(TF)
        TF_base_list.append(TF_FixedToEndEffector)

    return TF_FixedToEndEffector, TF_base_list

def GetJacobian(config, DHParam):

    #Create five configurations of the robot with different values for theta1 to theta8
    TF_FixedToEndEffector, TF_list = FKinDHParam(config, DHParam)

    #Append points to the list
    T_EF = TF_FixedToEndEffector

    #Find jacobian from the transformations
    TF_list.insert(0, eye(4))
    Jacobian = []
    #J[i] = [Jv{3x1} Jw{3x1}] {6x1} --> each column
    for i in range(1,len(DHParam)+1):
        Joint = 'J'+str(i)

        if DHParam[Joint].type == 'R':

             #J linear velocity component for the revolute joint is given as \
            # Zi
            Ji_angular = np.squeeze(TF_list[i][:3,2])

            # All w.r.t to fixed frame ie. Zi-1 to {0} frame, Oi-1, On etc
            Zi_1 = np.squeeze(TF_list[i-1][:3,2])
            Oi_1 = np.squeeze(TF_list[i-1][:3,3])
            On = np.squeeze(TF_list[len(TF_list)-1][:3,3])

            O_diff = On - Oi_1
            
            #J linear velocity component for the revolute joint is given as 
            # Zi-1x(On - Oi-1)
            Ji_linear = np.cross(Zi_1, O_diff)

            #Concatenate linear and angular velocity components
            Ji = np.concatenate((Ji_angular, Ji_linear), axis=0)
            Jacobian.append(Ji)
           
        if DHParam[Joint].type == 'P':
            Zi = np.squeeze(TF_list[i][:3,2])

            Ji_linear = Zi
            Ji_angular = np.zeros(3)
            Ji = np.concatenate((Ji_linear, Ji_angular), axis=0)
            Jacobian.append(Ji)
    
    #Take transpose to match dimentions as [6xn]
    Jacobian = np.array(Jacobian, dtype=np.float32).T

    return Jacobian, T_EF