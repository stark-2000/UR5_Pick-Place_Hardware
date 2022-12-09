from sympy import symbols
from math import pi

#Create class to store different DH paramters of each joint:
class DHParam:
   def __init__(self, d, alpha, a, theta, topic, type = 'R'):
    self.d = d
    self.alpha = alpha
    self.a = a
    self.theta = theta
    self.type = type
    self.topic = topic

#http://dr-kors.at/pdf/Dynamic_parameter_identification_of_the_Universal_Robots_UR5.pdf
#Store DH Parameters of Panda robot in home position

#DH Parameter Table:
Table_DHParam = {
    'J1' : DHParam(d=0.089159 , alpha= pi/2  , a =  0        , theta = symbols("θ1"), topic= "/shoulder_sweep_position_controller/command" ,type= 'R'),
    'J2' : DHParam(d=0.0      , alpha= 0     , a = -0.425    , theta = symbols("θ2"), topic= "/shoulder_lift_position_controller/command" ,type= 'R'),
    'J3' : DHParam(d=0.0      , alpha= 0     , a = -0.39225  , theta = symbols("θ3"), topic= "/elbow_position_controller/command" ,type= 'R'),
    'J4' : DHParam(d=0.10915  , alpha= pi/2  , a =  0.0      , theta = symbols("θ4"), topic= "/wrist_pitch_position_controller/command" ,type= 'R'),
    'J5' : DHParam(d=0.09465  , alpha=-pi/2  , a =  0.0      , theta = symbols("θ5"), topic= "/wrist_roll_position_controller/command" ,type= 'R'),
    'J6' : DHParam(d=0.0823   , alpha= 0     , a =  0.0      , theta = symbols("θ6"), topic= "/wrist_yaw_position_controller/command" ,type= 'R'),
}