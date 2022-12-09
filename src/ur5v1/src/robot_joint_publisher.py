import rospy
from std_msgs.msg import Float64

from robot_params import *
from utilities import *

class robot_joint_publisher():

    joint_publishers = {}
    def __init__(self) -> None:
        #Perform Forward Kinematics to find the Transformations and 
        # enf effector location
        for i in range(1,len(Table_DHParam)+1):
            Joint = 'J'+str(i)
            self.joint_publishers[Joint] = rospy.Publisher(Table_DHParam[Joint].topic, Float64, queue_size=10)
    
    def publish_joint(self, config):
        for i in range(1,len(Table_DHParam)+1):
            Joint = 'J'+str(i)
            self.joint_publishers[Joint].publish(config[i-1])