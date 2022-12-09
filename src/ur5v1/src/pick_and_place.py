from IK_Nemerical_DH import *
from robot_joint_publisher import *

if __name__ =="__main__":

    rospy.init_node("main_ur5v1")

    rate = rospy.Rate(10)
    joint_publisher = robot_joint_publisher()

    theta_pick      = np.array([-pi/2, -pi/3,    pi/2,   -pi/6,  pi/2, pi/2])
    theta_lean = np.array([-pi/2, -pi/1.5,  pi/1.5, -pi/16, pi/2, pi/2])
    theta_turn      = np.array([ pi/2, -pi/1.5,  pi/1.5, -pi/16, pi/2, pi/2])
    theta_place     = np.array([ pi/2, -pi/3,    pi/2,   -pi/6,  pi/2, pi/2])

    theta_zero = np.array([0,0,0,0,0,0])
    rospy.sleep(1.5)
    print("\nMoving Robot to Zero location   : ", theta_pick)
    joint_publisher.publish_joint(theta_zero)

    Tsd_Pick,  _ = FKinDHParam(theta_pick, Table_DHParam)
    Tsd_Lean,  _ = FKinDHParam(theta_lean, Table_DHParam)
    Tsd_Turn,  _ = FKinDHParam(theta_turn, Table_DHParam)
    Tsd_Place, _ = FKinDHParam(theta_place, Table_DHParam)

    print(theta_pick)
    print(Tsd_Pick.flatten())
    
    theta_pick_guess     = np.array([-pi/1.3, -pi/1.5,    pi/2,   -pi/5.5,  pi/2.5, pi/3.5])
    theta_lean_guess     = np.array([-pi/2, -pi/1.5,  pi/1.6, -pi/16, pi/2, pi/2])
    theta_turn_guess     = np.array([ pi/1.8, -pi/1.5,  pi/1.8, -pi/16, pi/1.5, pi/2])
    theta_place_guess    = np.array([ pi/2, -pi/1.2,    pi/2,   -pi/6,  pi/2, pi/1.5])

    print("\nActual Pick location   : ", theta_pick)
    print("Guess Pick location    : ", theta_pick_guess)
    print("Running IK for Pick location : ")
    theta_pick_sol, done = ik(Table_DHParam,Tsd_Pick, theta_pick_guess, 0.0001)
    if done:
        print("Computed Pick location : ", theta_pick_sol)
    else:
        print("IK failed for Pick")

    print("\nActual Lean location   : ", theta_lean)
    print("Guess Lean location    : ", theta_lean_guess)
    print("Running IK for Lean location : ")
    theta_lean_sol, done = ik(Table_DHParam,Tsd_Lean, theta_lean_guess, 0.0001)
    if done:
        print("Computed Lean location : ", theta_lean_sol)
    else:
        print("IK failed for Lean")

    print("\nActual Turn location   : ", theta_turn)
    print("Guess Turn location    : ", theta_turn_guess)
    print("Running IK for Turn location : ")
    theta_turn_sol, done = ik(Table_DHParam,Tsd_Turn, theta_turn_guess, 0.0001)
    if done:
        print("Computed Turn location : ", theta_turn_sol)
    else:
        print("IK failed for Turn")

    print("\nActual Place location   : ", theta_place)
    print("Guess Place location    : ", theta_place_guess)
    print("Running IK for Place location : ")
    theta_place_sol, done = ik(Table_DHParam,Tsd_Place,theta_place_guess, 0.0001)
    if done:
        print("Computed Place location : ", theta_place_sol)
    else:
        print("IK failed for Place")

    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_pick_sol)
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_lean_sol)
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_turn_sol)
    rospy.sleep(1.5)
    joint_publisher.publish_joint(theta_place_sol)

    print("Pick and Place completed")

    while not rospy.is_shutdown():
        rate.sleep()