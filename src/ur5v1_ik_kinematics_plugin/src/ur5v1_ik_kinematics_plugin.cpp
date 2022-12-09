/******************************************************************************
* MIT License
*
* Copyright (c) 2022 <Team member names>
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
******************************************************************************/

/**
 * @file dhparam.cpp
 * @author Tej Kiran, Dhinesh Rajasekaran, Arshad Shaik
 * @brief This is cpp file which has the DH table and computes IK, FK
 * @version 1.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <NumCpp.hpp>

#include "dhparam.h"

namespace ur5v1_ik_kinematics_plugin
{

  class UR5V1_IKKinematicsPlugin : public kinematics::KinematicsBase
  {
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;

    uint num_joints_;
    bool active_; // Internal variable that indicates whether solvers are configured and ready

    KDL::Chain chain;
    bool position_ik_;
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<std::string>& getLinkNames() const { return link_names_; }

    KDL::JntArray joint_min, joint_max;

    std::string solve_type;

  public:

    /** @class
     *  @brief Interface for an ur5v1-ik kinematics plugin
     */
    UR5V1_IKKinematicsPlugin(): active_(false), position_ik_(false){}

    ~UR5V1_IKKinematicsPlugin() {
    }

    /**
     * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param solution the solution vector
     * @param error_code an error code that encodes the reason for failure or success
     * @return True if a valid solution was found, false otherwise
     */

    // Returns the first IK solution that is within joint limits, this is called by get_ik() service
    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
                       moveit_msgs::MoveItErrorCodes &error_code,
                       const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
     * around those specified in the seed state are admissible and need to be searched.
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param consistency_limit the distance that the redundancy can be from the current position
     * @return True if a valid solution was found, false otherwise
     */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const std::vector<double> &consistency_limits,
                          const kinematics::KinematicsQueryOptions &options) const;

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose
     *
     * This FK routine is only used if 'use_plugin_fk' is set in the 'arm_kinematics_constraint_aware' node,
     * otherwise ROS TF is used to calculate the forward kinematics
     *
     * @param link_names A set of links for which FK needs to be computed
     * @param joint_angles The state for which FK is being computed
     * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
     * @return True if a valid solution was found, false otherwise
     */
    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles,
                       std::vector<geometry_msgs::Pose> &poses) const;


  bool clamp(std::vector<double> &q_targ);

  private:



    bool initialize(const std::string &robot_description,
                    const std::string& group_name,
                    const std::string& base_name,
                    const std::string& tip_name,
                    double search_discretization);


    int getKDLSegmentIndex(const std::string &name) const;

  }; // end class

  bool UR5V1_IKKinematicsPlugin::initialize(const std::string &robot_description,
                                           const std::string& group_name,
                                           const std::string& base_name,
                                           const std::string& tip_name,
                                           double search_discretization)
  {
    setValues(robot_description, group_name, base_name, tip_name, search_discretization);

    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,robot_description);
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_NAMED("ur5v1_ik","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
        ROS_FATAL_NAMED("ur5v1_ik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return false;
      }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("ur5v1_ik","Reading joints and links from URDF");

    KDL::Tree tree;
    
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
      ROS_FATAL("Failed to extract kdl tree from xml robot description");
      return false;
    }

    if(!tree.getChain(base_name, tip_name, chain)) {
      ROS_FATAL("Couldn't find chain %s to %s",base_name.c_str(),tip_name.c_str());
      return false;
    }
    
    num_joints_=chain.getNrOfJoints();
    
    std::vector<KDL::Segment> chain_segs = chain.segments;

    std::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    joint_min.resize(num_joints_);
    joint_max.resize(num_joints_);

    uint joint_num=0;
    for(unsigned int i = 0; i < chain_segs.size(); ++i) {

      link_names_.push_back(chain_segs[i].getName());
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
        joint_num++;
        assert(joint_num<=num_joints_);
        float lower, upper;
        int hasLimits;
        joint_names_.push_back(joint->name);
        if ( joint->type != urdf::Joint::CONTINUOUS ) {
          if(joint->safety) {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          } else {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else {
          hasLimits = 0;
        }

        if(hasLimits) {
          joint_min(joint_num-1)=lower;
          joint_max(joint_num-1)=upper;
        }
        else {
          joint_min(joint_num-1)=std::numeric_limits<float>::lowest();
          joint_max(joint_num-1)=std::numeric_limits<float>::max();
        }
        ROS_INFO_STREAM("IK Using joint "<<chain_segs[i].getName()<<" "<<joint_min(joint_num-1)<<" "<<joint_max(joint_num-1));
      
      }
    }

    ROS_INFO_NAMED("ur5v1-ik plugin","Looking in private handle: %s for param name: %s",
                    node_handle.getNamespace().c_str(),
                    (group_name+"/position_only_ik").c_str());

    node_handle.param(group_name+"/position_only_ik", position_ik_, false);

    ROS_INFO_NAMED("ur5v1-ik plugin","Looking in private handle: %s for param name: %s",
                    node_handle.getNamespace().c_str(),
                    (group_name+"/solve_type").c_str());

    node_handle.param(group_name+"/solve_type", solve_type, std::string("Speed"));
    ROS_INFO_NAMED("ur5v1_ik plugin","Using solve type %s",solve_type.c_str());

    active_ = true;
    return true;
  }


  bool UR5V1_IKKinematicsPlugin::clamp(std::vector<double> &q_targ)
{
	bool found = true;
  double PI = M_PI;
	for(int i = 0; i < q_targ.size(); i++)
	{
		if(joint_min(i) <= std::fmod(q_targ[i],2*PI) && std::fmod(q_targ[i],2*PI) <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI);
		else if(joint_min(i) <= std::fmod(q_targ[i],2*PI)-2*PI && std::fmod(q_targ[i],2*PI)-2*PI <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)-2*PI;
		else if(joint_min(i) <= std::fmod(q_targ[i],2*PI)+2*PI && std::fmod(q_targ[i],2*PI)+2*PI <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)+2*PI;
		else
		{
			if(q_targ[i] < joint_min(i))
			{
				q_targ[i] = joint_min(i);
			}
			else
			{
				q_targ[i] = joint_max(i);
			}
			found = false;
		}
	}
	return found;		
}


  int UR5V1_IKKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
  {
    int i=0;
    while (i < (int)chain.getNrOfSegments()) {
      if (chain.getSegment(i).getName() == name) {
        return i+1;
      }
      i++;
    }
    return -1;
  }


  bool UR5V1_IKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                              const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::Pose> &poses) const
  {
  if(!active_)
  {
    ROS_ERROR_NAMED("ur5v1_ik","kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if(joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("ur5v1_ik","Joint angles vector must have size: %d",num_joints_);
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  KDL::JntArray jnt_pos_in(num_joints_);
  for(unsigned int i=0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    ROS_DEBUG_NAMED("ur5v1_ik","End effector index: %d",getKDLSegmentIndex(link_names[i]));
    if(fk_solver.JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
    {
      tf::poseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR_NAMED("ur5v1_ik","Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}

  bool UR5V1_IKKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                          const std::vector<double> &ik_seed_state,
                                          std::vector<double> &solution,
                                          moveit_msgs::MoveItErrorCodes &error_code,
                                          const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            default_timeout_,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool UR5V1_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             std::vector<double> &solution,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool UR5V1_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             const std::vector<double> &consistency_limits,
                                             std::vector<double> &solution,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool UR5V1_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             std::vector<double> &solution,
                                             const IKCallbackFn &solution_callback,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool UR5V1_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             const std::vector<double> &consistency_limits,
                                             std::vector<double> &solution,
                                             const IKCallbackFn &solution_callback,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    return searchPositionIK(ik_pose,
                            ik_seed_state,
                            timeout,
                            solution,
                            solution_callback,
                            error_code,
                            consistency_limits,
                            options);
  }

  bool UR5V1_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             std::vector<double> &solution,
                                             const IKCallbackFn &solution_callback,
                                             moveit_msgs::MoveItErrorCodes &error_code,
                                             const std::vector<double> &consistency_limits,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    ROS_INFO_STREAM_NAMED("ur5v1_ik","getPositionIK");

    if(!active_) {
      ROS_ERROR("kinematics not active");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    if (ik_seed_state.size() != num_joints_) {
      ROS_ERROR_STREAM_NAMED("ur5v1_ik","Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    KDL::Frame frame;
    tf::poseMsgToKDL(ik_pose,frame);

    KDL::JntArray in(num_joints_), out(num_joints_);

    for (uint z=0; z< num_joints_; z++)
        in(z)=ik_seed_state[z];

    KDL::Twist bounds=KDL::Twist::Zero();
    
    if (position_ik_)  {
      bounds.rot.x(std::numeric_limits<float>::max());
      bounds.rot.y(std::numeric_limits<float>::max());
      bounds.rot.z(std::numeric_limits<float>::max());
    }

    double epsilon = 1e-5;  //Same as MoveIt's KDL plugin

    // ur5v1_ik::SolveType solvetype;

    // if (solve_type == "Manipulation1")
    //   solvetype = ur5v1_ik::Manip1;
    // else if (solve_type == "Manipulation2")
    //   solvetype = ur5v1_ik::Manip2;
    // else if (solve_type == "Distance")
    //   solvetype = ur5v1_ik::Distance;
    // else {
    //     if (solve_type != "Speed") {
    //         ROS_WARN_STREAM_NAMED("ur5v1_ik", solve_type << " is not a valid solve_type; setting to default: Speed");
    //     }
    //     solvetype = ur5v1_ik::Speed;
    // }
    
    Eigen::Matrix4d eigenPose = Eigen::Matrix4d::Identity();
    eigenPose.block<3, 3>(0, 0) = Eigen::Quaterniond(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z).toRotationMatrix();
    eigenPose(0,3) = ik_pose.position.x;
    eigenPose(1,3) = ik_pose.position.y;
    eigenPose(2,3) = ik_pose.position.z;

    nc::NdArray<float> Tsd = {
                              {eigenPose(0,0),eigenPose(0,1),eigenPose(0,2),eigenPose(0,3)},
                              {eigenPose(1,0),eigenPose(1,1),eigenPose(1,2),eigenPose(1,3)},
                              {eigenPose(2,0),eigenPose(2,1),eigenPose(2,2),eigenPose(2,3)},
                              {eigenPose(3,0),eigenPose(3,1),eigenPose(3,2),eigenPose(3,3)}
                            };
    
    nc::NdArray<float> theta = {{ik_seed_state[0],ik_seed_state[1],ik_seed_state[2],ik_seed_state[3],ik_seed_state[4],ik_seed_state[5]}};

    std::shared_ptr<ikret> ret =  ik(Tsd, theta,  0.0001);
    solution.resize(num_joints_);
    if(ret->err)
    {
        ROS_INFO_STREAM_NAMED("ur5v1_ik","solution success");
        error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
        
        ROS_INFO_STREAM_NAMED("ur5v1_ik","solution ret->theta : "+std::to_string(ret->theta(0,0)));
        ret->theta.print();


        for (uint z=0; z< num_joints_; z++)
          solution[z]=ret->theta(0,z);

        solution[5] = 0;
        

std::vector<double> q_targ = solution;

      bool found = true;
  double PI = M_PI;
	for(int i = 0; i < q_targ.size(); i++)
	{
		if(joint_min(i) <= std::fmod(q_targ[i],2*PI) && std::fmod(q_targ[i],2*PI) <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI);
		else if(joint_min(i) <= std::fmod(q_targ[i],2*PI)-2*PI && std::fmod(q_targ[i],2*PI)-2*PI <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)-2*PI;
		else if(joint_min(i) <= std::fmod(q_targ[i],2*PI)+2*PI && std::fmod(q_targ[i],2*PI)+2*PI <= joint_max(i))
			q_targ[i] = std::fmod(q_targ[i],2*PI)+2*PI;
		else
		{
			if(q_targ[i] < joint_min(i))
			{
				q_targ[i] = joint_min(i);
			}
			else
			{
				q_targ[i] = joint_max(i);
			}
			found = false;
		}
	}

solution = q_targ;

				if(found)
				{
          ROS_INFO_STREAM_NAMED("ur5v1_ik","solution clamp success");
				}
				else
        {
              ROS_DEBUG_STREAM_NAMED("ur5v1_ik","solution clamp failed");
              return false;
        }
          
        ROS_INFO_STREAM_NAMED("ur5v1_ik","solution solution : "+ std::to_string(solution[0]));

        if( !solution_callback.empty() )
        {
          solution_callback(ik_pose, solution, error_code);
          if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
          {
            ROS_DEBUG_STREAM_NAMED("ur5v1_ik","Solution passes callback");
            return true;
          }
          else
          {
            ROS_DEBUG_STREAM_NAMED("ur5v1_ik","Solution has error code " << error_code);
            return false;
          }
        }
        return true; 
    }
    else
    {
    ROS_INFO_STREAM_NAMED("ur5v1_ik","solution failed");
      error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // ur5v1_ik::ur5v1_ik ik_solver(chain, joint_min, joint_max, timeout, epsilon, solvetype);

    // int rc = ik_solver.CartToJnt(in, frame, out, bounds);


    // solution.resize(num_joints_);

    // if (rc >=0) {
    //   for (uint z=0; z< num_joints_; z++)
    //     solution[z]=out(z);

    //   // check for collisions if a callback is provided 
    //   if( !solution_callback.empty() )
    //     {
    //       solution_callback(ik_pose, solution, error_code);
    //       if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    //         {
    //           ROS_DEBUG_STREAM_NAMED("ur5v1_ik","Solution passes callback");
    //           return true;
    //         }
    //       else
    //         {
    //           ROS_DEBUG_STREAM_NAMED("ur5v1_ik","Solution has error code " << error_code);
    //           return false;
    //         }
    //     }
    //   else
    //       return true; // no collision check callback provided
    // }

    // error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }



} // end namespace

//register UR5V1_IKKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ur5v1_ik_kinematics_plugin::UR5V1_IKKinematicsPlugin, kinematics::KinematicsBase);
