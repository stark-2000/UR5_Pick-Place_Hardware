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
#include "dhparam.h"

#include <string>

#include "NumCpp/Core/Internal/StaticAsserts.hpp"
#include "NumCpp/Core/Types.hpp"
#include "NumCpp/Functions/zeros.hpp"
#include "NumCpp/Linalg/svd.hpp"
#include "NumCpp/NdArray.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR> 

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenIntMatrix;
typedef Eigen::Map<EigenIntMatrix> EigenIntMatrixMap;

dhparam::dhparam(float d, float alpha, float  a, float theta , std::string type)
{
    this->d = d;
    this->theta = theta;
    this->a = a;
    this->alpha = alpha;
    this->type = type;
}

dhparam::~dhparam()
{
}

std::map<std::string, std::shared_ptr<dhparam>> Table_DHParam = { 
    {"J1" ,  std::make_shared<dhparam>(0.089159 ,   M_PI/2,   0 ,        0    , "R")},
    {"J2" ,  std::make_shared<dhparam>(0.0      ,    0,       -0.425 ,   0    , "R")},
    {"J3" ,  std::make_shared<dhparam>(0.0      ,    0,       -0.39225 , 0    , "R")},
    {"J4" ,  std::make_shared<dhparam>(0.10915  ,    M_PI/2,  0 ,        0    , "R")},
    {"J5" ,  std::make_shared<dhparam>(0.09465  , -1*M_PI/2,  0 ,        0    , "R")},
    {"J6" ,  std::make_shared<dhparam>(0.0823   ,    0,       0 ,        0    , "R")}
    };


nc::NdArray<float> FKinDHParam(nc::NdArray<float> config)
{
   nc::NdArray<float> TF_FixedToEndEffector = nc::eye<float>(4);

    //Perform Forward Kinematics to find the Transformations and 
    // enf effector location
    for(int i=1; i<=6; i++){
       std::string  Joint = "J"+std::to_string(i);
        
        //Compute transformation matrix from i-1 to ith link
        if(Table_DHParam[Joint]->type == "R"){

           float d = Table_DHParam[Joint]->d;
           float a = Table_DHParam[Joint]->a;
           float alpha = Table_DHParam[Joint]->alpha;
           float theta =config[i-1];

           nc::NdArray<float> TF = nc::NdArray<float>({
                {cos(theta),   -1*sin(theta)*cos(alpha),    sin(theta)*sin(alpha),  a*cos(theta)},
                {sin(theta)  ,    cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha),  a*sin(theta)},
                {0           ,    sin(alpha),               cos(alpha),             d           },
                {0,               0,                        0,                      1           }
            });

        // Find Each links transformation with respect to the fixed frame
        TF_FixedToEndEffector = nc::matmul(TF_FixedToEndEffector, TF);
        }
    }
    return TF_FixedToEndEffector;
}

nc::NdArray<float> GetGradient(nc::NdArray<float> joints){
   float delta = 0.0001;
   nc::NdArray<float> jac = nc::zeros<float>(16, 6);
    for(int i=0; i<6; i++)
    {
        nc::NdArray<float> joints_m = joints.copy();
        nc::NdArray<float> joints_p = joints.copy();
        joints_m[i] -= delta;
        joints_p[i] += delta;
        nc::NdArray<float>Tm = FKinDHParam(joints_m);
        nc::NdArray<float> Tp = FKinDHParam(joints_p);
        nc::NdArray<float> diff = (Tp - Tm).flatten();
        nc::NdArray<float> scaled_diff = diff / (2 * delta);

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","joints_m : "+std::to_string(i));
        // joints_m.print();
        jac(0,i) = scaled_diff(0,0);
        jac(1,i) = scaled_diff(0,1);
        jac(2,i) = scaled_diff(0,2);
        jac(3,i) = scaled_diff(0,3);
        jac(4,i) = scaled_diff(0,4);
        jac(5,i) = scaled_diff(0,5);
        jac(6,i) = scaled_diff(0,6);
        jac(7,i) = scaled_diff(0,7);
        jac(8,i) = scaled_diff(0,8);
        jac(9,i) = scaled_diff(0,9);
        jac(10,i) = scaled_diff(0,10);
        jac(11,i) = scaled_diff(0,11);
        jac(12,i) = scaled_diff(0,12);
        jac(13,i) = scaled_diff(0,13);
        jac(14,i) = scaled_diff(0,14);
        jac(15,i) = scaled_diff(0,15);
    }
    return jac;
}


template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{

	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        // For a non-square matrix
        // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

ikret::ikret(){}

ikret::~ikret(){}

std::shared_ptr<ikret> ik(nc::NdArray<float> T_tar, nc::NdArray<float> theta, float tolerance){
    
    nc::NdArray<float> T_FK = FKinDHParam(theta);
    // ROS_INFO_STREAM_NAMED("ur5v1_ik","joints : ");
    // theta.print();
    // ROS_INFO_STREAM_NAMED("ur5v1_ik","T_FK : ");
    // T_FK.print();

    float step = 0.5;
    nc::NdArray<float> joints = theta.copy();

    for(int i =0; i<10000; i++) {
       nc::NdArray<float> T_cur = FKinDHParam(joints);
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","T_cur : ");
        // T_cur.print();
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","T_tar : ");
        // T_tar.print();
       nc::NdArray<float> deltaT = (T_tar - T_cur).flatten();
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","deltaT : ");
        // deltaT.print();
       nc::NdArray<double> error = nc::norm(deltaT);
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","error : ");
        // error.print();
        if(error[0] < tolerance)
        {
            std::shared_ptr<ikret> objRet = std::make_shared<ikret>();
            objRet->theta = joints.copy();
            objRet->err = true;                                      
            return objRet;
        }
  
        nc::NdArray<float> jac = GetGradient(joints);
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","jac : ");
        // jac.print();

        Eigen::MatrixXd m(16, 6);
        for(int i=0; i<6; i++)
        {
            m(0,i)  = jac(0,i) ;m(1,i)  = jac(1,i) ;m(2,i)  = jac(2,i) ;m(3,i)  = jac(3,i) ;m(4,i)  = jac(4,i) ;
            m(5,i)  = jac(5,i) ;m(6,i)  = jac(6,i) ;m(7,i)  = jac(7,i) ;m(8,i)  = jac(8,i) ;m(9,i)  = jac(9,i) ;
            m(10,i) = jac(10,i);m(11,i) = jac(11,i);m(12,i) = jac(12,i);m(13,i) = jac(13,i);m(14,i) = jac(14,i);
            m(15,i) = jac(15,i);
        }
        
        auto pinv_jac_eig = pseudoInverse(m);
        // ROS_INFO_STREAM_NAMED("ur5v1_ik","pinv_jac_eig : "+
        // std::to_string(pinv_jac_eig.rows()) + " " +
        // std::to_string(pinv_jac_eig.cols()));
        
        nc::NdArray<float> inv_jac = nc::zeros<float>(6, 16);
        
        for(int i=0; i<16; i++)
        {
            inv_jac(0,i)  = pinv_jac_eig(0,i) ;
            inv_jac(1,i)  = pinv_jac_eig(1,i) ;
            inv_jac(2,i)  = pinv_jac_eig(2,i) ;
            inv_jac(3,i)  = pinv_jac_eig(3,i) ;
            inv_jac(4,i)  = pinv_jac_eig(4,i) ;
            inv_jac(5,i)  = pinv_jac_eig(5,i) ;
        }

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","inv_jac : ");
        // inv_jac.print();

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","deltaT : ");
        // deltaT.print();

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","joints : ");
        // joints.print();

        nc::NdArray<float> deltaq = nc::matmul(inv_jac ,deltaT.transpose());
        
        nc::NdArray<float> delta_grad = {
        deltaq(0,0),
        deltaq(1,0),
        deltaq(2,0),
        deltaq(3,0),
        deltaq(4,0),
        deltaq(5,0)};

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","delta_grad : ");
        // delta_grad.print();

        joints = joints +  step*delta_grad;

        // ROS_INFO_STREAM_NAMED("ur5v1_ik","joints : ");
        // joints.print();
    }

    std::shared_ptr<ikret> objRet = std::make_shared<ikret>();
    objRet->theta = joints.copy();
    objRet->err = false;                                      
    return objRet;
}