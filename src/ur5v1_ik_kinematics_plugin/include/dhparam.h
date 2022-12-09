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
 * @file dhparam.hpp
 * @author Tej Kiran, Dhinesh Rajasekaran, Arshad Shaik
 * @brief This is header file consisting of function declarations
 * @version 1.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

//Dependecies:
#include <iostream> ///> Basic header
#include <string> ///> Basic header
#include <memory> ///> Basic header
#include <map> ///> Basic header
#include <NumCpp.hpp> ///> Using Numpy equivalent for C++

/**
 * @brief This class gets the 4 parameters from DH table for computing FK 
 * Transformation matrix
 *
 */
class dhparam
{
public:
    float d;
    float theta;
    float a;
    float alpha;
    std::string type;

    dhparam(float d, float alpha, float  a, float theta , std::string type);
    ~dhparam();
};

/**
 * @brief This class holds the joint angles theta
 *
 */
class ikret
{
public:
    nc::NdArray<float> theta;
    bool err;
    ikret();
    ~ikret();
};

extern std::map<std::string, std::shared_ptr<dhparam>> Table_DHParam;

nc::NdArray<float> FKinDHParam(nc::NdArray<float> config);

nc::NdArray<float> GetGradient(nc::NdArray<float> joints);

std::shared_ptr<ikret> ik(nc::NdArray<float> T_tar, nc::NdArray<float> theta, float tolerance);
