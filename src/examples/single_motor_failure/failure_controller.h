/****************************************************************************
 *
 *   Copyright (C) 2024 Inter-IIT Team 62. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file failure_controller.h
 *  Header of Controller class
 *
 */
#pragma once

#include <px4_platform_common/app.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/drone_state.h>
#include <vector>
#include <cmath>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

// Quaternion structure
typedef struct {
    double w;  // scalar part
    double x;  // vector components
    double y;
    double z;
} Quaternion;

// 3D Vector structure
typedef struct {
    double x;
    double y;
    double z;
} Vector3;

class Controller
{
public:
    Controller() {}
    ~Controller() {}

    int main(int detected_motor);

    static px4::AppState appState; /* track requests to terminate app */

    double* quaternionToRPY (double qw, double qx, double qy, double qz);
    Vector vectorFRD2Plus (Vector v);
    double* vectorFRD2Plus (double* v);

    Vector AorBodyFrame (double* q, double* N);
    double* rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate);
    double calculateNorm(Vector vec);

    void innerLoop();
    void outerLoop();

    Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);
    Vector3 transform_vector_to_child_frame(Vector3 parent_vector, Quaternion q);
    Quaternion quaternion_conjugate(Quaternion q);
    Quaternion vector_to_quaternion(Vector3 v);
    Vector3 quaternion_to_vector(Quaternion q);

    drone_state_s state_data = {};  // Zero-initialized
    orb_advert_t drone_state_publisher = orb_advertise(ORB_ID(drone_state), &state_data);

    double nz_bar = 0.966399726135702;
    double ny_bar = 0.25704390544193;
    double f2_bar = 6.300532966498725;
    double fmax = 14.1328;
    // double alpha = 1;

    // double nz_bar = 0.9906986385217392;
    // double ny_bar = 0.13607427248078469;
    // double f2_bar = 7.126586433828183;
    // double fmax = 13.14;
    double nx_bar = 0.0;
    double mass = 1.535;
    double f_sigma = 10.0;            //initialisation

    double transformationMatrix[3][3] =
    {
        {(1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {-1.0 * (1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {0.0, 0.0, -1.0}
    };

    Vector kp_gain = {0.0, 0.0, 6};
    Vector damping_const = {0.7, 0.7, 0.7};


    bool init_landing = false;

private:

};
