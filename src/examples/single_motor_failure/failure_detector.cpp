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
 * @file  failure_detector.cpp
 * Implementation of Detector Class
 * Detects motor failure using adaptive thresholding on the accelation values
 */




#include "failure_detector.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <array>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_odometry.h>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

px4::AppState Detector::appState;

Vector nat_freq = {1.5, 1.5, 3};
Vector damping_const = {0.7, 0.7, 0.7};
Vector desired_position = {0.0 , 0.0 , -10};
Vector current_position = {0.0 , 0.0 , 0.0};
Vector current_position_plus = {0.0 , 0.0 , 0.0};
Vector current_velocity_plus = {0.0 , 0.0 , 0.0};
Vector desired_position_plus = {0.0 , 0.0 , 0.0};
Vector desired_accel_plus = {0.0 , 0.0 , 0.0};
Vector pos_deviation = {0.0 , 0.0 , 0.0};
Vector normal_body_frame = {0.0, 0.0, 0.0};
Vector q_inverse ={0.0, 0.0, 0.0, 0.0};

double transformationMatrix[3][3] =
{
    {(1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
    {-1.0 * (1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
    {0.0, 0.0, -1.0}
};

double v_rot[3] = {0.0, 0.0, 0.0};
void Detector::vectorFRD2Plus (Vector &v){
    v_rot[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    v_rot[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    v_rot[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];
    for (int i = 0; i < 3; i++)
    {
        v[i]=v_rot[i];
    }
}

double Detector::calculateNorm(Vector vec) {
    double sum = 0.0;
    for (double component : vec) {
        sum += component * component;
    }
    return abs(std::sqrt(sum));
}

Vector Detector::hamiltonianProduct (Vector &q1, Vector &q2){
    Vector hamiltonian_product = {0.0, 0.0, 0.0, 0.0};
    hamiltonian_product[0] = (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    hamiltonian_product[1] = (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    hamiltonian_product[2] = (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    hamiltonian_product[3] = (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
    return hamiltonian_product;
}

Vector new_v = {0.0, 0.0, 0.0};
Vector v_q  = {0.0, 0.0, 0.0, 0.0};
Vector new_v_q = {0.0, 0.0, 0.0, 0.0};
Vector q_dash = {0.0, 0.0, 0.0, 0.0};
Vector Detector::vectorQuaternionTransformation (Vector &q, Vector &v){
    v_q[0] = 0;
    v_q[1] = v[0];
    v_q[2] = v[1];
    v_q[3] = v[2];
    q_dash[0] = q[0];
    q_dash[1] = -1.0 * q[1];
    q_dash[2] = -1.0 * q[2];
    q_dash[3] = -1.0 * q[3];
    new_v_q = hamiltonianProduct(q, v_q);
    new_v_q = hamiltonianProduct(new_v_q, q_dash);
    new_v[0] = new_v_q[1];
    new_v[1] = new_v_q[2];
    new_v[2] = new_v_q[3];
    return new_v;
}

void Detector::AorBodyFrame (Vector &q, Vector &N){
    vectorFRD2Plus(N);
    normal_body_frame[0] = N[0];
    normal_body_frame[1] = N[1];
    normal_body_frame[2] = N[2];
    // normal_body_frame = vectorFRD2Plus(vectorQuaternionTransformation(q, normal_body_frame));
    normal_body_frame = vectorQuaternionTransformation(q, normal_body_frame);
    vectorFRD2Plus(normal_body_frame);
}


void Detector::print_primary_axis(const int vehicle_odometry_fd, vehicle_odometry_s odometry){
    orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
    current_position[0] = (double)odometry.position[0];
    current_position[1] = (double)odometry.position[1];
    current_position[2] = (double)odometry.position[2];

    current_position_plus = current_position;
    vectorFRD2Plus(current_position_plus);
    desired_position_plus = desired_position;
    vectorFRD2Plus(desired_position_plus);
    current_velocity_plus = {(double)odometry.velocity[0], (double)odometry.velocity[1], (double)odometry.velocity[2]};
    vectorFRD2Plus(current_velocity_plus);

    pos_deviation[0] = current_position_plus[0] - desired_position_plus[0];
    pos_deviation[1] = current_position_plus[1] - desired_position_plus[1];
    pos_deviation[2] = current_position_plus[2] - desired_position_plus[2];

    desired_accel_plus[0] = -2*damping_const[0]*nat_freq[0]*(double)current_velocity_plus[0] - nat_freq[0]*nat_freq[0]*pos_deviation[0];
    desired_accel_plus[1] = -2*damping_const[1]*nat_freq[1]*(double)current_velocity_plus[1] - nat_freq[1]*nat_freq[1]*pos_deviation[1];
    desired_accel_plus[2] = -2*damping_const[2]*nat_freq[2]*(double)current_velocity_plus[2] - nat_freq[2]*nat_freq[2]*pos_deviation[2];

    desired_accel_plus[2]+= 9.8; // d_des - g

    desired_accel_plus[0] = desired_accel_plus[0] * (d_mass / d_nz_bar);
    desired_accel_plus[1] = desired_accel_plus[1] * (d_mass / d_nz_bar);
    desired_accel_plus[2] = desired_accel_plus[2] * (d_mass / d_nz_bar);

    d_f_sigma = calculateNorm(desired_accel_plus);
    desired_accel_plus[0] = desired_accel_plus[0] / d_f_sigma;
    desired_accel_plus[1] = desired_accel_plus[1] / d_f_sigma;
    desired_accel_plus[2] = desired_accel_plus[2] / d_f_sigma;

    // printf("---------------\nCurrent primary axis in ground frame: {%f, %f, %f}\n", desired_accel_plus[0], desired_accel_plus[1], desired_accel_plus[2]);
    d_state_data.p_s = (double)desired_accel_plus[0];
    d_state_data.q_s = (double)desired_accel_plus[1];
    d_state_data.r_s = (double)desired_accel_plus[2];

    q_inverse[0] = odometry.q[0];
    q_inverse[1] = -1.0 * (double)odometry.q[1];
    q_inverse[2] = -1.0 * (double)odometry.q[2];
    q_inverse[3] = -1.0 * (double)odometry.q[3];
    AorBodyFrame(q_inverse, desired_accel_plus);

    d_state_data.timestamp = odometry.timestamp;
    d_state_data.u1 = 0.0;
    d_state_data.u2 = 0.0;
    d_state_data.n_x = (double)normal_body_frame[0];
    d_state_data.n_y = (double)normal_body_frame[1];
    d_state_data.n_z = (double)normal_body_frame[2];
    d_state_data.f1 = (double)d_f_sigma;
    d_state_data.f2 = 0.0;
    d_state_data.f3 = 0.0;
    d_state_data.roll_rad = 0.0;
    d_state_data.pitch_rad = 0.0;
    d_state_data.yaw_rad = 0.0;
    d_state_data.x = (double)odometry.position[0];
    d_state_data.y = (double)odometry.position[1];
    d_state_data.z = (double)odometry.position[2];
    orb_publish(ORB_ID(drone_state), d_drone_state_publisher, &d_state_data);
    // printf("Current primary axis in body frame: {%f, %f, %f}\n\n", normal_body_frame[0], normal_body_frame[1], normal_body_frame[2]);
}


int Detector::main(){
    appState.setRunning(true);
    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;
    while (!appState.exitRequested()) {
        px4_sleep(0.001);
        odometryUpdate(vehicle_odometry_fd, odometry);
        // print_primary_axis(vehicle_odometry_fd, odometry);
         if (detected_motor_failure == -1) {
            detectFailure();
           }
         else{
            return (detected_motor_failure+1);
         }
    }
    return 0;
}

// Calculates roll, pitch, yaw from quaternion
double* Detector::quaternionToRPY (double qw, double qx, double qy, double qz){

    double* rpy = new double[3];


   // Roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    double qroll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    double qpitch;
    if (std::abs(sinp) >= 1)
        qpitch = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
        qpitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double qyaw = std::atan2(siny_cosp, cosy_cosp);

    rpy[0] = qroll;
    rpy[1] = qpitch;
    rpy[2] = qyaw;

    return rpy;

}

// Calculates x, y, z accelerations
    void Detector::calculateAccelerations(int timestmp) {
        if (prev_timestamp <= 0.0) {
            ax = ay = az = 0.0;
            return;
        }

        double dt = (timestmp - prev_timestamp) * 1e-6;
        if (dt > 0) {
            ax = (vx - prev_vx) / dt;
            ay = (vy - prev_vy) / dt;
            az = (vz - prev_vz) / dt;
        }
    }

// Calculate normalised angles
    double Detector::normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

// Calculates angular rate
    double Detector::calculateAngularRate(double current, double previous, double dt) {
        double diff = normalizeAngle(current - previous);
        return dt > 0 ? diff / dt : 0.0;
    }

// Calculates p,q,r rates
    void Detector::calculateAttitudeRates(int timestmp, double roll_rad, double pitch_rad, double yaw_rad) {
        if (prev_timestamp <= 0.0) {
            roll_rate = pitch_rate = yaw_rate = 0.0;
            return;
        }

        double dt = (timestmp - prev_timestamp) * 1e-6;
        if (dt > 0) {
            double roll_rate_rad = calculateAngularRate(roll_rad, prev_roll_rad, dt);
            double pitch_rate_rad = calculateAngularRate(pitch_rad, prev_pitch_rad, dt);
            double yaw_rate_rad = calculateAngularRate(yaw_rad, prev_yaw_rad, dt);

            roll_rate = roll_rate_rad * (180.0 / M_PI);
            pitch_rate = pitch_rate_rad * (180.0 / M_PI);
            yaw_rate = yaw_rate_rad * (180.0 / M_PI);
        }
    }


    void Detector::detectFailure() {

        //For calculating average over 20 values
        if (last_20.size() < 20) {
            last_20.push_back({az, roll_rate, pitch_rate, yaw_rate});
        } else {
            last_20.erase(last_20.begin());
            last_20.push_back({az, roll_rate, pitch_rate, yaw_rate});
        }

        double avg_az = 0.0, avg_roll_rate = 0.0, avg_pitch_rate = 0.0, avg_yaw_rate = 0.0;
        for (const auto& data : last_20) {
            avg_az += data[0];
            avg_roll_rate += data[1];
            avg_pitch_rate += data[2];
            avg_yaw_rate += data[3];
        }
        int size = last_20.size();
        avg_az /= size;
        avg_roll_rate /= size;
        avg_pitch_rate /= size;
        avg_yaw_rate /= size;

        // Calculating average slope for az and roll_rate

        double az_slope = 0.0, roll_rate_slope = 0.0;
        if (size >= 5) {
            az_slope = (last_20[size - 1][0] - last_20[size - 5][0]) / 5.0;
            roll_rate_slope = (last_20[size - 1][1] - last_20[size - 5][1]) / 5.0;
        }

        // Dynamic threshold adjustment based on slope
        if (az_slope > 0) {
            threshold_acceleration = std::max(1.5, 3.0 - az_slope * slope_sensitivity_factor_az);
        } else {
            threshold_acceleration = 3.0;
        }

        threshold_roll_rate = std::max(25.0, 50.0 - abs(roll_rate_slope) * slope_sensitivity_factor_roll_rate);
        // threshold_acceleration = 3.0;
        // threshold_roll_rate = 50.0;
        // Motor failure detection
        if (avg_az >= threshold_acceleration || abs(avg_roll_rate)>threshold_roll_rate) {
            // failure_detected_timestamp = timestamp;
            if (avg_pitch_rate < 0 && avg_yaw_rate < 0) {
                detected_motor_failure = 0;
            }
            else if (avg_pitch_rate > 0 && avg_yaw_rate < 0) {
                detected_motor_failure = 1;
            }
            else if (avg_pitch_rate < 0 && avg_yaw_rate > 0) {
                detected_motor_failure = 2;
            }
            else if (avg_pitch_rate > 0 && avg_yaw_rate > 0) {
                detected_motor_failure = 3;
            }
        }
    }

// Updates odometry to get latest values
    void Detector::odometryUpdate(int vehicle_odometry_fd, vehicle_odometry_s &odometry) {

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);

        double current_timestamp = odometry.timestamp;

        timestamp = current_timestamp;

        x = odometry.position[0];
        y = odometry.position[1];
        z = odometry.position[2];

        vx = odometry.velocity[0];
        vy = odometry.velocity[1];
        vz = odometry.velocity[2];

        calculateAccelerations(current_timestamp);

        double* rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        double roll_rad = rpy_rad[0];
        double pitch_rad = rpy_rad[1];
        double yaw_rad = rpy_rad[2];

        calculateAttitudeRates(current_timestamp, roll_rad, pitch_rad, yaw_rad);

        roll = roll_rad * (180.0 / M_PI);
        pitch = pitch_rad * (180.0 / M_PI);
        yaw = yaw_rad * (180.0 / M_PI);

        prev_vx = vx;
        prev_vy = vy;
        prev_vz = vz;
        prev_roll_rad = roll_rad;
        prev_pitch_rad = pitch_rad;
        prev_yaw_rad = yaw_rad;
        prev_timestamp = timestamp;

    }
