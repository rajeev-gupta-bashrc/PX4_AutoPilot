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
 * @file failure_controller.cpp
 *  Implementation of Controller class
 *  Controls in case of motor failure
 */

// #pragma once



// #include "failure_controller.h"
#include "lqr.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <px4_platform_common/app.h>

// #include <pthread.h>
// #include <threads.h>

#include <uORB/topics/custom_actuator_motors.h>
#include <uORB/topics/transfer_control.h>



px4::AppState appState;  /* track requests to terminate app */


//     // Shared state structure
// struct State {
//     // double desired_position = 0.0;  // Outer loop target
//     // double current_position = 0.0; // System state
//     double desired_velocity = 0.0; // Inner loop target
//     double current_velocity = 0.0; // System state

//     int check_flag = 0;
//     int detected_motor;

//     double nx_des;
//     double ny_des;

//     float Fsum = 0.8;
// };

// State shared_state;
// pthread_mutex_t shared_state_mutex; // Mutex to protect shared state


// Outer loop: Position control
// void* Controller::outerLoop(void* ) {

//     // {
//     //     std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
//     //     int detected_motor = shared_state.detected_motor;
//     // }
//     // int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
//     // vehicle_odometry_s odometry;

//     int vehicle_attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
//     vehicle_attitude_s attitude;

//     int vehicle_local_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));
//     vehicle_local_position_s local_position;

//     int vehicle_angular_velocity_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
//     vehicle_angular_velocity_s angular_velocity;

//     double desired_position[] = {0.0 , 0.0 , -10.0};
//     double current_position[] = {0.0 , 0.0 , 0.0};
//     double pos_deviation[] = {0.0 , 0.0 , 0.0};

//     double desired_accel[]  = {0.0 , 0.0 , 0.0};

//     double nx_des;
//     double ny_des;

//     double nat_freq = 1;
//     double damping_const = 0.7;

//     double* rpy_rad;

//     double roll_rad;
//     double pitch_rad;
//     double yaw_rad;

//     // double* rpy_plus;


double* quaternionToRPY (double qw, double qx, double qy, double qz){

    double* rpy = new double[3];


   // Roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    double qroll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    double qpitch;
    if (abs(sinp) >= 1)
        qpitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
        qpitch = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    double qyaw = atan2(siny_cosp, cosy_cosp);

    rpy[0] = qroll;
    rpy[1] = qpitch;
    rpy[2] = qyaw;

    return rpy;

}

double* hamiltonianProduct (double* q1, double* q2){
    double* hamiltonian_product = new double[4];
    hamiltonian_product[0] = (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    hamiltonian_product[1] = (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    hamiltonian_product[2] = (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    hamiltonian_product[3] = (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
    return hamiltonian_product;
}

double* vectorQuaternionTransformation (double* q, double* v){
    double* new_v = new double[3];
    double* v_q  = new double[4];
    double* new_v_q = new double[4];
    double* q_dash = new double[4];
    v_q[0] = 0;
    v_q[1] = v[0];
    v_q[2] = v[1];
    v_q[3] = v[2];
    q_dash[0] = q[0];
    q_dash[1] = -1.0 * q[1];
    q_dash[2] = -1.0 * q[2];
    q_dash[3] = -1.0 * q[3];

    new_v_q = hamiltonianProduct(hamiltonianProduct(q, v_q), q_dash);

    new_v[0] = new_v_q[1];
    new_v[1] = new_v_q[2];
    new_v[2] = new_v_q[3];

    delete[] v_q;
    delete[] new_v_q;
    delete[] q_dash;

    return new_v;
}

double* vectorFRD2Plus (double* v){
    double* new_v = new double[3];
    double transformationMatrix[3][3] =
    {
        {(1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {-1.0 * (1/sqrt(2)), -1.0 * (1/sqrt(2)), 0.0},
        {0.0, 0.0, -1.0}

    };
    new_v[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    new_v[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    new_v[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];

    return new_v;
}

// Calculates unit vector along normal
double* vectorAlongNormal (double* q){
    double* vector_along_normal = new double[3];
    vector_along_normal[0] = 0.0;
    vector_along_normal[1] = 0.0;
    vector_along_normal[2] = -1.0;

    vector_along_normal = vectorFRD2Plus(vectorQuaternionTransformation(q, vector_along_normal));

    return vector_along_normal;
}

double* AorBodyFrame (double* q, double* N){

    double* vector_along_normal = new double[3];
    double* N_frd = new double[3];
    N_frd = vectorFRD2Plus(N);
    vector_along_normal[0] = N_frd[0];
    vector_along_normal[1] = N_frd[1];
    vector_along_normal[2] = N_frd[2];

    vector_along_normal = vectorFRD2Plus(vectorQuaternionTransformation(q, vector_along_normal));

    return vector_along_normal;
}
// Converts rpy_rate to plus configuration
double* rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate){
    double roll_rate_rad = roll_rate;// * (M_PI / 180.0);
    double pitch_rate_rad = pitch_rate;// * (M_PI / 180.0);
    double yaw_rate_rad = yaw_rate;// * (M_PI / 180.0);

    double* rpy_rate_plus = new double[3];

    rpy_rate_plus[0] = cos(M_PI / 4) * roll_rate_rad - sin(M_PI / 4) * pitch_rate_rad;
    rpy_rate_plus[1] = - cos(M_PI / 4) * pitch_rate_rad - sin(M_PI / 4) * roll_rate_rad;
    rpy_rate_plus[2] = -1.0 * yaw_rate_rad;

    return rpy_rate_plus;
}

// Function to calculate the inverse of the rotation matrix (transpose of R)
double** calculateRotationMatrixInverse(double roll, double pitch, double yaw) {
    // Allocate memory for a 3x3 matrix
    double** inverse = (double**)malloc(3 * sizeof(double*));
    for (size_t i = 0; i < 3; ++i) {
        inverse[i] = (double*)malloc(3 * sizeof(double));  // Allocate each row of the matrix
    }

    // Precompute sines and cosines of the angles
    double c_roll = cos(roll);
    double s_roll = sin(roll);
    double c_pitch = cos(pitch);
    double s_pitch = sin(pitch);
    double c_yaw = cos(yaw);
    double s_yaw = sin(yaw);

    // Inverse of the rotation matrix (transpose)
    inverse[0][0] = c_pitch * c_yaw;
    inverse[0][1] = c_pitch * s_yaw;
    inverse[0][2] = -s_pitch;

    inverse[1][0] = c_yaw * s_pitch * s_roll - c_roll * s_yaw;
    inverse[1][1] = s_yaw * s_pitch * s_roll + c_roll * c_yaw;
    inverse[1][2] = c_pitch * s_roll;

    inverse[2][0] = c_roll * c_yaw * s_pitch + s_roll * s_yaw;
    inverse[2][1] = c_roll * s_pitch * s_yaw - s_roll * c_yaw;
    inverse[2][2] = c_pitch * c_roll;

    return inverse;
}

// Function to free the allocated memory for the 2D matrix
void freeMatrix(double** matrix) {
    for (size_t i = 0; i < 3; ++i) {
        free(matrix[i]);  // Free each row
    }
    free(matrix);  // Free the array of pointers
}

// Matrix multiplication: 3x3 matrix * 3x1 vector = 3x1 vector
void multiply(double** matrix, double* vector, double* result) {
    for (size_t i = 0; i < 3; ++i) {
        result[i] = 0;  // Initialize result[i] to 0
        for (size_t j = 0; j < 3; ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}


//     while (true) {


//         // orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
//         orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_fd, &attitude);
//         orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_fd, &local_position);
//         orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_fd, &angular_velocity);

//         rpy_rad = Controller::quaternionToRPY(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);

//         roll_rad = rpy_rad[0];
//         pitch_rad = rpy_rad[1];
//         yaw_rad = rpy_rad[2];

//         // current_position[0] = odometry.position[0];
//         // current_position[1] = odometry.position[1];
//         // current_position[2] = odometry.position[2];

//         current_position[0] = local_position.x;
//         current_position[1] = local_position.y;
//         current_position[2] = local_position.z;

//         pos_deviation[0] = current_position[0] - desired_position[0];
//         pos_deviation[1] = current_position[1] - desired_position[1];
//         pos_deviation[2] = current_position[2] - desired_position[2];

//         desired_accel[0] = -2*damping_const*nat_freq*(double)local_position.vx - nat_freq*nat_freq*pos_deviation[0];
//         desired_accel[1] = -2*damping_const*nat_freq*(double)local_position.vy - nat_freq*nat_freq*pos_deviation[1];
//         desired_accel[2] = -2*damping_const*nat_freq*(double)local_position.vz - nat_freq*nat_freq*pos_deviation[2];

//         desired_accel[2]+= 9.8;

//         // rpy_plus = rpy_rate_plus(roll_rad, pitch_rad , yaw_rad);

//         // Vector ndes = multiply(calculateRotationMatrixInverse(rpy_plus[0],rpy_plus[1],rpy_plus[2]), desired_accel);
//         double** rotation_matrix_inverse = calculateRotationMatrixInverse(roll_rad, pitch_rad, yaw_rad);
//         double ndes[3];
//         multiply(rotation_matrix_inverse, desired_accel, ndes);
//         freeMatrix(rotation_matrix_inverse);

//         // double ndes[] = multiply(calculateRotationMatrixInverse(roll_rad,pitch_rad,yaw_rad), desired_accel);

//         nx_des = (ndes[0]*2)/ (0.9583*22.075);
//         ny_des = (ndes[1]*2)/ (0.9583*22.075);

//         pthread_mutex_lock(&shared_state_mutex);

//         shared_state.nx_des = nx_des;
//         shared_state.ny_des = ny_des;

//         // Unlock the mutex
//         pthread_mutex_unlock(&shared_state_mutex);

//         // Unlock automatically when lock_guard goes out of scope
//         usleep(2000); // Sleep for 100 ms
//     }

//  return nullptr;

// }



// Inner loop: Velocity control
extern "C" __EXPORT int smf_module_main(int argc, char *argv[]);
int smf_module_main(int argc, char *argv[]){
    // while (true) {

    //     {
    //     std::lock_guard<std::mutex> lock(state_mutex);
    //     state.check_flag++;
    //     printf("Inner Loop flag = %d \n",state.check_flag);
    //     }


    //     // Mutex is released automatically when lock_guard goes out of scope

    //     usleep(500000); // Sleep for 10 ms
    // }
    // int detected_motor = 1;
    // float Fsum = 0.8;

    // pthread_mutex_lock(&shared_state_mutex);

    // detected_motor = shared_state.detected_motor;
    // double fsum = (double)shared_state.Fsum*9.8;

    // // Unlock the mutex
    // pthread_mutex_unlock(&shared_state_mutex);
    PX4_INFO("Injected Failure at motor 1\n");
    int detected_motor = 1;

    int detected_motor_index = detected_motor - 1;
    int opposite_motor_index  = (detected_motor_index < 2) ? (1 - detected_motor_index) : (5 - detected_motor_index);
    int diagonally_working_1 =  (detected_motor_index < 2) ? 2 : 0;
    int diagonally_working_2 =  (detected_motor_index < 2) ? 3 : 1;


    // actuator_motors_s act = {};  // Zero-initialized
    // orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);
    transfer_control_s _tfc = {};  // Zero-initialized
    orb_advert_t transfer_control_publisher = orb_advertise(ORB_ID(transfer_control), &_tfc);
    custom_actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t custom_actuator_motors_publisher = orb_advertise(ORB_ID(custom_actuator_motors), &act);

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    int vehicle_attitude_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    vehicle_attitude_s attitude;

    int vehicle_local_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    vehicle_local_position_s local_position;

    int vehicle_angular_velocity_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
    vehicle_angular_velocity_s angular_velocity;


    // vehicle_control_mode_s ctrlmode = {};
    //  orb_advert_t ctrlmode_pub_fd = orb_advertise(ORB_ID(vehicle_control_mode), &ctrlmode);


    // double* rpy_rad;

    // double roll_rad;
    // double pitch_rad;
    // double yaw_rad;

    double* normal;
    double n_x;
    double n_y;
    // double n_z;

    time_t start_time = time(NULL);
    time_t current_time;
    int timeout_seconds = 3;  // 10 seconds timeout
    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;
    // double r_s;

    double u1;
    double u2;
    double f1;                  //      Anticlockwise first motor to the failed
    double f2;                  //      Opposite to failed modtor
    double f3;                  //      Clockwise first motor to the failed

    LQR lqr;

    double current_state[4];
    double inputs[2];

    // double signal = 0.0;


    // printf("Controller Initiated\n");
    PX4_INFO("Controller Initiated\n\n");

    while (!appState.exitRequested()) {



//         flag_control_auto_enabled			# true if onboard autopilot should act
// bool flag_control_offboard_enabled		# true if offboard control should be used
// bool flag_control_position_enabled		# true if position is controlled
// bool flag_control_velocity_enabled		# true if horizontal velocity (implies direction) is controlled
// bool flag_control_altitude_enabled		# true if altitude is controlled
// bool flag_control_climb_rate_enabled		# true if climb rate is controlled
// bool flag_control_acceleration_enabled		# true if acceleration is controlled
// bool flag_control_attitude_enabled		# true if attitude stabilization is mixed in
// bool flag_control_rates_enabled			# true if rates are stabilized
// bool flag_control_allocation_enabled
        // ctrlmode.flag_control_auto_enabled = false;
        // ctrlmode.flag_control_offboard_enabled = false;
        // ctrlmode.flag_control_position_enabled = false;
        // ctrlmode.flag_control_velocity_enabled = false;
        // ctrlmode.flag_control_altitude_enabled = false;
        // ctrlmode.flag_control_climb_rate_enabled = false;
        // ctrlmode.flag_control_acceleration_enabled = false;
        // ctrlmode.flag_control_attitude_enabled = false;
        // ctrlmode.flag_control_rates_enabled = false;
        // ctrlmode.flag_control_allocation_enabled = false;
        // ctrlmode.flag_control_termination_enabled = true;

        // orb_publish(ORB_ID(vehicle_control_mode), ctrlmode_pub_fd, &ctrlmode);




        // px4_sleep(0.5);
        usleep(500);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);
        orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_fd, &local_position);
        orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_fd, &attitude);
        orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_fd, &angular_velocity);


        // rpy_rad = quaternionToRPY(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);

        // roll_rad = rpy_rad[0];
        // pitch_rad = rpy_rad[1];
        // yaw_rad = rpy_rad[2];

        roll_rate = angular_velocity.xyz[0];
        pitch_rate = angular_velocity.xyz[1];
        yaw_rate = angular_velocity.xyz[2];

        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];
        // r_s = rpy_plus_rate[2];

        double* q_inverse = new double[4];
        q_inverse[0] = attitude.q[0];
        q_inverse[1] = -1.0 * (double)attitude.q[1];
        q_inverse[2] = -1.0 * (double)attitude.q[2];
        q_inverse[3] = -1.0 * (double)attitude.q[3];
        double* axis_of_rotation = new double [3];
            axis_of_rotation[0] = 0;
            axis_of_rotation[1] = 0;
            axis_of_rotation[2] = 1;

        normal = AorBodyFrame(q_inverse, axis_of_rotation);
        n_x = normal[0];
        n_y = normal[1];
        // n_z = normal[2];

        current_state[0] = p_s;

        current_state[1] = q_s;
        current_state[2] = n_x;
        current_state[3] = n_y;

    // pthread_mutex_lock(&shared_state_mutex);


    lqr.getControlInputs(current_state, detected_motor, inputs);     // Apply LQR controller
    // Unlock the mutex
    // pthread_mutex_unlock(&shared_state_mutex);


        u1 = inputs[0];
        u2 = inputs[1];

        double f1_bar=0.0, f2_bar=0.0, f3_bar=0.0;
        double f_max = 4.580; //maximum thrust available per motor for maxRotVel = 1000
        double fsum = 1.01*9.8;
        // double fsum = (double)Fsum*9.8;
        double rho = 0.4;

        f1_bar = fsum * (1 / (rho + (2 )));
        f2_bar = fsum * (rho / (rho + (2 )));
        f3_bar = fsum * (1 / (rho + (2 )));

        f3 = (2*f3_bar + u1 - u2)/2;
        f2 = f2_bar + u2;
        f1 = (2*f1_bar - u1 - u2)/2;

        (f1 < 0) ? f1 = f_max : (f1 > f_max) ? f1 = f_max : f1 = f1;
        (f2 < 0) ? f2 = f_max : (f2 > f_max) ? f2 = f_max : f2 = f2;
        (f3 < 0) ? f3 = f_max : (f3 > f_max) ? f3 = f_max : f3 = f3;


        if( abs(local_position.z) < 4.0f){

        // if(detected_motor == 1){
        act.control[0] = (float)nan("1");
        act.control[1] = f2/ f_max;
        act.control[2] = f1/ f_max;
        act.control[3] = f3/ f_max;

        PX4_INFO("Three motor control: ");
        printf("u1 = %f, u2 = %f , f1 = %f , f2 = %f , f3 = %f ,yaw_rate = %f   \n",u1, u2 ,f1, f2, f3, yaw_rate);
        // printf("state p = %f, q = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, n_x, n_y);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %lld\n", roll_rad, pitch_rad, yaw_rad, local_position.timestamp);
        // printf("x = %f, y = %f, z = %f   \n", (double)local_position.x,(double)local_position.y,(double)local_position.z);
        // printf("nx_des = %f, ny_des = %f, nz_des = %f");
        // printf("fsum = %f", fsum);
        }

        else if(abs(local_position.z) > 4.0f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = (float)0.5;
        act.control[diagonally_working_2] = (float)0.5;

        // act.control[0] = (float)nan("1");
        // act.control[1] = (float)nan("1");
        // act.control[2] = (float)0.6;
        // act.control[3] = (float)nan("1");
        PX4_INFO("Two motors control");
        printf("f1 = %f , f2 = %f , f3 = %f ,yaw_rate = %f   \n",f1, f2, f3, yaw_rate);
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, r_s, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %lld\n", roll_rad, pitch_rad, yaw_rad, local_position.timestamp);
        }

        else if(local_position.z < 0.3f)
        {
        act.control[0] = (float)nan("1");
        act.control[1] = (float)nan("1");
        act.control[2] = (float)nan("1");
        act.control[3] = (float)nan("1");

        PX4_WARN("Low altitude Warning, Engaging Kill\n");
        printf("z: %f < 0.3\n", (double)local_position.z);

        }

        // printf("Alt = %f \n" , (double)odometry.position[2]);


        // orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
        _tfc.to_manual = true;
        _tfc.timestamp = odometry.timestamp;
        orb_publish(ORB_ID(custom_actuator_motors), custom_actuator_motors_publisher, &act);
        orb_publish(ORB_ID(transfer_control), transfer_control_publisher, &_tfc);

        // printf("global position: %f\n", (double)local_position.z);
        // pthread_mutex_lock(&shared_state_mutex);

        // shared_state.nx_des = 0.0;
        // shared_state.ny_des = 0.003001140204869386;
        // Unlock the mutex
        current_time = time(NULL);
        if (current_time - start_time >= timeout_seconds) {
            break;  // Exit loop after 10 seconds
        }
        // pthread_mutex_unlock(&shared_state_mutex);
    }


     return 0;
}

// int Controller::main(int detected_motor, float Fsum){
//     printf("before appstate\n");
//     appState.setRunning(true);

//     printf("before_inside_mutex\n");
    // pthread_mutex_lock(&shared_state_mutex);
    // printf("inside_mutex\n");
    // shared_state.detected_motor = detected_motor;
    // shared_state.Fsum = Fsum;

    // Unlock the mutex
    // pthread_mutex_unlock(&shared_state_mutex);
    // printf("outside_mutex\n");

    // pthread_t outer_thread, inner_thread;
    // pthread_t inner_thread;

    // Initialize the mutex
    // if (pthread_mutex_init(&shared_state_mutex, NULL) != 0) {
    //     perror("Mutex initialization failed");
    //     return 1;
    // }

    // // Create the updater thread
    // if (pthread_create(&outer_thread, NULL, outerLoop, NULL) != 0) {
    //     perror("Failed to create outerLoop thread");
    //     return 1;
    // }

    // Create the reader thread
    // if (pthread_create(&inner_thread, NULL, innerLoop, NULL) != 0) {
    //     perror("Failed to create innerLoop thread");
    //     return 1;
    // }

    // Wait for both threads to complete
    // pthread_join(outer_thread, NULL);
    // pthread_join(inner_thread, NULL);

    // Destroy the mutex
    // pthread_mutex_destroy(&shared_state_mutex);

//     printf("Program completed successfully.\n");

//     return 0;
// }


