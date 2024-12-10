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




#include "failure_controller.h"
#include "lqr.h"
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
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/drone_state.h>

#include <thread>
#include <mutex>




using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState Controller::appState;  /* track requests to terminate app */


    // Shared state structure
struct State {
    // double desired_position = 0.0;  // Outer loop target
    // double current_position = 0.0; // System state
    double desired_velocity = 0.0; // Inner loop target
    double current_velocity = 0.0; // System state

    int check_flag = 0;
    int detected_motor;

    double alt_error;////

    double nx_des = 0.0;
    double ny_des = 0.0;
    double nz_des = -1.0;

    double fsum = 10.0;           //initialisation
};

State shared_state;
std::mutex shared_state_mutex; // Mutex to protect shared state


// Outer loop: Position control
void Controller::outerLoop() {

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    Vector desired_position = {0.0 , 0.0 , -10};
    Vector current_position = {0.0 , 0.0 , 0.0};
    Vector pos_deviation = {0.0 , 0.0 , 0.0};

    Vector desired_accel  = {0.0 , 0.0 , 0.0};

    double alt_error;
    double fsum=0.0;

    // Vector nat_freq = {1, 1, 1};
    // Vector damping_const = {0.7, 0.7, 0.7};
    // Vector nat_freq = {1.5, 1.5, 2.5};           //good
    // Vector nat_freq = {1.5, 1.5, 3};
    // Vector nat_freq = {1.5, 1.5, 3};
    // Vector kp_gain = {2.25, 2.25, 9};


    // kp_gain = {0.0, 0.0, 6};
    // kp_gain = {0.0, 0.0, 6};
    kp_gain = {0.0, 0.0, 4};
    // kp_gain = {1.0, 1.0, 7};
    damping_const = {0.7, 0.7, 0.0};
    // damping_const = {0, 0, 0};


    while (true) {


        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);

        // rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        current_position[0] = (double)odometry.position[0];
        current_position[1] = (double)odometry.position[1];
        current_position[2] = (double)odometry.position[2];


        pos_deviation[0] = current_position[0] - desired_position[0];
        pos_deviation[1] = current_position[1] - desired_position[1];
        pos_deviation[2] = current_position[2] - desired_position[2];

        desired_accel[0] = -2*damping_const[0]*sqrt(kp_gain[0])*(double)odometry.velocity[0] - kp_gain[0]*pos_deviation[0];
        desired_accel[1] = -2*damping_const[1]*sqrt(kp_gain[1])*(double)odometry.velocity[1] - kp_gain[1]*pos_deviation[1];
        desired_accel[2] = -2*damping_const[2]*sqrt(kp_gain[2])*(double)odometry.velocity[2] - kp_gain[2]*pos_deviation[2];
        // err[n]=e=des-pos[n]
        // err[n+1]=des-pos[n+1]
        // acc[n+1]=2*kd*(err[n+1]-err[n])/dt
        //

        desired_accel[2]-= 9.81; // d_des - g
        ///////////////////////////////////////////////////
        if (desired_accel[2] > -2){
            desired_position[2] = current_position[2]-1;
            pos_deviation[2] = current_position[2] - desired_position[2];
            desired_accel[2] = -2*damping_const[2]*sqrt(kp_gain[2])*(double)odometry.velocity[2] - kp_gain[2]*pos_deviation[2];
            // continue;
        }
        Vector ndes_g = desired_accel;
        ndes_g[0] = ndes_g[0] * (mass / nz_bar);
        ndes_g[1] = ndes_g[1] * (mass / nz_bar);
        ndes_g[2] = ndes_g[2] * (mass / nz_bar);

        fsum = calculateNorm(ndes_g);
        ndes_g[0] = ndes_g[0] / fsum;
        ndes_g[1] = ndes_g[1] / fsum;
        ndes_g[2] = ndes_g[2] / fsum;

        alt_error = pos_deviation[2];

        {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        // shared_state.nx_des = 0;        // shared_state.ny_des = 0;

        // shared_state.ny_des = 0;
        // shared_state.nz_des = 1;
        shared_state.nx_des = ndes_g[0];
        shared_state.ny_des = ndes_g[1];
        shared_state.nz_des = ndes_g[2];
        shared_state.fsum = fsum;

        shared_state.alt_error = alt_error;
        }

        // usleep(1000);
        // usleep(2000);            //best
        // usleep(4000);
        usleep(20000);
    }



}

// Inner loop: Velocity control
void Controller::innerLoop() {
    int detected_motor;
     {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        detected_motor = shared_state.detected_motor;
    }


    int detected_motor_index = detected_motor - 1;
    int opposite_motor_index  = (detected_motor_index < 2) ? (1 - detected_motor_index) : (5 - detected_motor_index);
    int diagonally_working_1 =  (detected_motor_index < 2) ? 2 : 0;
    int diagonally_working_2 =  (detected_motor_index < 2) ? 3 : 1;


    actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    double* rpy_rad;

    Vector normal_body_frame;
    double n_x;
    double n_y;
    double n_z;


    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;
    double r_s;

    double u1;
    double u2;
    double f1;                  //      Anticlockwise first motor to the failed
    double f2;                  //      Opposite to failed modtor
    double f3;                  //      Clockwise first motor to the failed

    LQR lqr;

    Vector current_state;
    Vector inputs;




    while (!appState.exitRequested()) {

        // px4_sleep(0.5);
        usleep(100);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);


        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        roll_rate = odometry.angular_velocity[0];
        pitch_rate = odometry.angular_velocity[1];
        yaw_rate = odometry.angular_velocity[2];

        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];
        r_s = rpy_plus_rate[2];

        double q_frd_body[4] = {0.0, 0.0, 0.0, 0.0};
        q_frd_body[0] = (double)odometry.q[0];
        q_frd_body[1] = (double)odometry.q[1];
        q_frd_body[2] = (double)odometry.q[2];
        q_frd_body[3] = (double)odometry.q[3];

        {
            std::lock_guard<std::mutex> lock(shared_state_mutex);
            double axis_of_rotation[3] = {0.0, 0.0, 0.0};
            // axis_of_rotation[0] = 0.0;
            // axis_of_rotation[1] = 0.0;
            // axis_of_rotation[2] = -1.0;
            axis_of_rotation[0] = shared_state.nx_des;
            axis_of_rotation[1] = shared_state.ny_des;
            axis_of_rotation[2] = shared_state.nz_des;
            f_sigma = shared_state.fsum;
            normal_body_frame = AorBodyFrame(q_frd_body, axis_of_rotation);
        }
        n_x = normal_body_frame[0];
        n_y = normal_body_frame[1];
        n_z = normal_body_frame[2];

        // current_state = {p_s, q_s, nx_bar, ny_bar};        // Get current state
        // inputs = lqr.getControlInputs(current_state, detected_motor, n_x, n_y);     // Apply LQR controller
        current_state = {p_s, q_s, n_x, n_y};        // Get current state
        inputs = lqr.getControlInputs(current_state, detected_motor, nx_bar, ny_bar);     // Apply LQR controller

        u1 = inputs[0];
        u2 = inputs[1];
        // u3 = inputs[2];


        // Thrust equations after solving input relation
        ///////////////////////////////////////////////////////////
        // f2 = u2 + 8.83/2;
        // f1 = ( 8.83 - (u1 + u2)/2 > 0) ? 8.83 - (u1 + u2)/2 : 0.0 ;
        // f3 = (8.83 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 8.83 + (u1 - u2)/2  ;
        // // for outer loop control
        f1 = (f_sigma - (u1+u2+f2_bar)) / 2;
        f2 = u2 + f2_bar;
        f3 = f1 + u1;
        ////////////////////////////////////////////////////////////

        //////////////////////////////
        // f1 = u1 + shared_state.fsum * (1.28 / (1 + (2 * 1.28)));
        // f2 = u2 + shared_state.fsum * (1 / (1 + (2 * 1.28)));
        // f3 = u3 + shared_state.fsum * (1.28 / (1 + (2 * 1.28)));

        (f1 < 0) ? f1 = 0 : (f1 > fmax) ? f1 = fmax : f1 = f1;
        (f2 < 0) ? f2 = 0 : (f2 > fmax) ? f2 = fmax : f2 = f2;
        (f3 < 0) ? f3 = 0 : (f3 > fmax) ? f3 = fmax : f3 = f3;


        // (f1 > fmax) ? f1 = fmax : f1 = f1;
        // (f2 > fmax) ? f2 = fmax : f2 = f2;
        // (f3 > fmax) ? f3 = fmax : f3 = f3;
        //////////////////////////////
        //////////////////////////////

        // Apply control for different cases of failure
        // Use only two opposite motors first to accelerate to desired yaw rate and then apply control

        // if(abs(yaw_rate) > 15 && !init_landing){
        //     //initiating landing
        //     f_sigma /= kp_gain[2] * 1;
        //     kp_gain = {0.0, 0.0, 1};
        //     printf("initiating controller landing !!!\n");
        //     init_landing = true;
        // }

        if( abs(yaw_rate) > 5 && odometry.position[2] < -2.5f){


        if(detected_motor == 1){
        act.control[0] = (float)nan("1");
        act.control[1] = f2/fmax;
        act.control[2] = f1/fmax;
        act.control[3] = f3/fmax;

        // act.control[1] = (f2 > 0) ? sqrt(f2/fmax) : -sqrt(f2/fmax);
        // act.control[2] = (f1 > 0) ? sqrt(f1/fmax) : -sqrt(f1/fmax);
        // act.control[3] = (f3 > 0) ? sqrt(f3/fmax) : -sqrt(f3/fmax);



        // act.control[1] = f2/ 14;
        // act.control[2] = f1/ 14;
        // act.control[3] = f3/ 14;

        }

        else if (detected_motor == 2)
        {
            act.control[0] = f2/ 14.56;
            act.control[1] = (float)nan("1");
            act.control[2] = f3/14.56;
            act.control[3] = f1/14.56;

        }

         else if (detected_motor == 3)
        {
            act.control[0] = f3/14.56;
            act.control[1] = f1/14.56;
            act.control[2] = (float)nan("1");
            act.control[3] = f2/ 14.56;

        }

         else if (detected_motor == 4)
        {
            act.control[0] = f1/14.56;
            act.control[1] = f3/14.56;
            act.control[2] = f2/ 14.56;
            act.control[3] = (float)nan("1");

        }

        // printf("-----------------\n");
        // printf("u1 = %f, u2 = %f, f1 = %f , f2 = %f , f3 = %f , f_sigma = %f \n",u1, u2,f1, f2, f3, shared_state.fsum);
        // // printf("u1 = %f, u2 = %f, u3 = %f, f1 = %f , f2 = %f , f3 = %f , f_sigma = %f \n",u1, u2,f1, f2, f3, f_sigma);
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, r_s, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", rpy_rad[0], rpy_rad[1], rpy_rad[2], odometry.timestamp);
        // printf("x = %f, y = %f, z = %f \n", (double)odometry.position[0],(double)odometry.position[1],(double)odometry.position[2]);
        // // printf("nx_des = %f, ny_des = %f, ny_des = %f", shared_state.nx_des, shared_state.ny_des, shared_state.nz_des);
        // printf("\n\n");
        state_data.timestamp = odometry.timestamp;
        state_data.u1 = u1;
        state_data.u2 = u2;
        state_data.f1 = f1;
        state_data.f2 = f2;
        state_data.f3 = f3;
        state_data.p_s = p_s;
        state_data.q_s = q_s;
        state_data.r_s = r_s;
        state_data.n_x = n_x;
        state_data.n_y = n_y;
        state_data.n_z = n_z;
        state_data.roll_rad = (double)rpy_rad[0];
        state_data.pitch_rad = (double)rpy_rad[1];
        state_data.yaw_rad = (double)rpy_rad[2];
        state_data.x = (double)odometry.position[0];
        state_data.y = (double)odometry.position[1];
        state_data.z = (double)odometry.position[2];

        }

        else if(abs(yaw_rate) < 5 && odometry.position[2] < -2.5f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = 0.67;
        act.control[diagonally_working_2] = 0.67;
        // printf("Two motors only , yaw_rate = %f\n",yaw_rate);
        // printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        // printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", rpy_rad[0], rpy_rad[1], rpy_rad[2], odometry.timestamp);
        }

        else if(odometry.position[2] > -2.5f)
        {
        act.control[0] = 0;
        act.control[1] = 0;
        act.control[2] = 0;
        act.control[3] = 0;
        }

        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
        orb_publish(ORB_ID(drone_state), drone_state_publisher, &state_data);
    }
}

int Controller::main(int detected_motor){

    appState.setRunning(true);

     {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state

        shared_state.detected_motor = detected_motor;
    }
    std::thread outer_thread(&Controller::outerLoop, this);
    std::thread inner_thread(&Controller::innerLoop, this);

    outer_thread.join();
    inner_thread.join();
    return 0;
}


double* Controller::quaternionToRPY (double qw, double qx, double qy, double qz){
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


double* Controller::vectorFRD2Plus (double* v){
    double* new_v = new double[3];
    new_v[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    new_v[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    new_v[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];
    return new_v;
}


Vector Controller::vectorFRD2Plus (Vector v){
    Vector new_v(3);
    new_v[0] = transformationMatrix[0][0] * v[0] + transformationMatrix[0][1] * v[1] + transformationMatrix[0][2] * v[2];
    new_v[1] = transformationMatrix[1][0] * v[0] + transformationMatrix[1][1] * v[1] + transformationMatrix[1][2] * v[2];
    new_v[2] = transformationMatrix[2][0] * v[0] + transformationMatrix[2][1] * v[1] + transformationMatrix[2][2] * v[2];
    return new_v;
}

// Calculates unit vector along normal
Vector Controller::AorBodyFrame (double* q, double* N){
    Quaternion quat_frd = {q[0], q[1], q[2], q[3]};
    Vector3 n_des_frd_gnd = {N[0], N[1], N[2]};
    Vector3 n_des_frd_body = transform_vector_to_child_frame(n_des_frd_gnd, quat_frd);
    Vector n3 = {n_des_frd_body.x, n_des_frd_body.y, n_des_frd_body.z};
    n3 = vectorFRD2Plus(n3);
    state_data.nx_des_frd_gnd  = (double)n_des_frd_gnd.x;
    state_data.ny_des_frd_gnd  = (double)n_des_frd_gnd.y;
    state_data.nz_des_frd_gnd  = (double)n_des_frd_gnd.z;
    state_data.nx_des_frd_body = (double)n_des_frd_body.x;
    state_data.ny_des_frd_body = (double)n_des_frd_body.y;
    state_data.nz_des_frd_body = (double)n_des_frd_body.z;
    // printf("n_des_frd_gnd %f, %f, %f and n_des_frd_body %f, %f, %f and n_des_plus_body %f, %f, %f\n\n",
    //                 N[0], N[1], N[2], n_des_frd_body.x, n_des_frd_body.y, n_des_frd_body.z, n3[0], n3[1], n3[2]);
    return n3;
}

double Controller::calculateNorm(Vector vec) {
    double sum = 0.0;
    for (double component : vec) {
        sum += component * component;
    }
    return abs(std::sqrt(sum));
}

// Converts rpy_rate to plus configuration
double* Controller::rpy_rate_plus(double roll_rate, double pitch_rate, double yaw_rate){
    double roll_rate_rad = roll_rate;// * (M_PI / 180.0);
    double pitch_rate_rad = pitch_rate;// * (M_PI / 180.0);
    double yaw_rate_rad = yaw_rate;// * (M_PI / 180.0);

    double* rpy_rate_plus = new double[3];

    rpy_rate_plus[0] = cos(M_PI / 4) * roll_rate_rad - sin(M_PI / 4) * pitch_rate_rad;
    rpy_rate_plus[1] = - cos(M_PI / 4) * pitch_rate_rad - sin(M_PI / 4) * roll_rate_rad;
    rpy_rate_plus[2] = -1.0 * yaw_rate_rad;

    return rpy_rate_plus;
}

// Quaternion multiplication
Quaternion Controller::quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// Quaternion conjugate
Quaternion Controller::quaternion_conjugate(Quaternion q) {
    Quaternion result;
    result.w = q.w;
    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;
    return result;
}

// Vector to Quaternion conversion (pure quaternion)
Quaternion Controller::vector_to_quaternion(Vector3 v) {
    Quaternion q;
    q.w = 0;
    q.x = v.x;
    q.y = v.y;
    q.z = v.z;
    return q;
}

// Quaternion to Vector conversion
Vector3 Controller::quaternion_to_vector(Quaternion q) {
    Vector3 v;
    v.x = q.x;
    v.y = q.y;
    v.z = q.z;
    return v;
}

// Transform vector from parent frame to child frame
// q is the quaternion representing child frame orientation relative to parent frame
Vector3 Controller::transform_vector_to_child_frame(Vector3 parent_vector, Quaternion q) {
    // Convert vector to quaternion
    Quaternion v_quat = vector_to_quaternion(parent_vector);
    // Quaternion rotation: v_child = q * v_parent * q^-1
    // q^-1 is the conjugate for unit quaternions
    Quaternion q_conj = quaternion_conjugate(q);
    Quaternion rotated_quat = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj);
    // Convert back to vector
    return quaternion_to_vector(rotated_quat);
}
