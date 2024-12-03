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
#include <uORB/topics/drone_state.h>


#include <thread>
#include <mutex>




using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState Controller::appState;  /* track requests to terminate app */


// Shared state structure
struct State {
    double desired_velocity = 0.0; // Inner loop target
    double current_velocity = 0.0; // System state

    int check_flag = 0;
    int detected_motor;

    double nx_des;
    double ny_des;
};

State shared_state;
std::mutex shared_state_mutex; // Mutex to protect shared state


// Outer loop: Position control
void Controller::outerLoop() {

    // {
    //     std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
    //     int detected_motor = shared_state.detected_motor;
    // }
    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    Vector desired_position = {0.0 , 0.0 , -10.0};
    Vector current_position = {0.0 , 0.0 , 0.0};
    Vector pos_deviation = {0.0 , 0.0 , 0.0};

    Vector desired_accel  = {0.0 , 0.0 , 0.0};

    double nx_des;
    double ny_des;
    // double nz_bar = 0.7071587135350774;
    double nz_bar = 0.9819333920816344;
    // double nz_bar = 0.8253375925852746;

    double nat_freq = 1;
    double damping_const = 0.7;

    double* rpy_rad;

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    KP = 8;
    drone_weight = 15.05835;        //iris
    // drone_weight = 20.250858461538463;

    // double* rpy_plus;


    while (true) {
        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);

        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        roll_rad = rpy_rad[0];
        pitch_rad = rpy_rad[1];
        yaw_rad = rpy_rad[2];

        current_position[0] = odometry.position[0];
        current_position[1] = odometry.position[1];
        current_position[2] = odometry.position[2];

        pos_deviation[0] = current_position[0] - desired_position[0];
        pos_deviation[1] = current_position[1] - desired_position[1];
        pos_deviation[2] = current_position[2] - desired_position[2];

        desired_accel[0] = -2*damping_const*nat_freq*(double)odometry.velocity[0] - nat_freq*nat_freq*pos_deviation[0];
        desired_accel[1] = -2*damping_const*nat_freq*(double)odometry.velocity[1] - nat_freq*nat_freq*pos_deviation[1];
        desired_accel[2] = -2*damping_const*nat_freq*(double)odometry.velocity[2] - nat_freq*nat_freq*pos_deviation[2];

        desired_accel[2]+= 9.8;

        // rpy_plus = rpy_rate_plus(roll_rad, pitch_rad , yaw_rad);

        // Vector ndes = multiply(calculateRotationMatrixInverse(rpy_plus[0],rpy_plus[1],rpy_plus[2]), desired_accel);
        Vector ndes = multiply(calculateRotationMatrixInverse(roll_rad,pitch_rad,yaw_rad), desired_accel);

        // recompute f_sigma and n_desired
        f_sigma = drone_weight + KP * pos_deviation[2];
        nx_des = (ndes[0]*2)/ (nz_bar*f_sigma);
        ny_des = (ndes[1]*2)/ (nz_bar*f_sigma);

        {
        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        shared_state.nx_des = nx_des;
        shared_state.ny_des = ny_des;
        }

        // Unlock automatically when lock_guard goes out of scope
        usleep(2000);
        // usleep(1000);
    }



}

// Inner loop: Velocity control
void Controller::innerLoop() {
    // while (true) {

    //     {
    //     std::lock_guard<std::mutex> lock(state_mutex);
    //     state.check_flag++;
    //     printf("Inner Loop flag = %d \n",state.check_flag);
    //     }


    //     // Mutex is released automatically when lock_guard goes out of scope

    //     usleep(500000); // Sleep for 10 ms
    // }
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

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double* normal;
    double n_x;
    double n_y;
    double n_z;


    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;

    double u1=0.0;
    double u2=0.0;
    double f1;                  //      Anticlockwise first motor to the failed
    double f2;                  //      Opposite to failed modtor
    double f3;                  //      Clockwise first motor to the failed

    LQR lqr;

    Vector current_state;
    Vector inputs;

    double signal;
    double r_max_to_start_control = 5;




    while (!appState.exitRequested()) {

        usleep(100);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);


        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        roll_rad = rpy_rad[0];
        pitch_rad = rpy_rad[1];
        yaw_rad = rpy_rad[2];

        roll_rate = odometry.angular_velocity[0];
        pitch_rate = odometry.angular_velocity[1];
        yaw_rate = odometry.angular_velocity[2];

        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];

        double* q_inverse = new double[4];
        q_inverse[0] = odometry.q[0];
        q_inverse[1] = -1.0 * (double)odometry.q[1];
        q_inverse[2] = -1.0 * (double)odometry.q[2];
        q_inverse[3] = -1.0 * (double)odometry.q[3];

        normal = vectorAlongNormal(q_inverse);
        n_x = normal[0];
        n_y = normal[1];
        n_z = normal[2];

        current_state = {p_s, q_s, n_x, n_y};        // Get current state

        std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        inputs = lqr.getControlInputs(current_state, detected_motor, shared_state.nx_des, shared_state.ny_des);     // Apply LQR controller



        u1 = inputs[0];
        u2 = inputs[1];
        double f_max = 14.1328; //IRIS maximum thrust available per motor for maxRotVel = 1000
        // double f_max = 17.0916; //maximum thrust available per motor for maxRotVel = 1000
        // double f_max = 34.1832; //maximum thrust available per motor for maxRotVel = 2000
        // double f1_bar=0.0;
        // double f2_bar=0.0;
        // double f3_bar=0.0;
        // f1_bar = 4.210580583085526;
        // f2_bar = 6.898615227327325;
        // f3_bar = f1_bar;


        // with no altitude control
        // f3 = (2*f3_bar + u1 - u2)/2;
        // f2 = f2_bar + u2;
        // f1 = (2*f1_bar - u1 - u2)/2;

        // // for outer loop control
        double f2_bar = 6.898615227327325;
        f1 = (f_sigma - (u1+u2+f2_bar)) / 2;
        f2 = u2 + f2_bar;
        f3 = f1 + u1;

        if (f1 < 0.0) f1 = 0.0;
        if (f2 < 0.0) f2 = 0.0;
        if (f3 < 0.0) f3 = 0.0;
        if (f1 > f_max) f1 = f_max;
        if (f2 > f_max) f2 = f_max;
        if (f3 > f_max) f3 = f_max;



        // signal = 9.3 + 0.1*((double)odometry.position[2] + 10);

        // Thrust equations after solving input relation

        // f2 = u2 + 8.83/2;
        // f1 = ( 8.83 - (u1 + u2)/2 > 0) ? 8.83 - (u1 + u2)/2 : 0.0 ;
        // f3 = (8.83 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 8.83 + (u1 - u2)/2  ;

        // f2 = u2 + (9.4/2);
        // f1 = ( 9.4 - (u1 + u2)/2 > 0) ? 9.4 - (u1 + u2)/2 : 0.0 ;
        // f3 = (9.4 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 9.4 + (u1 - u2)/2  ;

        // f2 = u2 + (signal/2);
        // f1 = ( signal - (u1 + u2)/2 > 0) ? signal - (u1 + u2)/2 : 0.0 ;
        // f3 = (signal + (u1 - u2)/2 > 14.56 ) ? 14.56 : signal + (u1 - u2)/2  ;

        // Apply control for different cases of failure
        // Use only two opposite motors first to accelerate to desired yaw rate and then apply control


        if( abs(yaw_rate) > r_max_to_start_control && odometry.position[2] < -2.5f){


        if(detected_motor == 1){
            act.control[0] = (float)nan("1");
            act.control[1] = f2 / f_max;
            act.control[2] = f1 / f_max;
            act.control[3] = f3 / f_max;
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

        printf("header\n");
        printf("-----------------\n");
        printf("u1 = %f, u2 = %f , f1 = %f , f2 = %f , f3 = %f ,yaw_rate = %f   \n",u1, u2 ,f1, f2, f3, yaw_rate );
        printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
        printf("x = %f, y = %f, z = %f  signal = %f \n", (double)odometry.position[0],(double)odometry.position[1],(double)odometry.position[2],signal);
        printf("\n\n");
        }

        else if(abs(yaw_rate) < r_max_to_start_control && odometry.position[2] < -2.5f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = 0.67;
        act.control[diagonally_working_2] = 0.67;
        printf("header\n");
        printf("-----------------\n");
        printf("Two motors only , yaw_rate = %f\n",yaw_rate);
        printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
        printf("\n\n");
        }

        else if(odometry.position[2] > -2.5f)
        {
        act.control[0] = 0;
        act.control[1] = 0;
        act.control[2] = 0;
        act.control[3] = 0;
        }

        // printf("Alt = %f \n" , (double)odometry.position[2]);

        state_data.timestamp = odometry.timestamp;
        state_data.u1 = u1;
        state_data.u2 = u2;
        state_data.f1 = f1;
        state_data.f2 = f2;
        state_data.f3 = f3;
        state_data.p_s = p_s;
        state_data.q_s = q_s;
        state_data.r_s = yaw_rate;
        state_data.n_x = n_x;
        state_data.n_y = n_y;
        state_data.n_z = n_z;
        state_data.roll_rad = roll_rad;
        state_data.pitch_rad = pitch_rad;
        state_data.yaw_rad = yaw_rad;
        state_data.x = (double)odometry.position[0];
        state_data.y = (double)odometry.position[1];
        state_data.z = (double)odometry.position[2];

        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);
        orb_publish(ORB_ID(drone_state), drone_state_publisher, &state_data);


        // {
        // std::lock_guard<std::mutex> lock(shared_state_mutex); // Lock mutex for shared state
        // shared_state.nx_des = 0.0;
        // shared_state.ny_des = 0.2855;
        // }



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


            // Let the simulation run for 5 seconds
        // sleep(5);

        // // Detach threads (graceful shutdown logic can be added here)
        // outer_thread.detach();
        // inner_thread.detach();



    /*

    int detected_motor_index = detected_motor - 1;
    int opposite_motor_index  = (detected_motor_index < 2) ? (1 - detected_motor_index) : (5 - detected_motor_index);
    int diagonally_working_1 =  (detected_motor_index < 2) ? 2 : 0;
    int diagonally_working_2 =  (detected_motor_index < 2) ? 3 : 1;


    actuator_motors_s act = {};  // Zero-initialized
    orb_advert_t act_pub_fd = orb_advertise(ORB_ID(actuator_motors), &act);

    int vehicle_odometry_fd = orb_subscribe(ORB_ID(vehicle_odometry));
    vehicle_odometry_s odometry;

    double* rpy_rad;

    double roll_rad;
    double pitch_rad;
    double yaw_rad;

    double* normal;
    double n_x;
    double n_y;
    double n_z;


    double roll_rate;
    double pitch_rate;
    double yaw_rate;

    double* rpy_plus_rate;

    double p_s;
    double q_s;

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
        usleep(1000);

        orb_copy(ORB_ID(vehicle_odometry), vehicle_odometry_fd, &odometry);


        rpy_rad = quaternionToRPY(odometry.q[0], odometry.q[1], odometry.q[2], odometry.q[3]);

        roll_rad = rpy_rad[0];
        pitch_rad = rpy_rad[1];
        yaw_rad = rpy_rad[2];

        roll_rate = odometry.angular_velocity[0];
        pitch_rate = odometry.angular_velocity[1];
        yaw_rate = odometry.angular_velocity[2];

        rpy_plus_rate = rpy_rate_plus(roll_rate, pitch_rate , yaw_rate);
        p_s = rpy_plus_rate[0];
        q_s = rpy_plus_rate[1];

        double* q_inverse = new double[4];
        q_inverse[0] = odometry.q[0];
        q_inverse[1] = -1.0 * (double)odometry.q[1];
        q_inverse[2] = -1.0 * (double)odometry.q[2];
        q_inverse[3] = -1.0 * (double)odometry.q[3];

        normal = vectorAlongNormal(q_inverse);
        n_x = normal[0];
        n_y = normal[1];
        n_z = normal[2];

        current_state = {p_s, q_s, n_x, n_y};        // Get current state


        // if (abs(yaw_rate) < 20 && odometry.position[2] < -2.5f)
        // {
        //     inputs = lqr.getControlInputs(current_state, detected_motor , 2);
        //     u1 = inputs[0];
        //     f3 = (u1 + 20.0)/2;
        //     f1 = (20.0 -u1)/2;

        //      act.control[0] = (float)nan("1");
        //      act.control[1] = (float)nan("1");
        //      act.control[2] = f1/14.56;
        //      act.control[3] = f3/14.56;

        //      printf("Two motors only , yaw_rate = %f  f1 = %f  f2 = %f , u = %f \n",yaw_rate,f1, f3,inputs[0]);
        //      printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        // }


        inputs = lqr.getControlInputs(current_state, detected_motor);     // Apply LQR controller


        u1 = inputs[0];
        u2 = inputs[1];

        // Thrust equations after solving input relation

        // f2 = u2 + 8.83/2;
        // f1 = ( 8.83 - (u1 + u2)/2 > 0) ? 8.83 - (u1 + u2)/2 : 0.0 ;
        // f3 = (8.83 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 8.83 + (u1 - u2)/2  ;

        f2 = u2 + 10.79/2;
        f1 = ( 10.79 - (u1 + u2)/2 > 0) ? 10.79 - (u1 + u2)/2 : 0.0 ;
        f3 = (10.79 + (u1 - u2)/2 > 14.56 ) ? 14.56 : 10.79 + (u1 - u2)/2  ;

        // Apply control for different cases of failure
        // Use only two opposite motors first to accelerate to desired yaw rate and then apply control



        if( abs(yaw_rate) > 5 && odometry.position[2] < -2.5f){


        if(detected_motor == 1){
        act.control[0] = (float)nan("1");
        act.control[1] = f2/ 14.56;
        act.control[2] = f1/14.56;
        act.control[3] = f3/14.56;

        // act.control[1] = f2/ 12.5;
        // act.control[2] = f1/ 12.5;
        // act.control[3] = f3/ 12.5;

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

        printf("u1 = %f, u2 = %f , f1 = %f , f2 = %f , f3 = %f ,yaw_rate = %f   \n",u1, u2 ,f1, f2, f3, yaw_rate );
        printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);


        }

        else if(abs(yaw_rate) < 5 && odometry.position[2] < -2.5f)
        {
        act.control[detected_motor_index] = (float)nan("1");
        act.control[opposite_motor_index] = (float)nan("1");
        act.control[diagonally_working_1] = 0.67;
        act.control[diagonally_working_2] = 0.67;
        printf("Two motors only , yaw_rate = %f\n",yaw_rate);
        printf("state p = %f, q = %f, r = %f, n_x = %f, n_y = %f, n_z = %f \n", p_s, q_s, yaw_rate, n_x, n_y, n_z);
        printf("roll = %f pitch = %f yaw = %f timestamp = %d\n", roll_rad, pitch_rad, yaw_rad, odometry.timestamp);
        }

        else if(odometry.position[2] > -2.5f)
        {
        act.control[0] = 0;
        act.control[1] = 0;
        act.control[2] = 0;
        act.control[3] = 0;
        }

        // printf("Alt = %f \n" , (double)odometry.position[2]);


        orb_publish(ORB_ID(actuator_motors), act_pub_fd, &act);



    }

*/

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

double* Controller::hamiltonianProduct (double* q1, double* q2){
    double* hamiltonian_product = new double[4];
    hamiltonian_product[0] = (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    hamiltonian_product[1] = (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    hamiltonian_product[2] = (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    hamiltonian_product[3] = (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
    return hamiltonian_product;
}

double* Controller::vectorQuaternionTransformation (double* q, double* v){
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

double* Controller::vectorFRD2Plus (double* v){
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
double* Controller::vectorAlongNormal (double* q){
    double* vector_along_normal = new double[3];
    vector_along_normal[0] = 0.0;
    vector_along_normal[1] = 0.0;
    vector_along_normal[2] = -1.0;

    vector_along_normal = vectorFRD2Plus(vectorQuaternionTransformation(q, vector_along_normal));

    return vector_along_normal;
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

// Function to calculate the inverse of the rotation matrix (transpose of R)
Matrix Controller::calculateRotationMatrixInverse(double roll, double pitch, double yaw) {

    Matrix inverse(3, std::vector<double>(3));
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



// Adds two vectors element-wise
Vector Controller::vectorAdd(const Vector &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

// Multiplies a matrix by a vector
Vector Controller::multiply(const Matrix &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            result[i] += a[i][j] * b[j];
        }
    }
    return result;
}
