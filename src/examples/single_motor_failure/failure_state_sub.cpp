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
 * @file failure_state_pub.cpp
*/



#include "failure_state_sub.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/drone_state.h>



#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <vector>
#include <array>

#include <uORB/uORB.h>

#include <iostream>  // For std::cout
#include <cstdio>    // For printf
#include <fstream>   // For std::ofstream
#include <filesystem>



using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


px4::AppState StateSub::appState;  /* track requests to terminate app */

int StateSub::main(){
    int drone_state_sub = orb_subscribe(ORB_ID(drone_state));
    drone_state_s drone_state_data;

    std::string baseFilePath = "/home/rajeev-gupta/ros2/inter-iit_ws/src/Inter-IIT_IdeaForge-PS/detection_tests/px4_detection_csv/odometry_data";
    std::string filePath = baseFilePath;
    int counter = 1;

    // Check if the file exists and modify the filename if necessary
    while (std::filesystem::exists(filePath + ".csv")) {
        filePath = baseFilePath + "_" + std::to_string(counter);
        counter++;
    }
    filePath = filePath + ".csv";

    // Open the file (it will be a new file or a modified filename)
    std::ofstream csvFile(filePath, std::ios::out | std::ios::trunc);
    if (!csvFile.is_open()) {
        std::cerr << "Failed to open CSV file." << std::endl;
        return -1; // Exit if file cannot be opened
    }

    // File is ready for writing
    std::cout << "Opened file: " << filePath << std::endl;

    while (!appState.exitRequested()) {
        orb_copy(ORB_ID(drone_state), drone_state_sub, &drone_state_data);
        usleep(1000);
        // Save odometry data to CSV
        if(drone_state_data.timestamp == prev_timestamp || drone_state_data.timestamp > 99999999){
            continue;
        }
        csvFile  << drone_state_data.timestamp << ","
                 << drone_state_data.u1 << ","
                 << drone_state_data.u2 << ","
                 << drone_state_data.f1 << ","
                 << drone_state_data.f2 << ","
                 << drone_state_data.f3 << ","
                 << drone_state_data.p_s << ","
                 << drone_state_data.q_s << ","
                 << drone_state_data.r_s << ","
                 << drone_state_data.n_x << ","
                 << drone_state_data.n_y << ","
                 << drone_state_data.n_z << ","
                 << drone_state_data.roll_rad << ","
                 << drone_state_data.pitch_rad << ","
                 << drone_state_data.yaw_rad << ","
                 << drone_state_data.x << ","
                 << drone_state_data.y << ","
                 << drone_state_data.z << std::endl;
        prev_timestamp = drone_state_data.timestamp;
    }
    csvFile.close(); // Close the file when done

    return 0;
}

