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
 * @file lqr.cpp
 * Implementation of LQR class
 *
 */

#include "lqr.h"
#include <iostream>
#include <vector>
#include <cmath>


using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;


// Function to print the matrix
void printMatrix(const Matrix& mat) {
    for (const auto& row : mat) {
        for (double elem : row) {
            // Print each element followed by a space
            printf("%.2f ", elem);
        }
        // Newline after each row
        printf("\n");
    }
}

// Computes the inverse of a 2x2 matrix
Matrix LQR::inverse2x2(const Matrix &a) {
    double determinant = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (determinant <= 0) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }
    double invDet = 1.0 / determinant;
    Matrix result(2, std::vector<double>(2));
    result[0][0] =  a[1][1] * invDet;
    result[0][1] = -a[0][1] * invDet;
    result[1][0] = -a[1][0] * invDet;
    result[1][1] =  a[0][0] * invDet;
    return result;
}

// Returns the transpose of a matrix
Matrix LQR::transpose(const Matrix &a) {
    int rows = a.size();
    int cols = a[0].size();
    Matrix result(cols, std::vector<double>(rows));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j][i] = a[i][j];
        }
    }
    return result;
}

// Creates an identity matrix of a given size
Matrix LQR::identityMatrix(int size) {
    Matrix identity(size, std::vector<double>(size, 0.0));
    for (int i = 0; i < size; ++i) {
        identity[i][i] = 1.0;
    }
    return identity;
}

// Multiplies a matrix by a vector
Vector LQR::multiply(const Matrix &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = 0;
        for (size_t j = 0; j < b.size(); ++j) {
            result[i] += a[i][j] * b[j];
        }
    }
    return result;
}

// Multiplies two matrices
Matrix LQR::multiply(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(b[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < b[0].size(); ++j) {
            result[i][j] = 0;
            for (size_t k = 0; k < b.size(); ++k) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return result;
}
// Adds two matrices element-wise
Matrix LQR::matrixAdd(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(a[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < a[0].size(); ++j) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
    return result;
}

// Subtracts two matrices element-wise
Matrix LQR::matrixSubtract(const Matrix &a, const Matrix &b) {
    Matrix result(a.size(), std::vector<double>(a[0].size()));
    for (size_t i = 0; i < a.size(); ++i) {
        for (size_t j = 0; j < a[0].size(); ++j) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
    return result;
}

// Clamps each element in the vector to be within the specified min and max values
Vector LQR::clampVector(const Vector &vec, double minVal, double maxVal) {
    Vector clamped = vec;
    for (size_t i = 0; i < clamped.size(); ++i) {
        if (clamped[i] < minVal) {
            clamped[i] = minVal;
        } else if (clamped[i] > maxVal) {
            clamped[i] = maxVal;
        }
    }
    return clamped;
}

// Computes the Euclidean norm of a vector
double LQR::norm(const Vector &vec) {
    double sum = 0.0;
    for (double val : vec) {
        sum += val * val;
    }
    return std::sqrt(sum);
}

// Adds two vectors element-wise
Vector LQR::vectorAdd(const Vector &a, const Vector &b) {
    Vector result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

Matrix LQR::getB(double deltat) {
    Matrix B = {
        {0                  ,    l*deltat/ I_xxt},
        {l*deltat/ I_xxt    ,                  0},
        {0                  ,                  0},
        {0                  ,                  0}
    };
    return B;
}

Matrix LQR::getA(double deltat) {
    Matrix A = {
        {0+1                ,a_const*deltat      ,    0             ,                 0},
        {-a_const*deltat  ,0 +1                  ,    0             ,                 0},
        {0                ,-nz_eq*deltat      ,    0   +1          ,       r_eq*deltat},
        {nz_eq*deltat     ,0                   ,  -r_eq*deltat    ,                 0 + 1}
    };
    return A;
}


Vector LQR::lqr(const Vector &actual_state, const Vector &desired_state, const Matrix &Q, const Matrix &R, const Matrix &A, const Matrix &B, double dt) {
    Vector x_error = actual_state;
    for (size_t i = 0; i < actual_state.size(); ++i) {
        x_error[i] = (desired_state[i] - actual_state[i]);
    }
    Matrix K;
    Vector u;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // K = {{ 0.7604957888197689, 14.915744438424122, -10.338666088015943, -24.43673459546383 },
    //  { 11.650618106028348, 0.229400126528235, -21.547359765479555, 37.52883999916331 }};  //IRIS RHO=1.28
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // K = {{ -9.566987799048773e-16, 0.588213695875172, -5.0850384014349945, -37.615644370919675 },
    //  { 0.5882136958751719, 9.687075512007543e-16, -37.61564437091968, 5.08503840143499 }};       //rho = 0.53
    // K = {{ -1.3009502237200213e-16, 1.315762738415036, -3.008772667752987, -9.21522612703843 },
    //  { 1.315762738415035, 4.002923765292373e-17, -9.215226127038402, 3.0087726677529867 }};       //rho = 0.33
    // K = {{ 0.21087984342956764, 3.304121854415287, -6.310767885888993, -9.666700314309152 },
    //  { 2.623361365649567, 0.2108798434295682, -11.519430165261205, 10.954749205791732 }};   //rho = 1.24
    // K = {{ 1.8013156943815674e-16, 1.0197757223020965, -2.9626391764975972, -6.5812766299717165 },
    //  { 1.019775722302095, 9.006578471907837e-17, -6.581276629971724, 2.962639176497599 }};          //rho = 0.8937820376119587
    // K = {{ -0.2933665382353208, 2.262488091733113, -15.424404213739042, -9.183808675393825 },
    //  { 3.8695272638412597, -0.2933665382353237, -8.627435916948276, 5.910235135594249 }};          //rho = 1.32

    // K = {{ -2.052599442467586e-15, 4.894330457577658, -10.939222704494966, -32.51680107335382 },
    //  { 4.894330457577662, 9.951997296812538e-16, -32.516801073353875, 10.939222704495007 }};     //IRIS RHO=1.28
    // K = {{ 0.6054503996511734, 13.085503321306403, -10.409305389211733, -24.86456678689718 },
    //  { 11.540789078053141, 0.5219970386634336, -22.24679861238172, 36.045848747737644 }};  //IRIS RHO=1.28 well and good

    // K = {{ 1.458704149454036, 63.36483614088804, -4455.453307752515, -754.6491904440179 },
    //  { 63.364836140888194, -1.4587041494536406, -754.6491904439973, 4455.453307752521 }};   //pole placement

    // K = {{ -1.258704315648492, 7.979273556467416, -26.80128379629879, -12.449420633671924 },
    //  { 12.63502611607351, -0.834073285153794, -9.808466878398608, 18.468957570664294 }};   //iris rho=1.15, l=0.25, drag coeff sorted

    // K = {{ 0.19889958601691507, 3.547610329740018, -12.848681911389528, -15.439129845703212 },
    //  { 5.211267758810277, 0.397799172033829, -19.71959524937223, 18.582762608611844 }};


/////////////////////////////////////////////////////////////////////////////////////////////////////
    // K = {{ 0.19889958601691507, 3.547610329740018, -12.848681911389528, -15.439129845703212 },
    //  { 5.211267758810277, 0.397799172033829, -19.71959524937223, 18.582762608611844 }};      //iris rho=1.36, descent

    // K = {{ -0.6857115904253366, 11.8318671656736, -27.520869518628704, -12.210950795202343 },
    //  { 12.842301265034303, -0.6451212768084172, -9.140122818971712, 29.885837563265714 }};     //good

    // K = {{ -0.6857383153577681, 11.846302163601928, -29.026288712570405, -12.865521242146855 },
    //  { 12.856164462355135, -0.6452228174333394, -9.62710442104796, 31.520879521684247 }};

    // K = {{ -0.6857636306748175, 11.859551160202788, -30.40973690501298, -13.465886672609324 },
    //  { 12.868892198554576, -0.6453164547151957, -10.073476785778581, 33.0234783517902 }}; //good, control out

    // K = {{ -0.5777627800796667, 11.26812405369197, -28.78715682519047, -13.150229422221035 },
    //  { 12.134885008072423, -0.5456518718240264, -10.30450680556764, 31.023116678961483 }};

    // K = {{ -0.45052299733107143, 12.317363077356474, -25.996484568691585, -11.90927199139858 },
    //  { 12.407605619265805, -0.48655104287324263, -9.224602672448528, 31.033648741509722 }};


/////////////////////////////////////////good to go
    K = {{ -0.37411555954446124, 13.023960916126605, -25.61061624374246, -11.921944552854015 },
     { 12.436213546065465, -0.47030185250502166, -9.098541674122346, 32.82293270509998 }};

    // K = {{ -0.30894948408605594, 12.479790900663913, -24.49375057449049, -11.719989240868868 },
    //  { 11.867102467822153, -0.39273474926130647, -9.257670332459625, 31.248491031690797 }};

    // K = {{ -0.30894948408605594, 12.479790900663913, -24.49375057449049, -11.719989240868868 },
    //  { 11.867102467822153, -0.39273474926130647, -9.257670332459625, 31.248491031690797 }};


    u = multiply(K, x_error);
    return u;
}

Vector LQR::getControlInputs(Vector actual_state, int detected_motor, double nx_des, double ny_des) {

    if(detected_motor == 1 || detected_motor == 3){
        r_eq = +8.5;
        a_const = ((I_xxt - I_zzt)*r_eq/I_xxt) + I_zzp*( w1_eq + w2_eq + w3_eq + w4_eq ) / I_xxt ;
    }
    else{
        r_eq = -8.5;
        a_const = ((I_xxt - I_zzt)*r_eq/I_xxt) + I_zzp*( w1_eq + w2_eq + w3_eq + w4_eq ) / I_xxt ;
    }

    double dt = 1;

    Vector desired_state;


    if(detected_motor == 1){
        // newly calculated values
        desired_state = {0.0, 4.7317317657445965, nx_des, ny_des};
        // desired_state = {0.0, 4.7317317657445965, nx_des, ny_des};


        // desired_state = {0.0, 2.25361451700798, nx_des, ny_des};
        // desired_state = {0.0, 1.9644959812893703, nx_des, ny_des};
        //desired_state = {0.0, 6.29369911312751, 0.0, 0.29394435248697914};
        // when outer loop control is working
        // desired_state = {0.0, 6.29369911312751, nx_des, ny_des};
        // desired_state = {0.0, 10.421486739437066, nx_des, ny_des};
        // desired_state = {0.0, 11.153835166845344, nx_des, ny_des};
        // desired_state = {0.0, 13.901672761162269, 0.0, 0.7079288615564396};
        // desired_state = {0.0, 13.901672761162269, nx_des, ny_des};

    }
    else if (detected_motor == 2)
    {
     desired_state = {0.0, -2.53, 0.0, 0.2855};
    }
    else if (detected_motor == 3)
    {
     desired_state = {2.53, 0.0, -0.2855, 0.0};
    }
    else if(detected_motor == 4)
    {
     desired_state = {-2.53, 0.0, 0.2855, 0.0};
    }



    Matrix R = {{1   ,  0},
                {0   ,  1}};

    Matrix Q = {{1     ,0      ,0     ,0},
                {0     ,1      ,0     ,0},
                {0     ,0      ,20    ,0},
                {1     ,0      ,0    ,20}};



    Vector state_error = actual_state;
    for (size_t j = 0; j < actual_state.size(); ++j) {
            state_error[j] -= desired_state[j];
    }

    Vector desired_state_error = {0.0,  0.0,   0.0,   0.0};


    Matrix A = getA(dt);
    Matrix B = getB(dt);
    Vector optimal_control_input = lqr(state_error, desired_state_error, Q, R, A, B, dt);


    return optimal_control_input;
}


