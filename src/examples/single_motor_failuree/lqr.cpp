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

Matrix LQR::getA(double deltat) {
    // Matrix A = {
    //     {       1, a_const, 0, 0},
    //     {-a_const,       1, 0, 0},
    //     {       0,  -nz_eq, 1, 0},
    //     {   nz_eq,       0, 0, 1}
    // };
     Matrix A = {
         {       1, a_const,   b_const, 0, 0},
         {-a_const,       1,   c_const, 0, 0},
         {       0,       0, d_const+1, 0, 0},
         {       0,  -nz_eq,     ny_eq, 1, r_eq},
         {   nz_eq,       0,    -nx_eq, -r_eq, 1}
     };
    return A;
}

Matrix LQR::getB(double deltat) {
    // Matrix B = {
    //     {            0,      l / I_xxt},
    //     {    l / I_xxt,              0},
    //     {            0,              0},
    //     {            0,              0}
    // };
    // Matrix B = {
    //     {            0,      l / I_xxt,             0},
    //     {   -l / I_xxt,              0,    -l / I_xxt},
    //     {k_tau / I_zzt, -k_tau / I_zzt, k_tau / I_zzt},
    //     {            0,              0,             0},
    //     {            0,              0,             0}
    // };
    Matrix B = {
        {            0,      l / I_xxt,             0},
        {    l / I_xxt,              0,             0},
        {            0, -k_tau / I_zzt, k_tau / I_zzt},
        {            0,              0,             0},
        {            0,              0,             0}
    };
    return B;
}


Vector LQR::lqr(const Vector &actual_state, const Vector &desired_state, const Matrix &Q, const Matrix &R, const Matrix &A, const Matrix &B, double dt) {


    Vector x_error = actual_state;
    for (size_t i = 0; i < actual_state.size(); ++i) {
        x_error[i] = desired_state[i] - actual_state[i];
    }

    // int N = 500;
    // std::vector<Matrix> P(N + 1);
    // P[N] = Q;
    // Matrix P_next = Q;

    // for (int i = N; i > 0; --i) {


    //     Matrix P_current = matrixAdd(Q, multiply(transpose(A), multiply(P_next, A)));
    //     P_current = matrixSubtract(P_current,
    //                  multiply(multiply(transpose(A), multiply(P_next, B)),
    //                  multiply(inverse2x2(matrixAdd(R, multiply(transpose(B), multiply(P_next, B)))),
    //                  multiply(transpose(B), multiply(P_next, A)))));
    //     P[i - 1] = P_current;
    //     P_next = P_current;
    // }

    Matrix K;
    Vector u;

    // K = multiply(inverse2x2(matrixAdd(R, multiply(transpose(B), multiply(P[N], B)))),
    //                  multiply(transpose(B), multiply(P[N], A)));


    // K = { {-8.00641604e-17,  2.91835387e+00, -4.75855745e+00, -7.05048669e-01},
    //     { 2.91835387e+00, -3.00240602e-17, -7.05048669e-01,  4.75855745e+00} };

    // K = { {-1.20096241e-16,  2.23534736e+00, -2.44976309e+00, -4.83194352e-01} ,
    //     { 2.23534736e+00, -8.00641604e-17, -4.83194352e-01,  2.44976309e+00 } };

// K = {
//   {1.36716834e-02, -1.83232922e+00, 1.57208984e+00, 5.57003129e+00, 1.01627901e+00},
//   {2.64825271e+00, 1.38475409e-01, -3.46629984e-01, -1.71082611e+00, 7.23063172e+00},
//   {3.64585536e-01, 1.90629226e+00, 2.16782078e+00, -5.49295879e+00, 1.93671812e-01}
// };////////////
/////////////////////////////////////////////////////////////////////////////////////////
K = {
  {-3.31812040e-02, -4.21937270e-01, 5.36364498e-01, 6.35371466e-01, 6.31023742e-01},
  {4.55570092e-01, 2.92259473e-02, -5.68221882e-01, -7.48228833e-01, 5.84679071e-01},
  {6.02633476e-02, 4.56929927e-01, 8.18210808e-01, -7.01855853e-01, -6.19868705e-01}
};
/////////////////////////////////////////////////////////////////////////////////////////
// K = {
//   {-5.21329912e-02, -5.91430433e-01, 6.88879076e-01, 1.31338383e+00, 7.58333325e-01},
//   {6.82775094e-01, 5.85954668e-02, -6.36827444e-01, -1.05264403e+00, 1.33278769e+00},
//   {1.30435779e-01, 6.56808270e-01, 1.21546104e+00, -1.46997969e+00, -6.43653072e-01}
// };/////////////
// K = {
//   {-1.13880532e-01, -6.79394170e-01, 6.60852882e-01, 1.61060445e+00, 5.87726530e-01},
//   {1.01087668e+00, 1.29134727e-01, -5.37988857e-01, -6.48283600e-01, 2.18988106e+00},
//   {2.19029789e-01, 7.54035037e-01, 1.26204338e+00, -1.77026791e+00, -3.56572170e-01}
// };
// K = {
//   {-4.43266426e-02, -5.44778759e-01, 6.97352733e-01, 9.60988199e-01, 6.30511573e-01},
//   {6.21137551e-01, 4.79389961e-02, -6.68257243e-01, -8.33718514e-01, 9.57717784e-01},
//   {1.14067878e-01, 6.07295288e-01, 1.20088846e+00, -1.08615557e+00, -5.60981430e-01}
// };
// K = {
//   {4.47297951e-02, -1.59090181e+00, 2.79972462e+00, 4.80795030e+00, 1.06729509e+00},
//   {2.33608871e+00, 5.12540238e-02, -3.09606149e-02, -2.12171378e+00, 6.30177019e+00},
//   {1.55801541e-01, 1.59946551e+00, 3.04675439e+00, -4.71460899e+00, -5.99597996e-01}
// };

// K = {
//   {2.19163548e+02, 3.38218496e-01, 2.74832931e-01, -2.58023307e+04, 5.48426257e+01, 9.19752393e-02, 4.00327687e-01},
//   {1.72839919e-01, 2.43313798e+00, -3.05486336e+00, -2.00199283e+01, -1.11416187e+03, -1.38076174e-06, 2.41012752e-05},
//   {2.21281953e+02, 4.94994559e-01, 2.60530812e-01, -2.52153671e+04, 4.73425619e+01, 9.38969177e-02, 4.08691545e-01}
// };
    u = multiply(K, x_error);

    // printMatrix(K);

    return u;
}

Vector LQR::getControlInputs(Vector actual_state, int detected_motor, double nx_des, double ny_des) {

    if(detected_motor == 1){
        p_eq = 0.0;
        q_eq = 2.53;
        r_eq = 8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
        b_const = (I_xxt - I_zzt) * q_eq / I_xxt;
        c_const = (I_zzt - I_xxt) * p_eq / I_xxt;
        d_const = -gamma/I_zzt;
    }
    if(detected_motor == 2){
        /////////////////p_eq = 0.0;
        /////////////////q_eq = -2.53;
        r_eq = -8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
        /////////////////b_const = (I_xxt - I_zzt) * q_eq / I_xxt;
        /////////////////c_const = (I_zzt - I_xxt) * p_eq / I_xxt;
        /////////////////d_const = -gamma/I_zzt;
    }
    if(detected_motor == 3){
        ///////////////p_eq = 2.53;
        ///////////////q_eq = 0.0;
        r_eq = 8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
        ///////////////b_const = (I_xxt - I_zzt) * q_eq / I_xxt;
        ///////////////c_const = (I_zzt - I_xxt) * p_eq / I_xxt;
        ///////////////d_const = -gamma/I_zzt;
    }
    if(detected_motor == 4){
        //////////////p_eq = -2.53;
        //////////////q_eq = 0.0;
        r_eq = -8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
        //////////////b_const = (I_xxt - I_zzt) * q_eq / I_xxt;
        //////////////c_const = (I_zzt - I_xxt) * p_eq / I_xxt;
        //////////////d_const = -gamma/I_zzt;
    }

    double dt = 1;

    Vector desired_state;


    if(detected_motor == 1){
     desired_state = {0.0, 2.25361451700798, 11.694423666728527, 0.0, 0.1892};
    //  desired_state = {0.0, 2.53, 10, nx_des, ny_des, -10.0, 0.0};
    }
    else if (detected_motor == 2)
    {
     desired_state = {0.0, -2.53, 0.0, 0.2855};
    //////////////desired_state = {0.0, -2.53, -8.5, 0.0, 0.2855};
    }
    else if (detected_motor == 3)
    {
     desired_state = {2.53, 0.0, -0.2855, 0.0};
    //////////////desired_state = {2.53, 0.0, 8.5, -0.2855, 0.0};
    }
    else if(detected_motor == 4)
    {
     desired_state = {-2.53, 0.0, 0.2855, 0.0};
    //////////////desired_state = {-2.53, 0.0, -8.5, 0.2855, 0.0};
    }



    // Matrix R = {{ 1,  0},
    //             { 0,  1}};
    Matrix R = {{ 1,  0,  0},
               { 0,  1,  0},
                { 0,  0,  1}};

    // Matrix Q = {{1, 0,  0,  0},
    //             {0, 1,  0,  0},
    //             {0, 0, 20,  0},
    //             {0, 0,  0, 20}};
    Matrix Q = {{1, 0, 0,  0,  0},
                {0, 1, 0,  0,  0},
                {0, 0, 0,  0,  0},
                {0, 0, 0, 20,  0},
                {0, 0, 0,  0, 20}};



    Vector state_error = actual_state;
    for (size_t j = 0; j < actual_state.size(); ++j) {
            state_error[j] -= desired_state[j];
    }

    // Vector desired_state_error = {0.0,  0.0,   0.0,  0.0};
    // Vector desired_state_error = {0.0,  0.0,   0.0,   0.0,  0.0, 0.0, 0.0};
    Vector desired_state_error = {0.0,  0.0,   0.0,   0.0,  0.0};


    Matrix A = getA(dt);
    Matrix B = getB(dt);
    Vector optimal_control_input = lqr(state_error, desired_state_error, Q, R, A, B, dt);


    return optimal_control_input;
}

