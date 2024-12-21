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

// #pragma once

#include "lqr.h"
#include <stdio.h>
#include <math.h>




// Define the size of the state vector and control input vector
#define STATE_SIZE 4
#define CONTROL_SIZE 2

// Function to multiply a 4x4 matrix with a 4x1 vector (returns a 4x1 vector)
void multiply(double matrix[STATE_SIZE][STATE_SIZE], double vector[STATE_SIZE], double result[STATE_SIZE]) {
    for (int i = 0; i < STATE_SIZE; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < STATE_SIZE; ++j) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
}


// Function to print the matrix
// void printMatrix(const Matrix& mat) {
//     mat.print();
// }

// Computes the inverse of a 2x2 matrix
// Matrix LQR::inverse2x2(const Matrix &a) {
//     double determinant = a[0][0] * a[1][1] - a[0][1] * a[1][0];
//     if (determinant <= 0) {
//         throw std::runtime_error("Matrix is singular and cannot be inverted.");
//     }
//     double invDet = 1.0 / determinant;
//     Matrix result(2, 2);
//     result[0][0] =  a[1][1] * invDet;
//     result[0][1] = -a[0][1] * invDet;
//     result[1][0] = -a[1][0] * invDet;
//     result[1][1] =  a[0][0] * invDet;
//     return result;
// }

// Returns the transpose of a matrix
// Matrix LQR::transpose(const Matrix &a) {
//     int rows = a.size();
//     int cols = a[0].size();
//     Matrix result(cols, rows);
//     for (int i = 0; i < rows; ++i) {
//         for (int j = 0; j < cols; ++j) {
//             result[j][i] = a[i][j];
//         }
//     }
//     return result;
// }

// Creates an identity matrix of a given size
// Matrix LQR::identityMatrix(int size) {
//     Matrix identity(size, size);
//     for (int i = 0; i < size; ++i) {
//         identity[i][i] = 1.0;
//         for (int j = 0; j < size; ++j) {
//             identity[i][j] = 0.0;
//         }
//     }
//     return identity;
// }


// Multiplies two matrices
// Matrix LQR::multiply(const Matrix &a, const Matrix &b) {
//     Matrix result(a.size(), std::vector<double>(b[0].size()));
//     for (size_t i = 0; i < a.size(); ++i) {
//         for (size_t j = 0; j < b[0].size(); ++j) {
//             result[i][j] = 0;
//             for (size_t k = 0; k < b.size(); ++k) {
//                 result[i][j] += a[i][k] * b[k][j];
//             }
//         }
//     }
//     return result;
// }
// Adds two matrices element-wise
// Matrix LQR::matrixAdd(const Matrix &a, const Matrix &b) {
//     Matrix result(a.size(), std::vector<double>(a[0].size()));
//     for (size_t i = 0; i < a.size(); ++i) {
//         for (size_t j = 0; j < a[0].size(); ++j) {
//             result[i][j] = a[i][j] + b[i][j];
//         }
//     }
//     return result;
// }

// Subtracts two matrices element-wise
// Matrix LQR::matrixSubtract(const Matrix &a, const Matrix &b) {
//     Matrix result(a.size(), std::vector<double>(a[0].size()));
//     for (size_t i = 0; i < a.size(); ++i) {
//         for (size_t j = 0; j < a[0].size(); ++j) {
//             result[i][j] = a[i][j] - b[i][j];
//         }
//     }
//     return result;
// }

// Clamps each element in the vector to be within the specified min and max values
// Vector LQR::clampVector(const Vector &vec, double minVal, double maxVal) {
//     Vector clamped = vec;
//     for (size_t i = 0; i < clamped.size(); ++i) {
//         if (clamped[i] < minVal) {
//             clamped[i] = minVal;
//         } else if (clamped[i] > maxVal) {
//             clamped[i] = maxVal;
//         }
//     }
//     return clamped;
// }

// Computes the Euclidean norm of a vector
// double LQR::norm(const Vector &vec) {
//     double sum = 0.0;
//     for (double val : vec) {
//         sum += val * val;
//     }
//     return std::sqrt(sum);
// }

// Adds two vectors element-wise
// Vector LQR::vectorAdd(const Vector &a, const Vector &b) {
//     Vector result(a.size());
//     for (size_t i = 0; i < a.size(); ++i) {
//         result[i] = a[i] + b[i];
//     }
//     return result;
// }

// Matrix LQR::getB(double deltat) {
//     Matrix B = {
//         {0                  ,    l*deltat/ I_xxt},
//         {l*deltat/ I_xxt    ,                  0},
//         {0                  ,                  0},
//         {0                  ,                  0}
//     };
//     return B;
// }

// Matrix LQR::getA(double deltat) {
//     Matrix A = {
//         {0+1                ,a_const*deltat      ,    0             ,                 0},
//         {-a_const*deltat  ,0 +1                  ,    0             ,                 0},
//         {0                ,-nz_eq*deltat      ,    0   +1          ,       r_eq*deltat},
//         {nz_eq*deltat     ,0                   ,  -r_eq*deltat    ,                 0 + 1}
//     };
//     return A;
// }


void lqr(double actual_state[STATE_SIZE], double desired_state[STATE_SIZE],
         double Q[STATE_SIZE][STATE_SIZE], double R[CONTROL_SIZE][CONTROL_SIZE],
         double A[STATE_SIZE][STATE_SIZE], double B[STATE_SIZE][CONTROL_SIZE],
         double dt, double optimal_control_input[CONTROL_SIZE]) {

    double x_error[STATE_SIZE];

    for (int i = 0; i < STATE_SIZE; ++i) {
        x_error[i] = desired_state[i] - actual_state[i];
    }

    // Placeholder K matrix (use proper computation if needed)
    double K[CONTROL_SIZE][STATE_SIZE] = {
        // {2.80224562e-16, 1.48291299e+00, -5.43459898e+00, -3.01180852e+00},
        // {1.48291299e+00, 1.20096241e-16, -3.01180852e+00, 5.43459898e+00}
        { 1.5827832246392713e-16, 0.31101679923644965, -0.40982308572437715, -2.680113543501652 },
     { 0.311016799236451, 1.973058540303749e-16, -2.6801135435016796, 0.40982308572437476 }
    };

    // Compute control input: u = K * x_error
    multiply(K, x_error, optimal_control_input);
}

// LQR control function
void LQR::getControlInputs(double actual_state[4], int detected_motor,
                      double optimal_control_input[2]) {


    if(detected_motor == 1 || detected_motor == 3) {
        r_eq = +8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
    } else {
        r_eq = -8.5;
        a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt;
    }

    double dt = 1.0;

    // Initialize desired_state array
    double desired_state[4] = {0.0,0.0,0.0,0.0};

    // if(detected_motor == 1) {
        desired_state[0] = 0.0;
        desired_state[1] = 5.486146414446868;
        desired_state[2] = 0.0;
        desired_state[3] = 0.4272152616685171;
    // } else if(detected_motor == 2) {
    //     desired_state[0] = 0.0;
    //     desired_state[1] = -2.53;
    //     desired_state[2] = 0.0;
    //     desired_state[3] = 0.2855;
    // } else if(detected_motor == 3) {
    //     desired_state[0] = 2.53;
    //     desired_state[1] = 0.0;
    //     desired_state[2] = -0.2855;
    //     desired_state[3] = 0.0;
    // } else if(detected_motor == 4) {
    //     desired_state[0] = -2.53;
    //     desired_state[1] = 0.0;
    //     desired_state[2] = 0.2855;
    //     desired_state[3] = 0.0;
    // }

    // R and Q matrices
    double R[2][2] = {{1.0, 0.0},
                     {0.0, 1.0}};


    double Q[4][4] = {{1.0, 0.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 20.0, 0.0},
                      {1.0, 0.0, 0.0, 20.0}};

    // State error: actual_state - desired_state
    double state_error[4];
    for (size_t j = 0; j < 4; ++j) {
        state_error[j] = actual_state[j] - desired_state[j];
    }

    // Desired state error
    double desired_state_error[4] = {0.0, 0.0, 0.0, 0.0};

    // A and B matrices
    double A[4][4] = {
        {1.0, a_const * dt, 0.0, 0.0},
        {-a_const * dt, 1.0, 0.0, 0.0},
        {0.0, -nz_eq * dt, 1.0, r_eq * dt},
        {nz_eq * dt, 0.0, -r_eq * dt, 1.0}
    };

    double B[4][2] = {
        {0.0, l * dt / I_xxt},
        {l * dt / I_xxt, 0.0},
        {0.0, 0.0},
        {0.0, 0.0}
    };

    // Perform LQR calculation (you should implement lqr function)
    double optimal_control_input_temp[2];  // To store result of lqr computation
    lqr(state_error, desired_state_error, Q, R, A, B, dt, optimal_control_input_temp);

    // Copy result into the output array
    for (int i = 0; i < 2; ++i) {
        optimal_control_input[i] = optimal_control_input_temp[i];
    }
}


