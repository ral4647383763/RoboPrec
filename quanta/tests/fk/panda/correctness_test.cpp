#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <algorithm> // For std::max
#include <iomanip>   // For std::fixed and std::setprecision


#include "double.hpp"
#include "float.hpp"
#include "fixed32.hpp"
#include "fixed7-25.hpp"
#include "fixed16.hpp"

#include "../../util/correctness_utils.hpp"

// Pinocchio includes
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp" // For forwardKinematics derivatives
#include "pinocchio/algorithm/kinematics.hpp"       // For forwardKinematics

#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR "/home/pinocchio/models/" // Default path, adjust if necessary
#endif


// --- Helper Function to Calculate Maximum Error ---
// This function compares the results from your custom FK function (in fk_return)
// with the reference Pinocchio data.
// - custom_fk_results: The output from your FK_Roarm3_* function.
// - pinocchio_data: The Data object from Pinocchio, populated by its forwardKinematics.
// - num_joints_to_check: The number of joints whose data should be compared (typically model.nv).
double calculate_forward_kinematics_error(
    const fk_return& custom_fk_results,
    const pinocchio::DataTpl<double>& pinocchio_data) 
{
    double current_iteration_max_err = 0.0;

    for (int j = 0; j < NU; ++j) {
        // Pinocchio's joint data is typically 1-indexed (joint 0 is often the universe)
        // So, we compare custom_fk_results.field[j] with pinocchio_data.field[j+1]

        // 1. Compare oMi (Homogeneous Transformation Matrices for each joint)
        // Assuming custom_fk_results.oMis[j].rotation is a 3x3 array/matrix like structure
        // And pinocchio_data.oMi[j+1].rotation() is an Eigen 3x3 matrix.
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                current_iteration_max_err = std::max(current_iteration_max_err,
                    std::abs(custom_fk_results.oMis[j].rotation[r][c] - pinocchio_data.oMi[j + 1].rotation()(r, c)));
            }
        }

        for (int r = 0; r < 3; ++r) {
            current_iteration_max_err = std::max(current_iteration_max_err,
                std::abs(custom_fk_results.oMis[j].translation[r] - pinocchio_data.oMi[j + 1].translation()(r)));
        }

        // 2. Compare v (Spatial Velocities for each joint)
        // Assuming custom_fk_results.v[j] is an array/vector of 6 elements: [vx, vy, vz, wx, wy, wz]
        // Pinocchio's data.v[j+1] is a pinocchio::Motion object.
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][0] - pinocchio_data.v[j + 1].linear()(0)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][1] - pinocchio_data.v[j + 1].linear()(1)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][2] - pinocchio_data.v[j + 1].linear()(2)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][3] - pinocchio_data.v[j + 1].angular()(0)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][4] - pinocchio_data.v[j + 1].angular()(1)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.v[j][5] - pinocchio_data.v[j + 1].angular()(2)));

        // 3. Compare a (Spatial Accelerations for each joint)
        // Assuming custom_fk_results.a[j] is an array/vector of 6 elements: [ax, ay, az, ang_acc_x, ang_acc_y, ang_acc_z]
        // Pinocchio's data.a[j+1] is a pinocchio::Motion object.
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][0] - pinocchio_data.a[j + 1].linear()(0)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][1] - pinocchio_data.a[j + 1].linear()(1)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][2] - pinocchio_data.a[j + 1].linear()(2)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][3] - pinocchio_data.a[j + 1].angular()(0)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][4] - pinocchio_data.a[j + 1].angular()(1)));
        current_iteration_max_err = std::max(current_iteration_max_err, std::abs(custom_fk_results.a[j][5] - pinocchio_data.a[j + 1].angular()(2)));
    }
    return current_iteration_max_err;
}


int main() {
    using namespace pinocchio;

    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string(
        "example-robot-data/robots/"
        "panda_description/urdf/panda.urdf");

    // Use double precision for the reference Pinocchio model and data
    ModelTpl<double> model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    DataTpl<double> data_ref(model); // Reference data

    // Joint configuration, velocity, and acceleration vectors (using double for generation)
    Eigen::VectorXd q_pin_double = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd v_pin_double = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_pin_double = Eigen::VectorXd::Zero(model.nv);

    // Variables to store the maximum error found across all samples for each precision
    double max_err_custom_double = 0.0;
    double max_err_custom_float = 0.0;
    double max_err_custom_fixed32 = 0.0;
    double max_err_custom_fixed7_25 = 0.0;
    double max_err_custom_fixed16 = 0.0;

    if (model.nv != NU) {
        std::cerr << "Error: The robot model has " << model.nv << " degrees of freedom, but the FK_Roarm3_* functions expect inputs for " << NU << " DoFs." << std::endl;
        return 1;
    }

    std::cout << "Running " << NUMBER_OF_SAMPLES << " samples." << std::endl;
    std::cout << "Model " << model.name << " has " << model.nv << " DoF." << std::endl;

    for (int i = 0; i < NUMBER_OF_SAMPLES; ++i) {
        // Randomize q, v, a for Pinocchio's double precision vectors
        q_pin_double = randomConfiguration(model); // More robust way to get random configurations
        v_pin_double = Eigen::VectorXd::Random(model.nv);
        v_pin_double = 0.5 * v_pin_double; // Scale velocities to be between -0.5 and 0.5
        a_pin_double = Eigen::VectorXd::Random(model.nv);


        // Run Pinocchio's forward kinematics using double precision (this is our reference)
        pinocchio::forwardKinematics(model, data_ref, q_pin_double, v_pin_double, a_pin_double);
        
        // Prepare arguments for your FK_Roarm3_* functions
        // These functions seem to take cos(q_i), sin(q_i), v_i, a_i for i=0 to 4.
        double cos_q_vals[NU], sin_q_vals[NU];
        double v_vals[NU], a_vals[NU];

        for(int k=0; k < NU; ++k) {
            cos_q_vals[k] = cos(q_pin_double[k]);
            sin_q_vals[k] = sin(q_pin_double[k]);
            v_vals[k]     = v_pin_double[k];
            a_vals[k]     = a_pin_double[k];
        }

        // --- Custom Double Precision ---
        fk_return ret_custom_double = FK_Panda_double(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_double = calculate_forward_kinematics_error(ret_custom_double, data_ref);
        max_err_custom_double = std::max(max_err_custom_double, current_err_double);

        // --- Custom Float Precision ---
        fk_return ret_custom_float = FK_Panda_float(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_float = calculate_forward_kinematics_error(ret_custom_float, data_ref);
        max_err_custom_float = std::max(max_err_custom_float, current_err_float);
        
        // --- Custom Fixed32 Precision ---
        fk_return ret_custom_fixed32 = FK_Panda_fixed32(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed32 = calculate_forward_kinematics_error(ret_custom_fixed32, data_ref);
        max_err_custom_fixed32 = std::max(max_err_custom_fixed32, current_err_fixed32);
         
        // --- Custom Fixed7-25 Precision ---
        fk_return ret_custom_fixed7_25 = FK_Panda_fixed7_25(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed7_25 = calculate_forward_kinematics_error(ret_custom_fixed7_25, data_ref);
        max_err_custom_fixed7_25 = std::max(max_err_custom_fixed7_25, current_err_fixed7_25);
        
            // --- Custom Fixed16 Precision ---
        fk_return ret_custom_fixed16 = FK_Panda_fixed16(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed16 = calculate_forward_kinematics_error(ret_custom_fixed16, data_ref);
        max_err_custom_fixed16 = std::max(max_err_custom_fixed16, current_err_fixed16);
        
    }
    
    std::cout << std::fixed << std::setprecision(1e-1 == 0.1 ? 17 : 16); // Set precision for error output
    std::cout << "Max error (Custom Double vs. Pinocchio Double): " << max_err_custom_double << std::endl;
    std::cout << "Max error (Custom Float vs. Pinocchio Double):  " << max_err_custom_float << std::endl;
    std::cout << "Max error (Custom Fixed32 vs. Pinocchio Double): " << max_err_custom_fixed32 << std::endl;
    std::cout << "Max error (Custom Fixed7-25 vs. Pinocchio Double): " << max_err_custom_fixed7_25 << std::endl;
    
    return 0;
}