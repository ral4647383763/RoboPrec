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
#include "fixed14_18.hpp"
#include "fixed16.hpp"

#include "../../util/correctness_utils.hpp"

// Pinocchio includes
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR "/home/pinocchio/models/" // Default path, adjust if necessary
#endif



double calculate_error(
    const rnea_return& custom_results,
    const pinocchio::DataTpl<double>& pinocchio_data) 
{
    double current_iteration_max_err = 0.0;

    for (int j = 0; j < NU; ++j) {
        // Calculate the error between the custom results and Pinocchio's results
        double custom_tau = custom_results.taus[j];
        double pinocchio_tau = pinocchio_data.tau[j];

        // Calculate the absolute error
        double err = std::abs(custom_tau - pinocchio_tau);

        // Update the maximum error found in this iteration
        current_iteration_max_err = std::max(current_iteration_max_err, err);
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
    double max_err_custom_fixed14_18 = 0.0;
    double max_err_custom_fixed16 = 0.0;

    if (model.nv != NU) {
        std::cerr << "Error: The robot model has " << model.nv << " degrees of freedom, but the FK_Panda_* functions expect inputs for " << NU << " DoFs." << std::endl;
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
        pinocchio::rnea(model, data_ref, q_pin_double, v_pin_double, a_pin_double);
        
        // Prepare arguments for your FK_Panda_* functions
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
        rnea_return ret_custom_double = RNEA_Panda_double(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_double = calculate_error(ret_custom_double, data_ref);
        max_err_custom_double = std::max(max_err_custom_double, current_err_double);

        // --- Custom Float Precision ---
        rnea_return ret_custom_float = RNEA_Panda_float(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_float = calculate_error(ret_custom_float, data_ref);
        max_err_custom_float = std::max(max_err_custom_float, current_err_float);
        
        // --- Custom Fixed32 Precision ---
        rnea_return ret_custom_fixed32 = RNEA_Panda_fixed32(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed32 = calculate_error(ret_custom_fixed32, data_ref);
        max_err_custom_fixed32 = std::max(max_err_custom_fixed32, current_err_fixed32);
        
        // --- Custom Fixed14-18 Precision ---
        rnea_return ret_custom_fixed14_18 = RNEA_Panda_fixed14_18(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed14_18 = calculate_error(ret_custom_fixed14_18, data_ref);
        max_err_custom_fixed14_18 = std::max(max_err_custom_fixed14_18, current_err_fixed14_18);

        // --- Custom Fixed16 Precision ---
        rnea_return ret_custom_fixed16 = RNEA_Panda_fixed16(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5], cos_q_vals[6],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5], sin_q_vals[6],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5], v_vals[6],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5], a_vals[6]
        );
        double current_err_fixed16 = calculate_error(ret_custom_fixed16, data_ref);
        max_err_custom_fixed16 = std::max(max_err_custom_fixed16, current_err_fixed16);

    }
    
    std::cout << std::fixed << std::setprecision(1e-1 == 0.1 ? 17 : 16); // Set precision for error output
    std::cout << "Max error (Custom Double vs. Pinocchio Double): " << max_err_custom_double << std::endl;
    std::cout << "Max error (Custom Float vs. Pinocchio Double):  " << max_err_custom_float << std::endl;
    std::cout << "Max error (Custom Fixed32 vs. Pinocchio Double): " << max_err_custom_fixed32 << std::endl;
    std::cout << "Max error (Custom Fixed14-18 vs. Pinocchio Double): " << max_err_custom_fixed14_18 << std::endl;
    std::cout << "Max error (Custom Fixed16 vs. Pinocchio Double): " << max_err_custom_fixed16 << std::endl;

    return 0;
}