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
    const rneaderiv_return& custom_results,
    Eigen::MatrixXd djoint_torque_dq,
    Eigen::MatrixXd djoint_torque_dv,
    Eigen::MatrixXd djoint_torque_da
) 
{
    double current_iteration_max_err = 0.0;

    for (int j = 0; j < NU; ++j) {
        for (int k = 0; k < NU; ++k) {
            double pinocchio_partial_da = djoint_torque_da(j, k);
            double custom_partial_da = custom_results.partial_da[j][k];
            double err = std::abs(pinocchio_partial_da - custom_partial_da);
            current_iteration_max_err = std::max(current_iteration_max_err, err);

            double pinocchio_partial_dv = djoint_torque_dv(j, k);
            double custom_partial_dv = custom_results.partial_dv[j][k];
            err = std::abs(pinocchio_partial_dv - custom_partial_dv);
            current_iteration_max_err = std::max(current_iteration_max_err, err);

            double pinocchio_partial_dq = djoint_torque_dq(j, k);
            double custom_partial_dq = custom_results.partial_dq[j][k];
            err = std::abs(pinocchio_partial_dq - custom_partial_dq);
            current_iteration_max_err = std::max(current_iteration_max_err, err);
        }
    }
    return current_iteration_max_err;
}


int main() {
    using namespace pinocchio;

    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string(
        "example-robot-data/robots/"
        "roarm_m3_description/roarm_m2.urdf");

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
    double max_err_custom_fixed16 = 0.0;
    double max_err_custom_fixed13_19 = 0.0;

    if (model.nv != NU) {
        std::cerr << "Error: The robot model has " << model.nv << " degrees of freedom, but the FK_Roarm2_* functions expect inputs for " << NU << " DoFs." << std::endl;
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

        // Allocate result containers.
        Eigen::MatrixXd djoint_torque_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
        Eigen::MatrixXd djoint_torque_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
        Eigen::MatrixXd djoint_torque_da = Eigen::MatrixXd::Zero(model.nv, model.nv);

        // call rneaderiv
        pinocchio::computeRNEADerivatives(
            model, data_ref, q_pin_double, v_pin_double, a_pin_double,
            djoint_torque_dq, djoint_torque_dv, djoint_torque_da
        );

        // Prepare arguments for your FK_Roarm2_* functions
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
        rneaderiv_return ret_custom_double = RNEADERIV_Roarm2_double(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3]
        );
        double current_err_double = calculate_error(ret_custom_double, 
                                    djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
        max_err_custom_double = std::max(max_err_custom_double, current_err_double);

        // --- Custom Float Precision ---
        rneaderiv_return ret_custom_float = RNEADERIV_Roarm2_float(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3]
        );
        double current_err_float = calculate_error(ret_custom_float, 
                                    djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
        max_err_custom_float = std::max(max_err_custom_float, current_err_float);
        
        // --- Custom Fixed32 Precision ---
        rneaderiv_return ret_custom_fixed32 = RNEADERIV_Roarm2_fixed32(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3]
        );
        double current_err_fixed32 = calculate_error(ret_custom_fixed32,
                                    djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
        max_err_custom_fixed32 = std::max(max_err_custom_fixed32, current_err_fixed32);

        // --- Custom Fixed16 Precision ---
        rneaderiv_return ret_custom_fixed16 = RNEADERIV_Roarm2_fixed16(
            cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3],
            sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3],
            v_vals[0], v_vals[1], v_vals[2], v_vals[3],
            a_vals[0], a_vals[1], a_vals[2], a_vals[3]
        );
        double current_err_fixed16 = calculate_error(ret_custom_fixed16,
                                    djoint_torque_dq, djoint_torque_dv, djoint_torque_da);
        max_err_custom_fixed16 = std::max(max_err_custom_fixed16, current_err_fixed16);

        // // --- Custom Fixed13-19 Precision ---
        // rnea_return ret_custom_fixed13_19 = RNEA_Roarm2_fixed13_19(
        //     cos_q_vals[0], cos_q_vals[1], cos_q_vals[2], cos_q_vals[3], cos_q_vals[4], cos_q_vals[5],
        //     sin_q_vals[0], sin_q_vals[1], sin_q_vals[2], sin_q_vals[3], sin_q_vals[4], sin_q_vals[5],
        //     v_vals[0], v_vals[1], v_vals[2], v_vals[3], v_vals[4], v_vals[5],
        //     a_vals[0], a_vals[1], a_vals[2], a_vals[3], a_vals[4], a_vals[5]
        // );
        // double current_err_fixed13_19 = calculate_error(ret_custom_fixed13_19, data_ref);
        // max_err_custom_fixed13_19 = std::max(max_err_custom_fixed13_19, current_err_fixed13_19);
        
    }
    
    std::cout << std::fixed << std::setprecision(1e-1 == 0.1 ? 17 : 16); // Set precision for error output
    std::cout << "Max error (Custom Double vs. Pinocchio Double): " << max_err_custom_double << std::endl;
    std::cout << "Max error (Custom Float vs. Pinocchio Double):  " << max_err_custom_float << std::endl;
    std::cout << "Max error (Custom Fixed32 vs. Pinocchio Double): " << max_err_custom_fixed32 << std::endl;
    std::cout << "Max error (Custom Fixed16 vs. Pinocchio Double): " << max_err_custom_fixed16 << std::endl;
    
    return 0;
}