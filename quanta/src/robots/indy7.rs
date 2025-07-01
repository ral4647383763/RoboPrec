use crate::types::alias::{Scalar, Vector, Matrix};
use super::robot_info::RobotInfo;

use super::helper::sin_cos_extremes;


fn rpy_to_matrix_from_trig_components(
    cos_r: Scalar, sin_r: Scalar, // cos(roll), sin(roll)
    cos_p: Scalar, sin_p: Scalar, // cos(pitch), sin(pitch)
    cos_y: Scalar, sin_y: Scalar, // cos(yaw), sin(yaw)
) -> Matrix<3, 3> {
    // R = R_Z(yaw) * R_Y(pitch) * R_X(roll)
    let r11 = &cos_y * &cos_p;
    let r12 = &(&(&cos_y * &sin_p) * &sin_r) - &(&sin_y * &cos_r);
    let r13 = &(&cos_y * &sin_p) * &cos_r + (&sin_y * &sin_r);

    let r21 = &sin_y * &cos_p;
    let r22 = (&(&sin_y * &sin_p) * &sin_r) + (&cos_y * &cos_r);
    let r23 = &(&(&sin_y * &sin_p) * &cos_r) - &(&cos_y * &sin_r);

    let r31 = -&sin_p;
    let r32 = &cos_p * &sin_r;
    let r33 = &cos_p * &cos_r;

    Matrix::<3, 3>::new([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33],
    ])
}

pub fn calc_limi(
    rotation_matrix_joint: Matrix<3, 3>, // This is R_Z(theta_i) for the current joint's motion
    joint_index: usize,
) -> Matrix<3, 3> {
    // Determine RPY angle values (as f64 for matching) based on joint_index
    // These are the literal values from the Indy7 URDF rpy attributes
    let (roll_angle_literal, pitch_angle_literal, yaw_angle_literal) = if joint_index == 0 {
        // URDF: joint0: rpy="0 0 0"
        (0.0, 0.0, 0.0)
    } else if joint_index == 1 {
        // URDF: joint1: rpy="1.570796327 1.570796327 0"
        (1.570796327, 1.570796327, 0.0)
    } else if joint_index == 2 {
        // URDF: joint2: rpy="0 0 0"
        (0.0, 0.0, 0.0)
    } else if joint_index == 3 {
        // URDF: joint3: rpy="-1.570796327 0 1.570796327"
        (-1.570796327, 0.0, 1.570796327)
    } else if joint_index == 4 {
        // URDF: joint4: rpy="1.570796327 1.570796327 0"
        (1.570796327, 1.570796327, 0.0)
    } else if joint_index == 5 {
        // URDF: joint5: rpy="-1.570796327 0 1.570796327"
        (-1.570796327, 0.0, 1.570796327)
    } else {
        panic!("Unsupported joint_index for Indy7 calc_limi: {}", joint_index);
    };

    // Determine cos_r, sin_r based on roll_angle_literal, using direct numerical sin/cos values
    let (cos_r, sin_r) = match roll_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(1.0)), // cos(pi/2_urdf), sin(pi/2_urdf)
        -1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(-1.0)),// cos(-pi/2_urdf), sin(-pi/2_urdf)
        _ => panic!("Internal error: Roll angle literal {} not handled directly for sin/cos", roll_angle_literal),
    };

    // Determine cos_p, sin_p based on pitch_angle_literal
    let (cos_p, sin_p) = match pitch_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(1.0)), // cos(pi/2_urdf), sin(pi/2_urdf)
        -1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(-1.0)),// cos(-pi/2_urdf), sin(-pi/2_urdf)
        _ => panic!("Internal error: Pitch angle literal {} not handled directly for sin/cos", pitch_angle_literal),
    };

    // Determine cos_y, sin_y based on yaw_angle_literal
    let (cos_y, sin_y) = match yaw_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(1.0)), // cos(pi/2_urdf), sin(pi/2_urdf)
        -1.570796327 => (Scalar::new(-7.549790482024008e-9), Scalar::new(-1.0)),// cos(-pi/2_urdf), sin(-pi/2_urdf)
        _ => panic!("Internal error: Yaw angle literal {} not handled directly for sin/cos", yaw_angle_literal),
    };

    // Construct the R_fix matrix using the selected trig components
    let r_fix = rpy_to_matrix_from_trig_components(cos_r, sin_r, cos_p, sin_p, cos_y, sin_y);

    // The final rotation is R_fix * R_joint(theta_i)
    // Assuming your Matrix<3,3> type has `impl Mul for Matrix<3,3>`
    let result_matrix = &r_fix * &rotation_matrix_joint;
    
    // Maintaining your define style for debugging output
    result_matrix.define(format!("limi_rotation_{}", joint_index).as_str())
}


pub fn ind7_get_bounds() -> Vec<((f64, f64), (f64, f64))> {
    let joint_bounds = vec![
        (-3.0543261909900767, 3.0543261909900767),
        (-3.0543261909900767, 3.0543261909900767),
        (-3.0543261909900767, 3.0543261909900767),
        (-3.0543261909900767, 3.0543261909900767),
        (-3.0543261909900767, 3.0543261909900767),
        (-3.7524578917878086, 3.7524578917878086),
    ];

    let minmax_sincos = vec![
        sin_cos_extremes(joint_bounds[0].0, joint_bounds[0].1),
        sin_cos_extremes(joint_bounds[1].0, joint_bounds[1].1),
        sin_cos_extremes(joint_bounds[2].0, joint_bounds[2].1),
        sin_cos_extremes(joint_bounds[3].0, joint_bounds[3].1),
        sin_cos_extremes(joint_bounds[4].0, joint_bounds[4].1),
        sin_cos_extremes(joint_bounds[5].0, joint_bounds[5].1),
    ];

    minmax_sincos
}


pub fn indy7() -> RobotInfo<6> {

    let limi_translations = vec![
        Vector::<3>::new([0.0, 0.0, 0.0775]).define("limi_translation_0"),
        Vector::<3>::new([0.0, -0.109, 0.222]).define("limi_translation_1"),
        Vector::<3>::new([-0.45, 0.0, -0.0305]).define("limi_translation_2"),
        Vector::<3>::new([-0.267, 0.0, -0.075]).define("limi_translation_3"),
        Vector::<3>::new([0.0, -0.114, 0.083]).define("limi_translation_4"),
        Vector::<3>::new([-0.168, 0.0, 0.069]).define("limi_translation_5")
    ];

    let n_joints = 6;

    let levers = vec![
        Vector::<3>::new([-0.00023749, -0.04310313, 0.13245396]).define("lever_0"),
        Vector::<3>::new([-0.29616699, 2.254e-05, 0.04483069]).define("lever_1"),
        Vector::<3>::new([-0.16804016, 0.00021421, -0.07000383]).define("lever_2"),
        Vector::<3>::new([-0.00026847, -0.0709844, 0.07649128]).define("lever_3"),
        Vector::<3>::new([-0.09796232, -0.00023114, 0.06445892]).define("lever_4"),
        Vector::<3>::new([8.147e-05, -0.00046556, 0.03079097]).define("lever_5")
    ];


    let masses = Vector::<6>::new([11.44444535, 5.84766553, 2.68206064, 2.12987371, 2.22412271, 0.38254932]).define("masses");

    let inertias = vec![
        Matrix::<3, 3>::new([
            [0.35065005, 0.00011931, -0.00037553],
            [0.00011931, 0.304798, -0.10984447],
            [-0.00037553, -0.10984447, 0.06003147]
        ]).define("inertia_0"),
        Matrix::<3, 3>::new([
            [0.03599743, -4.693e-05, -0.05240346],
            [-4.693e-05, 0.72293306, 1.76e-06],
            [-0.05240346, 1.76e-06, 0.70024119]
        ]).define("inertia_1"),
        Matrix::<3, 3>::new([
            [0.0161721, -0.00011817, 0.03341882],
            [-0.00011817, 0.11364055, -4.371e-05],
            [0.03341882, -4.371e-05, 0.10022522]
        ]).define("inertia_2"),
        Matrix::<3, 3>::new([
            [0.02798891, 3.893e-05, -4.768e-05],
            [3.893e-05, 0.01443076, -0.01266296],
            [-4.768e-05, -0.01266296, 0.01496211]
        ]).define("inertia_3"),
        Matrix::<3, 3>::new([
            [0.01105297, 5.517e-05, -0.01481977],
            [5.517e-05, 0.03698291, -3.74e-05],
            [-0.01481977, -3.74e-05, 0.02754795]
        ]).define("inertia_4"),
        Matrix::<3, 3>::new([
            [0.00078982, -3.4e-07, 8.3e-07],
            [-3.4e-07, 0.00079764, -5.08e-06],
            [8.3e-07, -5.08e-06, 0.00058319]
        ]).define("inertia_5")
    ];

    let joint_axes = vec![2, 2, 2, 2, 2, 2];

    RobotInfo {
        n_joints: n_joints,
        limi_translations: limi_translations,
        calc_limi: Box::new(calc_limi),
        joint_axes: joint_axes,
        levers: levers,
        masses: masses,
        inertias: inertias
    }
}



