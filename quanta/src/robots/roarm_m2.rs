use crate::types::alias::{Scalar, Vector, Matrix};
use super::robot_info::RobotInfo;

use super::helper::sin_cos_extremes;

pub fn calc_limi( // Made pub to match potential usage in RobotInfo
    rotation_matrix_joint: Matrix<3, 3>, // This is R_Z(theta_i) for the current joint's motion
    joint_index: usize,
) -> Matrix<3, 3> {
    // Determine RPY angle values (as f64 for matching) based on joint_index
    // These are the literal values from the URDF rpy attributes
    let (roll_angle_literal, pitch_angle_literal, yaw_angle_literal) = if joint_index == 0 {
        // URDF: base_link_to_link1: rpy="0 0 0"
        (0.0, 0.0, 0.0)
    } else if joint_index == 1 {
        // URDF: link1_to_link2: rpy="-1.5708 -1.5708 0"
        (-1.5708, -1.5708, 0.0)
    } else if joint_index == 2 {
        // URDF: link2_to_link3: rpy="0 0 1.5708"
        (0.0, 0.0, 1.5708)
    } else if joint_index == 3 {
        // URDF: link3_to_gripper_link: rpy="-1.5708 0 -1.5708"
        (-1.5708, 0.0, -1.5708)
    } else {
        panic!("Unsupported joint_index for calc_limi: {}", joint_index);
    };

    // Determine cos_r, sin_r based on roll_angle_literal, using direct numerical sin/cos values
    let (cos_r, sin_r) = match roll_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(0.9999999932519606)), // cos(1.5708), sin(1.5708)
        -1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(-0.9999999932519606)),// cos(-1.5708), sin(-1.5708)
        _ => panic!("Internal error: Roll angle literal {} not handled directly for sin/cos", roll_angle_literal),
    };

    // Determine cos_p, sin_p based on pitch_angle_literal
    let (cos_p, sin_p) = match pitch_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(0.9999999932519606)), // cos(1.5708), sin(1.5708)
        -1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(-0.9999999932519606)),// cos(-1.5708), sin(-1.5708)
        _ => panic!("Internal error: Pitch angle literal {} not handled directly for sin/cos", pitch_angle_literal),
    };

    // Determine cos_y, sin_y based on yaw_angle_literal
    let (cos_y, sin_y) = match yaw_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(0.9999999932519606)), // cos(1.5708), sin(1.5708)
        -1.5708 => (Scalar::new(-0.000003673205100000002), Scalar::new(-0.9999999932519606)),// cos(-1.5708), sin(-1.5708)
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

// Helper function (as defined in previous response, repeated here for completeness)
// Creates rotation matrix from pre-calculated trigonometric components
// RPY order is ZYX intrinsic: Yaw (Z), Pitch (Y), Roll (X)
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

pub fn roarm_m2_get_bounds() -> Vec<((f64, f64), (f64, f64))> {
    let joint_bounds = vec![
        (-3.1416, 3.1416),    // base_link_to_link1
        (-1.5708, 1.5708),    // link1_to_link2
        (-1.0, 2.95),         // link2_to_link3
        (0.0, 1.5)            // link3_to_gripper_link
    ];

    let minmax_sincos = vec![
        sin_cos_extremes(joint_bounds[0].0, joint_bounds[0].1),
        sin_cos_extremes(joint_bounds[1].0, joint_bounds[1].1),
        sin_cos_extremes(joint_bounds[2].0, joint_bounds[2].1),
        sin_cos_extremes(joint_bounds[3].0, joint_bounds[3].1)
    ];

    minmax_sincos
}

pub fn roarm_m2() -> RobotInfo<4> {

    let limi_translations = vec![
        Vector::new([0.0100000008759151, 0.0, 0.123059270461044]).define("limi_translation_0"), // base_link_to_link1
        Vector::new([0.0, 0.0, 0.0]).define("limi_translation_1"),                            // link1_to_link2
        Vector::new([0.236815132922094, 0.0300023995170449, 0.0]).define("limi_translation_2"), // link2_to_link3
        Vector::new([0.002906, -0.21599, -0.00066683]).define("limi_translation_3")             // link3_to_gripper_link
    ];

    let n_joints = 4;

    let levers = vec![
        Vector::new([0.0, 0.0, -0.015]).define("lever_0"),           // link1
        Vector::new([0.122, 0.0, 0.0]).define("lever_1"),            // link2
        Vector::new([0.002, -0.13687, 0.0059]).define("lever_2"),    // link3
        Vector::new([0.029, 0.0027, -0.00078]).define("lever_3")     // gripper_link
    ];


    let masses = Vector::new([0.0729177, 0.0703216, 0.0269773, 0.0028708]).define("masses"); // link1, link2, link3, gripper_link

    let inertias = vec![
        // Inertia for link1 from URDF
        Matrix::<3, 3>::new([
            [Scalar::new(4.68465E-05), Scalar::new(0.0), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(3.32107E-05), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(5.01023E-05)]
        ]).define("inertia_0"),
        // Inertia for link2 from URDF
        Matrix::<3, 3>::new([
            [Scalar::new(5.46501E-05), Scalar::new(0.0), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.000423091), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(0.000404557)]
        ]).define("inertia_1"),
        // Inertia for link3 from URDF
        Matrix::<3, 3>::new([
            [Scalar::new(0.000199591), Scalar::new(0.0), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(8.01674E-06), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(0.000196918)]
        ]).define("inertia_2"),
        // Inertia for gripper_link from URDF
        Matrix::<3, 3>::new([
            [Scalar::new(5.23216E-07), Scalar::new(0.0), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(1.82071E-06), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(1.60231E-06)]
        ]).define("inertia_3")
    ];

    // All revolute joints in the URDF specify <axis xyz="0 0 1"/> (Z-axis)
    let joint_axes = vec![2, 2, 2, 2];

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