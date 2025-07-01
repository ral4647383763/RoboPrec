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
    // These are the literal values from the Panda URDF rpy attributes
    let (roll_angle_literal, pitch_angle_literal, yaw_angle_literal) = if joint_index == 0 {
        // URDF: panda_joint1: rpy="0 0 0"
        (0.0, 0.0, 0.0)
    } else if joint_index == 1 {
        // URDF: panda_joint2: rpy="-1.5707963267948966 0 0"
        (-1.5707963267948966, 0.0, 0.0)
    } else if joint_index == 2 {
        // URDF: panda_joint3: rpy="1.5707963267948966 0 0"
        (1.5707963267948966, 0.0, 0.0)
    } else if joint_index == 3 {
        // URDF: panda_joint4: rpy="1.5707963267948966 0 0"
        (1.5707963267948966, 0.0, 0.0)
    } else if joint_index == 4 {
        // URDF: panda_joint5: rpy="-1.5707963267948966 0 0"
        (-1.5707963267948966, 0.0, 0.0)
    } else if joint_index == 5 {
        // URDF: panda_joint6: rpy="1.5707963267948966 0 0"
        (1.5707963267948966, 0.0, 0.0)
    } else if joint_index == 6 {
        // URDF: panda_joint7: rpy="1.5707963267948966 0 0"
        (1.5707963267948966, 0.0, 0.0)
    } else {
        panic!("Unsupported joint_index for Panda calc_limi: {}", joint_index);
    };

    // Determine cos_r, sin_r based on roll_angle_literal, using direct numerical sin/cos values
    let (cos_r, sin_r) = match roll_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(1.0)), // cos(pi/2_panda), sin(pi/2_panda)
        -1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(-1.0)),// cos(-pi/2_panda), sin(-pi/2_panda)
        _ => panic!("Internal error: Roll angle literal {} not handled directly for sin/cos", roll_angle_literal),
    };

    // Determine cos_p, sin_p based on pitch_angle_literal
    let (cos_p, sin_p) = match pitch_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        // Panda URDF does not use non-zero pitch for these joints, but including for completeness if structure is copied
        1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(1.0)),
        -1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(-1.0)),
        _ => panic!("Internal error: Pitch angle literal {} not handled directly for sin/cos", pitch_angle_literal),
    };

    // Determine cos_y, sin_y based on yaw_angle_literal
    let (cos_y, sin_y) = match yaw_angle_literal {
        0.0 => (Scalar::new(1.0), Scalar::new(0.0)), // cos(0), sin(0)
        // Panda URDF does not use non-zero yaw for these joints, but including for completeness
        1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(1.0)),
        -1.5707963267948966 => (Scalar::new(6.123233995736766e-17), Scalar::new(-1.0)),
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


pub fn panda_get_bounds() -> Vec<((f64, f64), (f64, f64))> {
    let joint_bounds = vec![
        (-2.8973, 2.8973),    
        (-1.7628, 1.7628),    
        (-2.8973, 2.8973),    
        (-3.0718, -0.0698),   
        (-2.8973, 2.8973),    
        (-0.0175, 3.7525),    
        (-2.8973, 2.8973),    
    ];

    let minmax_sincos = vec![
        sin_cos_extremes(joint_bounds[0].0, joint_bounds[0].1),
        sin_cos_extremes(joint_bounds[1].0, joint_bounds[1].1),
        sin_cos_extremes(joint_bounds[2].0, joint_bounds[2].1),
        sin_cos_extremes(joint_bounds[3].0, joint_bounds[3].1),
        sin_cos_extremes(joint_bounds[4].0, joint_bounds[4].1),
        sin_cos_extremes(joint_bounds[5].0, joint_bounds[5].1),
        sin_cos_extremes(joint_bounds[6].0, joint_bounds[6].1), 
    ];

    minmax_sincos
}


pub fn panda() -> RobotInfo<7> {

    let limi_translations = vec![
        Vector::new([0.0, 0.0, 0.333]).define("limi_translation_0"),       
        Vector::new([0.0, 0.0, 0.0]).define("limi_translation_1"),         
        Vector::new([0.0, -0.316, 0.0]).define("limi_translation_2"),      
        Vector::new([0.0825, 0.0, 0.0]).define("limi_translation_3"),      
        Vector::new([-0.0825, 0.384, 0.0]).define("limi_translation_4"),   
        Vector::new([0.0, 0.0, 0.0]).define("limi_translation_5"),         
        Vector::new([0.088, 0.0, 0.0]).define("limi_translation_6"),       
    ];

    let n_joints = 7;

    let levers = vec![
        Vector::new([0.003875, 0.002081, -0.04762]).define("lever_0"),
        Vector::new([-0.003141, -0.02872, 0.003495]).define("lever_1"),
        Vector::new([0.027518, 0.039252, -0.066502]).define("lever_2"),
        Vector::new([-0.05317, 0.104419, 0.027454]).define("lever_3"),
        Vector::new([-0.011953, 0.041065, -0.038437]).define("lever_4"),
        Vector::new([0.060149, -0.014117, -0.010517]).define("lever_5"),
        Vector::new([0.010517, -0.004252, 0.061597]).define("lever_6"),
    ];


    let masses = Vector::new([4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 0.735522]).define("masses");

    let inertias = vec![
        Matrix::<3, 3>::new([
            [0.70337, -0.000139, 0.006772],
            [-0.000139, 0.70661, 0.019169],
            [0.006772, 0.019169, 0.009117]
        ]).define("inertia_0"),
        Matrix::<3, 3>::new([
            [0.007962, -0.003925, 0.010254],
            [-0.003925, 0.02811, 0.000704],
            [0.010254, 0.000704, 0.025995]
        ]).define("inertia_1"),
        Matrix::<3, 3>::new([
            [0.037242, -0.004761, -0.011396],
            [-0.004761, 0.036155, -0.012805],
            [-0.011396, -0.012805, 0.01083]
        ]).define("inertia_2"),
        Matrix::<3, 3>::new([
            [0.025853, 0.007796, -0.001332],
            [0.007796, 0.019552, 0.008641],
            [-0.001332, 0.008641, 0.028323]
        ]).define("inertia_3"),
        Matrix::<3, 3>::new([
            [0.035549, -0.002117, -0.004037],
            [-0.002117, 0.029474, 0.000229],
            [-0.004037, 0.000229, 0.008627]
        ]).define("inertia_4"),
        Matrix::<3, 3>::new([
            [0.001964, 0.000109, -0.001158],
            [0.000109, 0.004354, 0.000341],
            [-0.001158, 0.000341, 0.005433]
        ]).define("inertia_5"),
        Matrix::<3, 3>::new([
            [0.012516, -0.000428, -0.001196],
            [-0.000428, 0.010027, -0.000741],
            [-0.001196, -0.000741, 0.004815]
        ]).define("inertia_6")
    ];

    let joint_axes = vec![2, 2, 2, 2, 2, 2, 2]; // All joints are <axis xyz="0 0 1"/> (Z-axis)

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