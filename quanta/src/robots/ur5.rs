use crate::types::alias::{Scalar, Vector, Matrix};
use super::robot_info::RobotInfo;

use super::helper::sin_cos_extremes;

// calc limi_rotation from rotation matrix
fn calc_limi(
    rotation_matrix: Matrix<3, 3>,
    joint_index: usize,
) -> Matrix<3, 3> {
    if joint_index == 0 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 0)), rotation_matrix.at((0, 1)), Scalar::new(0.0)],
            [rotation_matrix.at((1, 0)), rotation_matrix.at((1, 1)), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(1.0)]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else if joint_index == 1 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 1)), Scalar::new(0.0) ,rotation_matrix.at((0, 0))],
            [Scalar::new(0.0), Scalar::new(1.0), Scalar::new(0.0)],
            [-&rotation_matrix.at((1, 1)), Scalar::new(0.0), -&rotation_matrix.at((1, 0))]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else if joint_index == 2 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 0)), Scalar::new(0.0), -&rotation_matrix.at((0, 1))],
            [Scalar::new(0.0), Scalar::new(1.0), Scalar::new(0.0)],
            [-&rotation_matrix.at((1, 0)), Scalar::new(0.0), rotation_matrix.at((1, 1))]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else if joint_index == 3 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 1)), Scalar::new(0.0),rotation_matrix.at((0, 0))],
            [Scalar::new(0.0), Scalar::new(1.0), Scalar::new(0.0)],
            [-&rotation_matrix.at((1, 1)), Scalar::new(0.0), -&rotation_matrix.at((1, 0))]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else if joint_index == 4 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 0)), rotation_matrix.at((0, 1)), Scalar::new(0.0)],
            [rotation_matrix.at((1, 0)), rotation_matrix.at((1, 1)), Scalar::new(0.0)],
            [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(1.0)]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else if joint_index == 5 {
        Matrix::<3, 3>::new([
            [rotation_matrix.at((0, 0)), Scalar::new(0.0), -&rotation_matrix.at((0, 1))],
            [Scalar::new(0.0), Scalar::new(1.0), Scalar::new(0.0)],
            [-&rotation_matrix.at((1, 0)), Scalar::new(0.0), rotation_matrix.at((1, 1))]
        ]).define(format!("limi_rotation_{}", joint_index).as_str())
    } else {
        panic!("Unsupported joint_index")
    }
}

pub fn ur5_get_bounds() -> Vec<((f64, f64), (f64, f64))> {
    let joint_bounds = vec![
        (-3.1543261909900767, 3.1543261909900767),
        (-3.1543261909900767, 3.1543261909900767),
        (-3.1543261909900767, 3.1543261909900767),
        (-3.1543261909900767, 3.1543261909900767),
        (-3.1543261909900767, 3.1543261909900767),
        (-3.1543261909900767, 3.1543261909900767),
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

pub fn ur5() -> RobotInfo<6> {
    let limi_translations = vec![
        Vector::<3>::new([0.0, 0.0, 0.089159]).define("limi_translation_0"),
        Vector::<3>::new([0.0, 0.13585, 0.0]).define("limi_translation_1"),
        Vector::<3>::new([0.0, -0.1197, 0.425]).define("limi_translation_2"),
        Vector::<3>::new([0.0, 0.0, 0.39225]).define("limi_translation_3"),
        Vector::<3>::new([0.0, 0.093, 0.0]).define("limi_translation_4"),
        Vector::<3>::new([0.0, 0.0, 0.09465]).define("limi_translation_5")
    ];

    let n_joints = 6;

    let levers = vec![
        Vector::<3>::new([0.0, 0.0, 0.0]).define("lever_0"),
        Vector::<3>::new([0.0, 0.0, 0.28]).define("lever_1"),
        Vector::<3>::new([0.0, 0.0, 0.25]).define("lever_2"),
        Vector::<3>::new([0.0, 0.0, 0.0]).define("lever_3"),
        Vector::<3>::new([0.0, 0.0, 0.0]).define("lever_4"),
        Vector::<3>::new([0.0, 0.0, 0.0]).define("lever_5")
    ];

    let masses = Vector::<6>::new([3.7, 8.393, 2.275, 1.219, 1.219, 0.1879]).define("masses");

    let inertias = vec![
        Matrix::<3, 3>::new([
            [0.010267495893, 0.0, 0.0],
            [0.0, 0.010267495893, 0.0],
            [0.0, 0.0, 0.00666]
        ]).define("inertia_0"),
        Matrix::<3, 3>::new([
            [0.22689067591, 0.0, 0.0],
            [0.0, 0.22689067591, 0.0],
            [0.0, 0.0, 0.0151074]
        ]).define("inertia_1"),
        Matrix::<3, 3>::new([
            [0.049443313556, 0.0, 0.0],
            [0.0, 0.049443313556, 0.0],
            [0.0, 0.0, 0.004095]
        ]).define("inertia_2"),
        Matrix::<3, 3>::new([
            [0.111172755531, 0.0, 0.0],
            [0.0, 0.111172755531, 0.0],
            [0.0, 0.0, 0.21942]
        ]).define("inertia_3"),
        Matrix::<3, 3>::new([
            [0.111172755531, 0.0, 0.0],
            [0.0, 0.111172755531, 0.0],
            [0.0, 0.0, 0.21942]
        ]).define("inertia_4"),
        Matrix::<3, 3>::new([
            [0.0171364731454, 0.0, 0.0],
            [0.0, 0.0171364731454, 0.0],
            [0.0, 0.0, 0.033822]
        ]).define("inertia_5")
    ];


    let joint_axes = vec![2, 1, 1, 1, 2, 1];


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




