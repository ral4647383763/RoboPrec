use crate::types::alias::{Vector, Matrix};

pub struct RobotInfo<const N: usize> {
    pub n_joints: usize,
    pub limi_translations: Vec<Vector<3>>,
    pub calc_limi: Box<dyn Fn(Matrix<3, 3>, usize) -> Matrix<3, 3>>,
    pub joint_axes: Vec<usize>,
    pub levers: Vec<Vector<3>>,
    pub masses: Vector<N>,
    pub inertias: Vec<Matrix<3, 3>>
}

