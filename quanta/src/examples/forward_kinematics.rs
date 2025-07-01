use crate::types::alias::{Scalar, Vector, Matrix};
use crate::robots::robot_info::RobotInfo;
use crate::logging::set_function_output;



fn act_motion_inv(
    translation: Vector<3>,
    rotation: Matrix<3, 3>,
    current: Vector<6>,
    parent: Vector<6>,
    joint_index: usize,
) -> Vector<6> {
    let linear_parent = Vector::<3>::new([parent.at(0), parent.at(1), parent.at(2)]);
    let angular_parent = Vector::<3>::new([parent.at(3), parent.at(4), parent.at(5)]);
    let linear_current = Vector::<3>::new([current.at(0), current.at(1), current.at(2)]);
    let angular_current = Vector::<3>::new([current.at(3), current.at(4), current.at(5)]);

    let act_inv1 = translation.cross(&angular_parent);
    let act_inv2 = &linear_parent - &act_inv1;
    let act_inv3 = rotation.transpose();
    let act_inv4 = &act_inv3 * &act_inv2;
    let new_linear = &linear_current + &act_inv4;
    let act_inv5 = &act_inv3 * &angular_parent;
    let new_angular = &angular_current + &act_inv5;
    let act_inv_res = Vector::<6>::new([
        new_linear.at(0),
        new_linear.at(1),
        new_linear.at(2),
        new_angular.at(0),
        new_angular.at(1),
        new_angular.at(2),
    ]);
    let act_inv_res = act_inv_res.define(format!("act_motion_inv_{}", joint_index).as_str());

    act_inv_res
}


fn alpha_cross_linear(
    s: &Scalar,
    vin: Vector<3>,
    joint_index: usize
) -> Vector<3> {
    let alpha_cross1 = &(-s) * &vin.at(1);
    let alpha_cross2 = s * &vin.at(0);
    let alpha_cross = Vector::<3>::new([alpha_cross1, alpha_cross2, Scalar::new(0.0)]);
    let alpha_cross = alpha_cross.define(format!("alpha_cross_linear_{}", joint_index).as_str());

    alpha_cross
}


fn alpha_cross_angular(
    s: &Scalar,
    vin: Vector<3>,
    joint_index: usize
) -> Vector<3> {
    let alpha_cross1 = &(-s) * &vin.at(1);
    let alpha_cross2 = s * &vin.at(0);
    let alpha_cross = Vector::<3>::new([alpha_cross1, alpha_cross2, Scalar::new(0.0)]);
    let alpha_cross = alpha_cross.define(format!("alpha_cross_angular_{}", joint_index).as_str());
    alpha_cross
}

fn alpha_cross(
    s: Scalar,
    vin: Vector<6>,
    joint_index: usize
) -> Vector<6> {
    let vin_linear = Vector::<3>::new([
        vin.at(0),
        vin.at(1),
        vin.at(2),
    ]);
    let vin_angular = Vector::<3>::new([
        vin.at(3),
        vin.at(4),
        vin.at(5),
    ]);
    let alpha_cross_linear = alpha_cross_linear(&s, vin_linear, joint_index);
    let alpha_cross_angular = alpha_cross_angular(&s, vin_angular, joint_index);
    let alpha_cross = Vector::<6>::new([
        alpha_cross_linear.at(0),
        alpha_cross_linear.at(1),
        alpha_cross_linear.at(2),
        alpha_cross_angular.at(0),
        alpha_cross_angular.at(1),
        alpha_cross_angular.at(2),
    ]);
    let alpha_cross = alpha_cross.define(format!("alpha_cross_{}", joint_index).as_str());
    alpha_cross
}

fn act_inv(
    translation: Vector<3>,
    rotation: Matrix<3, 3>,
    parent: Vector<6>,
    joint_index: usize,
) -> Vector<6> {
    let linear_parent = Vector::<3>::new([parent.at(0), parent.at(1), parent.at(2)]);
    let angular_parent = Vector::<3>::new([parent.at(3), parent.at(4), parent.at(5)]);
    let act_inv1 = translation.cross(&angular_parent);
    let act_inv2 = &linear_parent - &act_inv1;
    let act_inv3 = rotation.transpose();
    let act_inv4 = &act_inv3 * &act_inv2;
    let act_inv5 = &act_inv3 * &angular_parent;

    let act_inv_res = Vector::<6>::new([
        act_inv4.at(0),
        act_inv4.at(1),
        act_inv4.at(2),
        act_inv5.at(0),
        act_inv5.at(1),
        act_inv5.at(2),
    ]);
    let act_inv_res = act_inv_res.define(format!("act_inv_{}", joint_index).as_str());

    act_inv_res
}

fn forward_kinematics_helper<const N: usize>(
    qcos: Vector<N>,
    qsin: Vector<N>,
    all_joint_v: Vector<N>,
    all_joint_a: Vector<N>,

    all_v: &mut Vec<Vector<6>>,
    all_a: &mut Vec<Vector<6>>,

    limi_translations: &Vec<Vector<3>>,
    limi_rotations: &mut Vec<Matrix<3, 3>>,

    omi_translations: &mut Vec<Vector<3>>,
    omi_rotations: &mut Vec<Matrix<3, 3>>,

    joint_index: usize,

    robot_info: &RobotInfo<N>,
) -> () {

    let rotation_matrix = Matrix::<3, 3>::new([
        [qcos.at(joint_index), -&qsin.at(joint_index), Scalar::new(0.0)],
        [qsin.at(joint_index), qcos.at(joint_index), Scalar::new(0.0)],
        [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(1.0)]
    ]);

    let limi_rotation = (robot_info.calc_limi)(
        rotation_matrix,
        joint_index,
    );

    limi_rotations.push(limi_rotation.clone());
    let limi_translation = limi_translations[joint_index].clone();

    match joint_index {
        0 => {
            omi_rotations.push(limi_rotation.clone());
            omi_translations.push(limi_translation.clone());
            all_v.push(
                Vector::<6>::new([
                    Scalar::new(0.0),
                    Scalar::new(0.0),
                    Scalar::new(0.0),
                    Scalar::new(0.0),
                    Scalar::new(0.0),
                    all_joint_v.at(joint_index)
                ])
            );
        }
        _ => {
            // the multiplication between oMi and liMi is defined as:
            //{ return SE3Tpl(rot*m2.rotation()
            //    ,translation()+rotation()*m2.translation());}
            let omi_rotation_i = &omi_rotations[joint_index - 1] * &limi_rotation;
            let omi_translation_to_add = &omi_rotations[joint_index - 1] * &limi_translation;
            let omi_translation_i = &omi_translations[joint_index - 1] + &omi_translation_to_add;
            //oMis.push((omi_rotation_i.clone(), omi_translation_i.clone()));
            omi_rotations.push(omi_rotation_i);
            omi_translations.push(omi_translation_i);
            let temp_v = Vector::<6>::new([
                Scalar::new(0.0),
                Scalar::new(0.0),
                Scalar::new(0.0),
                Scalar::new(0.0),
                Scalar::new(0.0),
                all_joint_v.at(joint_index)
            ]);
            let new_v = act_motion_inv(
                limi_translation.clone(),
                limi_rotation.clone(),
                temp_v,
                all_v[joint_index - 1].clone(),
                joint_index,
            );
            all_v.push(new_v);
        }
    }

    // data.a[i]  = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
    let minus_m_w = -&all_joint_v.at(joint_index);
    let temp_a1 = alpha_cross(
        minus_m_w,
        all_v[joint_index].clone(),
        joint_index,
    );

    // jdata.S() * jmodel.jointVelocitySelector(a);
    let temp_a2 = &all_joint_a.at(joint_index) + &temp_a1.at(5);

    let temp_a3 = Vector::<6>::new([
        temp_a1.at(0),
        temp_a1.at(1),
        temp_a1.at(2),
        temp_a1.at(3),
        temp_a1.at(4),
        temp_a2,
    ]);

    match joint_index {
        0 => {
            all_a.push(temp_a3);
        }
        _ => {
            let add_a = act_inv(
                limi_translation,
                limi_rotation,
                all_a[joint_index - 1].clone(),
                joint_index,
            );
            let new_a = &temp_a3 + &add_a;
            all_a.push(new_a);
        }
    }

}


pub fn forward_kinematics<const N: usize> (
    qcos: Vector<N>,
    qsin: Vector<N>,
    v: Vector<N>,
    a: Vector<N>,
    robot_info: &RobotInfo<N>,
) -> () {
    let n_joints = robot_info.n_joints;
    let limi_translations = robot_info.limi_translations.clone();

    let mut all_v = Vec::<Vector<6>>::new();
    let mut all_a = Vec::<Vector<6>>::new();
    let mut limi_rotations = Vec::<Matrix<3, 3>>::new();
    let mut omi_translations = Vec::<Vector<3>>::new();
    let mut omi_rotations = Vec::<Matrix<3, 3>>::new();

    for i in 0..n_joints {
        forward_kinematics_helper(
            qcos.clone(),
            qsin.clone(),
            v.clone(),
            a.clone(),
            &mut all_v,
            &mut all_a,
            &limi_translations,
            &mut limi_rotations,
            &mut omi_translations,
            &mut omi_rotations,
            i,
            robot_info,
        );
    }

    for i in 0..N {
        omi_translations[i].clone().define(format!("final_omi_translations_{}", i).as_str());
        omi_rotations[i].clone().define(format!("final_omi_rotations_{}", i).as_str());
        all_v[i].clone().define(format!("final_v_{}", i).as_str());
        all_a[i].clone().define(format!("final_a_{}", i).as_str());
    }

    // setting one 
    set_function_output(all_v[0].at(0).clone());

}














