use crate::types::alias::{Scalar, Vector, Matrix};
use crate::robots::robot_info::RobotInfo;
use crate::logging::set_function_output;


fn act_inv(
    translation: Vector<3>,
    rotation: Matrix<3, 3>,
    linear: &Vector<3>,
    angular: &Vector<3>,
    linear_parent: Vector<3>,
    angular_parent: Vector<3>,
    joint_index: usize,
) -> (Vector<3>, Vector<3>) {
    let act_inv1 = translation.cross(&angular_parent).define(format!("act_inv_fun_1_{}", joint_index).as_str());
    let act_inv2 = (&linear_parent - &act_inv1).define(format!("act_inv_fun_2_{}", joint_index).as_str());
    let act_inv3 = rotation.transpose().define(format!("act_inv_fun_3_{}", joint_index).as_str());
    let act_inv4 = (&act_inv3 * &act_inv2).define(format!("act_inv_fun_4_{}", joint_index).as_str());
    let act_inv5 = (&act_inv3 * &angular_parent).define(format!("act_inv_fun_5_{}", joint_index).as_str());

    (linear + &act_inv4, angular + &act_inv5)
}

fn alpha_cross_linear(
    s: &Scalar,
    vin: &Vector<3>,
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
    vin: &Vector<3>,
    joint_index: usize
) -> Vector<3> {
    let alpha_cross1 = &(-s) * &vin.at(1);
    let alpha_cross2 = s * &vin.at(0);
    let alpha_cross = Vector::<3>::new([alpha_cross1, alpha_cross2, Scalar::new(0.0)]);
    let alpha_cross = alpha_cross.define(format!("alpha_cross_angular_{}", joint_index).as_str());
    alpha_cross
}

fn rhs_mult(
    inertia: Matrix<3, 3>,
    vin: &Vector<3>,
    joint_index: usize
) -> Vector<3> {
    let vout_0 = &inertia.at((0, 0)) * &vin.at(0) + &inertia.at((0, 1)) * &vin.at(1) + &inertia.at((0, 2)) * &vin.at(2);
    let vout_1 = &inertia.at((0, 1)) * &vin.at(0) + &inertia.at((1, 1)) * &vin.at(1) + &inertia.at((1, 2)) * &vin.at(2);
    let vout_2 = &inertia.at((0, 2)) * &vin.at(0) + &inertia.at((1, 2)) * &vin.at(1) + &inertia.at((2, 2)) * &vin.at(2);
    
    let vout = Vector::<3>::new([vout_0, vout_1, vout_2]).define(format!("rhs_mult_{}", joint_index).as_str());

    vout
}

fn act(
    rotation: Matrix<3, 3>,
    translation: Vector<3>,
    f: Vector<6>,
    joint_index: usize,
) -> Vector<6> {
    let f_linear = Vector::<3>::new([f.at(0), f.at(1), f.at(2)]);
    let f_angular = Vector::<3>::new([f.at(3), f.at(4), f.at(5)]);

    let new_f_linear = &rotation * &f_linear;
    let new_f_angular = &rotation * &f_angular;

    let f_angular_cross = translation.cross(&new_f_linear);

    let new_f_angular = new_f_angular + f_angular_cross;

    let new_f = Vector::<6>::new([
        new_f_linear.at(0),
        new_f_linear.at(1),
        new_f_linear.at(2),
        new_f_angular.at(0),
        new_f_angular.at(1),
        new_f_angular.at(2),
    ]).define(format!("act_{}", joint_index).as_str());

    new_f
}

fn first_pass<const N: usize>(
    qsin: Scalar,
    qcos: Scalar,
    data_v: &Vector<6>,
    v: &Vector<N>,
    a: &Vector<N>,
    parent_v: &Vector<6>,
    parent_a_gf: &Vector<6>,
    limi_translations: &Vec<Vector<3>>,
    mut limi_rotations: Vec<Matrix<3, 3>>,
    joint_index: usize,
    levers: &Vec<Vector<3>>,
    masses: &Vector<N>,
    inertias: &Vec<Matrix<3, 3>>,
    robot_info: &RobotInfo<N>,
) -> (Vec<Matrix<3, 3>>, Vector<6>, Vector<6>, Vector<6>, Vector<6>) {

    let rotation_matrix = Matrix::<3, 3>::new([
        [qcos.clone(), -&qsin, Scalar::new(0.0)],
        [qsin, qcos, Scalar::new(0.0)],
        [Scalar::new(0.0), Scalar::new(0.0), Scalar::new(1.0)],
    ]);

    let limi_rotation = (robot_info.calc_limi)(rotation_matrix, joint_index);
    limi_rotations.push(limi_rotation.clone());
    let limi_translation = limi_translations[joint_index].clone();

    let mut new_v_linear = Vector::<3>::new([
        data_v.at(0), data_v.at(1), data_v.at(2)
    ]);

    let mut new_v_angular = Vector::<3>::new([
        data_v.at(3), data_v.at(4), v.at(joint_index)
    ]);

    let parent_v_linear = Vector::<3>::new([
        parent_v.at(0), parent_v.at(1), parent_v.at(2),
    ]);
    let parent_v_angular = Vector::<3>::new([
        parent_v.at(3), parent_v.at(4), parent_v.at(5),
    ]);
    
    let parent_a_gf_linear = Vector::<3>::new([
        parent_a_gf.at(0), parent_a_gf.at(1), parent_a_gf.at(2),
    ]);
    let parent_a_gf_angular = Vector::<3>::new([
        parent_a_gf.at(3), parent_a_gf.at(4), parent_a_gf.at(5),
    ]);

    // if parent > 0
    if joint_index > 0 {
        //data.v[i] += data.liMi[i].actInv(data.v[parent]);
        (new_v_linear, new_v_angular) = act_inv(
            limi_translation.clone(),
            limi_rotation.clone(),
            &new_v_linear,
            &new_v_angular,
            parent_v_linear,
            parent_v_angular,
            joint_index,
        );
    }
    new_v_linear = new_v_linear.define(format!("new_v_linear_{}", joint_index).as_str());
    new_v_angular = new_v_angular.define(format!("new_v_angular_{}", joint_index).as_str());

    //data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
    // ^ operator is actually implemented in pinocchio/include/pinocchio/spatial/cartesian-axis.hpp inline void CartesianAxis<2>::alphaCross
    // vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
    let minus_m_w = -&(v.at(joint_index));
    let alpha_cross_linear = alpha_cross_linear(&minus_m_w, &new_v_linear, joint_index);
    let alpha_cross_angular = alpha_cross_angular(&minus_m_w, &new_v_angular, joint_index);

    let new_a_gf = Vector::<6>::new([
        alpha_cross_linear.at(0),
        alpha_cross_linear.at(1),
        alpha_cross_linear.at(2),
        alpha_cross_angular.at(0),
        alpha_cross_angular.at(1),
        alpha_cross_angular.at(2),
    ]).define(format!("new_a_gf_{}", joint_index).as_str());

    // data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a);
    // jointVelocitySelector(a) is only a[joint_id]
    // I couldn't print out info about jdata.S() easily but it is ConstraintRevoluteTpl, and I believe the only thing this line does is
    // data.a_gf[i][5] = jmodel.jointVelocitySelector(a)

    let new_a_gf_up1 = &a.at(joint_index) + &new_a_gf.at(5);
    
    let new_a_gf2_linear = Vector::<3>::new([
        new_a_gf.at(0), new_a_gf.at(1), new_a_gf.at(2)
    ]).define(format!("new_a_gf2_linear_{}", joint_index).as_str());
    let new_a_gf2_angular = Vector::<3>::new([
        new_a_gf.at(3), new_a_gf.at(4), new_a_gf_up1
    ]).define(format!("new_a_gf2_angular_{}", joint_index).as_str());

    let (new_a_gf_up2_linear, new_a_gf_up2_angular) = act_inv(
        limi_translation.clone(),
        limi_rotation.clone(),
        &new_a_gf2_linear,
        &new_a_gf2_angular,
        parent_a_gf_linear,
        parent_a_gf_angular,
        joint_index,
    );

    let new_a_gf_up3 = Vector::<6>::new([
        new_a_gf_up2_linear.at(0),
        new_a_gf_up2_linear.at(1),
        new_a_gf_up2_linear.at(2),
        new_a_gf_up2_angular.at(0),
        new_a_gf_up2_angular.at(1),
        new_a_gf_up2_angular.at(2),
    ]).define(format!("new_a_gf_up_final_{}", joint_index).as_str());

    // this line updates spatial momenta
    // model.inertias[i].__mult__(data.v[i],data.h[i]);
    //let data_h = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define(format!("data_h_{}", joint_id).as_str());
    // data.v[i] is new_v at this point
    // firstly mass * (v.linear - lever.cross(v.angular))
    let h_linear_1 = &levers[joint_index].cross(&new_v_angular).define(format!("h_linear_1_{}", joint_index).as_str());
    let h_linear_2 = &new_v_linear - &h_linear_1;
    let h_linear = &masses.at(joint_index).element_mul(&h_linear_2);

    // next line is Symmetric3::rhsMult(inertia(),v.angular(),f.angular());
    let h_angular = rhs_mult(inertias[joint_index].clone(), &new_v_angular, joint_index );

    // next line is f.angular() += lever().cross(f.linear());
    let h_angular_1 = levers[joint_index].cross(&h_linear);
    let h_angular_2 = &h_angular + &h_angular_1;

    // next line is model.inertias[i].__mult__(data.a_gf[i],data.f[i]);
    // firstly mass * (a_gf.linear - lever.cross(a_gf.angular))
    let f_linear_1 = levers[joint_index].cross(&new_a_gf_up2_angular);
    let f_linear_2 = &new_a_gf_up2_linear - &f_linear_1;
    let f_linear_3 = masses.at(joint_index).element_mul(&f_linear_2);

    // next line is Symmetric3::rhsMult(inertia(),a_gf.angular(),f.angular());
    let f_angular = rhs_mult(inertias[joint_index].clone(), &new_a_gf_up2_angular, joint_index);

    // next line is f.angular() += lever().cross(f.linear());
    let f_angular_1 = levers[joint_index].cross(&f_linear_3);
    let f_angular_2 = &f_angular + &f_angular_1;

    // the cross here is not the regular cross product since the vectors are 6D
    // it is implemented in pinocchio/include/pinocchio/spatial/motion-dense.hpp cross_impl, 
    // and it calls a motionAction, which is implemented in pinocchio/include/pinocchio/spatial/force-dense.hpp motionAction
    // final line is data.f[i] += data.v[i].cross(data.h[i]);
    /*
    void motionAction(const MotionDense<M1> & v, ForceDense<M2> & fout) const
    {
      std::cout << "ForceDense::motionAction" << std::endl;
      fout.linear().noalias() = v.angular().cross(linear());
      fout.angular().noalias() = v.angular().cross(angular())+v.linear().cross(linear());
    }
    */
    let f_linear_4_temp = &new_v_angular.cross(&h_linear);
    let f_linear_4 = &f_linear_3 + &f_linear_4_temp;

    let f_angular_3_temp = &new_v_angular.cross(&h_angular_2);
    let f_angular_3 = &f_angular_2 + &f_angular_3_temp;
    let f_angular_4_temp = &new_v_linear.cross(&h_linear);
    let f_angular_4 = &f_angular_3 + &f_angular_4_temp;

    let h = Vector::<6>::new([
        h_linear.at(0),
        h_linear.at(1),
        h_linear.at(2),
        h_angular_2.at(0),
        h_angular_2.at(1),
        h_angular_2.at(2),
    ]).define(format!("h_{}", joint_index).as_str());

    let f = Vector::<6>::new([
        f_linear_4.at(0),
        f_linear_4.at(1),
        f_linear_4.at(2),
        f_angular_4.at(0),
        f_angular_4.at(1),
        f_angular_4.at(2),
    ]).define(format!("f_{}", joint_index).as_str());

    let new_v = Vector::<6>::new([
        new_v_linear.at(0),
        new_v_linear.at(1),
        new_v_linear.at(2),
        new_v_angular.at(0),
        new_v_angular.at(1),
        new_v_angular.at(2),
    ]).define(format!("new_v_{}", joint_index).as_str());

    (
        limi_rotations,
        new_v,
        new_a_gf_up3,
        h,
        f,
    )

}


fn sec_pass<const N: usize>(
    mut all_f: Vec<Vector<6>>,
    limi_rotations: Vec<Matrix<3, 3>>,
    limi_translations: &Vec<Vector<3>>,
    n_joints: usize
) -> (Vec<Vector<6>>, Vector<N>) {
    // jmodel.jointVelocitySelector(data.tau) = jdata.S().transpose()*data.f[i];
    // I again couldn't print out info about jdata.S(), 
    // but at least for panda it works like this:
    // before data.tau:        0
    //     0
    //     0
    //     0
    // 0.381501
    // 0.471101
    // data.f[i]: 
    //      linear = -39.5124  42.6572  22.4058
    //      angular = 4.46655 1.28701  5.7779
    // 
    // after data.tau:        0
    //     0
    //     0
    // 5.7779
    // 0.381501
    // 0.471101
    // in each iteration, we set data.tau[joint_id] = data.f[i].angular[2];
    // the behaviour of S() ConstraintTpl was similar in forward pass

    let mut data_taus: Vec<Scalar> = vec![];

    for i in (0..n_joints).rev() {
        data_taus.push(all_f[i].at(5));

        //if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
        if i > 0 {
            let new_data_f_parent_add = act(limi_rotations[i].clone(), limi_translations[i].clone(), all_f[i].clone(), i);
            let new_data_f_parent = all_f[i - 1].clone() + new_data_f_parent_add;
            all_f[i - 1] = new_data_f_parent;
        }
    }

    //let data_tau = Vector!(
    //    [data_taus[5].clone(),
    //    data_taus[4].clone(),
    //    data_taus[3].clone(),
    //    data_taus[2].clone(),
    //    data_taus[1].clone(),
    //    data_taus[0].clone()]
    //).define("data_tau");

    let mut data_tau_elements: Vec<Scalar> = vec![];  
    for i in (0..n_joints).rev() {
        data_tau_elements.push(data_taus[i].clone());
    }
    /* // This version does not work
    let data_tau = Vector::<6>::new([
        data_tau_elements[0].clone(),
        data_tau_elements[1].clone(),
        data_tau_elements[2].clone(),
        data_tau_elements[3].clone(),
        data_tau_elements[4].clone(),
        data_tau_elements[5].clone()
    ]).define("final_tau");
    */

    let arr: [Scalar; N] =
        std::array::from_fn(|i| data_tau_elements[i].clone());

    let data_tau = Vector::<N>::new(arr)
                    .define("final_tau");

    (all_f, data_tau)  

}

pub fn rnea<const N: usize>(
    qcos: Vector<N>,
    qsin: Vector<N>,
    v: Vector<N>,
    a: Vector<N>,
    robot_info: &RobotInfo<N>
) -> () {
    let n_joints = robot_info.n_joints;
    let limi_translations = robot_info.limi_translations.clone();
    let levers = robot_info.levers.clone();
    let masses = robot_info.masses.clone();
    let inertias = robot_info.inertias.clone();

    let mut limi_rotations: Vec<Matrix<3, 3>> = vec![];

    // we also have data.v, which v[0] is set to zero explicitly, and I assume the rest should also be zero
    // I will keep it constant here, too, but keep in mind 
    let mut data_v: Vec<Vector<6>> = vec![];

    let mut all_v: Vec<Vector<6>> = vec![];
    let mut all_a_gf: Vec<Vector<6>> = vec![];
    let mut all_h: Vec<Vector<6>> = vec![];
    let mut all_f: Vec<Vector<6>> = vec![];

    let parent_v = Vector::<6>::new([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).define("parent_v");
    let parent_a_gf = Vector::<6>::new([0.0, 0.0, 9.81, 0.0, 0.0, 0.0]).define("parent_a_gf");

    let mut new_v: Vector::<6>;
    let mut new_a_gf: Vector::<6>;
    let mut new_h: Vector::<6>;
    let mut new_f: Vector::<6>;

    for i in 0..n_joints {
        data_v.push(Vector::<6>::new([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).define(format!("data_v_{}", i).as_str()));
        if i == 0 {
            (limi_rotations, new_v, new_a_gf, new_h, new_f) = first_pass(
                qsin.at(i).clone(),
                qcos.at(i).clone(),
                &data_v[i],
                &v,
                &a,
                &parent_v,
                &parent_a_gf,
                &limi_translations,
                limi_rotations,
                i,
                &levers,
                &masses,
                &inertias,
                &robot_info
            );
        }
        else{
            (limi_rotations, new_v, new_a_gf, new_h, new_f) = first_pass(
                qsin.at(i).clone(),
                qcos.at(i).clone(),
                &data_v[i],
                &v,
                &a,
                &all_v[i - 1],
                &all_a_gf[i - 1],
                &limi_translations,
                limi_rotations,
                i,
                &levers,
                &masses,
                &inertias,
                &robot_info
            );
        }

        all_v.push(new_v.clone());
        all_a_gf.push(new_a_gf.clone());
        all_h.push(new_h.clone());
        all_f.push(new_f.clone());
    }

    // sec_pass will do its own iteration
    let (_new_f, taus) = sec_pass::<N>(
        all_f,
        limi_rotations,
        &limi_translations,
        n_joints
    );

    // print return normally for now
    //my_println!("{}", format!("final_tau_{:?}_0", robot_info.n_joints - 1));
    set_function_output(taus.at(0).clone());

}



















