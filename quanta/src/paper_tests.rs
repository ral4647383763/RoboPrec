

use crate::robots::roarm_m2::{roarm_m2, roarm_m2_get_bounds};
use crate::robots::roarm_m3::{roarm_m3, roarm_m3_get_bounds};
use crate::robots::indy7::{indy7, ind7_get_bounds};
use crate::robots::panda::{panda, panda_get_bounds};
use crate::robots::robot_info::RobotInfo;

use crate::examples::forward_kinematics;
use crate::examples::rnea;
use crate::examples::rnea_derivatives;
use crate::logging::{add_function_input_vector, print_function, set_function_name};
use crate::types::alias::Vector;

use clap::{Parser, ValueEnum};


#[derive(Parser, Debug, Clone, ValueEnum)]
enum Algorithm {
    Rnea,
    RneaDerivatives,
    ForwardKinematics
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of degrees of freedom
    #[arg(short, long)]
    dof: usize,

    /// Algorithm to run
    #[arg(short, long)]
    algorithm: Algorithm,
}


// This is a temporary solution to avoid code duplication.
enum RobotSetup {
    Dof7 {
        info: RobotInfo<7>,
        qsin: Vector<7>,
        qcos: Vector<7>,
        v: Vector<7>,
        a: Vector<7>,
    },
    Dof6 {
        info: RobotInfo<6>,
        qsin: Vector<6>,
        qcos: Vector<6>,
        v: Vector<6>,
        a: Vector<6>,
    },
    Dof5 {
        info: RobotInfo<5>,
        qsin: Vector<5>,
        qcos: Vector<5>,
        v: Vector<5>,
        a: Vector<5>,
    },
    Dof4 {
        info: RobotInfo<4>,
        qsin: Vector<4>,
        qcos: Vector<4>,
        v: Vector<4>,
        a: Vector<4>,
    }
}


pub fn paper_tests() {

    let args = Args::parse();
    
    let dof = args.dof;
    let algorithm = args.algorithm;

    let minmax_sincos = match dof {
        7 => panda_get_bounds(),
        6 => ind7_get_bounds(),
        5 => roarm_m3_get_bounds(),
        4 => roarm_m2_get_bounds(),
        _ => panic!("Unsupported number of degrees of freedom")
    };
    //print_init(&algorithm, dof, &ranges, &minmax_sincos);
    
    let setup = match dof {
        7 => RobotSetup::Dof7 {
            info: panda(),
            qsin: Vector::<7>::new_as_input("qsin"),
            qcos: Vector::<7>::new_as_input("qcos"),
            v: Vector::<7>::new_as_input("v"),
            a: Vector::<7>::new_as_input("a")
        },
        6 => RobotSetup::Dof6 {
            info: indy7(),
            qsin: Vector::<6>::new_as_input("qsin"),
            qcos: Vector::<6>::new_as_input("qcos"),
            v: Vector::<6>::new_as_input("v"),
            a: Vector::<6>::new_as_input("a")
        },
        5 => RobotSetup::Dof5 {
            info: roarm_m3(),
            qsin: Vector::<5>::new_as_input("qsin"),
            qcos: Vector::<5>::new_as_input("qcos"),
            v: Vector::<5>::new_as_input("v"),
            a: Vector::<5>::new_as_input("a")
        },
        4 => RobotSetup::Dof4 {
            info: roarm_m2(),
            qsin: Vector::<4>::new_as_input("qsin"),
            qcos: Vector::<4>::new_as_input("qcos"),
            v: Vector::<4>::new_as_input("v"),
            a: Vector::<4>::new_as_input("a")
        },
        _ => panic!("Unsupported number of degrees of freedom")
    };

    // That's not very good code, but it works for now.
    match setup {
        RobotSetup::Dof7 { info: _, ref qsin, ref qcos, ref v, ref a } => {
            add_function_input_vector(qsin.clone(), minmax_sincos.iter().map(|x| x.0).collect());
            add_function_input_vector(qcos.clone(), minmax_sincos.iter().map(|x| x.1).collect());
            add_function_input_vector(v.clone(), vec![(-0.5, 0.5); 7]);
            add_function_input_vector(a.clone(), vec![(-1.0, 1.0); 7]);
        }
        RobotSetup::Dof6 { info: _, ref qsin, ref qcos, ref v, ref a } => {
            add_function_input_vector(qsin.clone(), minmax_sincos.iter().map(|x| x.0).collect());
            add_function_input_vector(qcos.clone(), minmax_sincos.iter().map(|x| x.1).collect());
            add_function_input_vector(v.clone(), vec![(-0.5, 0.5); 6]);
            add_function_input_vector(a.clone(), vec![(-1.0, 1.0); 6]);
        }
        RobotSetup::Dof5 { info: _, ref qsin, ref qcos, ref v, ref a } => {
            add_function_input_vector(qsin.clone(), minmax_sincos.iter().map(|x| x.0).collect());
            add_function_input_vector(qcos.clone(), minmax_sincos.iter().map(|x| x.1).collect());
            add_function_input_vector(v.clone(), vec![(-0.5, 0.5); 5]);
            add_function_input_vector(a.clone(), vec![(-1.0, 1.0); 5]);
        }
        RobotSetup::Dof4 { info: _, ref qsin, ref qcos, ref v, ref a } => {
            add_function_input_vector(qsin.clone(), minmax_sincos.iter().map(|x| x.0).collect());
            add_function_input_vector(qcos.clone(), minmax_sincos.iter().map(|x| x.1).collect());
            add_function_input_vector(v.clone(), vec![(-0.5, 0.5); 4]);
            add_function_input_vector(a.clone(), vec![(-1.0, 1.0); 4]);
        }
    };

    // Next, match on the algorithm.
    match algorithm {
        // For example, for ForwardKinematics, we delegate to the corresponding function.
        Algorithm::ForwardKinematics => {
            set_function_name("ForwardKinematics");
            match setup {
                RobotSetup::Dof7 { info, qsin, qcos, v, a } => {
                    forward_kinematics::forward_kinematics(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof6 { info, qsin, qcos, v, a } => {
                    forward_kinematics::forward_kinematics(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof5 { info, qsin, qcos, v, a } => {
                    forward_kinematics::forward_kinematics(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof4 { info, qsin, qcos, v, a } => {
                    forward_kinematics::forward_kinematics(qcos, qsin, v, a, &info)
                }
            }
        }
        Algorithm::Rnea => {
            set_function_name("Rnea");
            match setup {
                RobotSetup::Dof7 { info, qsin, qcos, v, a } => {
                    rnea::rnea(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof6 { info, qsin, qcos, v, a } => {
                    rnea::rnea(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof5 { info, qsin, qcos, v, a } => {
                    rnea::rnea(qcos, qsin, v, a, &info)
                }
                RobotSetup::Dof4 { info, qsin, qcos, v, a } => {
                    rnea::rnea(qcos, qsin, v, a, &info)
                }
            }
        }
        Algorithm::RneaDerivatives => {
            set_function_name("RneaDerivatives");
            match setup {
                RobotSetup::Dof7 { info, qsin, qcos, v, a } => {
                    rnea_derivatives::rneaderivatives(qcos, qsin, v, a, info)
                }
                RobotSetup::Dof6 { info, qsin, qcos, v, a } => {
                    rnea_derivatives::rneaderivatives(qcos, qsin, v, a, info)
                }
                RobotSetup::Dof5 { info, qsin, qcos, v, a } => {
                    rnea_derivatives::rneaderivatives(qcos, qsin, v, a, info)
                }
                RobotSetup::Dof4 { info, qsin, qcos, v, a } => {
                    rnea_derivatives::rneaderivatives(qcos, qsin, v, a, info)
                }
            }
        }
    }
    


    print_function();





}


















