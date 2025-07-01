use std::vec;

use crate::types::alias::{Scalar, Vector, Matrix};
use crate::logging::{add_function_input_vector, print_function, set_function_name, set_function_output};

fn func_to_generate(
    a: Vector<3>,
    b: Vector<3>
) -> Scalar {
    // This is the function that will be generated in Daisy DSL
    
    let mut acc = Scalar::new(0.0);

    // We can use loops and conditionals, too.
    for i in 0..3 {
        if i == 0 {
            acc = acc + a.dot(&b);
        }
        else if i == 1 {
            acc = acc + a.at(i) * b.at(i);
        }
        else {
            acc = acc - a.at(i) * b.at(i);
        }
    }

    acc
}


pub fn ex2(){
    // This will set the function name in Daisy DSL
    set_function_name("ex2");

    // The first parameter is the name of the variable in the generated code
    // The second parameter is the default value of the variable. It is not used in the generated code,
    // but it is used to carry out the calculations in the Rust code.
    // It is only to be used to debug the algorithm without generating new code or carrying out the analysis.
    let a = Vector::<3>::new_with_val("a", vec![vec![1.0], vec![2.0], vec![3.0]]);
    let b = Vector::<3>::new_with_val("b", vec![vec![4.0], vec![5.0], vec![6.0]]);
    add_function_input_vector(a.clone(), vec![(0.0, 10.0); 3]);
    add_function_input_vector(b.clone(), vec![(0.0, 10.0); 3]);

    let res = func_to_generate(a, b);

    set_function_output(res.clone());
    print_function();

    // And you can also check if your algorithm is working without generating new code or analysis:
    let expected = Scalar::new(24.0);
    assert_eq!(res.get_values(), expected.get_values());
}

