use std::vec;

use crate::types::alias::{Scalar, Vector, Matrix};
use crate::logging::{add_function_input_vector, print_function, set_function_name, set_function_output};

fn example(
    a: &Vector<3>,
    b: &Vector<3>,
    c: &Vector<3>
) -> Scalar {
    let res1 = a.dot(b);
    let res2 = b.dot(c);

    res1 + res2
}


pub fn ex3(){
    // This will set the function name in Daisy DSL
    set_function_name("example");

    // The first parameter is the name of the variable in the generated code
    // The second parameter is the default value of the variable. It is not used in the generated code,
    // but it is used to carry out the calculations in the Rust code.
    // It is only to be used to debug the algorithm without generating new code or carrying out the analysis.
    let a = Vector::<3>::new_with_val("a", vec![vec![1.0], vec![2.0], vec![3.0]]);
    let b = Vector::<3>::new_with_val("b", vec![vec![4.0], vec![5.0], vec![6.0]]);
    let c = Vector::<3>::new_with_val("c", vec![vec![7.0], vec![8.0], vec![9.0]]);
    add_function_input_vector(a.clone(), vec![(0.0, 10.0); 3]);
    add_function_input_vector(b.clone(), vec![(0.0, 10.0); 3]);
    add_function_input_vector(c.clone(), vec![(0.0, 10.0); 3]);

    let res = example(&a, &b, &c);

    set_function_output(res.clone());
    print_function();

    // And you can also check if your algorithm is working without generating new code or analysis:
    let expected = Scalar::new(154.0);
    assert_eq!(res.get_values(), expected.get_values());
}


// or
/*
pub fn ex3(){
    // This will set the function name in Daisy DSL
    set_function_name("example");

    let a = Vector::<3>::new("a");
    let b = Vector::<3>::new("b");
    add_function_input_vector(a.clone(), vec![(0.0, 10.0); 3]);
    add_function_input_vector(b.clone(), vec![(0.0, 10.0); 3]);

    let res = example(&a, &b);

    set_function_output(res.clone());
    print_function();

    // And you can also check if your algorithm is working without generating new code or analysis:
    let expected = Scalar::new(32.0);
    assert_eq!(res.get_values(), expected.get_values());
}
*/