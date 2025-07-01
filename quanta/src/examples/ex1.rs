use crate::types::alias::Scalar;
use crate::logging::{add_function_input_scalar, print_function, set_function_name, set_function_output};


pub fn ex1() {
    // This will set the function name in Daisy DSL
    set_function_name("ex1");

    // The first parameter is the name of the variable in the generated code
    // The second parameter is the default value of the variable. It is not used in the generated code,
    // but it is used to carry out the calculations in the Rust code.
    // It is only to be used to debug the algorithm without generating new code or carrying out the analysis.
    let a = Scalar::new_with_val("a", vec![vec![1.0]]);
    let b = Scalar::new_with_val("b", vec![vec![2.0]]);
    add_function_input_scalar(a.clone(), (0.0, 10.0));
    add_function_input_scalar(b.clone(), (0.0, 10.0));

    let result = a + b;

    set_function_output(result.clone());

    print_function();

    // And you can also check if your algorithm is working:
    let expected = Scalar::new(3.0);
    assert_eq!(result.get_values(), expected.get_values());
}