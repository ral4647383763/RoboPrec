use crate::types::alias::{Scalar, Vector, Matrix};


#[test]
fn test_scalar_scalar_div(){
    let a = Scalar::new(6.0);
    let b = Scalar::new(2.0);
    let result = a / b;

    let result_expected = Scalar::new(3.0);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
#[should_panic]
fn test_scalar_scalar_div_by_zero() {
    let a = Scalar::new(6.0);
    let b = Scalar::new(0.0);
    let _result = a / b;   
    let result_values = _result.get_values();
    println!("Result values: {:?}", result_values);
}

#[test]
fn test_scalar_scalar_div_fraction() {
    let a = Scalar::new(1.0);
    let b = Scalar::new(3.0);
    let result = a / b;

    let result_expected = Scalar::new(1.0 / 3.0);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_scalar_vector_div() {
    let a = Scalar::new(6.0);
    let b = Vector::<3>::new([Scalar::new(2.0), Scalar::new(3.0), Scalar::new(4.0)]);
    let result = a.element_div(&b);

    let result_expected = Vector::<3>::new([
        Scalar::new(3.0),
        Scalar::new(2.0),
        Scalar::new(1.5),
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
#[should_panic]
fn test_scalar_vector_div_by_zero() {
    let a = Scalar::new(6.0);
    let b = Vector::<3>::new([Scalar::new(2.0), Scalar::new(0.0), Scalar::new(4.0)]);
    let _result = a.element_div(&b);
    
    // This should panic due to division by zero
}

#[test]
fn test_vector_vector_div() {
    let a = Vector::<3>::new([Scalar::new(6.0), Scalar::new(9.0), Scalar::new(12.0)]);
    let b = Vector::<3>::new([Scalar::new(2.0), Scalar::new(3.0), Scalar::new(4.0)]);
    let result = a / b;

    let result_expected = Vector::<3>::new([
        Scalar::new(3.0),
        Scalar::new(3.0),
        Scalar::new(3.0),
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
#[should_panic]
fn test_vector_vector_div_by_zero() {
    let a = Vector::<3>::new([Scalar::new(6.0), Scalar::new(9.0), Scalar::new(12.0)]);
    let b = Vector::<3>::new([Scalar::new(2.0), Scalar::new(0.0), Scalar::new(4.0)]);
    let _result = a / b;

    // This should panic due to division by zero
}

#[test]
fn test_scalar_matrix_div() {
    let a = Scalar::new(8.0);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(2.0), Scalar::new(4.0)],
        [Scalar::new(8.0), Scalar::new(16.0)],
    ]);
    let result = a.element_div(&b);

    let result_expected = Matrix::<2, 2>::new([
        [Scalar::new(4.0), Scalar::new(2.0)],
        [Scalar::new(1.0), Scalar::new(0.5)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
#[should_panic]
fn test_scalar_matrix_div_by_zero() {
    let a = Scalar::new(8.0);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(2.0), Scalar::new(0.0)],
        [Scalar::new(8.0), Scalar::new(16.0)],
    ]);
    let _result = a.element_div(&b);

    // This should panic due to division by zero
}

#[test]
fn test_matrix_matrix_div() {
    let a = Matrix::<2, 2>::new([
        [Scalar::new(8.0), Scalar::new(12.0)],
        [Scalar::new(16.0), Scalar::new(24.0)],
    ]);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(2.0), Scalar::new(3.0)],
        [Scalar::new(4.0), Scalar::new(6.0)],
    ]);
    let result = a / b;

    let result_expected = Matrix::<2, 2>::new([
        [Scalar::new(4.0), Scalar::new(4.0)],
        [Scalar::new(4.0), Scalar::new(4.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
#[should_panic]
fn test_matrix_matrix_div_by_zero() {
    let a = Matrix::<2, 2>::new([
        [Scalar::new(8.0), Scalar::new(12.0)],
        [Scalar::new(16.0), Scalar::new(24.0)],
    ]);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(2.0), Scalar::new(0.0)],
        [Scalar::new(4.0), Scalar::new(6.0)],
    ]);
    let _result = a / b;

    // This should panic due to division by zero
}
