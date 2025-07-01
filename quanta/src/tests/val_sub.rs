use crate::types::alias::{Scalar, Vector, Matrix};


#[test]
fn test_scalar_scalar_sub() {
    let a = Scalar::new(3.0);
    let b = Scalar::new(1.0);
    let result = a - b;

    let result_expected = Scalar::new(2.0);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_scalar_vector_sub() {
    let a = Scalar::new(3.0);
    let b = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let result = a.element_sub(&b);

    let result_expected = Vector::<3>::new([Scalar::new(2.0), Scalar::new(1.0), Scalar::new(0.0)]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_vector_vector_sub() {
    let a = Vector::<3>::new([Scalar::new(4.0), Scalar::new(5.0), Scalar::new(6.0)]);
    let b = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let result = a - b;

    let result_expected = Vector::<3>::new([Scalar::new(3.0), Scalar::new(3.0), Scalar::new(3.0)]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_scalar_matrix_sub() {
    let a = Scalar::new(5.0);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(1.0), Scalar::new(2.0)],
        [Scalar::new(3.0), Scalar::new(4.0)],
    ]);
    let result = a.element_sub(&b);

    let result_expected = Matrix::<2, 2>::new([
        [Scalar::new(4.0), Scalar::new(3.0)],
        [Scalar::new(2.0), Scalar::new(1.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_matrix_matrix_sub() {
    let a = Matrix::<2, 2>::new([
        [Scalar::new(5.0), Scalar::new(6.0)],
        [Scalar::new(7.0), Scalar::new(8.0)],
    ]);
    let b = Matrix::<2, 2>::new([
        [Scalar::new(1.0), Scalar::new(2.0)],
        [Scalar::new(3.0), Scalar::new(4.0)],
    ]);
    let result = a - b;

    let result_expected = Matrix::<2, 2>::new([
        [Scalar::new(4.0), Scalar::new(4.0)],
        [Scalar::new(4.0), Scalar::new(4.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}


