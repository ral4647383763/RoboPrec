use crate::types::alias::{Scalar, Vector, Matrix};


#[test]
fn test_scalar_neg() {
    let a = Scalar::new(2.0);
    let result = -a;

    let result_expected = Scalar::new(-2.0);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_vector_neg() {
    let a = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let result = -&a;

    let result_expected = Vector::<3>::new([Scalar::new(-1.0), Scalar::new(-2.0), Scalar::new(-3.0)]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_matrix_neg() {
    let a = Matrix::<2, 2>::new([
        [Scalar::new(1.0), Scalar::new(2.0)],
        [Scalar::new(3.0), Scalar::new(4.0)],
    ]);
    let result = -&a;

    let result_expected = Matrix::<2, 2>::new([
        [Scalar::new(-1.0), Scalar::new(-2.0)],
        [Scalar::new(-3.0), Scalar::new(-4.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}