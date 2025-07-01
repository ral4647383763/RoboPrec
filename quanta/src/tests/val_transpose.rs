use crate::types::alias::{Scalar, Vector, Matrix};


#[test]
fn test_vector_transpose() {
    let a = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let result = a.transpose();

    let result_expected = Matrix::<1, 3>::new([
        [Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_matrix_transpose() {
    let a = Matrix::<2, 3>::new([
        [Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)],
        [Scalar::new(4.0), Scalar::new(5.0), Scalar::new(6.0)],
    ]);
    let result = a.transpose();

    let result_expected = Matrix::<3, 2>::new([
        [Scalar::new(1.0), Scalar::new(4.0)],
        [Scalar::new(2.0), Scalar::new(5.0)],
        [Scalar::new(3.0), Scalar::new(6.0)],
    ]);
    assert_eq!(result.get_values(), result_expected.get_values());
}