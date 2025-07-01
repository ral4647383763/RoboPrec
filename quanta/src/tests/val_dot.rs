use crate::types::alias::{Scalar, Vector};


#[test]
fn test_vector_vector_dot() {
    let a = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let b = Vector::<3>::new([Scalar::new(4.0), Scalar::new(5.0), Scalar::new(6.0)]);
    let result = a.dot(&b);

    let result_expected = Scalar::new(32.0); // 1*4 + 2*5 + 3*6
    assert_eq!(result.get_values(), result_expected.get_values());
}

#[test]
fn test_vector_vector_dot_6() {
    let a = Vector::<6>::new([
        Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0),
        Scalar::new(4.0), Scalar::new(5.0), Scalar::new(6.0)
    ]);
    let b = Vector::<6>::new([
        Scalar::new(7.0), Scalar::new(8.0), Scalar::new(9.0),
        Scalar::new(10.0), Scalar::new(11.0), Scalar::new(12.0)
    ]);
    let result = a.dot(&b);

    let result_expected = Scalar::new(217.0); // 1*7 + 2*8 + 3*9 + 4*10 + 5*11 + 6*12
    assert_eq!(result.get_values(), result_expected.get_values());
}