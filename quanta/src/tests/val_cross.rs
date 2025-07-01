use crate::types::alias::{Scalar, Vector};


#[test]
fn test_cross(){
    let a = Vector::<3>::new([Scalar::new(1.0), Scalar::new(2.0), Scalar::new(3.0)]);
    let b = Vector::<3>::new([Scalar::new(4.0), Scalar::new(5.0), Scalar::new(6.0)]);
    let result = a.cross(&b);

    let result_expected = Vector::<3>::new([
        Scalar::new(-3.0),
        Scalar::new(6.0),
        Scalar::new(-3.0),
    ]);
    
    assert_eq!(result.get_values(), result_expected.get_values());
}