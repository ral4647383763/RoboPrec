use std::ops::Sub;
use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::{my_print, my_println};


/// Elementwise subtraction for borrowed matrices.
impl<const NROWS: usize, const NCOLS: usize> Sub for &BaseMatrix<NROWS, NCOLS>
{
    type Output = BaseMatrix<NROWS, NCOLS>;
    fn sub(self, rhs: Self) -> BaseMatrix<NROWS, NCOLS> {
        let new_name = generate_name("sub");
        let val1 = self.get_values();
        let val2 = rhs.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        for r in 0..NROWS {
            new_values.push(Vec::new());
            for c in 0..NCOLS {
                my_println!(
                    "val {}_{}_{}: Real = {}_{}_{} - {}_{}_{}",
                    new_name,
                    r,
                    c,
                    self.get_name(), r, c,
                    rhs.get_name(), r, c
                );

                new_values[r].push(
                    val1[r][c] - val2[r][c]
                );
            }
        }
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}

/// Elementwise subtraction for matrices.
impl<const NROWS: usize, const NCOLS: usize> Sub for BaseMatrix<NROWS, NCOLS>
{
    type Output = BaseMatrix<NROWS, NCOLS>;
    fn sub(self, rhs: Self) -> BaseMatrix<NROWS, NCOLS> {
        &self - &rhs // use the borrowed version
    }
}

/// Scalar - sth
impl BaseMatrix<1, 1> {
    /// Scalar subtraction.
    pub fn element_sub<const A: usize, const B: usize>(self, other: &BaseMatrix<A, B>) -> BaseMatrix<A, B> {
        let new_name = generate_name("scalar_sub");
        let val1 = self.get_values();
        let val2 = other.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        for r in 0..A {
            new_values.push(Vec::new());
            for c in 0..B {
                my_print!("val {}_{}_{}: Real = ", new_name, r, c);
                my_print!(
                    "({}_0_0 - {}_{}_{})",
                    self.get_name(), other.get_name(), r, c
                );
                my_println!();

                new_values[r].push(
                    val1[0][0] - val2[r][c]
                );
            }
        }
        
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}