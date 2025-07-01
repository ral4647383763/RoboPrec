use std::ops::Mul;
use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::{my_print, my_println};

// scalar times scalar multiplication
impl Mul for BaseMatrix<1, 1> {
    type Output = BaseMatrix<1, 1>;
    fn mul(self, rhs: Self) -> BaseMatrix<1, 1> {
        let new_name = generate_name("scalar_mul");
        my_println!("val {}_0_0: Real = {}_0_0 * {}_0_0", new_name, self.get_name(), rhs.get_name());
        BaseMatrix {
            name: new_name,
            values: vec![vec![self.get_values()[0][0] * rhs.get_values()[0][0]]],
        }
    }
}

// scalar times sth multiplication
// implement it by .element_mul() in the matrix module
impl BaseMatrix<1, 1> {
    /// Scalar multiplication.
    pub fn element_mul<const A: usize, const B: usize>(self, other: &BaseMatrix<A, B>) -> BaseMatrix<A, B> {
        let new_name = generate_name("scalar_mul");
        let val1 = self.get_values();
        let val2 = other.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        for r in 0..A {
            new_values.push(Vec::new());
            for c in 0..B {
                my_print!("val {}_{}_{}: Real = ", new_name, r, c);
                my_print!(
                    "({}_0_0 * {}_{}_{})",
                    self.get_name(), other.get_name(), r, c
                );
                my_println!();

                new_values[r].push(
                    val1[0][0] * val2[r][c]
                );
            }
        }
        
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}

/// Matrix multiplication.
/// Multiply a Matrix<T, M, A> by a Matrix<T, A, B> to yield a Matrix<T, M, B>.
impl<const M: usize, const A: usize, const B: usize> Mul<&BaseMatrix<A, B>> for &BaseMatrix<M, A>
{
    type Output = BaseMatrix<M, B>;
    fn mul(self, rhs: &BaseMatrix<A, B>) -> BaseMatrix<M, B> {
        let new_name = generate_name("mul");
        let val1 = self.get_values();
        let val2 = rhs.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // The left matrix is M x A and the right matrix is A x B, yielding an M x B matrix.
        for r in 0..M {
            new_values.push(Vec::new());
            for c in 0..B {
                my_print!("val {}_{}_{}: Real = ", new_name, r, c);
                let mut first = true;
                for k in 0..A {
                    if !first {
                        my_print!(" + ");
                    }
                    my_print!(
                        "({}_{}_{} * {}_{}_{})",
                        self.get_name(), r, k,
                        rhs.get_name(), k, c
                    );
                    first = false;
                }
                my_println!();


                let mut sum = 0.0;
                for k in 0..A {
                    sum += val1[r][k] * val2[k][c];
                }
                new_values[r].push(sum);

            }
        }
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}
