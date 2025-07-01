use std::ops::Neg;
use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::my_println;


impl<const NROWS: usize, const NCOLS: usize> Neg for &BaseMatrix<NROWS, NCOLS> {
    type Output = BaseMatrix<NROWS, NCOLS>;

    fn neg(self) -> Self::Output {
        let new_name = generate_name("matrix_negate");
        let val1 = self.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // In a complete implementation, you would actually multiply each entry by -1.
        // Here, we simply print the negation operation for each value.
        for r in 0..NROWS {
            new_values.push(Vec::new());
            for c in 0..NCOLS {
                my_println!(
                    "val {}_{}_{}: Real = -({}_{}_{})",
                    new_name, r, c, self.get_name(), r, c
                );

                new_values[r].push(
                    -val1[r][c] // Negate each element
                );
            }
        }
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}

impl<const NROWS: usize, const NCOLS: usize> Neg for BaseMatrix<NROWS, NCOLS> {
    type Output = BaseMatrix<NROWS, NCOLS>;

    fn neg(self) -> Self::Output {
        let new_name = generate_name("matrix_negate");
        let val1 = self.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // In a complete implementation, you would actually multiply each entry by -1.
        // Here, we simply print the negation operation for each value.
        for r in 0..NROWS {
            new_values.push(Vec::new());
            for c in 0..NCOLS {
                my_println!(
                    "val {}_{}_{}: Real = -({}_{}_{})",
                    new_name, r, c, self.get_name(), r, c
                );

                new_values[r].push(
                    -val1[r][c] // Negate each element
                );
            }
        }
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}