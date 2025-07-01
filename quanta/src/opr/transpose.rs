use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::my_println;

impl<const NROWS: usize, const NCOLS: usize> BaseMatrix<NROWS, NCOLS> {
    /// Transpose the matrix.
    ///
    /// This method returns a new matrix with dimensions reversed.
    /// For each element in the original matrix at position (r, c),
    /// the transposed matrix will have that element at position (c, r).
    pub fn transpose(&self) -> BaseMatrix<NCOLS, NROWS> {
        let new_name = generate_name("transpose");
        let val1 = self.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // Iterate over the transposed dimensions:
        // The new row index is the original column index.
        // The new column index is the original row index.
        for r in 0..NCOLS {
            new_values.push(Vec::new());
            for c in 0..NROWS {
                my_println!(
                    "val {}_{}_{}: Real = {}_{}_{}",
                    new_name,
                    r,
                    c,
                    self.get_name(),
                    c,
                    r
                );

                new_values[r].push(
                    val1[c][r] // Access the original matrix at (c, r)
                );
            }
        }
        BaseMatrix {
            name: new_name,
            values: new_values,
        }
    }
}