use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::my_println;

/// ===================================================
/// Dot Product for 3D Vectors
/// ===================================================
/// 
impl BaseMatrix<3, 1> {
    /// Compute the dot product of two 3D vectors.
    ///
    /// Given two vectors A and B with entries:
    ///   A = [a, b, c]  and  B = [d, e, f],
    /// this prints DSL lines corresponding to:
    ///   C₀ = a * d + b * e + c * f.
    ///
    /// The returned vector (of type `BaseMatrix<1, 1>`) will have a new generated name.
    pub fn dot(&self, rhs: &Self) -> BaseMatrix<1, 1> {
        let new_name = generate_name("vector_dot");
        let val1 = self.get_values();
        let val2 = rhs.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // Print DSL for the dot product.
        my_println!(
            "val {}_0_0: Real = {}_0_0 * {}_0_0 + {}_1_0 * {}_1_0 + {}_2_0 * {}_2_0",
            new_name,
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
        );

        new_values.push(vec![
            val1[0][0] * val2[0][0] + val1[1][0] * val2[1][0] + val1[2][0] * val2[2][0],
        ]);

        BaseMatrix::<1, 1> {
            name: new_name,
            values: new_values,
        }
    }
}

// Dot product for 6D Vectors
/// ===================================================
/// Dot Product for 3D Vectors
/// ===================================================
/// 
impl BaseMatrix<6, 1> {
    /// Compute the dot product of two 3D vectors.
    ///
    /// Given two vectors A and B with entries:
    ///   A = [a, b, c]  and  B = [d, e, f],
    /// this prints DSL lines corresponding to:
    ///   C₀ = a * d + b * e + c * f.
    ///
    /// The returned vector (of type `BaseMatrix<1, 1>`) will have a new generated name.
    pub fn dot(&self, rhs: &Self) -> BaseMatrix<1, 1> {
        let new_name = generate_name("vector_dot");
        let val1 = self.get_values();
        let val2 = rhs.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // Print DSL for the dot product.
        my_println!(
            "val {}_0_0: Real = {}_0_0 * {}_0_0 + {}_1_0 * {}_1_0 + {}_2_0 * {}_2_0 + {}_3_0 * {}_3_0 + {}_4_0 * {}_4_0 + {}_5_0 * {}_5_0",
            new_name,
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
        );

        new_values.push(vec![
            val1[0][0] * val2[0][0] + val1[1][0] * val2[1][0] + val1[2][0] * val2[2][0] +
            val1[3][0] * val2[3][0] + val1[4][0] * val2[4][0] + val1[5][0] * val2[5][0],
        ]);
        BaseMatrix::<1, 1> {
            name: new_name,
            values: new_values,
        }

    }
}