use crate::types::matrix::BaseMatrix;
use crate::opr::helper::generate_name;
use crate::my_println;

/// ===================================================
/// Cross Product for 3D Vectors
/// ===================================================
///
impl BaseMatrix<3, 1> {
    /// Compute the cross product of two 3D vectors.
    ///
    /// Given two vectors A and B with entries:
    ///   A = [a, b, c]  and  B = [d, e, f],
    /// this prints DSL lines corresponding to:
    ///   C₀ = b * f - c * e,
    ///   C₁ = c * d - a * f,
    ///   C₂ = a * e - b * d.
    ///
    /// The returned vector (of type `BaseMatrix<3, 1>`) will have a new generated name.
    pub fn cross(&self, rhs: &Self) -> Self {
        let new_name = generate_name("vector_cross");
        let val1 = self.get_values();
        let val2 = rhs.get_values();
        let mut new_values: Vec<Vec<f64>> = Vec::new();

        // Print DSL for each component of the cross product.
        my_println!(
            "val {}_0_0: Real = {}_1_0 * {}_2_0 - {}_2_0 * {}_1_0",
            new_name,
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
        );
        my_println!(
            "val {}_1_0: Real = {}_2_0 * {}_0_0 - {}_0_0 * {}_2_0",
            new_name,
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
        );
        my_println!(
            "val {}_2_0: Real = {}_0_0 * {}_1_0 - {}_1_0 * {}_0_0",
            new_name,
            self.get_name(), rhs.get_name(),
            self.get_name(), rhs.get_name(),
        );

        new_values.push(vec![
            val1[1][0] * val2[2][0] - val1[2][0] * val2[1][0],
        ]);
        new_values.push(vec![
            val1[2][0] * val2[0][0] - val1[0][0] * val2[2][0],
        ]);
        new_values.push(vec![
            val1[0][0] * val2[1][0] - val1[1][0] * val2[0][0],
        ]);

        Self { name: new_name, values: new_values}
    }
}


