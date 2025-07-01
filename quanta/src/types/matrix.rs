use crate::opr::helper::{generate_name, add_name, check_if_name_exists};
use crate::my_println;
use crate::logging::add_function_input_scalar;

/// ===================================
/// Base Matrix Type (for f64)
/// ===================================
///
/// The Matrix struct is now defined as Matrix<NROWS, NCOLS>,
/// meaning that a matrix has NROWS rows and NCOLS columns.  
/// The constructor expects a 2D array of shape [[f64; NCOLS]; NROWS]
/// or, via our extended DSL conversion, other types that print DSL lines.
#[derive(Debug, Clone)]
pub struct BaseMatrix<const NROWS: usize, const NCOLS: usize> {
    pub name: String,
    pub values: Vec<Vec<f64>>, // This is not for DSL printing, it supports executable code
}

impl<const NROWS: usize, const NCOLS: usize> BaseMatrix<NROWS, NCOLS> {
    /// Create a new matrix from any type that implements IntoMatrixSpecial.
    ///
    /// This method generates a new unique name (via generate_name), passes that name
    /// to the conversion trait to print the DSL lines, and returns a new matrix
    /// with that name for future use.
    pub fn new<I>(input: I) -> Self 
    where
         I: IntoMatrixSpecial<NROWS, NCOLS>,
    {
        let new_name = generate_name("");
        add_name(&new_name);
        let values = input.into_matrix_special(&new_name);
        Self { name: new_name, values }
    }

    /// This function assumes that the name is already checked and defined
    /// Create a new matrix when the DSL is already defined.
    pub fn new_with_name(name: String, value: Vec<Vec<f64>>) -> Self {
        Self { name, values: value }
    }

    // Another function so it is clear
    // Only to be used for inputs, as they should not be separately printed
    pub fn new_as_input(name: &str) -> Self {
        //println!("WARNING! This function is deprecated and will be removed in the future. Use `new_as_input_with_val` instead.");
        let new_name = if check_if_name_exists(name) {
            panic!("Input name already exists: {}", name);
        } else {
            name.to_string()
        };


        Self {
            name: new_name.to_string(),
            values: vec![vec![0.0; NCOLS]; NROWS],
        }
    }

    pub fn new_with_val(name: &str, value: Vec<Vec<f64>>) -> Self {
        let new_name = if check_if_name_exists(name) {
            panic!("Input name already exists: {}", name);
        } else {
            name.to_string()
        };

        if value.len() != NROWS || value[0].len() != NCOLS {
            panic!("Invalid dimensions for input values: expected {}x{}, got {}x{}",
                   NROWS, NCOLS, value.len(), value[0].len());
        }

        Self {
            name: new_name.to_string(),
            values: value,
        }
    }

    /// Rename this matrix.
    pub fn define(mut self, new_name: &str) -> Self {
        let name = if check_if_name_exists(new_name) {
            generate_name(new_name)
        } else{
            new_name.to_string()
        };
        add_name(&name);

        for r in 0..NROWS {
            for c in 0..NCOLS {
                my_println!("val {}_{}_{}: Real = {}_{}_{}", name, r, c, self.name, r, c);
            }
        }
        self.name = name.to_string();
        self
    }

    /// Get the matrix’s internal name.
    pub fn get_name(&self) -> &str {
        &self.name
    }
}

/// -----------------------------------------------------------------
/// DSL Conversion Trait for new()
/// -----------------------------------------------------------------
///
/// Instead of only converting into a numeric 2D array, this trait
/// prints DSL lines defining the matrix given the new_name provided.
/// Implementations must cover each input case.
pub trait IntoMatrixSpecial<const NROWS: usize, const NCOLS: usize> {
    /// Given a new name, print DSL lines that define the matrix.
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>>;
}

/// -----------------------------------------------------------------
/// Implementation for a plain 2D array of f64
/// -----------------------------------------------------------------
impl<const NROWS: usize, const NCOLS: usize> IntoMatrixSpecial<NROWS, NCOLS>
    for [[f64; NCOLS]; NROWS]
{
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>> {
        let mut values = Vec::with_capacity(NROWS);
        for r in 0..NROWS {
            let mut row = Vec::with_capacity(NCOLS);
            for c in 0..NCOLS {
                my_println!("val {}_{}_{}: Real = {}", new_name, r, c, self[r][c]);
                row.push(self[r][c]);
            }
            values.push(row);
        }
        values
    }
}

impl IntoMatrixSpecial<1, 1> for f64 {
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>> {
        // Here I will have a small workaround for a Daisy bug.
        // When I have a number that is too close to a whole number,
        // like 0.999999, 16-bit floats behave weird.
        // So, if the number is epsilon away from a whole number, I will
        // print it as a whole number.
        // This is not a problem for 32-bit floats, but I will do it anyway.

        let epsilon = 1e-4;
        let mut value = self;
        if (value - value.round()).abs() < epsilon {
            value = value.round();
        }
        // Now I will print the value
        my_println!("val {}_0_0: Real = {}", new_name, value);
        vec![vec![value]]
    }
}

/// -----------------------------------------------------------------
/// Implementation for an array of DSL scalars
/// -----------------------------------------------------------------
///
/// This covers the case where we want to build, say, a vector (an N×1 matrix)
/// from an array of scalars, i.e. `[BaseMatrix<1,1>; N]`. In that case, for each row
/// r the DSL will reference the DSL definition of that scalar.
/// For each element (r, 0) of the new matrix, it prints:
///
///     val {new_name}_{r}_0: Real = {scalar_name}_0_0
///
/// where {scalar_name} is obtained via the scalar’s get_name() method.
impl<const N: usize> IntoMatrixSpecial<N, 1> for [BaseMatrix<1, 1>; N] {
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>> {
        let mut values = Vec::with_capacity(N);
        for r in 0..N {
            my_println!("val {}_{}_0: Real = {}_0_0", new_name, r, self[r].get_name());
            values.push(vec![self[r].values[0][0]]);
        }
        values
    }
}

/// -----------------------------------------------------------------
/// Implementation for a 2D array of DSL scalars (BaseMatrix<1,1>)
/// -----------------------------------------------------------------
///
/// This covers the case where you build a general matrix (e.g. 3×3) from
/// a literal two-dimensional array whose elements are DSL scalars.
/// For each element at (r, c), we print a DSL line that references the DSL name
/// of that scalar (using the convention that each scalar’s definition is like:
///   val {scalar_name}_0_0: Real = ...).
impl<const NROWS: usize, const NCOLS: usize> IntoMatrixSpecial<NROWS, NCOLS>
    for [[BaseMatrix<1, 1>; NCOLS]; NROWS]
{
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>> {
        let mut values = Vec::with_capacity(NROWS);
        for r in 0..NROWS {
            let mut row = Vec::with_capacity(NCOLS);
            for c in 0..NCOLS {
                my_println!(
                    "val {}_{}_{}: Real = {}_0_0",
                    new_name,
                    r,
                    c,
                    self[r][c].get_name()
                );
                row.push(self[r][c].values[0][0]);
            }
            values.push(row);
        }
        values
    }
}

impl<const N: usize> IntoMatrixSpecial<N, 1> for [f64; N] {
    fn into_matrix_special(self, new_name: &str) -> Vec<Vec<f64>> {
        let mut values = Vec::with_capacity(N);
        for r in 0..N {
            my_println!("val {}_{}_0: Real = {}", new_name, r, self[r]);
            values.push(vec![self[r]]);
        }
        values
    }
}

/// -----------------------------------------------------------------
/// A helper trait for extracting a (row, column) pair from an index argument.
///
/// For vectors (of type BaseMatrix<_,1>), a single usize is interpreted as (index, 0).
/// For matrices, a tuple (usize, usize) is used directly.
pub trait IndexInfo {
    fn index_tuple(&self) -> (usize, usize);
}

impl IndexInfo for usize {
    fn index_tuple(&self) -> (usize, usize) {
        // When indexing a vector, the given index is the row (and column is 0)
        (*self, 0)
    }
}

impl IndexInfo for (usize, usize) {
    fn index_tuple(&self) -> (usize, usize) {
        *self
    }
}

impl<const NROWS: usize, const NCOLS: usize> BaseMatrix<NROWS, NCOLS> {
    /// Returns a scalar (BaseMatrix<1, 1>) corresponding to the element at the given index.
    ///
    /// For vectors, use a single index (usize) e.g.:
    ///     my_vector.at(2)
    ///
    /// For matrices, use a tuple of indices (usize, usize) e.g.:
    ///     my_matrix.at((1, 2))
    ///
    /// This function prints a DSL line such as:
    ///    val {new_name}: Real = {self_name}_{row}_{col}
    /// and returns a new scalar with that generated name.
    /// I need to rewrite this at some point...
    pub fn at<I: IndexInfo>(&self, idx: I) -> BaseMatrix<1, 1> {
        if NROWS == 1 {
            let (row, _) = idx.index_tuple();
            let new_name = generate_name("scalar");
            my_println!("val {}_0_0: Real = {}_{}_{}", new_name, self.get_name(), 0, row);
            BaseMatrix::<1, 1>::new_with_name(new_name, vec![vec![self.values[0][row]]])    
        }
        else if NCOLS == 1 {
            let (row, _) = idx.index_tuple();
            let new_name = generate_name("scalar");
            my_println!("val {}_0_0: Real = {}_{}_{}", new_name, self.get_name(), row, 0);
            BaseMatrix::<1, 1>::new_with_name(new_name, vec![vec![self.values[row][0]]])
        }
        else {
            let (row, col) = idx.index_tuple();
            let new_name = generate_name("scalar");
            my_println!("val {}_0_0: Real = {}_{}_{}", new_name, self.get_name(), row, col);
            BaseMatrix::<1, 1>::new_with_name(new_name, vec![vec![self.values[row][col]]])
        }
    }


    // implement col function so to extract a column
    pub fn col(&self, col: usize) -> BaseMatrix<NROWS, 1> {
        let new_name = generate_name("col");
        for r in 0..NROWS {
            my_println!("val {}_{}_0: Real = {}_{}_{}", new_name, r, self.get_name(), r, col);
        }
        BaseMatrix::<NROWS, 1>::new_with_name(new_name, 
            (0..NROWS).map(|r| vec![self.values[r][col]]).collect()
        )
    }

    pub fn get_values(&self) -> &Vec<Vec<f64>> {
        &self.values
    }

    pub fn set_values(&mut self, values: Vec<Vec<f64>>) {
        if values.len() != NROWS || values[0].len() != NCOLS {
            panic!("Invalid dimensions for setting values: expected {}x{}, got {}x{}",
                   NROWS, NCOLS, values.len(), values[0].len());
        }
        self.values = values;
    }

}
