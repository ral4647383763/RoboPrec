use super::matrix::BaseMatrix;


pub type Scalar = BaseMatrix<1, 1>;
pub type Vector<const N: usize> = BaseMatrix<N, 1>;
pub type Matrix<const NROWS: usize, const NCOLS: usize> = BaseMatrix<NROWS, NCOLS>;
