# Permitted Operations in Quanta Framework

This document outlines the mathematical operations available, primarily defined in the `quanta/src/opr/` directory. You can see more comprehensive examples in `src/examples`.

## Arithmetic Operations

### Addition (`+` or `element_add`)
- **Scalar + Scalar**: `Scalar::new(a) + Scalar::new(b)`
  - Uses standard `Add` trait. See `quanta/src/opr/add.rs`.
- **Vector + Vector**: `Vector::<N>::new(...) + Vector::<N>::new(...)`
  - Uses standard `Add` trait. See `quanta/src/opr/add.rs`.
- **Matrix + Matrix**: `Matrix::<R,C>::new(...) + Matrix::<R,C>::new(...)`
  - Uses standard `Add` trait. See `quanta/src/opr/add.rs`.
- **Scalar + Vector** (element-wise): `scalar_value.element_add(&vector_value)`
  - Provided by `BaseMatrix<1, 1>::element_add` in `quanta/src/opr/add.rs`.
- **Scalar + Matrix** (element-wise): `scalar_value.element_add(&matrix_value)`
  - Provided by `BaseMatrix<1, 1>::element_add` in `quanta/src/opr/add.rs`.

### Subtraction (`-` or `element_sub`)
- **Scalar - Scalar**: `Scalar::new(a) - Scalar::new(b)`
  - Uses standard `Sub` trait. See `quanta/src/opr/sub.rs`.
- **Vector - Vector**: `Vector::<N>::new(...) - Vector::<N>::new(...)`
  - Uses standard `Sub` trait. See `quanta/src/opr/sub.rs`.
- **Matrix - Matrix**: `Matrix::<R,C>::new(...) - Matrix::<R,C>::new(...)`
  - Uses standard `Sub` trait. See `quanta/src/opr/sub.rs`.
- **Scalar - Vector** (element-wise): `scalar_value.element_sub(&vector_value)`
  - Provided by `BaseMatrix<1, 1>::element_sub` in `quanta/src/opr/sub.rs`.
- **Scalar - Matrix** (element-wise): `scalar_value.element_sub(&matrix_value)`
  - Provided by `BaseMatrix<1, 1>::element_sub` in `quanta/src/opr/sub.rs`.

### Multiplication (`*` or `element_mul`)
- **Scalar * Scalar**: `Scalar::new(a) * Scalar::new(b)`
  - Implemented by `Mul` trait for `BaseMatrix<1,1>` in `quanta/src/opr/mul.rs`.
- **Matrix * Matrix** (standard matrix multiplication): `Matrix::<A,B>::new(...) * Matrix::<B,C>::new(...)`
  - Implemented by `Mul` trait for `&BaseMatrix<A,B>` and `&BaseMatrix<B,C>` in `quanta/src/opr/mul.rs`.
- **Matrix * Vector** (standard matrix-vector multiplication): `Matrix::<R,C>::new(...) * Vector::<C>::new(...)`
  - This is a case of Matrix * Matrix where the second matrix is C×1.
- **Scalar * Vector** (element-wise): `scalar_value.element_mul(&vector_value)` or `scalar_value * vector_value`
  - `element_mul` provided by `BaseMatrix<1, 1>::element_mul` in `quanta/src/opr/mul.rs`. Operator `*` may also be overloaded.
- **Scalar * Matrix** (element-wise): `scalar_value.element_mul(&matrix_value)` or `scalar_value * matrix_value`
  - `element_mul` provided by `BaseMatrix<1, 1>::element_mul` in `quanta/src/opr/mul.rs`. Operator `*` may also be overloaded.

### Division (`/` or `element_div`)
- **Scalar / Scalar**: `Scalar::new(a) / Scalar::new(b)`
  - See `quanta/src/opr/div.rs`.
- **Scalar / Vector** (element-wise): `scalar_value.element_div(&vector_value)` (assuming `element_div` method)
  - See `quanta/src/opr/div.rs`.
- **Vector / Vector**: `vector1.element_div(&vector2)` (assuming `element_div` method)
  - See `quanta/src/opr/div.rs`.
- **Scalar / Matrix** (element-wise): `scalar_value.element_div(&matrix_value)` (assuming `element_div` method)
  - See `quanta/src/opr/div.rs`.
- **Matrix / Matrix**: `matrix1.element_div(&matrix2)` (assuming `element_div` method)
  - See `quanta/src/opr/div.rs`.
- *Note: Division by zero behavior is critical and should be handled (e.g., panic).*

### Negation (`-`)
- **-Scalar**: `-Scalar::new(a)`
  - See `quanta/src/opr/neg.rs`.
- **-Vector**: `-Vector::<N>::new(...)`
  - See `quanta/src/opr/neg.rs`.
- **-Matrix**: `-Matrix::<R,C>::new(...)`
  - See `quanta/src/opr/neg.rs`.

## Vector Operations

### Dot Product (`dot`)
- **Vector . Vector**: `vector1.dot(&vector2)`
  - Implemented in `quanta/src/opr/dot.rs`.

### Cross Product (`cross`)
- **3D Vector x 3D Vector**: `vector_3d_a.cross(&vector_3d_b)`
  - Implemented for `BaseMatrix<3, 1>` (3D Vectors) in `quanta/src/opr/cross.rs`.

## Matrix Operations

### Transpose (`transpose`)
- **Matrix Transpose**: `matrix.transpose()`
  - Implemented in `quanta/src/opr/transpose.rs`.
