use std::ops::Mul;

use faer::{
    prelude::SpSolver,
    sparse::{linalg::solvers, SparseColMatRef},
    Mat, MatRef,
};

use crate::dtype;

/// Trait to solve sparse linear systems
pub trait LinearSolver: Default {
    /// Solve a symmetric linear system
    ///
    /// This will be used by Cholesky to solve A^T A and by Levenberg-Marquardt
    /// to solve J^T J
    fn solve_symmetric(&mut self, a: SparseColMatRef<usize, dtype>, b: MatRef<dtype>)
        -> Mat<dtype>;

    /// Solve a least squares problem
    ///
    /// Used by QR to solve Ax = b, where the number of rows in A is greater
    /// than the number of columns
    fn solve_lst_sq(&mut self, a: SparseColMatRef<usize, dtype>, b: MatRef<dtype>) -> Mat<dtype>;
}

// ------------------------- Cholesky Linear Solver ------------------------- //

/// Cholesky linear solver
#[derive(Default)]
pub struct CholeskySolver {
    sparsity_pattern: Option<solvers::SymbolicCholesky<usize>>,
}

impl LinearSolver for CholeskySolver {
    fn solve_symmetric(
        &mut self,
        a: SparseColMatRef<usize, dtype>,
        b: MatRef<dtype>,
    ) -> Mat<dtype> {
        if self.sparsity_pattern.is_none() {
            self.sparsity_pattern = Some(
                solvers::SymbolicCholesky::try_new(a.symbolic(), faer::Side::Lower)
                    .expect("Symbolic cholesky failed"),
            );
        }

        solvers::Cholesky::try_new_with_symbolic(
            self.sparsity_pattern
                .clone()
                .expect("Missing symbol cholesky"),
            a,
            faer::Side::Lower,
        )
        .expect("Cholesky decomp failed")
        .solve(&b)
    }

    fn solve_lst_sq(&mut self, a: SparseColMatRef<usize, dtype>, b: MatRef<dtype>) -> Mat<dtype> {
        let ata = a
            .transpose()
            .to_col_major()
            .expect("Failed to transpose A matrix")
            .mul(a);

        let atb = a.transpose().mul(b);

        self.solve_symmetric(ata.as_ref(), atb.as_ref())
    }
}

// ------------------------- QR Linear Solver ------------------------- //

/// QR linear solver
#[derive(Default)]
pub struct QRSolver {
    sparsity_pattern: Option<solvers::SymbolicQr<usize>>,
}

impl LinearSolver for QRSolver {
    fn solve_symmetric(
        &mut self,
        a: SparseColMatRef<usize, dtype>,
        b: MatRef<dtype>,
    ) -> Mat<dtype> {
        self.solve_lst_sq(a, b)
    }

    fn solve_lst_sq(&mut self, a: SparseColMatRef<usize, dtype>, b: MatRef<dtype>) -> Mat<dtype> {
        if self.sparsity_pattern.is_none() {
            self.sparsity_pattern =
                Some(solvers::SymbolicQr::try_new(a.symbolic()).expect("Symbolic QR failed"));
        }

        // TODO: I think we're doing an extra copy here from solution -> slice solution
        solvers::Qr::try_new_with_symbolic(
            self.sparsity_pattern.clone().expect("Missing symbolic QR"),
            a,
        )
        .expect("QR failed")
        .solve(&b)
        .as_ref()
        .subrows(0, a.ncols())
        .to_owned()
    }
}

// ------------------------- LU Linear Solver ------------------------- //

/// LU linear solver
#[derive(Default)]
pub struct LUSolver {
    sparsity_pattern: Option<solvers::SymbolicLu<usize>>,
}

impl LinearSolver for LUSolver {
    fn solve_symmetric(
        &mut self,
        a: SparseColMatRef<usize, dtype>,
        b: MatRef<dtype>,
    ) -> Mat<dtype> {
        if self.sparsity_pattern.is_none() {
            self.sparsity_pattern =
                Some(solvers::SymbolicLu::try_new(a.symbolic()).expect("Symbolic LU failed"));
        }

        solvers::Lu::try_new_with_symbolic(
            self.sparsity_pattern.clone().expect("Symbolic LU missing"),
            a.as_ref(),
        )
        .expect("LU decomp failed")
        .solve(&b)
    }

    fn solve_lst_sq(&mut self, a: SparseColMatRef<usize, dtype>, b: MatRef<dtype>) -> Mat<dtype> {
        let ata = a
            .transpose()
            .to_col_major()
            .expect("Failed to transpose A matrix")
            .mul(a);
        let atb = a.transpose().mul(b);

        self.solve_symmetric(ata.as_ref(), atb.as_ref())
    }
}

#[cfg(test)]
mod test {
    use faer::{mat, sparse::SparseColMat};
    use matrixcompare::assert_matrix_eq;

    use super::*;

    fn solve<T: LinearSolver>(solver: &mut T) {
        let a = SparseColMat::<usize, dtype>::try_new_from_triplets(
            3,
            2,
            &[
                (0, 0, 10.0),
                (1, 0, 2.0),
                (2, 0, 3.0),
                (0, 1, 4.0),
                (1, 1, 20.0),
                (2, 1, -45.0),
            ],
        )
        .expect("Failed to make symbolic matrix");
        let b = mat![[15.0], [-3.0], [33.0]];

        let x_exp = mat![[1.874901], [-0.566112]];
        let x = solver.solve_lst_sq(a.as_ref(), b.as_ref());
        println!("{:?}", x);

        assert_matrix_eq!(x, x_exp, comp = abs, tol = 1e-6);
    }

    #[test]
    fn test_cholesky_solver() {
        let mut solver = CholeskySolver::default();
        solve(&mut solver);
    }

    #[test]
    fn test_qr_solver() {
        let mut solver = QRSolver::default();
        solve(&mut solver);
    }

    #[test]
    fn test_lu_solver() {
        let mut solver = LUSolver::default();
        solve(&mut solver);
    }
}
