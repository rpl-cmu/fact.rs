use crate::dtype;
use crate::linalg::{Const, DualNum, DualVec, Dyn, MatrixX, VectorViewX, VectorX};

use std::fmt::{Debug, Display};

pub trait Variable<D: DualNum = dtype>: Clone + Sized + Display + Debug {
    const DIM: usize;
    type Dual: Variable<DualVec>;

    // Group operations
    fn identity() -> Self;
    fn inverse(&self) -> Self;
    fn compose(&self, other: &Self) -> Self;
    fn exp(delta: VectorViewX<D>) -> Self; // trivial if linear (just itself)
    fn log(&self) -> VectorX<D>; // trivial if linear (just itself)

    // Conversion to dual space
    fn dual_self(&self) -> Self::Dual;

    // Helpers for enum
    fn dim(&self) -> usize {
        Self::DIM
    }
    fn identity_enum(&self) -> Self {
        Self::identity()
    }

    // Moves to and from vector space
    fn oplus(&self, delta: VectorViewX<D>) -> Self {
        if cfg!(feature = "left") {
            Self::exp(delta).compose(self)
        } else {
            self.compose(&Self::exp(delta))
        }
    }
    fn minus(&self, other: &Self) -> Self {
        if cfg!(feature = "left") {
            self.compose(&other.inverse())
        } else {
            other.inverse().compose(self)
        }
    }
    fn ominus(&self, other: &Self) -> VectorX<D> {
        self.minus(other).log()
    }

    // Create tangent vector w/ duals set up properly
    fn dual_tangent(&self, idx: usize, total: usize) -> VectorX<DualVec> {
        let mut tv: VectorX<DualVec> = VectorX::zeros(self.dim());
        for (i, tvi) in tv.iter_mut().enumerate() {
            tvi.eps = num_dual::Derivative::derivative_generic(Dyn(total), Const::<1>, idx + i);
        }
        tv
    }
    // Applies the tangent vector in dual space
    fn dual(&self, idx: usize, total: usize) -> Self::Dual {
        self.dual_self()
            .oplus(self.dual_tangent(idx, total).as_view())
    }
}

// TODO: Expand Lie group definition & move all tests together
pub trait LieGroup<D: DualNum>: Variable<D> {
    fn adjoint(&self) -> MatrixX<D>;

    fn hat(xi: VectorViewX<D>) -> MatrixX<D>;

    // fn vee(xi: MatrixX<D>) -> VectorX<D>;

    // fn apply(&self, v: &VectorX<D>) -> VectorX<D>;

    // fn hat_swap(&self, xi: VectorViewX<D>) -> MatrixX<D>;

    // fn to_matrix(&self) -> MatrixX<D>;

    // fn from_matrix(mat: &MatrixX<D>) -> Self;
}
