use std::{
    fmt::{Debug, Display},
    ops::Index,
};

use crate::{
    dtype,
    linalg::{
        AllocatorBuffer, Const, DefaultAllocator, DimName, DualAllocator, DualVector, Numeric,
        Vector, VectorDim, VectorViewX, VectorX,
    },
    variables::Variable,
};

// ------------------------- Our needs ------------------------- //
/// Newtype wrapper around nalgebra::Vector
///
/// We create a newtype specifically for vectors we're estimating over due to,
/// 1 - So we can manually implement Debug/Display
/// 2 - Overcome identity issues with the underlying Vector type
/// 3 - Impl Into\<Rerun types\>
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VectorVar<const N: usize, T: Numeric = dtype>(pub Vector<N, T>);

#[factrs::tag]
impl<const N: usize, T: Numeric> Variable<T> for VectorVar<N, T> {
    type Dim = Const<N>;
    type Alias<TT: Numeric> = VectorVar<N, TT>;

    fn identity() -> Self {
        VectorVar(Vector::zeros())
    }

    fn inverse(&self) -> Self {
        VectorVar(-self.0)
    }

    fn compose(&self, other: &Self) -> Self {
        VectorVar(self.0 + other.0)
    }

    fn exp(delta: VectorViewX<T>) -> Self {
        VectorVar(Vector::from_iterator(delta.iter().cloned()))
    }

    fn log(&self) -> VectorX<T> {
        VectorX::from_iterator(Self::DIM, self.0.iter().cloned())
    }

    fn dual_convert<TT: Numeric>(other: &Self::Alias<dtype>) -> Self::Alias<TT> {
        VectorVar(other.0.map(|x| x.into()))
    }

    // Mostly unncessary, but avoids having to convert VectorX to static vector
    fn dual_setup<NN: DimName>(idx: usize) -> Self::Alias<DualVector<NN>>
    where
        AllocatorBuffer<NN>: Sync + Send,
        DefaultAllocator: DualAllocator<NN>,
        DualVector<NN>: Copy,
    {
        let n = VectorDim::<NN>::zeros().shape_generic().0;
        let mut tv = Vector::<N, DualVector<NN>>::zeros();
        for (i, tvi) in tv.iter_mut().enumerate() {
            tvi.eps = num_dual::Derivative::derivative_generic(n, Const::<1>, idx + i);
        }
        VectorVar(tv)
    }
}

macro_rules! impl_vector_new {
    ($($num:literal, [$($args:ident),*]);* $(;)?) => {$(
        impl<T: Numeric> VectorVar<$num, T> {
            pub fn new($($args: T),*) -> Self {
                VectorVar(Vector::<$num, T>::new($($args),*))
            }
        }
    )*};
}

impl_vector_new!(
    1, [x];
    2, [x, y];
    3, [x, y, z];
    4, [x, y, z, w];
    5, [x, y, z, w, a];
    6, [x, y, z, w, a, b];
);

impl<const N: usize, T: Numeric> From<Vector<N, T>> for VectorVar<N, T> {
    fn from(v: Vector<N, T>) -> Self {
        VectorVar(v)
    }
}

impl<const N: usize, T: Numeric> From<VectorVar<N, T>> for Vector<N, T> {
    fn from(v: VectorVar<N, T>) -> Self {
        v.0
    }
}

impl<const N: usize, T: Numeric> Display for VectorVar<N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Vector{}(", N)?;
        for (i, x) in self.0.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{:.3}", x)?;
        }
        write!(f, ")")
    }
}

impl<const N: usize, T: Numeric> Debug for VectorVar<N, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Display::fmt(self, f)
    }
}

impl<const N: usize, T: Numeric> Index<usize> for VectorVar<N, T> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

/// 1D Vector Variable
pub type VectorVar1<T = dtype> = VectorVar<1, T>;
/// 2D Vector Variable
pub type VectorVar2<T = dtype> = VectorVar<2, T>;
/// 3D Vector Variable
pub type VectorVar3<T = dtype> = VectorVar<3, T>;
/// 4D Vector Variable
pub type VectorVar4<T = dtype> = VectorVar<4, T>;
/// 5D Vector Variable
pub type VectorVar5<T = dtype> = VectorVar<5, T>;
/// 6D Vector Variable
pub type VectorVar6<T = dtype> = VectorVar<6, T>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_variable;

    // Be lazy and only test Vector6 - others should work the same
    test_variable!(VectorVar6);
}
