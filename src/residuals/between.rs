use nalgebra::{DimNameAdd, DimNameSum};

use crate::{
    linalg::{
        AllocatorBuffer, DefaultAllocator, DualAllocator, DualVector, ForwardProp, Numeric, VectorX,
    },
    residuals::Residual2,
    variables::{Variable, VariableDtype},
};

/// Binary factor between variables.
///
/// This residual is used to enforce a constraint between two variables.
/// Specifically it computes
///
/// $$
/// r = (v_1 z) \ominus v_2
/// $$
///
/// where $z$ is the measured value.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct BetweenResidual<P: Variable> {
    delta: P,
}

impl<P: Variable> BetweenResidual<P> {
    pub fn new(delta: P) -> Self {
        Self { delta }
    }
}

#[factrs::mark]
impl<P: VariableDtype + 'static> Residual2 for BetweenResidual<P>
where
    AllocatorBuffer<DimNameSum<P::Dim, P::Dim>>: Sync + Send,
    DefaultAllocator: DualAllocator<DimNameSum<P::Dim, P::Dim>>,
    DualVector<DimNameSum<P::Dim, P::Dim>>: Copy,
    P::Dim: DimNameAdd<P::Dim>,
{
    type Differ = ForwardProp<DimNameSum<P::Dim, P::Dim>>;
    type V1 = P;
    type V2 = P;
    type DimOut = P::Dim;
    type DimIn = DimNameSum<P::Dim, P::Dim>;

    fn residual2<T: Numeric>(&self, v1: P::Alias<T>, v2: P::Alias<T>) -> VectorX<T> {
        let delta = self.delta.cast::<T>();
        v1.compose(&delta).ominus(&v2)
    }
}
