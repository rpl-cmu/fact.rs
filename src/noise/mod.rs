//! Noise model representations
//!
//! Represent Gaussian noise models in a factor graph, specifically used when
//! constructing a [factor](crate::containers::Factor).

use std::fmt::{Debug, Display};

use crate::linalg::{DimName, MatrixX, VectorX};

/*
TODO:
- Make list of Variable/Residuals to tag as variables are added
- Make sure this works across crates? Think it should just fine
*/

/// The trait for a noise model.
#[cfg_attr(feature = "serde", typetag::serde(tag = "tag"))]
pub trait NoiseModel: Debug + Display {
    /// The dimension of the noise model
    type Dim: DimName
    where
        Self: Sized;

    fn dim(&self) -> usize
    where
        Self: Sized,
    {
        Self::Dim::USIZE
    }

    /// Whiten a vector
    fn whiten_vec(&self, v: VectorX) -> VectorX;

    /// Whiten a matrix
    fn whiten_mat(&self, m: MatrixX) -> MatrixX;
}

#[cfg(feature = "serde")]
pub use register_noisemodel as tag_noise;

mod gaussian;
pub use gaussian::GaussianNoise;

mod unit;
pub use unit::UnitNoise;
