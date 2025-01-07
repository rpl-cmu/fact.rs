//! Struct & traits for implementing residuals
//!
//! Residuals are the main building block of a
//! [factor](crate::containers::Factor).
//!
//! # Examples
//! Here we make a custom residual for a z-position measurement.
//! The `DimIn` and `DimOut` are the dimensions in and out, respectively,
//! while `V1` represents the type of the variable used (and more higher
//! numbered residuals have `V2`, `V3`, etc).
//!
//! `Differ` is the object that computes our auto-differentation. Out of the box
//! factrs comes with [ForwardProp](factrs::linalg::ForwardProp) and
//! [NumericalDiff](factrs::linalg::NumericalDiff). We recommend
//! [ForwardProp](factrs::linalg::ForwardProp) as it should be faster and more
//! accurate.
//!
//! Finally, the residual is defined through a single function that is generic
//! over the datatype. That's it! factrs handles the rest for you.
//!
//! ```
//! use std::fmt;
//!
//! use factrs::{
//!     dtype,
//!     linalg::{Const, ForwardProp, Numeric, VectorX},
//!     residuals,
//!     variables::SE3,
//! };
//!
//! #[derive(Debug, Clone)]
//! # #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
//! struct ZResidual {
//!     value: dtype,
//! }
//!
//! #[factrs::mark]
//! impl residuals::Residual1 for ZResidual {
//!     type DimIn = Const<6>;
//!     type DimOut = Const<1>;
//!     type V1 = SE3;
//!     type Differ = ForwardProp<Const<6>>;
//!
//!     fn residual1<T: Numeric>(&self, x1: SE3<T>) -> VectorX<T> {
//!         VectorX::from_element(1, T::from(self.value) - x1.xyz().z)
//!     }
//! }
//! ```
mod traits;
#[cfg(feature = "serde")]
pub use traits::tag_residual;
pub use traits::{Residual, Residual1, Residual2, Residual3, Residual4, Residual5, Residual6};

mod prior;
pub use prior::PriorResidual;

mod between;
pub use between::BetweenResidual;

pub mod imu_preint;
pub use imu_preint::{Accel, Gravity, Gyro, ImuCovariance, ImuPreintegrator};
