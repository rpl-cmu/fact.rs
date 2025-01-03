//! fact.rs is a nonlinear least squares optimization library over factor
//! graphs, specifically geared for sensor fusion in robotics.
//!
//! Currently, it supports the following features
//! - Gauss-Newton & Levenberg-Marquadt Optimizers
//! - Common Lie Groups supported (SO2, SO3, SE2, SE3) with optimization in Lie
//!   Algebras
//! - Automatic differentiation via dual numbers
//! - First class support for robust kernels
//! - Serialization of graphs & variables via optional serde support
//! - Easy conversion to rerun types for simple visualization
//!
//! # Background
//!
//! Specifically, we solve the following problem,
//!
//! $$
//! \blue{\Theta^*} = \red{\argmin_{\Theta}}
//! \sum_{i} \green{\rho_i(||r_i(\Theta)||_{\Sigma_i} )}
//! $$
//!
//! The fact.rs API takes heavy inspiration from the [gtsam library](https://gtsam.org/),
//! and is loosely structured as follows,
//! - <blue>Variables</blue>: These are the unknowns in the optimization
//!   problem. They are all lie-group based (even if trivially so). See the
//!   [variable](crate::variables) module for details on custom implementations.
//!   A collection of variables is stored in a
//!   [Values](crate::containers::Values) container.
//! - <red>Optimizers</red>: The optimizer is responsible for finding the
//!   optimal variables that minimize the factors. More info on factor graph
//!   optimizers in [Optimizers](crate::optimizers).
//! - <green>Factors</green>: Each factor represents a probabilistic constraint
//!   in the optimization. More info in [Factor](crate::containers::Factor). A
//!   collection of factors is stored in a [Graph](crate::containers::Graph)
//!   container.
//!
//! # Example
//! ```
//! use factrs::{
//!     assign_symbols,
//!     core::{BetweenResidual, GaussNewton, Graph, Huber, PriorResidual, Values, SO2},
//!     fac,
//!     traits::*,
//! };
//!
//! // Assign symbols to variable types
//! assign_symbols!(X: SO2);
//!
//! // Make all the values
//! let mut values = Values::new();
//!
//! let x = SO2::from_theta(1.0);
//! let y = SO2::from_theta(2.0);
//! values.insert(X(0), SO2::identity());
//! values.insert(X(1), SO2::identity());
//!
//! // Make the factors & insert into graph
//! let mut graph = Graph::new();
//! let res = PriorResidual::new(x.clone());
//! let factor = fac![res, X(0)];
//! graph.add_factor(factor);
//!
//! let res = BetweenResidual::new(y.minus(&x));
//! let factor = fac![res, (X(0), X(1)), 0.1 as std, Huber::default()];
//! // fac! is syntactic sugar for the following
//! // let noise = GaussianNoise::from_scalar_sigma(0.1);
//! // let factor = FactorBuilder::new2(res, X(0), X(1))
//! //     .noise(GaussianNoise::from_scalar_sigma(0.1))
//! //     .robust(Huber::default())
//! //     .build();
//! graph.add_factor(factor);
//!
//! // Optimize!
//! let mut opt: GaussNewton = GaussNewton::new(graph);
//! let result = opt.optimize(values).unwrap();
//! println!("Results {:#}", result);
//! ```

#![warn(clippy::unwrap_used)]

/// The default floating point type used in the library
#[cfg(not(feature = "f32"))]
#[allow(non_camel_case_types)]
pub type dtype = f64;

#[cfg(feature = "f32")]
#[allow(non_camel_case_types)]
pub type dtype = f32;

// Hack to be able to use our proc macro inside and out of our crate
// https://users.rust-lang.org/t/how-to-express-crate-path-in-procedural-macros/91274/10
#[doc(hidden)]
extern crate self as factrs;
pub use factrs_proc::{fac, mark};

pub mod containers;
pub mod linalg;
pub mod linear;
pub mod noise;
pub mod optimizers;
pub mod residuals;
pub mod robust;
pub mod utils;
pub mod variables;

/// Untagged symbols if `unchecked` API is desired.
///
/// We strongly recommend using [assign_symbols] to
/// create and tag symbols with the appropriate types. However, we provide a
/// number of pre-defined symbols if desired. Note these objects can't be tagged
/// due to the orphan rules.
pub mod symbols {
    crate::assign_symbols!(
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z
    );
}

/// Helper module to import common traits
///
/// This module is meant to be glob imported to make it easier to use the traits
/// and functionality in the library.
/// ```
/// use factrs::traits::*;
/// ```
pub mod traits {
    pub use crate::{
        linalg::Diff, noise::NoiseModel, optimizers::Optimizer, residuals::Residual,
        robust::RobustCost, variables::Variable,
    };
}

/// Helper module to group together common types
///
/// Specifically, this contains everything that would be needed to implement a
/// simple pose graph. While we recommend against it, for quick usage it can be
/// glob imported as
/// ```
/// use factrs::core::*;
/// ```
pub mod core {
    pub use crate::{
        assign_symbols,
        containers::{Factor, Graph, Values},
        fac,
        noise::{GaussianNoise, UnitNoise},
        optimizers::{GaussNewton, LevenMarquardt},
        residuals::{BetweenResidual, PriorResidual},
        robust::{GemanMcClure, Huber, L2},
        variables::{VectorVar, VectorVar1, VectorVar2, VectorVar3, SE2, SE3, SO2, SO3},
    };
}

#[cfg(feature = "rerun")]
pub mod rerun;

#[cfg(feature = "serde")]
pub mod serde {
    pub use crate::{
        noise::tag_noise, residuals::tag_residual, robust::tag_robust, variables::tag_variable,
    };
}
