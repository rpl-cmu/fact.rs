use super::{Symbol, TypedSymbol};
use crate::{
    containers::{Key, Values},
    dtype,
    linalg::{Const, DiffResult, MatrixBlock},
    linear::LinearFactor,
    noise::{NoiseModel, NoiseModelSafe, UnitNoise},
    residuals::ResidualSafe,
    robust::{RobustCostSafe, L2},
};

/// Main structure to represent a factor in the graph.
///
/// $$ \blue{\rho_i}(||\purple{r_i}(\green{\Theta})||_{\red{\Sigma_i}} ) $$
///
/// Factors are the main building block of the factor graph. They are composed
/// of four pieces:
/// - <green>Keys</green>: The variables that the factor depends on, given by a
///   slice of [Keys](Key).
/// - <purple>Residual</purple>: The vector-valued function that computes the
///   error of the factor given a set of values, from the
///   [residual](crate::residuals) module.
/// - <red>Noise Model</red>: The noise model describes the uncertainty of the
///   residual, given by the traits in the [noise](crate::noise) module.
/// - <blue>Robust Kernel</blue>: The robust kernel weights the error of the
///   factor, given by the traits in the [robust](crate::robust) module.
///
/// To construct a factor, please see the [FactorBuilder] struct.
///
/// During optimization the factor is linearized around a set of values into a
/// [LinearFactor].
///
///  ```
/// # use factrs::{
///    assign_symbols,
///    containers::FactorBuilder,
///    noise::GaussianNoise,
///    optimizers::GaussNewton,
///    residuals::{PriorResidual},
///    robust::GemanMcClure,
///    variables::VectorVar3,
/// };
/// # assign_symbols!(X: VectorVar3);
/// let prior = VectorVar3::new(1.0, 2.0, 3.0);
/// let residual = PriorResidual::new(prior);
/// let noise = GaussianNoise::<3>::from_diag_sigmas(1e-1, 2e-1, 3e-1);
/// let robust = GemanMcClure::default();
/// let factor = FactorBuilder::new1(residual,
///     X(0)).noise(noise).robust(robust).build();
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Factor {
    keys: Vec<Key>,
    residual: Box<dyn ResidualSafe>,
    noise: Box<dyn NoiseModelSafe>,
    robust: Box<dyn RobustCostSafe>,
}

impl Factor {
    /// Compute the error of the factor given a set of values.
    pub fn error(&self, values: &Values) -> dtype {
        let r = self.residual.residual(values, &self.keys);
        let r = self.noise.whiten_vec(r);
        let norm2 = r.norm_squared();
        self.robust.loss(norm2)
    }

    /// Compute the dimension of the output of the factor.
    pub fn dim_out(&self) -> usize {
        self.residual.dim_out()
    }

    /// Linearize the factor given a set of values into a [LinearFactor].
    pub fn linearize(&self, values: &Values) -> LinearFactor {
        // Compute residual and jacobian
        let DiffResult { value: r, diff: a } = self.residual.residual_jacobian(values, &self.keys);

        // Whiten residual and jacobian
        let r = self.noise.whiten_vec(r);
        let a = self.noise.whiten_mat(a);

        // Weight according to robust cost
        let norm2 = r.norm_squared();
        let weight = self.robust.weight(norm2).sqrt();
        let a = weight * a;
        let b = -weight * r;

        // Turn A into a MatrixBlock
        let idx = self
            .keys
            .iter()
            .scan(0, |sum, k| {
                let out = Some(*sum);
                *sum += values.get_raw(*k).unwrap().dim();
                out
            })
            .collect::<Vec<_>>();
        let a = MatrixBlock::new(a, idx);

        LinearFactor::new(self.keys.clone(), a, b)
    }

    /// Get the keys of the factor.
    pub fn keys(&self) -> &[Key] {
        &self.keys
    }
}

/// Builder for a factor.
///
/// If the noise model or robust kernel aren't set, they default to [UnitNoise]
/// and [L2] respectively.
pub struct FactorBuilder<const DIM_OUT: usize> {
    keys: Vec<Key>,
    residual: Box<dyn ResidualSafe>,
    noise: Option<Box<dyn NoiseModelSafe>>,
    robust: Option<Box<dyn RobustCostSafe>>,
}

macro_rules! impl_new_builder {
    ($($num:expr, $( ($key:ident, $key_type:ident, $var:ident) ),*);* $(;)?) => {$(
        paste::paste! {
            #[doc = "Create a new factor with " $num " variable connections, while verifying the key types."]
            pub fn [<new $num>]<R, $($key_type),*>(residual: R, $($key: $key_type),*) -> Self
            where
                R: crate::residuals::[<Residual $num>]<DimOut = Const<DIM_OUT>> + ResidualSafe + 'static,
                $(
                    $key_type: TypedSymbol<R::$var>,
                )*
            {
                Self {
                    keys: vec![$( $key.into() ),*],
                    residual: Box::new(residual),
                    noise: None,
                    robust: None,
                }
            }

            #[doc = "Create a new factor with " $num " variable connections, without verifying the key types."]
            pub fn [<new $num _unchecked>]<R, $($key_type),*>(residual: R, $($key: $key_type),*) -> Self
            where
                R: crate::residuals::[<Residual $num>]<DimOut = Const<DIM_OUT>> + ResidualSafe + 'static,
                $(
                    $key_type: Symbol,
                )*
            {
                Self {
                    keys: vec![$( $key.into() ),*],
                    residual: Box::new(residual),
                    noise: None,
                    robust: None,
                }
            }
        }
    )*};
}

impl<const DIM_OUT: usize> FactorBuilder<DIM_OUT> {
    impl_new_builder! {
        1, (key1, K1, V1);
        2, (key1, K1, V1), (key2, K2, V2);
        3, (key1, K1, V1), (key2, K2, V2), (key3, K3, V3);
        4, (key1, K1, V1), (key2, K2, V2), (key3, K3, V3), (key4, K4, V4);
        5, (key1, K1, V1), (key2, K2, V2), (key3, K3, V3), (key4, K4, V4), (key5, K5, V5);
        6, (key1, K1, V1), (key2, K2, V2), (key3, K3, V3), (key4, K4, V4), (key5, K5, V5), (key6, K6, V6);
    }

    /// Add a noise model to the factor.
    pub fn noise<N>(mut self, noise: N) -> Self
    where
        N: 'static + NoiseModel<Dim = Const<DIM_OUT>> + NoiseModelSafe,
    {
        self.noise = Some(Box::new(noise));
        self
    }

    /// Add a robust kernel to the factor.
    pub fn robust<C>(mut self, robust: C) -> Self
    where
        C: 'static + RobustCostSafe,
    {
        self.robust = Some(Box::new(robust));
        self
    }

    /// Build the factor.
    pub fn build(self) -> Factor
    where
        UnitNoise<DIM_OUT>: NoiseModelSafe,
    {
        let noise = self.noise.unwrap_or_else(|| Box::new(UnitNoise::<DIM_OUT>));
        let robust = self.robust.unwrap_or_else(|| Box::new(L2));
        Factor {
            keys: self.keys.to_vec(),
            residual: self.residual,
            noise,
            robust,
        }
    }
}

#[cfg(test)]
mod tests {

    use matrixcompare::assert_matrix_eq;

    use super::*;
    use crate::{
        assign_symbols,
        linalg::{Diff, NumericalDiff},
        noise::GaussianNoise,
        residuals::{BetweenResidual, PriorResidual},
        robust::GemanMcClure,
        variables::{Variable, VectorVar3},
    };

    #[cfg(not(feature = "f32"))]
    const PWR: i32 = 6;
    #[cfg(not(feature = "f32"))]
    const TOL: f64 = 1e-6;

    #[cfg(feature = "f32")]
    const PWR: i32 = 3;
    #[cfg(feature = "f32")]
    const TOL: f32 = 1e-3;

    assign_symbols!(X: VectorVar3);

    #[test]
    fn linearize_a() {
        let prior = VectorVar3::new(1.0, 2.0, 3.0);
        let x = VectorVar3::identity();

        let residual = PriorResidual::new(prior);
        let noise = GaussianNoise::<3>::from_diag_sigmas(1e-1, 2e-1, 3e-1);
        let robust = GemanMcClure::default();

        let factor = FactorBuilder::new1(residual, X(0))
            .noise(noise)
            .robust(robust)
            .build();

        let f = |x: VectorVar3| {
            let mut values = Values::new();
            values.insert_unchecked(X(0), x);
            factor.error(&values)
        };

        let mut values = Values::new();
        values.insert_unchecked(X(0), x.clone());

        let linear = factor.linearize(&values);
        let grad_got = -linear.a.mat().transpose() * linear.b;
        println!("Received {:}", grad_got);

        let grad_num = NumericalDiff::<PWR>::gradient_1(f, &x).diff;
        println!("Expected {:}", grad_num);

        assert_matrix_eq!(grad_got, grad_num, comp = abs, tol = TOL);
    }

    #[test]
    fn linearize_block() {
        let bet = VectorVar3::new(1.0, 2.0, 3.0);
        let x = <VectorVar3 as Variable>::identity();

        let residual = BetweenResidual::new(bet);
        let noise = GaussianNoise::<3>::from_diag_sigmas(1e-1, 2e-1, 3e-1);
        let robust = GemanMcClure::default();

        let factor = FactorBuilder::new2(residual, X(0), X(1))
            .noise(noise)
            .robust(robust)
            .build();

        let mut values = Values::new();
        values.insert_unchecked(X(0), x.clone());
        values.insert_unchecked(X(1), x);

        let linear = factor.linearize(&values);

        println!("Full Mat {:}", linear.a.mat());
        println!("First Block {:}", linear.a.get_block(0));
        println!("Second Block {:}", linear.a.get_block(1));

        assert_matrix_eq!(
            linear.a.get_block(0),
            linear.a.mat().columns(0, 3),
            comp = float
        );
        assert_matrix_eq!(
            linear.a.get_block(1),
            linear.a.mat().columns(3, 3),
            comp = float
        );
    }
}
