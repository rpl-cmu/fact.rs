use std::{fmt, ops};

use super::Variable;
use crate::{
    dtype,
    linalg::{Const, DualVector, Numeric, SupersetOf, Vector3, VectorDim},
    residuals::{Accel, Gyro},
};

// TODO: Use newtypes internally as well?
/// IMU bias
///
/// The IMU bias is a 6D vector containing the gyro and accel biases. It is
/// treated as a 6D vector for optimization purposes.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuBias<T: Numeric = dtype> {
    gyro: Vector3<T>,
    accel: Vector3<T>,
}

impl<T: Numeric> ImuBias<T> {
    /// Create a new IMU bias
    pub fn new(gyro: Gyro<T>, accel: Accel<T>) -> Self {
        ImuBias {
            gyro: gyro.0,
            accel: accel.0,
        }
    }

    /// Create an IMU bias of zeros (same as identity)
    pub fn zeros() -> Self {
        ImuBias {
            gyro: Vector3::zeros(),
            accel: Vector3::zeros(),
        }
    }

    /// Get the gyro bias
    pub fn gyro(&self) -> &Vector3<T> {
        &self.gyro
    }

    /// Get the accel bias
    pub fn accel(&self) -> &Vector3<T> {
        &self.accel
    }
}

#[factrs::mark]
impl<T: Numeric> Variable for ImuBias<T> {
    type T = T;
    type Dim = crate::linalg::Const<6>;
    type Alias<TT: Numeric> = ImuBias<TT>;

    fn identity() -> Self {
        ImuBias {
            gyro: Vector3::zeros(),
            accel: Vector3::zeros(),
        }
    }

    fn inverse(&self) -> Self {
        ImuBias {
            gyro: -self.gyro,
            accel: -self.accel,
        }
    }

    fn compose(&self, other: &Self) -> Self {
        ImuBias {
            gyro: self.gyro + other.gyro,
            accel: self.accel + other.accel,
        }
    }

    fn exp(delta: crate::linalg::VectorViewX<T>) -> Self {
        let gyro = Vector3::new(delta[0], delta[1], delta[2]);
        let accel = Vector3::new(delta[3], delta[4], delta[5]);
        ImuBias { gyro, accel }
    }

    fn log(&self) -> crate::linalg::VectorX<T> {
        crate::linalg::vectorx![
            self.gyro.x,
            self.gyro.y,
            self.gyro.z,
            self.accel.x,
            self.accel.y,
            self.accel.z
        ]
    }

    fn cast<TT: Numeric + SupersetOf<Self::T>>(&self) -> Self::Alias<TT> {
        ImuBias {
            gyro: self.gyro.cast(),
            accel: self.accel.cast(),
        }
    }

    fn dual_exp<N: nalgebra::DimName>(idx: usize) -> Self::Alias<crate::linalg::DualVector<N>>
    where
        crate::linalg::AllocatorBuffer<N>: Sync + Send,
        nalgebra::DefaultAllocator: crate::linalg::DualAllocator<N>,
        crate::linalg::DualVector<N>: Copy,
    {
        let n = VectorDim::<N>::zeros().shape_generic().0;

        let mut gyro = Vector3::<DualVector<N>>::zeros();
        for (i, gi) in gyro.iter_mut().enumerate() {
            gi.eps = num_dual::Derivative::derivative_generic(n, Const::<1>, idx + i);
        }

        let mut accel = Vector3::<DualVector<N>>::zeros();
        for (i, ai) in accel.iter_mut().enumerate() {
            ai.eps = num_dual::Derivative::derivative_generic(n, Const::<1>, idx + i + 3);
        }

        ImuBias { gyro, accel }
    }
}

impl<T: Numeric> fmt::Display for ImuBias<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let precision = f.precision().unwrap_or(3);
        write!(
            f,
            "ImuBias(g: ({:.p$}, {:.p$}, {:.p$}), a: ({:.p$}, {:.p$}, {:.p$}))",
            self.gyro.x,
            self.gyro.y,
            self.gyro.z,
            self.accel.x,
            self.accel.y,
            self.accel.z,
            p = precision
        )
    }
}

impl<T: Numeric> fmt::Debug for ImuBias<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

impl<T: Numeric> ops::Sub for ImuBias<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        ImuBias {
            gyro: self.gyro - rhs.gyro,
            accel: self.accel - rhs.accel,
        }
    }
}

impl<T: Numeric> ops::Sub for &ImuBias<T> {
    type Output = ImuBias<T>;

    fn sub(self, rhs: Self) -> ImuBias<T> {
        ImuBias {
            gyro: self.gyro - rhs.gyro,
            accel: self.accel - rhs.accel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_variable;

    test_variable!(ImuBias);
}
