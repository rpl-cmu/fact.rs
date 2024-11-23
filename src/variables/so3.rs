use std::{fmt, ops};

use super::VectorVar4;
use crate::{
    dtype,
    linalg::{
        vectorx, AllocatorBuffer, Const, DefaultAllocator, Derivative, DimName, DualAllocator,
        DualVector, Matrix3, MatrixView, Numeric, Vector3, Vector4, VectorDim, VectorView3,
        VectorViewX, VectorX,
    },
    variables::{MatrixLieGroup, Variable},
};

/// 3D Special Orthogonal Group
///
/// Implementation of SO(3) for 3D rotations. Specifically, we use quaternions
/// to represent rotations due to their underyling efficiency when computing
/// log/exp maps.
#[derive(Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SO3<T: Numeric = dtype> {
    pub xyzw: Vector4<T>,
}

impl<T: Numeric> SO3<T> {
    /// Create a new SO3 from a Vector4
    pub fn from_vec(xyzw: Vector4<T>) -> Self {
        SO3 { xyzw }
    }

    /// Create a new SO3 from x, y, z, w
    pub fn from_xyzw(x: T, y: T, z: T, w: T) -> Self {
        SO3 {
            xyzw: Vector4::<T>::new(x, y, z, w),
        }
    }

    pub fn x(&self) -> T {
        self.xyzw[0]
    }

    pub fn y(&self) -> T {
        self.xyzw[1]
    }

    pub fn z(&self) -> T {
        self.xyzw[2]
    }

    pub fn w(&self) -> T {
        self.xyzw[3]
    }

    pub fn dexp(xi: VectorView3<T>) -> Matrix3<T> {
        let theta2 = xi.norm_squared();

        let (a, b) = if theta2 < T::from(1e-6) {
            (T::from(0.5), T::from(1.0) / T::from(6.0))
        } else {
            let theta = theta2.sqrt();
            let a = (T::from(1.0) - theta.cos()) / theta2;
            let b = (theta - theta.sin()) / (theta * theta2);
            (a, b)
        };

        let hat = SO3::hat(xi);
        // gtsam says minus here for -hat a, but ethan eade says plus
        // Empirically (via our test & jac in ImuDelta), minus is correct
        // Need to find reference to confirm
        Matrix3::identity() - hat * a + hat * hat * b
    }
}

#[factrs::tag]
impl<T: Numeric> Variable<T> for SO3<T> {
    type Dim = Const<3>;
    type Alias<TT: Numeric> = SO3<TT>;

    fn identity() -> Self {
        SO3 { xyzw: Vector4::w() }
    }

    fn inverse(&self) -> Self {
        SO3 {
            xyzw: Vector4::new(-self.xyzw[0], -self.xyzw[1], -self.xyzw[2], self.xyzw[3]),
        }
    }

    fn compose(&self, other: &Self) -> Self {
        let x0 = self.xyzw.x;
        let y0 = self.xyzw.y;
        let z0 = self.xyzw.z;
        let w0 = self.xyzw.w;

        let x1 = other.xyzw.x;
        let y1 = other.xyzw.y;
        let z1 = other.xyzw.z;
        let w1 = other.xyzw.w;

        // Compute the product of the two quaternions, term by term
        let mut xyzw = Vector4::zeros();
        xyzw[0] = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
        xyzw[1] = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
        xyzw[2] = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;
        xyzw[3] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;

        SO3 { xyzw }
    }

    fn exp(xi: VectorViewX<T>) -> Self {
        let mut xyzw = Vector4::zeros();

        let theta = xi.norm();

        xyzw.w = (theta * T::from(0.5)).cos();

        if theta < T::from(1e-3) {
            let tmp = xyzw.w * T::from(0.5);
            xyzw.x = xi[0] * tmp;
            xyzw.y = xi[1] * tmp;
            xyzw.z = xi[2] * tmp;
        } else {
            let omega = xi / theta;
            let sin_theta_half = (T::from(1.0) - xyzw.w * xyzw.w).sqrt();
            xyzw.x = omega[0] * sin_theta_half;
            xyzw.y = omega[1] * sin_theta_half;
            xyzw.z = omega[2] * sin_theta_half;
        }

        SO3 { xyzw }
    }

    fn log(&self) -> VectorX<T> {
        let xi = vectorx![self.xyzw.x, self.xyzw.y, self.xyzw.z];
        // Abs value in case we had a negative quaternion
        let w = self.xyzw.w.abs();

        let norm_v = xi.norm();
        if norm_v < T::from(1e-3) {
            xi * T::from(2.0)
        } else {
            xi * norm_v.atan2(w) * T::from(2.0) / norm_v
        }
    }

    fn dual_convert<TT: Numeric>(other: &Self::Alias<dtype>) -> Self::Alias<TT> {
        SO3 {
            xyzw: VectorVar4::<dtype>::dual_convert(&other.xyzw.into()).into(),
        }
    }

    fn dual_setup<N: DimName>(idx: usize) -> Self::Alias<DualVector<N>>
    where
        AllocatorBuffer<N>: Sync + Send,
        DefaultAllocator: DualAllocator<N>,
        DualVector<N>: Copy,
    {
        let mut x = DualVector::<N>::from_re(0.0);
        let mut eps = VectorDim::<N>::zeros();
        eps[idx] = 0.5;
        x.eps = Derivative::new(Some(eps));

        let mut y = DualVector::<N>::from_re(0.0);
        let mut eps = VectorDim::<N>::zeros();
        eps[idx + 1] = 0.5;
        y.eps = Derivative::new(Some(eps));

        let mut z = DualVector::<N>::from_re(0.0);
        let mut eps = VectorDim::<N>::zeros();
        eps[idx + 2] = 0.5;
        z.eps = Derivative::new(Some(eps));

        let w = DualVector::<N>::from_re(1.0);

        SO3::from_xyzw(x, y, z, w)
    }
}

impl<T: Numeric> MatrixLieGroup<T> for SO3<T> {
    type TangentDim = Const<3>;
    type MatrixDim = Const<3>;
    type VectorDim = Const<3>;

    fn adjoint(&self) -> Matrix3<T> {
        let q0 = self.xyzw.w;
        let q1 = self.xyzw.x;
        let q2 = self.xyzw.y;
        let q3 = self.xyzw.z;

        // Same as to_matrix function of SO3 -> Just avoiding copying from Matrix3 to
        // MatrixD
        let mut mat = Matrix3::zeros();
        mat[(0, 0)] = T::from(1.0) - (q2 * q2 + q3 * q3) * 2.0;
        mat[(0, 1)] = (q1 * q2 - q0 * q3) * 2.0;
        mat[(0, 2)] = (q1 * q3 + q0 * q2) * 2.0;
        mat[(1, 0)] = (q1 * q2 + q0 * q3) * 2.0;
        mat[(1, 1)] = T::from(1.0) - (q1 * q1 + q3 * q3) * 2.0;
        mat[(1, 2)] = (q2 * q3 - q0 * q1) * 2.0;
        mat[(2, 0)] = (q1 * q3 - q0 * q2) * 2.0;
        mat[(2, 1)] = (q2 * q3 + q0 * q1) * 2.0;
        mat[(2, 2)] = T::from(1.0) - (q1 * q1 + q2 * q2) * 2.0;

        mat
    }

    fn hat(xi: VectorView3<T>) -> Matrix3<T> {
        let mut xi_hat = Matrix3::zeros();
        xi_hat[(0, 1)] = -xi[2];
        xi_hat[(0, 2)] = xi[1];
        xi_hat[(1, 0)] = xi[2];
        xi_hat[(1, 2)] = -xi[0];
        xi_hat[(2, 0)] = -xi[1];
        xi_hat[(2, 1)] = xi[0];

        xi_hat
    }

    fn vee(xi: MatrixView<3, 3, T>) -> Vector3<T> {
        Vector3::new(xi[(2, 1)], xi[(0, 2)], xi[(1, 0)])
    }

    fn hat_swap(xi: VectorView3<T>) -> Matrix3<T> {
        -Self::hat(xi)
    }

    fn from_matrix(mat: MatrixView<3, 3, T>) -> Self {
        let trace = mat[(0, 0)] + mat[(1, 1)] + mat[(2, 2)];
        let mut xyzw = Vector4::zeros();
        let zero = T::from(0.0);
        let quarter = T::from(0.25);
        let one = T::from(1.0);
        let two = T::from(2.0);

        if trace > zero {
            let s = T::from(0.5) / (trace + 1.0).sqrt();
            xyzw[3] = quarter / s;
            xyzw[0] = (mat[(2, 1)] - mat[(1, 2)]) * s;
            xyzw[1] = (mat[(0, 2)] - mat[(2, 0)]) * s;
            xyzw[2] = (mat[(1, 0)] - mat[(0, 1)]) * s;
        } else if mat[(0, 0)] > mat[(1, 1)] && mat[(0, 0)] > mat[(2, 2)] {
            let s = two * (one + mat[(0, 0)] - mat[(1, 1)] - mat[(2, 2)]).sqrt();
            xyzw[3] = (mat[(2, 1)] - mat[(1, 2)]) / s;
            xyzw[0] = s * quarter;
            xyzw[1] = (mat[(0, 1)] + mat[(1, 0)]) / s;
            xyzw[2] = (mat[(0, 2)] + mat[(2, 0)]) / s;
        } else if mat[(1, 1)] > mat[(2, 2)] {
            let s = two * (one + mat[(1, 1)] - mat[(0, 0)] - mat[(2, 2)]).sqrt();
            xyzw[3] = (mat[(0, 2)] - mat[(2, 0)]) / s;
            xyzw[0] = (mat[(0, 1)] + mat[(1, 0)]) / s;
            xyzw[1] = s * quarter;
            xyzw[2] = (mat[(1, 2)] + mat[(2, 1)]) / s;
        } else {
            let s = two * (one + mat[(2, 2)] - mat[(0, 0)] - mat[(1, 1)]).sqrt();
            xyzw[3] = (mat[(1, 0)] - mat[(0, 1)]) / s;
            xyzw[0] = (mat[(0, 2)] + mat[(2, 0)]) / s;
            xyzw[1] = (mat[(1, 2)] + mat[(2, 1)]) / s;
            xyzw[2] = s * quarter;
        }

        SO3 { xyzw }
    }

    fn to_matrix(&self) -> Matrix3<T> {
        let q0 = self.xyzw[3];
        let q1 = self.xyzw[0];
        let q2 = self.xyzw[1];
        let q3 = self.xyzw[2];

        let mut mat = Matrix3::zeros();
        mat[(0, 0)] = T::from(1.0) - (q2 * q2 + q3 * q3) * 2.0;
        mat[(0, 1)] = (q1 * q2 - q0 * q3) * 2.0;
        mat[(0, 2)] = (q1 * q3 + q0 * q2) * 2.0;
        mat[(1, 0)] = (q1 * q2 + q0 * q3) * 2.0;
        mat[(1, 1)] = T::from(1.0) - (q1 * q1 + q3 * q3) * 2.0;
        mat[(1, 2)] = (q2 * q3 - q0 * q1) * 2.0;
        mat[(2, 0)] = (q1 * q3 - q0 * q2) * 2.0;
        mat[(2, 1)] = (q2 * q3 + q0 * q1) * 2.0;
        mat[(2, 2)] = T::from(1.0) - (q1 * q1 + q2 * q2) * 2.0;

        mat
    }

    fn apply(&self, v: VectorView3<T>) -> Vector3<T> {
        let qv = Self::from_xyzw(v[0], v[1], v[2], (0.0).into());
        let inv = self.inverse();

        let v_rot = (&(self * &qv) * &inv).xyzw;
        Vector3::new(v_rot[0], v_rot[1], v_rot[2])
    }
}

impl<T: Numeric> ops::Mul for SO3<T> {
    type Output = SO3<T>;

    fn mul(self, other: Self) -> Self::Output {
        self.compose(&other)
    }
}

impl<T: Numeric> ops::Mul for &SO3<T> {
    type Output = SO3<T>;

    fn mul(self, other: Self) -> Self::Output {
        self.compose(other)
    }
}

impl<T: Numeric> fmt::Display for SO3<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "SO3({:.3}, {:.3}, {:.3}, {:.3})",
            self.xyzw[0], self.xyzw[1], self.xyzw[2], self.xyzw[3]
        )
    }
}

impl<T: Numeric> fmt::Debug for SO3<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

#[cfg(test)]
mod tests {
    use matrixcompare::assert_matrix_eq;

    use super::*;
    use crate::{linalg::NumericalDiff, test_lie, test_variable, variables::VectorVar3};

    test_variable!(SO3);

    test_lie!(SO3);

    #[test]
    fn dexp() {
        let xi = Vector3::new(0.1, 0.2, 0.3);
        let got = SO3::dexp(xi.as_view());

        let exp = NumericalDiff::<6>::jacobian_variable_1(
            |x: VectorVar3| SO3::exp(Vector3::from(x).as_view()),
            &VectorVar3::from(xi),
        )
        .diff;

        println!("got: {}", got);
        println!("exp: {}", exp);
        assert_matrix_eq!(got, exp, comp = abs, tol = 1e-6);
    }
}
