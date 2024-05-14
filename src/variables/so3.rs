use crate::dtype;
use crate::traits::{LieGroup, Variable};
use crate::variables::{Vector3, Vector4, VectorD};
use nalgebra::{dvector, ComplexField, RealField};
use std::fmt;
use std::ops::Mul;

#[derive(Clone)]
pub struct SO3 {
    xyzw: Vector4,
}

impl SO3 {
    pub fn from_vec(xyzw: Vector4) -> Self {
        SO3 { xyzw }
    }

    pub fn from_xyzw(x: dtype, y: dtype, z: dtype, w: dtype) -> Self {
        SO3 {
            xyzw: Vector4::new(x, y, z, w),
        }
    }

    pub fn from_matrix(mat: &nalgebra::Matrix3<dtype>) -> Self {
        let trace = mat[(0, 0)] + mat[(1, 1)] + mat[(2, 2)];
        let mut xyzw = Vector4::zeros();
        let zero: dtype = (0.0).into();
        let quarter: dtype = (0.25).into();
        let half: dtype = (0.5).into();
        let one: dtype = (1.0).into();
        let two: dtype = (2.0).into();

        if trace > zero {
            let s = half / (trace + 1.0).sqrt();
            xyzw[3] = quarter / s;
            xyzw[0] = (mat[(2, 1)] - mat[(1, 2)]) * s;
            xyzw[1] = (mat[(0, 2)] - mat[(2, 0)]) * s;
            xyzw[2] = (mat[(1, 0)] - mat[(0, 1)]) * s;
        } else if mat[(0, 0)] > mat[(1, 1)] && mat[(0, 0)] > mat[(2, 2)] {
            let s = two * (one + mat[(0, 0)] - mat[(1, 1)] - mat[(2, 2)]).sqrt();
            xyzw[3] = (mat[(2, 1)] - mat[(1, 2)]) / s;
            xyzw[0] = s * 0.25;
            xyzw[1] = (mat[(0, 1)] + mat[(1, 0)]) / s;
            xyzw[2] = (mat[(0, 2)] + mat[(2, 0)]) / s;
        } else if mat[(1, 1)] > mat[(2, 2)] {
            let s = two * (one + mat[(1, 1)] - mat[(0, 0)] - mat[(2, 2)]).sqrt();
            xyzw[3] = (mat[(0, 2)] - mat[(2, 0)]) / s;
            xyzw[0] = (mat[(0, 1)] + mat[(1, 0)]) / s;
            xyzw[1] = s * 0.25;
            xyzw[2] = (mat[(1, 2)] + mat[(2, 1)]) / s;
        } else {
            let s = two * (one + mat[(2, 2)] - mat[(0, 0)] - mat[(1, 1)]).sqrt();
            xyzw[3] = (mat[(1, 0)] - mat[(0, 1)]) / s;
            xyzw[0] = (mat[(0, 2)] + mat[(2, 0)]) / s;
            xyzw[1] = (mat[(1, 2)] + mat[(2, 1)]) / s;
            xyzw[2] = s * 0.25;
        }

        SO3 { xyzw }
    }

    pub fn to_matrix(&self) -> nalgebra::Matrix3<dtype> {
        let q0 = self.xyzw[3];
        let q1 = self.xyzw[0];
        let q2 = self.xyzw[1];
        let q3 = self.xyzw[2];
        let one: dtype = (1.0).into();

        let mut mat = nalgebra::Matrix3::zeros();
        mat[(0, 0)] = one - (q2 * q2 + q3 * q3) * 2.0;
        mat[(0, 1)] = (q1 * q2 - q0 * q3) * 2.0;
        mat[(0, 2)] = (q1 * q3 + q0 * q2) * 2.0;
        mat[(1, 0)] = (q1 * q2 + q0 * q3) * 2.0;
        mat[(1, 1)] = one - (q1 * q1 + q3 * q3) * 2.0;
        mat[(1, 2)] = (q2 * q3 - q0 * q1) * 2.0;
        mat[(2, 0)] = (q1 * q3 - q0 * q2) * 2.0;
        mat[(2, 1)] = (q2 * q3 + q0 * q1) * 2.0;
        mat[(2, 2)] = one - (q1 * q1 + q2 * q2) * 2.0;

        mat
    }

    pub fn apply(&self, v: &Vector3) -> Vector3 {
        let qv = Self::from_xyzw(v[0], v[1], v[2], (0.0).into());
        let inv = self.inverse();

        let v_rot = (&(&inv * &qv) * self).xyzw;
        Vector3::new(v_rot[0], v_rot[1], v_rot[2])
    }
}

impl Variable for SO3 {
    const DIM: usize = 3;

    fn identity() -> Self {
        SO3 {
            xyzw: Vector4::new((0.0).into(), (0.0).into(), (0.0).into(), (1.0).into()),
        }
    }

    fn inverse(&self) -> Self {
        SO3 {
            xyzw: Vector4::new(-self.xyzw[0], -self.xyzw[1], -self.xyzw[2], self.xyzw[3]),
        }
    }

    fn oplus(&self, delta: &VectorD) -> Self {
        let e = Self::exp(delta);
        self * &e
    }

    fn ominus(&self, other: &Self) -> VectorD {
        (&Variable::inverse(self) * other).log()
    }
}

impl LieGroup for SO3 {
    fn exp(xi: &VectorD) -> Self {
        let mut xyzw = Vector4::zeros();
        let one: dtype = (1.0).into();

        let theta = xi.norm();
        if theta < (1e-2).into() {
            xyzw[0] = xi[0] * (one - theta * theta / 6.0);
            xyzw[1] = xi[1] * (one - theta * theta / 6.0);
            xyzw[2] = xi[2] * (one - theta * theta / 6.0);
            xyzw[3] = one;
        } else {
            let theta_half = theta / 2.0;
            let sin_theta = theta_half.sin();
            xyzw[0] = xi[0] * sin_theta / theta;
            xyzw[1] = xi[1] * sin_theta / theta;
            xyzw[2] = xi[2] * sin_theta / theta;
            xyzw[3] = theta_half.cos();
        }

        SO3 { xyzw }
    }

    fn log(&self) -> VectorD {
        let xi = dvector![self.xyzw[0], self.xyzw[1], self.xyzw[2]];
        let w = self.xyzw[3];

        let norm_v = xi.norm();
        let two: dtype = (2.0).into();
        // TODO: Broken
        xi * norm_v.atan2(w) / norm_v
    }
}

impl Mul for SO3 {
    type Output = SO3;

    fn mul(self, other: Self) -> SO3 {
        &self * &other
    }
}

impl Mul for &SO3 {
    type Output = SO3;

    fn mul(self, other: Self) -> SO3 {
        let x0 = self.xyzw[0];
        let y0 = self.xyzw[1];
        let z0 = self.xyzw[2];
        let w0 = self.xyzw[3];

        let x1 = other.xyzw[0];
        let y1 = other.xyzw[1];
        let z1 = other.xyzw[2];
        let w1 = other.xyzw[3];

        // Compute the product of the two quaternions, term by term
        let mut xyzw = Vector4::zeros();
        xyzw[0] = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1;
        xyzw[1] = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1;
        xyzw[2] = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1;
        xyzw[3] = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1;

        SO3 { xyzw }
    }
}

impl fmt::Display for SO3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "SO3({:.3}, {:.3}, {:.3}, {:.3})",
            self.xyzw[0], self.xyzw[1], self.xyzw[2], self.xyzw[3]
        )
    }
}

impl fmt::Debug for SO3 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}
