use crate::dtype;
use crate::linalg::{DualNum, DualVec, Matrix3, Matrix4, MatrixX, Vector3, VectorViewX, VectorX};
use crate::variables::{LieGroup, Variable, SO3};
use std::fmt;
use std::ops;

#[derive(Clone, Debug)]
pub struct SE3<D: DualNum = dtype> {
    rot: SO3<D>,
    xyz: Vector3<D>,
}

impl<D: DualNum> SE3<D> {
    pub fn to_matrix(&self) -> Matrix4<D> {
        let mut mat = Matrix4::<D>::identity();
        mat.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.rot.to_matrix());
        mat.fixed_view_mut::<3, 1>(0, 3).copy_from(&self.xyz);
        mat
    }

    pub fn from_matrix(mat: &Matrix4<D>) -> Self {
        let rot = mat.fixed_view::<3, 3>(0, 0).into();
        let rot = SO3::from_matrix(&rot);

        let xyz = mat.fixed_view::<3, 1>(0, 3).into();

        SE3 { rot, xyz }
    }

    pub fn apply(&self, v: &Vector3<D>) -> Vector3<D> {
        self.rot.apply(v) + self.xyz.clone()
    }
}

impl<D: DualNum> Variable<D> for SE3<D> {
    const DIM: usize = 6;
    type Dual = SE3<DualVec>;

    fn identity() -> Self {
        SE3 {
            rot: Variable::identity(),
            xyz: Variable::identity(),
        }
    }

    fn compose(&self, other: &Self) -> Self {
        SE3 {
            rot: &self.rot * &other.rot,
            xyz: self.rot.apply(&other.xyz) + self.xyz.clone(),
        }
    }

    fn inverse(&self) -> Self {
        let inv = self.rot.inverse();
        SE3 {
            xyz: -&inv.apply(&self.xyz),
            rot: inv,
        }
    }

    #[allow(non_snake_case)]
    fn exp(xi: VectorViewX<D>) -> Self {
        let xi_rot = xi.rows(0, 3);
        let xyz = Vector3::new(xi[3].clone(), xi[4].clone(), xi[5].clone());

        let w2 = xi_rot.norm_squared();

        let B;
        let C;
        if w2.clone() < D::from(1e-5) {
            B = D::from(0.5);
            C = D::from(1.0 / 6.0);
        } else {
            let w = w2.clone().sqrt();
            let A = w.clone().sin() / w.clone();
            B = (D::from(1.0) - w.clone().cos()) / w2.clone();
            C = (D::from(1.0) - A) / w2.clone();
        };
        let I = Matrix3::identity();
        let wx = SO3::hat(xi_rot);
        let V = I + &wx * B + &wx * &wx * C;

        let rot = SO3::<D>::exp(xi.rows(0, 3));

        SE3 { rot, xyz: V * xyz }
    }

    #[allow(non_snake_case)]
    fn log(&self) -> VectorX<D> {
        let mut xi = VectorX::zeros(6);
        let xi_theta = self.rot.log();

        let w2 = xi_theta.norm_squared();

        let B;
        let C;
        if w2.clone() < D::from(1e-5) {
            B = D::from(0.5);
            C = D::from(1.0 / 6.0);
        } else {
            let w = w2.clone().sqrt();
            let A = w.clone().sin() / w.clone();
            B = (D::from(1.0) - w.clone().cos()) / w2.clone();
            C = (D::from(1.0) - A) / w2.clone();
        };

        let I = Matrix3::identity();
        let wx = SO3::hat(xi_theta.as_view());
        let V = I + &wx * B + &wx * &wx * C;

        let Vinv = V.try_inverse().expect("V is not invertible");
        let xyz = &Vinv * &self.xyz;

        xi.as_mut_slice()[0..3].clone_from_slice(xi_theta.as_slice());
        xi.as_mut_slice()[3..6].clone_from_slice(xyz.as_slice());

        xi
    }

    fn dual_self(&self) -> Self::Dual {
        SE3 {
            rot: self.rot.dual_self(),
            xyz: self.xyz.dual_self(),
        }
    }
}

impl<D: DualNum> LieGroup<D> for SE3<D> {
    fn hat(xi: VectorViewX<D>) -> MatrixX<D> {
        let mut mat = MatrixX::<D>::zeros(4, 4);
        mat[(0, 1)] = -xi[2].clone();
        mat[(0, 2)] = xi[1].clone();
        mat[(1, 0)] = xi[2].clone();
        mat[(1, 2)] = -xi[0].clone();
        mat[(2, 0)] = -xi[1].clone();
        mat[(2, 1)] = xi[0].clone();

        mat[(0, 3)] = xi[3].clone();
        mat[(1, 3)] = xi[4].clone();
        mat[(2, 3)] = xi[5].clone();

        mat
    }

    fn adjoint(&self) -> MatrixX<D> {
        let mut mat = MatrixX::<D>::zeros(Self::DIM, Self::DIM);
        let xyz = self.xyz.as_view();

        let r_mat = self.rot.to_matrix();
        let t_r_mat = &SO3::hat(xyz) * &r_mat;

        mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&r_mat);
        mat.fixed_view_mut::<3, 3>(3, 3).copy_from(&r_mat);
        mat.fixed_view_mut::<3, 3>(3, 0).copy_from(&t_r_mat);

        mat
    }
}

impl<D: DualNum> ops::Mul for SE3<D> {
    type Output = SE3<D>;

    fn mul(self, other: Self) -> Self::Output {
        self.compose(&other)
    }
}

impl<D: DualNum> ops::Mul for &SE3<D> {
    type Output = SE3<D>;

    fn mul(self, other: Self) -> Self::Output {
        self.compose(other)
    }
}

impl<D: DualNum> fmt::Display for SE3<D> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} {:?}", self.rot, self.xyz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linalg::{dvector, Diff, DiffResult, DualNum, ForwardProp, Matrix3x4, Matrix4x6};
    use matrixcompare::assert_matrix_eq;

    #[test]
    fn matrix() {
        // to_matrix -> from_matrix shoudl give back original vector
        let xi = dvector![0.1, 0.2, 0.3, 1.0, 2.0, 3.0];
        let se3 = SE3::exp(xi.as_view());
        let mat = se3.to_matrix();

        let se3_hat = SE3::from_matrix(&mat);

        assert_matrix_eq!(se3.ominus(&se3_hat), VectorX::zeros(6), comp = float);
    }

    #[test]
    fn jacobian() {
        fn rotate<D: DualNum>(r: SE3<D>) -> VectorX<D> {
            let v = Vector3::new(D::from(1.0), D::from(2.0), D::from(3.0));
            let rotated = r.apply(&v);
            dvector![rotated[0].clone(), rotated[1].clone(), rotated[2].clone()]
        }

        let t = SE3::exp(dvector![0.1, 0.2, 0.3, 0.4, 0.5, 0.6].as_view());
        let DiffResult {
            value: _x,
            diff: dx,
        } = ForwardProp::jacobian_1(rotate, &t);

        let dropper: Matrix3x4 = Matrix3x4::identity();
        let v = dvector!(1.0, 2.0, 3.0);
        let mut jac = Matrix4x6::zeros();
        jac.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&SO3::hat((-v).as_view()));
        jac.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&Matrix3::identity());

        #[cfg(not(feature = "left"))]
        let dx_exp = dropper * t.to_matrix() * jac;
        // TODO: Verify left jacobian
        #[cfg(feature = "left")]
        let dx_exp = dropper * t.to_matrix() * t.inverse().adjoint() * jac;

        println!("Expected: {}", dx_exp);
        println!("Actual: {}", dx);

        assert_matrix_eq!(dx, dx_exp, comp = float);
    }
}
