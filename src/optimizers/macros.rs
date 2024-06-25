#[macro_export]
macro_rules! test_optimizer {
    ($optimizer:ident $(<$($gen:ident),* >)?) => {
        #[test]
        fn priorvector3() {
            $crate::optimizers::test::optimize_prior::<$optimizer$(< $($gen),* >)?, $crate::variables::Vector3>()
        }

        #[test]
        fn priorso3() {
            $crate::optimizers::test::optimize_prior::<$optimizer$(< $($gen),* >)?, $crate::variables::SO3>();
        }

        #[test]
        fn betweenvector3() {
            $crate::optimizers::test::optimize_between::<$optimizer$(< $($gen),* >)?, $crate::variables::Vector3>();
        }

        #[test]
        fn betweenso3() {
            $crate::optimizers::test::optimize_between::<$optimizer$(< $($gen),* >)?, $crate::variables::SO3>();
        }
    };
}
