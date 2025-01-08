use factrs::{core::SE3, dtype, linalg::Vector6, traits::*};
use nalgebra::Const;

fn main() {
    let xi = Vector6::new(0.1, 0.2, 0.3, 1.0, 2.0, 3.0);
    let se3: SE3 = SE3::exp(xi.as_view());
    let dual = se3.dual::<Const<6>>(0);

    let dual_exp = SE3::<dtype>::dual_exp::<Const<6>>(0);

    let by_hand = se3.cast() * dual_exp.clone();

    println!("SE3: {:#.5?}", se3);
    println!("dual_exp: {:#.5?}\n", dual_exp);

    println!("dual: {:#.5?}", dual);
    println!("by_hand: {:#.5?}\n", by_hand);

    // rotation
    println!("mine_x: {:.5?}", dual.rot().x());
    println!("by_hand_x: {:.5?}\n", by_hand.rot().x());
    assert_eq!(dual.rot().x().re, by_hand.rot().x().re);
    matrixcompare::assert_matrix_eq!(
        dual.rot().x().eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.rot().x().eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    println!("mine_y: {:.5?}", dual.rot().y());
    println!("by_hand_y: {:.5?}\n", by_hand.rot().y());
    assert_eq!(dual.rot().y().re, by_hand.rot().y().re);
    matrixcompare::assert_matrix_eq!(
        dual.rot().y().eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.rot().y().eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    println!("mine_z: {:.5?}", dual.rot().z());
    println!("by_hand_z: {:.5?}\n", by_hand.rot().z());
    assert_eq!(dual.rot().z().re, by_hand.rot().z().re);
    matrixcompare::assert_matrix_eq!(
        dual.rot().z().eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.rot().z().eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    println!("mine_w: {:.5?}", dual.rot().w());
    println!("by_hand_w: {:.5?}\n", by_hand.rot().w());
    assert_eq!(dual.rot().w().re, by_hand.rot().w().re);
    matrixcompare::assert_matrix_eq!(
        dual.rot().w().eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.rot().w().eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    // translation
    println!("mine_x: {:.5?}", dual.xyz().x);
    println!("by_hand_x: {:.5?}\n", by_hand.xyz().x);
    assert_eq!(dual.xyz().x.re, by_hand.xyz().x.re);
    matrixcompare::assert_matrix_eq!(
        dual.xyz().x.eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.xyz().x.eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    println!("mine_y: {:.5?}", dual.xyz().y);
    println!("by_hand_y: {:.5?}\n", by_hand.xyz().y);
    assert_eq!(dual.xyz().y.re, by_hand.xyz().y.re);
    matrixcompare::assert_matrix_eq!(
        dual.xyz().y.eps.unwrap_generic(Const::<6>, Const::<1>),
        by_hand.xyz().y.eps.unwrap_generic(Const::<6>, Const::<1>),
        comp = abs,
        tol = 1e-6
    );

    // println!("mine_z: {:.5?}", dual.xyz().z);
    // println!("by_hand_z: {:.5?}\n", by_hand.xyz().z);
    // assert_eq!(dual.xyz().z.re, by_hand.xyz().z.re);
    // matrixcompare::assert_matrix_eq!(
    //     dual.xyz().z.eps.unwrap_generic(Const::<6>, Const::<1>),
    //     by_hand.xyz().z.eps.unwrap_generic(Const::<6>, Const::<1>)
    // );
}
