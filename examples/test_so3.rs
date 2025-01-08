use factrs::{core::SO3, dtype, linalg::Vector3, traits::*};
use nalgebra::Const;

fn main() {
    let xi = Vector3::new(0.1, 0.2, 0.3);
    let so3: SO3 = SO3::exp(xi.as_view());
    let dual = so3.dual::<Const<3>>(0);

    let dual_exp = SO3::<dtype>::dual_exp::<Const<3>>(0);

    let by_hand = so3.cast() * dual_exp.clone();

    println!("so3: {:#.5?}", so3);
    println!("dual_exp: {:#.5?}\n", dual_exp);

    println!("mine_x: {:.5?}", dual.x());
    println!("by_hand_x: {:.5?}\n", by_hand.x());
    assert_eq!(dual.x().re, by_hand.x().re);
    matrixcompare::assert_matrix_eq!(
        dual.x().eps.unwrap_generic(Const::<3>, Const::<1>),
        by_hand.x().eps.unwrap_generic(Const::<3>, Const::<1>)
    );

    println!("mine_y: {:.5?}", dual.y());
    println!("by_hand_y: {:.5?}\n", by_hand.y());
    assert_eq!(dual.y().re, by_hand.y().re);
    matrixcompare::assert_matrix_eq!(
        dual.y().eps.unwrap_generic(Const::<3>, Const::<1>),
        by_hand.y().eps.unwrap_generic(Const::<3>, Const::<1>)
    );

    println!("mine_z: {:.5?}", dual.z());
    println!("by_hand_z: {:.5?}\n", by_hand.z());
    assert_eq!(dual.z().re, by_hand.z().re);
    matrixcompare::assert_matrix_eq!(
        dual.z().eps.unwrap_generic(Const::<3>, Const::<1>),
        by_hand.z().eps.unwrap_generic(Const::<3>, Const::<1>)
    );

    println!("mine_w: {:.5?}", dual.w());
    println!("by_hand_w: {:.5?}\n", by_hand.w());
    assert_eq!(dual.w().re, by_hand.w().re);
    matrixcompare::assert_matrix_eq!(
        dual.w().eps.unwrap_generic(Const::<3>, Const::<1>),
        by_hand.w().eps.unwrap_generic(Const::<3>, Const::<1>)
    );
}
