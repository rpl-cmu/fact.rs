use std::collections::hash_map::Iter as HashMapIter;

use crate::{
    containers::{Idx, Key, Symbol, Values, ValuesOrder},
    linalg::{VectorViewX, VectorX},
};

/// Structure to store linear (aka all vector) values
///
/// This structure is the linear equivalent of [Values]. It stores all values in
/// a single vector, along with indices and dimensions of each variable. Ideally
/// *shouldn't* ever be needed in practice.
pub struct LinearValues {
    values: VectorX,
    order: ValuesOrder,
}

impl LinearValues {
    /// Create a zero/identity LinearValues from a [ValuesOrder]
    pub fn zero_from_order(order: ValuesOrder) -> Self {
        let values = VectorX::zeros(order.dim());
        Self { values, order }
    }

    /// Create a zero/identity LinearValues from a [Values]
    ///
    /// The order is inferred from the values
    pub fn zero_from_values(values: &Values) -> Self {
        let order = ValuesOrder::from_values(values);
        let values = VectorX::zeros(order.dim());
        Self { values, order }
    }

    /// Create a LinearValues from a [ValuesOrder] and a vector
    pub fn from_order_and_vector(order: ValuesOrder, values: VectorX) -> Self {
        assert!(
            values.len() == order.dim(),
            "Vector and order must have the same dimension when creating LinearValues"
        );
        Self { values, order }
    }

    /// Create a LinearValues from a [Values] and a vector
    ///
    /// The order is inferred from the values
    pub fn from_values_and_vector(values: &Values, vector: VectorX) -> Self {
        let order = ValuesOrder::from_values(values);
        assert!(
            vector.len() == order.dim(),
            "Vector and values must have the same dimension when creating LinearValues"
        );
        Self::from_order_and_vector(order, vector)
    }

    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    pub fn dim(&self) -> usize {
        self.values.len()
    }

    fn get_idx(&self, idx: &Idx) -> VectorViewX<'_> {
        self.values.rows(idx.idx, idx.dim)
    }

    /// Retrieve a vector from the LinearValues
    pub fn get(&self, key: impl Symbol) -> Option<VectorViewX<'_>> {
        let idx = self.order.get(key)?;
        self.get_idx(idx).into()
    }

    pub fn iter(&self) -> Iter<'_> {
        Iter {
            values: self,
            idx: self.order.iter(),
        }
    }
}

pub struct Iter<'a> {
    values: &'a LinearValues,
    idx: HashMapIter<'a, Key, Idx>,
}

impl<'a> Iterator for Iter<'a> {
    type Item = (&'a Key, VectorViewX<'a>);

    fn next(&mut self) -> Option<Self::Item> {
        let n = self.idx.next()?;
        Some((n.0, self.values.get_idx(n.1)))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{
        dtype,
        symbols::X,
        variables::{Variable, VectorVar2, VectorVar3, VectorVar6},
    };

    fn make_order_vector() -> (ValuesOrder, VectorX) {
        // Create some form of values
        let mut v = Values::new();
        v.insert_unchecked(X(0), VectorVar2::identity());
        v.insert_unchecked(X(1), VectorVar6::identity());
        v.insert_unchecked(X(2), VectorVar3::identity());

        // Create an order
        let order = ValuesOrder::from_values(&v);
        let vector = VectorX::from_fn(order.dim(), |i, _| i as dtype);
        (order, vector)
    }

    #[test]
    fn from_order_and_vector() {
        let (order, vector) = make_order_vector();

        // Create LinearValues
        let linear_values = LinearValues::from_order_and_vector(order, vector);
        assert!(linear_values.len() == 3);
        assert!(linear_values.dim() == 11);
        assert!(linear_values.get(X(0)).expect("Key was missing").len() == 2);
        assert!(linear_values.get(X(1)).expect("Key was missing").len() == 6);
        assert!(linear_values.get(X(2)).expect("Key was missing").len() == 3);
        assert!(linear_values.get(X(3)).is_none());
    }

    #[test]
    #[should_panic]
    fn mismatched_size() {
        let (order, vector) = make_order_vector();
        let vector = vector.push(0.0);
        LinearValues::from_order_and_vector(order, vector);
    }
}
