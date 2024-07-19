use std::ops::Mul;

use faer::{scale, sparse::SparseColMat};
use faer_ext::IntoNalgebra;

use super::{GraphOptimizer, OptError, OptObserverVec, OptParams, OptResult, Optimizer};
use crate::{
    containers::{Graph, GraphOrder, Values, ValuesOrder},
    dtype,
    linalg::DiffResult,
    linear::{CholeskySolver, LinearSolver, LinearValues},
};

pub struct LevenParams {
    pub lambda_min: dtype,
    pub lambda_max: dtype,
    pub lambda_factor: dtype,
    pub diagonal_damping: bool,
}

impl Default for LevenParams {
    fn default() -> Self {
        Self {
            lambda_min: 0.0,
            lambda_max: 1e5,
            lambda_factor: 10.0,
            diagonal_damping: true,
        }
    }
}

pub struct LevenMarquardt<S: LinearSolver = CholeskySolver> {
    graph: Graph,
    solver: S,
    pub params_base: OptParams,
    pub params_leven: LevenParams,
    observers: OptObserverVec<Values>,
    lambda: dtype,
    // For caching computation between steps
    graph_order: Option<GraphOrder>,
}

impl<S: LinearSolver> GraphOptimizer for LevenMarquardt<S> {
    fn new(graph: Graph) -> Self {
        Self {
            graph,
            solver: S::default(),
            params_base: OptParams::default(),
            params_leven: LevenParams::default(),
            observers: OptObserverVec::default(),
            lambda: 1e-5,
            graph_order: None,
        }
    }

    fn graph(&self) -> &Graph {
        &self.graph
    }
}

impl<S: LinearSolver> Optimizer for LevenMarquardt<S> {
    type Input = Values;

    fn params(&self) -> &OptParams {
        &self.params_base
    }

    fn error(&self, values: &Values) -> crate::dtype {
        self.graph.error(values)
    }

    fn init(&mut self, _values: &Values) {
        // TODO: Some way to manual specify how to computer ValuesOrder
        // Precompute the sparsity pattern
        self.graph_order = Some(
            self.graph
                .sparsity_pattern(ValuesOrder::from_values(_values)),
        );
    }

    // TODO: Some form of logging of the lambda value
    // TODO: More sophisticated stopping criteria based on magnitude of the gradient
    fn step(&mut self, mut values: Values, idx: usize) -> OptResult<Values> {
        // Make an ordering
        let order = ValuesOrder::from_values(&values);

        // Solve the linear system
        let linear_graph = self.graph.linearize(&values);
        let DiffResult { value: r, diff: j } =
            linear_graph.residual_jacobian(self.graph_order.as_ref().unwrap());

        // Form A
        let jtj = j
            .as_ref()
            .transpose()
            .to_col_major()
            .unwrap()
            .mul(j.as_ref());

        // Form I
        let triplets_i = if self.params_leven.diagonal_damping {
            (0..jtj.ncols())
                .map(|i| (i as isize, i as isize, jtj[(i, i)]))
                .collect::<Vec<_>>()
        } else {
            (0..jtj.ncols())
                .map(|i| (i as isize, i as isize, 1.0))
                .collect::<Vec<_>>()
        };
        let i = SparseColMat::<usize, dtype>::try_new_from_nonnegative_triplets(
            jtj.ncols(),
            jtj.ncols(),
            &triplets_i,
        )
        .unwrap();

        // Form b
        let b = j.as_ref().transpose().mul(&r);

        let mut dx = LinearValues::zero_from_order(order.clone());
        let old_error = linear_graph.error(&dx);

        loop {
            // Make Ax = b
            let a = &jtj + (&i * scale(self.lambda));

            // Solve Ax = b
            let delta = self
                .solver
                .solve_symmetric(a.as_ref(), b.as_ref())
                .as_ref()
                .into_nalgebra()
                .column(0)
                .clone_owned();
            dx = LinearValues::from_order_and_vector(
                self.graph_order.as_ref().unwrap().order.clone(),
                delta,
            );

            // Update our cost
            let curr_error = linear_graph.error(&dx);

            if curr_error < old_error {
                break;
            }

            self.lambda *= self.params_leven.lambda_factor;
            if self.lambda > self.params_leven.lambda_max {
                return Err(OptError::FailedToStep);
            }
        }

        // Update the values
        values.oplus_mut(&dx);
        self.lambda /= self.params_leven.lambda_factor;
        if self.lambda < self.params_leven.lambda_min {
            self.lambda = self.params_leven.lambda_min;
        }

        self.observers.notify(&values, idx);

        Ok(values)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_optimizer;

    test_optimizer!(LevenMarquardt);
}
