use crate::dtype;

/// Error types for optimizers
#[derive(Debug)]
pub enum OptError<Input> {
    MaxIterations(Input),
    InvalidSystem,
    FailedToStep,
}

/// Result type for optimizers
pub type OptResult<Input> = Result<Input, OptError<Input>>;

// ------------------------- Optimizer Params ------------------------- //
/// Parameters for the optimizer
#[derive(Debug, Clone)]
pub struct OptParams {
    pub max_iterations: usize,
    pub error_tol_relative: dtype,
    pub error_tol_absolute: dtype,
    pub error_tol: dtype,
}

impl Default for OptParams {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            error_tol_relative: 1e-6,
            error_tol_absolute: 1e-6,
            error_tol: 0.0,
        }
    }
}

// ------------------------- Optimizer Observers ------------------------- //
/// Observer trait for optimization
///
/// This trait is used to observe the optimization process. It is called at each
/// step of the optimization process.
pub trait OptObserver {
    type Input;
    fn on_step(&self, values: &Self::Input, time: f64);
}

/// Observer collection for optimization
///
/// This struct holds a collection of observers for optimization. It is used to
/// notify all observers at each step of the optimization process.
pub struct OptObserverVec<I> {
    observers: Vec<Box<dyn OptObserver<Input = I>>>,
}

impl<I> OptObserverVec<I> {
    pub fn add(&mut self, callback: impl OptObserver<Input = I> + 'static) {
        let boxed = Box::new(callback);
        self.observers.push(boxed);
    }

    pub fn notify(&self, values: &I, idx: usize) {
        for callback in &self.observers {
            callback.on_step(values, idx as f64);
        }
    }
}

impl<I> Default for OptObserverVec<I> {
    fn default() -> Self {
        Self {
            observers: Vec::new(),
        }
    }
}

// ------------------------- Actual Trait Impl ------------------------- //
/// Trait for optimization algorithms
///
/// This trait is used to define the core optimization functions for an
/// optimizer, specifically a handful of stopping criteria and the main loop.
pub trait Optimizer {
    /// Values the optimizer is optimizing
    type Input;

    /// Parameters for the optimizer
    fn params(&self) -> &OptParams;

    /// Perform a single step of optimization
    fn step(&mut self, values: Self::Input, idx: usize) -> OptResult<Self::Input>;

    /// Compute the error of the current values
    fn error(&self, values: &Self::Input) -> dtype;

    /// Initialize the optimizer, optional
    fn init(&mut self, _values: &Self::Input) {}

    // TODO: Custom logging based on optimizer
    /// Main optimization call function
    fn optimize(&mut self, mut values: Self::Input) -> OptResult<Self::Input> {
        // Setup up everything from our values
        self.init(&values);

        // Check if we need to optimize at all
        let mut error_old = self.error(&values);
        if error_old <= self.params().error_tol {
            log::info!("Error is already below tolerance, skipping optimization");
            return Ok(values);
        }

        log::info!(
            "{:^5} | {:^12} | {:^12} | {:^12}",
            "Iter",
            "Error",
            "ErrorAbs",
            "ErrorRel"
        );
        log::info!(
            "{:^5} | {:^12} | {:^12} | {:^12}",
            "-----",
            "------------",
            "------------",
            "------------"
        );
        log::info!(
            "{:^5} | {:^12.4e} | {:^12} | {:^12}",
            0,
            error_old,
            "-",
            "-"
        );

        // Begin iterations
        let mut error_new = error_old;
        for i in 1..self.params().max_iterations + 1 {
            error_old = error_new;
            values = self.step(values, i)?;

            // Evaluate error again to see how we did
            error_new = self.error(&values);

            let error_decrease_abs = error_old - error_new;
            let error_decrease_rel = error_decrease_abs / error_old;

            log::info!(
                "{:^5} | {:^12.4e} | {:^12.4e} | {:^12.4e}",
                i,
                error_new,
                error_decrease_abs,
                error_decrease_rel
            );

            // Check if we need to stop
            if error_new <= self.params().error_tol {
                log::info!("Error is below tolerance, stopping optimization");
                return Ok(values);
            }
            if error_decrease_abs <= self.params().error_tol_absolute {
                log::info!("Error decrease is below absolute tolerance, stopping optimization");
                return Ok(values);
            }
            if error_decrease_rel <= self.params().error_tol_relative {
                log::info!("Error decrease is below relative tolerance, stopping optimization");
                return Ok(values);
            }
        }

        Err(OptError::MaxIterations(values))
    }
}
