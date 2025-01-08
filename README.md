# fact.rs

[![minimum rustc 1.81](https://img.shields.io/badge/rustc-1.81+-red.svg)](https://rust-lang.github.io/rfcs/2495-min-rust-version.html)
[![crate](https://img.shields.io/crates/v/factrs.svg)](https://crates.io/crates/factrs)
[![doc](https://docs.rs/factrs/badge.svg)](https://docs.rs/factrs)
[![ci](https://github.com/rpl-cmu/factrs/actions/workflows/ci.yml/badge.svg)](https://github.com/rpl-cmu/factrs/actions/workflows/ci.yml)

fact.rs (pronounced factors) is a nonlinear least squares optimization library over factor graphs written in Rust.

It is specifically geared toward sensor fusion in robotics. It aims to be fast, easy to use, and safe. The fact.rs API takes heavy inspiration from the [gtsam library](https://gtsam.org/).

Currently, it supports the following features
- Gauss-Newton & Levenberg-Marquadt Optimizers
- Common Lie Groups supported (SO2, SO3, SE2, SE3) with optimization in Lie
  Algebras
- Automatic differentiation via dual numbers
- Serialization of graphs & variables via optional serde support
- Easy conversion to rerun types for straightforward visualization

We recommend you checkout the [docs](https://docs.rs/factrs/latest/factrs/) for more info. For usage, simply add factrs to your `Cargo.toml` and start using it!

# Examples
There's a number of examples found in the [examples](/examples/) folder, including loading g20 files, serialization, and custom factors.

To run a simple pose graph optimization, simply clone this repository and run,
```bash
cargo run --release --example g20 ./examples/data/M3500.g20
```
to visualize the optimization steps with [rerun](https://rerun.io) simply add `--features rerun` to the above command.

Running the other examples can be done similarly,
```bash
cargo run --release --example gps
cargo run --release --example serde --features serde
``` 

Additionally, we recommend checking out the [tests](/tests/) folder for more examples of how to make custom noise models, residuals, robust kernels, and variables.

<details>
<summary>Full Example</summary>

```rust
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Huber, PriorResidual, Values, SO2},
    fac,
    traits::*,
};

// Assign symbols to variable types
assign_symbols!(X: SO2);

fn main() {
    // Make all the values
    let mut values = Values::new();

    let x = SO2::from_theta(1.0);
    let y = SO2::from_theta(2.0);
    values.insert(X(0), SO2::identity());
    values.insert(X(1), SO2::identity());

    // Make the factors & insert into graph
    let mut graph = Graph::new();
    let res = PriorResidual::new(x.clone());
    let factor = fac![res, X(0)];
    graph.add_factor(factor);

    let res = BetweenResidual::new(y.minus(&x));
    let factor = fac![res, (X(0), X(1)), 0.1 as std, Huber::default()];
    graph.add_factor(factor);

    // Optimize!
    let mut opt: GaussNewton = GaussNewton::new(graph);
    let result = opt.optimize(values).unwrap();
    println!("Results {:#}", result);
}
```
</details>
</br>

# Compile-time Errors

fact.rs leans into the Rust way of doing things, and attempts to compile-time error as much as possible. This includes the following,
- Symbols are assigned to variables at compile-time, ensuring that symbols are can not be mismatched
- Enforcing the correct number of keys for a factor
- Ensuring that noise model dimensions match the residual dimensions

A few examples,
```rust
use factrs::core::{assign_symbols, fac, PriorResidual, Values, VectorVar2, SO2};

// Assign symbols to variable types
assign_symbols(X: SO2, Y: SO2);
let mut values = Values::new();

// Proper usage
let id = SO2::identity();
values.insert(X(0), id);
let prior = PriorResidual::new(id);
let f = fac![prior, X(0), (0.1, 0.2) as std];

// These will all compile-time error
// mismatched symbol-variable types
values.insert(X(5), VectorVar2::identity());
// wrong number of keys 
let f = fac![PriorResidual::new(id), (X(0), X(1))]; 
// wrong noise-model dimension
let n = GaussianNoise::<5>::from_scalar_sigma(0.1);
let f = fac![PriorResidual::new(id), X(0), n];
// mismatched symbol-variable types
let f = fac![PriorResidual::new(id), Y(0), 0.1 as std];
```

# Benchmarks
Performance-wise, factrs is competitive with alternative libraries. Benchmarks were ran on a 12th Gen Intel i9 and are all single-threaded (for now). Current benchmarks include [gtsam](https://github.com/borglab/gtsam/) and [tiny-solver-rs](https://github.com/powei-lin/tiny-solver-rs) and data can be found in the [examples/data](/examples/data) folder.

### 2D Benchmarks
| benchmark   | args  | fastest   | median    | mean      |
|-------------|-------|-----------|-----------|-----------|
| factrs      | M3500 | 81.23 ms  | 82.13 ms  | 82.80 ms  |
| gtsam       | M3500 | 160.00 ms | 161.13 ms | 161.14 ms |
| tiny-solver | M3500 | 125.13 ms | 130.46 ms | 132.08 ms |


### 3D Benchmarks
| benchmark   | args           | fastest   | median    | mean      |
|-------------|----------------|-----------|-----------|-----------|
| factrs      | sphere2500     | 352.97 ms | 355.01 ms | 355.14 ms |
| gtsam       | sphere2500     | 389.81 ms | 395.16 ms | 396.65 ms |
| tiny-solver | sphere2500     | 600.80 ms | 615.90 ms | 616.75 ms |
| factrs      | parking-garage | 292.14 ms | 294.01 ms | 294.10 ms |
| gtsam       | parking-garage | 113.24 ms | 114.74 ms | 114.45 ms |
| tiny-solver | parking-garage | 329.48 ms | 334.78 ms | 335.17 ms |

*Note, gtsam is significantly faster for the parking garage due to leveraging the sparsity of the pose graph better using the Baye's tree, something that is planned for factrs.*

To run the rust benchmarks after cloning, simply run,
```bash
cargo bench -p factrs-bench
```
and the C++ benchmarks can be run with,
```bash
cmake -B build factrs-bench/cpp
cmake --build build
./build/bench
```

both of which have alias commands in the root justfile.

There is still some benchmarking work to be done and we'd love some help if you'd like an easy way to contribute! There's a few libraries that could be added, specifically ceres and sophus-rs. Additionally, it'd be nice if all benchmarks had a rust frontend using FFI for easier running - this was begun in the `easton/benches` branch.

# Installation
Simply add via cargo as you do any rust dependency,
```bash
cargo add factrs
```
</details>


# Contributions

Contributions are more than welcome! Feel free to open an issue or a pull request with any ideas, bugs, features, etc you might have or want. 

We feel rust and robotics are a good match and want to see rust robotics libraries catch-up to their C++ counterparts.