# ---------------------- General helpers that should work for everyone ---------------------- #
# build and run rust benchmarks
bench-rust:
    cargo bench -p factrs-bench

# build and run cpp benchmarks
bench-cpp:
    cmake -B build factrs-bench/cpp
    cmake --build build
    ./build/bench

# profile the g2o example using flamegraph
profile:
    cargo flamegraph --profile profile --example g2o -- ./examples/data/parking-garage.g2o

# build docs with latex support
doc:
    RUSTDOCFLAGS="--cfg docsrs --html-in-header $PWD/assets/katex-header.html" cargo doc --features="serde rerun" -Zunstable-options -Zrustdoc-scrape-examples

bacon-doc:
    RUSTDOCFLAGS="--cfg docsrs --html-in-header $PWD/assets/katex-header.html" bacon doc --features="serde rerun" -- -Zunstable-options -Zrustdoc-scrape-examples

# ---------------------- Easton specific helpers that work on my system ---------------------- #
# tune the system for benchmarking using pyperf
perf-tune:
    sudo ~/.local/share/uv/tools/pyperf/bin/pyperf system tune

# reset the system after benchmarking using pyperf
perf-reset:
    sudo ~/.local/share/uv/tools/pyperf/bin/pyperf system reset
