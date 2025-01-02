bench-rust:
    cargo bench -p factrs-bench

bench-cpp:
    cmake -B build factrs-bench/cpp
    cmake --build build
    ./build/bench

docs:
    RUSTDOCFLAGS="--html-in-header $PWD/assets/katex-header.html" cargo doc