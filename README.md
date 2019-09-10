# SlidingMatrixSolver

Solver of [SlidingMatrix](https://github.com/eehyqx/SlidingMatrix/).

## Usage

Rust nightly is needed!

```bash
cargo build --release
cargo run --release -- $n < testcase/input$n.txt
```

## TODO

- Modify next steps: allow adjacent row/col moves instead of interleaving ones.
- Parallelize the search function.
