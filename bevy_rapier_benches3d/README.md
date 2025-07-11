# bevy_rapier custom benches

`bevy_rapier_benches3d` 's objective is to measure timings with detailed information
without spending too much time running multiple times expensive benchmarks.

It is implemented as a standalone binary, running different scenes setup, gathering information
and outputs them at the end.

```sh
cargo run --release --bin bench
```

To visually make sure your benchmarks setup is correct, you can use the feature `visual`,
and edit `bench.rs` to test only the scene you want.

```sh
cargo run --release --bin bench --features visual
```

## cargo bench

For short-lived benchmarks based on statistical analysis,
benchmarks can be run through the [divan](https://github.com/nvzqz/divan) bench harness.

```sh
cargo bench -p bevy_rapier_benches3d
```

## Other resources

- [Bevy profiling](https://github.com/bevyengine/bevy/blob/main/docs/profiling.md)
