
## cargo bench

within `bevy_rapier3d`:

```sh
cargo bench
```

## Custom benches

```sh
cargo run --release -p custom_benches
```

## Flamegraph

```sh
CARGO_PROFILE_RELEASE_DEBUG=true cargo flamegraph
```
