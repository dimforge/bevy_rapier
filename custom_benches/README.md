
## cargo bench

```sh
cargo bench -p bevy_rapier3d
```

## Custom benches

```sh
cargo run --release -p custom_benches
```

## Flamegraph

```sh
CARGO_PROFILE_RELEASE_DEBUG=true cargo flamegraph
```
