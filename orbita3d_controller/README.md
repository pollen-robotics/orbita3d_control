
# For Orbita3d testing

## Assuming Orbita3d firmware is flashed and configured with a zero

- Start the Ethercat master
- Start `poulpe_ethercat_controller` server TODO

- Install `poulpe_ethercat_grpc` (match or configure the path in Cargo.toml)
- `cargo install binstall`
- `cargo binstall rerun-cli`