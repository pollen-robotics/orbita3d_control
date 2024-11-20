
# For Orbita3d testing

## Assuming Orbita3d firmware is flashed and configured with a zero

- Start the Ethercat master
- Start `poulpe_ethercat_controller` server TODO

- Install `poulpe_ethercat_grpc` (match or configure the path in Cargo.toml) FIXME: make it a github dependency
- `cargo install binstall`
- `cargo binstall rerun-cli`

- Run the test program with an input csv file:
  - ie: `cargo run --example=poulpe3d_test -- --configfile=config/fake.yaml --input-csv=config/test_input.csv --viewer`
