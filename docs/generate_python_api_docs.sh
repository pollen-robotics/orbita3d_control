echo "Generating API documentation..."
rm -r docs/api/
RUSTDOCFLAGS="--enable-index-page -Zunstable-options" cargo +nightly doc --no-deps --lib
RUSTDOCFLAGS="--enable-index-page -Zunstable-options" cargo +nightly doc --no-deps --package poulpe_ethercat_controller
cp -r target/doc/ docs/api

# pdoc 
cd orbita3d_c_api/python
pdoc orbita3d -d markdown -o ../../docs/api/orbita3d_c_api/python --logo ../../../img/favicon.ico --footer-text "Pollen Robotics 2025"