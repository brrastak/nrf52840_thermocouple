
flash: build
	cargo flash --release --chip nRF52840_xxAA

build:
	cargo build --release --target=thumbv7em-none-eabihf
	cargo size --release

test:
	cargo test --lib --target=x86_64-unknown-linux-gnu

size:
	cargo size --release

rtt:
	cargo embed --release

bloat:
	cargo bloat --release --crates --split-std
