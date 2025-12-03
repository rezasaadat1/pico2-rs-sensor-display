#!/bin/bash

# This script runs after the devcontainer is created
# Rust is pre-installed in the Docker image, but we need to ensure
# the user's environment is set up correctly

# Check if Rust is already installed system-wide
if command -v rustc &> /dev/null; then
    echo "Rust is already installed system-wide"
    rustc --version
else
    # Install Rust for the user if not present
    echo "Installing Rust for user..."
    curl https://sh.rustup.rs -sSf | sh -s -- -y
    source "$HOME/.cargo/env"
    rustup component add rust-analyzer clippy rustfmt
    rustup target add thumbv8m.main-none-eabihf
    rustup target add thumbv6m-none-eabi
    rustup target add riscv32imac-unknown-none-elf
fi

# Install additional cargo tools if not present
if ! command -v cargo-watch &> /dev/null; then
    cargo install cargo-watch 2>/dev/null || true
fi

if ! command -v cargo-edit &> /dev/null; then
    cargo install cargo-edit 2>/dev/null || true
fi

echo "Development environment ready!"