# [Choice] Ubuntu version (use jammy on local arm64/Apple Silicon): jammy, focal
FROM ubuntu:jammy

LABEL dev.containers.features="common"
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    cmake make gdb-multiarch git curl ca-certificates \
    build-essential pkg-config libssl-dev libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Rust toolchain
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain stable --profile minimal \
    && rustup component add rust-analyzer clippy rustfmt \
    && rustup target add thumbv8m.main-none-eabihf \
    && rustup target add thumbv6m-none-eabi \
    && rustup target add riscv32imac-unknown-none-elf \
    && cargo install flip-link \
    && cargo install elf2uf2-rs \
    && chmod -R a+w $RUSTUP_HOME $CARGO_HOME
