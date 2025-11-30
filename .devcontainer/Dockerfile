# Use the Rust devcontainer image as base
FROM mcr.microsoft.com/devcontainers/rust:2-1-bookworm

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    build-essential \
    git \
    pkg-config \
    libssl-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

