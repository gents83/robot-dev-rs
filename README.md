# Humanoid Robot Control System

This repository contains a modular robot control system in Rust, designed for safety, async execution, and modularity.

## Architecture

The project is organized as a Rust workspace with the following packages:

*   **brain**: High-level logic and decision-making components. Responsible for planning and state management.
*   **kinematics**: Mathematical foundations for motion, including forward and inverse kinematics.
*   **interface**: Abstraction layer for hardware communication and simulation bridging.

## Building and Running

To build the entire workspace:

```bash
cargo build
```

To run tests:

```bash
cargo test
```

## Contributing

Please ensure all tests pass before submitting changes.
