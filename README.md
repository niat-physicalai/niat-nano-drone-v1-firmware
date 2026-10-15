# NIAT Nano-Drone V1 Firmware

![build](https://github.com/niat-physicalai/niat-nano-drone-v1-firmware/actions/workflows/build.yml/badge.svg)

STM32F405RGT6 firmware for the NIAT Nano-Drone V1.

## Scope (October 2026)
Repository and build system, sensor drivers (IMU, barometer, PMW3901, OPT3101),
all-sensor EKF-friendly logging, interrupt-driven execution skeleton with timing report.

Out of scope: controller integration (F5/F6), radio link firmware, PCB layout, POC flights.

## Conventions
- SI units; body frame FLU; takeoff-aligned world frame; Hamilton quaternions (body to world)
- 1 MHz hardware timer extended to 64-bit microseconds
- float32 only (`-Wdouble-promotion`), no dynamic allocation after start-up
- DMA buffers never in CCM RAM
- Drivers live in L2, output only structs, never call each other

## Repository layout
| Folder | Purpose |
|---|---|
| `L0_hal` to `L7_app` | Software layers |
| `interfaces/` | Shared structs, units and frame definitions |
| `app/sensor_tests/` | Ready-to-flash sensor test programs |
| `tests/pc/` | Host-side unit tests |
| `tools/log_parser/` | UART log parser |
| `docs/` | Architecture, hardware, test cases, decisions, reports |

## Status
Work in progress. See the GitHub Issues board for KPI status.
