# Repository Guidelines

## Project Structure & Module Organization

This repository contains the SeekFree i.MX RT1064 open-source library and example projects. Core source lives under `libraries/`:

- `zf_common/`: shared infrastructure such as typedefs, clock, debug, FIFO, and interrupt/vector support.
- `zf_driver/`: MCU peripheral drivers such as GPIO, UART, ADC, PWM, PIT, TIMER, and EXTI.
- `zf_device/`: external device drivers such as displays, cameras, IMUs, wireless modules, and sensors.
- `sdk/` and `components/`: NXP SDK and middleware.
- `Example/`: board demos, usually with `user/inc` and `user/src`, plus IDE projects under `iar/` and `mdk/`.
- `doc/`, `【文档】...`, `【原理图】...`: version notes, documentation, schematics, and related assets.

## Build, Test, and Development Commands

- `git submodule update --init --recursive`: initialize bundled submodules if required.
- Keil MDK: open `Example/**/mdk/*.uvprojx`, then use `Rebuild` and `Download`.
- IAR: open `Example/**/iar/*.eww`, run `Project -> Clean` after moving paths, then `Make`.
- Cleanup helpers:
  - `Example/**/iar/IAR删除临时文件.bat`
  - `Example/**/mdk/MDK删除临时文件.bat`

There are no host-side build or test commands; validation is done on hardware.

## Coding Style & Naming Conventions

- Language: embedded C. Keep existing file headers intact.
- Indentation: follow the surrounding file style, usually 4 spaces.
- Naming: functions use `snake_case` such as `gpio_init`; macros use `UPPER_SNAKE_CASE`.
- Prefer edits in `Example/**/user/` for demo behavior. Only change `zf_common`, `zf_driver`, or `zf_device` when the fix belongs in the shared library.

## Testing Guidelines

- No unit-test framework is present.
- Validate by building and flashing the relevant example on RT1064 hardware.
- Check behavior with LEDs, GPIO signals, UART logs, display output, or debugger watch windows.
- If changing interrupt or pin behavior, document the exact example, board, and wiring used.

## Commit & Pull Request Guidelines

- Follow the existing history: short descriptive commits are common, including version-prefixed messages like `V3.9.2 ...`.
- Keep commit messages specific, for example: `修复 UART 接收 FIFO 溢出判断`.
- PRs should include:
  - affected example or library module
  - toolchain and version (`MDK 5.33`, `IAR 8.32.4`, etc.)
  - reproduction and verification steps
  - wiring or pin-mapping changes
  - screenshots or logs when UI, display, or serial output changed
