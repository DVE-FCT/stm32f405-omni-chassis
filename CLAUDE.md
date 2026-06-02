# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F405RGTx bare-metal firmware project — currently a fresh STM32CubeMX skeleton with no application logic yet. CMake + GCC (arm-none-eabi) toolchain, Ninja build generator.

## Hardware Configuration

- **MCU**: STM32F405RGTx (LQFP64), 168 MHz Cortex-M4 with FPU (hard-float ABI)
- **Clock**: **HSE 25 MHz** → PLLM=25, PLLN=336, PLLP=2 → SYSCLK 168 MHz, APB1=42 MHz (÷4), APB2=84 MHz (÷2)
- **Memory**: 1024 KB Flash, 128 KB SRAM + 64 KB CCM RAM (CCM at 0x10000000)
- **Currently configured peripherals**: GPIO only (PA13=SWDIO, PA14=SWCLK for debug; PH0/PH1 for HSE oscillator)
- **No UART, I2C, SPI, timers, or other peripherals configured yet** — add them via CubeMX as needed

## Build & Flash

### Prerequisites

- `arm-none-eabi-gcc` must be on your PATH
- CMake ≥ 3.22
- Ninja (or the CMake preset can be modified for another generator)

### Build commands

```bash
# Configure (Debug)
cmake --preset Debug

# Build
cmake --build build/Debug

# Or configure + build Release
cmake --preset Release
cmake --build build/Release
```

Build artifacts:
- `build/<config>/stm_code.elf` — executable
- `build/<config>/stm_code.map` — linker map
- `build/<config>/compile_commands.json` — for clangd/LSP

### Flash

Flash is not configured in CMake. Use your preferred STM32 programmer (ST-LINK, J-Link, etc.) with the `.elf` or convert to `.bin`/`.hex` via `arm-none-eabi-objcopy`.

## CubeMX Code Generation

- **`.ioc` file**: `stm_code.ioc` (CubeMX v6.17.0, FW_F4 V1.28.3)
- **Toolchain**: CMake (not Keil/SW4STM32)
- **`ProjectManager.KeepUserCode=true`** — regenerating code preserves code between `/* USER CODE BEGIN */` / `/* USER CODE END */` markers. All application logic **must** live inside these markers.
- **`SystemClock_Config()` is NOT in a USER CODE block** — regenerating will overwrite it. If you change clock settings in CubeMX, the function in `main.c` will be replaced.

### Pin/project generation note

The `.ioc` file path appears in `.mxproject` with relative paths. When regenerating, CubeMX will update:
- `Core/Src/main.c`, `Core/Src/gpio.c`, `Core/Src/stm32f4xx_it.c`, `Core/Src/stm32f4xx_hal_msp.c`
- `Core/Inc/main.h`, `Core/Inc/gpio.h`, `Core/Inc/stm32f4xx_it.h`, `Core/Inc/stm32f4xx_hal_conf.h`
- `cmake/stm32cubemx/CMakeLists.txt`
- `Drivers/` (HAL + CMSIS files may be refreshed)

## Code Architecture

```
stm_code/
├── Core/
│   ├── Inc/                   # Headers
│   │   ├── main.h             # HAL include + Error_Handler prototype
│   │   ├── gpio.h             # MX_GPIO_Init prototype
│   │   ├── stm32f4xx_it.h     # IRQ handler prototypes
│   │   └── stm32f4xx_hal_conf.h
│   └── Src/                   # Sources
│       ├── main.c             # main(), SystemClock_Config(), Error_Handler()
│       ├── gpio.c             # MX_GPIO_Init() — GPIO port clock enable only
│       ├── stm32f4xx_it.c     # IRQ handlers (SysTick → HAL_IncTick)
│       ├── stm32f4xx_hal_msp.c # HAL_MspInit() — enables SYSCFG + PWR clocks
│       ├── sysmem.c           # _sbrk for newlib heap
│       ├── syscalls.c         # Newlib syscall stubs (empty)
│       └── system_stm32f4xx.c # CMSIS system init
├── Drivers/                   # STM32F4xx HAL + CMSIS
├── cmake/
│   ├── gcc-arm-none-eabi.cmake  # Toolchain definition
│   ├── starm-clang.cmake        # Alternate ST ARM Clang toolchain
│   └── stm32cubemx/CMakeLists.txt  # CubeMX-generated source list + library targets
├── CMakeLists.txt             # Top-level CMake project
├── CMakePresets.json          # Debug/Release configure + build presets
├── STM32F405XX_FLASH.ld       # Linker script (1024K Flash, 128K RAM, 64K CCM)
├── startup_stm32f405xx.s      # Reset handler + vector table
├── stm_code.ioc               # CubeMX project
└── .gitignore                 # Ignores build/, mx.scratch
```

### Build target structure

The CubeMX-generated CMake creates two library targets:

1. **`stm32cubemx`** (INTERFACE) — include paths + compile definitions (`USE_HAL_DRIVER`, `STM32F405xx`)
2. **`STM32_Drivers`** (OBJECT) — HAL driver `.c` files compiled as objects, linked into the final executable

Application sources (`Core/Src/*.c`, `startup_stm32f405xx.s`) are added directly to `${CMAKE_PROJECT_NAME}`.

### Compiler flags (from `cmake/gcc-arm-none-eabi.cmake`)

- `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`
- `-Wall -fdata-sections -ffunction-sections -fstack-usage`
- Debug: `-O0 -g3` / Release: `-Os -g0`
- Linker: `--specs=nano.specs`, `--gc-sections`, `--print-memory-usage`
- Math lib linked: `-lm`

## Adding New Peripherals

When adding peripherals in CubeMX (UART, I2C, SPI, timers, etc.):

1. Open `stm_code.ioc` in CubeMX
2. Enable the desired IP(s) and configure pins
3. Regenerate code — new `MX_*_Init()` functions appear in their own `.c`/`.h` files under `Core/`
4. The `cmake/stm32cubemx/CMakeLists.txt` automatically updates with the new source files
5. User code between `/* USER CODE BEGIN */` markers in existing files is preserved

**Important**: `SystemClock_Config()` in `main.c` is NOT protected by USER CODE markers. If you change clock settings (e.g., to HSI), the function will be overwritten on regeneration.

## Current Project State

This is a **fresh project** — only the minimal CubeMX skeleton exists:

- `MX_GPIO_Init()` enables GPIOH and GPIOA clocks (for HSE oscillator pins and SWD debug pins only)
- No application code in the main loop yet
- No UART/I2C/SPI/timer drivers initialized
- Ready for adding peripherals and application logic
