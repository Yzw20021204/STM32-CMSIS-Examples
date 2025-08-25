# STM32 CMSIS Examples: Register-Level Peripheral Tutorials

[![Releases](https://img.shields.io/badge/Releases-v1.0-blue?logo=github&style=for-the-badge)](https://github.com/Yzw20021204/STM32-CMSIS-Examples/releases)

<img src="https://upload.wikimedia.org/wikipedia/commons/1/12/STM32F4_Discovery_board.jpg" alt="STM32 board" width="720"/>

Short, focused examples that show how STM32 peripherals work at the register level. Each example uses CMSIS headers and direct register writes so you see how clocks, GPIO, timers, ADC, DAC, I2C, SPI, UART, and PWM tie together.

Badges
- Topics: adc • cmsis • dac • i2c • low-level-programming • pwm • register-level • spi • stm32 • timer • uart
- License: MIT

Table of contents
- What you will learn
- Repo structure
- Hardware supported
- Quick start (build, flash, run)
- Example list and reference code pointers
- How the examples use CMSIS
- Tips for register-level debugging
- Contributing
- Releases and downloads
- License

What you will learn
- How to enable peripheral clocks (RCC) and set up GPIO pins.
- How to configure NVIC and interrupts.
- How timers and PWM generate precise pulses.
- How ADC and DAC convert analog signals and route them to peripherals.
- How to talk to sensors using I2C and SPI.
- How to send and receive data over UART.
- How to read and modify registers safely using CMSIS types.

Why register-level examples
- They show the real hardware state.
- They reveal the role of each register bit.
- They reduce abstraction to a minimum so you learn core behavior.
- They map concepts from datasheets to working code.

Repository structure
- /examples
  - /gpio-led-toggle
  - /timer-pwm
  - /adc-simple
  - /dac-waveform
  - /i2c-scan
  - /spi-loopback
  - /uart-echo
  - /peripheral-basics
- /docs
  - register-maps.md
  - build-flashing.md
  - wiring-diagrams.md
- /tools
  - scripts to build and flash (Linux/Windows)
- /assets
  - images and schematics
- Makefile
- README.md
- LICENSE

Hardware supported
- STM32F0/F1/F3/F4 families (examples use conditional defines to select MCU)
- Basic boards: Nucleo, Discovery, and generic STM32 breakout boards
- Minimal wiring needed: USB ST-Link, jumper wires, breadboard, sensor modules (if required)

Prerequisites
- arm-none-eabi-gcc toolchain or STM32CubeIDE
- OpenOCD or ST-Link CLI or vendor programmer
- A supported STM32 board and a host PC
- Basic knowledge of C and a terminal

Quick start — build, flash, run
1) Clone repository
   - git clone https://github.com/Yzw20021204/STM32-CMSIS-Examples.git

2) Choose an example
   - cd examples/gpio-led-toggle

3) Build (Makefile)
   - make

4) Flash
   - make flash
   - Or use the provided script:
     - Linux: ./tools/flash_example.sh examples/gpio-led-toggle
     - Windows: tools\flash_example.bat examples\gpio-led-toggle

5) Run
   - Reset the board if needed.
   - Observe LED toggle or test with a serial terminal.

Prebuilt releases
- The Releases page provides ready-to-flash archives and a small runner to help new users.
- Download the release file from the Releases page and execute the included runner script appropriate for your OS (run_example.sh for Linux/macOS or flash_example.bat for Windows). The runner locates the board, flashes the selected example, and opens a serial terminal.
- Visit the releases here: https://github.com/Yzw20021204/STM32-CMSIS-Examples/releases

Example details (what each folder shows)

- gpio-led-toggle
  - Show how to enable AHB/APB clock, configure GPIO mode, set output type, and toggle pins.
  - Code highlights: GPIOx->MODER, GPIOx->OTYPER, GPIOx->BSRR.

- timer-pwm
  - Configure TIMx prescaler and auto-reload to get a 1 kHz PWM.
  - Show dead-time, capture/compare registers, and duty-cycle updates.
  - Code highlights: TIMx->PSC, TIMx->ARR, TIMx->CCRn, TIMx->CCER.

- adc-simple
  - Set ADC sample time, channel selection, and start conversion.
  - Show single-shot and continuous modes.
  - Code highlights: ADCx->SQR, ADCx->SMPR, ADCx->CR.

- dac-waveform
  - Use DAC to output a simple waveform.
  - Show triggering options and buffer configuration.
  - Code highlights: DAC->CR, DAC->DHRx.

- i2c-scan
  - Implement basic master write/read and a bus scan routine.
  - Show START/STOP generation and error handling.
  - Code highlights: I2Cx->CR1, I2Cx->CR2, I2Cx->SR1/SR2.

- spi-loopback
  - Configure full-duplex SPI and run a loopback test.
  - Show clock polarity/phase and data frame size.
  - Code highlights: SPIx->CR1, SPIx->DR, SPIx->SR.

- uart-echo
  - Configure UART baud rate, parity, and use IRQ to echo bytes.
  - Show enabling USART clocks and pin AF selection.
  - Code highlights: USARTx->BRR, USARTx->CR1, NVIC->ISER.

- peripheral-basics
  - Small demos: RCC config, flash latency, system clock switch, watchdog.
  - Code highlights: RCC->CFGR, FLASH->ACR, IWDG->KR.

Build system
- Makefile supports:
  - TARGET=stm32f103, stm32f401, etc.
  - BOARD selection
  - make clean, make all, make flash
- You can import the example C file into STM32CubeIDE or Keil. The code relies on CMSIS core headers and device headers. No HAL is required.

How the examples use CMSIS
- All examples include CMSIS core headers for the selected device.
- The code uses the CMSIS register definitions (e.g., GPIO_TypeDef, TIM_TypeDef).
- You will see direct register access like RCC->AHBENR or TIM1->CCR1.
- Examples avoid HAL and LL to keep focus on the register behavior.

Wiring and schematics
- Each example includes a wiring diagram in /docs/wiring-diagrams.md.
- For I2C and SPI examples, wire pull-ups and common ground.
- For ADC and DAC examples, use stable reference voltages and add buffering where needed.

Debugging tips
- Use a logic analyzer to observe SPI, I2C, or UART lines.
- Use LED or GPIO toggles to mark code execution points.
- Read status flags before clearing them to avoid missed events.
- When a peripheral does not start, check:
  - Clock enable (RCC)
  - Pin alternate function and mode
  - Correct interrupt enable and NVIC priority
- Use volatile pointers and read-modify-write carefully to avoid race conditions.

Testing on multiple MCU families
- The examples use conditional compilation to select register names and offsets.
- Define TARGET in the Makefile. Example:
  - make TARGET=stm32f103 BOARD=nucleo-f103rb
- Adjust linker script and startup file when you move to a different core.

Scripts and utilities
- /tools contains:
  - flash_example.sh — auto-detect ST-Link, flash .bin, open serial.
  - flash_example.bat — Windows counterpart.
  - build_all.sh — build all examples for a target.
- These scripts expect arm-none-eabi tools and st-flash or stlink utilities in PATH.

Best practices for register-level code
- Group configuration code into init functions.
- Keep the main loop simple and deterministic.
- Use static inline helper functions for register bit masks.
- Avoid magic numbers; define register bit names.
- Prefer read-modify-write with bit-band or atomic operations when needed.

Contributing
- Use feature branches.
- Open issues for bugs or suggested examples.
- Add unit tests where possible (host simulations for logic).
- Keep examples small and focused; one peripheral per demo is ideal.

References and learning resources
- ARM CMSIS core documentation
- STM32 reference manuals and datasheets
- Sample application notes on timers and ADC
- Community tutorials on register programming

Releases and downloads
- Prebuilt binaries and a runnable package live on the Releases page. Download the release file and run the included runner script to flash an example to your board. The runner simplifies flashing and opens a terminal for UART examples.
- Direct link to releases: https://github.com/Yzw20021204/STM32-CMSIS-Examples/releases

License
- MIT License — see LICENSE file for details

Contact
- Open an issue on GitHub for bugs, questions, or pull requests.