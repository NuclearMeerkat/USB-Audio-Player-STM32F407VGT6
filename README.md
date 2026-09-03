# USB Audio Player with Real-Time Digital Equalizer

An embedded USB MP3 audio player based on the **STM32F407VGT6**, extended with a real-time **10-band digital equalizer** and an **ESP32-based Wi-Fi control interface**.

The project was developed as a **Bachelor's Diploma Project in Computer Engineering** at Ternopil Ivan Pul'uj National Technical University.

## Overview

The system combines an STM32F407VGT6 microcontroller responsible for USB audio playback and real-time audio processing with an ESP32 providing a wireless browser-based control interface.

MP3 files are read from a USB storage device, decoded on the STM32, processed through a chain of digital biquad filters, and sent to the audio output. The equalizer parameters and volume can be adjusted remotely through a web interface hosted directly by the ESP32.

The ESP32 and STM32 communicate using a custom UART protocol with structured packets and checksums.

## Features

- USB MP3 audio playback
- STM32F407VGT6-based embedded audio processing
- Real-time 10-band digital equalizer
- Biquad digital filters based on CMSIS-DSP
- Adjustable gain from **-20 dB to +20 dB** for each equalizer band
- Equalizer bands from **32 Hz to 16 kHz**
- ESP32 Wi-Fi access point
- Browser-based control interface
- Remote volume control
- Remote equalizer control
- Custom UART communication protocol
- Runtime filter coefficient updates
- Checksum-based packet validation
- DMA-based audio playback
- Hardware and software architecture documentation

## System Architecture

The system is divided into two main processing units:

- **STM32F407VGT6** — USB host, MP3 decoding, audio processing and output
- **ESP32** — Wi-Fi access point, web server and user interface

The ESP32 provides the user interface while the STM32 performs the time-critical audio processing.

![System Structural Diagram](Diagrams/Structural%20Diagram.drawio.png)

### Main data flow

```text
USB Storage
     │
     ▼
 STM32F407VGT6
     │
     ├── USB Host
     │
     ├── MP3 Decoder
     │
     ├── 10-Band Equalizer
     │
     └── Audio Output
     
ESP32
     │
     ├── Wi-Fi Access Point
     ├── Web Server
     └── Browser UI
          │
          ▼
      UART Protocol
          │
          ▼
       STM32F407
```

## Audio Processing

The STM32 performs real-time digital audio processing using a cascade of **biquad filters**.

The equalizer contains ten configurable frequency bands:

| Band | Center Frequency |
|---:|---:|
| 1 | 32 Hz |
| 2 | 64 Hz |
| 3 | 125 Hz |
| 4 | 250 Hz |
| 5 | 500 Hz |
| 6 | 1 kHz |
| 7 | 2 kHz |
| 8 | 4 kHz |
| 9 | 8 kHz |
| 10 | 16 kHz |

Each band can be independently adjusted between **-20 dB and +20 dB**.

The ESP32 calculates the filter coefficients from the selected frequency and gain and sends the resulting coefficients to the STM32. The STM32 then updates the corresponding filter and applies it during audio processing.

![Audio Callback Block Diagram](Diagrams/AudioCallback%20Block%20Diagram.drawio.png)

The audio processing pipeline can be summarized as:

```text
USB MP3 File
     │
     ▼
 MP3 Decoder
     │
     ▼
 PCM Audio Samples
     │
     ▼
 10-Band Biquad EQ
     │
     ▼
 Audio Output
```

The project uses CMSIS-DSP functionality for the biquad filtering implementation.

## ESP32 Wi-Fi Control Interface

The ESP32 operates as a standalone Wi-Fi access point and hosts a lightweight web interface.

After connecting to the ESP32 network, the user can open the control page in a browser and adjust:

- Master volume
- Individual equalizer bands
- Frequency-specific gain

The interface contains ten vertical EQ sliders corresponding to the ten frequency bands.

```text
ESP32
 │
 ├── Wi-Fi Access Point
 │
 ├── HTTP Web Server
 │
 └── Web UI
       │
       ├── Volume
       └── 10-Band Equalizer
```

The interface is designed to work on both desktop and mobile screens.

## STM32 ↔ ESP32 Communication

The ESP32 communicates with the STM32 through UART at **38400 baud**.

A custom binary protocol is used to transfer volume settings and filter coefficients.

### Packet structure

All packets start with the following header:

```text
0xAA 0x55
```

Two command types are currently implemented.

### Volume packet

```text
+--------+--------+---------+--------+----------+
| Header | Header | Command | Volume | Checksum |
|  0xAA  |  0x55  |  0x01   |  1 byte|  1 byte  |
+--------+--------+---------+--------+----------+
```

Total size: **5 bytes**

### Filter coefficient packet

```text
+--------+--------+---------+----------+------------------+----------+
| Header | Header | Command | Filter ID | 5 × float (20 B) | Checksum |
|  0xAA  |  0x55  |  0x02   |  1 byte   |                  | 1 byte   |
+--------+--------+---------+----------+------------------+----------+
```

Total size: **25 bytes**

The checksum is calculated as the sum of all preceding packet bytes modulo 256.

On the STM32 side, received packets are validated before their contents are applied. Filter coefficient packets contain the five normalized biquad coefficients required by the corresponding filter.

## Hardware

The main processing platform is the **STM32F407VGT6** development board, with an **ESP32** used as the wireless control module.

The STM32 handles the time-critical audio path, while the ESP32 is responsible for network connectivity and user interaction.

![Electrical Schematic](Diagrams/Electrical%20Schematic%20Diagram.drawio.png)

![Wiring Diagram](Diagrams/Wiring%20Diagram.png)

## Program Structure

The STM32 firmware contains several major components:

```text
src/
├── main.c
├── Audio.c
├── stm32_ub_uart.c
├── stm32f4xx_it.c
├── usb_bsp.c
└── usbh_usr.c
```

The main application is responsible for:

- USB host initialization
- MP3 file handling
- MP3 decoding
- Audio playback
- Equalizer initialization
- UART packet reception
- Filter coefficient updates

The ESP32 firmware is located in:

```text
ESP32_Hot_Spot/
└── ESP32_Diplom.ino
```

## Repository Structure

```text
.
├── CompiledLibs/       # Precompiled libraries
├── Diagrams/           # System and hardware documentation
├── ESP32_Hot_Spot/     # ESP32 Wi-Fi control interface
├── inc/                # STM32 header files
├── lib/                # STM32 libraries and CMSIS
├── src/                # STM32 application source code
├── build/              # Compiled firmware
├── Makefile
└── README.md
```

## Documentation

The `Diagrams/` directory contains the technical documentation created for the project, including:

- System structural diagram
- Main program block diagram
- MP3 playback flow
- Directory playback flow
- Audio callback processing
- Electrical schematic
- Wiring diagram

These diagrams describe both the hardware organization and the software/audio-processing architecture.

## Building the Project

The STM32 firmware is an embedded C project designed for the **STM32F407VGT6** and can be built using the provided Makefile and ARM GCC toolchain.

Make sure the ARM GCC toolchain is installed and available in your system `PATH`.

The repository also contains pre-built firmware files in:

```text
build/
├── stm32F4_usb_mp3.bin
├── stm32F4_usb_mp3.elf
└── stm32F4_usb_mp3.hex
```

The ESP32 part of the project can be opened and flashed using the Arduino IDE or another compatible ESP32 development environment.

### ESP32 configuration

The ESP32 creates the following Wi-Fi network:

```text
SSID: USB_AUDIO_PLAYER
Password: 12345678
```

After connecting to the network, use the IP address printed to the serial monitor by the ESP32 to access the web control interface.

> **Security note:** The credentials above are intended for the local project/demo environment. Change them before using the ESP32 in a non-laboratory environment.

## Technologies

### Embedded / Hardware

- STM32F407VGT6
- ESP32
- C / C++
- ARM Cortex-M4
- USB Host
- UART / USART
- DMA
- I2S / audio peripherals

### Audio / DSP

- MP3 decoding
- PCM audio processing
- Digital biquad filters
- 10-band equalizer
- CMSIS-DSP
- Real-time audio processing

### ESP32

- Wi-Fi
- HTTP web server
- HTML / CSS / JavaScript
- UART communication

## Project Documentation

This project was developed as a **Bachelor's Diploma Project in Computer Engineering**.

The repository contains the implementation together with system architecture, hardware schematics, wiring documentation and audio-processing diagrams to provide a complete technical overview of the project.

## Credits

The original USB MP3 player implementation was based on the STM32F4 USB Host and MP3 Player project by **Benjamin Vedder**:

http://vedder.se/2012/12/stm32f4-discovery-usb-host-and-mp3-player/

The original project provided the foundation for USB host functionality and MP3 playback. This repository extends that foundation with the project's own functionality, including the real-time 10-band equalizer, digital filtering, ESP32 Wi-Fi control interface, UART communication protocol, and associated system documentation.

## License

Please refer to the original project and included source files for licensing information.

If you use or modify code originating from the original project, retain the applicable copyright and attribution notices.
