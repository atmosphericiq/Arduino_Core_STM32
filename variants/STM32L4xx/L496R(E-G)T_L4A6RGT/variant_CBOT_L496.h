/*
 *******************************************************************************
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA2                     0  // GNSS_EN

#define PB1                     1  // SD_DET

#define PB10                    2  // SCL
#define PB11                    3  // SDA

#define PB12                    4  // SD_CS
#define PB14                    5  // MISO
#define PB15                    6  // MOSI
#define PA9                     7  // SCK

#define PA11                    8  // USB_DM
#define PA12                    9  // USB_DP

#define PA13                    10  // SWDIO
#define PA14                    11  // SWCLK

#define PA15                    12  // AVCC_EN

#define PC10                    13  // SARA_RESET
#define PC11                    14  // SARA_ON
#define PC12                    15  // SARA_TXD
#define PD2                     16  // SARA_RXD

#define PB3                     17  // SWO

#define PB4                     18  // SARA_RTS
#define PB5                     19  // SARA_CTS

#define PB6                     20  // Button_LED

// Alternate pins number
#define PA0_ALT1                (PA0  | ALT1)
#define PA1_ALT1                (PA1  | ALT1)
#define PA1_ALT2                (PA1  | ALT2)
#define PA2_ALT1                (PA2  | ALT1)
#define PA2_ALT2                (PA2  | ALT2)
#define PA3_ALT1                (PA3  | ALT1)
#define PA3_ALT2                (PA3  | ALT2)
#define PA4_ALT1                (PA4  | ALT1)
#define PA5_ALT1                (PA5  | ALT1)
#define PA6_ALT1                (PA6  | ALT1)
#define PA7_ALT1                (PA7  | ALT1)
#define PA7_ALT2                (PA7  | ALT2)
#define PA7_ALT3                (PA7  | ALT3)
#define PA15_ALT1               (PA15 | ALT1)
#define PB0_ALT1                (PB0  | ALT1)
#define PB0_ALT2                (PB0  | ALT2)
#define PB1_ALT1                (PB1  | ALT1)
#define PB1_ALT2                (PB1  | ALT2)
#define PB3_ALT1                (PB3  | ALT1)
#define PB4_ALT1                (PB4  | ALT1)
#define PB5_ALT1                (PB5  | ALT1)
#define PB6_ALT1                (PB6  | ALT1)
#define PB7_ALT1                (PB7  | ALT1)
#define PB8_ALT1                (PB8  | ALT1)
#define PB9_ALT1                (PB9  | ALT1)
#define PB10_ALT1               (PB10 | ALT1)
#define PB11_ALT1               (PB11 | ALT1)
#define PB13_ALT1               (PB13 | ALT1)
#define PB14_ALT1               (PB14 | ALT1)
#define PB14_ALT2               (PB14 | ALT2)
#define PB15_ALT1               (PB15 | ALT1)
#define PB15_ALT2               (PB15 | ALT2)
#define PC0_ALT1                (PC0  | ALT1)
#define PC0_ALT2                (PC0  | ALT2)
#define PC1_ALT1                (PC1  | ALT1)
#define PC1_ALT2                (PC1  | ALT2)
#define PC2_ALT1                (PC2  | ALT1)
#define PC2_ALT2                (PC2  | ALT2)
#define PC3_ALT1                (PC3  | ALT1)
#define PC3_ALT2                (PC3  | ALT2)
#define PC4_ALT1                (PC4  | ALT1)
#define PC5_ALT1                (PC5  | ALT1)
#define PC6_ALT1                (PC6  | ALT1)
#define PC7_ALT1                (PC7  | ALT1)
#define PC8_ALT1                (PC8  | ALT1)
#define PC9_ALT1                (PC9  | ALT1)
#define PC10_ALT1               (PC10 | ALT1)
#define PC11_ALT1               (PC11 | ALT1)

#define NUM_DIGITAL_PINS        21
#define NUM_ANALOG_INPUTS       0

// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PB12
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PB15
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PB14
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PA9
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PB11
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PB10
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  5
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PD2
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PC12
#endif

// Extra HAL modules
#if !defined(HAL_CRC_MODULE_ENABLED)
  #define HAL_CRC_MODULE_ENABLED
#endif
// Value of the External oscillator in Hz
#define HSE_VALUE               8000000U

// Extra HAL modules

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial5
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_USBVIRTUAL  SerialUSB
  #endif
#endif
