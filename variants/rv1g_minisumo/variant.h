/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO_ZERO_
#define _VARIANT_ARDUINO_ZERO_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK			  (48000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (35u)
#define NUM_DIGITAL_PINS     (28u)
#define NUM_ANALOG_INPUTS    (4u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 4u) ? (p) + 30u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
// #define PIN_LED_RXL          (11u)
// #define PIN_LED_TXL          (12u)
// #define PIN_LED2             PIN_LED_RXL
// #define PIN_LED3             PIN_LED_TXL


/*
 * Analog pins
 */
#define PIN_A0               (30ul)
#define PIN_A1               (31ul)
#define PIN_A2               (32ul)
#define PIN_A3               (33ul)


static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
#define ADC_RESOLUTION		12


/*
 * Serial interfaces
 */
// Serial 
#define PIN_SERIAL_RX       (0ul)
#define PIN_SERIAL_TX       (1ul)
#define PAD_SERIAL_TX       (UART_TX_PAD_0)
#define PAD_SERIAL_RX       (SERCOM_RX_PAD_1)



/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 3

// #define PIN_WIRE_SDA         (16u)
// #define PIN_WIRE_SCL         (17u)
// #define PERIPH_WIRE          sercom3
// #define WIRE_IT_HANDLER      SERCOM3_Handler

// ToF sensor
#define PIN_WIRE_SDA         (20u)
#define PIN_WIRE_SCL         (21u)
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// IMU interface
#define PIN_WIRE1_SDA         (16u)
#define PIN_WIRE1_SCL         (17u)
#define PERIPH_WIRE1          sercom3
#define WIRE1_IT_HANDLER      SERCOM3_Handler

// OLED interface
#define PIN_WIRE2_SDA         (18u)
#define PIN_WIRE2_SCL         (19u)
#define PERIPH_WIRE2          sercom2
#define WIRE2_IT_HANDLER      SERCOM2_Handler



/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (0)
#define PIN_SPI_MOSI         (1)
#define PIN_SPI_SCK          (2)
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

static const uint8_t SS   = PIN_A2 ;  // SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * USB
 */
#define PIN_USB_DM          (28ul)
#define PIN_USB_DP          (29ul)





/* 
 * Mini Sumo
 */
#define PIN_FLR_LEFT        (30ul)
#define PIN_FLR_RIGHT       (31ul)
#define PIN_FLR_BACK        (32ul)
#define PIN_ADC_BAT         (33ul)

#define PIN_IR_SENSOR1      (2ul)
#define PIN_IR_SENSOR2      (3ul)
#define PIN_IR_SENSOR3      (4ul)
#define PIN_IR_SENSOR4      (5ul)
#define PIN_IR_SENSOR5      (6ul)
#define PIN_IR_LED          (26ul)

#define PIN_LED_RED         (11u)
#define PIN_LED_GREEN       (12u)

#define PIN_MOTOR1_1        (7ul)
#define PIN_MOTOR2_2        (8ul)
#define PIN_MOTOR1_2        (9ul)
#define PIN_MOTOR2_1        (10ul)

#define PIN_BUTTON1         (13ul)
#define PIN_BUTTON2         (14ul)
#define PIN_BUTTON3         (34ul)

#define PIN_LASER_1_RST     (22ul)
#define PIN_LASER_2_RST     (23ul)
#define PIN_LASER_3_RST     (24ul)

#define PIN_LED_NEO         (25ul)
#define COUNT_LED_NEO       (5u)

#define PIN_BUZZER          (27ul)



#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial;

#endif

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
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         Serial
// // Serial has no physical pins broken out, so it's not listed as HARDWARE port
// #define SERIAL_PORT_HARDWARE        Serial1
// #define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_ARDUINO_ZERO_ */

