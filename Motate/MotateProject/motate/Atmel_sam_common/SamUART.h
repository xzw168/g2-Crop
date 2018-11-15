/*
 SamUART.h - Library for the Motate system
 http://github.com/synthetos/motate/

 Copyright (c) 2016 Robert Giseburt

 This file is part of the Motate Library.

 This file ("the software") is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License, version 2 as published by the
 Free Software Foundation. You should have received a copy of the GNU General Public
 License, version 2 along with the software. If not, see <http://www.gnu.org/licenses/>.

 As a special exception, you may use this file as part of a software library without
 restriction. Specifically, if other files instantiate templates or use macros or
 inline functions from this file, or you compile this file and link it with  other
 files to produce an executable, this file does not by itself cause the resulting
 executable to be covered by the GNU General Public License. This exception does not
 however invalidate any other reasons why the executable file might be covered by the
 GNU General Public License.

 THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#ifndef SAMUART_H_ONCE
#define SAMUART_H_ONCE

#include <MotateUART.h>
#include <MotateBuffer.h>
#include <type_traits>
#include <algorithm> // for std::max, etc.
#include <functional>

//#include "SamCommon.h" // pull in defines and fix them
//#include "SamDMA.h" // pull in defines and fix them

namespace Motate {
    // Convenience template classes for specialization:
#define US_MR_USART_MODE_Pos 0
#define US_MR_USCLKS_Pos 4
#define US_MR_CHRL_Pos 6
#define US_MR_PAR_Pos 9
#define US_MR_NBSTOP_Pos 12
#define US_MR_CHMODE_Pos 14

    template<pin_number rxPinNumber, pin_number txPinNumber>
    using IsValidUART = typename std::enable_if<
        IsUARTRxPin<rxPinNumber>() &&
        IsUARTTxPin<txPinNumber>() &&
        rxPinNumber != txPinNumber &&
        UARTTxPin<txPinNumber>::moduleId == UARTRxPin<rxPinNumber>::moduleId
	>::type;

    static const char kUARTXOn  = 0x11;
    static const char kUARTXOff = 0x13;

    enum class USART_MODE_t : uint32_t {
        USART_NORMAL    = 0x0 << US_MR_USART_MODE_Pos,
        RS485           = 0x1 << US_MR_USART_MODE_Pos,
        HW_HANDSHAKING  = 0x2 << US_MR_USART_MODE_Pos,
        IS07816_T_0     = 0x4 << US_MR_USART_MODE_Pos,
        IS07816_T_1     = 0x5 << US_MR_USART_MODE_Pos,
        IRDA            = 0x8 << US_MR_USART_MODE_Pos,
        LIN_MASTER      = 0xA << US_MR_USART_MODE_Pos,
        LIN_SLAVE       = 0xB << US_MR_USART_MODE_Pos,
        SPI_MASTER      = 0xE << US_MR_USART_MODE_Pos,
        SPI_SLAVE       = 0xF << US_MR_USART_MODE_Pos
    };

    enum class USCLKS_t : uint32_t {
        MCK = 0x0 << US_MR_USCLKS_Pos,
        DIV = 0x1 << US_MR_USCLKS_Pos,
        SCK = 0x3 << US_MR_USCLKS_Pos
    };

    enum class CHRL_t : uint32_t  {
        CH_5_BIT = 0x0 << US_MR_CHRL_Pos,
        CH_6_BIT = 0x1 << US_MR_CHRL_Pos,
        CH_7_BIT = 0x2 << US_MR_CHRL_Pos,
        CH_8_BIT = 0x3 << US_MR_CHRL_Pos
    };

    enum class PAR_t : uint32_t {
        EVEN        = 0x0 << US_MR_PAR_Pos,
        ODD         = 0x1 << US_MR_PAR_Pos,
        SPACE       = 0x2 << US_MR_PAR_Pos,
        MARK        = 0x3 << US_MR_PAR_Pos,
        NO          = 0x4 << US_MR_PAR_Pos,
        MULTIDROP   = 0x6 << US_MR_PAR_Pos
    };

    enum class NBSTOP_t : uint32_t {
        STOP_1_BIT      = 0x0 << US_MR_NBSTOP_Pos,
        STOP_1_5_BIT    = 0x1 << US_MR_NBSTOP_Pos,
        STOP_2_BIT      = 0x2 << US_MR_NBSTOP_Pos
    };

    enum class CHMODE_t : uint32_t {
        CHMODE_NORMAL       = 0x0 << US_MR_CHMODE_Pos,
        AUTOMATIC           = 0x1 << US_MR_CHMODE_Pos,
        LOCAL_LOOPBACK      = 0x2 << US_MR_CHMODE_Pos,
        REMOTE_LOOPBACK     = 0x3 << US_MR_CHMODE_Pos
    };

    // USART peripherals
    template<uint8_t uartPeripheralNumber>
    struct _USARTHardware {





        static constexpr const uint8_t uartPeripheralNum=uartPeripheralNumber;

        std::function<void(uint16_t)> _uartInterruptHandler;

        typedef _USARTHardware<uartPeripheralNumber> this_type_t;

        static std::function<void()> _uartInterruptHandlerJumper;

        _USARTHardware()
        {
            // We DON'T init here, because the optimizer is fickle, and will remove this whole area.
            // Instead, we call init from UART<>::init(), so that the optimizer will keep it.
        };



        void init() {

        };

        void enable() { };
        void disable () {  };

        void setOptions(const uint32_t baud, const uint16_t options, const bool fromConstructor=false) {


        };

        void setInterrupts(const uint16_t interrupts) {

        };

        void setInterruptHandler(std::function<void(uint16_t)> &&handler) {
            _uartInterruptHandler = std::move(handler);
        }

        void _setInterruptTxReady(bool value) {

        };

        void setInterruptRxReady(bool value) {

        };

        void _setInterruptCTSChange(bool value) {

        };

        void setInterruptTxTransferDone(bool value) {

        };

        void setInterruptRxTransferDone(bool value) {

        };

        static uint16_t getInterruptCause() { // __attribute__ (( noinline ))

            return 0;
        }

        int16_t readByte() {

            return -1;
        };

        int16_t writeByte(const char value) {

            return -1;
        };

        void flush() {
            // Wait for the buffer to be empty

        };

        void flushRead() {
            // kill any incoming transfers

        };


        // ***** Connection status check (simple)
        bool isConnected() {

            return 0; // active LOW
        };


        // ***** Handle Tranfers
        bool startRXTransfer(char *buffer, const uint16_t length) {

            return 0;
        };

        char* getRXTransferPosition() {
            return 0;
        };

        bool _tx_paused = false;
        bool startTXTransfer(char *buffer, const uint16_t length) {
            return 0;
        };

        char* getTXTransferPosition() {
            return 0;
        };

        void pauseTX() {

        };

        void resumeTX() {

        };
    };


    // UART peripheral
    template<uint8_t uartPeripheralNumber>
    struct _UARTHardware {

		static constexpr const uint8_t uartPeripheralNum=uartPeripheralNumber;

        std::function<void(uint16_t)> _uartInterruptHandler;


        typedef _UARTHardware<uartPeripheralNumber> this_type_t;

        static std::function<void()> _uartInterruptHandlerJumper;

        _UARTHardware()
        {

        };


        void init() {

        };


        void setOptions(const uint32_t baud, const uint16_t options, const bool fromConstructor=false) {


        };

        void setInterrupts(const uint16_t interrupts) {

        };

        void setInterruptHandler(std::function<void(uint16_t)> &&handler) {
 
        }

        void _setInterruptTxReady(bool value) {

        };

        void setInterruptRxReady(bool value) {

        };

        void _setInterruptCTSChange(bool value) {

        };

        void setInterruptTxTransferDone(bool value) {

        };

        void setInterruptRxTransferDone(bool value) {

        };

        uint16_t getInterruptCause() { // __attribute__ (( noinline ))

            return 0;
        }

        int16_t readByte() {
            return 0;
        };

        int16_t writeByte(const char value) {
            return 1;
        };

        void flush() {
            // Wait for the buffer to be empty

        };

        void flushRead() {
            // kill any incoming transfers

        };


        // ***** Connection status check (simple)
        bool isConnected() {
            // The cts pin allows to know if we're allowed to send,
            // which gives us a reasonable guess, at least.

            // read the CTS pin
            return true; // assume always for now
        };


        // ***** Handle Tranfers
        bool startRXTransfer(char *buffer, const uint16_t length) {
            const bool handleInterrupts = true;
            const bool includeNext = true;
            return 0;
        };

        char* getRXTransferPosition() {
            return 0;
        };

        bool _tx_paused = false;
        bool startTXTransfer(char *buffer, const uint16_t length) {
            return 0;
        };

        char* getTXTransferPosition() {
            return 0;
        };

        void pauseTX() {

        };

        void resumeTX() {

        };
};

    template<uint8_t uartPeripheralNumber>
    using _USART_Or_UART = typename std::conditional< (uartPeripheralNumber < 4), _USARTHardware<uartPeripheralNumber>, _UARTHardware<uartPeripheralNumber-4>>::type;

    template<pin_number rxPinNumber, pin_number txPinNumber>
    using UARTGetHardware = typename std::conditional<
        IsUARTRxPin<rxPinNumber>() &&
        IsUARTTxPin<txPinNumber>() &&
        rxPinNumber != txPinNumber &&
        UARTTxPin<txPinNumber>::uartNum == UARTRxPin<rxPinNumber>::uartNum,
        /* True:  */ _USART_Or_UART<UARTTxPin<txPinNumber>::uartNum>,
        /* False: */ _UARTHardware<0xff> // static_assert below should prevent this
    >::type;

    template<pin_number rtsPinNumber, pin_number rxPinNumber>
    constexpr const bool isRealAndCorrectRTSPin() {
        return IsUARTRTSPin<rtsPinNumber>() && (UARTRTSPin<rtsPinNumber>::uartNum == UARTRxPin<rxPinNumber>::uartNum);
    }
    template<pin_number ctsPinNumber, pin_number rxPinNumber>
    constexpr const bool isRealAndCorrectCTSPin() {
        return IsUARTCTSPin<ctsPinNumber>() && (UARTCTSPin<ctsPinNumber>::uartNum == UARTRxPin<rxPinNumber>::uartNum);
    }


}

#endif /* end of include guard: SAMUART_H_ONCE */
