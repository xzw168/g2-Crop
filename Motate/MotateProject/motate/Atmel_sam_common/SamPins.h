

#ifndef SAMPINS_H_ONCE
#define SAMPINS_H_ONCE

 #include "HPins.h"
// #include <chip.h>
//#include "sam.h"
//#include "SamCommon.h"
//#include "MotateTimers.h"

#include <functional>   // for std::function
#include <type_traits>

namespace Motate {
    // Numbering is arbitrary:
    enum PinMode : PinMode_t {
        kUnchanged      = 0,
        kOutput         = 1,
        kInput          = 2,
        // These next two are NOT available on other platforms,
        // but cannot be masked out since they are required for
        // special pin functions. These should not be used in
        // end-user (sketch) code.
        kPeripheralA    = 3,
        kPeripheralB    = 4,
        kPeripheralC    = 5,
        kPeripheralD    = 6,
    };

    // Numbering is arbitrary, but bit unique for bitwise operations (unlike other architectures):
    enum PinOptions : PinOptions_t {
        kNormal         = 0,
        kTotem          = 0, // alias
        kPullUp         = 1<<1,
#if !defined(MOTATE_AVR_COMPATIBILITY)
        kWiredAnd       = 1<<2,
        kDriveLowOnly   = 1<<2, // alias
        kWiredAndPull   = kWiredAnd|kPullUp,
        kDriveLowPullUp = kDriveLowOnly|kPullUp, // alias
#endif // !MOTATE_AVR_COMPATIBILITY
#if !defined(MOTATE_AVR_COMPATIBILITY) && !defined(MOTATE_AVRX_COMPATIBILITY)
        kDeglitch       = 1<<4,
        kDebounce       = 1<<5,
#endif // !MOTATE_AVR_COMPATIBILITY && !MOTATE_SAM_COMPATIBILITY

        // Set the intialized value of the pin
        kStartHigh      = 1<<6,
        kStartLow       = 1<<7,

        // For use on PWM pins only!
        kPWMPinInverted = 1<<8,
    };

    enum PinInterruptOptions : PinInterruptOptions_t {
        kPinInterruptsOff                = 0,

        kPinInterruptOnChange            = 1,

        kPinInterruptOnRisingEdge        = 1<<1,
        kPinInterruptOnFallingEdge       = 2<<1,

        kPinInterruptOnLowLevel          = 3<<1,
        kPinInterruptOnHighLevel         = 4<<1,

        kPinInterruptAdvancedMask        = ((1<<3)-1)<<1,

        /* This turns the IRQ on, but doesn't set the timer to ever trigger it. */
        kPinInterruptOnSoftwareTrigger   = 1<<4,

        kPinInterruptTypeMask            = (1<<5)-1,

        /* Set priority levels here as well: */
        kPinInterruptPriorityHighest     = 1<<5,
        kPinInterruptPriorityHigh        = 1<<6,
        kPinInterruptPriorityMedium      = 1<<7,
        kPinInterruptPriorityLow         = 1<<8,
        kPinInterruptPriorityLowest      = 1<<9,

        kPinInterruptPriorityMask        = ((1<<10) - (1<<5))
    };

    struct _pinChangeInterrupt {
        const uint32_t pc_mask; // Pin uses "mask" so we use a different name. "pc" for pinChange
        std::function<void(void)> interrupt_handler;
        _pinChangeInterrupt *next;

        _pinChangeInterrupt(const _pinChangeInterrupt &) = delete; // delete the copy constructor, we only allow moves
        _pinChangeInterrupt &operator=(const _pinChangeInterrupt &) = delete; // delete the assigment operator, we only allow moves
        _pinChangeInterrupt &operator=(const _pinChangeInterrupt &&) = delete; // delete the move assigment operator, we only allow moves


        _pinChangeInterrupt(const uint32_t _mask, std::function<void(void)> &&_interrupt, _pinChangeInterrupt *&_first)
            : pc_mask{_mask}, interrupt_handler{std::move(_interrupt)}, next{nullptr}
        {
//            if (interrupt_handler) { // std::function returns false if the function isn't valid
                if (_first == nullptr) {
                    _first = this;
                    return;
                }

                _pinChangeInterrupt *i = _first;
                while (i->next != nullptr) {
                    i = i->next;
                }
                i->next = this;
//            }
        };

        void setInterrupt(std::function<void(void)> &&_interrupt)
        {
            interrupt_handler = std::move(_interrupt);
        };

        void setInterrupt(const std::function<void(void)> &_interrupt)
        {
            interrupt_handler = _interrupt;
        };
    };

    typedef uint32_t uintPort_t;

#pragma mark PortHardware
    /**************************************************
     *
     * HARDWARE LAYER: PortHardware
     *
     **************************************************/

    template <unsigned char portLetter>
    struct PortHardware {
        static const uint8_t letter = portLetter;

        // The constexpr functions we can define here, and get really great optimization.
        // These switch statements are handled by the compiler, not at runtime.
        constexpr Pio* const rawPort() const
        {

                return 0;

        };
        constexpr static const uint32_t peripheralId()
        {

                return 0;

        };
        constexpr const IRQn_Type _IRQn() const
        {

                return 0;

        };


        static _pinChangeInterrupt *_firstInterrupt;

        void setModes(const PinMode type, const uintPort_t mask) {

        };
        // Returns the mode of ONE pin, and only Input or Output
        PinMode getMode(const uintPort_t mask) {
            return kInput;
        };
        void setOptions(const PinOptions_t options, const uintPort_t mask) {

        };
        PinOptions_t getOptions(const uintPort_t mask) {
            return 0;
        };
        void set(const uintPort_t mask) {
           
        };
        void clear(const uintPort_t mask) {
            
        };
        void toggle(const uintPort_t mask) {

        };
        void write(const uintPort_t value) {

        };
        void write(const uintPort_t value, const uintPort_t mask) {

        };
        uintPort_t getInputValues(const uintPort_t mask) {
            return 0;
        };
        uintPort_t getOutputValues(const uintPort_t mask) {
            return 0;
        };
        Pio* portPtr() {
            return 0;
        };
        void setInterrupts(const uint32_t interrupts, const uintPort_t mask) {

        };

		void addInterrupt(_pinChangeInterrupt *newInt) {
		}
    };

    /**************************************************
     *
     * BASIC PINS: _MAKE_MOTATE_PIN
     *
     **************************************************/

#define _MAKE_MOTATE_PIN(pinNum, registerChar, registerPin) \
    template<> \
    struct Pin<pinNum> : RealPin<registerChar, registerPin> { \
        static const int16_t number = pinNum; \
        static const uint8_t portLetter = (uint8_t) registerChar; \
        Pin() : RealPin<registerChar, registerPin>() {}; \
        Pin(const PinMode type, const PinOptions_t options = kNormal) : RealPin<registerChar, registerPin>(type, options) {}; \
    }; \
    template<> \
    struct ReversePinLookup<registerChar, registerPin> : Pin<pinNum> { \
        ReversePinLookup() {}; \
        ReversePinLookup(const PinMode type, const PinOptions_t options = kNormal) : Pin<pinNum>(type, options) {}; \
    };



#pragma mark IRQPin support
    /**************************************************
     *
     * PIN CHANGE INTERRUPT SUPPORT: IsIRQPin / MOTATE_PIN_INTERRUPT
     *
     **************************************************/

    template<int16_t pinNum>
    constexpr const bool IsIRQPin() { return !Pin<pinNum>::isNull(); }; // Basically return if we have a valid pin.

#define MOTATE_PIN_INTERRUPT(number) \
    template<> void Motate::IRQPin<number>::interrupt()



#if defined(__SAM3X8E__) || defined(__SAM3X8C__)

#pragma mark ADC_Module/ACD_Pin (Sam3x)
    /**************************************************
     *
     * PIN CHANGE INTERRUPT SUPPORT: IsIRQPin / MOTATE_PIN_INTERRUPT
     *
     **************************************************/

    constexpr uint32_t startup_table[] = { 0, 8, 16, 24, 64, 80, 96, 112, 512, 576, 640, 704, 768, 832, 896, 960 };

    // Internal ADC object, and a parent of the ADCPin objects.
    // Handles: Setting options for the ADC module as a whole,
    //          and initializing the ADC module once.
    struct ADC_Module {
        static const uint32_t default_adc_clock_frequency = 20000000;
        static const uint32_t default_adc_startup_time = 12;
        static const uint32_t peripheralId() { return ID_ADC; }

        static bool inited_;

        void init(const uint32_t adc_clock_frequency, const uint8_t adc_startuptime) {
 
        };

        ADC_Module() {
            init(default_adc_clock_frequency, default_adc_startup_time);
        };

        static void startSampling() {
            
        };

        static void startFreeRunning() {
            
        };
    };

    template<pin_number pinNum>
    struct ADCPinParent {
        static const uint32_t adcMask = 0;
        static const uint32_t adcNumber = 0;
        static const uint16_t getTop() { return 4095; };
    };

    template<pin_number pinNum>
    struct ADCPin : ADCPinParent<pinNum>, Pin<pinNum>, ADC_Module {
        using ADCPinParent<pinNum>::adcMask;
        using ADCPinParent<pinNum>::adcNumber;
        using ADCPinParent<pinNum>::getTop;

        ADCPin() : ADCPinParent<pinNum>(), Pin<pinNum>(kInput), ADC_Module() { init(); };
        ADCPin(const PinOptions_t options) : ADCPinParent<pinNum>(), Pin<pinNum>(kInput), ADC_Module() { init(); };

        void init() {

        };
        uint32_t getRaw() {
            return 0;
        };
        uint32_t getValue() {

            return getRaw();
        };
        operator int16_t() {
            return getValue();
        };
        operator float() {
            return (float)getValue() / getTop();
        };
        static const bool is_real = true;
        void setInterrupts(const uint32_t interrupts) {
        };
        static void interrupt();
    };

    template<int16_t adcNum>
    struct ReverseADCPin : ADCPin<-1> {
        ReverseADCPin() : ADCPin<-1>() {};
        ReverseADCPin(const PinOptions_t options) : ADCPin<-1>() {};
    };

#define _MAKE_MOTATE_ADC_PIN(registerChar, registerPin, adcNum) \
    template<> \
    struct ADCPinParent< ReversePinLookup<registerChar, registerPin>::number > { \
        static const uint32_t adcMask = 1 << adcNum; \
        static const uint32_t adcNumber = adcNum; \
        static const uint16_t getTop() { return 4095; }; \
    }; \
    template<> \
    struct ReverseADCPin<adcNum> : ADCPin<ReversePinLookup<registerChar, registerPin>::number> { \
        ReverseADCPin() : ADCPin<ReversePinLookup<registerChar, registerPin>::number>() {}; \
        ReverseADCPin(const PinOptions_t options) : ADCPin<ReversePinLookup<registerChar, registerPin>::number>(options) {}; \
    };

    template<int16_t pinNum>
    constexpr const bool IsADCPin() { return ADCPin<pinNum>::is_real; };

    template<uint8_t portChar, int16_t portPin>
    using LookupADCPin = ADCPin< ReversePinLookup<portChar, portPin>::number >;

    template<int16_t adcNum>
    using LookupADCPinByADC = ADCPin< ReverseADCPin< adcNum >::number >;

#else // not Sam3x

#pragma mark ADC_Module/ACD_Pin (Sam3x)
    /**************************************************
     *
     * PIN CHANGE INTERRUPT SUPPORT: IsIRQPin / MOTATE_PIN_INTERRUPT
     *
     **************************************************/

    template<pin_number pinNum>
    struct ADCPinParent {
        static const uint32_t adcMask = 0;
        static const uint32_t adcNumber = 0;
        static const uint16_t getTop() { return 4095; };
    };

    // Some pins are ADC pins.
     template<pin_number n>
     struct ADCPin : Pin<-1> {
         ADCPin() : Pin<-1>() {};
         ADCPin(const PinOptions_t options) : Pin<-1>() {};
    
         uint32_t getRaw() {
             return 0;
         };
         uint32_t getValue() {
             return 0;
         };
         operator int16_t() {
             return getValue();
         };
         operator float() {
             return 0.0;
         };
         static const uint16_t getTop() { return 4095; };
    
         static const bool is_real = false;
         void setInterrupts(const uint32_t interrupts) {};
         static void interrupt() __attribute__ (( weak )); // Allow setting an interrupt on a invalid ADC pin -- will never be called
     };

    template<int16_t adcNum>
    struct ReverseADCPin : ADCPin<-1> {
        ReverseADCPin() : ADCPin<-1>() {};
        ReverseADCPin(const PinOptions_t options) : ADCPin<-1>() {};
    };

    #define _MAKE_MOTATE_ADC_PIN(registerChar, registerPin, adcNum) \
        template<> \
            struct ADCPinParent< ReversePinLookup<registerChar, registerPin>::number > { \
            static const uint32_t adcMask = 1 << adcNum; \
            static const uint32_t adcNumber = adcNum; \
            static const uint16_t getTop() { return 4095; }; \
        }; \
        template<> \
        struct ReverseADCPin<adcNum> : ADCPin<ReversePinLookup<registerChar, registerPin>::number> { \
            ReverseADCPin() : ADCPin<ReversePinLookup<registerChar, registerPin>::number>() {}; \
            ReverseADCPin(const PinOptions_t options) : ADCPin<ReversePinLookup<registerChar, registerPin>::number>(options) {}; \
        };

    template<int16_t pinNum>
    constexpr const bool IsADCPin() { return ADCPin<pinNum>::is_real; };

    template<uint8_t portChar, int16_t portPin>
    using LookupADCPin = ADCPin< ReversePinLookup<portChar, portPin>::number >;

    template<int16_t adcNum>
    using LookupADCPinByADC = ADCPin< ReverseADCPin< adcNum >::number >;

#endif


#pragma mark PWMOutputPin support
    /**************************************************
     *
     * PWM ("fake" analog) output pin support: _MAKE_MOTATE_PWM_PIN
     *
     **************************************************/
#if 0
    #define _MAKE_MOTATE_PWM_PIN(registerChar, registerPin, timerOrPWM, peripheralAorB, invertedByDefault) \
        template<> \
        struct AvailablePWMOutputPin< ReversePinLookup<registerChar, registerPin>::number > : RealPWMOutputPin< ReversePinLookup<registerChar, registerPin>::number, timerOrPWM > { \
            typedef timerOrPWM parentTimerType; \
            static const pin_number pinNum = ReversePinLookup<registerChar, registerPin>::number; \
            AvailablePWMOutputPin() : RealPWMOutputPin<pinNum, timerOrPWM>(kPeripheral ## peripheralAorB) { pwmpin_init(invertedByDefault ? kPWMOnInverted : kPWMOn);}; \
            AvailablePWMOutputPin(const PinOptions_t options, const uint32_t freq) : RealPWMOutputPin<pinNum, timerOrPWM>(kPeripheral ## peripheralAorB, options, freq) { \
                pwmpin_init((invertedByDefault ^ ((options & kPWMPinInverted)?true:false)) ? kPWMOnInverted : kPWMOn); \
            }; \
            using RealPWMOutputPin<pinNum, timerOrPWM>::operator=; \
            /* Signal to _GetAvailablePWMOrAlike that we're here, AND a real Pin<> exists. */ \
            static constexpr bool _isAvailable() { return !ReversePinLookup<registerChar, registerPin>::isNull(); };  \
        };
#endif

#pragma mark SPI Pins support
    /**************************************************
     *
     * SPI PIN METADATA and wiring: specializes SPIChipSelectPin / SPIMISOPin / SPIMOSIPin / SPISCKPin
     *
     * Provides: _MAKE_MOTATE_SPI_CS_PIN
     *           _MAKE_MOTATE_SPI_MISO_PIN
     *           _MAKE_MOTATE_SPI_MOSI_PIN
     *           _MAKE_MOTATE_SPI_SCK_PIN
     *
     **************************************************/


    #define _MAKE_MOTATE_SPI_CS_PIN(registerChar, registerPin, spiNumber, peripheralAorB, csNum) \
        template<> \
        struct SPIChipSelectPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> { \
            SPIChipSelectPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {}; \
            static constexpr bool is_real = true; \
            static constexpr uint8_t spiNum = spiNumber; \
            static constexpr uint8_t csNumber =  csNum; \
            static constexpr uint8_t csValue  = ~csNum; \
            static constexpr bool usesDecoder = false; \
        };

    #define _MAKE_MOTATE_SPI_MISO_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
        template<>\
        struct SPIMISOPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            SPIMISOPin() : ReversePinLookup<registerChar, registerPin>{kPeripheral ## peripheralAorB} {};\
            static constexpr bool is_real = true; \
            static constexpr uint8_t spiNum = spiNumber; \
        };


    #define _MAKE_MOTATE_SPI_MOSI_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
        template<>\
        struct SPIMOSIPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            SPIMOSIPin() : ReversePinLookup<registerChar, registerPin>{kPeripheral ## peripheralAorB} {};\
            static constexpr bool is_real = true; \
            static constexpr uint8_t spiNum = spiNumber; \
        };


    #define _MAKE_MOTATE_SPI_SCK_PIN(registerChar, registerPin, spiNumber, peripheralAorB)\
        template<>\
        struct SPISCKPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            SPISCKPin() : ReversePinLookup<registerChar, registerPin> { kPeripheral ## peripheralAorB } {};\
            static constexpr bool is_real = true; \
            static constexpr uint8_t spiNum = spiNumber; \
        };


#pragma mark UART / USART Pin support
    /**************************************************
     *
     * UART/USART PIN METADATA and wiring: specializes UARTTxPin / UARTRxPin / UARTRTSPin / UARTCTSPin
     *
     * Provides: _MAKE_MOTATE_UART_TX_PIN
     *           _MAKE_MOTATE_UART_RX_PIN
     *           _MAKE_MOTATE_UART_RTS_PIN
     *           _MAKE_MOTATE_UART_CTS_PIN
     *
     **************************************************/

    #define _MAKE_MOTATE_UART_TX_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
        template<>\
        struct UARTTxPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            UARTTxPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, kPullUp) {};\
            static const uint8_t uartNum = uartNumVal;\
            static const bool is_real = true;\
        };

    #define _MAKE_MOTATE_UART_RX_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
        template<>\
        struct UARTRxPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            UARTRxPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {};\
            static const uint8_t uartNum = uartNumVal;\
            static const bool is_real = true;\
        };

    #define _MAKE_MOTATE_UART_RTS_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
        template<>\
        struct UARTRTSPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            UARTRTSPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {};\
            static const uint8_t uartNum = uartNumVal;\
            static const bool is_real = true;\
            void operator=(const bool value); /*Will cause a failure if used.*/\
        };

    #define _MAKE_MOTATE_UART_CTS_PIN(registerChar, registerPin, uartNumVal, peripheralAorB)\
        template<>\
        struct UARTCTSPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            UARTCTSPin() : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, kPullUp) {};\
            UARTCTSPin(const PinOptions_t options, const std::function<void(void)> &&_interrupt, const uint32_t interrupt_settings = kPinInterruptOnChange|kPinInterruptPriorityMedium) : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB, options) {};\
            void setInterrupts(const uint32_t interrupts); /*Will cause a failure if used.*/\
            static const uint8_t uartNum = uartNumVal;\
            static const bool is_real = true;\
        };

#pragma mark ClockOutputPin
    /**************************************************
     *
     * Clock Output PIN METADATA and wiring: CLKOutPin
     *
     * Provides: _MAKE_MOTATE_CLOCK_OUTPUT_PIN
     *
     **************************************************/

    #define _MAKE_MOTATE_CLOCK_OUTPUT_PIN(registerChar, registerPin, clockNumber, peripheralAorB)\
        template<>\
        struct ClockOutputPin< ReversePinLookup<registerChar, registerPin>::number > : ReversePinLookup<registerChar, registerPin> {\
            ClockOutputPin(const uint32_t target_freq) : ReversePinLookup<registerChar, registerPin>(kPeripheral ## peripheralAorB) {\
                uint32_t prescaler = PMC_PCK_PRES_CLK_1;\
                if ((SystemCoreClock >> 1) < target_freq) { prescaler = PMC_PCK_PRES_CLK_2; }\
                if ((SystemCoreClock >> 2) < target_freq) { prescaler = PMC_PCK_PRES_CLK_4; }\
                if ((SystemCoreClock >> 3) < target_freq) { prescaler = PMC_PCK_PRES_CLK_8; }\
                if ((SystemCoreClock >> 4) < target_freq) { prescaler = PMC_PCK_PRES_CLK_16; }\
                if ((SystemCoreClock >> 5) < target_freq) { prescaler = PMC_PCK_PRES_CLK_32; }\
                if ((SystemCoreClock >> 6) < target_freq) { prescaler = PMC_PCK_PRES_CLK_64; }\
                PMC->PMC_PCK[clockNumber] = PMC_PCK_CSS_MCK | prescaler;\
            };\
            static const bool is_real = true;\
            void operator=(const bool value); /*Will cause a failure if used.*/\
        };

} // end namespace Motate

#endif /* end of include guard: SAMPINS_H_ONCE */
