

#ifndef SAMTIMERS_H_ONCE
#define SAMTIMERS_H_ONCE

#include "HPins.h"
#include <functional> // for std::function and related
#include <type_traits> // for std::extent and std::alignment_of
#include <time.h>


namespace Motate {

    enum TimerMode {
        /* InputCapture mode (WAVE = 0) */
        kTimerInputCapture         = 0,
        /* InputCapture mode (WAVE = 0), counts up to RC */
        kTimerInputCaptureToMatch ,

        /* Waveform select, Up to 0xFFFFFFFF */
        kTimerUp  ,
        /* Waveform select, Up to TOP (RC) */
        kTimerUpToTop   ,
        /* Keep the "ToMatch" naming for compatibility */
        kTimerUpToMatch  ,
        /* For PWM, we'll alias kTimerUpToMatch as: */
        kPWMLeftAligned   ,
        /* Waveform select, Up to 0xFFFFFFFF, then Down */
        kTimerUpDown ,
        /* Waveform select, Up to TOP (RC), then Down */
        kTimerUpDownToTop ,
        /* Keep the "ToMatch" naming for compatibility */
        kTimerUpDownToMatch,
        /* For PWM, we'll alias kTimerUpDownToMatch as: */
        kPWMCenterAligned ,
    };

    enum TimerSyncMode {
        kTimerSyncManually = 0,
        kTimerSyncDMA      = 1
    };

    /* We're trading acronyms for verbose CamelCase. Dubious. */
    enum TimerChannelOutputOptions {
        kOutputDisconnected = 0,

        kToggleOnMatch     = 1<<0,
        kClearOnMatch      = 1<<1,
        kSetOnMatch        = 1<<2,

        kToggleOnOverflow  = 1<<3,
        kClearOnOverflow   = 1<<4,
        kSetOnOverflow     = 1<<5,


        /* Aliases for use with PWM */
        kPWMOn             = kClearOnMatch | kSetOnOverflow,
        kPWMOnInverted     = kSetOnMatch | kClearOnOverflow,
    };

    /* We use TC_CMR_EEVT_XC0 in the above to allow TIOB to be an output.
     * The defualt is for it to be the input for ExternalEvent.
     * By setting it to XC0, we allow it to be an output.
     */

    enum TimerChannelInterruptOptions {
        kInterruptsOff              = 0,
        /* Alias for "off" to make more sense
         when returned from setInterruptPending(). */
        kInterruptUnknown           = 0,

        kInterruptOnMatch          = 1<<1,
        /* Note: Interrupt on overflow could be a match C as well. */
        kInterruptOnOverflow        = 1<<3,

        /* This turns the IRQ on, but doesn't set the timer to ever trigger it. */
        kInterruptOnSoftwareTrigger = 1<<4,

        /* Set priority levels here as well: */
        kInterruptPriorityHighest   = 1<<5,
        kInterruptPriorityHigh      = 1<<6,
        kInterruptPriorityMedium    = 1<<7,
        kInterruptPriorityLow       = 1<<8,
        kInterruptPriorityLowest    = 1<<9,
    };


    typedef const uint8_t timer_number;


    template <uint8_t timerNum>
    struct Timer {

		int irqEn;

        Timer() { init(); };
        Timer(const TimerMode mode, const uint32_t freq) {
            init();
            setModeAndFrequency(mode, freq);
        };

		void init() { 
			irqEn = 0;
		}

        int32_t setModeAndFrequency(const TimerMode mode, uint32_t freq) {
            return 0;
        };
        uint32_t getValue() {
            return clock();
        }
        void start() {
			irqEn = 1;
        };
        void stop() {
			irqEn = 0;
        };
        void setInterrupts(const uint32_t interrupts, const int16_t channel = -1) {

        }
        void setInterruptPending() {
			irqEn = 1;
        }
        // Placeholder for user code.
        static void interrupt();
    }; // Timer<>

    template<uint8_t timerNum, uint8_t channelNum>
    struct TimerChannel : Timer<timerNum> {
        TimerChannel() : Timer<timerNum>{} {};
        TimerChannel(const TimerMode mode, const uint32_t freq) : Timer<timerNum>{mode, freq} {};

        void setInterrupts(const uint32_t interrupts) {}
        void getInterruptCause(int16_t &channel) {}
        void getInterruptCause() {}
        // Placeholder for user code.
        static void interrupt();
    };


    struct SysTickEvent {
        const std::function<void(void)> callback;
        SysTickEvent *next;
    };

    static const timer_number SysTickTimerNum = 0xFF;
    template <>
	struct Timer<SysTickTimerNum> {
		static volatile uint32_t _motateTickCount;
		SysTickEvent *firstEvent = nullptr;

		Timer() { init(); };
		Timer(const TimerMode mode, const uint32_t freq) {
			init();
		};

		void init() {
			_motateTickCount = 500;
		};

		// Return the current value of the counter. This is a fleeting thing...
		uint32_t getValue() {
			return _motateTickCount;
		};

		void _increment() {
			_motateTickCount++;
		};

		void registerEvent(SysTickEvent *new_event) {
			if (firstEvent == nullptr) {
				firstEvent = new_event;
				return;
			}
			SysTickEvent *event = firstEvent;
			if (new_event == event) { return; }
			while (event->next != nullptr) {
				event = event->next;
				if (new_event == event) { return; }
			}
			event->next = new_event;
			new_event->next = nullptr;
		};

		void unregisterEvent(SysTickEvent *new_event) {
			if (firstEvent == new_event) {
				firstEvent = firstEvent->next;
				return;
			}
			SysTickEvent *event = firstEvent;
			while (event->next != nullptr) {
				if (event->next == new_event) {
					event->next = event->next->next;
					return;
				}
				event = event->next;
			}
		};

		void _handleEvents() {
			SysTickEvent *event = firstEvent;
			while (event != nullptr) {
				event->callback();
				event = event->next;
			}
		};

		void SysTick_Handler()
		{
			_increment();
			//if (interrupt) {
			//	interrupt();
			//}
			_handleEvents();
		}

		// Placeholder for user code.
		static void interrupt() ;
	};
	extern Timer<SysTickTimerNum> SysTickTimer;


    inline void delay( uint32_t microseconds )
    {

    }

    struct Timeout {
        uint32_t start_, delay_;
        Timeout() : start_ {0}, delay_ {0} {};

        bool isSet() {
            return (start_ > 0);
        }

        bool isPast() {
            if (!isSet()) {
                return false;
            }
            return ((SysTickTimer.getValue() - start_) > delay_);
        };

        void set(uint32_t delay) {
            start_ = SysTickTimer.getValue();
            delay_ = delay;
        };

        void clear() {
            start_ = 0;
            delay_ = 0;
        }
    };

} // namespace Motate


#endif /* end of include guard: SAMTIMERS_H_ONCE */
