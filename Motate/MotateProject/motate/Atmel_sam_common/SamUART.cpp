

#include "MotateUART.h"
#include "MotateDebug.h"

namespace Motate {
    template<> std::function<void()> _USARTHardware<0>::_uartInterruptHandlerJumper {};

#ifdef HAS_USART1
    template<> std::function<void()> _USARTHardware<1>::_uartInterruptHandlerJumper {};
#endif

    template<> std::function<void()> _UARTHardware<0>::_uartInterruptHandlerJumper {};

#ifdef HAS_UART1
    template<> std::function<void()> _UARTHardware<1>::_uartInterruptHandlerJumper {};
#endif
#ifdef HAS_UART2
    template<> std::function<void()> _UARTHardware<2>::_uartInterruptHandlerJumper {};
#endif
#ifdef HAS_UART3
    template<> std::function<void()> _UARTHardware<3>::_uartInterruptHandlerJumper {};
#endif

}

#ifdef HAD_UART
extern "C" void UART_Handler(void) __attribute__ ((weak, alias("UART0_Handler")));
#endif

extern "C" void USART0_Handler(void)  {
    if (Motate::_USARTHardware<0u>::_uartInterruptHandlerJumper) {
        Motate::_USARTHardware<0u>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}

#ifdef HAS_USART1
extern "C" void USART1_Handler(void)  {
    if (Motate::_USARTHardware<1u>::_uartInterruptHandlerJumper) {
        Motate::_USARTHardware<1u>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}
#endif


extern "C" void UART0_Handler(void)  {
    if (Motate::_UARTHardware<0>::_uartInterruptHandlerJumper) {
        Motate::_UARTHardware<0>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}

#ifdef HAS_UART1
extern "C" void UART1_Handler(void)  {
    if (Motate::_UARTHardware<1>::_uartInterruptHandlerJumper) {
        Motate::_UARTHardware<1>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}
#endif

#ifdef HAS_UART2
extern "C" void UART2_Handler(void)  {
    if (Motate::_UARTHardware<2>::_uartInterruptHandlerJumper) {
        Motate::_UARTHardware<2>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}
#endif

#ifdef HAS_UART3
extern "C" void UART3_Handler(void)  {
    if (Motate::_UARTHardware<3>::_uartInterruptHandlerJumper) {
        Motate::_UARTHardware<3>::_uartInterruptHandlerJumper();
        return;
    }
#if IN_DEBUGGER == 1
    __asm__("BKPT");
#endif
    //while (1) ;
}
#endif
