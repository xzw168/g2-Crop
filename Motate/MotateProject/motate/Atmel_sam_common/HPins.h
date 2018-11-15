

#ifndef HPins_H_ONCE
#define HPins_H_ONCE


#define ID_SUPC   ( 0) /**< \brief Supply Controller (SUPC) */
#define ID_RSTC   ( 1) /**< \brief Reset Controller (RSTC) */
#define ID_RTC    ( 2) /**< \brief Real Time Clock (RTC) */
#define ID_RTT    ( 3) /**< \brief Real Time Timer (RTT) */
#define ID_WDT    ( 4) /**< \brief Watchdog Timer (WDT) */
#define ID_PMC    ( 5) /**< \brief Power Management Controller (PMC) */
#define ID_EFC    ( 6) /**< \brief Enhanced Embedded Flash Controller (EFC) */
#define ID_UART0  ( 8) /**< \brief UART 0 (UART0) */
#define ID_UART1  ( 9) /**< \brief UART 1 (UART1) */
#define ID_PIOA   (11) /**< \brief Parallel I/O Controller A (PIOA) */
#define ID_PIOB   (12) /**< \brief Parallel I/O Controller B (PIOB) */
#define ID_USART0 (14) /**< \brief USART 0 (USART0) */
#define ID_USART1 (15) /**< \brief USART 1 (USART1) */
#define ID_HSMCI  (18) /**< \brief Multimedia Card Interface (HSMCI) */
#define ID_TWI0   (19) /**< \brief Two Wire Interface 0 (TWI0) */
#define ID_TWI1   (20) /**< \brief Two Wire Interface 1 (TWI1) */
#define ID_SPI    (21) /**< \brief Serial Peripheral Interface (SPI) */
#define ID_SSC    (22) /**< \brief Synchronous Serial Controler (SSC) */
#define ID_TC0    (23) /**< \brief Timer/Counter 0 (TC0) */
#define ID_TC1    (24) /**< \brief Timer/Counter 1 (TC1) */
#define ID_TC2    (25) /**< \brief Timer/Counter 2 (TC2) */
#define ID_ADC    (29) /**< \brief Analog To Digital Converter (ADC) */
#define ID_DACC   (30) /**< \brief Digital To Analog Converter (DACC) */
#define ID_PWM    (31) /**< \brief Pulse Width Modulation (PWM) */
#define ID_CRCCU  (32) /**< \brief CRC Calculation Unit (CRCCU) */
#define ID_ACC    (33) /**< \brief Analog Comparator (ACC) */
#define ID_UDP    (34) /**< \brief USB Device Port (UDP) */


typedef enum IRQn
{
	/******  Cortex-M3 Processor Exceptions Numbers ******************************/
	NonMaskableInt_IRQn = -14, /**<  2 Non Maskable Interrupt                */
	MemoryManagement_IRQn = -12, /**<  4 Cortex-M3 Memory Management Interrupt */
	BusFault_IRQn = -11, /**<  5 Cortex-M3 Bus Fault Interrupt         */
	UsageFault_IRQn = -10, /**<  6 Cortex-M3 Usage Fault Interrupt       */
	SVCall_IRQn = -5,  /**< 11 Cortex-M3 SV Call Interrupt           */
	DebugMonitor_IRQn = -4,  /**< 12 Cortex-M3 Debug Monitor Interrupt     */
	PendSV_IRQn = -2,  /**< 14 Cortex-M3 Pend SV Interrupt           */
	SysTick_IRQn = -1,  /**< 15 Cortex-M3 System Tick Interrupt       */
						/******  SAM3S8B specific Interrupt Numbers *********************************/

						SUPC_IRQn = 0, /**<  0 SAM3S8B Supply Controller (SUPC) */
						RSTC_IRQn = 1, /**<  1 SAM3S8B Reset Controller (RSTC) */
						RTC_IRQn = 2, /**<  2 SAM3S8B Real Time Clock (RTC) */
						RTT_IRQn = 3, /**<  3 SAM3S8B Real Time Timer (RTT) */
						WDT_IRQn = 4, /**<  4 SAM3S8B Watchdog Timer (WDT) */
						PMC_IRQn = 5, /**<  5 SAM3S8B Power Management Controller (PMC) */
						EFC_IRQn = 6, /**<  6 SAM3S8B Enhanced Embedded Flash Controller (EFC) */
						UART0_IRQn = 8, /**<  8 SAM3S8B UART 0 (UART0) */
						UART1_IRQn = 9, /**<  9 SAM3S8B UART 1 (UART1) */
						PIOA_IRQn = 11, /**< 11 SAM3S8B Parallel I/O Controller A (PIOA) */
						PIOB_IRQn = 12, /**< 12 SAM3S8B Parallel I/O Controller B (PIOB) */
						USART0_IRQn = 14, /**< 14 SAM3S8B USART 0 (USART0) */
						USART1_IRQn = 15, /**< 15 SAM3S8B USART 1 (USART1) */
						HSMCI_IRQn = 18, /**< 18 SAM3S8B Multimedia Card Interface (HSMCI) */
						TWI0_IRQn = 19, /**< 19 SAM3S8B Two Wire Interface 0 (TWI0) */
						TWI1_IRQn = 20, /**< 20 SAM3S8B Two Wire Interface 1 (TWI1) */
						SPI_IRQn = 21, /**< 21 SAM3S8B Serial Peripheral Interface (SPI) */
						SSC_IRQn = 22, /**< 22 SAM3S8B Synchronous Serial Controler (SSC) */
						TC0_IRQn = 23, /**< 23 SAM3S8B Timer/Counter 0 (TC0) */
						TC1_IRQn = 24, /**< 24 SAM3S8B Timer/Counter 1 (TC1) */
						TC2_IRQn = 25, /**< 25 SAM3S8B Timer/Counter 2 (TC2) */
						ADC_IRQn = 29, /**< 29 SAM3S8B Analog To Digital Converter (ADC) */
						DACC_IRQn = 30, /**< 30 SAM3S8B Digital To Analog Converter (DACC) */
						PWM_IRQn = 31, /**< 31 SAM3S8B Pulse Width Modulation (PWM) */
						CRCCU_IRQn = 32, /**< 32 SAM3S8B CRC Calculation Unit (CRCCU) */
						ACC_IRQn = 33, /**< 33 SAM3S8B Analog Comparator (ACC) */
						UDP_IRQn = 34  /**< 34 SAM3S8B USB Device Port (UDP) */
} IRQn_Type;


typedef struct {
	int PIO_PER;       /**< \brief (Pio Offset: 0x0000) PIO Enable Register */
} Pio;


typedef struct {
	int  TC_CHANNEL;

} Tc;

#endif /* end of include guard: SAMPINS_H_ONCE */
