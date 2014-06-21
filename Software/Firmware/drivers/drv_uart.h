#ifndef __UART_H__
#define __UART_H__


/**
 * @brief UART Databit type definitions
 */
typedef enum {
	UART_DATABIT_5		= 0,     		/*!< UART 5 bit data mode */
	UART_DATABIT_6,		     			/*!< UART 6 bit data mode */
	UART_DATABIT_7,		     			/*!< UART 7 bit data mode */
	UART_DATABIT_8		     			/*!< UART 8 bit data mode */
} UART_DATABIT_Type;

/**
 * @brief UART Stop bit type definitions
 */
typedef enum {
	UART_STOPBIT_1		= (0),   					/*!< UART 1 Stop Bits Select */
	UART_STOPBIT_2		 							/*!< UART Two Stop Bits Select */
} UART_STOPBIT_Type;

/**
 * @brief UART Parity type definitions
 */
typedef enum {
	UART_PARITY_NONE 	= 0,					/*!< No parity */
	UART_PARITY_ODD,	 						/*!< Odd parity */
	UART_PARITY_EVEN, 							/*!< Even parity */
	UART_PARITY_SP_1, 							/*!< Forced "1" stick parity */
	UART_PARITY_SP_0 							/*!< Forced "0" stick parity */
} UART_PARITY_Type;

/**
 * @brief FIFO Level type definitions
 */
typedef enum {
	UART_FIFO_TRGLEV0 = 0,	/*!< UART FIFO trigger level 0: 1 character */
	UART_FIFO_TRGLEV1, 		/*!< UART FIFO trigger level 1: 4 character */
	UART_FIFO_TRGLEV2,		/*!< UART FIFO trigger level 2: 8 character */
	UART_FIFO_TRGLEV3		/*!< UART FIFO trigger level 3: 14 character */
} UART_FITO_LEVEL_Type;


/********************************************************************//**
* @brief UART Interrupt Type definitions
**********************************************************************/
typedef enum {
	UART_INTCFG_RBR = 0,	/*!< RBR Interrupt enable*/
	UART_INTCFG_THRE,		/*!< THR Interrupt enable*/
	UART_INTCFG_RLS,		/*!< RX line status interrupt enable*/
	UART1_INTCFG_MS,		/*!< Modem status interrupt enable (UART1 only) */
	UART1_INTCFG_CTS,		/*!< CTS1 signal transition interrupt enable (UART1 only) */
	UART_INTCFG_ABEO,		/*!< Enables the end of auto-baud interrupt */
	UART_INTCFG_ABTO		/*!< Enables the auto-baud time-out interrupt */
} UART_INT_Type;

///**
// * @brief UART Line Status Type definition
// */
//typedef enum {
//	UART_LINESTAT_RDR	= UART_LSR_RDR,			/*!<Line status register: Receive data ready*/
//	UART_LINESTAT_OE	= UART_LSR_OE,			/*!<Line status register: Overrun error*/
//	UART_LINESTAT_PE	= UART_LSR_PE,			/*!<Line status register: Parity error*/
//	UART_LINESTAT_FE	= UART_LSR_FE,			/*!<Line status register: Framing error*/
//	UART_LINESTAT_BI	= UART_LSR_BI,			/*!<Line status register: Break interrupt*/
//	UART_LINESTAT_THRE	= UART_LSR_THRE,		/*!<Line status register: Transmit holding register empty*/
//	UART_LINESTAT_TEMT	= UART_LSR_TEMT,		/*!<Line status register: Transmitter empty*/
//	UART_LINESTAT_RXFE	= UART_LSR_RXFE			/*!<Error in RX FIFO*/
//} UART_LS_Type;

/**
 * @brief UART Auto-baudrate mode type definition
 */
typedef enum {
	UART_AUTOBAUD_MODE0				= 0,			/**< UART Auto baudrate Mode 0 */
	UART_AUTOBAUD_MODE1							/**< UART Auto baudrate Mode 1 */
} UART_AB_MODE_Type;


void rt_hw_uart_init(void);

#endif
