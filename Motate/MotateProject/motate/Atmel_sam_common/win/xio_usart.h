

#ifndef xio_usart_h
#define xio_usart_h

// Baud rate configuration
#define	XIO_BAUD_DEFAULT XIO_BAUD_115200

typedef enum {         		        // BSEL	  BSCALE
		XIO_BAUD_UNSPECIFIED = 0,	//	0		0	  // use default value
		XIO_BAUD_9600,				//	207		0
		XIO_BAUD_19200,				//	103		0
		XIO_BAUD_38400,				//	51		0
		XIO_BAUD_57600,				//	34		0
		XIO_BAUD_115200,			//	33		(-1<<4)
		XIO_BAUD_230400,			//	31		(-2<<4)
		XIO_BAUD_460800,			//	27		(-3<<4)
		XIO_BAUD_921600,			//	19		(-4<<4)
		XIO_BAUD_500000,			//	1		(1<<4)
		XIO_BAUD_1000000			//	1		0
} xioBAUDRATES;

int xio_usart_gets(char *buf, const int size);
//³õÊ¼»¯
void xio_usart_Init(void);

#endif
