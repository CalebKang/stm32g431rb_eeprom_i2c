This is an exmple of STM32G431 for read/write to M24C01/02 or M24C04-W M24C04-R M24C04(EEPROM) with LL.


TXIS flag: 
In the Master case of a write transfer, the TXIS flag is set after each byte transmission, after the 9th
SCL pulse when an ACK is received.
A transmit interrupt status (TXIS) is generated when the I2C_TXDR register becomes empty.
The TXIS bit is not set when a NACK is received.

Automatic end mode (AUTOEND = ‘1’ in the I2C_CR2 register). 
In this mode, the master automatically sends a STOP condition once the number of bytes programmed in the NBYTES[7:0] bit field is transferred.

Software end mode (AUTOEND = ‘0’ in the I2C_CR2 register). In this mode, software action is expected once the number of bytes programmed in the NBYTES[7:0] bit field
is transferred; the TC flag is set and an interrupt is generated if the TCIE bit is set. The SCL signal is stretched as long as the TC flag is set. The TC flag is cleared by software when the START or STOP bit is set in the I2C_CR2 register. This mode must be used when the master wants to send a RESTART condition.

NBYTES[7:0]: Number of bytes
The number of bytes to be transmitted/received is programmed there. This field is don’t care in slave mode with SBC=0.

TC: Transfer Complete (master mode)
This flag is set by hardware when RELOAD=0, AUTOEND=0 and NBYTES data have been transferred. It is cleared by software when START bit or STOP bit is set.

TXE: Transmit data register empty (transmitters)
This bit is set by hardware when the I2C_TXDR register is empty. It is cleared when the next data to be sent is written in the I2C_TXDR register.
This bit can be written to ‘1’ by software in order to flush the transmit data register I2C_TXDR.

RXNE: Receive data register not empty (receivers)
This bit is set by hardware when the received data is copied into the I2C_RXDR register, and is ready to be read. It is cleared when I2C_RXDR is read.
Note: This bit is cleared by hardware when PE=0.

Software reset
PE must be kept low during at least 3 APB clock cycles in order to perform the software reset. This is ensured by writing the following software sequence: - Write PE=0 - Check PE=0 - Write PE=1.



