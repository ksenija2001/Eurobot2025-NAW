/*
 * fdcan.c
 *
 *  Created on: Oct 25, 2024
 *      Author: xenia
 */

#include <stm32g4xx.h>

// Impending Errors
/*
Restricted operation mode is automatically entered when the Tx handler is not able to read
data from the message RAM in time. To leave restricted operation mode, the software has to
clear the ASM bit of FDCAN_CCCR. */

void Init_FDCAN()
{
	// Enables clock source for FDCAN1 peripheral
	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

	// Allows the CPU to access write protected FDCAN configuration registers
	FDCAN1->CCCR |= (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);

	// Enables FDCAN frame transmission and reception operations
	FDCAN1->CCCR |= (FDCAN_CCCR_FDOE);

	// Enable bit rate switching for the data field of a FDCAN frame
	// All Tx buffer elements with FDF and BRS bits set are transmitted in CAN FD format with bit rate switching.
	FDCAN1->CCCR |= (FDCAN_CCCR_BRSE);

	// Set bit timing for the arbitration phase
	FDCAN1->NBTP |= (0x01 << FDCAN_NBTP_NBRP_Pos);
	FDCAN1->NBTP |= (0x6B << FDCAN_NBTP_NTSEG1_Pos);
	FDCAN1->NBTP |= (0x24 << FDCAN_NBTP_NTSEG2_Pos);
	FDCAN1->NBTP |= (0x24 << FDCAN_NBTP_NSJW_Pos);

	// Set bit timing for the data phase
	FDCAN1->DBTP |= (0x01 << FDCAN_DBTP_DBRP_Pos);
	FDCAN1->DBTP |= (0x08 << FDCAN_DBTP_DTSEG1_Pos);
	FDCAN1->DBTP |= (0x09 << FDCAN_DBTP_DTSEG2_Pos);
	FDCAN1->DBTP |= (0x09 << FDCAN_DBTP_DSJW_Pos);

	// Set Tx buffer to queue mode
	FDCAN1->TXBC |= (FDCAN_TXBC_TFQM);

	// Enable interrupt generating
	FDCAN1->IE |= (FDCAN_IE_RF0NE);
	// Enable interrupt for Rx new message
	FDCAN1->IR |= (FDCAN_IR_RF0N);
	// Enable FDCAN interrupt line 0
	FDCAN1->ILE |= (FDCAN_ILE_EINT0);
	// Assign interrupt line 0 to Rx interrupts
	FDCAN1->ILS |= (FDCAN_ILS_RXFIFO0);

	// Enable overwrite mode for Rx FIFO
	FDCAN1->RXGFC |= (FDCAN_RXGFC_F0OM);

	// Exit initialization mode
	FDCAN1->CCCR &= ~(1 << FDCAN_CCCR_INIT_Pos);
}
