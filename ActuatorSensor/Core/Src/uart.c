///*
// * uart.c
// *
// *  Created on: Mar 23, 2023
// *      Author: PC
// */
//
//#include "uart.h"
//#include "../lib/periferije/tajmeri/tajmeri.h"
//
//#include <stdint.h>
//#include <stdbool.h>
//#include "stm32f4xx.h"
//
//static void uart6_init();
//static void uart1_init();
//static void parse_buffer();
//static void parse_buffer6();
//static void uart2_init();
//
//volatile static uint8_t buffer[30] = { 0 };
//volatile static uint8_t buffer6[30] = { 0 };
//volatile static uint8_t buffer2[30] = { 0 };
//volatile static uint8_t size = 0;
//volatile static uint8_t size2 = 0;
//volatile static uint8_t size6 = 0;
//volatile static uint8_t write_index = 0;
//volatile static uint8_t write_index2 = 0;
//volatile static uint8_t write_index6 = 0;
//volatile static uint8_t read_index = 0;
//volatile static uint8_t read_index2 = 0;
//volatile static uint8_t read_index6 = 0;
//
//volatile static uint16_t arr_AX[6] = { 0 };
//volatile static uint16_t arr_communication[5] = { 0 };
//
//volatile static uint8_t cs = 0;
//volatile static uint8_t cs6 = 0;
//volatile static uint16_t angle = 0;
//volatile static uint16_t p1 = 0;
//volatile static uint16_t p2 = 0;
//volatile static uint16_t p = 0;
//
//volatile static uint16_t x1 = 0;
//volatile static uint16_t x2 = 0;
//volatile static uint16_t x = 0;
//volatile static uint16_t y1 = 0;
//volatile static uint16_t y2 = 0;
//volatile static uint16_t y = 0;
//
//void uart_init()
//{
//	uart6_init();
//	uart1_init();
//	uart2_init();
//}
//
//uint16_t get_angle(void)
//{
//	return angle;
//}
//
//uint16_t get_x_uart()
//{
//	return x;
//}
//
//uint16_t get_y_uart()
//{
//	return y;
//}
//
//static void uart1_init()
//{
//	RCC->APB2ENR |= (0b1 << 4); //usart 1
//	RCC->AHB1ENR |= (0b1 << 0); //pinovi 9 i 10, port A
//	const uint8_t pin_rx = 10;
//	const uint8_t pin_tx = 9;
//
//	GPIOA->MODER &= ~(0b11 << 2 * pin_tx);
//	GPIOA->MODER |= (0b10 << 2 * pin_tx);
//	GPIOA->MODER &= ~(0b11 << 2 * pin_rx);
//	GPIOA->MODER |= (0b10 << 2 * pin_rx);
//	//Half duplex pitaj Cilaga
//	GPIOA->OTYPER |= (0b01 << pin_tx);
//	GPIOA->PUPDR &= ~(0b11 << 2 * pin_tx);
//	GPIOA->PUPDR |= (0b01 << 2 * pin_tx);
//	//Podesavanje AF
//	//AF7
//	const uint8_t alt_function = 7;
//
//	GPIOA->AFR[1] &= ~(0b1111 << 4);
//	GPIOA->AFR[1] |= (alt_function << 4);
//	GPIOA->AFR[1] &= ~(0b1111 << 8);
//	GPIOA->AFR[1] |= (alt_function << 8);
//	//Podesavanja UART-a
//	USART1->CR1 &= ~(0b01 << 12); //8 bitova rec
//	USART1->CR2 &= ~(0b11 << 12); //1 stop bit
//	USART1->BRR &= ~(0xFFFF << 0); //fraction je 0
//	//USART1->BRR |= (91<<4|2); 						ZA 57600
//	USART1->BRR |= ((546<<4)|14);
//	//USART1->BRR |= ((45 << 4) | 9); //mantisa (mislim da je to vezano za baudrate)PITAJ CILAGA KAKO SE RACUNA MOZDA JE RAZLICITO ZA DRUGI UART
//	//Ukljucivanje TX i RX pina
//	USART1->CR1 |= ((0b01 << 2) | (0b01 << 3));
//	USART1->CR1 |= (0b01 << 5);
//	//Namestamo sledece na nulu jer hocemo half duplex
//	USART1->CR2 &= ~((0b01 << 14) | (0b01 << 11));
//	USART1->CR3 &= ~((0b01 << 5) | (0b01 << 7)); //uart4 nema ovaj peti bit
//	USART1->CR3 |= (0b01 << 3); //Ovo se postavlja na jedan za half duplex
//	//Paljenje uart-a
//	USART1->CR1 |= (0b01 << 13);
//
//	const uint8_t interrupt_index = 37;
//	NVIC->ISER[interrupt_index / 32] |= (0b01 << (interrupt_index % 32));
//}
//
//static void uart2_init()
//{
//	RCC->APB1ENR |= (0b1 << 17); //usart 2
//	RCC->AHB1ENR |= (0b1 << 0); //pinovi 9 i 10, port A
//	const uint8_t pin_rx = 3;
//	const uint8_t pin_tx = 2;
//
//	GPIOA->MODER &= ~(0b11 << 2 * pin_tx);
//	GPIOA->MODER |= (0b10 << 2 * pin_tx);
//	GPIOA->MODER &= ~(0b11 << 2 * pin_rx);
//	GPIOA->MODER |= (0b10 << 2 * pin_rx);
//	//Half duplex pitaj Cilaga
//	GPIOA->OTYPER |= (0b01 << pin_tx);
//	GPIOA->PUPDR &= ~(0b11 << 2 * pin_tx);
//	GPIOA->PUPDR |= (0b01 << 2 * pin_tx);
//	//Podesavanje AF
//	//AF7
//	const uint8_t alt_function = 7;
//
//
//	GPIOA->AFR[0] &= ~(0b1111 << 8);
//	GPIOA->AFR[0] |= (alt_function << 8);
//	GPIOA->AFR[0] &= ~(0b1111 << 12);
//	GPIOA->AFR[0] |= (alt_function << 12);
//	//Podesavanja UART-a
//	USART2->CR1 &= ~(0b01 << 12); //8 bitova rec
//	USART2->CR2 &= ~(0b11 << 12); //1 stop bit
//	USART2->BRR &= ~(0xFFFF << 0); //fraction je 0
//	USART2->BRR |= (91<<4|2);
//	//USART1->BRR |= ((546<<4)|14);
//	//USART1->BRR |= ((45 << 4) | 9); //mantisa (mislim da je to vezano za baudrate)PITAJ CILAGA KAKO SE RACUNA MOZDA JE RAZLICITO ZA DRUGI UART
//	//Ukljucivanje TX i RX pina
//	USART2->CR1 |= ((0b01 << 2) | (0b01 << 3));
//	USART2->CR1 |= (0b01 << 5);
//	USART2->CR1 |= (0b01 << 7);
//	//Namestamo sledece na nulu jer hocemo half duplex
//	USART2->CR2 &= ~((0b01 << 14) | (0b01 << 11));
//	USART2->CR3 &= ~((0b01 << 5) | (0b01 << 7)); //uart4 nema ovaj peti bit
//	USART2->CR3 &= ~(0b01 << 3); //Ovo se postavlja na jedan za half duplex
//	//Paljenje uart-a
//	USART2->CR1 |= (0b01 << 13);
//
//	const uint8_t interrupt_index = 38;
//	NVIC->ISER[interrupt_index / 32] |= (0b01 << (interrupt_index % 32));
//}
//
//static void uart6_init()
//{
//	RCC->APB2ENR |= (0b1 << 5);
//	RCC->AHB1ENR |= (0b1 << 2); //pinovi 6 i 7, port C
//	const uint8_t pin_rx = 7;
//	const uint8_t pin_tx = 6;
//
//	GPIOC->MODER &= ~(0b11 << 2 * pin_tx);
//	GPIOC->MODER |= (0b10 << 2 * pin_tx);
//	GPIOC->MODER &= ~(0b11 << 2 * pin_rx);
//	GPIOC->MODER |= (0b10 << 2 * pin_rx);
//	//Half duplex pitaj Cilaga
//	GPIOC->OTYPER |= (0b01 << pin_tx);
//	GPIOC->PUPDR &= ~(0b11 << 2 * pin_tx);
//	GPIOC->PUPDR |= (0b01 << 2 * pin_tx);
//	//Podesavanje AF
//	//AF7
//	const uint8_t alt_function = 8;
//
//	GPIOC->AFR[0] &= ~(0b1111 << 8);
//	GPIOC->AFR[0] |= (alt_function << 24);
//	GPIOC->AFR[0] &= ~(0b1111 << 12);
//	GPIOC->AFR[0] |= (alt_function << 28);
//	//Podesavanja UART-a
//	USART6->CR1 &= ~(0b01 << 12); //8 bitova rec
//	USART6->CR2 &= ~(0b11 << 12); //1 stop bit
//	USART6->BRR &= ~(0xFFFF << 0); //fraction je 0
//	USART6->BRR |= ((91 << 4) | 2); //mantisa (mislim da je to vezano za baudrate)PITAJ CILAGA KAKO SE RACUNA MOZDA JE RAZLICITO ZA DRUGI UART
//	//Ukljucivanje TX i RX pina
//	USART6->CR1 |= ((0b01 << 2) | (0b01 << 3));
//	USART6->CR1 |= (0b01 << 5);
//	//Namestamo sledece na nulu jer hocemo half duplex
//	USART6->CR2 &= ~((0b01 << 14) | (0b01 << 11));
//	USART6->CR3 &= ~((0b01 << 5) | (0b01 << 7)); //uart4 nema ovaj peti bit
//	//USART6->CR3 |= (0b01 << 3); //Ovo se postavlja na jedan za half duplex
//	//Paljenje uart-a
//	USART6->CR1 |= (0b01 << 13);
//
//	const uint8_t interrupt_index = 71;
//	NVIC->ISER[interrupt_index / 32] |= (0b01 << (interrupt_index % 32));
//}
//
//void USART6_IRQHandler(void) {
//	if (USART6->SR & (0b01 << 5)) {
//		write_to_buffer6(USART6->DR);
//		parse_buffer6();
//	}
//}
//
//void USART2_IRQHandler(void) {
////	if (USART2->SR & (0b01 << 5)) {
////		write_to_buffer2(USART2->DR);
////	}
//    if((USART2->SR & (0b01 << 7)) && (USART2->CR1 & (0b01 << 7))) {
//        uint8_t c = 4;
////        if (*sendbuf == 0)
//        USART2->CR1 &= ~(0b01 << 7);
//    	USART2->DR = c;
//    }
//}
//
//void USART1_IRQHandler(void) {
//	if (USART1->SR & (0b01 << 5)) {
//		write_to_buffer1(USART1->DR);
//		//parse_buffer();
//		//provera_bauda();
//	}
//}
//
//void write_to_buffer6(uint8_t data)
//{
//	if (size6 != 30) {
//		buffer6[write_index6] = data;
//		write_index6 = (write_index6 + 1) % 30;
//		size6++;
//	} else {
//		buffer6[write_index6] = data;
//		write_index6 = (write_index6 + 1) % 30;
//		read_index6 = (read_index6 + 1) % 30;
//	}
//}
//
//void write_to_buffer1(uint8_t data)
//{
//	if (size != 30) {
//		buffer[write_index] = data;
//		write_index = (write_index + 1) % 30;
//		size++;
//	} else {
//		buffer[write_index] = data;
//		write_index = (write_index + 1) % 30;
//		read_index = (read_index + 1) % 30;
//	}
//}
//
//void write_to_buffer2(uint8_t data)
//{
//	if (size != 30) {
//		buffer2[write_index2] = data;
//		write_index2 = (write_index2 + 1) % 30;
//		size2++;
//	} else {
//		buffer2[write_index2] = data;
//		write_index2 = (write_index2 + 1) % 30;
//		read_index2 = (read_index2 + 1) % 30;
//	}
//}
//
//
//void uart_send_byte1(uint8_t data) {
//	USART1->DR = data;
//	while (!(USART1->SR & (0b01 << 6))) {
//		__NOP();
//	}
//	USART1->SR &= ~(0b01 << 6);
//}
//
//void uart_send_byte2(uint8_t data) {
//	USART2->CR1 |= (0b01 << 7);
//	//USART2->DR = data;
////	while (!(USART2->SR & (0b01 << 6))) {
////		__NOP();
////	}
////	USART2->SR &= ~(0b01 << 6);
////	USART2->DR = data;
////	USART2->CR1 &= ~(0b01 << 7);
//}
//
//void uart_send_byte6(uint8_t data) {
//	USART6->DR = data;
//	while (!(USART6->SR & (0b01 << 6))) {
//		__NOP();
//	}
//	USART6->SR &= ~(0b01 << 6);
//}
//
//void uart_send_str(char *str) {
//	while (*str != '\0') {
//		uart_send_byte6(*str);
//		str++;
//	}
//}
//
//void move_ax(uint8_t id, uint16_t angle, uint16_t brzina)
//{
//	uint16_t x;
//	uint8_t x_prvih_8;
//	uint8_t x_drugih_8;
//
//	uint16_t v;
//	uint8_t v_prvih_8;
//	uint8_t v_drugih_8;
//
//	uint8_t checksum;
//
//	x = (angle * 1023) / 300;
//
//	x_prvih_8 = x;
//	x_drugih_8 = (x >> 8);
//
//	v = (brzina * 1023) / 114;
//
//	v_prvih_8 = v;
//	v_drugih_8 = (v >> 8);
//
//	checksum = ~(id + 7 + 3 + 0x1e + x_prvih_8 + x_drugih_8 + v_prvih_8 + v_drugih_8);
//
//	uint8_t move[] = {0xff, 0xff, id, 0x07, 0x03, 0x1e, x_prvih_8, x_drugih_8, v_prvih_8, v_drugih_8, checksum};
//
//	for (uint8_t i = 0; i < sizeof(move) / sizeof(  *move); i++) {
//		uart_send_byte1(move[i]);
//	}
//	tajmeri_delay(25);
//}
//
//void read_ax(uint8_t id)
//{
//	uint8_t checksum = 0;
//
//	checksum = ~(id + 4 + 2 + 0x24 + 2);
//
//	uint8_t read[] = { 0xFF, 0xFF, id, 0x04, 0x02, 0x24, 0x02, checksum };
//
//	for (uint8_t i = 0; i < sizeof(read) / sizeof(*read); i++) {
//		uart_send_byte1(read[i]);
//	}
//	tajmeri_delay(25);
//	if (write_index == 0)
//	{
//		p1 = buffer[27];
//		p2 = buffer[28];
//		p = p1|(p2<<8);
//		angle = (p*300) / 1023;
//	}
//	else if (write_index == 1)
//	{
//		p1 = buffer[28];
//		p2 = buffer[29];
//		p = p1|(p2<<8);
//		angle = (p*300) / 1023;
//	}
//	else if (write_index == 2)
//	{
//		p1 = buffer[29];
//		p2 = buffer[0];
//		p = p1|(p2<<8);
//		angle = (p*300) / 1023;
//	}
//	else
//	{
//		p1 = buffer[write_index - 3];
//		p2 = buffer[write_index - 2];
//		p = p1|(p2<<8);
//		angle = (p*300) / 1023;
//	}
//}
//
//static void parse_buffer() {
//	// Prosledjujemo nas volatile static buffer u ovu funkciju tako da ce ga ona upamtiti u trenutku pozivanja funkcije
//	// i nece dozvoliti da nam se tokom koriscenja ove funkcije menjaju vrednosti u bufferu
//	// Ova funkcija moze biti i int tipa pa da vraca vrednosti parametara i errora, ne znam sta nam je cilj
//	// Shvatio sam da ovo nije optimalno, trebali bismo non stop da proveravamo sta se nalazi u bufferu i da upisujemo pristigle poruke negde
//	static uint8_t i = 0;
//	static uint8_t step = 1;
//	//uint16_t checksum;
//	//    uint8_t parametri_sa_ax[30] = {0}; // Ovako izlazi upozorenje variable 'error_sa_ax' set but not used [-Wunused-but-set-variable]
//	//    uint8_t error_sa_ax; // Takodje ovde
//
//	if (size == 0) //proveriti da li buffer nije prazan
//	{
//		return ;
//	}
//
//	// Izgled paketa: Header1 Header2  ID Length Error Param 1 … Param N Checksum
//	// Check Sum = ~ (ID + Length + Error + Parameter1 + ... Parameter N)
//
//	switch (step)
//	{
//		case 1:
//			if (buffer[read_index] == 0xFF) //da li je 255
//			{
//				i = 0;
//				read_index = (read_index + 1) % 30;
//				size--;
//				step = 2;
//			}
//			break;
//		case 2:
//			if (buffer[read_index] == 0xFF)
//			{
//				read_index = (read_index + 1) % 30;
//				size--;
//				step = 3;
//			}
//			else
//			{
//				read_index = (read_index + 1) % 30;
//				size--;
//				step = 1;
//			}
//			break;
//		case 3:
//			arr_AX[0] = buffer[read_index]; //ovde citamo ID
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 4;
//			break;
//		case 4:
//			arr_AX[1] = buffer[read_index]; //ovde citamo len
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 5;
//			break;
//		case 5:
//			arr_AX[2] = buffer[read_index]; //citamo error
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 6;
//			break;
//		case 6:
//			arr_AX[3] = buffer[read_index]; //prvi parametar pozicije
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 7;
//		case 7:
//			arr_AX[4] = buffer[read_index]; //drugi parametar pozicije
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 8;
//			break;
//		case 8:
//			arr_AX[5] = buffer[read_index]; //checksum
//			read_index = (read_index + 1) % 30;
//			size--;
//			i++;
//			step = 9;
//			break;
//		case 9:
//			if (arr_AX [2] == 0)  //Ovako je napravljeno da ako dobijemo eror samo odbacujemo celu poruku umesto da citamo eror
//			{
//				step = 10;
//			}
//			else
//			{
//				step = 1;
//			}
//			break;
//
//		case 10:
//			cs = arr_AX[0] + arr_AX[1] + arr_AX[2] + arr_AX[3] + arr_AX[4];   //ovaj case bi treba da se cita pre nego sto se cita eror (kada napravimo da citamo eror)
//			cs = ~cs;
//			if (cs == arr_AX[5])
//			{
//				step = 11;
//			}
//			else
//			{
//				step = 1;
//			}
//			break;
//
//		case 11:
//			p1 = arr_AX[3];
//			p2 = arr_AX[4];
//
//			p = p1 | (p2<<8);
//
//			angle = p * 300 / 1023;
//
//			step = 1;
//			break;
//	}
//}
//
//static void parse_buffer6 ()
//{
//	static uint8_t step = 1;
//
//	if (size6 == 0)
//	{
//		return;
//	}
//
//	switch (step)
//	{
//		case 1:
//			if (buffer6[read_index6] == 0xFF) //da li je 255
//			{
//				read_index6 = (read_index6 + 1) % 30;
//				size6--;
//				step = 2;
//			}
//			break;
//		case 2:
//			if (buffer6[read_index6] == 0xFF)
//			{
//				read_index6 = (read_index6 + 1) % 30;
//				size6--;
//				step = 3;
//			}
//			else
//			{
//				read_index6 = (read_index6 + 1) % 30;
//				size6--;
//				step = 1;
//			}
//			break;
//		case 3:
//			//id
//			arr_communication[0] = buffer6[read_index6];
//			read_index6 = (read_index6 + 1) % 30;
//			size6--;
//			step = 4;
//			break;
//		case 4:
//			//len
//			arr_communication[1] = buffer6[read_index6];
//			read_index6 = (read_index6 + 1) % 30;
//			size6--;
//			step = 5;
//			break;
//		case 5:
//			//instrukcija
//			arr_communication[2] = buffer6[read_index6];
//			read_index6 = (read_index6 + 1) % 30;
//			size6--;
//			step = 6;
//			break;
//		case 6:
//			//parametar
//			arr_communication[3] = buffer6[read_index6];
//			read_index6 = (read_index6 + 1) % 30;
//			size6--;
//			step = 7;
//			break;
//		case 7:
//			//cs
//			arr_communication[4] = buffer6[read_index6];
//			read_index6 = (read_index6 + 1) % 30;
//			size6--;
//			step = 8;
//			break;
//		case 8:
//			x1 = arr_communication[1];
//			x2 = arr_communication[2];
//			y1 = arr_communication[3];
//			y2 = arr_communication[4];
//			x = (x1<<8)|x2;
//			y = (y1<<8)|y2;
//			step = 1;
//			break;
//	}
//}
//
