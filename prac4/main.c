/**
  ******************************************************************************
  * @file    ex10_spi/main.c 
  * @author  MDS
  * @date    10-January-2014
  * @brief   SPI Read 32-bit Register nrf24l01plus status (0x07) register
  *			 NOTE: This example does not send or transmit with the nrf24l01plus.
  *			 REFERENCE: nrf24l01p_datasheet.pdf
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include "nrf24l01plus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//for checking against
uint8_t packet_type[] = {0x20};
uint8_t des_addr[] = {0x77, 0x56, 0x34, 0x12};
uint8_t src_addr[] = {0x28, 0x92, 0x96, 0x42};

uint8_t packet_header[] = {0x20, 0x77, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42};
uint8_t tx_buffer[32];	/* buffer initialised to 32 bytes (max length) */
uint8_t rx_buffer[32];
uint8_t decoded[16];
uint8_t payload[5];
int payload_counter = 0;
int i, j = 0, k;
uint8_t RxByte;
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO unsigned long nCount);
void HardwareInit();
uint8_t spi_sendbyte(uint8_t sendbyte);
uint8_t hamming_hbyte_encoder(uint8_t input);
uint8_t hamming_hbyte_decoder(uint8_t input1, uint8_t input2);
uint8_t hamming_error(uint8_t input);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void) {
	
	NP2_boardinit();
	HardwareInit();
	
	/* Initialise NRF24l01plus */ 
	nrf24l01plus_init();

	/* Write TX_ADDR */
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);		//Set Chip Select low
	spi_sendbyte(0x30);									//Command Write to TX_ADDR Register
	for (i = 0; i < 4; i++) {
		spi_sendbyte(des_addr[i]);
	}
	spi_sendbyte(0x00);
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);		//Set Chip Select high
	
	/* Write RF_CH */
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	spi_sendbyte(0x25);									//Command Write to RF_CH Register
	spi_sendbyte(50);									//Set to rf_ch
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	
/********************************************************Might Be NEEDED********************************************************/
	/* Write RX_ADDR_P1
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);	
	spi_sendbyte(0x2A);								
	for (i = 0; i < 4; i++)
	{
		spi_sendbyte(des_addr[i]);
	}
	spi_sendbyte(0x00);
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	*/
	
	/* Write EN_RXADDR
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	spi_sendbyte(0x22);							
	spi_sendbyte(0b00000001);
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	*/
/********************************************************Might Be NEEDED********************************************************/
	
	nrf24l01plus_mode_rx();
	
	for (i = 0; i < 32; i++) {
		rx_buffer[i] = 0; //clear packetbuffer
	}

	/* Main Processing Loop */
    while (1) {

		//encode packet headers and put to packet buffer
		for (i=0; i<9; i++) {
			tx_buffer[(2*i)] = hamming_hbyte_encoder(packet_header[i] & 0xF); //packetbuffer 0, 2, 4, 6, 8, 10, 12, 14, 16
			tx_buffer[1+(2*i)] = hamming_hbyte_encoder(packet_header[i] >> 4); //packetbuffer 1, 3, 5, 7, 8, 11, 13, 15, 17
		}
		
		for (i=0; i<2; i++) {
			tx_buffer[28+(2*i)] = hamming_hbyte_encoder(0x00); //packetbuffer 28, 30
			tx_buffer[29+(2*i)] = hamming_hbyte_encoder(0x00); //packetbuffer 29, 31
		}
		
		//read keystroke and put to payload
		if (VCP_getchar(&RxByte)) {
			if (RxByte == 13) {
				payload_counter = 5;
			}
			else {
				payload[payload_counter] = RxByte;
				payload_counter++;
			}
				
			if (payload_counter == 5) { //encode payload and put to packet buffer
				debug_printf("Sending ");
				for (i = 0; i < 5; i++) {
					debug_printf("%c", payload[i]);
					tx_buffer[18+(2*i)] = hamming_hbyte_encoder(payload[i] & 0xF); //packetbuffer 18, 20, 22, 24, 26
					tx_buffer[19+(2*i)] = hamming_hbyte_encoder(payload[i] >> 4); //packetbuffer 19, 21, 23, 25, 27
					payload[i] = 0; //clear payload
				}
				debug_printf("\r\n");
				nrf24l01plus_send_packet(tx_buffer);
				Delay(50000);
				nrf24l01plus_mode_rx(); //change to rx mode
				debug_printf("encoded\r\n");
				for (i = 0; i < 32; i++) {
					debug_printf("%x ", tx_buffer[i]);
				}
				debug_printf("\r\n\r\n");
				
				/*****************************checking if decoder works***************************
				debug_printf("decoded \r\n");
				for (i = 0; i < 16; i++) {
					decoded[i] = hamming_hbyte_decoder(tx_buffer[2*i], tx_buffer[2*i+1]);
				}
				for (i = 0; i < 4; i++) {
					if (decoded[i+5] == src_addr[i]) {
						j++;
					}
					if (j == 4) {
						j = 0;
						for (k = 0; k < 16; k++) {
							debug_printf("%x ", decoded[k]);
						}
					}
				}
				debug_printf("\r\n");
				debug_printf("\r\n");**/
				
				for (i = 0; i < 32; i++) {
					tx_buffer[i] = 0; //clear packetbuffer
				}
				payload_counter = 0;
			}
		}
		

		/* Check for received packet and print if packet is received */
		if (nrf24l01plus_receive_packet(rx_buffer) == 1) {
			debug_printf("Received: ");			
			for (i = 0; i < 32; i++ ) {
				debug_printf("%x ", rx_buffer[i]);
			}
			debug_printf("\r\ndecoded: ");
			for (i = 0; i < 16; i++) {
				decoded[i] = hamming_hbyte_decoder(rx_buffer[2*i], rx_buffer[2*i+1]);
				debug_printf("%x ", decoded[i]);
			}
			debug_printf("\r\n");
			for (i = 0; i < 4; i++) {
				if (decoded[i+1] == src_addr[i]) {
					j++;
				}
				if (j == 4) {
					for (k = 9; k < 14; k++) {
						debug_printf("%c ", decoded[k]);
					}
					j = 0;
				}
			}
			for (i = 0; i < 16; i++) {
				decoded[k] = 0; //clear decode buffer
			}
			debug_printf("\r\n\r\n");
			
			for (i = 0; i < 32; i++) {
				rx_buffer[i] = 0; //clear rx_buffer
			}
		}
    	NP2_LEDToggle();	//Toggle LED on/off
    	Delay(50000);	//Delay function
  	}

}

/**
  * @brief  Initialise hardware modules
  * @param  None
  * @retval None
  */
void HardwareInit() {
	
	GPIO_InitTypeDef GPIO_spi;	
	SPI_InitTypeDef	NP2_spiInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

	RCC_AHB1PeriphClockCmd(NP2_D10_GPIO_CLK, ENABLE);

	/* Set SPI clodk */
	RCC_APB1PeriphClockCmd(NP2_SPI_CLK, ENABLE);
	
	/* SPI SCK pin configuration */
  	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
  	GPIO_spi.GPIO_OType = GPIO_OType_PP;
  	GPIO_spi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_DOWN;	//Pull down resistor
  	GPIO_spi.GPIO_Pin = NP2_SPI_SCK_PIN;
  	GPIO_Init(NP2_SPI_SCK_GPIO_PORT, &GPIO_spi);

  	/* SPI MISO pin configuration */
	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
	GPIO_spi.GPIO_OType = GPIO_OType_PP;
	GPIO_spi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_UP;		//Pull up resistor
  	GPIO_spi.GPIO_Pin = NP2_SPI_MISO_PIN;
  	GPIO_Init(NP2_SPI_MISO_GPIO_PORT, &GPIO_spi);

	/* SPI  MOSI pin configuration */
	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
	GPIO_spi.GPIO_OType = GPIO_OType_PP;	
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_DOWN;	//Pull down resistor	
  	GPIO_spi.GPIO_Pin =  NP2_SPI_MOSI_PIN;
  	GPIO_Init(NP2_SPI_MOSI_GPIO_PORT, &GPIO_spi);

	/* Connect SPI pins */
	GPIO_PinAFConfig(NP2_SPI_SCK_GPIO_PORT, NP2_SPI_SCK_SOURCE, NP2_SPI_SCK_AF);
	GPIO_PinAFConfig(NP2_SPI_MISO_GPIO_PORT, NP2_SPI_MISO_SOURCE, NP2_SPI_MISO_AF);
	GPIO_PinAFConfig(NP2_SPI_MOSI_GPIO_PORT, NP2_SPI_MOSI_SOURCE, NP2_SPI_MOSI_AF);

	/* SPI configuration */
  	SPI_I2S_DeInit(NP2_SPI);

  	NP2_spiInitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  	NP2_spiInitStruct.SPI_DataSize = SPI_DataSize_8b;
  	NP2_spiInitStruct.SPI_CPOL = SPI_CPOL_Low;	//SPI_CPOL_Low;
  	NP2_spiInitStruct.SPI_CPHA = SPI_CPHA_1Edge;	//SPI_CPHA_1Edge;
  	NP2_spiInitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;	//SPI_NSS_Soft;
  	NP2_spiInitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//256;
  	NP2_spiInitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  	NP2_spiInitStruct.SPI_CRCPolynomial = 0;	//7;
  	NP2_spiInitStruct.SPI_Mode = SPI_Mode_Master;
  	SPI_Init(NP2_SPI, &NP2_spiInitStruct);

  	/* Enable NP2 external SPI */
  	SPI_Cmd(NP2_SPI, ENABLE);

	/* Configure GPIO PIN for Chip select */
  	GPIO_InitStructure.GPIO_Pin = NP2_SPI_CS_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(NP2_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Set chip select high */
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
}

uint8_t hamming_hbyte_encoder(uint8_t in) {

	uint8_t d0, d1, d2, d3;
	uint8_t p0 = 0, h0, h1, h2;
	uint8_t z;
	uint8_t out;
	
	/* extract bits */
	d0 = !!(in & 0x1);
	d1 = !!(in & 0x2);
	d2 = !!(in & 0x4);
	d3 = !!(in & 0x8);
	
	/* calculate hamming parity bits */
	h0 = d1 ^ d2 ^ d3;
	h1 = d0 ^ d2 ^ d3;
	h2 = d0 ^ d1 ^ d3;
	
	/* generate out byte without parity bit P0 */
	out = (h0 << 1) | (h1 << 2) | (h2 << 3) |
		(d0 << 4) | (d1 << 5) | (d2 << 6) | (d3 << 7);

	/* calculate even parity bit */
	for (z = 1; z<8; z++)		
		p0 = p0 ^ !!(out & (1 << z));
	
	out |= p0;

	return(out);

}

uint8_t hamming_hbyte_decoder(uint8_t in1, uint8_t in2) {

	uint8_t out;
	
	out = hamming_error(in1) | (hamming_error(in2) << 4);
	
	return(out);
}

uint8_t hamming_error(uint8_t in) {
	
	uint8_t ex[8];
	uint8_t syn1, syn2, syn3, syndrome, pcheck;
	uint8_t syndrome_array[] = {100, 010, 001, 011, 101, 110, 111};
	uint8_t z;
	uint8_t out;
	
	/* extract bits */
	ex[7] = !!(in & 0x1);		//p0
	ex[6] = !!(in & 0x2);		//h0
	ex[5] = !!(in & 0x4);		//h1
	ex[4] = !!(in & 0x8);		//h2
	ex[3] = !!(in & 0x10);		//d0
	ex[2] = !!(in & 0x20);		//d1
	ex[1] = !!(in & 0x40);		//d2
	ex[0] = !!(in & 0x80);		//d3
	
	syn1 = ex[0] ^ ex[1] ^ ex[2] ^ ex[6];
	syn2 = ex[0] ^ ex[1] ^ ex[3] ^ ex[5];
	syn3 = ex[0] ^ ex[2] ^ ex[3] ^ ex[4];
	syndrome = (syn1 * 100) + (syn2 * 10) + syn3;
	
	for (z = 0; z < 7; z++) {
		k = 0;
		if (syndrome == 000) {
			continue;
		}
		else if (syndrome == syndrome_array[z]) {
			ex[z] = ~ex[z];
			k++;
		}
	}
	
	pcheck = ex[0] ^ ex[1] ^ ex[2] ^ ex[3] ^ ex[4] ^ ex[5] ^ ex[6] ^ ex[7];
	
	out = (ex[0] << 3) | (ex[1] << 2) | (ex[2] << 1) | ex[3];
	
	return out;
	
}
/**
  * @brief  Send byte through SPI.
  * @param  sendbyte: byte to be transmitted via SPI.
  * @retval None
  */
uint8_t spi_sendbyte(uint8_t sendbyte) {

	uint8_t readbyte;

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_TXE) == RESET);
	
	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(NP2_SPI, sendbyte);

	/* Wait while busy flag is set */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_BSY) == SET);
	
	/* Wait to receive a Byte */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the Byte read from the SPI bus */
	readbyte = (uint8_t)SPI_I2S_ReceiveData(NP2_SPI);

	/* Return the Byte read from the SPI bus */
	return readbyte;
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO unsigned long nCount) {
  while(nCount--) {
  }
}

