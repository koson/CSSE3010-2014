/*
 ******************************************************************************
 * @file    proj2.1/decoder.c 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.1 decode command function
 ******************************************************************************
 */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "main.h"

/***********************************
			DECODE CMD
 ***********************************/
extern portBASE_TYPE prvDecodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;
	uint32_t xbyte = 0;
	char * pEnd;
	uint8_t inputbits[24];
	uint8_t inputbytes[8];
	uint8_t hamming = 0;
	uint8_t hamming_array[8];
	uint8_t totalhamming_array[8];
	uint8_t node[4] = {0, 0, 0, 0};
	uint8_t allpath[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t path[4] = {0, 1, 2, 3};
	int8_t i, j;
	// array of possible hamming output
	uint8_t check[] = {0, 3, 5, 6, 7, 4, 2, 1};
	uint8_t smallest = 0;
	uint8_t smallest_location = 0;
	uint8_t outputbits[8];
	uint8_t outputbyte = 0;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check if string is hex
	if (cCmd_string[0] == '0' && (cCmd_string[1] == 'x' || cCmd_string[1] == 'X')) {
		xbyte = strtol(cCmd_string, &pEnd, 0);
	}
	else {
		debug_printf("Invalid Entry\r\n");
		return 0;
	}

	//xbyte ^= 0x8080; // force error

	debug_printf("\r\nNum: Input\r\n");

	// extract bits from byte
	for (i = 0; i < 24; i++) {
		inputbits[i] = ((xbyte >> i) & 1);
	}

	// put bits into 3bits input
	for (i = 0; i < 8; i++) {
		inputbytes[i] = (inputbits[i*3] << 2 | inputbits[i*3+1] << 1 | inputbits[i*3+2]);
		debug_printf("%d:    %d%d%d\r\n", i+1, inputbits[i*3], inputbits[i*3+1], inputbits[i*3+2]);
	}

	// run through all inputbytes
	for (i = 0; i < 8; i++) {
		// check hamming error and record hamming distance
		for (j = 0; j < 8; j++) {
			hamming = inputbytes[i] ^ check[j];
			hamming_array[j] = (hamming & 1) + ((hamming & 2) >> 1) + ((hamming & 4) >> 2);
		}

		debug_printf("total hamming array: ");
		// sum hamming distance with previous
		for (j = 0; j < 2; j++) {
			totalhamming_array[4*j] = hamming_array[4*j] + node[j];
			totalhamming_array[(4*j)+1] = hamming_array[(4*j)+1] + node[j+2];
			totalhamming_array[(4*j)+2] = hamming_array[(4*j)+2] + node[j];
			totalhamming_array[(4*j)+3] = hamming_array[(4*j)+3] + node[j+2];
			if (i == 0) {
				totalhamming_array[1] = 9;
				totalhamming_array[3] = 9;
				totalhamming_array[4] = 0;
				totalhamming_array[5] = 0;
				totalhamming_array[6] = 0;
				totalhamming_array[7] = 0;
			}
			if (i == 1) {
				totalhamming_array[1] = 9;
				totalhamming_array[3] = 9;
				totalhamming_array[5] = 9;
				totalhamming_array[7] = 9;
			}
			debug_printf("%d", totalhamming_array[4*j]);
			debug_printf("%d", totalhamming_array[(4*j)+1]);
			debug_printf("%d", totalhamming_array[(4*j)+2]);
			debug_printf("%d", totalhamming_array[(4*j)+3]);
		}
		debug_printf("\r\n");
		// record and store path
		if (i >= 2) {
			allpath[0] = path[0] << 1;
			allpath[1] = path[2] << 1;
			allpath[2] = (path[0] << 1) | 1;
			allpath[3] = (path[2] << 1) | 1;
			allpath[4] = path[1] << 1;
			allpath[5] = path[3] << 1;
			allpath[6] = (path[1] << 1) | 1;
			allpath[7] = (path[3] << 1) | 1;
		}

		debug_printf("node hamming array: ");
		// check hamming distance and delete biggest error
		for (j = 0; j < 4; j++)	{
			if (totalhamming_array[2*j] <= totalhamming_array[(2*j)+1]) {
				node[j] = totalhamming_array[2*j];
				if (i >= 2) {
					path[j] = allpath[2*j];
				}
			}
			else {
				node[j] = totalhamming_array[(2*j)+1];
				if (i >= 2) {
					path[j] = allpath[(2*j)+1];
				}
			}
			debug_printf("%d", node[j]);
		}
		debug_printf("\r\n");
	}

	// check for smallest error distance on node
	smallest = node[0];
	for (i = 0; i < 4; i++)
	{
		if (node[i] < smallest ) {
			smallest = node[i];
			smallest_location = i;
		}
	}

	// select the shortest path and used the data, extract the bits
	for (i = 0; i < 8; i++) {
		outputbits[i] = ((path[smallest_location] >> i) & 1);
	}

	// put the bits into a output byte
	for (i = 0; i < 8; i++) {
		outputbyte |= (outputbits[7-i] << i);
	}

	debug_printf("\n\rDecoded Output: 0x%02x in Hex or %c in Character\r\n", outputbyte, outputbyte);

	memset(inputbits, 0, sizeof(inputbits));

	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r");

	return pdFALSE;
}