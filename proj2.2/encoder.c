/*
 ******************************************************************************
 * @file    proj2.1/encoder.c 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.1 encode command function
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
			ENCODE CMD
 ***********************************/
extern portBASE_TYPE prvEncodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;
	char * pEnd;
	uint8_t i;
	unsigned char xbyte = 0;
	unsigned char sBit[8];
	unsigned char mj=0, mj1=0, mj2=0;
	unsigned char out1[8], out2[8], out3[8];
	unsigned int encodedmessage = 0;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check if string is a single char
	if (strlen(cCmd_string) == 1) {
		xbyte = cCmd_string[0];
	}
	// check if string is hex
	else if (cCmd_string[0] == '0' && (cCmd_string[1] == 'x' || cCmd_string[1] == 'X')) {
		// convert to hex
		xbyte = strtol(cCmd_string, &pEnd, 0);
	}
	else {
		debug_printf("Invalid Entry\r\n");
		return 0;
	}

	// extract bits from byte
	for (i=0; i<8; i++) {
		sBit[i] = !!(xbyte & (1 << i));
	}
	debug_printf("\r\nNum: Out1  Out2  Out3\r\n");
	// calulate all 8 sets of output for 8 inputs
	for (i=0; i<8; i++)	{
		mj = sBit[i];
		out1[i] = mj ^ mj1;
		out2[i] = mj1 ^ mj2;
		out3[i] = mj ^ out2[i];
		mj2 = mj1;
		mj1 = mj;
		debug_printf("%d:    %d     %d     %d\r\n", i+1, out1[i], out2[i], out3[i]);
	}
	for (i=0; i<8; i++)	{
		encodedmessage |= (out3[7-i] << (23-(i*3))) | (out2[7-i] << (22-(i*3))) | (out1[7-i] << (21-(i*3)));
	}
	debug_printf("\n\rEncoded Output: %06x\r\n", encodedmessage);
	mj = 0;
	mj1 = 0;
	mj2 = 0;
	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r");
	return pdFALSE;
}
