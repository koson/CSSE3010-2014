/*
 ******************************************************************************
 * @file    proj2.1/main.h 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.1 main program header file
 ******************************************************************************
 */ 

#ifndef __DECODER_H
#define __DECODER_H

extern portBASE_TYPE prvDecodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvEncodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern void PanTiltTask(void * pvParameters);
extern void DistanceTask(void * pvParameters);
extern void CLITask(void * pvParameters);

#endif /* FREERTOS_CONFIG_H */