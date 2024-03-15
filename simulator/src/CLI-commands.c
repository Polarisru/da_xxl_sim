/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

extern xQueueHandle xQueue;

/*
 * Defines a command that expects exactly three parameters.  Each of the three
 * parameter are echoed back one at a time.
 */
static BaseType_t prvSendMsgCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static BaseType_t prvSendMsg2Command(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static BaseType_t prvSetVoltageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static BaseType_t prvSetCurrentCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

static BaseType_t prvSetTargetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);


/* Structure that defines the "can" command line command.  This adds CAN command
to a message queue. */
static const CLI_Command_Definition_t xSendMsgCAN =
{
	"can",
	"\r\ncan <id> <data>:\r\n Send CAN message to queue\r\n\r\n",
	prvSendMsgCommand, /* The function to run. */
	2 /* Two parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t xSendMsgRS485 =
{
	"rs485",
	"\r\nrs485 <cmd> <id> <arg1>:\r\n Send RS485 message to queue\r\n\r\n",
	prvSendMsg2Command, /* The function to run. */
	4 /* Four parameters are expected, which can take any value. */
};

static const CLI_Command_Definition_t xSetVoltage =
{
	"volt",
	"\r\nvolt <voltage1> <voltage2>:\r\n Set input voltages, V\r\n\r\n",
	prvSetVoltageCommand, /* The function to run. */
	2 /* Two parameters are expected */
};

static const CLI_Command_Definition_t xSetCurrent =
{
	"current",
	"\r\ncurrent <current>:\r\n Set current value, A\r\n\r\n",
	prvSetCurrentCommand, /* The function to run. */
	1 /* One parameter is expected */
};

static const CLI_Command_Definition_t xSetTarget =
{
	"target",
	"\r\ntarget <angle>:\r\n Set target position, °\r\n\r\n",
	prvSetTargetCommand, /* The function to run. */
	1 /* One parameter is expected */
};

void vRegisterCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand(&xSendMsgCAN);
	FreeRTOS_CLIRegisterCommand(&xSendMsgRS485);
}
static BaseType_t prvSendMsgCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  const char *pcParameter1;
  BaseType_t lParameter1tringLength;
  //static BaseType_t lParameterNumber = 0;
  int i;
  unsigned long ulValueToSend;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	//( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	pcParameter1 = FreeRTOS_CLIGetParameter
	(
		pcCommandString,
		1,
		&lParameter1tringLength
	);

	i = atoi(pcParameter1);
	ulValueToSend = (unsigned long)i;
	xQueueSend(xQueue, &ulValueToSend, 0U);

	//pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString,
	//	2,
	//	&xParameter2StringLength);

	*pcWriteBuffer = NULL;

	return pdFALSE;
}

static BaseType_t prvSendMsg2Command(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  const char *pcParameter1;
  BaseType_t lParameter1tringLength;
  //static BaseType_t lParameterNumber = 0;
  int i;
  unsigned long ulValueToSend;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	//( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	pcParameter1 = FreeRTOS_CLIGetParameter
	(
		pcCommandString,
		1,
		&lParameter1tringLength
	);

	i = atoi(pcParameter1);
	ulValueToSend = (unsigned long)i;
	xQueueSend(xQueue, &ulValueToSend, 0U);

	//pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString,
	//	2,
	//	&xParameter2StringLength);

	*pcWriteBuffer = (char*)NULL;

	return pdFALSE;
}

static BaseType_t prvSetVoltageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  (void)pcWriteBuffer;
  (void)xWriteBufferLen;
  (void)pcCommandString;
  return pdFALSE;
}

static BaseType_t prvSetCurrentCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  (void)pcWriteBuffer;
  (void)xWriteBufferLen;
  (void)pcCommandString;
  return pdFALSE;
}

static BaseType_t prvSetTargetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
  (void)pcWriteBuffer;
  (void)xWriteBufferLen;
  (void)pcCommandString;
  return pdFALSE;
}
