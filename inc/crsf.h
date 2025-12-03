#ifndef CRSF_PROTOCOL
#define CRSF_PROTOCOL
#include<stdio.h>
#include<pico/stdlib.h>

// ---------------------------------------
// Defines
// ---------------------------------------
#define CRSF_BAUD_RATE 420000
#define CRSF_MAX_CHANNEL_SIZE 11
#define CRSF_MAX_NUMBER_OF_CHANNELS 16
#define CRSF_SYNC 0xC8
#define CRSF_TYPE_RC  0x16
#define CRC_POLY 0xD5

// ---------------------------------------
// state machine enum
// ---------------------------------------
typedef enum 
{
	state_wait,
	state_check_lenght,
	state_read_payload
}state;
// ---------------------------------------
// CRSF Struct
// ---------------------------------------
typedef struct 
{
	uart_inst_t* UartCRSFPort;
	uint8_t UartTxPin;
	uint8_t UartRxPin;
	
	uint16_t rawData[16]; // raw data for calculations 
	uint16_t PWMData[16]; // data for PWM signals to motors
	uint8_t frame[26]; // max frame size 
	uint8_t lut[256]; // look-up table for crc8 calculations 
	state state; 
	uint8_t byteID; // current id of byte in packet
	uint8_t packetLen; // lenght of packet that is recived 
}crsf_data;



// ---------------------------------------
// Prototypes
// ---------------------------------------

void CRSF_UartInit(crsf_data* CRSF);
void CRSF_Init(crsf_data* CRSF);
void CRSF_StateMachine(crsf_data* CRSF);
void CRSF_ChannelMaping(crsf_data* CRSF);
void CRSF_UnpackPayloadData(crsf_data* CRSF);
void CRSF_CRC8LutInit(crsf_data* CRSF);
uint8_t CRSF_CRC8Check(crsf_data* CRSF);



#endif