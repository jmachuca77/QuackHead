///////////////////////////////////
// CONTROL BOARD PIN OUT
///////////////////////////////////
// Only change if you using a
// different PCB
///////////////////////////////////

#define RIGHT_EAR_ENC_A 36
#define RIGHT_EAR_ENC_B 39
#define LEFT_EAR_ENC_A  34
#define LEFT_EAR_ENC_B  35
#define LEFT_EYE_PIN    32
#define RIGHT_EYE_PIN   33

#define SD_MISO_PIN  	19
#define SD_SCK_PIN   	18
#define SD_CS_PIN    	5
#define SD_MOSI_PIN  	23

#define I2S_DOUT_PIN 	25  // I2S-DIN audio output
#define I2S_LRC_PIN  	26  // I2S-LRCK audio output
#define I2S_BCLK_PIN 	27  // I2S-BCK audio output

#define RXDX2_PIN       14
#define TXDX2_PIN       13
#define LCD_DC_PIN      15

#define RS_RTS_PIN      12
#define TXD1_PIN        2
#define RXD1_PIN        4

#define SDA_PIN         21
#define SCL_PIN         22

#define RS_SERIAL       Serial1
#define EXT_SERIAL    	Serial2

#define RS_SERIAL_INIT(baud)    RS_SERIAL.begin(baud, SERIAL_8N1, RXD1_PIN, TXD1_PIN)
#define EXT_SERIAL_INIT(baud)	EXT_SERIAL.begin(baud, SERIAL_8N1, RXDX2_PIN, TXDX2_PIN)
