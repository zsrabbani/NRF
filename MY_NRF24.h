#include "stm32f4xx_hal.h"  
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

//**** TypeDefs ****//
/* Registers */
typedef enum {
    REG_CONFIG      = 0x00,
    REG_EN_AA       = 0x01,
    REG_EN_RXADDR   = 0x02,
    REG_SETUP_AW   = 0x03,
    REG_SETUP_RETR  = 0x04,
    REG_RF_CH       = 0x05,
    REG_RF_SETUP     = 0x06,
    REG_STATUS      = 0x07,
    REG_OBSERVE_TX  = 0x08,
    REG_CD         = 0x09,
    REG_RX_ADDR_P0  = 0x0A,
    REG_RX_ADDR_P1  = 0x0B,
    REG_RX_ADDR_P2  = 0x0C,
    REG_RX_ADDR_P3  = 0x0D,
    REG_RX_ADDR_P4  = 0x0E,
    REG_RX_ADDR_P5  = 0x0F,
    REG_TX_ADDR     = 0x10,
    REG_RX_PW_P0    = 0x11,
    REG_RX_PW_P1    = 0x12,
    REG_RX_PW_P2    = 0x13,
    REG_RX_PW_P3    = 0x14,
    REG_RX_PW_P4    = 0x15,
    REG_RX_PW_P5    = 0x16,
    REG_FIFO_STATUS = 0x17,
    REG_DYNPD       = 0x1C,
    REG_FEATURE     = 0x1D,
	//command
		CMD_R_REGISTER         = 0x00,
    CMD_W_REGISTER         = 0x20,
    CMD_R_RX_PAYLOAD       = 0x61,
    CMD_W_TX_PAYLOAD       = 0xA0,
    CMD_FLUSH_TX           = 0xE1,
    CMD_FLUSH_RX           = 0xE2,
    CMD_REUSE_TX_PL        = 0xE3,
    CMD_ACTIVATE           = 0x50,
    CMD_R_RX_PL_WID        = 0x60,
    CMD_W_ACK_PAYLOAD      = 0xA8,
    CMD_W_TX_PAYLOAD_NOACK = 0xB0,
    CMD_NOP                = 0xFF,
	//new one
	BIT_RX_DR =6,
    BIT_TX_DS =5,
	BIT_ARD =4,
    BIT_ARC =0,
	BIT_EN_DPL =2,
	BIT_DPL_P5 =5,
    BIT_DPL_P3 =3,
    BIT_DPL_P2 =2,
    BIT_DPL_P1 =1,
	BIT_DPL_P0 =0,
	RF_DR_LOW =5,
    RF_DR_HIGH =3,
    RF_PWR_LOW =1,
    RF_PWR_HIGH =2,
	BIT_EN_CRC =3,
	BIT_CRCO =2,
	BIT_PWR_UP =1,
	BIT_RX_P_NO =1,
	BIT_PRIM_RX =0,
	REG_RPD =0x09,
	BIT_EN_ACK_PAY =1,
    BIT_EN_DYN_ACK =0,
	LNA_HCURR =0,
	BIT_RX_EMPTY =0,
	MASK_RX_DR =6,
    MASK_TX_DS =5,
    MASK_MAX_RT =4,
    BIT_PRIM_RX =0,
    BIT_ENAA_P5 =5,
    BIT_ENAA_P4 =4,
    BIT_ENAA_P3 =3,
	BIT_ENAA_P2 =2,
    BIT_ENAA_P1 =1,
    BIT_ENAA_P0 =0,
    BIT_ERX_P5 =5,
    BIT_ERX_P4 =4,
    BIT_ERX_P3 =3,
    BIT_ERX_P2 =2,
    BIT_ERX_P1 =1,
    BIT_ERX_P0 =0,
    BIT_AW =0,
    BIT_ARD =4,
    BIT_ARC =0,
    BIT_PLL_LOCK =4,
    BIT_RF_DR =3,
    BIT_RF_PWR =6,
    BIT_RX_DR =6,
    BIT_TX_DS =5,
    BIT_MAX_RT =4,
    BIT_TX_FULL =0,
    BIT_PLOS_CNT =4,
    BIT_ARC_CNT =0,
    BIT_TX_REUSE =6,
    BIT_FIFO_FULL =5,
    BIT_TX_EMPTY =4,
    BIT_RX_FULL =1,
	
	static const RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0, 
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
	  REG_RX_PW_P5}

} NRF_RE_COM;

typedef struct {
	
    uint8_t  payload_size;
	bool  wide_band;
	bool p_variant;
	uint64_t pipe0_reading_address;
    bool ack_payload_available; 
	uint8_t ack_payload_length;
	uint8_t payload_size; 

    SPI_HandleTypeDef* nrf24_hspi;
    uint32_t           spi_timeout;
	
	UART_HandleTypeDef nrf24_huart;

    GPIO_TypeDef* nrf24_CSN_PORT;
		GPIO_TypeDef* nrf24_CE_PORT;
    uint16_t      nrf24_CSN_PIN;
    uint16_t      nrf24_CE_PIN;

} nrf24l01_config;

//1. Power Amplifier function, NRF24_setPALevel() 
typedef enum { 
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR 
}rf24_pa_dbm_e ;

//2. NRF24_setDataRate() input
typedef enum { 
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
}rf24_datarate_e;

//3. NRF24_setCRCLength() input
typedef enum { 
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;

//**** Functions prototypes ****//
//Microsecond delay function
void NRF24_Delay_MAX(void);
void NRF24_Delay_MIN(void);
//1. Chip Select function
void NRF24_csn(nrf24l01_config *var ,int mode);
//2. Chip Enable
void NRF24_ce(nrf24l01_config *var ,int level);
//3. Read single byte from a register
uint8_t NRF24_read_register(nrf24l01_config *svar ,NRF_RE_COM *reg);
//4. Read multiple bytes register
void NRF24_read_registerN(nrf24l01_config *svar ,NRF_RE_COM *reg, uint8_t *buf, uint8_t len);
//5. Write single byte register
void NRF24_write_register(nrf24l01_config *svar ,NRF_RE_COM *reg, uint8_t value);
//6. Write multipl bytes register
void NRF24_write_registerN(nrf24l01_config *svar ,NRF_RE_COM *reg, const uint8_t* buf, uint8_t len);
//7. Write transmit payload
void NRF24_write_payload(nrf24l01_config *svar ,NRF_RE_COM *reg ,const void* buf, uint8_t len);
//8. Read receive payload
void NRF24_read_payload(nrf24l01_config *svar ,NRF_RE_COM *reg ,void* buf, uint8_t len);
//9. Flush Tx buffer
void NRF24_flush_tx(nrf24l01_config *svar , NRF_RE_COM *con);
//10. Flush Rx buffer
void NRF24_flush_rx(nrf24l01_config *svar, NRF_RE_COM *com);
//11. Get status register value
uint8_t NRF24_get_status(nrf24l01_config *svar,NRF_RE_COM *com);
//12. Begin function
void NRF24_begin(nrf24l01_config *svar ,NRF_RE_COM *var ,nrf24l01_config *con);
//13. Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(NRF_RE_COM * con);
//14. Stop listening (essential before any write operation)
void NRF24_stopListening(void);
//15. Write(Transmit data), returns true if successfully sent
bool NRF24_write( nrf24l01_config *con,NRF_RE_COM *com,const void* buf, uint8_t len );
//16. Check for available data to read
bool NRF24_available(void);
//17. Read received data
uint8_t NRF24_read( NRF_RE_COM *com,void* buf, uint8_t len );
//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24_openWritingPipe(NRF_RE_COM *com ,uint64_t address);
//19. Open reading pipe
void NRF24_openReadingPipe(nrf24l01_config *con,uint8_t number, uint64_t address);
//20 set transmit retries (rf24_Retries_e) and delay
void NRF24_setRetries(NRF_RE_COM *con,uint8_t delay, uint8_t count);
//21. Set RF channel frequency
void NRF24_setChannel(NRF_RE_COM *com,uint8_t channel);
//22. Set payload size
void NRF24_setPayloadSize(nrf24l01_config *con,uint8_t size);
//23. Get payload size
uint8_t NRF24_getPayloadSize(nrf24l01_config *con);
//24. Get dynamic payload size, of latest packet received
uint8_t NRF24_getDynamicPayloadSize(NRF_RE_COM *com);
//26. Enable dynamic payloads
void NRF24_enableDynamicPayloads(nrf24l01_config *con,NRF_RE_COM *com);
void NRF24_disableDynamicPayloads(nrf24l01_config *con,NRF_RE_COM *com);

//28. Set Auto Ack for all
void NRF24_setAutoAck(bool enable);
//29. Set Auto Ack for certain pipe
void NRF24_setAutoAckPipe(NRF_RE_COM *com,uint8_t pipe, bool enable );
//30. Set transmit power level
void NRF24_setPALevel( NRF_RE_COM *com,rf24_pa_dbm_e *level ) ;
//31. Get transmit power level
rf24_pa_dbm_e NRF24_getPALevel(NRF_RE_COM *com,rf24_pa_dbm_e *level ) ;
//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24_setDataRate(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed);
//33. Get data rate
rf24_datarate_e NRF24_getDataRate(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed  );
//34. Set crc length (disable, 8-bits or 16-bits)
void NRF24_setCRCLength(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed ,rf24_crclength_e *length);
//35. Get CRC length
rf24_crclength_e NRF24_getCRCLength(nrf24l01_config *con,NRF_RE_COM *com,rf24_crclength_e *length);
//36. Disable CRC
void NRF24_disableCRC(NRF_RE_COM *com) ;
//37. power up
void NRF24_powerUp(NRF_RE_COM *com) ;
//38. power down
void NRF24_powerDown(NRF_RE_COM *com);
//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24_availablePipe(NRF_RE_COM *com,uint8_t* pipe_num);
//40. Start write (for IRQ mode)
void NRF24_startWrite( NRF_RE_COM *com,const void* buf, uint8_t len );
//41. Write acknowledge payload
void NRF24_writeAckPayload(NRF_RE_COM *com,uint8_t pipe, const void* buf, uint8_t len);
//42. Check if an Ack payload is available
bool NRF24_isAckPayloadAvailable(nrf24l01_config *con);
//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24_testCarrier(NRF_RE_COM *com);
//45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24_testRPD(NRF_RE_COM *com);
//46. Reset Status
void NRF24_resetStatus(NRF_RE_COM *com);
//47. ACTIVATE cmd
void NRF24_ACTIVATE_cmd(nrf24l01_config *con,NRF_RE_COM *com);
//48. Get AckPayload Size
uint8_t NRF24_GetAckPayloadSize(nrf24l01_config *con);

//**********  DEBUG Functions **********//
//1. Print radio settings
void printRadioSettings(NRF_RE_COM *val);
//2. Print Status 
void printStatusReg(NRF_RE_COM *val);
//3. Print Config 
void printConfigReg(NRF_RE_COM *con);
//5. FIFO Status
void printFIFOstatus(NRF_RE_COM *con);
// min max _bool:
uint8_t MAX(uint8_t x, uint8_t y);
uint8_t MIN(uint8_t x, uint8_t y);
bool _BOOL(uint8_t x);
uint8_t _BV(uint8_t x);
NRF_RE_COM _BV(NRF_RE_COM *reg);



