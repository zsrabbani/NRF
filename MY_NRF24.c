#include "MY_NRF24.h"

uint8_t MAX(uint8_t x , uint8_t y)
{
	if (x > y)
		return x;
	else
		return y;
}

uint8_t MIN(uint8_t x , uint8_t y)
{
	if (x < y)
		return x;
	else
		return y;
}

bool _BOOL(uint8_t x)
{
if (x > 0)
	return true;
else 
	return false;
}
uint8_t _BV(uint8_t x)
{
	x = 1<<(x);
	return x;
}
NRF_RE_COM _BV(NRF_RE_COM *reg)
{
	x = 1<<(x);
	return x;
}

//**** Functions prototypes ****//
//Microsecond delay function
void NRF24_Delay_MAX(void)
{
	HAL_Delay(1.5);
}
void NRF24_Delay_MIN(void)
{
	HAL_Delay(0.015);
}
//1. Chip Select function
void NRF24_csn(nrf24l01_config *var ,int state)
{
    if(state) HAL_GPIO_WritePin(nrf24l01_config->nrf24_CSN_PORT, nrf24l01_config->nrf24_CSN_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(nrf24l01_config->nrf24_CSN_PORT, nrf24l01_config->nrf24_CSN_PIN, GPIO_PIN_RESET);
}
//2. Chip Enable
void NRF24_ce(nrf24l01_config *var ,int state)
{
    if(state) HAL_GPIO_WritePin(nrf24l01_config->nrf24_CE_PORT, nrf24l01_config->nrf24_CE_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(nrf24l01_config->nrf24_CE_PORT, nrf24l01_config->nrf24_CE_PIN, GPIO_PIN_RESET);
}
//3. Read single byte from a register
uint8_t NRF24_read_register(nrf24l01_config* svar ,NRF_RE_COM* reg)
{
    uint8_t spiBuf[3];
    uint8_t retData;
    //Put CSN low
    NRF24_csn(svar,0);
    //Transmit register address
    spiBuf[0] = reg&0x1F;
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), spiBuf, 1, 100);
    //Receive data
    HAL_SPI_Receive(&(nrf24l01_config->nrf24_hspi), &spiBuf[1], 1, 100);
    retData = spiBuf[1];
    //Bring CSN high
    NRF24_csn(svar,1);
    return retData;
}
//4. Read multiple bytes register
void NRF24_read_registerN(nrf24l01_config *svar ,NRF_RE_COM *reg, uint8_t *buf, uint8_t len)
{
    uint8_t spiBuf[3];
    //Put CSN low
    NRF24_csn(svar,0);
    //Transmit register address
    spiBuf[0] = reg&0x1F;
    //spiStatus = NRF24_SPI_Write(spiBuf, 1);
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), spiBuf, 1, 100);
    //Receive data
    HAL_SPI_Receive(&(nrf24l01_config->nrf24_hspi), buf, len, 100);
    //Bring CSN high
    NRF24_csn(svar,1);
}
//5. Write single byte register
void NRF24_write_register(nrf24l01_config *svar ,NRF_RE_COM *reg, uint8_t value)
{
    uint8_t spiBuf[3];
    //Put CSN low
    NRF24_csn(svar , 0);
    //Transmit register address and data
    spiBuf[0] = reg|0x20;
    spiBuf[1] = value;
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), spiBuf, 2, 100);
    //Bring CSN high
    NRF24_csn(svar , 1);
}
//6. Write multiple bytes register
void NRF24_write_registerN(nrf24l01_config *svar ,NRF_RE_COM *reg, const uint8_t* buf, uint8_t len)
{
    uint8_t spiBuf[3];
    //Put CSN low
    NRF24_csn(svar , 0);
    //Transmit register address and data
    spiBuf[0] = reg|0x20;
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), spiBuf, 1, 100);
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), (uint8_t*)buf, len, 100);
    //Bring CSN high
    NRF24_csn(svar , 1);
}
//7. Write transmit payload
void NRF24_write_payload(nrf24l01_config *svar ,NRF_RE_COM *reg ,const void* buf, uint8_t len)
{
    uint8_t wrPayloadCmd;
    //Bring CSN low
    NRF24_csn(svar , 0);
    //Send Write Tx payload command followed by pbuf data
    wrPayloadCmd = (uint8_t)&(NRF_RE_COM->CMD_W_TX_PAYLOAD);
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), &wrPayloadCmd, 1, 100);
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), (uint8_t *)buf, len, 100);
    //Bring CSN high
    NRF24_csn(svar , 1);
}
//8. Read receive payload
void NRF24_read_payload(nrf24l01_config *svar ,NRF_RE_COM *reg ,void* buf, uint8_t len)
{
    uint8_t cmdRxBuf;
    //Get data length using payload size
    uint8_t data_len = MIN(len, NRF24_getPayloadSize(svar));
    //Read data from Rx payload buffer
    NRF24_csn(svar , 0);
    cmdRxBuf = (uint8_t)&(NRF_RE_COM->CMD_R_RX_PAYLOAD);
    HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), &cmdRxBuf, 1, 100);
    HAL_SPI_Receive(&(nrf24l01_config->nrf24_hspi), buf, data_len, 100);
    NRF24_csn(svar , 1);
}
//9. Flush Tx buffer
void NRF24_flush_tx(nrf24l01_config *svar, NRF_RE_COM *com)
{
    NRF24_write_register(svar , NRF_RE_COM->CMD_FLUSH_TX, 0xFF);
}
//10. Flush Rx buffer
void NRF24_flush_rx(nrf24l01_config *svar , NRF_RE_COM *com)
{
    NRF24_write_register(svar , NRF_RE_COM->CMD_FLUSH_RX, 0xFF);
}
//11. Get status register value
uint8_t NRF24_get_status(nrf24l01_config *svar,NRF_RE_COM *com)
{
    uint8_t statReg;
    statReg = NRF24_read_register(svar , NRF_RE_COM->REG_STATUS);
    return statReg;
}

//12. Begin function
void NRF24_begin(nrf24l01_config *svar ,NRF_RE_COM *var ,nrf24l01_config *con)
{
    //Put pins to idle state
    NRF24_csn(svar , 1);
    NRF24_ce(svar , 0);
    //5 ms initial delay
    HAL_Delay(5);
    //**** Soft Reset Registers default values ****//
    NRF24_write_register(svar ,NRF_RE_COM->REG_CONFIG, 0x08); //crc enable
    NRF24_write_register(svar ,NRF_RE_COM->REG_EN_AA, 0x3f); //enable autoack for all channels
    NRF24_write_register(svar ,NRF_RE_COM->REG_EN_RXADDR, 0x03); //enable pipe 0,1
    NRF24_write_register(svar ,NRF_RE_COM->REG_SETUP_AW, 0x03);//5 byte adress width
    NRF24_write_register(svar ,NRF_RE_COM->REG_SETUP_RETR, 0x03);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RF_CH, 0x02);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RF_SETUP, 0x0f);
    NRF24_write_register(svar ,NRF_RE_COM->REG_STATUS, 0x0e);
    NRF24_write_register(svar ,NRF_RE_COM->REG_OBSERVE_TX, 0x00);
    NRF24_write_register(svar ,NRF_RE_COM->REG_CD, 0x00);
   
    uint8_t pipeAddrVar[6];
    pipeAddrVar[4]=0xE7; 
    pipeAddrVar[3]=0xE7; 
    pipeAddrVar[2]=0xE7; 
    pipeAddrVar[1]=0xE7; 
    pipeAddrVar[0]=0xE7; 
    NRF24_write_registerN(svar ,NRF_RE_COM->REG_RX_ADDR_P0, pipeAddrVar, 5);
    pipeAddrVar[4]=0xC2; 
    pipeAddrVar[3]=0xC2; 
    pipeAddrVar[2]=0xC2; 
    pipeAddrVar[1]=0xC2; 
    pipeAddrVar[0]=0xC2; 
    NRF24_write_registerN(svar ,NRF_RE_COM->REG_RX_ADDR_P1, pipeAddrVar, 5);
  
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_ADDR_P2, 0xC3);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_ADDR_P3, 0xC4);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_ADDR_P4, 0xC5);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_ADDR_P5, 0xC6);
    
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P0, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P1, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P2, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P3, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P4, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_RX_PW_P5, 0);
    
    NRF24_ACTIVATE_cmd();
    NRF24_write_register(svar ,NRF_RE_COM->REG_DYNPD, 0);
    NRF24_write_register(svar ,NRF_RE_COM->REG_FEATURE, 0);
    printRadioSettings();
    //Initialise retries 15 and delay 1250 usec
    NRF24_setRetries(15, 15);
    //Initialise PA level to max (0dB)
    NRF24_setPALevel(RF24_PA_0dB);
    //Initialise data rate to 1Mbps
    NRF24_setDataRate(RF24_2MBPS);
    //Initalise CRC length to 16-bit (2 bytes)
    NRF24_setCRCLength(RF24_CRC_16);
    //Set payload size
    NRF24_setPayloadSize(32);
    //Reset status register
    NRF24_resetStatus();
    //Initialise channel to 76
    NRF24_setChannel(76);
    //Flush buffers
    NRF24_flush_tx();
    NRF24_flush_rx();
    
    NRF24_powerDown();
    
}
//13. Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(nrf24l01_config * con,NRF_RE_COM * con)
{
	//Power up and set to RX mode
	NRF24_write_register(con,NRF_RE_COM ->REG_CONFIG, NRF24_read_register(NRF_RE_COM ->REG_CONFIG) | (1UL<<1) |(1UL <<0));
	//Restore pipe 0 address if exists
	if(nrf24l01_config ->pipe0_reading_address)
	NRF24_write_registerN(con,NRF_RE_COM ->REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);
	//Flush buffers
	NRF24_flush_tx();
	NRF24_flush_rx();
	//Set CE HIGH to start listenning
	NRF24_ce(con,1);
	//Wait for 150 uSec for the radio to come on
	NRF24_Delay_MAX();
}
//14. Stop listening (essential before any write operation)
void NRF24_stopListening(nrf24l01_config * con)
{
	NRF24_ce(con,0);
	NRF24_flush_tx();
	NRF24_flush_rx();
}

//15. Write(Transmit data), returns true if successfully sent
bool NRF24_write( nrf24l01_config *con,NRF_RE_COM *com,const void* buf, uint8_t len )
{
	bool retStatus;
	//Start writing
	NRF24_resetStatus();
	NRF24_startWrite(buf,len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    NRF24_read_registerN(NRF_RE_COM->REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = NRF24_get_status();
  }
	//if ack received or max retransmition or timeout come out from while
  while( ! ( status & ( _BV((uint8_t)BIT_TX_DS) | _BV((uint8_t)BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
	
	bool tx_ok, tx_fail;
  NRF24_whatHappened(&tx_ok,&tx_fail, &(nrf24l01_config ->ack_payload_available));
	retStatus = tx_ok;
	if ( nrf24l01_config ->ack_payload_available )
  {
    nrf24l01_config -> ack_payload_length = NRF24_getDynamicPayloadSize();
	}
	//Power down
	NRF24_available();
	NRF24_flush_tx();
	return retStatus;
}
//16. Check for available data to read
bool NRF24_available(void)
{
	return NRF24_availablePipe(NULL);
}

//17. Read received data
uint8_t NRF24_read(NRF_RE_COM *com, void* buf, uint8_t len )
{
	NRF24_read_payload( buf, len );
	uint8_t rxStatus = NRF24_read_register(NRF_RE_COM-> REG_FIFO_STATUS) & _BV(NRF_RE_COM->BIT_RX_EMPTY);
	NRF24_flush_rx();
	NRF24_getDynamicPayloadSize();
	return rxStatus;
}
//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24_openWritingPipe(NRF_RE_COM *com ,uint64_t address)
{
	NRF24_write_registerN(NRF_RE_COM->REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  NRF24_write_registerN(NRF_RE_COM->REG_TX_ADDR, (uint8_t *)(&address), 5);
	
	const uint8_t max_payload_size = 32;
  NRF24_write_register(NRF_RE_COM->REG_RX_PW_P0,MIN(nrf24l01_config->payload_size,max_payload_size));
}
//19. Open reading pipe
void NRF24_openReadingPipe(nrf24l01_config *con,uint8_t number, uint64_t address)
{
	if (number == 0)
    nrf24l01_config->pipe0_reading_address = address;
	
	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			NRF24_write_registerN(NRF_RE_COM->NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			NRF24_write_registerN(NRF_RE_COM->NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		NRF24_write_register(NRF_RE_COM->RF24_RX_PW_PIPE[number],nrf24l01_config->payload_size);
		//Enable pipe
		NRF24_write_register(NRF_RE_COM->REG_EN_RXADDR, NRF24_read_register(NRF_RE_COM->REG_EN_RXADDR) | _BV(number));
	}
	
}
//20 set transmit retries (rf24_Retries_e) and delay
void NRF24_setRetries(nrf24l01_config *con,NRF_RE_COM *com,uint8_t delay, uint8_t count)
{
	NRF24_write_register(con, NRF_RE_COM->REG_SETUP_RETR,(delay&0xf)<<NRF_RE_COM->BIT_ARD | (count&0xf)<<NRF_RE_COM->BIT_ARC);
}

//21. Set RF channel frequency
void NRF24_setChannel(NRF_RE_COM *com,uint8_t channel)
{
	const uint8_t max_channel = 127;
  NRF24_write_register(NRF_RE_COM->REG_RF_CH,MIN(channel,max_channel));
}
//22. Set payload size
void NRF24_setPayloadSize(nrf24l01_config *con,uint8_t size)
{
	const uint8_t max_payload_size = 32;
  nrf24l01_config->payload_size = MIN(size,max_payload_size);
}
//23. Get payload size
uint8_t NRF24_getPayloadSize(nrf24l01_config *con)
{
	return nrf24l01_config->payload_size;
}
//24. Get dynamic payload size, of latest packet received
uint8_t NRF24_getDynamicPayloadSize(NRF_RE_COM *com)
{
	return NRF24_read_register(NRF_RE_COM ->CMD_R_RX_PL_WID);
}

//26. Enable dynamic payloads
void NRF24_enableDynamicPayloads(nrf24l01_config *con,NRF_RE_COM *com)
{
	//Enable dynamic payload through FEATURE register
	NRF24_write_register(NRF_RE_COM->REG_FEATURE,NRF24_read_register(NRF_RE_COM->REG_FEATURE) |  _BV(NRF_RE_COM->BIT_EN_DPL) );
	if(!NRF24_read_register(NRF_RE_COM->REG_FEATURE))
	{
		//NRF24_ACTIVATE_cmd();
		NRF24_write_register(NRF_RE_COM->REG_FEATURE,NRF24_read_register(NRF_RE_COM->REG_FEATURE) |  _BV(NRF_RE_COM->BIT_EN_DPL) );
	}
	//Enable Dynamic payload on all pipes
	NRF24_write_register(NRF_RE_COM->REG_DYNPD,NRF24_read_register(NRF_RE_COM->REG_DYNPD) | _BV(NRF_RE_COM->BIT_DPL_P5) | _BV(NRF_RE_COM->BIT_DPL_P4) | _BV(NRF_RE_COM->BIT_DPL_P3) | _BV(NRF_RE_COM->BIT_DPL_P2) | _BV(NRF_RE_COM->BIT_DPL_P1) | _BV(NRF_RE_COM->BIT_DPL_P0));
  dynamic_payloads_enabled = true;
	
}
void NRF24_disableDynamicPayloads(nrf24l01_config *con,NRF_RE_COM *com)
{
	NRF24_write_register(NRF_RE_COM->REG_FEATURE,NRF24_read_register(NRF_RE_COM->REG_FEATURE) &  ~(_BV(NRF_RE_COM->BIT_EN_DPL)) );
	//Disable for all pipes 
	NRF24_write_register(NRF_RE_COM->REG_DYNPD,0);
	nrf24l01_config->dynamic_payloads_enabled = false;
}

//28. Set Auto Ack for all
void NRF24_setAutoAck(NRF_RE_COM *var ,bool enable)
{
	if ( enable )
    NRF24_write_register(NRF_RE_COM->REG_EN_AA, 0x3F);
  else
    NRF24_write_register(NRF_RE_COM->REG_EN_AA, 0x00);
}
//29. Set Auto Ack for certain pipe
void NRF24_setAutoAckPipe(NRF_RE_COM *com,uint8_t pipe, bool enable )
{
	if ( pipe <= 6 )
  {
    uint8_t en_aa = NRF24_read_register( NRF_RE_COM-> REG_EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    NRF24_write_register( NRF_RE_COM-> REG_EN_AA, en_aa ) ;
  }
}

//30. Set transmit power level
void NRF24_setPALevel(NRF_RE_COM *com,rf24_pa_dbm_e *level )
{
	uint8_t setup = NRF24_read_register(NRF_RE_COM->REG_RF_SETUP) ;
  setup &= ~(_BV(NRF_RE_COM->RF_PWR_LOW) | _BV(NRF_RE_COM->RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == rf24_pa_dbm_e->RF24_PA_0dB)
  {
    setup |= (_BV(NRF_RE_COM->RF_PWR_LOW) | _BV(NRF_RE_COM->RF_PWR_HIGH)) ;
  }
  else if ( level == rf24_pa_dbm_e->RF24_PA_m6dB )
  {
    setup |= _BV(NRF_RE_COM->RF_PWR_HIGH) ;
  }
  else if ( level == rf24_pa_dbm_e->RF24_PA_m12dB )
  {
    setup |= _BV(NRF_RE_COM->RF_PWR_LOW);
  }
  else if ( level == rf24_pa_dbm_e->RF24_PA_m18dB )
  {
    // nothing
  }
  else if ( level == rf24_pa_dbm_e->RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(NRF_RE_COM->RF_PWR_LOW) | _BV(NRF_RE_COM->RF_PWR_HIGH)) ;
  }

  NRF24_write_register( NRF_RE_COM->REG_RF_SETUP, setup ) ;
}
//31. Get transmit power level
rf24_pa_dbm_e NRF24_getPALevel( NRF_RE_COM *com,rf24_pa_dbm_e *level)
{
  rf24_pa_dbm_e result = rf24_pa_dbm_e->RF24_PA_ERROR ;
  uint8_t power = NRF24_read_register(NRF_RE_COM->REG_RF_SETUP) & (_BV(NRF_RE_COM->RF_PWR_LOW) | _BV(NRF_RE_COM->RF_PWR_HIGH));

  // switch uses RAM (evil!)
  if ( power == (_BV(NRF_RE_COM->RF_PWR_LOW) | _BV(NRF_RE_COM->RF_PWR_HIGH)) )
  {
    result = rf24_pa_dbm_e->RF24_PA_0dB ;
  }
  else if ( power == _BV(NRF_RE_COM->RF_PWR_HIGH) )
  {
    result = rf24_pa_dbm_e->RF24_PA_m6dB ;
  }
  else if ( power == _BV(NRF_RE_COM->RF_PWR_LOW) )
  {
    result = rf24_pa_dbm_e->RF24_PA_m12dB ;
  }
  else
  {
    result = rf24_pa_dbm_e->RF24_PA_m18dB ;
  }

  return result ;
}
//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24_setDataRate(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed)
{
	bool result = false;
  uint8_t setup = NRF24_read_register(NRF_RE_COM->REG_RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  nrf24l01_config ->wide_band = false ;
  setup &= ~(_BV(NRF_RE_COM->RF_DR_LOW) | _BV(NRF_RE_COM->RF_DR_HIGH)) ;
  if( speed == rf24_datarate_e ->RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    nrf24l01_config ->wide_band = false ;
    setup |= _BV( NRF_RE_COM->RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == rf24_datarate_e-> RF24_2MBPS )
    {
      nrf24l01_config ->wide_band = true ;
      setup |= _BV(NRF_RE_COM-> RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      nrf24l01_config ->wide_band = false ;
    }
  }
  NRF24_write_register(NRF_RE_COM->REG_RF_SETUP,setup);

  // Verify our result
  if ( NRF24_read_register(NRF_RE_COM->REG_RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    nrf24l01_config ->wide_band = false;
  }

  return result;
}
//33. Get data rate
rf24_datarate_e NRF24_getDataRate(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed )
{
	rf24_datarate_e result ;
  uint8_t dr = NRF24_read_register(NRF_RE_COM ->REG_RF_SETUP) & (_BV(NRF_RE_COM ->RF_DR_LOW) | _BV(NRF_RE_COM ->RF_DR_HIGH));
  
  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(NRF_RE_COM ->RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = rf24_datarate_e->RF24_250KBPS ;
  }
  else if ( dr == _BV(NRF_RE_COM ->RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = rf24_datarate_e->RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = rf24_datarate_e->RF24_1MBPS ;
  }
  return result ;
}
//34. Set crc length (disable, 8-bits or 16-bits)
void NRF24_setCRCLength(nrf24l01_config *con,NRF_RE_COM *com,rf24_datarate_e *speed ,rf24_crclength_e *length)
{
	uint8_t config = NRF24_read_register(NRF_RE_COM ->REG_CONFIG) & ~( _BV(NRF_RE_COM ->BIT_CRCO) | _BV(NRF_RE_COM ->BIT_EN_CRC)) ;
  
  // switch uses RAM
  if ( length == rf24_crclength_e->RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == rf24_crclength_e->RF24_CRC_8 )
  {
    config |= _BV(NRF_RE_COM ->BIT_EN_CRC);
  }
  else
  {
    config |= _BV(NRF_RE_COM ->BIT_EN_CRC);
    config |= _BV( NRF_RE_COM ->BIT_CRCO );
  }
  NRF24_write_register( NRF_RE_COM->REG_CONFIG, config );
}
//35. Get CRC length
rf24_crclength_e NRF24_getCRCLength(nrf24l01_config *con,NRF_RE_COM *com,rf24_crclength_e *length)
{
	rf24_crclength_e result = f24_crclength_e->RF24_CRC_DISABLED;
  uint8_t config = NRF24_read_register(NRF_RE_COM ->REG_CONFIG) & ( _BV(NRF_RE_COM ->BIT_CRCO) | _BV(NRF_RE_COM ->BIT_EN_CRC)) ;

  if ( config & _BV(NRF_RE_COM ->BIT_EN_CRC ) )
  {
    if ( config & _BV(NRF_RE_COM ->BIT_CRCO) )
      result = rf24_crclength_e-> RF24_CRC_16;
    else
      result = rf24_crclength_e-> RF24_CRC_8;
  }

  return result;
}
//36. Disable CRC
void NRF24_disableCRC(NRF_RE_COM *com)
{
	uint8_t disable = NRF24_read_register(NRF_RE_COM ->REG_CONFIG) & ~_BV(NRF_RE_COM ->BIT_EN_CRC) ;
  NRF24_write_register( NRF_RE_COM->REG_CONFIG, disable ) ;
}
//37. power up
void NRF24_powerUp(NRF_RE_COM *com)
{
	NRF24_write_register(NRF_RE_COM->REG_CONFIG,NRF24_read_register(NRF_RE_COM ->REG_CONFIG) | _BV(NRF_RE_COM ->BIT_PWR_UP));
}
//38. power down
void NRF24_powerDown(NRF_RE_COM *com)
{
	NRF24_write_register(NRF_RE_COM->REG_CONFIG,NRF24_read_register(NRF_RE_COM->REG_CONFIG) & ~_BV(NRF_RE_COM->BIT_PWR_UP));
}
//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24_availablePipe(NRF_RE_COM *com,uint8_t* pipe_num)
{
	uint8_t status = NRF24_get_status();

  bool result = ( status & _BV(NRF_RE_COM->BIT_RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> NRF_RE_COM->BIT_RX_P_NO ) & 0x7;

    // Clear the status bit
    NRF24_write_register(NRF_RE_COM->REG_STATUS,_BV(NRF_RE_COM->BIT_RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(NRF_RE_COM->BIT_TX_DS) )
    {
      NRF24_write_register(NRF_RE_COM->REG_STATUS,_BV(NRF_RE_COM->BIT_TX_DS));
    }
  }
  return result;
}

//40. Start write (for IRQ mode)
void NRF24_startWrite(nrf24l01_config *con, NRF_RE_COM *com,const void* buf, uint8_t len )
{
	// Transmitter power-up
  NRF24_write_register(con,NRF_RE_COM->REG_CONFIG, ( NRF24_read_register(NRF_RE_COM->REG_CONFIG) | _BV((uint8_t)NRF_RE_COM->BIT_PWR_UP) ) & ~_BV((uint8_t)NRF_RE_COM->BIT_PRIM_RX) );
  NRF24_Delay_MAX();

  // Send the payload
  NRF24_write_payload( con,buf, len );

  // Enable Tx for 15usec
  NRF24_ce(con,1);
  NRF24_Delay_MAX();
  NRF24_ce(con,0);
}
//41. Write acknowledge payload
void NRF24_writeAckPayload(NRF_RE_COM *com,uint8_t pipe, const void* buf, uint8_t len)
{
	const uint8_t* current = (uint8_t *)buf;
	const uint8_t max_payload_size = 32;
  uint8_t data_len = MIN(len,max_payload_size);
	
  NRF24_csn(0);
	NRF24_write_registerN(NRF_RE_COM->CMD_W_ACK_PAYLOAD | ( pipe & 0x7 ) , current, data_len);
  NRF24_csn(1);
}
//42. Check if an Ack payload is available
bool NRF24_isAckPayloadAvailable(nrf24l01_config *con)
{
	bool result = nrf24l01_config->ack_payload_available;
  nrf24l01_config->ack_payload_available = false;
  return result;
}
//43. Check interrupt flags
void NRF24_whatHappened(NRF_RE_COM *com,bool *tx_ok,bool *tx_fail,bool *rx_ready)
{
	uint8_t status = NRF24_get_status();
	*tx_ok = 0;
	
	NRF24_write_register(NRF_RE_COM->REG_STATUS,_BV(NRF_RE_COM->BIT_RX_DR) | _BV(NRF_RE_COM->BIT_TX_DS) | _BV(NRF_RE_COM->BIT_MAX_RT) );
  // Report to the user what happened
  *tx_ok = status & _BV(NRF_RE_COM->BIT_TX_DS);
  *tx_fail = status & _BV(NRF_RE_COM->BIT_MAX_RT);
  *rx_ready = status & _BV(NRF_RE_COM->BIT_RX_DR);
}
//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24_testCarrier(NRF_RE_COM *com)
{
	return NRF24_read_register(NRF_RE_COM->REG_CD) & 1;
}
//45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24_testRPD(nrf24l01_config *con,NRF_RE_COM *com)
{
	return NRF24_read_register(con,NRF_RE_COM->REG_RPD) & 1;
}

//46. Reset Status
void NRF24_resetStatus(nrf24l01_config *con,NRF_RE_COM *com)
{
	NRF24_write_register(con,NRF_RE_COM->REG_STATUS,_BV((uint8_t)NRF_RE_COM->BIT_RX_DR) | _BV((uint8_t)NRF_RE_COM->BIT_TX_DS) | _BV((uint8_t)NRF_RE_COM->BIT_MAX_RT) );
}

//47. ACTIVATE cmd
//void NRF24_ACTIVATE_cmd(nrf24l01_config *con,NRF_RE_COM *com)
//{
//	uint8_t cmdRxBuf[2];
//	//Read data from Rx payload buffer
//	NRF24_csn(con,0);
//	cmdRxBuf[0] = NRF_RE_COM->CMD_ACTIVATE;
//	cmdRxBuf[1] = 0x73;
//	HAL_SPI_Transmit(&(nrf24l01_config->nrf24_hspi), cmdRxBuf, 2, 100);
//	NRF24_csn(con, 1);
//}

//48. Get AckPayload Size
uint8_t NRF24_GetAckPayloadSize(nrf24l01_config *con)
{
	return nrf24l01_config ->ack_payload_length;
}
//1. Print radio settings
void printRadioSettings(nrf24l01_config *con,NRF_RE_COM *com)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//a) Get CRC settings - Config Register
	reg8Val = NRF24_read_register(NRF_RE_COM->REG_CONFIG);
	if(reg8Val & (1 << 3))
	{
		if(reg8Val & (1 << 2)) sprintf(uartTxBuf, "CRC:\r\n	Enabled, 2 Bytes \r\n");
		else sprintf(uartTxBuf, "CRC:\r\n	Enabled, 1 Byte \r\n");	
	}
	else
	{
		sprintf(uartTxBuf, "CRC:\r\n		Disabled \r\n");
	}
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//b) AutoAck on pipes
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_EN_AA);
	sprintf(uartTxBuf, "ENAA:\r\n	P0:	%d\r\n	P1:	%d\r\n	P2:	%d\r\nP3:	%d\r\n	P4:	%d\r\n	P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//c) Enabled Rx addresses
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_EN_RXADDR);
	sprintf(uartTxBuf, "EN_RXADDR:\r\n	P0:	%d\r\n	P1:	%d\r\n	P2:	%d\r\n	P3:	%d\r\n	P4:	%d\r\n	P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//d) Address width
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_SETUP_AW)&(NRF_RE_COM ->REG_SETUP_AW);
	reg8Val +=2;
	sprintf(uartTxBuf, "SETUP_AW:\r\n %d bytes \r\n", reg8Val);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//e) RF channel
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RF_CH);
	sprintf(uartTxBuf, "RF_CH:\r\n	%d CH \r\n", reg8Val&0x7F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//f) Data rate & RF_PWR
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RF_SETUP);
	if(reg8Val & (1 << 3)) sprintf(uartTxBuf, "Data Rate:\r\n 2Mbps \r\n");
	else sprintf(uartTxBuf, "Data Rate:\r\n	1Mbps \r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	reg8Val &= (3 << 1);
	reg8Val = (reg8Val>>1);
	if(reg8Val == 0) sprintf(uartTxBuf, "RF_PWR:\r\n -18dB \r\n");
	else if(reg8Val == 1) sprintf(uartTxBuf, "RF_PWR:\r\n -12dB \r\n");
	else if(reg8Val == 2) sprintf(uartTxBuf, "RF_PWR:\r\n -6dB \r\n");
	else if(reg8Val == 3) sprintf(uartTxBuf, "RF_PWR:\r\n 0dB \r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	//g) RX pipes addresses
	uint8_t pipeAddrs[6];
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P0, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe0 Addrs:\r\n	%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P1, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe1 Addrs:\r\n	%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&( ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P2, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe2 Addrs:\r\n	xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P3, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe3 Adnrf24l01_configdrs:\r\n	xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P4, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe4 Addrs:\r\n	xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P5, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe5 Addrs:\r\n	xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	NRF24_read_registerN(NRF_RE_COM->REG_RX_ADDR_P6, pipeAddrs, 5);
	sprintf(uartTxBuf, "TX Addrs:\r\n	%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	//h) RX PW (Payload Width 0 - 32)
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P0);
	sprintf(uartTxBuf, "RX_PW_P0:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P1);
	sprintf(uartTxBuf, "RX_PW_P1:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P2);
	sprintf(uartTxBuf, "RX_PW_P2:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P3);
	sprintf(uartTxBuf, "RX_PW_P3:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P4);
	sprintf(uartTxBuf, "RX_PW_P4:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM ->REG_RX_PW_P5);
	sprintf(uartTxBuf, "RX_PW_P5:\r\n	%d bytes \r\n", reg8Val&0x3F);
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}

//2. Print Status 
void printStatusReg(NRF_RE_COM *val)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM->REG_STATUS);
	sprintf(uartTxBuf, "STATUS reg:\r\n	RX_DR:%d\r\n TX_DS:%d\r\n	MAX_RT:%d\r\n RX_P_NO:%d\r\n	TX_FULL:%d\r\n",
	_BOOL(reg8Val&(1<<6)), _BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(3<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}
//3. Print Config 
void printConfigReg(NRF_RE_COM *con)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM->REG_CONFIG);
	sprintf(uartTxBuf, "CONFIG reg:\r\n	PWR_UP:	%d\r\n	PRIM_RX:%d\r\n",_BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	_BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}

//5. FIFO Status
void printFIFOstatus(NRF_RE_COM *con)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	reg8Val = NRF24_read_register(NRF_RE_COM -> REG_FIFO_STATUS);
	sprintf(uartTxBuf, "FIFO Status reg:\r\n TX_FULL:%d\r\n	TX_EMPTY:%d\r\n	RX_FULL:%d\r\n	RX_EMPTY:%d\r\n",
	_BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&(nrf24l01_config ->nrf24_huart), (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
	
}
