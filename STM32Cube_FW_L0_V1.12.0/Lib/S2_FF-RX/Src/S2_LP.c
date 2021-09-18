#include "s2_lp.h"
#include "stm32l0xx_hal.h"
#include "stdio.h"

//* Read single or multiple registers.
//* cRegAddress: base register's address to be read
//* cNbBytes: number of registers and bytes to be read
//* pcBuffer: pointer to the buffer of registers' values read

HAL_StatusTypeDef S2LPSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t *pcBuffer)
{
  uint8_t tx_buff[255]={READ_HEADER,cRegAddress};
  uint8_t rx_buff[2];
	SPI_ENTER_CRITICAL();
  
	SdkEvalSPICSLow();

	HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, 1000);
	HAL_SPI_TransmitReceive(&hspi1, tx_buff, pcBuffer, cNbBytes, 1000);

  SdkEvalSPICSHigh();
  SPI_EXIT_CRITICAL();

  //((uint8_t*)&status)[1]=rx_buff[0];
  //((uint8_t*)&status)[0]=rx_buff[1];
			
	//printf("S2LP_ReadRegisters() reg=%x, val=%x\n", cRegAddress, *pcBuffer);
  return HAL_OK;
}

/**
* @brief  Write single or multiple registers.
* @param  cRegAddress: base register's address to be write
* @param  cNbBytes: number of registers and bytes to be write
* @param  pcBuffer: pointer to the buffer of values have to be written into registers
*/
HAL_StatusTypeDef S2LPSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[2]={WRITE_HEADER,cRegAddress};
  uint8_t rx_buff[255];
  
  SPI_ENTER_CRITICAL();
  
  /* Puts the SPI chip select low to start the transaction */
  SdkEvalSPICSLow();
  
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&hspi1, pcBuffer, &rx_buff[2], cNbBytes, 1000);
  
  /* Puts the SPI chip select high to end the transaction */
  SdkEvalSPICSHigh();
  
  SPI_EXIT_CRITICAL();
  
  //((uint8_t*)&status)[1]=rx_buff[0];
  //((uint8_t*)&status)[0]=rx_buff[1];
  	//printf("S2LP_WriteRegisters() reg=%x, val=%x\n", cRegAddress, *pcBuffer);
    return HAL_OK;
  
}

/**
* @brief  Read data from RX FIFO.
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval Device status
*/
HAL_StatusTypeDef SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[130]={READ_HEADER,LINEAR_FIFO_ADDRESS};
  uint8_t rx_buff[2];
  //StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  SdkEvalSPICSLow();
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, pcBuffer, cNbBytes, 1000);
  SdkEvalSPICSHigh();
  SPI_EXIT_CRITICAL();
  
  //((uint8_t*)&status)[1]=rx_buff[0];
  //((uint8_t*)&status)[0]=rx_buff[1];
  uint8_t j;
	printf("Bytes: ");
	for(j=0;j<cNbBytes;j++){
		printf("%x ",pcBuffer[j]);
	}
	printf("\n");
     return HAL_OK;
  //return status;
}

/**
* @brief  Write data into TX FIFO.
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval Device status
*/
HAL_StatusTypeDef SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t tx_buff[2]={WRITE_HEADER,LINEAR_FIFO_ADDRESS};
  uint8_t rx_buff[130];
  //StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  SdkEvalSPICSLow();
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, 1000);
  HAL_SPI_TransmitReceive(&hspi1, pcBuffer, &rx_buff[2], cNbBytes, 1000);
  SdkEvalSPICSHigh();
  SPI_EXIT_CRITICAL();
  
  //((uint8_t*)&status)[1]=rx_buff[0];
  //((uint8_t*)&status)[0]=rx_buff[1];
   return HAL_OK;
  //return status;
}

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval Device status
*/
HAL_StatusTypeDef SdkEvalSpiCommandStrobes(uint8_t cCommandCode)
{
  uint8_t tx_buff[2]={COMMAND_HEADER,cCommandCode};
  uint8_t rx_buff[2];
  //StatusBytes status;
  
  SPI_ENTER_CRITICAL();
  SdkEvalSPICSLow();
  HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, 1000);
  SdkEvalSPICSHigh();
  SPI_EXIT_CRITICAL();
  //printf("Command code: %x\n",cCommandCode);
  //((uint8_t*)&status)[1]=rx_buff[0];
  //((uint8_t*)&status)[0]=rx_buff[1];
   return HAL_OK;
  //return status;
}


/* This is the function that initializes the S2-LP with the configuration 
that the user has exported using the GUI */
HAL_StatusTypeDef SpiritBaseConfiguration(void)
{
  uint8_t tmp[8];

  tmp[0] = 0xA3; /* reg. GPIO3_CONF (0x03) */
  S2LPSpiWriteRegisters(0x03, 1, tmp);
  tmp[0] = 0x62; /* reg. SYNT3 (0x05) */
  tmp[1] = 0x2B; /* reg. SYNT2 (0x06) */
  tmp[2] = 0x85; /* reg. SYNT1 (0x07) */
  tmp[3] = 0x18; /* reg. SYNT0 (0x08) */
  tmp[4] = 0x2F; /* reg. IF_OFFSET_ANA (0x09) */
  tmp[5] = 0xC2; /* reg. IF_OFFSET_DIG (0x0A) */
  S2LPSpiWriteRegisters(0x05, 6, tmp);
  tmp[0] = 0x92; /* reg. MOD4 (0x0E) */
  tmp[1] = 0xA7; /* reg. MOD3 (0x0F) */
  tmp[2] = 0x27; /* reg. MOD2 (0x10) */
  S2LPSpiWriteRegisters(0x0E, 3, tmp);
  tmp[0] = 0xA3; /* reg. MOD0 (0x12) */
  tmp[1] = 0x13; /* reg. CHFLT (0x13) */
  S2LPSpiWriteRegisters(0x12, 2, tmp);
  tmp[0] = 0x10; /* reg. RSSI_TH (0x18) */
  S2LPSpiWriteRegisters(0x18, 1, tmp);
  tmp[0] = 0x55; /* reg. ANT_SELECT_CONF (0x1F) */
  S2LPSpiWriteRegisters(0x1F, 1, tmp);
  tmp[0] = 0x08; /* reg. PCKTCTRL4 (0x2D) */
	tmp[1] = 0xC0; /* reg. PCKTCTRL3 (0x2E) -> ABILITO STPACKET CON 3 TENTATIVI DI TRAMISSIONE*/
  tmp[2] = 0x01; /* reg. PCKTCTRL2 (0x2F) */
  tmp[3] = 0x30; /* reg. PCKTCTRL1 (0x30) */
  S2LPSpiWriteRegisters(0x2D, 4, tmp);
  tmp[0] = 0x78; /* reg. SYNC3 (0x33) */
  tmp[1] = 0x56; /* reg. SYNC2 (0x34) */
  tmp[2] = 0x34; /* reg. SYNC1 (0x35) */
  tmp[3] = 0x12; /* reg. SYNC0 (0x36) */
  S2LPSpiWriteRegisters(0x33, 4, tmp);
  tmp[0] = 0x44; /* reg. PROTOCOL2 (0x39) */
  tmp[1] = 0x01; /* reg. PROTOCOL1 (0x3A) */
  
	tmp[2] = 0x0C; /* reg. PROTOCOL0 (0x3B) -> NELL RX METTO AUTOACK */
	
  tmp[3] = 0x40; /* reg. FIFO_CONFIG3 (0x3C) */
  tmp[4] = 0x40; /* reg. FIFO_CONFIG2 (0x3D) */
  tmp[5] = 0x40; /* reg. FIFO_CONFIG1 (0x3E) */
  tmp[6] = 0x40; /* reg. FIFO_CONFIG0 (0x3F) */
	
  tmp[7] = 0x43; /* reg. PCKT_FLT_OPTIONS (0x40)  Filtro sugli indirizzi sotto*/
  S2LPSpiWriteRegisters(0x39, 8, tmp);
  
	tmp[0] = 0x00; /* reg. TIMERS5 (0x46) */
  tmp[1] = 0x09; /* reg. TIMERS4 (0x47) */
  S2LPSpiWriteRegisters(0x46, 2, tmp);
  tmp[0] = 0x02; /* reg. IRQ_MASK1 (0x52) */
  S2LPSpiWriteRegisters(0x52, 1, tmp);
	
	// INDIRIZZI RX/TX
	tmp[0] = 0xA5; /* reg. PCKT_FLT_GOALS3 (0x42) */
  S2LPSpiWriteRegisters(0x42, 1, tmp);
  tmp[0] = 0xA4; /* reg. PCKT_FLT_GOALS0 (0x45) */
  S2LPSpiWriteRegisters(0x45, 1, tmp);					
	
  tmp[0] = 0x1D; /* reg. PA_POWER8 (0x5A) */
  S2LPSpiWriteRegisters(0x5A, 1, tmp);
  tmp[0] = 0x07; /* reg. PA_POWER0 (0x62) */
  tmp[1] = 0x01; /* reg. PA_CONFIG1 (0x63) */
  S2LPSpiWriteRegisters(0x62, 2, tmp);
  tmp[0] = 0x87; /* reg. PM_CONF3 (0x76) */
  tmp[1] = 0xFC; /* reg. PM_CONF2 (0x77) */
  S2LPSpiWriteRegisters(0x76, 2, tmp);
	
	tmp[0] = 0x02; /* reg. GPIO1_CONF (0x01) */
	S2LPSpiWriteRegisters(0x01,1,tmp);
	return HAL_OK;		
}

/**
 * @brief  Set the payload length for S2LP Basic packets. Since the packet length
 *         depends from the address and the control field size, this
 *         function reads the correspondent registers in order to determine
 *         the correct packet length to be written.
 * @param  nPayloadLength payload length in bytes.
 *         This parameter is an uint16_t.
 * @retval None.
 */
/*SE SETTI IL PACCHETTO 2 SOMMA 2..DA VERIFICARE*/
HAL_StatusTypeDef S2LPPktBasicSetPayloadLength(uint16_t nPayloadLength)
{
  uint8_t tmpBuffer[2];
  
  tmpBuffer[0] = (uint8_t)(nPayloadLength>>8);
  tmpBuffer[1] = (uint8_t)nPayloadLength;
  S2LPSpiWriteRegisters(PCKTLEN1_ADDR, 2, tmpBuffer);
	return HAL_OK;
}




/**
 * @brief  Set the RX timeout timer counter and prescaler from the desired value in ms. it is possible to fix the RX_Timeout to
 *         a minimum value of 50.417us to a maximum value of about 3.28 s.
 * @param  lDesiredUsec desired timer value.
 *         This parameter must be a uint32_t.
 * @retval None
 */
void S2LPTimerSetRxTimerUs(uint32_t lDesiredUsec)
{
  uint8_t tmpBuffer[2];
  S2LPTimerComputeRxTimerRegValues(lDesiredUsec , &tmpBuffer[0] , &tmpBuffer[1]);
  S2LPSpiWriteRegisters(TIMERS5_ADDR, 2, tmpBuffer);
}


/**
 * @brief  Computes the values of the rx_timeout timer counter and prescaler from the user time expressed in millisecond.
 *         The prescaler and the counter values are computed maintaining the prescaler value as
 *         small as possible in order to obtain the best resolution, and in the meantime minimizing the error.
 * @param  lDesiredUsec desired rx_timeout in microsecs.
 *         This parameter must be a float. Since the counter and prescaler are 8 bit registers the maximum
 *         reachable value is maxTime = fTclk x 255 x 255.
 * @param  pcCounter pointer to the variable in which the value for the rx_timeout counter has to be stored.
 *         This parameter must be a uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the value for the rx_timeout prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None
 */
void S2LPTimerComputeRxTimerRegValues(uint32_t lDesiredUsec , uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint32_t f_dig = 50000000;
  uint32_t n;
  uint64_t tgt,tgt1,tgt2;
  
  /* if xtal is doubled divide it by 2 */
  if(f_dig>30000000) {
    f_dig >>= 1;
  }
  
  /* N cycles in the time base of the timer: 
     - clock of the timer is f_dig/1210
     - divide times 1000000 more because we have an input in us
  */
  tgt=(uint64_t)lDesiredUsec*f_dig;
  n=(uint32_t)(tgt/1210000000);
  tgt1=(uint64_t)1210000000*n;
  tgt2=(uint64_t)1210000000*(n+1);
  
  n=((tgt2-tgt)<(tgt-tgt1))?(n+1):(n);
  
  /* check if it is possible to reach that target with prescaler and counter of S2LP */
  if(n/0xFF>0xFD) {
    /* if not return the maximum possible value */
    (*pcCounter) = 0xFF;
    (*pcPrescaler) = 0xFF;
    return;
  }
  
  /* prescaler is really 2 as min value */
  (*pcPrescaler)=(n/0xFF)+2;
  (*pcCounter) = n / (*pcPrescaler);
  
    
  /* decrement prescaler and counter according to the logic of this timer in S2LP */ 
  (*pcPrescaler)--;

  if((*pcCounter)==0)
    (*pcCounter)=1;
}

