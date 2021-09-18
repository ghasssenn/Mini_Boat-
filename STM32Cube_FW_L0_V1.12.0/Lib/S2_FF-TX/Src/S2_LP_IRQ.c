#include "s2_lp_irq.h"


/**HAL_StatusTypeDef **
 * @brief  Enable or disables a specific IRQ.
 * @param  xIrq IRQ to enable or disable.
 *         This parameter can be any value of @ref IrqList.
 * @param  xNewState new state for the IRQ.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
HAL_StatusTypeDef S2LPGpioIrqConfig(IrqList xIrq, SFunctionalState xNewState)
{
  uint8_t tmpBuffer[4];
  uint32_t tempValue = 0;

  S2LPSpiReadRegisters(IRQ_MASK3_ADDR, 4, tmpBuffer);

  /* Build the IRQ mask word */
  for(char i=0; i<4; i++) {
    tempValue += ((uint32_t)tmpBuffer[i])<<(8*(3-i));
  }
  
  /* Rebuild the new mask according to user request */
  if(xNewState == S_DISABLE) {
    tempValue &= (~xIrq);
  }
  else {
    tempValue |= (xIrq);
  }

  /* Build the array of bytes to write in the IRQ_MASK registers */
  for(char j=0; j<4; j++) {
    tmpBuffer[j] = (uint8_t)(tempValue>>(8*(3-j)));
  }
  
  S2LPSpiWriteRegisters(IRQ_MASK3_ADDR, 4, tmpBuffer);
	return HAL_OK;

}

/**
 * @brief  Deinit the S2LPIrqs structure setting all the bitfield to 0.
 *         Moreover, it sets the IRQ mask registers to 0x00000000, disabling all IRQs.

 */
HAL_StatusTypeDef S2LPGpioIrqDeInit()
{
  uint8_t tmp[4] = {0x00,0x00,0x00,0x00};
  S2LPSpiWriteRegisters(IRQ_MASK3_ADDR, 4, tmp);
		return HAL_OK;	
}

/**
 * @brief  Fill a pointer to a structure of S2LPIrqs type reading the IRQ_STATUS registers.
 * @param  pxIrqStatus pointer to a variable of type @ref S2LPIrqs, through which the
 *         user can read the status of all the IRQs. All the bitfields equals to one correspond
 *         to the raised interrupts. This parameter is a pointer to a S2LPIrqs.
 *         For example suppose that the XO settling timeout is raised as well as the Sync word
 *         detection.
 * @code
 * S2LPIrqs myIrqStatus;
 * S2LPGpioIrqGetStatus(&myIrqStatus);
 * @endcode
 * Then
 * myIrqStatus.IRQ_XO_COUNT_EXPIRED and myIrqStatus.IRQ_VALID_SYNC are equals to 1
 * while all the other bitfields are equals to zero.
 * @retval None.
 */
HAL_StatusTypeDef S2LPGpioIrqGetStatus(S2LPIrqs* pxIrqStatus)
{
  uint8_t tmp[4];
  uint8_t* pIrqPointer = (uint8_t*)pxIrqStatus;
  
  S2LPSpiReadRegisters(IRQ_STATUS3_ADDR, 4, tmp);

  /* Build the IRQ Status word */
  for(uint8_t i=0; i<4; i++) {
    *pIrqPointer = tmp[3-i];
    pIrqPointer++;
  }
	return HAL_OK;
}

/**
 * @brief  Clear the IRQ status registers.
 * @param  None.
 * @retval None.
 */
HAL_StatusTypeDef  S2LPGpioIrqClearStatus(void)
{
  uint8_t tmp[4];
  S2LPSpiReadRegisters(IRQ_STATUS3_ADDR, 4, tmp);
		return HAL_OK;	
}

