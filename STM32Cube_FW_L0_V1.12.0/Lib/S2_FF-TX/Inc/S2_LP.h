#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l0xx_hal.h"
extern SPI_HandleTypeDef hspi1;
		
#define SPI_ENTER_CRITICAL()           //__disable_irq()
#define SPI_EXIT_CRITICAL()            //__enable_irq()
		
#define SdkEvalSPICSLow()        HAL_GPIO_WritePin(S2_CSn_GPIO_Port, S2_CSn_Pin, GPIO_PIN_RESET);
#define SdkEvalSPICSHigh()  		 HAL_GPIO_WritePin(S2_CSn_GPIO_Port, S2_CSn_Pin, GPIO_PIN_SET);

#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/
#define LINEAR_FIFO_ADDRESS	 0xFF  /*!< Linear FIFO address*/	
		
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/		
		

/* list of the command codes of S2-LP */
#define	COMMAND_TX                                          ((uint8_t)(0x60)) /*!< Start to transmit; valid only from READY */
#define	COMMAND_RX                                          ((uint8_t)(0x61)) /*!< Start to receive; valid only from READY */
#define	COMMAND_READY                                       ((uint8_t)(0x62)) /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
#define	COMMAND_STANDBY                                     ((uint8_t)(0x63)) /*!< Go to STANDBY; valid only from READY */
#define	COMMAND_SLEEP                                       ((uint8_t)(0x64)) /*!< Go to SLEEP; valid only from READY */
#define	COMMAND_LOCKRX                                      ((uint8_t)(0x65)) /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
#define	COMMAND_LOCKTX                                      ((uint8_t)(0x66)) /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
#define	COMMAND_SABORT                                      ((uint8_t)(0x67)) /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
#define	COMMAND_SRES                                        ((uint8_t)(0x70)) /*!< Reset of all digital part, except SPI registers */
#define	COMMAND_FLUSHRXFIFO                                 ((uint8_t)(0x71)) /*!< Clean the RX FIFO; valid from all states */
#define	COMMAND_FLUSHTXFIFO                                 ((uint8_t)(0x72)) /*!< Clean the TX FIFO; valid from all states */

#define PCKTLEN1_ADDR			((uint8_t)0x31)
#define TIMERS5_ADDR			((uint8_t)0x46)

void S2LPTimerComputeRxTimerRegValues(uint32_t lDesiredUsec , uint8_t* pcCounter , uint8_t* pcPrescaler);
extern void S2LPTimerSetRxTimerUs(uint32_t lDesiredUsec);

extern HAL_StatusTypeDef S2LPSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
extern HAL_StatusTypeDef S2LPSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
extern HAL_StatusTypeDef SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
extern HAL_StatusTypeDef SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
extern HAL_StatusTypeDef SpiritBaseConfiguration(void);
extern HAL_StatusTypeDef SdkEvalSpiCommandStrobes(uint8_t cCommandCode);
extern HAL_StatusTypeDef S2LPPktBasicSetPayloadLength(uint16_t nPayloadLength);





