#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l0xx_hal.h"
#include "s2_lp.h"		
		
extern SPI_HandleTypeDef hspi1;

#define IRQ_MASK3_ADDR			((uint8_t)0x50)
#define IRQ_STATUS3_ADDR			((uint8_t)0xFA)

		
	
/**
 * @brief  S2LP Functional state. Used to enable or disable a specific option.
 */
typedef enum {
  S_DISABLE = 0,
  S_ENABLE = !S_DISABLE
} SFunctionalState;		

/**
 * @brief  S2LP Flag status. Used to control the state of a flag.
 */
typedef enum {
  S_RESET = 0,
  S_SET = !S_RESET
} SFlagStatus;
		
/**
 * @brief  IRQ list enumeration for S2LP. This enumeration type can be used to address a
 *         specific IRQ.
 */
typedef enum {
  RX_DATA_READY = 0x00000001,           /*!< IRQ: RX data ready */
  RX_DATA_DISC = 0x00000002,            /*!< IRQ: RX data discarded (upon filtering) */
  TX_DATA_SENT = 0x00000004,            /*!< IRQ: TX data sent */
  MAX_RE_TX_REACH = 0x00000008,         /*!< IRQ: Max re-TX reached */
  CRC_ERROR = 0x00000010,               /*!< IRQ: CRC error */
  TX_FIFO_ERROR = 0x00000020,           /*!< IRQ: TX FIFO underflow/overflow error */
  RX_FIFO_ERROR = 0x00000040,           /*!< IRQ: RX FIFO underflow/overflow error */
  TX_FIFO_ALMOST_FULL = 0x00000080,     /*!< IRQ: TX FIFO almost full */
  TX_FIFO_ALMOST_EMPTY = 0x00000100,    /*!< IRQ: TX FIFO almost empty */
  RX_FIFO_ALMOST_FULL = 0x00000200,     /*!< IRQ: RX FIFO almost full */
  RX_FIFO_ALMOST_EMPTY = 0x00000400,    /*!< IRQ: RX FIFO almost empty  */
  MAX_BO_CCA_REACH = 0x00000800,        /*!< IRQ: Max number of back-off during CCA */
  VALID_PREAMBLE = 0x00001000,          /*!< IRQ: Valid preamble detected */
  VALID_SYNC = 0x00002000,              /*!< IRQ: Sync word detected */
  RSSI_ABOVE_TH = 0x00004000,           /*!< IRQ: RSSI above threshold */
  WKUP_TOUT_LDC = 0x00008000,           /*!< IRQ: Wake-up timeout in LDC mode */
  READY = 0x00010000,                   /*!< IRQ: READY state */
  STANDBY_DELAYED = 0x00020000,         /*!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
  LOW_BATT_LVL = 0x00040000,            /*!< IRQ: Battery level below threshold*/
  POR = 0x00080000,                     /*!< IRQ: Power On Reset */
  BOR = 0x00100000,                     /*!< IRQ: Brown out event (both accurate and inaccurate)*/
  LOCK = 0x00200000,                    /*!< IRQ: LOCK state */
  VCO_CALIBRATION_END = 0x00400000,        /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
  PA_CALIBRATION_END = 0x00800000,        /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
  PM_COUNT_EXPIRED = 0x01000000,        /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
  XO_COUNT_EXPIRED = 0x02000000,        /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
  TX_START_TIME = 0x04000000,	        /*!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER */
  RX_START_TIME = 0x08000000,	        /*!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER */
  RX_TIMEOUT = 0x10000000,	        /*!< IRQ: RX operation timeout */
  RX_SNIFF_TIMEOUT = 0x20000000,                 /*!< IRQ: RX sniff operation timeout */
  ALL_IRQ = 0x7FFFFFFF			/*!< All the above mentioned IRQs */
} IrqList;		

/**
 * @brief IRQ bitfield structure for S2LP. This structure is used to read or write the single IRQ bit.
 *        During the initialization the user has to fill this structure setting to one the single field related
 *        to the IRQ he wants to enable, and to zero the single field related to all the IRQs he wants to disable.
 *        The same structure can be used to retrieve all the IRQ events from the IRQ registers IRQ_STATUS[3:0],
 *        and read if one or more specific IRQ raised.
 * @note  The fields order in the structure depends on used endianness (little or big
 *        endian). The actual definition is valid ONLY for LITTLE ENDIAN mode. Be sure to
 *        change opportunely the fields order when use a different endianness.
 */
typedef struct {
  SFlagStatus  IRQ_RX_DATA_READY:1;            /*!< IRQ: RX data ready */
  SFlagStatus  IRQ_RX_DATA_DISC:1;             /*!< IRQ: RX data discarded (upon filtering) */
  SFlagStatus  IRQ_TX_DATA_SENT:1;             /*!< IRQ: TX data sent */
  SFlagStatus  IRQ_MAX_RE_TX_REACH:1;          /*!< IRQ: Max re-TX reached */
  SFlagStatus  IRQ_CRC_ERROR:1;                /*!< IRQ: CRC error */
  SFlagStatus  IRQ_TX_FIFO_ERROR:1;            /*!< IRQ: TX FIFO underflow/overflow error */
  SFlagStatus  IRQ_RX_FIFO_ERROR:1;            /*!< IRQ: RX FIFO underflow/overflow error */
  SFlagStatus  IRQ_TX_FIFO_ALMOST_FULL:1;      /*!< IRQ: TX FIFO almost full */

  SFlagStatus  IRQ_TX_FIFO_ALMOST_EMPTY:1;     /*!< IRQ: TX FIFO almost empty */
  SFlagStatus  IRQ_RX_FIFO_ALMOST_FULL:1;      /*!< IRQ: RX FIFO almost full */
  SFlagStatus  IRQ_RX_FIFO_ALMOST_EMPTY:1;     /*!< IRQ: RX FIFO almost empty  */
  SFlagStatus  IRQ_MAX_BO_CCA_REACH:1;         /*!< IRQ: Max number of back-off during CCA */
  SFlagStatus  IRQ_VALID_PREAMBLE:1;           /*!< IRQ: Valid preamble detected */
  SFlagStatus  IRQ_VALID_SYNC:1;               /*!< IRQ: Sync word detected */
  SFlagStatus  IRQ_RSSI_ABOVE_TH:1;            /*!< IRQ: RSSI above threshold */
  SFlagStatus  IRQ_WKUP_TOUT_LDC:1;            /*!< IRQ: Wake-up timeout in LDC mode */

  SFlagStatus  IRQ_READY:1;                    /*!< IRQ: READY state */
  SFlagStatus  IRQ_STANDBY_DELAYED:1;          /*!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles */
  SFlagStatus  IRQ_LOW_BATT_LVL:1;             /*!< IRQ: Battery level below threshold*/
  SFlagStatus  IRQ_POR:1;                      /*!< IRQ: Power On Reset */
  SFlagStatus  IRQ_BOR:1;                      /*!< IRQ: Brown out event (both accurate and inaccurate)*/
  SFlagStatus  IRQ_LOCK:1;                     /*!< IRQ: LOCK state */
  SFlagStatus  IRQ_VCO_CALIBRATION_END:1;      /*!< IRQ: End of VCO calibration procedure */
  SFlagStatus  IRQ_PA_CALIBRATION_END:1;       /*!< IRQ: End of PA calibration procedure */
  
  SFlagStatus  IRQ_PM_COUNT_EXPIRED:1;         /*!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5) */
  SFlagStatus  IRQ_XO_COUNT_EXPIRED:1;         /*!< IRQ: only for debug; Crystal oscillator settling time counter expired */
  SFlagStatus  IRQ_TX_START_TIME:1;            /*!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER */
  SFlagStatus  IRQ_RX_START_TIME:1;            /*!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER */
  SFlagStatus  IRQ_RX_TIMEOUT:1;               /*!< IRQ: RX operation timeout */
  SFlagStatus  IRQ_RX_SNIFF_TIMEOUT:1;         /*!< IRQ: RX sniff opeartion timeout */
  SFlagStatus  :2;                             /*!< Reserved bit */
} S2LPIrqs;


extern HAL_StatusTypeDef S2LPGpioIrqConfig(IrqList xIrq, SFunctionalState xNewState);
extern HAL_StatusTypeDef S2LPGpioIrqDeInit(void);
extern HAL_StatusTypeDef S2LPGpioIrqGetStatus(S2LPIrqs* pxIrqStatus);
extern HAL_StatusTypeDef S2LPGpioIrqClearStatus(void);

