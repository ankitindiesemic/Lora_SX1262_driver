#ifndef _LORA_DRIVER_SX127X_
#define _LORA_DRIVER_SX127X_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "gpio_config.h"

//* Register Definitions for LORA Mode */
// SX126X register map
#define SX126X_REG_FSK_WHITENING_INITIAL_MSB 0x06B8
#define SX126X_REG_FSK_CRC_INITIAL_MSB 0x06BC
#define SX126X_REG_FSK_SYNC_WORD_0 0x06C0
#define SX126X_REG_FSK_NODE_ADDRESS 0x06CD
#define SX126X_REG_IQ_POLARITY_SETUP 0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB 0x0740
#define SX126X_REG_RANDOM_NUMBER_GEN 0x0819
#define SX126X_REG_TX_MODULATION 0x0889
#define SX126X_REG_RX_GAIN 0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG 0x08D8
#define SX126X_REG_OCP_CONFIGURATION 0x08E7
#define SX126X_REG_RTC_CONTROL 0x0902
#define SX126X_REG_XTA_TRIM 0x0911
#define SX126X_REG_XTB_TRIM 0x0912
#define SX126X_REG_EVENT_MASK 0x0944

/* SetSleep */
#define SX126X_SLEEP_COLD_START 0x00     // sleep mode: cold start, configuration is lost (default)
#define SX126X_SLEEP_WARM_START 0x04     //             warm start, configuration is retained
#define SX126X_SLEEP_COLD_START_RTC 0x01 //             cold start and wake on RTC timeout
#define SX126X_SLEEP_WARM_START_RTC 0x05 //             warm start and wake on RTC timeout

/* SetStandby */
#define SX126X_STANDBY_RC 0x00   // standby mode: using 13 MHz RC oscillator
#define SX126X_STANDBY_XOSC 0x01 //               using 32 MHz crystal oscillator

/* SetTx */
#define SX126X_TX_SINGLE 0x000000 // Tx timeout duration: no timeout (Rx single mode)

/* SetRx */
#define SX126X_RX_SINGLE 0x000000     // Rx timeout duration: no timeout (Rx single mode)
#define SX126X_RX_CONTINUOUS 0xFFFFFF //                      infinite (Rx continuous mode)

/* SetRegulatorMode */
#define SX126X_REGULATOR_LDO 0x00   // set regulator mode: LDO (default)
#define SX126X_REGULATOR_DC_DC 0x01 //                     DC-DC

/* CalibrateImage */
#define SX126X_CAL_IMG_430 0x6B // ISM band: 430-440 Mhz Freq1
#define SX126X_CAL_IMG_440 0x6F //           430-440 Mhz Freq2
#define SX126X_CAL_IMG_470 0x75 //           470-510 Mhz Freq1
#define SX126X_CAL_IMG_510 0x81 //           470-510 Mhz Freq2
#define SX126X_CAL_IMG_779 0xC1 //           779-787 Mhz Freq1
#define SX126X_CAL_IMG_787 0xC5 //           779-787 Mhz Freq2
#define SX126X_CAL_IMG_863 0xD7 //           863-870 Mhz Freq1
#define SX126X_CAL_IMG_870 0xDB //           863-870 Mhz Freq2
#define SX126X_CAL_IMG_902 0xE1 //           902-928 Mhz Freq1
#define SX126X_CAL_IMG_928 0xE9 //           902-928 Mhz Freq2

/* SetPaConfig */
#define SX126X_TX_POWER_SX1261 0x01 // device version for TX power: SX1261
#define SX126X_TX_POWER_SX1262 0x02 //                            : SX1262
#define SX126X_TX_POWER_SX1268 0x08 //                            : SX1268

/* SetRxTxFallbackMode */
#define SX126X_FALLBACK_FS 0x40         // after Rx/Tx go to: FS mode
#define SX126X_FALLBACK_STDBY_XOSC 0x30 //                    standby mode with crystal oscillator
#define SX126X_FALLBACK_STDBY_RC 0x20   //                    standby mode with RC oscillator (default)

/* SetDioIrqParams */
#define SX126X_IRQ_TX_DONE 0x0001           // packet transmission completed
#define SX126X_IRQ_RX_DONE 0x0002           // packet received
#define SX126X_IRQ_PREAMBLE_DETECTED 0x0004 // preamble detected
#define SX126X_IRQ_SYNC_WORD_VALID 0x0008   // valid sync word detected
#define SX126X_IRQ_HEADER_VALID 0x0010      // valid LoRa header received
#define SX126X_IRQ_HEADER_ERR 0x0020        // LoRa header CRC error
#define SX126X_IRQ_CRC_ERR 0x0040           // wrong CRC received
#define SX126X_IRQ_CAD_DONE 0x0080          // channel activity detection finished
#define SX126X_IRQ_CAD_DETECTED 0x0100      // channel activity detected
#define SX126X_IRQ_TIMEOUT 0x0200           // Rx or Tx timeout
#define SX126X_IRQ_ALL 0x03FF               // all interrupts
#define SX126X_IRQ_NONE 0x0000              // no interrupts

/* SetDio2AsRfSwitc */
#define SX126X_DIO2_AS_IRQ 0x00       // DIO2 configuration: IRQ
#define SX126X_DIO2_AS_RF_SWITCH 0x01 //                     RF switch control

/* SetDio3AsTcxoCtrl */
#define SX126X_DIO3_OUTPUT_1_6 0x00 // DIO3 voltage output for TCXO: 1.6 V
#define SX126X_DIO3_OUTPUT_1_7 0x01 //                               1.7 V
#define SX126X_DIO3_OUTPUT_1_8 0x02 //                               1.8 V
#define SX126X_DIO3_OUTPUT_2_2 0x03 //                               2.2 V
#define SX126X_DIO3_OUTPUT_2_4 0x04 //                               2.4 V
#define SX126X_DIO3_OUTPUT_2_7 0x05 //                               2.7 V
#define SX126X_DIO3_OUTPUT_3_0 0x06 //                               3.0 V
#define SX126X_DIO3_OUTPUT_3_3 0x07 //                               3.3 V
#define SX126X_TCXO_DELAY_1 0x0040  // TCXO delay time: 1 ms
#define SX126X_TCXO_DELAY_2 0x0080  //                  2 ms
#define SX126X_TCXO_DELAY_5 0x0140  //                  5 ms
#define SX126X_TCXO_DELAY_10 0x0280 //                  10 ms

/* SetRfFrequency */
#define SX126X_RF_FREQUENCY_XTAL 32000000 // XTAL frequency used for RF frequency calculation
#define SX126X_RF_FREQUENCY_SHIFT 25      // RfFreq = Frequency * 2^25 / 32000000

/* SetPacketType */
#define SX126X_FSK_MODEM 0x00  // GFSK packet type
#define SX126X_LORA_MODEM 0x01 // LoRa packet type

/* SetTxParams */
#define SX126X_PA_RAMP_10U 0x00   // ramp time: 10 us
#define SX126X_PA_RAMP_20U 0x01   //            20 us
#define SX126X_PA_RAMP_40U 0x02   //            40 us
#define SX126X_PA_RAMP_80U 0x03   //            80 us
#define SX126X_PA_RAMP_200U 0x04  //            200 us
#define SX126X_PA_RAMP_800U 0x05  //            800 us
#define SX126X_PA_RAMP_1700U 0x06 //            1700 us
#define SX126X_PA_RAMP_3400U 0x07 //            3400 us

/* SetModulationParams for LoRa packet type */
#define SX126X_BW_7800 0x00   // LoRa bandwidth: 7.8 kHz
#define SX126X_BW_10400 0x08  //                 10.4 kHz
#define SX126X_BW_15600 0x01  //                 15.6 kHz
#define SX126X_BW_20800 0x09  //                 20.8 kHz
#define SX126X_BW_31250 0x02  //                 31.25 kHz
#define SX126X_BW_41700 0x0A  //                 41.7 kHz
#define SX126X_BW_62500 0x03  //                 62.5 kHz
#define SX126X_BW_125000 0x04 //                 125 kHz
#define SX126X_BW_250000 0x05 //                 250 kHz
#define SX126X_BW_500000 0x06 //                 500 kHz
#define SX126X_CR_4_4 0x00    // LoRa coding rate: 4/4 (no coding rate)
#define SX126X_CR_4_5 0x01    //                   4/5
#define SX126X_CR_4_6 0x02    //                   4/6
#define SX126X_CR_4_7 0x03    //                   4/7
#define SX126X_CR_4_8 0x04    //                   4/8
#define SX126X_LDRO_OFF 0x00  // LoRa low data rate optimization: disabled
#define SX126X_LDRO_ON 0x00   //                                  enabled

/* SetModulationParams for FSK packet type */
#define SX126X_PULSE_NO_FILTER 0x00       // FSK pulse shape: no filter applied
#define SX126X_PULSE_GAUSSIAN_BT_0_3 0x08 //                  Gaussian BT 0.3
#define SX126X_PULSE_GAUSSIAN_BT_0_5 0x09 //                  Gaussian BT 0.5
#define SX126X_PULSE_GAUSSIAN_BT_0_7 0x0A //                  Gaussian BT 0.7
#define SX126X_PULSE_GAUSSIAN_BT_1 0x0B   //                  Gaussian BT 1
#define SX126X_BW_4800 0x1F               // FSK bandwidth: 4.8 kHz DSB
#define SX126X_BW_5800 0x17               //                5.8 kHz DSB
#define SX126X_BW_7300 0x0F               //                7.3 kHz DSB
#define SX126X_BW_9700 0x1E               //                9.7 kHz DSB
#define SX126X_BW_11700 0x16              //                11.7 kHz DSB
#define SX126X_BW_14600 0x0E              //                14.6 kHz DSB
#define SX126X_BW_19500 0x1D              //                19.5 kHz DSB
#define SX126X_BW_23400 0x15              //                23.4 kHz DSB
#define SX126X_BW_29300 0x0D              //                29.3 kHz DSB
#define SX126X_BW_39000 0x1C              //                39 kHz DSB
#define SX126X_BW_46900 0x14              //                46.9 kHz DSB
#define SX126X_BW_58600 0x0C              //                58.6 kHz DSB
#define SX126X_BW_78200 0x1B              //                78.2 kHz DSB
#define SX126X_BW_93800 0x13              //                93.8 kHz DSB
#define SX126X_BW_117300 0x0B             //                117.3 kHz DSB
#define SX126X_BW_156200 0x1A             //                156.2 kHz DSB
#define SX126X_BW_187200 0x12             //                187.2 kHz DSB
#define SX126X_BW_234300 0x0A             //                232.3 kHz DSB
#define SX126X_BW_312000 0x19             //                312 kHz DSB
#define SX126X_BW_373600 0x11             //                373.6 kHz DSB
#define SX126X_BW_467000 0x09             //                476 kHz DSB

/* SetPacketParams for LoRa packet type */
#define SX126X_HEADER_EXPLICIT 0x00 // LoRa header mode: explicit
#define SX126X_HEADER_IMPLICIT 0x01 //                   implicit
#define SX126X_CRC_OFF 0x00         // LoRa CRC mode: disabled
#define SX126X_CRC_ON 0x01          //                enabled
#define SX126X_IQ_STANDARD 0x00     // LoRa IQ setup: standard
#define SX126X_IQ_INVERTED 0x01     //                inverted

/* SetPacketParams for FSK packet type */
#define SX126X_PREAMBLE_DET_LEN_OFF 0x00 // FSK preamble detector length: off
#define SX126X_PREAMBLE_DET_LEN_8 0x04   //                               8-bit
#define SX126X_PREAMBLE_DET_LEN_16 0x05  //                               16-bit
#define SX126X_PREAMBLE_DET_LEN_24 0x06  //                               24-bit
#define SX126X_PREAMBLE_DET_LEN_32 0x07  //                               32-bit
#define SX126X_ADDR_COMP_OFF 0x00        // FSK address filtering: off
#define SX126X_ADDR_COMP_NODE 0x01       //                        filtering on node address
#define SX126X_ADDR_COMP_ALL 0x02        //                        filtering on node and broadcast address
#define SX126X_PACKET_KNOWN 0x00         // FSK packet type: the packet length known on both side
#define SX126X_PACKET_VARIABLE 0x01      //                  the packet length on variable size
#define SX126X_CRC_0 0x01                // FSK CRC type: no CRC
#define SX126X_CRC_1 0x00                //               CRC computed on 1 byte
#define SX126X_CRC_2 0x02                //               CRC computed on 2 byte
#define SX126X_CRC_1_INV 0x04            //               CRC computed on 1 byte and inverted
#define SX126X_CRC_2_INV 0x06            //               CRC computed on 2 byte and inverted
#define SX126X_WHITENING_OFF 0x00        // FSK whitening: no encoding
#define SX126X_WHITENING_ON 0x01         //                whitening enable

/* SetCadParams */
#define SX126X_CAD_ON_1_SYMB 0x00  // number of symbols used for CAD: 1
#define SX126X_CAD_ON_2_SYMB 0x01  //                                 2
#define SX126X_CAD_ON_4_SYMB 0x02  //                                 4
#define SX126X_CAD_ON_8_SYMB 0x03  //                                 8
#define SX126X_CAD_ON_16_SYMB 0x04 //                                 16
#define SX126X_CAD_EXIT_STDBY 0x00 // after CAD is done, always exit to STDBY_RC mode
#define SX126X_CAD_EXIT_RX 0x01    // after CAD is done, exit to Rx mode if activity is detected

/* GetStatus */
#define SX126X_STATUS_DATA_AVAILABLE 0x04  // command status: packet received and data can be retrieved
#define SX126X_STATUS_CMD_TIMEOUT 0x06     //                 SPI command timed out
#define SX126X_STATUS_CMD_ERROR 0x08       //                 invalid SPI command
#define SX126X_STATUS_CMD_FAILED 0x0A      //                 SPI command failed to execute
#define SX126X_STATUS_CMD_TX_DONE 0x0C     //                 packet transmission done
#define SX126X_STATUS_MODE_STDBY_RC 0x20   // current chip mode: STDBY_RC
#define SX126X_STATUS_MODE_STDBY_XOSC 0x30 //                    STDBY_XOSC
#define SX126X_STATUS_MODE_FS 0x40         //                    FS
#define SX126X_STATUS_MODE_RX 0x50         //                    RX
#define SX126X_STATUS_MODE_TX 0x60         //                    TX

/* GetDeviceErrors */
#define SX126X_RC64K_CALIB_ERR 0x0001 // device errors: RC64K calibration failed
#define SX126X_RC13M_CALIB_ERR 0x0002 //                RC13M calibration failed
#define SX126X_PLL_CALIB_ERR 0x0004   //                PLL calibration failed
#define SX126X_ADC_CALIB_ERR 0x0008   //                ADC calibration failed
#define SX126X_IMG_CALIB_ERR 0x0010   //                image calibration failed
#define SX126X_XOSC_START_ERR 0x0020  //                crystal oscillator failed to start
#define SX126X_PLL_LOCK_ERR 0x0040    //                PLL failed to lock
#define SX126X_PA_RAMP_ERR 0x0100     //                PA ramping failed

/* LoraSyncWord */
#define SX126X_LORA_SYNC_WORD_PUBLIC 0x3444  // LoRa SyncWord for public network
#define SX126X_LORA_SYNC_WORD_PRIVATE 0x0741 // LoRa SyncWord for private network (default)

/* RxGain */
#define SX126X_RX_GAIN_POWER_SAVING 0x00 // gain used in Rx mode: power saving gain (default)
#define SX126X_RX_GAIN_BOOSTED 0x01      // boosted gain
#define SX126X_POWER_SAVING_GAIN 0x94    // power saving gain register value
#define SX126X_BOOSTED_GAIN 0x96         // boosted gain register value

#define SX126X_BUSY_TIMEOUT 5000 // Default timeout for checking busy pin

// Status TX and RX operation
#define SX126X_STATUS_DEFAULT                   0
#define SX126X_STATUS_TX_WAIT                   1
#define SX126X_STATUS_TX_TIMEOUT                2
#define SX126X_STATUS_TX_DONE                   3
#define SX126X_STATUS_RX_WAIT                   4
#define SX126X_STATUS_RX_CONTINUOUS             5
#define SX126X_STATUS_RX_TIMEOUT                6
#define SX126X_STATUS_RX_DONE                   7
#define SX126X_STATUS_HEADER_ERR                8
#define SX126X_STATUS_CRC_ERR                   9
#define SX126X_STATUS_CAD_WAIT                  10
#define SX126X_STATUS_CAD_DETECTED              11
#define SX126X_STATUS_CAD_DONE                  12

/**
 * Function to provide spi parameters (Should be called first)
 * @param dev device type struct for spi device
 * @param cfg spi_config type struct for spi config
 * @param cs spi_cs_control type struct for cs pin
*/
void sx126x_spi_params(const struct device dev, const struct spi_config cfg, const struct spi_cs_control cs);
/**
 * Function do a reset of lora
 * @param spec Reset pin (gpio_dt_spec struct type)
*/
void sx126x_reset(struct gpio_dt_spec *spec);

/**
 * Function to write to register address
 * @param address Register address
 * @param data Data value
*/
void sx126x_write_register(uint16_t address, uint8_t data);

/**
 * Function to read a register address
 * @param address Register address
*/
uint8_t sx126x_read_register(uint16_t address);

/**
 * Function to set TX power
 * @param power tx power in dbm
 * @param paPin 0 for RFO pin or 1 for PA_BOOST pin
*/
void sx126x_setTxPower(uint8_t txPower, uint8_t version);
void sx126x_writeBuffer(uint8_t offset, uint8_t *data);
void sx126x_readBuffer(uint8_t offset, uint8_t *data);
bool sx126x_busyCheck(uint32_t timeout);
void sx126x_getStatus(uint8_t *status);
bool sx126x_begin();
int sx1262_spi_comm_init(void);

// SX126x driver: Workaround functions
void sx126x_fixResistanceAntenna();
void sx126x_fixLoRaBw500(uint32_t bw);
void sx126x_fixRxTimeout();
void sx126x_fixInvertedIq(uint8_t invertIq);

// SX126x driver: Operational Modes Commands
void sx126x_setStandby(uint8_t standbyConfig);
void sx126x_getStatus(uint8_t *status);
uint8_t getMode(void);
void sx126x_setPacketType(uint8_t packetType);

void sx126x_setSleep(uint8_t sleepConfig);
void sx126x_setStandby(uint8_t standbyConfig);
void sx126x_setFs();
void sx126x_setTx(uint32_t timeout);
void sx126x_setRx(uint32_t timeout);
void sx126x_stopTimerOnPreamble(uint8_t enable);
void sx126x_setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod);
void sx126x_setCad();
void sx126x_setTxContinuousWave();
void sx126x_setTxInfinitePreamble();
void sx126x_setRegulatorMode(uint8_t modeParam);
void sx126x_calibrate(uint8_t calibParam);
void sx126x_calibrateImage(uint8_t freq1, uint8_t freq2);
void sx126x_setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
void sx126x_setRxTxFallbackMode(uint8_t fallbackMode);
void sx126x_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
void sx126x_getIrqStatus(uint16_t *irqStatus);
void sx126x_clearIrqStatus(uint16_t clearIrqParam);
void sx126x_setDio2AsRfSwitchCtrl(uint8_t enable);
void sx126x_setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t delay);
void sx126x_setRfFrequency(uint32_t rfFreq);
void sx126x_setPacketType(uint8_t packetType);
void sx126x_getPacketType(uint8_t *packetType);
void sx126x_setTxParams(uint8_t power, uint8_t rampTime);
void sx126x_setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
void sx126x_setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
void sx126x_setPacketParamsLoRa(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq);
void sx126x_setPacketParamsFSK(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
void sx126x_setCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout);
void sx126x_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void sx126x_setLoRaSymbNumTimeout(uint8_t symbnum);
void sx126x_getRxBufferStatus(uint8_t *payloadLengthRx, uint8_t *rxStartBufferPointer);
void sx126x_getPacketStatus(uint8_t *rssiPkt, uint8_t *snrPkt, uint8_t *signalRssiPkt);
void sx126x_getRssiInst(uint8_t *rssiInst);
void sx126x_getStats(uint16_t *nbPktReceived, uint16_t *nbPktCrcError, uint16_t *nbPktHeaderErr);
void sx126x_resetStats();
void sx126x_getDeviceErrors(uint16_t *opError);
void sx126x_clearDeviceErrors();

void sx126x_setDio2RfSwitch(bool enable);
void sx126x_setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime);
void sx126x_setXtalCap(uint8_t xtalA, uint8_t xtalB);
void sx126x_setRegulator(uint8_t regMode);
void sx126x_setCurrentProtection(uint8_t current);
uint8_t sx126x_getModem();
void sx126x_setModem(uint8_t modem);
void sx126x_setFrequency(uint32_t frequency);
void sx126x_setTxPower(uint8_t txPower, uint8_t version);
void sx126x_setRxGain(uint8_t boost);
void sx126x_setLoRaModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro);
void sx126x_setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq);
void sx126x_setSpreadingFactor(uint8_t sf);
void sx126x_setBandwidth(uint32_t bw);
void sx126x_setCodeRate(uint8_t cr);
void sx126x_setLdroEnable(bool ldro);
void sx126x_setHeaderType(uint8_t headerType);
void sx126x_setPreambleLength(uint16_t preambleLength);
void sx126x_setPayloadLength(uint8_t payloadLength);
void sx126x_setCrcEnable(bool crcType);
void sx126x_setInvertIq(bool invertIq);
void sx126x_setSyncWord(uint16_t syncWord);
void sx126x_setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
void sx126x_setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
void sx126x_setFskSyncWord(uint8_t* sw);
void sx126x_setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr);
void sx126x_setFskCrc(uint16_t crcInit, uint16_t crcPolynom);
void sx126x_setFskWhitening(uint16_t whitening);
void sx126x_beginPacket();
bool sx126x_endPacket(uint32_t timeout);
void sx126x_write_data(uint8_t *data, uint8_t length);
void sx126x_write_char(char *data, uint8_t length);
void sx126x_write(uint8_t data);
bool sx126x_request(uint32_t timeout);
bool sx126x_listen(uint32_t rxPeriod, uint32_t sleepPeriod);
uint8_t sx126x_available();
uint8_t sx126x_read();
uint8_t sx126x_read_data(uint8_t *data, uint8_t length);
uint8_t sx126x_read_char(char *data, uint8_t length);
void sx126x_purge(uint8_t length);
bool sx126x_wait(uint32_t timeout);
uint8_t sx126x_status();
uint32_t sx126x_transmitTime();
float sx126x_dataRate();
int16_t sx126x_packetRssi();
float sx126x_snr();
int16_t sx126x_signalRssi();
int16_t sx126x_rssiInst();
uint16_t sx126x_getError();

// uint32_t sx126x_Random();

// void sx126x_InterruptTX();

// void sx126x_InterruptRX();

// void sx126x_InterruptRxContinuous();

/* Register Definitions for LORA Mode */
#define SX127X_REG_FIFO                         0x00
#define SX127X_REG_OP_MODE                      0x01
#define SX127X_REG_FRF_MSB                      0x06
#define SX127X_REG_FRF_MID                      0x07
#define SX127X_REG_FRF_LSB                      0x08
#define SX127X_REG_PA_CONFIG                    0x09
#define SX127X_REG_OCP                          0x0B
#define SX127X_REG_LNA                          0x0C
#define SX127X_REG_FIFO_ADDR_PTR                0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR            0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR            0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR         0x10
#define SX127X_REG_IRQ_FLAGS                    0x12
#define SX127X_REG_RX_NB_BYTES                  0x13
#define SX127X_REG_PKT_SNR_VALUE                0x19
#define SX127X_REG_PKT_RSSI_VALUE               0x1A
#define SX127X_REG_RSSI_VALUE                   0x1B
#define SX127X_REG_MODEM_CONFIG_1               0x1D
#define SX127X_REG_MODEM_CONFIG_2               0x1E
#define SX127X_REG_SYMB_TIMEOUT                 0x1F
#define SX127X_REG_PREAMBLE_MSB                 0x20
#define SX127X_REG_PREAMBLE_LSB                 0x21
#define SX127X_REG_PAYLOAD_LENGTH               0x22
#define SX127X_REG_MODEM_CONFIG_3               0x26
#define SX127X_REG_FREQ_ERROR_MSB               0x28
#define SX127X_REG_FREQ_ERROR_MID               0x29
#define SX127X_REG_FREQ_ERROR_LSB               0x2A
#define SX127X_REG_RSSI_WIDEBAND                0x2C
#define SX127X_REG_DETECTION_OPTIMIZE           0x31
#define SX127X_REG_INVERTIQ                     0x33
#define SX127X_REG_DETECTION_THRESHOLD          0x37
#define SX127X_REG_SYNC_WORD                    0x39
#define SX127X_REG_INVERTIQ2                    0x3B
#define SX127X_REG_DIO_MAPPING_1                0x40
#define SX127X_REG_VERSION                      0x42
#define SX127X_REG_TCXO                         0x4B
#define SX127X_REG_PA_DAC                       0x4D

/* Modem Options */
#define SX127X_FSK_MODEM                        0x00        
#define SX127X_LORA_MODEM                       0x01        
#define SX127X_OOK_MODEM                        0x02        

/* Long range mode and modulation type */
#define SX127X_LONG_RANGE_MODE                  0x80        
#define SX127X_MODULATION_OOK                   0x20        
#define SX127X_MODULATION_FSK                   0x00

/* Device Modes */
#define SX127X_MODE_SLEEP                       0x00        // sleep
#define SX127X_MODE_STDBY                       0x01        // standby
#define SX127X_MODE_TX                          0x03        // transmit
#define SX127X_MODE_RX_CONTINUOUS               0x05        // continuous receive
#define SX127X_MODE_RX_SINGLE                   0x06        // single receive
#define SX127X_MODE_CAD                         0x07        // channel activity detection (CAD)

/* Rx Operation mode */
#define SX127X_RX_SINGLE                        0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define SX127X_RX_CONTINUOUS                    0xFFFFFF    //                      infinite (Rx continuous mode)

/* Tx Power options */
#define SX127X_TX_POWER_RFO                     0x00        // output power is limited to +14 dBm
#define SX127X_TX_POWER_PA_BOOST                0x80        // output power is limited to +20 dBm

/* Rx Gain options */
#define SX127X_RX_GAIN_POWER_SAVING             0x00        // gain used in Rx mode: power saving gain (default)
#define SX127X_RX_GAIN_BOOSTED                  0x01        //                       boosted gain
#define SX127X_RX_GAIN_AUTO                     0x00        // option enable auto gain controller (AGC)

/* Header Type */
#define SX127X_HEADER_EXPLICIT                  0x00        // explicit header mode
#define SX127X_HEADER_IMPLICIT                  0x01        // implicit header mode

/* LORA Syncword */
#define SX127X_SYNCWORD_LORAWAN                 0x34        // reserved LoRaWAN syncword

/* Oscillator options */
#define SX127X_OSC_CRYSTAL                      0x00        // crystal oscillator with external crystal
#define SX127X_OSC_TCXO                         0x10        // external clipped sine TCXO AC-connected to XTA pin

/* DIO mapping */
#define SX127X_DIO0_RX_DONE                     0x00        // set DIO0 interrupt for: RX done
#define SX127X_DIO0_TX_DONE                     0x40        //                         TX done
#define SX127X_DIO0_CAD_DONE                    0x80        //                         CAD done

/* IRQ Flags */
#define SX127X_IRQ_CAD_DETECTED                 0x01        // Valid Lora signal detected during CAD operation
#define SX127X_IRQ_FHSS_CHANGE                  0x02        // FHSS change channel interrupt
#define SX127X_IRQ_CAD_DONE                     0x04        // channel activity detection finished
#define SX127X_IRQ_TX_DONE                      0x08        // packet transmission completed
#define SX127X_IRQ_HEADER_VALID                 0x10        // valid LoRa header received
#define SX127X_IRQ_CRC_ERR                      0x20        // wrong CRC received
#define SX127X_IRQ_RX_DONE                      0x40        // packet received
#define SX127X_IRQ_RX_TIMEOUT                   0x80        // waiting packet received timeout

/* RSSI offset */
#define SX127X_RSSI_OFFSET_LF                   164         // low band frequency RSSI offset
#define SX127X_RSSI_OFFSET_HF                   157         // high band frequency RSSI offset
#define SX127X_RSSI_OFFSET                      139         // frequency RSSI offset for SX1272
#define SX127X_BAND_THRESHOLD                   525E6       // threshold between low and high band frequency

// Status TX and RX operation
#define SX127X_STATUS_DEFAULT                     0           // default status (false)
#define SX127X_STATUS_TX_WAIT                     1
#define SX127X_STATUS_TX_TIMEOUT                  2
#define SX127X_STATUS_TX_DONE                     3
#define SX127X_STATUS_RX_WAIT                     4
#define SX127X_STATUS_RX_CONTINUOUS               5
#define SX127X_STATUS_RX_TIMEOUT                  6
#define SX127X_STATUS_RX_DONE                     7
#define SX127X_STATUS_HEADER_ERR                  8
#define SX127X_STATUS_CRC_ERR                     9
#define SX127X_STATUS_CAD_WAIT                    10
#define SX127X_STATUS_CAD_DETECTED                11
#define SX127X_STATUS_CAD_DONE                    12









void sx127x_setTXPower(uint8_t power, uint8_t paPin);

void sx127x_setRXGain(uint8_t boost, uint8_t level);

void sx127x_setLoraModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro);

void sx127x_setLoraPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq);

void sx127x_setSpreadingFactor(uint8_t sf);

void sx127x_setBandwidth(uint32_t bw);

void sx127x_setCodeRate(uint8_t cr);

void sx127x_setLdro(bool ldro);

void sx127x_setHeaderType(uint8_t headerType);

void sx127x_setPreambleLength(uint16_t preambleLength);

void sx127x_setPayloadLength(uint16_t payloadLength);

void sx127x_setCrcType(bool crcType);

void sx127x_setInvertIq(bool invertIq);

void sx127x_setSyncWord(uint16_t syncWord);

void sx127x_beginPacket();

bool sx127x_endPacket(uint32_t timeout);

void sx127x_write_char(char* data, uint8_t length);

void sx127x_write_data(uint8_t *data, uint8_t length);

bool sx127x_request(uint32_t timeout);

uint8_t sx127x_available();

uint8_t sx127x_read_char(char* data, uint8_t length);

uint8_t sx127x_read_data(uint8_t* data, uint8_t length);

void sx127x_purge(uint8_t length);

bool sx127x_wait(uint32_t timeout);

uint8_t sx127x_status();

uint32_t sx127x_transmitTime();

float sx127x_dataRate();

int16_t sx127x_PacketRssi();

int16_t sx127x_Rssi();

float sx127x_snr();

void sx127x_set_DIO0();




#endif