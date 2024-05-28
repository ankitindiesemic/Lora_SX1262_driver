#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include "lora_driver.h"
#include "gpio_config.h"

// LOG_MODULE_REGISTER(lora_sx127x, LOG_LEVEL_INF);
LOG_MODULE_REGISTER(lora_sx126x, LOG_LEVEL_DBG);

// Message to transmit
char message[] = "HeLoRa World!";
uint8_t nBytes = sizeof(message);
uint8_t counter = 0;

uint8_t sf = 9;      // LoRa spreading factor: 7
uint32_t bw = 125000; // Bandwidth: 125 kHz
uint8_t cr = 4;       // Coding rate: 4/5

int main(void)
{
    int ret;
    ret = sx1262_spi_comm_init();
    if (ret)
    {
        return ret;
    }

    sx126x_begin();
    LOG_INF("Starting LORA");
    uint8_t xtalA = 0x12;
    uint8_t xtalB = 0x12;
    
    LOG_INF("Set RF module to use XTAL as clock reference");
    sx126x_setXtalCap(xtalA, xtalB);
    LOG_INF("Setting lora frequecy!");
    sx126x_setFrequency(868000000);
    k_sleep(K_MSEC(100));
    LOG_INF("Set TX power");
    sx126x_setTxPower(17, SX126X_TX_POWER_SX1262);
    LOG_INF("Setting modulation parameters");
    sx126x_setLoRaModulation(sf, bw, cr, false);
    LOG_INF("Setting packet params");
    uint8_t headerType = SX126X_HEADER_EXPLICIT; // Explicit header mode
    uint16_t preambleLength = 12;                // Set preamble length to 12
    uint8_t payloadLength = 15;                  // Initialize payloadLength to 15
    bool crcType = true;                         // Set CRC enable
    sx126x_setLoRaPacket(headerType, preambleLength, payloadLength, crcType, false);
    LOG_INF("Set sync word");
    sx126x_setSyncWord(0x3444);
    LOG_INF("==============LORA TRANSMITTER==============");
    k_sleep(K_MSEC(100));

    while (true)
    {
        sx126x_beginPacket();
        sx126x_write_char(message, nBytes);
        // sx126x_write_data(&counter, 1);
        sx126x_write(&counter);
        sx126x_endPacket(0U);

        LOG_INF("%s %d", message, counter);
        counter++;

        sx126x_wait(0U);
        uint32_t time = sx126x_transmitTime();
        LOG_INF("Transmit Time: %d ms", time);
        uint8_t status = sx126x_status();

        if (status == SX126X_STATUS_TX_DONE)
        {
            LOG_INF("TxDone");
        }
        k_sleep(K_MSEC(5000));
    }

    return 0;
}

// int main(void)
// {
//     sx126x_begin();
//     LOG_INF("Starting LORA");
//     uint8_t xtalA = 0x12;
//     uint8_t xtalB = 0x12;
//     LOG_INF("Set RF module to use XTAL as clock reference");
//     sx126x_setXtalCap(xtalA, xtalB);
//     LOG_INF("Setting lora frequecy!");
//     sx126x_setFrequency(868000000);
//     k_sleep(K_MSEC(100));
//     LOG_INF("Set RX Gain");
//     sx126x_setRxGain(SX126X_RX_GAIN_POWER_SAVING);
//     LOG_INF("Setting modulation parameters");
//     sx126x_setSpreadingFactor(7);
//     sx126x_setBandwidth(125000);
//     sx126x_setCodeRate(5);
//     LOG_INF("Setting packet params");
//     sx126x_setHeaderType(SX127X_HEADER_EXPLICIT);
//     sx126x_setPreambleLength(12);
//     sx126x_setPayloadLength(15);
//     sx126x_setCrcEnable(true);
//     LOG_INF("Set sync word");
//     sx126x_setSyncWord(0x3444);
//     LOG_INF("==============LORA RECEIVER==============");
//     k_sleep(K_MSEC(100));

//     while (true)
//     {
//         LOG_INF("LORA request");
//         sx126x_request(0U);
//         sx126x_wait(0U);

//         // const uint8_t msg_len = sx126x_available() - 1;
//         // char message[msg_len];
//         // sx126x_read_char(message, msg_len);
//         // uint8_t counter;
//         // sx126x_read_data(&counter, 1);

//         // LOG_INF("%s %d", message, counter);
//         // LOG_INF("RSSI: %i", sx126x_packetRssi());
//         // uint8_t status = sx126x_status();
//         // if (status == SX126X_STATUS_CRC_ERR)
//         // {
//         //     LOG_WRN("CRC Error");
//         // } else if (status == SX126X_STATUS_HEADER_ERR)
//         // {
//         //     LOG_WRN("Packet header error");
//         // }

//     }

//     return 0;
// }


// int main(void)
// {
//     int ret;
//     ret = gpio_init();
//     if (ret)
//     {
//         return ret;
//     }
//     sx126x_reset(&lora_reset_pin);

//     LOG_INF("Starting LORA");
//     uint8_t xtalA = 0x12;
//     uint8_t xtalB = 0x12;
//     LOG_INF("Set RF module to use XTAL as clock reference");
//     // sx126x_setXtalCap(xtalA, xtalB);
//     sx127x_setModem(SX127X_LORA_MODEM);
//     sx127x_setTXPower(17, SX127X_TX_POWER_PA_BOOST);
//     // sx127x_setRXGain(SX127X_RX_GAIN_BOOSTED, SX127X_RX_GAIN_AUTO);
//     LOG_INF("Setting lora frequecy!");
//     sx127x_setFrequency(868000000);
//     k_sleep(K_MSEC(100));
//     LOG_INF("Set TX power");
//     sx127x_setTXPower(17, SX127X_TX_POWER_PA_BOOST);
//     LOG_INF("Setting modulation parameters");
//     sx127x_setSpreadingFactor(7);
//     sx127x_setBandwidth(125000);
//     sx127x_setCodeRate(5);
//     LOG_INF("Setting packet params");
//     sx127x_setHeaderType(SX127X_HEADER_EXPLICIT);
//     sx127x_setPreambleLength(12);
//     sx127x_setPayloadLength(15);
//     sx127x_setCrcType(true);
//     LOG_INF("Set sync word");
//     sx127x_setSyncWord(0x3444);
//     sx127x_set_DIO0();
//     LOG_INF("==============LORA TRANSMITTER==============");
//     k_sleep(K_MSEC(100));
//     while (true)
//     {
//         sx127x_beginPacket();
//         sx127x_write_char(message, nBytes);
//         sx127x_write_data(&counter, 1);
//         sx127x_endPacket(0U);

//         LOG_INF("%s %d", message, counter);
//         counter++;

//         sx127x_wait(0U);
//         uint32_t time = sx127x_transmitTime();
//         LOG_INF("Transmit Time: %d ms", time);
//         uint8_t status = sx127x_status();
//         if (status == SX127X_STATUS_TX_DONE)
//         {
//             LOG_INF("TxDone");
//         }
//         k_sleep(K_MSEC(5000));
//     }

//     return 0;
// }
