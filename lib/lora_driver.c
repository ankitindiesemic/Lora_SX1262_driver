#include "lora_driver.h"

// LOG_MODULE_DECLARE(lora_sx127x, LOG_LEVEL_INF);
LOG_MODULE_REGISTER(sx1262_driver, LOG_LEVEL_DBG);

#define LORA_DEVICE DT_COMPAT_GET_ANY_STATUS_OKAY(lora_sx127x)
#define SPI_OP SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |  SPI_LINES_SINGLE
#define LORA_SPI_DELAY 10

static struct spi_cs_control spi_cs_pin = {
    .delay = LORA_SPI_DELAY,
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(LORA_DEVICE)};

struct spi_config spi_cfg = {
    .frequency = DT_PROP(LORA_DEVICE, spi_max_frequency),
    .operation = SPI_OP,
    .slave = DT_REG_ADDR(LORA_DEVICE),
    .cs.gpio = SPI_CS_GPIOS_DT_SPEC_GET(LORA_DEVICE),
    .cs.delay = LORA_SPI_DELAY};

static const struct device *spi_dev = DEVICE_DT_GET(DT_BUS(LORA_DEVICE));

uint8_t _modem;
uint8_t _sf = 7;
uint32_t _bw = 125000;
uint8_t _cr = 4;
bool _ldro;
uint8_t _headerType = SX126X_HEADER_EXPLICIT;
uint16_t _preambleLength = 12;
uint8_t _payloadLength;
bool _crcType;
bool _invertIq;
uint8_t _statusWait;
volatile static uint16_t _statusIrq;
static uint32_t _transmitTime;
static uint8_t _bufferIndex;
static uint8_t _payloadTxRx;
static int8_t _irqStatic;
static int8_t _pinToLow;
uint16_t _random;
int8_t _irq;
uint32_t _frequency;

int lora_write_val(uint8_t opcode, uint16_t *address, uint8_t *data)
{
    int err;
    size_t add_len = sizeof(*address);
    size_t data_len = sizeof(*data);
    size_t opcode_len = sizeof(opcode);

    const struct spi_buf tx_bufs[] = {
        {.buf = opcode,
         .len = opcode_len},

        {.buf = address,
         .len = add_len},
        {.buf = data,
         .len = data_len}};
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)};

    if (sx126x_busyCheck(500))
    {
        return;
    }

    err = spi_write(spi_dev, &spi_cfg, &tx);
    if (err)
    {
        LOG_ERR("Spi write failed!, error code: %d", err);
        return err;
    }
    return 0;
}

int lora_read_val(uint8_t opcode, uint16_t *address, uint8_t *data)
{
    int err;
    size_t add_len = sizeof(*address);
    size_t data_len = sizeof(*data);
    size_t opcode_len = sizeof(opcode);

    const struct spi_buf tx_bufs[] = {

        {.buf = opcode,
         .len = opcode_len},

        {.buf = address,
         .len = add_len},
        {.buf = 0x00,
         .len = 1}};
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs)};
    /* Read register value. */
    const struct spi_buf rx_bufs[] = {
        {.buf = NULL, .len = 1},
        {.buf = data, .len = data_len}};
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs)};

    if (sx126x_busyCheck(500))
    {
        return;
    }

    err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (err)
    {
        LOG_ERR("Spi read failed! error code: %d", err);
        return err;
    }
    return 0;
}

void sx126x_reset(struct gpio_dt_spec *spec)
{
    if (!gpio_is_ready_dt(spec))
    {
        LOG_ERR("Reset pin not ready");
        return;
    }
    int err;
    err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
    if (err)
    {
        LOG_ERR("Coudn't configure reset pin");
        return;
    }
    gpio_pin_set_dt(spec, 0);
    k_sleep(K_MSEC(1));
    gpio_pin_set_dt(spec, 1);
    k_sleep(K_MSEC(5));
}

void sx126x_write_register(uint16_t address, uint8_t data)
{
    int err;
    // uint8_t addr = address | 0x80;
    uint8_t buf[2];

    buf[0] = address >> 8;
    buf[1] = address;

    LOG_DBG("BUF VALUES: %x, %x", buf[0], buf[1]);

    err = lora_write_val(0x0D, &buf, &data);
    if (err)
    {
        LOG_ERR("write reg failed!, error: %d", err);
        while (1)
        {
        }
    }
    LOG_DBG("BUF VALUES: %2x", data);
}

uint8_t sx126x_read_register(uint16_t address)
{
    int err;
    // uint8_t addr = address & 0x7F;
    uint8_t buf[2];
    buf[0] = address >> 8;
    buf[1] = address;
    uint8_t data;
    LOG_DBG("BUF VALUES: %x, %x", buf[0], buf[1]);

    err = lora_read_val(0x1D, &buf, &data);
    if (err)
    {
        LOG_ERR("Read reg failed!, error: %d", err);
        return err;
    }
    LOG_DBG("BUF VALUES: %2x", data);
    return data;
}

int sx1262_spi_comm_init(void)
{
    int err;
    if (!device_is_ready(spi_dev))
    {
        LOG_ERR("SPI device is not ready!");
        return -1;
    }

    if (!gpio_is_ready_dt(&spi_cfg.cs.gpio))
    {
        LOG_ERR("CS pin not ready!");
        return -1;
    }
    return 0;
}


bool sx126x_begin()
{
    int ret;
    ret = gpio_init();
    if (ret)
    {
        return ret;
    }
    sx126x_reset(&lora_reset_pin);

    // check if device connect and set modem to LoRa
    sx126x_setStandby(SX126X_STANDBY_RC);
    if (getMode() != SX126X_STATUS_MODE_STDBY_RC)
    {
        return false;
    }
    sx126x_setPacketType(SX126X_LORA_MODEM);

    sx126x_fixResistanceAntenna();
    return true;
}

bool sx126x_busyCheck(uint32_t timeout)
{
    uint32_t start_time = k_uptime_get_32();

    // volatile int lora_read;
    // lora_read = gpio_pin_get_dt(&busy_pin);
    // LOG_DBG("lora Busy pin status = %d", lora_read);

    while (gpio_pin_get_dt(&busy_pin) == 1)
    {
        if (k_uptime_get_32() - start_time > timeout)
        {
            return true;
        }
    }

    return false;
}

void sx126x_writeBuffer(uint8_t offset, uint8_t *data)
{
    int err;
    uint8_t bufOfs[1] = {offset};
    err = lora_write_val(0x0E, &bufOfs, &data);
    if (err)
    {
        LOG_ERR("write reg failed!, error: %d", err);
        while (1)
        {
        }
    }
    // LOG_INF("writeBuffer data %d",data);
    return 0;
}

void sx126x_readBuffer(uint8_t offset, uint8_t *data)
{
    int err;
    uint8_t bufOfs[2] = {offset, 0x00};

    err = lora_read_val(0x1E, &bufOfs, &data);
    if (err)
    {
        LOG_ERR("Read reg failed!, error: %d", err);
        return err;
    }
    // return data;
}

void sx126x_getStatus(uint8_t *status)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xC0, &status, &data);
    if (err)
    {
        LOG_ERR("getStatus!, error: %d", err);
        return err;
    }
    LOG_INF("get status %d:", data);
}

uint8_t getMode(void)
{
    uint8_t mode;
    sx126x_getStatus(&mode);
    mode = (mode & 0x70);
    return mode;
    LOG_INF("mode :%d",mode);
}

void sx126x_fixResistanceAntenna()
{

    uint8_t value = sx126x_read_register(SX126X_REG_TX_CLAMP_CONFIG);
    value |= 0x1E;
    sx126x_write_register(SX126X_REG_TX_CLAMP_CONFIG, &value);
}

void sx126x_fixLoRaBw500(uint32_t bw)
{
    uint8_t packetType;
    sx126x_getPacketType(&packetType);
    uint8_t value = sx126x_read_register(SX126X_REG_TX_MODULATION);
    if ((packetType == SX126X_LORA_MODEM) && (bw == 500000))
    {
        value &= 0xFB;
    }
    else
    {
        value |= 0x04;
    }

    sx126x_write_register(SX126X_REG_TX_MODULATION, &value);
}

void sx126x_fixRxTimeout()
{
    uint8_t value = 0x00;
    sx126x_write_register(SX126X_REG_RTC_CONTROL, &value);
    value = sx126x_read_register(SX126X_REG_EVENT_MASK);
    value = value | 0x02;
    sx126x_write_register(SX126X_REG_EVENT_MASK, &value);
}

void sx126x_fixInvertedIq(uint8_t invertIq)
{
    uint8_t value = sx126x_read_register(SX126X_REG_IQ_POLARITY_SETUP);
    if (invertIq)
    {
        value |= 0x04;
    }
    else
    {
        value &= 0xFB;
    }
    sx126x_write_register(SX126X_REG_IQ_POLARITY_SETUP, &value);
}



void sx126x_setSleep(uint8_t sleepConfig)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x84, &sleepConfig, &data);
    if (err)
    {
        LOG_ERR("setSleep!, error: %d", err);
        return err;
    }
}

void sx126x_setStandby(uint8_t standbyConfig)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x80, &standbyConfig, &data);
    if (err)
    {
        LOG_ERR("setSleep!, error: %d", err);
        return err;
    }
}

void sx126x_setFs()
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xC1, NULL, &data);
    if (err)
    {
        LOG_ERR("setSleep!, error: %d", err);
        return err;
    }
}

void sx126x_setTx(uint32_t timeout)
{
    int err;
    uint8_t buf[3];
    uint8_t *data;
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    err = lora_read_val(0x83, &buf, &data);
    if (err)
    {
        LOG_ERR("setTx!, error: %d", err);
        return err;
    }
}

void sx126x_setRx(uint32_t timeout)
{
    int err;
    uint8_t buf[3];
    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    uint8_t *data;
    err = lora_read_val(0x82, &buf, &data);
    if (err)
    {
        LOG_ERR("setTx!, error: %d", err);
        return err;
    }
}

void sx126x_stopTimerOnPreamble(uint8_t enable)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x9F, &enable, &data);
    if (err)
    {
        LOG_ERR("stopTimerOnPreamble!, error: %d", err);
        return err;
    }
}

void sx126x_setRxDutyCycle(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    int err;
    uint8_t buf[6];
    uint8_t *data;
    buf[0] = rxPeriod >> 16;
    buf[1] = rxPeriod >> 8;
    buf[2] = rxPeriod;
    buf[3] = sleepPeriod >> 16;
    buf[4] = sleepPeriod >> 8;
    buf[5] = sleepPeriod;
    err = lora_read_val(0x94, &buf, &data);
    if (err)
    {
        LOG_ERR("setRxDutyCycle!, error: %d", err);
        return err;
    }
}

void sx126x_setCad()
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xC5, NULL, &data);
    if (err)
    {
        LOG_ERR("setCad!, error: %d", err);
        return err;
    }
}

void sx126x_setTxContinuousWave()
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xD1, NULL, &data);
    if (err)
    {
        LOG_ERR("setTxContinuousWave!, error: %d", err);
        return err;
    }
}

void sx126x_setTxInfinitePreamble()
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xD2, NULL, &data);
    if (err)
    {
        LOG_ERR("setTxInfinitePreamble!, error: %d", err);
        return err;
    }
}

void sx126x_setRegulatorMode(uint8_t modeParam)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x96, &modeParam, &data);
    if (err)
    {
        LOG_ERR("setRegulatorMode!, error: %d", err);
        return err;
    }
}

void sx126x_calibrate(uint8_t calibParam)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x89, &calibParam, &data);
    if (err)
    {
        LOG_ERR("calibrate!, error: %d", err);
        return err;
    }
}

void sx126x_calibrateImage(uint8_t freq1, uint8_t freq2)
{
    int err;
    uint8_t buf[2];
    buf[0] = freq1;
    buf[1] = freq2;
    uint8_t *data;
    err = lora_read_val(0x98, &buf, &data);
    if (err)
    {
        LOG_ERR("calibrateImage!, error: %d", err);
        return err;
    }
    // LOG_INF("calibrateImage data :%d", data);
}

void sx126x_setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{

    int err;
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    uint8_t *data;
    err = lora_read_val(0x95, &buf, &data);
    if (err)
    {
        LOG_ERR("setPaConfig!, error: %d", err);
        return err;
    }
    // LOG_INF("setPaConfig data :%d", data);
}

void sx126x_setRxTxFallbackMode(uint8_t fallbackMode)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x93, &fallbackMode, &data);
    if (err)
    {
        LOG_ERR("setRxTxFallbackMode!, error: %d", err);
        return err;
    }
}

void sx126x_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    int err;
    uint8_t buf[8];
    buf[0] = irqMask >> 8;
    buf[1] = irqMask;
    buf[2] = dio1Mask >> 8;
    buf[3] = dio1Mask;
    buf[4] = dio2Mask >> 8;
    buf[5] = dio2Mask;
    buf[6] = dio3Mask >> 8;
    buf[7] = dio3Mask;
    uint8_t *data;
    err = lora_read_val(0x08, &buf, &data);
    if (err)
    {
        LOG_ERR("setDioIrqParams!, error: %d", err);
        return err;
    }
}

void sx126x_getIrqStatus(uint16_t *irqStatus)
{
    int err;
    uint8_t buf[3];
    uint8_t *data;
    err = lora_read_val(0x12, &buf, &data);
    if (err)
    {
        LOG_ERR("getIrqStatus!, error: %d", err);
        return err;
    }
    *irqStatus = (buf[1] << 8) | buf[2];
}

void sx126x_clearIrqStatus(uint16_t clearIrqParam)
{
    int err;
    uint8_t buf[2];
    buf[0] = clearIrqParam >> 8;
    buf[1] = clearIrqParam;
    uint8_t *data;
    err = lora_read_val(0x02, &buf, &data);
    if (err)
    {
        LOG_ERR("clearIrqStatus!, error: %d", err);
        return err;
    }
}

void sx126x_setDio2AsRfSwitchCtrl(uint8_t enable)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x9D, &enable, &data);
    if (err)
    {
        LOG_ERR("setDio2AsRfSwitchCtrl!, error: %d", err);
        return err;
    }
}

void sx126x_setDio3AsTcxoCtrl(uint8_t tcxoVoltage, uint32_t delay)
{
    int err;
    uint8_t buf[4];
    uint8_t *data;
    buf[0] = tcxoVoltage;
    buf[1] = delay >> 16;
    buf[2] = delay >> 8;
    buf[3] = delay;

    err = lora_read_val(0x97, &buf, &data);
    if (err)
    {
        LOG_ERR("setDio3AsTcxoCtrl!, error: %d", err);
        return err;
    }
}

void sx126x_setRfFrequency(uint32_t rfFreq)
{
    int err;
    uint8_t buf[4];
    uint8_t *data;
    buf[0] = rfFreq >> 24;
    buf[1] = rfFreq >> 16;
    buf[2] = rfFreq >> 8;
    buf[3] = rfFreq;
    err = lora_read_val(0x86, &buf, &data);
    if (err)
    {
        LOG_ERR("setRfFrequency!, error: %d", err);
        return err;
    }
    // LOG_INF("setRfFrequency data :%d", data);
}

void sx126x_setPacketType(uint8_t packetType)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0x8A, &packetType, &data);
    if (err)
    {
        LOG_ERR("setRfFrequency!, error: %d", err);
        return err;
    }
}

void sx126x_getPacketType(uint8_t *packetType)
{
    int err;
    uint8_t buf[2];
    uint8_t *data;
    err = lora_read_val(0x11, &buf, &data);
    if (err)
    {
        LOG_ERR("getPacketType!, error: %d", err);
        return err;
    }
    *packetType = buf[1];
    // LOG_INF("getPacketType data %d",data);
}

void sx126x_setTxParams(uint8_t power, uint8_t rampTime)
{
    int err;
    uint8_t *data;
    uint8_t buf[2];
    buf[0] = power;
    buf[1] = rampTime;
    err = lora_read_val(0x8E, &buf, &data);
    if (err)
    {
        LOG_ERR("setTxParams!, error: %d", err);
        return err;
    }
}

void sx126x_setModulationParamsLoRa(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
    int err;
    uint8_t *data;
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = ldro;
    err = lora_read_val(0x8B, &buf, &data);
    if (err)
    {
        LOG_ERR("getPacketType!, error: %d", err);
        return err;
    }
    // LOG_INF("setModulationParamsLoRa data %d",data);
}

void sx126x_setModulationParamsFSK(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{

    int err;
    uint8_t *data;
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = br >> 16;
    buf[1] = br >> 8;
    buf[2] = br;
    buf[3] = pulseShape;
    buf[4] = bandwidth;
    buf[5] = Fdev >> 16;
    buf[6] = Fdev >> 8;
    buf[7] = Fdev;
    err = lora_read_val(0x8B, &buf, &data);
    if (err)
    {
        LOG_ERR("getPacketType!, error: %d", err);
        return err;
    }
}

void sx126x_setPacketParamsLoRa(uint16_t preambleLength, uint8_t headerType, uint8_t payloadLength, uint8_t crcType, uint8_t invertIq)
{
    int err;
    uint8_t *data;
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength;
    buf[2] = headerType;
    buf[3] = payloadLength;
    buf[4] = crcType;
    buf[5] = invertIq;
    err = lora_read_val(0x8C, &buf, &data);
    if (err)
    {
        LOG_ERR("setPacketParamsLoRa!, error: %d", err);
        return err;
    }
    // LOG_INF("setPacketParamsLoRa data %d", data);
}

void sx126x_setPacketParamsFSK(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    int err;
    uint8_t *data;
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preambleLength >> 8;
    buf[1] = preambleLength;
    buf[2] = preambleDetector;
    buf[3] = syncWordLength;
    buf[4] = addrComp;
    buf[5] = packetType;
    buf[6] = payloadLength;
    buf[7] = crcType;
    buf[8] = whitening;
    err = lora_read_val(0x8C, &buf, &data);
    if (err)
    {
        LOG_ERR("setPacketParamsFSK!, error: %d", err);
        return err;
    }
}

void sx126x_setCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout)
{
    int err;
    uint8_t *data;
    uint8_t buf[7];
    buf[0] = cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = cadExitMode;
    buf[4] = cadTimeout >> 16;
    buf[5] = cadTimeout >> 8;
    buf[6] = cadTimeout;
    err = lora_read_val(0x88, &buf, &data);
    if (err)
    {
        LOG_ERR("setCadParams!, error: %d", err);
        return err;
    }
}

void sx126x_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
    int err;
    uint8_t *data;
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    err = lora_read_val(0x8F, &buf, &data);
    if (err)
    {
        LOG_ERR("setBufferBaseAddress!, error: %d", err);
        return err;
    }
    // LOG_INF("setBufferBaseAddress! data %d",data);
}

void sx126x_setLoRaSymbNumTimeout(uint8_t symbnum)
{
    int err;
    uint8_t *data;
    err = lora_read_val(0xA0, &symbnum, &data);
    if (err)
    {
        LOG_ERR("setLoRaSymbNumTimeout!, error: %d", err);
        return err;
    }
}

void sx126x_getRxBufferStatus(uint8_t *payloadLengthRx, uint8_t *rxStartBufferPointer)
{
    int err;
    uint8_t *data;
    uint8_t buf[3];
    // buf[1] = *payloadLengthRx;
    // buf[2] = *rxStartBufferPointer;
    err = lora_read_val(0x13, &buf, &data);
    if (err)
    {
        LOG_ERR("getRxBufferStatus!, error: %d", err);
        return err;
    }
    *payloadLengthRx = buf[1];
    *rxStartBufferPointer = buf[2];
}

void sx126x_getPacketStatus(uint8_t *rssiPkt, uint8_t *snrPkt, uint8_t *signalRssiPkt)
{
    int err;
    uint8_t *data;
    uint8_t buf[4];
    // buf[1] = *rssiPkt;
    // buf[2] = *snrPkt;
    // buf[3] = *signalRssiPkt;

    err = lora_read_val(0x14, &buf, &data);
    if (err)
    {
        LOG_ERR("getPacketStatus!, error: %d", err);
        return err;
    }
    *rssiPkt = buf[1];
    *snrPkt = buf[2];
    *signalRssiPkt = buf[3];
}

void sx126x_getRssiInst(uint8_t *rssiInst)
{
    int err;
    uint8_t *data;
    uint8_t buf[2];
    // buf[1] = *rssiInst;
    err = lora_read_val(0x15, &buf, &data);
    if (err)
    {
        LOG_ERR("getPacketStatus!, error: %d", err);
        return err;
    }
    *rssiInst = buf[1];
}

void sx126x_getStats(uint16_t *nbPktReceived, uint16_t *nbPktCrcError, uint16_t *nbPktHeaderErr)
{
    int err;
    uint8_t *data;
    uint8_t buf[7];

    err = lora_read_val(0x10, &buf, &data);
    if (err)
    {
        LOG_ERR("getStats!, error: %d", err);
        return err;
    }
    *nbPktReceived = (buf[1] << 8) | buf[2];
    *nbPktCrcError = (buf[3] << 8) | buf[4];
    *nbPktHeaderErr = (buf[5] << 8) | buf[6];
}

void sx126x_resetStats()
{
    int err;
    uint8_t *data;
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    err = lora_read_val(0x00, &buf, &data);
    if (err)
    {
        LOG_ERR("resetStats!, error: %d", err);
        return err;
    }
}

void sx126x_getDeviceErrors(uint16_t *opError)
{
    int err;
    uint8_t buf[3];
    uint8_t *data;
    err = lora_read_val(0x17, &buf, &data);
    if (err)
    {
        LOG_ERR("getDeviceErrors!, error: %d", err);
        return err;
    }
    *opError = buf[2];
}

void sx126x_clearDeviceErrors()
{
    int err;
    uint8_t *data;
    uint8_t buf[2] = {0, 0};
    err = lora_read_val(0x07, &buf, &data);
    if (err)
    {
        LOG_ERR("clearDeviceErrors!, error: %d", err);
        return err;
    }
}

void sx126x_setDio2RfSwitch(bool enable)
{
    if (enable)
    {
        sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_RF_SWITCH);
    }
    else
    {
        sx126x_setDio2AsRfSwitchCtrl(SX126X_DIO2_AS_IRQ);
    }
}

void sx126x_setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime)
{
    sx126x_setDio3AsTcxoCtrl(tcxoVoltage, delayTime);
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_calibrate(0xFF);
}

void sx126x_setXtalCap(uint8_t xtalA, uint8_t xtalB)
{
    sx126x_setStandby(SX126X_STANDBY_XOSC);
    uint8_t buf[2] = {xtalA, xtalB};
    sx126x_write_register(SX126X_REG_XTA_TRIM, buf);
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_calibrate(0xFF);
}

void sx126x_setRegulator(uint8_t regMode)
{
    sx126x_setRegulatorMode(regMode);
}

void sx126x_setCurrentProtection(uint8_t current)
{
    uint8_t currentmA = current * 2 / 5;
    sx126x_write_register(SX126X_REG_OCP_CONFIGURATION, &currentmA);
}

uint8_t sx126x_getModem()
{
    uint8_t modem;
    sx126x_getPacketType(&modem);
    return modem;
}

void sx126x_setModem(uint8_t modem)
{
    _modem = modem;
    sx126x_setStandby(SX126X_STANDBY_RC);
    sx126x_setPacketType(modem);
}

void sx126x_setFrequency(uint32_t frequency)
{
    uint8_t calFreq[2];
    if (frequency < 446000000)
    { // 430 - 440 Mhz
        calFreq[0] = SX126X_CAL_IMG_430;
        calFreq[1] = SX126X_CAL_IMG_440;
    }
    else if (frequency < 734000000)
    { // 470 - 510 Mhz
        calFreq[0] = SX126X_CAL_IMG_470;
        calFreq[1] = SX126X_CAL_IMG_510;
    }
    else if (frequency < 828000000)
    { // 779 - 787 Mhz
        calFreq[0] = SX126X_CAL_IMG_779;
        calFreq[1] = SX126X_CAL_IMG_787;
    }
    else if (frequency < 877000000)
    { // 863 - 870 Mhz
        calFreq[0] = SX126X_CAL_IMG_863;
        calFreq[1] = SX126X_CAL_IMG_870;
    }
    else if (frequency < 1100000000)
    { // 902 - 928 Mhz
        calFreq[0] = SX126X_CAL_IMG_902;
        calFreq[1] = SX126X_CAL_IMG_928;
    }
    // calculate frequency for setting configuration
    uint32_t rfFreq = ((uint64_t)frequency << SX126X_RF_FREQUENCY_SHIFT) / SX126X_RF_FREQUENCY_XTAL;

    // perform image calibration before set frequency
    sx126x_calibrateImage(calFreq[0], calFreq[1]);
    sx126x_setRfFrequency(rfFreq);
}

void sx126x_setTxPower(uint8_t txPower, uint8_t version)
{
    // maximum TX power is 22 dBm and 15 dBm for SX1261
    if (txPower > 22)
        txPower = 22;
    else if (txPower > 15 && version == SX126X_TX_POWER_SX1261)
        txPower = 15;

    uint8_t paDutyCycle = 0x00;
    uint8_t hpMax = 0x00;
    uint8_t deviceSel = version == SX126X_TX_POWER_SX1261 ? 0x01 : 0x00;
    uint8_t power = 0x0E;
    // set parameters for PA config and TX params configuration
    if (txPower == 22)
    {
        paDutyCycle = 0x04;
        hpMax = 0x07;
        power = 0x16;
    }
    else if (txPower >= 20)
    {
        paDutyCycle = 0x03;
        hpMax = 0x05;
        power = 0x16;
    }
    else if (txPower >= 17)
    {
        paDutyCycle = 0x02;
        hpMax = 0x03;
        power = 0x16;
    }
    else if (txPower >= 14 && version == SX126X_TX_POWER_SX1261)
    {
        paDutyCycle = 0x04;
        hpMax = 0x00;
        power = 0x0E;
    }
    else if (txPower >= 14 && version == SX126X_TX_POWER_SX1262)
    {
        paDutyCycle = 0x02;
        hpMax = 0x02;
        power = 0x16;
    }
    else if (txPower >= 14 && version == SX126X_TX_POWER_SX1268)
    {
        paDutyCycle = 0x04;
        hpMax = 0x06;
        power = 0x0F;
    }
    else if (txPower >= 10 && version == SX126X_TX_POWER_SX1261)
    {
        paDutyCycle = 0x01;
        hpMax = 0x00;
        power = 0x0D;
    }
    else if (txPower >= 10 && version == SX126X_TX_POWER_SX1268)
    {
        paDutyCycle = 0x00;
        hpMax = 0x03;
        power = 0x0F;
    }
    else
    {
        return;
    }

    // set power amplifier and TX power configuration
    sx126x_setPaConfig(paDutyCycle, hpMax, deviceSel, 0x01);
    sx126x_setTxParams(power, SX126X_PA_RAMP_800U);
}

void sx126x_setRxGain(uint8_t boost)
{
    // set power saving or boosted gain in register
    uint8_t gain = boost ? SX126X_BOOSTED_GAIN : SX126X_POWER_SAVING_GAIN;
    sx126x_write_register(SX126X_REG_RX_GAIN, &gain);
    if (boost)
    {
        // set certain register to retain configuration after wake from sleep mode
        uint8_t buf[3] = {0x01, 0x08, 0xAC};
        sx126x_write_register(0x029F, buf);
    }
}

void sx126x_setLoRaModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro)
{
    _sf = sf;
    _bw = bw;
    _cr = cr;
    _ldro = ldro;

    // valid spreading factor is between 5 and 12
    if (sf > 12)
        sf = 12;
    else if (sf < 5)
        sf = 5;
    // select bandwidth options
    if (bw < 9100)
        bw = SX126X_BW_7800; // 7.8 kHz
    else if (bw < 13000)
        bw = SX126X_BW_10400; // 10.4 kHz
    else if (bw < 18200)
        bw = SX126X_BW_15600; // 15.6 kHz
    else if (bw < 26000)
        bw = SX126X_BW_20800; // 20.8 kHz
    else if (bw < 36500)
        bw = SX126X_BW_31250; // 31.25 kHz
    else if (bw < 52100)
        bw = SX126X_BW_41700; // 41.7 kHz
    else if (bw < 93800)
        bw = SX126X_BW_62500; // 62.5 kHz
    else if (bw < 187500)
        bw = SX126X_BW_125000; // 125 kHz
    else if (bw < 375000)
        bw = SX126X_BW_250000; // 250 kHz
    else
        bw = SX126X_BW_500000; // 500 kHz
    // valid code rate denominator is between 4 and 8
    cr -= 4;
    if (cr > 4)
        cr = 0;

    sx126x_setModulationParamsLoRa(sf, (uint8_t)bw, cr, (uint8_t)ldro);
}

void sx126x_setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq)
{
    _headerType = headerType;
    _preambleLength = preambleLength;
    _payloadLength = payloadLength;
    _crcType = crcType;
    _invertIq = invertIq;

    // filter valid header type config
    if (headerType != SX126X_HEADER_IMPLICIT)
        headerType = SX126X_HEADER_EXPLICIT;

    sx126x_setPacketParamsLoRa(preambleLength, headerType, payloadLength, (uint8_t)crcType, (uint8_t)invertIq);
    sx126x_fixInvertedIq((uint8_t)invertIq);
}

void sx126x_setSpreadingFactor(uint8_t sf)
{
    sx126x_setLoRaModulation(sf, _bw, _cr, _ldro);
}

void sx126x_setBandwidth(uint32_t bw)
{
    sx126x_setLoRaModulation(_sf, bw, _cr, _ldro);
}

void sx126x_setCodeRate(uint8_t cr)
{
    sx126x_setLoRaModulation(_sf, _bw, cr, _ldro);
}

void sx126x_setLdroEnable(bool ldro)
{
    sx126x_setLoRaModulation(_sf, _bw, _cr, ldro);
}

void sx126x_setHeaderType(uint8_t headerType)
{
    sx126x_setLoRaPacket(headerType, _preambleLength, _payloadLength, _crcType, _invertIq);
}

void sx126x_setPreambleLength(uint16_t preambleLength)
{
    sx126x_setLoRaPacket(_headerType, preambleLength, _payloadLength, _crcType, _invertIq);
}

void sx126x_setPayloadLength(uint8_t payloadLength)
{
    sx126x_setLoRaPacket(_headerType, _preambleLength, payloadLength, _crcType, _invertIq);
}

void sx126x_setCrcEnable(bool crcType)
{
    sx126x_setLoRaPacket(_headerType, _preambleLength, _payloadLength, crcType, _invertIq);
}

void sx126x_setInvertIq(bool invertIq)
{
    sx126x_setLoRaPacket(_headerType, _preambleLength, _payloadLength, _crcType, invertIq);
}

void sx126x_setSyncWord(uint16_t syncWord)
{
    uint8_t buf[2];
    buf[0] = syncWord >> 8;
    buf[1] = syncWord & 0xFF;
    if (syncWord <= 0xFF)
    {
        LOG_INF("SYNC WORD FF");
        buf[0] = (syncWord & 0xF0) | 0x04;
        buf[1] = (syncWord << 4) | 0x04;
    }
    sx126x_write_register(SX126X_REG_LORA_SYNC_WORD_MSB, buf);
}

void sx126x_setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    sx126x_setModulationParamsFSK(br, pulseShape, bandwidth, Fdev);
}

void sx126x_setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    sx126x_setPacketParamsFSK(preambleLength, preambleDetector, syncWordLength, addrComp, packetType, payloadLength, crcType, whitening);
}

void sx126x_setFskSyncWord(uint8_t *sw)
{
    sx126x_write_register(SX126X_REG_FSK_SYNC_WORD_0, sw);
}

void sx126x_setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr)
{
    uint8_t buf[2] = {nodeAddr, broadcastAddr};
    sx126x_write_register(SX126X_REG_FSK_NODE_ADDRESS, buf);
}

void sx126x_setFskCrc(uint16_t crcInit, uint16_t crcPolynom)
{
    uint8_t buf[4];
    buf[0] = crcInit >> 8;
    buf[1] = crcInit & 0xFF;
    buf[2] = crcPolynom >> 8;
    buf[3] = crcPolynom & 0xFF;
    sx126x_write_register(SX126X_REG_FSK_CRC_INITIAL_MSB, &buf);
}

void sx126x_setFskWhitening(uint16_t whitening)
{
    uint8_t buf[2];
    buf[0] = whitening >> 8;
    buf[1] = whitening & 0xFF;
    sx126x_write_register(SX126X_REG_FSK_WHITENING_INITIAL_MSB, &buf);
}

void sx126x_beginPacket()
{
    // reset payload length and buffer index
    // LOG_INF("begin packet");
    _payloadTxRx = 0;
    sx126x_setBufferBaseAddress(_bufferIndex, _bufferIndex + 0xFF);

    sx126x_fixLoRaBw500(_bw);
}

bool sx126x_endPacket(uint32_t timeout)
{
    // skip to enter TX mode when previous TX operation incomplete
    // LOG_INF("end packet");
    if (getMode() == SX126X_STATUS_MODE_TX)
    {
        return false;
    }
    // clear previous interrupt and set TX done, and TX timeout as interrupt source
    // _irqSetup(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

    // set packet payload length
    sx126x_setLoRaPacket(_headerType, _preambleLength, _payloadTxRx, _crcType, _invertIq);

    // set status to TX wait
    _statusWait = SX126X_STATUS_TX_WAIT;
    _statusIrq = 0x0000;
    // calculate TX timeout config
    uint32_t txTimeout = timeout << 6;
    if (txTimeout > 0x00FFFFFF)
        txTimeout = SX126X_TX_SINGLE;

    // set device to transmit mode with configured timeout or single operation
    sx126x_setTx(txTimeout);
    _transmitTime = k_uptime_get_32();

    // // set operation status to wait and attach TX interrupt handler
    // if (_irq != -1) {
    //     attachInterrupt(_irqStatic, SX126x::_interruptTx, RISING);
    // }
    return true;
}

void sx126x_write_data(uint8_t *data, uint8_t length)
{
    // write multiple bytes of package to be transmitted
    // LOG_INF("write_data");
    sx126x_writeBuffer(_bufferIndex, data);
    _bufferIndex += length;
    _payloadTxRx += length;
}

void sx126x_write_char(char *data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    // LOG_INF("write_char");
    uint8_t *data_ = (uint8_t *)data;
    sx126x_writeBuffer(_bufferIndex, data_);
    _bufferIndex += length;
    _payloadTxRx += length;
}

void sx126x_write(uint8_t data)
{
    // write single byte of package to be transmitted
    sx126x_writeBuffer(_bufferIndex, &data);
    _bufferIndex++;
    _payloadTxRx++;
}

bool sx126x_request(uint32_t timeout)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode() == SX126X_STATUS_MODE_RX)
    {
        return false;
    }

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    // _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait or RX continuous wait
    _statusWait = SX126X_STATUS_RX_WAIT;
    _statusIrq = 0x0000;
    // calculate RX timeout config
    uint32_t rxTimeout = timeout << 6;
    if (rxTimeout > 0x00FFFFFF)
        rxTimeout = SX126X_RX_SINGLE;
    if (timeout == SX126X_RX_CONTINUOUS)
    {
        rxTimeout = SX126X_RX_CONTINUOUS;
        _statusWait = SX126X_STATUS_RX_CONTINUOUS;
    }

    // set device to receive mode with configured timeout, single, or continuous operation
    sx126x_setRx(rxTimeout);

    return true;
}

bool sx126x_listen(uint32_t rxPeriod, uint32_t sleepPeriod)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode() == SX126X_STATUS_MODE_RX)
        return false;

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    // _irqSetup(SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait
    _statusWait = SX126X_STATUS_RX_WAIT;
    _statusIrq = 0x0000;
    // calculate RX period and sleep period config
    rxPeriod = rxPeriod << 6;
    sleepPeriod = sleepPeriod << 6;
    if (rxPeriod > 0x00FFFFFF)
        rxPeriod = 0x00FFFFFF;
    if (sleepPeriod > 0x00FFFFFF)
        sleepPeriod = 0x00FFFFFF;

    // set device to receive mode with configured receive and sleep period
    sx126x_setRxDutyCycle(rxPeriod, sleepPeriod);

    return true;
}

uint8_t sx126x_available()
{
    // get size of package still available to read
    return _payloadTxRx;
}

uint8_t sx126x_read()
{
    // read single byte of received package
    uint8_t buf;
    sx126x_readBuffer(_bufferIndex, &buf);
    _bufferIndex++;
    if (_payloadTxRx > 0)
    {
        _payloadTxRx--;
    }
    return buf;
}

uint8_t sx126x_read_data(uint8_t *data, uint8_t length)
{
    // read multiple bytes of received package
    sx126x_readBuffer(_bufferIndex, data);
    // return smallest between read length and size of package available
    _bufferIndex += length;
    _payloadTxRx = _payloadTxRx > length ? _payloadTxRx - length : 0;
    return _payloadTxRx > length ? length : _payloadTxRx;
}

uint8_t sx126x_read_char(char *data, uint8_t length)
{
    // read multiple bytes of received package for char type
    uint8_t *data_ = (uint8_t *)data;
    sx126x_readBuffer(_bufferIndex, data_);
    _bufferIndex += length;
    _payloadTxRx = _payloadTxRx > length ? _payloadTxRx - length : 0;
    return _payloadTxRx > length ? length : _payloadTxRx;
}

void sx126x_purge(uint8_t length)
{
    // subtract or reset received payload length
    _payloadTxRx = (_payloadTxRx > length) && length ? _payloadTxRx - length : 0;
    _bufferIndex += length;
}

bool sx126x_wait(uint32_t timeout)
{
    // immediately return when currently not waiting transmit or receive process
    if (_statusIrq)
        return true;

    // wait transmit or receive process finish by checking interrupt status or IRQ status
    uint16_t irqStat = 0x0000;
    uint32_t t = k_uptime_get_32();
    while (irqStat == 0x0000 && _statusIrq == 0x0000)
    {
        // only check IRQ status register for non interrupt operation
        // if (_irq == -1)
        sx126x_getIrqStatus(&irqStat);
        // return when timeout reached
        if (k_uptime_get_32() - t > timeout && timeout != 0)
            return false;
        k_yield();
    }

    if (_statusIrq)
    {
        // immediately return when interrupt signal hit
        return true;
    }
    else if (_statusWait == SX126X_STATUS_TX_WAIT)
    {
        // for transmit, calculate transmit time and set back txen pin to low
        _transmitTime = k_uptime_get_32() - _transmitTime;
        // if (_txen != -1) digitalWrite(_txen, LOW);
    }
    else if (_statusWait == SX126X_STATUS_RX_WAIT)
    {
        // for receive, get received payload length and buffer index and set back rxen pin to low
        sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        // if (_rxen != -1)digitalWrite(_rxen, LOW);
        sx126x_fixRxTimeout();
    }
    else if (_statusWait == SX126X_STATUS_RX_CONTINUOUS)
    {
        // for receive continuous, get received payload length and buffer index and clear IRQ status
        sx126x_getRxBufferStatus(&_payloadTxRx, &_bufferIndex);
        sx126x_clearIrqStatus(0x03FF);
    }

    // store IRQ status
    _statusIrq = irqStat;
    return true;
}

uint8_t sx126x_status()
{
    // set back status IRQ for RX continuous operation
    uint16_t statusIrq = _statusIrq;
    if (_statusWait == SX126X_STATUS_RX_CONTINUOUS)
    {
        _statusIrq = 0x0000;
    }

    // get status for transmit and receive operation based on status IRQ
    if (statusIrq & SX126X_IRQ_TIMEOUT)
    {
        if (_statusWait == SX126X_STATUS_TX_WAIT)
            // return SX126X_STATUS_TX_TIMEOUT;
            return SX126X_STATUS_TX_DONE;
        else
            return SX126X_STATUS_RX_TIMEOUT;
    }
    else if (statusIrq && SX126X_IRQ_HEADER_ERR)
    {
        return SX126X_STATUS_HEADER_ERR;
    }
    else if (statusIrq && SX126X_IRQ_CRC_ERR)
    {
        return SX126X_STATUS_CRC_ERR;
    }
    else if (statusIrq && SX126X_IRQ_TX_DONE)
    {
        return SX126X_STATUS_TX_DONE;
    }
    else if (statusIrq && SX126X_IRQ_RX_DONE)
    {
        return SX126X_STATUS_RX_DONE;
    }

    // return TX or RX wait status
    return _statusWait;
    LOG_INF("status wait : %d", _statusWait);
}

uint32_t sx126x_transmitTime()
{
    // get transmit time in millisecond (ms)
    return _transmitTime;
}

float sx126x_dataRate()
{
    // get data rate last transmitted package in kbps
    return 1000.0 * _payloadTxRx / _transmitTime;
}

int16_t sx126x_packetRssi()
{
    // get relative signal strength index (RSSI) of last incoming package
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (rssiPkt / -2);
}

float sx126x_snr()
{
    // get signal to noise ratio (SNR) of last incoming package
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return ((int8_t)snrPkt / 4.0);
}

int16_t sx126x_signalRssi()
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(&rssiPkt, &snrPkt, &signalRssiPkt);
    return (signalRssiPkt / -2);
}

int16_t sx126x_rssiInst()
{
    uint8_t rssiInst;
    sx126x_getRssiInst(&rssiInst);
    return (rssiInst / -2);
}

uint16_t sx126x_getError()
{
    uint16_t error;
    sx126x_getDeviceErrors(&error);
    sx126x_clearDeviceErrors();
    return error;
}

void sx127x_write_bits(uint8_t address, uint8_t data, uint8_t position, uint8_t length)
{
    uint8_t read = sx126x_read_register(address);
    uint8_t mask = (0xFF >> (8 - length)) << position;
    uint8_t write = (data << position) | (read & ~mask);
    sx126x_write_register(address, write);
}

void sx127x_sleep()
{
    sx126x_write_register(SX127X_REG_OP_MODE, (_modem | SX127X_MODE_SLEEP));
}

void sx127x_standby()
{
    sx126x_write_register(SX127X_REG_OP_MODE, (_modem | SX127X_MODE_STDBY));
}

void sx127x_wakeup()
{
    sx126x_write_register(SX127X_REG_OP_MODE, (_modem | SX127X_MODE_STDBY));
}

void sx127x_currentprotection(uint8_t current)
{
    uint8_t ocpTrim = 27;
    // Formula availabe in datasheet
    if (current <= 120)
    {
        ocpTrim = (current - 45) / 5;
    }
    else if (current <= 240)
    {
        ocpTrim = (current + 30) / 10;
    }

    sx126x_write_register(SX127X_REG_OCP, (0x20 | ocpTrim));
}

void sx127x_setOscillator(uint8_t option)
{
    uint8_t cfg;

    if (option == SX127X_OSC_TCXO)
    {
        cfg = SX127X_OSC_TCXO;
    }
    else
    {
        cfg = SX127X_OSC_CRYSTAL;
    }
    sx126x_write_register(SX127X_REG_TCXO, cfg);
}

void sx127x_setModem(uint8_t modem)
{
    if (modem == SX127X_LORA_MODEM)
        _modem = SX127X_LONG_RANGE_MODE;
    else if (modem == SX127X_FSK_MODEM)
        _modem = SX127X_MODULATION_FSK;
    else
        _modem = SX127X_MODULATION_OOK;
    sx127x_sleep();
    sx126x_write_register(SX127X_REG_OP_MODE, (_modem | SX127X_MODE_STDBY));
}

void sx127x_setFrequency(uint32_t frequency)
{
    _frequency = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    sx126x_write_register(SX127X_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx126x_write_register(SX127X_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx126x_write_register(SX127X_REG_FRF_LSB, (uint8_t)frf);
}

void sx127x_setTXPower(uint8_t power, uint8_t paPin)
{
    if (power > 20)
        power = 20;
    else if (power > 14 && paPin == SX127X_TX_POWER_RFO)
        power = 14;

    uint8_t paConfig, outputPower;
    if (paPin == SX127X_TX_POWER_RFO)
    {
        // power = Pmax - (15 - outputPower)
        if (power == 14)
        {
            // max power (Pmax) 14.4 dBm
            paConfig = 0x60;
            outputPower = power + 1;
        }
        else
        {
            // max power (Pmax) 13.2 dBm
            paConfig = 0x40;
            outputPower = power + 2;
        }
    }
    else
    {
        paConfig = 0xC0;
        uint8_t paDac = 0x04;
        // txPower = 17 - (15 - outputPower)
        if (power > 17)
        {
            outputPower = 15;
            paDac = 0x07;
            sx127x_currentprotection(100); // max current 100 mA
        }
        else
        {
            if (power < 2)
                power = 2;
            outputPower = power - 2;
            sx127x_currentprotection(140); // max current 140 mA
        }
        // enable or disable +20 dBm option on PA_BOOST pin
        sx126x_write_register(SX127X_REG_PA_DAC, paDac);
    }
    // set PA config
    sx126x_write_register(SX127X_REG_PA_CONFIG, (paConfig | outputPower));
}

void sx127x_setRXGain(uint8_t boost, uint8_t level)
{
    // valid RX gain level 0 - 6 (0 -> AGC on)
    if (level > 6)
        level = 6;
    // boost LNA and automatic gain controller config
    uint8_t LnaBoostHf;
    if (boost == SX127X_RX_GAIN_BOOSTED)
    {
        LnaBoostHf = 0x03;
    }
    else
    {
        LnaBoostHf = 0x00;
    }
    uint8_t AgcOn;
    if (level == SX127X_RX_GAIN_AUTO)
    {
        AgcOn = 0x01;
    }
    else
    {
        AgcOn = 0x00;
    }
    sx126x_write_register(SX127X_REG_LNA, (LnaBoostHf | (level << 5)));
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_3, AgcOn, 2, 1);
}

void sx127x_setLoraModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro)
{
    sx127x_setSpreadingFactor(sf);
    sx127x_setBandwidth(bw);
    sx127x_setCodeRate(cr);
    sx127x_setLdro(ldro);
}

void sx127x_setLoraPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq)
{
    sx127x_setHeaderType(headerType);
    sx127x_setPreambleLength(preambleLength);
    sx127x_setPayloadLength(payloadLength);
    sx127x_setCrcType(crcType);
    // sx127x_setInvertIq(invertIq);
}

void sx127x_setSpreadingFactor(uint8_t sf)
{
    _sf = sf;
    // valid spreading factor is 6 - 12
    if (sf < 6)
        sf = 6;
    else if (sf > 12)
        sf = 12;
    // set appropriate signal detection optimize and threshold
    uint8_t optimize;
    if (sf == 6)
    {
        optimize = 0x05;
    }
    else
    {
        optimize = 0x03;
    }
    uint8_t threshold;
    if (sf == 6)
    {
        threshold = 0x0C;
    }
    else
    {
        threshold = 0x0A;
    }
    sx126x_write_register(SX127X_REG_DETECTION_OPTIMIZE, optimize);
    sx126x_write_register(SX127X_REG_DETECTION_THRESHOLD, threshold);
    // set spreading factor config
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_2, sf, 4, 4);
}

void sx127x_setBandwidth(uint32_t bw)
{
    _bw = bw;
    uint8_t bwCfg;
    if (bw < 9100)
        bwCfg = 0; // 7.8 kHz
    else if (bw < 13000)
        bwCfg = 1; // 10.4 kHz
    else if (bw < 18200)
        bwCfg = 2; // 15.6 kHz
    else if (bw < 26000)
        bwCfg = 3; // 20.8 kHz
    else if (bw < 36500)
        bwCfg = 4; // 31.25 kHz
    else if (bw < 52100)
        bwCfg = 5; // 41.7 kHz
    else if (bw < 93800)
        bwCfg = 6; // 62.5 kHz
    else if (bw < 187500)
        bwCfg = 7; // 125 kHz
    else if (bw < 375000)
        bwCfg = 8; // 250 kHz
    else
        bwCfg = 9; // 500 kHz
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_1, bwCfg, 4, 4);
}

void sx127x_setCodeRate(uint8_t cr)
{
    // valid code rate denominator is 5 - 8
    if (cr < 5)
        cr = 4;
    else if (cr > 8)
        cr = 8;
    uint8_t crCfg = cr - 4;
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_1, crCfg, 1, 3);
}

void sx127x_setLdro(bool ldro)
{
    uint8_t ldroCfg;
    if (ldro)
    {
        ldroCfg = 0x01;
    }
    else
    {
        ldroCfg = 0x00;
    }
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_3, ldroCfg, 3, 1);
}

void sx127x_setHeaderType(uint8_t headerType)
{
    _headerType = headerType;
    uint8_t headerTypeCfg;
    if (headerType == SX127X_HEADER_IMPLICIT)
    {
        headerTypeCfg = SX127X_HEADER_IMPLICIT;
    }
    else
    {
        headerTypeCfg = SX127X_HEADER_EXPLICIT;
    }
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_1, headerTypeCfg, 0, 1);
}

void sx127x_setPreambleLength(uint16_t preambleLength)
{
    sx126x_write_register(SX127X_REG_PREAMBLE_MSB, (uint8_t)(preambleLength >> 8));
    sx126x_write_register(SX127X_REG_PREAMBLE_LSB, (uint8_t)preambleLength);
}

void sx127x_setPayloadLength(uint16_t payloadLength)
{
    _payloadLength = payloadLength;
    sx126x_write_register(SX127X_REG_PAYLOAD_LENGTH, payloadLength);
}

void sx127x_setCrcType(bool crcType)
{
    uint8_t crcTypeCfg = crcType ? 0x01 : 0x00;
    if (crcType)
    {
        crcTypeCfg = 0x01;
    }
    else
    {
        crcTypeCfg = 0x00;
    }
    sx127x_write_bits(SX127X_REG_MODEM_CONFIG_2, crcTypeCfg, 2, 1);
}

void sx127x_setInvertIq(bool invertIq)
{
    uint8_t invertIqCfg1;
    uint8_t invertIqCfg2;
    if (invertIq)
    {
        invertIqCfg1 = 0x01;
        invertIqCfg2 = 0x19;
    }
    else
    {
        invertIqCfg1 = 0x00;
        invertIqCfg2 = 0x1D;
    }
    sx127x_write_bits(SX127X_REG_INVERTIQ, invertIqCfg1, 0, 1);
    sx127x_write_bits(SX127X_REG_INVERTIQ, invertIqCfg1, 6, 1);
    sx126x_write_register(SX127X_REG_INVERTIQ2, invertIqCfg2);
}

void sx127x_setSyncWord(uint16_t syncWord)
{
    uint8_t sw = syncWord;
    // keep compatibility between 1 and 2 bytes synchronize word
    if (syncWord > 0xFF)
    {
        sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F);
    }
    sx126x_write_register(SX127X_REG_SYNC_WORD, sw);
}

void sx127x_beginPacket()
{
    LOG_INF("begin packet");
    uint8_t data = sx126x_read_register(SX127X_REG_FIFO_ADDR_PTR);
    sx126x_write_register(SX127X_REG_FIFO_TX_BASE_ADDR, data);
    _payloadTxRx = 0;
}

bool sx127x_endPacket(uint32_t timeout)
{
    // skip to enter TX mode when previous TX operation incomplete
    LOG_INF("endPacket");
    uint8_t status = (sx126x_read_register(SX127X_REG_OP_MODE) & 0x07);
    if (status == SX127X_MODE_TX)
        return false;
    // clear IRQ flag from last TX or RX operation
    sx126x_write_register(SX127X_REG_IRQ_FLAGS, 0xFF);
    // set packet payload length
    sx126x_write_register(SX127X_REG_PAYLOAD_LENGTH, _payloadTxRx);

    // set status to TX wait
    _statusWait = SX127X_STATUS_TX_WAIT;
    _statusIrq = 0x00;

    sx126x_write_register(SX127X_REG_OP_MODE, (_modem | SX127X_MODE_TX));
    _transmitTime = k_uptime_get_32();

    return true;
}

void sx127x_write_char(char *data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    LOG_INF("write_char");
    uint8_t *data_ = (uint8_t *)data;
    sx127x_write_data(data_, length);
}

void sx127x_write_data(uint8_t *data, uint8_t length)
{
    // write multiple bytes of package to be transmitted in FIFO buffer
    LOG_INF("write_data");

    for (uint8_t i = 0; i < length; i++)
    {
        sx126x_write_register(SX127X_REG_FIFO, data[i]);
    }
    // increasing payload length
    _payloadTxRx += length;
}

bool sx127x_request(uint32_t timeout)
{
    uint8_t rxMode = (sx126x_read_register(SX127X_REG_OP_MODE) & 0x07);
    if (rxMode == SX127X_MODE_RX_SINGLE || rxMode == SX127X_MODE_RX_CONTINUOUS)
        return false;

    // clear IRQ flag from last TX or RX operation
    sx126x_write_register(SX127X_REG_IRQ_FLAGS, 0xFF);
    // set status to RX wait
    _statusWait = SX127X_STATUS_RX_WAIT;
    _statusIrq = 0x00;
    // select RX mode to RX continuous mode for RX single and continuos operation
    rxMode = SX127X_MODE_RX_CONTINUOUS;
    if (timeout == SX127X_RX_CONTINUOUS)
    {
        _statusWait = SX127X_STATUS_RX_CONTINUOUS;
    }
    else if (timeout > 0)
    {
        // Select RX mode to single mode for RX operation with timeout
        rxMode = SX127X_MODE_RX_SINGLE;
        // calculate and set symbol timeout
        uint16_t symbTimeout = (timeout * _bw / 1000) >> _sf; // devided by 1000, ms to s
        symbTimeout = symbTimeout < 0x03FF ? symbTimeout : 0x03FF;
        sx127x_write_bits(SX127X_REG_MODEM_CONFIG_2, (symbTimeout >> 8) & 0x03, 0, 2);
        sx126x_write_register(SX127X_REG_SYMB_TIMEOUT, symbTimeout);
    }

    // set device to receive mode
    sx126x_write_register(SX127X_REG_OP_MODE, _modem | rxMode);
    return true;
}

uint8_t sx127x_available()
{
    // get size of package still available to read
    return _payloadTxRx;
}

uint8_t sx127x_read_char(char *data, uint8_t length)
{
    // read multiple bytes of received package for char type
    uint8_t *data_ = (uint8_t *)data;
    return sx127x_read_data(data_, length);
}

uint8_t sx127x_read_data(uint8_t *data, uint8_t length)
{
    // calculate actual read length and remaining payload length
    if (_payloadTxRx > length)
    {
        _payloadTxRx -= length;
    }
    else
    {
        length = _payloadTxRx;
        _payloadTxRx = 0;
    }
    // read multiple bytes of received package in FIFO buffer
    for (uint8_t i = 0; i < length; i++)
    {
        data[i] = sx126x_read_register(SX127X_REG_FIFO);
    }
    return length;
}

void sx127x_purge(uint8_t length)
{
    // subtract or reset received payload length
    _payloadTxRx = (_payloadTxRx > length) && length ? _payloadTxRx - length : 0;
}

bool sx127x_wait(uint32_t timeout)
{
    // immediately return when currently not waiting transmit or receive process
    if (_statusIrq)
        return true;
    // wait transmit or receive process finish by checking interrupt status or IRQ status
    uint8_t irqFlag = 0x00;
    uint8_t irqFlagMask = _statusWait == SX127X_STATUS_TX_WAIT
                              ? SX127X_IRQ_TX_DONE
                              : SX127X_IRQ_RX_DONE | SX127X_IRQ_RX_TIMEOUT | SX127X_IRQ_CRC_ERR;
    uint32_t t = k_uptime_get_32();

    while (!(irqFlag & irqFlagMask) && _statusIrq == 0x00)
    {
        irqFlag = sx126x_read_register(SX127X_REG_IRQ_FLAGS);
        // return when timeout reached
        if (k_uptime_get_32() - t > timeout && timeout != 0)
            return false;
        k_yield();
    }
    if (_statusIrq)
    {
        // immediately return when interrupt signal hit
        return true;
    }
    else if (_statusWait == SX127X_STATUS_TX_WAIT)
    {
        // calculate transmit time and set back txen pin to low
        _transmitTime = k_uptime_get_32() - _transmitTime;
    }
    else if (_statusWait == SX127X_STATUS_RX_WAIT)
    {
        // terminate receive mode by setting mode to standby
        sx127x_standby();
        // set pointer to RX buffer base address and get packet payload length
        sx126x_write_register(SX127X_REG_FIFO_ADDR_PTR, sx126x_read_register(SX127X_REG_FIFO_RX_CURRENT_ADDR));
        _payloadTxRx = sx126x_read_register(SX127X_REG_RX_NB_BYTES);
    }
    else if (_statusWait == SX127X_STATUS_RX_CONTINUOUS)
    {
        // set pointer to RX buffer base address and get packet payload length
        sx126x_write_register(SX127X_REG_FIFO_ADDR_PTR, sx126x_read_register(SX127X_REG_FIFO_RX_CURRENT_ADDR));
        _payloadTxRx = sx126x_read_register(SX127X_REG_RX_NB_BYTES);
        // clear IRQ flag
        sx126x_write_register(SX127X_REG_IRQ_FLAGS, 0xFF);
    }

    // store IRQ status
    _statusIrq = irqFlag;
    return true;
}

uint8_t sx127x_status()
{
    // set back status IRQ for RX continuous operation
    uint8_t statusIrq = _statusIrq;
    if (_statusWait == SX127X_STATUS_RX_CONTINUOUS)
    {
        _statusIrq = 0x00;
    }

    // get status for transmit and receive operation based on status IRQ
    if (statusIrq & SX127X_IRQ_RX_TIMEOUT)
        return SX127X_STATUS_RX_TIMEOUT;
    else if (statusIrq & SX127X_IRQ_CRC_ERR)
        return SX127X_STATUS_CRC_ERR;
    else if (statusIrq & SX127X_IRQ_TX_DONE)
        return SX127X_STATUS_TX_DONE;
    else if (statusIrq & SX127X_IRQ_RX_DONE)
        return SX127X_STATUS_RX_DONE;

    // return TX or RX wait status
    return _statusWait;
}

uint32_t sx127x_transmitTime()
{
    // get transmit time in millisecond (ms)
    return _transmitTime;
}

float sx127x_dataRate()
{
    // get data rate last transmitted package in kbps
    return 1000.0 * _payloadTxRx / _transmitTime;
}

int16_t sx127x_PacketRssi()
{
    int16_t offset;
    if (_frequency < SX127X_BAND_THRESHOLD)
    {
        offset = SX127X_RSSI_OFFSET_LF;
    }
    else
    {
        offset = SX127X_RSSI_OFFSET_HF;
    }
    if (sx126x_read_register(SX127X_REG_VERSION) == 0x22)
    {
        offset = SX127X_RSSI_OFFSET;
    }

    return (int16_t)(sx126x_read_register(SX127X_REG_PKT_RSSI_VALUE) - offset);
}

int16_t sx127x_Rssi()
{
    int16_t offset;
    if (_frequency < SX127X_BAND_THRESHOLD)
    {
        offset = SX127X_RSSI_OFFSET_LF;
    }
    else
    {
        offset = SX127X_RSSI_OFFSET_HF;
    }
    if (sx126x_read_register(SX127X_REG_VERSION) == 0x22)
    {
        offset = SX127X_RSSI_OFFSET;
    }

    return (int16_t)(sx126x_read_register(SX127X_REG_RSSI_VALUE) - offset);
}

float sx127x_snr()
{
    // get signal to noise ratio (SNR) of last incoming package
    return (int8_t)(sx126x_read_register(SX127X_REG_PKT_SNR_VALUE) / 4.0);
}

void sx127x_set_DIO0()
{
    sx127x_write_bits(SX127X_REG_DIO_MAPPING_1, 0x01, 6, 2);
}