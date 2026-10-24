/* Initialization adapted from the supplied Pololu/ST library.
 * See LICENSE_VL53L0X.txt for retained copyright and license notices. */
#include "vl53l0x.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "delay.h"

#define SCL_PIN GPIO_Pin_8
#define SDA_PIN GPIO_Pin_9
#define TRY(call) do { VL53L0X_Status s_ = (call); if (s_ != VL53L0X_OK) return s_; } while (0)

static uint8_t initialized;
static uint8_t continuous;
static uint8_t stop_variable;
/* Open drain output: setting a bit releases the line, including SDA reads. */
static void scl(uint8_t high)
{
    if (high) GPIO_SetBits(GPIOA, SCL_PIN);
    else GPIO_ResetBits(GPIOA, SCL_PIN);
    delay_us(5);
}

static void sda(uint8_t high)
{
    if (high) GPIO_SetBits(GPIOC, SDA_PIN);
    else GPIO_ResetBits(GPIOC, SDA_PIN);
    delay_us(5);
}

static VL53L0X_Status clock_high(void)
{
    uint16_t n;
    scl(1);
    for (n = 0; n < 200; ++n) {
        if (GPIO_ReadInputDataBit(GPIOA, SCL_PIN)) return VL53L0X_OK;
        delay_us(5);
    }
    return VL53L0X_ERROR_TIMEOUT;
}

static VL53L0X_Status bus_stop(void)
{
    VL53L0X_Status status;
    scl(0); sda(0);
    status = clock_high();
    sda(1);
    return status;
}

static VL53L0X_Status bus_start(void)
{
    sda(1);
    TRY(clock_high());
    if (!GPIO_ReadInputDataBit(GPIOC, SDA_PIN)) return VL53L0X_ERROR_I2C;
    sda(0); scl(0);
    return VL53L0X_OK;
}

static VL53L0X_Status send_byte(uint8_t value)
{
    uint8_t i, ack;
    for (i = 0; i < 8; ++i) {
        sda((value & 0x80U) != 0);
        TRY(clock_high());
        scl(0); value <<= 1;
    }
    sda(1);
    TRY(clock_high());
    ack = !GPIO_ReadInputDataBit(GPIOC, SDA_PIN);
    scl(0);
    return ack ? VL53L0X_OK : VL53L0X_ERROR_I2C;
}

static VL53L0X_Status receive_byte(uint8_t *value, uint8_t ack)
{
    uint8_t i, data = 0;
    sda(1);
    for (i = 0; i < 8; ++i) {
        TRY(clock_high());
        data = (uint8_t)((data << 1) | GPIO_ReadInputDataBit(GPIOC, SDA_PIN));
        scl(0);
    }
    sda(!ack);
    TRY(clock_high());
    scl(0); sda(1);
    *value = data;
    return VL53L0X_OK;
}

/* All transactions generate STOP even on NACK or a clock timeout. */
static VL53L0X_Status transfer(uint8_t reg, uint8_t *data, uint8_t count, uint8_t read)
{
    VL53L0X_Status status, stopped;
    uint8_t i;
    status = bus_start();
    if (status == VL53L0X_OK) status = send_byte(VL53L0X_I2C_ADDR << 1);
    if (status == VL53L0X_OK) status = send_byte(reg);
    if (read && status == VL53L0X_OK) {
        status = bus_start();
        if (status == VL53L0X_OK) status = send_byte((VL53L0X_I2C_ADDR << 1) | 1);
    }
    for (i = 0; i < count && status == VL53L0X_OK; ++i) {
        if (read) status = receive_byte(&data[i], i + 1 < count);
        else status = send_byte(data[i]);
    }
    stopped = bus_stop();
    if (status == VL53L0X_OK) status = stopped;
    if (status != VL53L0X_OK) {
        /* A failed page switch leaves register state uncertain. Reinitialize. */
        initialized = 0;
        continuous = 0;
    }
    return status;
}

static VL53L0X_Status wr(uint8_t reg, uint8_t value)
{
    return transfer(reg, &value, 1, 0);
}

static VL53L0X_Status rd(uint8_t reg, uint8_t *value)
{
    return transfer(reg, value, 1, 1);
}

static VL53L0X_Status modify(uint8_t reg, uint8_t clear, uint8_t set)
{
    uint8_t value;
    TRY(rd(reg, &value));
    return wr(reg, (uint8_t)((value & ~clear) | set));
}

/* Bounded polling also works before the FreeRTOS scheduler starts. */
static VL53L0X_Status wait_reg(uint8_t reg, uint8_t mask, uint8_t nonzero)
{
    uint16_t i;
    uint8_t value;
    for (i = 0; i < VL53L0X_TIMEOUT_MS; ++i) {
        TRY(rd(reg, &value));
        if (((value & mask) != 0) == (nonzero != 0)) return VL53L0X_OK;
        delay_ms(1);
    }
    return VL53L0X_ERROR_TIMEOUT;
}

static VL53L0X_Status gpio_init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_SetBits(GPIOA, SCL_PIN);
    GPIO_SetBits(GPIOC, SDA_PIN);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Pin = SCL_PIN;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = SDA_PIN;
    GPIO_Init(GPIOC, &gpio);
    /* Release a slave left mid-byte after an MCU reset. */
    sda(1);
    for (i = 0; i < 9 && !GPIO_ReadInputDataBit(GPIOC, SDA_PIN); ++i) {
        scl(0); TRY(clock_high());
    }
    TRY(bus_stop());
    return GPIO_ReadInputDataBit(GPIOC, SDA_PIN) ? VL53L0X_OK : VL53L0X_ERROR_I2C;
}

static VL53L0X_Status read_nvm_word(uint8_t index, uint8_t data[4])
{
    TRY(wr(0x94, index));
    TRY(wr(0x83, 0));
    TRY(wait_reg(0x83, 0xFF, 1));
    TRY(wr(0x83, 1));
    return transfer(0x90, data, 4, 1);
}

static VL53L0X_Status setup_spads(void)
{
    uint8_t info, map[6], check[6], word[4], i, enabled = 0, first;
    TRY(wr(0x80, 1)); TRY(wr(0xFF, 1)); TRY(wr(0x00, 0));
    TRY(wr(0xFF, 6)); TRY(modify(0x83, 0, 4));
    TRY(wr(0xFF, 7)); TRY(wr(0x81, 1)); TRY(wr(0x80, 1));
    /* Read factory count/type and good-SPAD map directly from NVM.
     * Page-zero 0xB0 contains current enables, not an immutable factory map.
     * ST VL53L0X_get_info_from_device(option=1) uses NVM indices 0x24/0x25. */
    TRY(read_nvm_word(0x6B, word));
    info = word[2];
    TRY(read_nvm_word(0x24, word));
    for (i = 0; i < 4; ++i) map[i] = word[i];
    TRY(read_nvm_word(0x25, word));
    map[4] = word[0]; map[5] = word[1];
    TRY(wr(0x81, 0)); TRY(wr(0xFF, 6));
    /* Correct the misplaced parenthesis in the supplied Arduino example. */
    TRY(modify(0x83, 4, 0));
    TRY(wr(0xFF, 1)); TRY(wr(0x00, 1));
    TRY(wr(0xFF, 0)); TRY(wr(0x80, 0));
    TRY(wr(0xFF, 1)); TRY(wr(0x4F, 0)); TRY(wr(0x4E, 0x2C));
    TRY(wr(0xFF, 0)); TRY(wr(0xB6, 0xB4));
    first = (info & 0x80U) ? 12 : 0;
    for (i = 0; i < 48; ++i) {
        if (i < first || enabled == (info & 0x7FU)) map[i / 8] &= (uint8_t)~(1U << (i % 8));
        else if ((map[i / 8] >> (i % 8)) & 1U) ++enabled;
    }
    if (enabled == 0 || enabled != (info & 0x7FU)) return VL53L0X_ERROR_RANGE;
    TRY(transfer(0xB0, map, 6, 0));
    TRY(transfer(0xB0, check, 6, 1));
    for (i = 0; i < 6; ++i)
        if (check[i] != map[i]) return VL53L0X_ERROR_CONFIG;
    return VL53L0X_OK;
}

/* VL53L0X load_tuning_settings; populated from the supplied library. */
static const uint8_t tuning[][2] = {
    {0xFF, 0x01},
    {0x00, 0x00},
    {0xFF, 0x00},
    {0x09, 0x00},
    {0x10, 0x00},
    {0x11, 0x00},
    {0x24, 0x01},
    {0x25, 0xFF},
    {0x75, 0x00},
    {0xFF, 0x01},
    {0x4E, 0x2C},
    {0x48, 0x00},
    {0x30, 0x20},
    {0xFF, 0x00},
    {0x30, 0x09},
    {0x54, 0x00},
    {0x31, 0x04},
    {0x32, 0x03},
    {0x40, 0x83},
    {0x46, 0x25},
    {0x60, 0x00},
    {0x27, 0x00},
    {0x50, 0x06},
    {0x51, 0x00},
    {0x52, 0x96},
    {0x56, 0x08},
    {0x57, 0x30},
    {0x61, 0x00},
    {0x62, 0x00},
    {0x64, 0x00},
    {0x65, 0x00},
    {0x66, 0xA0},
    {0xFF, 0x01},
    {0x22, 0x32},
    {0x47, 0x14},
    {0x49, 0xFF},
    {0x4A, 0x00},
    {0xFF, 0x00},
    {0x7A, 0x0A},
    {0x7B, 0x00},
    {0x78, 0x21},
    {0xFF, 0x01},
    {0x23, 0x34},
    {0x42, 0x00},
    {0x44, 0xFF},
    {0x45, 0x26},
    {0x46, 0x05},
    {0x40, 0x40},
    {0x0E, 0x06},
    {0x20, 0x1A},
    {0x43, 0x40},
    {0xFF, 0x00},
    {0x34, 0x03},
    {0x35, 0x44},
    {0xFF, 0x01},
    {0x31, 0x04},
    {0x4B, 0x09},
    {0x4C, 0x05},
    {0x4D, 0x04},
    {0xFF, 0x00},
    {0x44, 0x00},
    {0x45, 0x20},
    {0x47, 0x08},
    {0x48, 0x28},
    {0x67, 0x00},
    {0x70, 0x04},
    {0x71, 0x01},
    {0x72, 0xFE},
    {0x76, 0x00},
    {0x77, 0x00},
    {0xFF, 0x01},
    {0x0D, 0x01},
    {0xFF, 0x00},
    {0x80, 0x01},
    {0x01, 0xF8},
    {0xFF, 0x01},
    {0x8E, 0x01},
    {0x00, 0x01},
    {0xFF, 0x00},
    {0x80, 0x00},
};

static uint32_t macro_ns(uint8_t period)
{
    return (2304UL * period * 1655UL + 500UL) / 1000UL;
}

static VL53L0X_Status set_default_budget(void)
{
    uint8_t pre_period, final_period, msrc, bytes[2];
    uint32_t pre_clks, pre_ns, final_ns, used_us, final_clks, mantissa;
    uint16_t encoded, exponent = 0;
    TRY(rd(0x50, &pre_period)); TRY(rd(0x70, &final_period));
    TRY(rd(0x46, &msrc)); TRY(transfer(0x51, bytes, 2, 1));
    /* Reject corrupt timeout data rather than shifting by >=32. */
    if (bytes[0] > 15) return VL53L0X_ERROR_RANGE;
    pre_clks = ((uint32_t)bytes[1] << bytes[0]) + 1;
    pre_ns = macro_ns((uint8_t)((pre_period + 1) * 2));
    final_ns = macro_ns((uint8_t)((final_period + 1) * 2));
    if (final_ns == 0) return VL53L0X_ERROR_RANGE;
    /* Sequence 0xE8: DSS + pre-range + final-range. */
    used_us = 1320 + 960 + 2 * ((((uint32_t)msrc + 1) * pre_ns + pre_ns / 2) / 1000 + 690)
              + (pre_clks * pre_ns + pre_ns / 2) / 1000 + 660 + 550;
    if (used_us >= VL53L0X_TIMING_BUDGET_US) return VL53L0X_ERROR_RANGE;
    final_clks = ((VL53L0X_TIMING_BUDGET_US - used_us) * 1000 + final_ns / 2) / final_ns + pre_clks;
    mantissa = final_clks - 1;
    while (mantissa > 255) { mantissa >>= 1; ++exponent; }
    encoded = (uint16_t)((exponent << 8) | mantissa);
    bytes[0] = (uint8_t)(encoded >> 8); bytes[1] = (uint8_t)encoded;
    return transfer(0x71, bytes, 2, 0);
}

static VL53L0X_Status calibrate(uint8_t sequence, uint8_t vhv)
{
    TRY(wr(0x01, sequence)); TRY(wr(0x00, (uint8_t)(1 | vhv)));
    TRY(wait_reg(0x13, 7, 1));
    TRY(wr(0x0B, 1));
    return wr(0x00, 0);
}

static VL53L0X_Status initialize_sensor(void)
{
    uint8_t id, i, rate[2] = {0, 32}, check[2];

    TRY(wr(0xFF, 0)); TRY(rd(0xC0, &id));
    if (id != 0xEE) return VL53L0X_ERROR_ID;
    TRY(modify(0x89, 0, 1)); /* 2V8 I/O mode. */
    TRY(wr(0x88, 0)); TRY(wr(0x80, 1)); TRY(wr(0xFF, 1));
    TRY(wr(0x00, 0)); TRY(rd(0x91, &stop_variable));
    TRY(wr(0x00, 1)); TRY(wr(0xFF, 0)); TRY(wr(0x80, 0));
    TRY(modify(0x60, 0, 0x12));
    TRY(transfer(0x44, rate, 2, 0)); /* 0.25 MCPS, Q9.7. */
    TRY(wr(0x01, 0xFF));

    TRY(setup_spads());

    for (i = 0; i < sizeof(tuning) / sizeof(tuning[0]); ++i) TRY(wr(tuning[i][0], tuning[i][1]));
    TRY(wr(0x0A, 4)); TRY(modify(0x84, 0x10, 0)); TRY(wr(0x0B, 1));

    TRY(wr(0x01, 0xE8)); TRY(set_default_budget());

    TRY(calibrate(1, 0x40));

    TRY(calibrate(2, 0));
    TRY(wr(0x01, 0xE8));
    TRY(rd(0x01, &id));
    if (id != 0xE8) return VL53L0X_ERROR_CONFIG;
    TRY(transfer(0x44, check, 2, 1));
    if (check[0] != rate[0] || check[1] != rate[1]) return VL53L0X_ERROR_CONFIG;
    return VL53L0X_OK;
}

VL53L0X_Status VL53L0X_Init(void)
{
    VL53L0X_Status status;
    initialized = 0; continuous = 0;

    TRY(gpio_init());
    delay_ms(5);
    status = initialize_sensor();
    if (status != VL53L0X_OK) {
        /* Best effort restore page zero after failed SPAD/calibration setup. */
        (void)wr(0xFF, 0); (void)wr(0x80, 0); (void)wr(0x00, 0);
        return status;
    }
    initialized = 1;

    return VL53L0X_OK;
}

static VL53L0X_Status restore_stop_variable(void)
{
    TRY(wr(0x80, 1)); TRY(wr(0xFF, 1)); TRY(wr(0x00, 0));
    TRY(wr(0x91, stop_variable)); TRY(wr(0x00, 1));
    TRY(wr(0xFF, 0));
    return wr(0x80, 0);
}

static VL53L0X_Status read_result(VL53L0X_Result *result)
{
    uint8_t data[12];
    TRY(transfer(0x14, data, sizeof(data), 1));
    TRY(wr(0x0B, 1));
    result->raw_range_status = (data[0] & 0x78U) >> 3;
    result->distance_mm = (uint16_t)(((uint16_t)data[10] << 8) | data[11]);
    /* Device status 11 is range complete; 0 is NOT a successful ST API status. */
    return result->raw_range_status == 11 ? VL53L0X_OK : VL53L0X_ERROR_RANGE;
}

VL53L0X_Status VL53L0X_ReadSingle(VL53L0X_Result *result)
{
    VL53L0X_Status status;
    if (!result) return VL53L0X_ERROR_ARGUMENT;
    if (!initialized || continuous) return VL53L0X_ERROR_STATE;
    TRY(restore_stop_variable()); TRY(wr(0x0B, 1)); TRY(wr(0x00, 1));
    status = wait_reg(0x00, 1, 0);
    if (status == VL53L0X_OK) status = wait_reg(0x13, 7, 1);
    if (status != VL53L0X_OK) {
        (void)wr(0x00, 0); (void)wr(0x0B, 1);
        return status;
    }
    return read_result(result);
}

VL53L0X_Status VL53L0X_StartContinuous(void)
{
    if (!initialized || continuous) return VL53L0X_ERROR_STATE;
    TRY(restore_stop_variable()); TRY(wr(0x0B, 1)); TRY(wr(0x00, 2));
    continuous = 1;
    return VL53L0X_OK;
}

VL53L0X_Status VL53L0X_StopContinuous(void)
{
    if (!initialized || !continuous) return VL53L0X_ERROR_STATE;
    TRY(wr(0x00, 1)); TRY(wr(0xFF, 1)); TRY(wr(0x00, 0));
    TRY(wr(0x91, 0)); TRY(wr(0x00, 1)); TRY(wr(0xFF, 0)); TRY(wr(0x0B, 1));
    continuous = 0;
    return VL53L0X_OK;
}

VL53L0X_Status VL53L0X_ReadContinuous(VL53L0X_Result *result)
{
    uint8_t ready;
    if (!result) return VL53L0X_ERROR_ARGUMENT;
    if (!initialized || !continuous) return VL53L0X_ERROR_STATE;
    TRY(rd(0x13, &ready));
    if ((ready & 7) == 0) return VL53L0X_NOT_READY;
    return read_result(result);
}
