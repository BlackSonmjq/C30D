/* Exercise real A4988 direction writes; no physical hardware is accessed. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef struct { u16 level, outputs; } GPIO_TypeDef;
typedef struct {
    u16 GPIO_Pin;
    unsigned GPIO_Mode, GPIO_OType, GPIO_PuPd, GPIO_Speed;
} GPIO_InitTypeDef;
static GPIO_TypeDef port_b, port_c;
#define GPIOB (&port_b)
#define GPIOC (&port_c)
#define GPIO_Pin_7 (1U << 7)
#define GPIO_Pin_8 (1U << 8)
#define GPIO_Pin_14 (1U << 14)
#define GPIO_Pin_15 (1U << 15)
#define GPIO_Mode_OUT 1U
#define GPIO_OType_PP 0U
#define GPIO_PuPd_NOPULL 0U
#define GPIO_Speed_2MHz 0U
#define RCC_AHB1Periph_GPIOB 2U
#define RCC_AHB1Periph_GPIOC 4U
#define ENABLE 1U
#define A4988_MOTOR_Left 1U
#define A4988_MOTOR_Right 2U
static unsigned checks, writes, waits;
static int forced_input = -1;
static u16 last_pin;
static GPIO_TypeDef *last_port;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #expr); exit(1); } } while (0)

void RCC_AHB1PeriphClockCmd(u32 mask, unsigned enabled) { (void)mask; (void)enabled; }
void GPIO_SetBits(GPIO_TypeDef *port, u16 pin) {
    ++writes; last_port = port; last_pin = pin; port->level |= pin;
}
void GPIO_ResetBits(GPIO_TypeDef *port, u16 pin) {
    ++writes; last_port = port; last_pin = pin; port->level &= ~pin;
}
void GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *gpio) {
    CHECK(gpio->GPIO_Mode == GPIO_Mode_OUT && gpio->GPIO_OType == GPIO_OType_PP);
    port->outputs |= gpio->GPIO_Pin;
}
void delay_us(u32 us) { CHECK(us == 2U); ++waits; }
u8 GPIO_ReadInputDataBit(GPIO_TypeDef *port, u16 pin) {
    CHECK(port->outputs & pin);
    CHECK(waits == writes); /* Readback must follow the setup delay. */
    return forced_input >= 0 ? (u8)forced_input : (port->level & pin) != 0;
}
#include ".build/a4988_direction.inc"

int main(void) {
    u8 motor;
    for (motor = 1; motor <= 2; ++motor) {
        GPIO_TypeDef *port = motor == 1 ? GPIOB : GPIOC;
        u16 dir = motor == 1 ? GPIO_Pin_14 : GPIO_Pin_7;
        u16 step = motor == 1 ? GPIO_Pin_15 : GPIO_Pin_8;
        port->level = step | dir;
        port->outputs = 0; /* Model a pin whose GPIO mode was lost. */
        CHECK(A4988_SetDir(motor, 0));
        CHECK(last_port == port && last_pin == dir);
        CHECK(!(port->level & dir) && (port->level & step));
        port->outputs = 0;
        CHECK(A4988_SetDir(motor, 1));
        CHECK((port->level & dir) && (port->level & step));
        CHECK(port->outputs == dir); /* Never reconfigure STEP here. */
        forced_input = 0;
        CHECK(!A4988_SetDir(motor, 1)); /* Stuck-low DIR must fail. */
        forced_input = 1;
        CHECK(!A4988_SetDir(motor, 0)); /* Stuck-high DIR must fail. */
        forced_input = -1;
    }
    CHECK(!A4988_SetDir(0, 0) && !A4988_SetDir(3, 1));
    CHECK(writes == 8 && waits == 8);
    printf("PASS: left/right DIR reversal, mode restore and stuck-pin detection, %u checks\n", checks);
    return 0;
}
