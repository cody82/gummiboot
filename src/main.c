#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "mpu9250.h"
#include "sbus.h"

void pwm_init_timer(volatile uint32_t *reg, uint32_t en, uint32_t timer_peripheral, uint32_t prescaler, uint32_t period)
{
  rcc_peripheral_enable_clock(reg, en);

  timer_reset(timer_peripheral);

  timer_set_mode(timer_peripheral, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_set_prescaler(timer_peripheral, prescaler);
  timer_set_repetition_counter(timer_peripheral, 0);
  timer_enable_preload(timer_peripheral);
  timer_continuous_mode(timer_peripheral);
  timer_set_period(timer_peripheral, period);
}

uint32_t temp32;
volatile uint32_t tick;
volatile uint32_t signal=100;

void sys_tick_handler(void)
{
  temp32++;
  tick++;
  signal++;

  if (temp32 == 50) {
    if(signal < 100 && !_failsafe)
      gpio_toggle(GPIOB, GPIO12);
    else
      gpio_set(GPIOB, GPIO12);

    temp32 = 0;
  }
}

void pwm_init_output_channel(uint32_t timer_peripheral, enum tim_oc_id oc_id,
     volatile uint32_t *gpio_reg, uint32_t gpio_en, uint32_t gpio_port, uint16_t gpio_pin)
{
   rcc_peripheral_enable_clock(gpio_reg, gpio_en);

   gpio_set_mode(gpio_port, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                 gpio_pin);

   timer_disable_oc_output(timer_peripheral, oc_id);
   timer_set_oc_mode(timer_peripheral, oc_id, TIM_OCM_PWM1);
   timer_set_oc_value(timer_peripheral, oc_id, 0);
   timer_enable_oc_output(timer_peripheral, oc_id);
}

static void clock_setup(void)
{
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable GPIOA clock. */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
  rcc_periph_clock_enable(RCC_USART1);

  
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_AFIO);
}

static void usart_setup(void)
{
  nvic_enable_irq(NVIC_USART1_IRQ);

  /* Setup GPIO pin GPIO_USART2_TX. */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 100000);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_2);
  usart_set_mode(USART1, USART_MODE_RX);
  usart_set_parity(USART1, USART_PARITY_EVEN);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  USART_CR1(USART1) |= USART_CR1_RXNEIE;

  /* Finally enable the USART. */
  usart_enable(USART1);
}

static void spi_setup(void)
{
  /* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 |
            								GPIO5 |
                                            GPIO7 );

  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
          GPIO6);

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI1);

  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_128, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  //spi_enable_software_slave_management(SPI1);
  spi_disable_software_slave_management(SPI1);
  //spi_set_nss_high(SPI1);

  /* Enable SPI1 periph. */
  spi_enable(SPI1);
}

static void gpio_setup(void)
{
  /* Set GPIO (in GPIO port A) to 'output push-pull'. */
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO14);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
}



mpu9250 mpu;
uint8_t mpu9250_spi_transfer_callback(mpu9250 *context, uint8_t *write, uint8_t *read, uint32_t size)
{
  uint32_t i;
  spi_enable(SPI1);
  for(i=0;i<size;++i)
  {
    read[i] = spi_xfer(SPI1, write[i]);
  }

  spi_disable(SPI1);
  return 0;
}

void usart1_isr(void)
{
  uint8_t data;

  if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
      ((USART_SR(USART1) & USART_SR_RXNE) != 0))
  {
    data = usart_recv(USART1);

    if(sbus_receive(data))
    {
      //gpio_toggle(GPIOB, GPIO12);	/* LED on/off */
      signal = 0;
    }
  }
}

uint16_t rx_value = 0x42;
int motor1,motor2;

#define PWM_MID (1450)

void main(void)
{
  int i, j = 0;
  uint8_t c;
  int counter = 0;

  clock_setup();
  gpio_setup();
  usart_setup();
  spi_setup();

  pwm_init_timer(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN, TIM2, 72, 20000);
  pwm_init_output_channel(TIM2, TIM_OC1, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH1_ETR);
  pwm_init_output_channel(TIM2, TIM_OC2, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH2);
  pwm_init_output_channel(TIM2, TIM_OC3, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH3);
  pwm_init_output_channel(TIM2, TIM_OC4, &RCC_APB2ENR, RCC_APB2ENR_IOPAEN, GPIOA, GPIO_TIM2_CH4);
  timer_enable_counter(TIM2);
  timer_set_oc_value(TIM2, TIM_OC1, PWM_MID);
  timer_set_oc_value(TIM2, TIM_OC2, PWM_MID);
  timer_set_oc_value(TIM2, TIM_OC3, PWM_MID);
  timer_set_oc_value(TIM2, TIM_OC4, PWM_MID);

  if(mpu9250_init(&mpu) != 0)
  {
    printf("mpu error\n");
    while(1);
  }

  printf("mpu ok\n");

  /* 72MHz / 8 => 9000000 counts per second */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
  /* SysTick interrupt every N clock pulses: set reload to N-1 */
  systick_set_reload(8999);
  systick_interrupt_enable();
  systick_counter_enable();

  while (1)
  {
    mpu9250_update(&mpu);
    if (signal < 100 && !_failsafe)
    {
      j = (mpu.gyro[2] + 20) * 200 / 32768;
      motor1 = PWM_MID + ((int)_channels[1] - 1000) / 2 - j + ((int)_channels[0] - 1000) / 2;
      motor2 = PWM_MID + ((int)_channels[1] - 1000) / 2 + j - ((int)_channels[0] - 1000) / 2;

      timer_set_oc_value(TIM2, TIM_OC1, motor1);
      timer_set_oc_value(TIM2, TIM_OC2, motor2);
    }
    else
    {
      timer_set_oc_value(TIM2, TIM_OC1, PWM_MID);
      timer_set_oc_value(TIM2, TIM_OC2, PWM_MID);
    }
  }
}
