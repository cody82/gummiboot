#include "mpu9250.h"


static uint8_t read_reg(mpu9250 *context, uint8_t reg)
{
  uint8_t read[2];
  uint8_t write[2] = {reg | READ_FLAG, 0};
  mpu9250_spi_transfer_callback(context, write, read, 2);
  return read[1];
}

static void write_reg(mpu9250 *context, uint8_t reg, uint8_t value)
{
  uint8_t read[2];
  uint8_t write[2] = {reg, value};
  mpu9250_spi_transfer_callback(context, write, read, 2);
}

uint8_t mpu9250_init(mpu9250 *context)
{
volatile uint32_t i;
  write_reg(context, MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
for(i=0;i<1000000;++i);
  write_reg(context, MPUREG_PWR_MGMT_1, 0x01);  
  write_reg(context, MPUREG_PWR_MGMT_2, 0x00);
for(i=0;i<1000000;++i);

write_reg(context, MPUREG_CONFIG, 0x03);  

  if(read_reg(context, MPUREG_WHOAMI) != 0x71)
  {
    return 1;
  }

  //uint8_t c =  read_reg(context, MPUREG_GYRO_CONFIG);
  //write_reg(context, MPUREG_GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  //write_reg(context, MPUREG_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  //writeByte(MPUREG_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

  return 0;
}

uint8_t mpu9250_update(mpu9250 *context)
{
  uint8_t read[7];
  uint8_t write[7] = {MPUREG_GYRO_XOUT_H | READ_FLAG};
  uint8_t i;
  uint16_t tmp;

  mpu9250_spi_transfer_callback(context, write, read, 7);

  for(i=0;i<3;++i)
  {
    context->gyro[i] = ((int16_t)(read[i*2 + 1])<<8) | (uint16_t)read[i*2+2];
    /*if(tmp >= (1<<15))
    {
      context->gyro[i] = ((1<<16) - 1) - tmp;
    }
    else
    {
      context->gyro[i] = -tmp;
    }*/
  }
  
  return 0;
}
