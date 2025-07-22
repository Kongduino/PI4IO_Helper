#ifndef __pi4io_UNITC6L__
#define __pi4io_UNITC6L__
#define PI4IO_M_ADDR 0x43
#include <Wire.h>

void pi4io_init_C6();
uint8_t buttonState_C6();
void i2c_write_byte(uint8_t, uint8_t, uint8_t);
void i2c_read_byte(uint8_t, uint8_t, uint8_t *);

// PI4IO registers
#define PI4IO_REG_CHIP_RESET 0x01
#define PI4IO_REG_IO_DIR 0x03
#define PI4IO_REG_OUT_SET 0x05
#define PI4IO_REG_OUT_H_IM 0x07
#define PI4IO_REG_IN_DEF_STA 0x09
#define PI4IO_REG_PULL_EN 0x0B
#define PI4IO_REG_PULL_SEL 0x0D
#define PI4IO_REG_IN_STA 0x0F
#define PI4IO_REG_INT_MASK 0x11
#define PI4IO_REG_IRQ_STA 0x13

void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  *value = Wire.read();
}

/*******************************************************************/
void pi4io_init_C6() {
  // P7 LoRa Reset
  // P6 RF Switch
  // P5 LNA Enable
  printf("pi4io_init\n");
  uint8_t in_data;
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_CHIP_RESET, 0xFF);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_read_byte(PI4IO_M_ADDR, PI4IO_REG_CHIP_RESET, &in_data);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_IO_DIR, 0b11100000);
  // 0: input 1: output   P7、P6、P5 set output
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_OUT_H_IM, 0b00111100);
  // The pin used turns off the High-Impedance parameter
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_PULL_SEL, 0b11000011);
  // pull up/down select, 0 down, 1 up
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_PULL_EN, 0b11000011);
  // pull up/down enable, 0 disable, 1 enable
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_IN_DEF_STA, 0b00000011);
  // P0 P1 The default level is high. Pressing the key triggers the interrupt
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_INT_MASK, 0b11111100);
  // P0 P1 disable 0 enable, 1 disable
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_byte(PI4IO_M_ADDR, PI4IO_REG_OUT_SET, 0b10000000);
  // Default output
  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_read_byte(PI4IO_M_ADDR, PI4IO_REG_IRQ_STA, &in_data);
  // 读取IRQ_STA清除标志
}

uint8_t buttonState_C6() {
  uint8_t in_data;
  i2c_read_byte(PI4IO_M_ADDR, PI4IO_REG_IRQ_STA, &in_data);
  return (in_data & 1);
}
#endif
