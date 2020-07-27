#ifndef __I2C_H_H__
#define __I2C_H_H__
// G1KuL1N
// ПОДКЛЮЧИТЬ БИБЛИОТЕКУ СВОЕГО КОНТРОЛЛЕРА

#include "stm32f1xx_hal.h"

// В CUBE MX порты I2C настроить на выход (либо в main.c вручну подать тактирование на нужны GPIO) 

//---подключение шины к пинам-----------------------------------------------------
#define SCL_I HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);   
#define SDA_I HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
#define SCL_O HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#define SDA_O HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//--------------------------------------------------------------------------------
void SCL_in (void);
void SCL_out (void);
void SDA_in (void);
void SDA_out (void);
__STATIC_INLINE void Delay_us (uint32_t __IO us);
void i2c_init (void);               // Инициализация шины
void i2c_start_cond (void);        // Генерация условия старт
void i2c_restart_cond (void);      // Генерация условия рестарт
void i2c_stop_cond (void) ;       // Генерация условия стоп  
uint8_t i2c_send_byte (uint8_t data) ;      //Передать байт (вх. аргумент передаваемый байт) (возвращает 0 - АСК, 1 - NACK) 
uint8_t i2c_get_byte (uint8_t last_byte) ;  //Принять байт (если последний байт то входной аргумент = 1, если будем считывать еще то 0)(возвращает принятый байт)
//--------------------------------------------------------------------------------
// ПРИМЕР ИСПОЛЬЗОВАНИЯ
//=========================================================================================
//   Запись uint16_t во внешнюю еепром (FRAM FM24CL64) или любой другой 24LC памяти, 
//	 две ячейки,  указывается адрес первой ячейки, следующая идет adr++
//=========================================================================================
//
//void FRAM_W_INT(uint16_t adr, uint16_t dat){
//i2c_start_cond ();
//i2c_send_byte (0xA2); //адрес чипа + что будем делать (записывать)
//i2c_send_byte    ((adr & 0xFF00) >> 8);  
//i2c_send_byte    (adr & 0x00FF);
//i2c_send_byte    ((dat & 0xFF00) >> 8);  
//i2c_send_byte    (dat & 0x00FF);
//i2c_stop_cond();
//}

//=========================================================================================
//   Считывание uint16_t из внешней еепром (FRAM FM24CL64) или любой другой 24LC памяти, 
//	 две ячейки,  указывается адрес первой ячейки, следующая идет adr++
//=========================================================================================
//uint16_t FRAM_R_INT(uint16_t adr){
//uint16_t dat;
//i2c_start_cond ();
//i2c_send_byte (0xA2);
//i2c_send_byte    ((adr & 0xFF00) >> 8);  
//i2c_send_byte    (adr & 0x00FF);
//i2c_restart_cond ();
//i2c_send_byte (0xA3);
//dat =  i2c_get_byte(0);	
//dat <<= 8; 
//dat |= i2c_get_byte(1);
//i2c_stop_cond();
//return dat;
//}

// G1KuL1N

#endif /* __I2C_H_H__ */