#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "lcd.h"

//�½���
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_Pin_1

//uart
#define UART1_PORT GPIOA
#define UART1_TX GPIO_Pin_9
#define UART1_RX GPIO_Pin_10

//������
#define USW_PORT GPIOC

#define USW1_TRIG_PIN GPIO_Pin_0
#define USW1_ECHO_PIN GPIO_Pin_1

#define USW2_TRIG_PIN GPIO_Pin_2
#define USW2_ECHO_PIN GPIO_Pin_3

#define USW3_TRIG_PIN GPIO_Pin_13
#define USW3_ECHO_PIN GPIO_Pin_14

//���͵���̹�
#define DRIVER_PORT GPIOB
#define DRIVER_PORT1 GPIO_Pin_1
#define DRIVER_PORT2 GPIO_Pin_8


//��������
#define SERVO_PORT GPIOB
#define SERVO_PIN GPIO_Pin_0


typedef enum { // ��ǳ�� ǳ������ Ÿ�� ����
   low,
   medium,
   high,
   stop,
}fan;

typedef enum { //  ��ǳ�� �������� Ÿ��
  left,
  right,
  front,
  all
}direct;

void Delay(uint32_t delayTime);
void delay(uint16_t time);
void delay_s(uint16_t time);
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);
void Time_Configure(void) ;
void TIM2_IRQHandler(void);
void USW_Set_Pin(GPIO_TypeDef *GPIOx, uint16_t TRIG_PIN, uint16_t ECHO_PIN);
int ReadDistance(GPIO_TypeDef *GPIOx, uint16_t TRIG_PIN, uint16_t ECHO_PIN);   //������ ���� �Ÿ� ����
void DC_Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DC_Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start (void);
uint8_t DHT11_Check_Response (void);
uint8_t DHT11_Read (void);
void GetValue(float *Temp, float *Rh);  //�½��� ���� ����
void SetPulse(uint32_t pulse);   //��������(400~2500)
void SetSpeed(uint32_t speed);  //DC���� �ӵ� ����(500~2000)
void USART1_Init(void);
void USART1_IRQHandler(void);
void USART2_Init(void);
void sendDataUART1(uint16_t data);
void sendDataUART2(uint16_t data);
