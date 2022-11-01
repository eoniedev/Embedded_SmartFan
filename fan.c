#include "fan.h"
#include "lcd.h"

uint32_t current_time = 0;                                  // 현재시각
TIM_OCInitTypeDef TIM_OCInitStructureServo;
TIM_OCInitTypeDef TIM_OCInitStructureDC;
  
void RCC_Configure(void){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   // RCC GPIO E
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);    // RCC GPIO C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // RCC GPIO B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);    // RCC GPIO A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   
    // TIM clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);    //온습도 delay TIM
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    //dc모터 pulse TIM
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //서보모터 pulse TIM
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    //초음파센서 delay TIM
   
    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    /* USART2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

void GPIO_Configure(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DHT11_PIN; // 온습도센서 PA1 enable
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(DHT11_PORT, DHT11_PIN);
    
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_8);
    // PB0 서보모터, PB1, PB8 dc모터 enable
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* UART pin setting */
    // UART1 pin setting
    GPIO_InitTypeDef GPIO_InitStructureA;
      //TX
    GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructureA);
    //RX
    GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructureA);
    
    // UART2 pin setting
    //TX
    GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructureA);
    //RX
    GPIO_InitStructureA.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructureA.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructureA.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructureA);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //TIMER 2
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // TIMER 3
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
    // TIMER 5
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
  
    // UART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // UART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Time_Configure(void) {
    // TIMER 2 ENABLE
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 72;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 1; // 1us 마다 interrupt
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM2, ENABLE);
    
    // TIMER 6 ENABLE
    TIM_TimeBaseStructure.TIM_Period = 0xffff-1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM6, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
    
    // TIMER 3 ENABLE
    uint16_t prescale = (uint16_t) (SystemCoreClock / 1000000);
    TIM_TimeBaseStructure.TIM_Period = 20000; 
    TIM_TimeBaseStructure.TIM_Prescaler = prescale; //72
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_OCInitStructureServo.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructureServo.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructureServo.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructureServo.TIM_Pulse = 1500; // us
    TIM_OC3Init(TIM3, &TIM_OCInitStructureServo);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
    // TIMER 4 ENABLE
    TIM_TimeBaseStructure.TIM_Period = 2000;  // 2ms
    TIM_TimeBaseStructure.TIM_Prescaler = prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_OCInitStructureDC.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructureDC.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructureDC.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructureDC.TIM_Pulse = 2000; // 2ms
    TIM_OC3Init(TIM4, &TIM_OCInitStructureDC);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// TIM2 Handler
void TIM2_IRQHandler(void){ 
    if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET){
        current_time++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
}

/**************************************************************************************************/
/********************************************온습도센서********************************************/
/**************************************************************************************************/
void DC_Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// 온습도에 사용하는 data pin을 input, output 모두 사용하기 위한 GPIO 함수 재정의 
void DC_Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// 18ms 대기 후 통신 시작 
void DHT11_Start (void) {
    DC_Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
    GPIO_ResetBits(DHT11_PORT, DHT11_PIN);   // pull the pin low
    delay (18000);   // wait for 18ms
    GPIO_SetBits(DHT11_PORT, DHT11_PIN);   // pull the pin high
    delay (20);   // wait for 20us
    DC_Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

// 온습도센서 pin response check
uint8_t DHT11_Check_Response (void) {
    uint8_t Response = 0;
    delay (40);
    if (!(GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)))
    {
        delay (80);
        if ((GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN))) Response = 1;
        else Response = -1; // 255
    }
    while ((GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low
    return Response;
}

// 40bit data 읽어오기 위해 8bit씩 총 5번 호출
uint8_t DHT11_Read (void) {
    uint8_t i,j;
    for (j=0;j<8;j++)
    {
        while (!(GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
        delay (40);   // wait for 40 us
        if (!(GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)))   // if the pin is low
        {
            i&= ~(1<<(7-j));   // write 0
        }
        else i|= (1<<(7-j));  // if the pin is high, write 1
        while ((GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
    }
    return i;
}

// 온습도 센서 측정값 저장
void GetValue(float *Temp, float *Rh){ 
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
    uint16_t SUM, RH, TEMP;
    uint8_t Presence = 0;
    DHT11_Start();
    Presence = DHT11_Check_Response();
    Rh_byte1 = DHT11_Read ();
    Rh_byte2 = DHT11_Read ();
    Temp_byte1 = DHT11_Read ();
    Temp_byte2 = DHT11_Read ();
    SUM = DHT11_Read();

    TEMP = Temp_byte1;
     RH = Rh_byte1;
                
    *Temp = (float)TEMP;
    *Rh = (float)RH;
}

/**************************************************************************************************/
/**********************************************서보모터********************************************/
/**************************************************************************************************/
void SetPulse(uint32_t pulse){ // 서보모터 방향 제어
    TIM_OCInitStructureServo.TIM_Pulse = pulse; // us
    TIM_OC3Init(TIM3, &TIM_OCInitStructureServo);
}

/**************************************************************************************************/
/***********************************************DC모터********************************************/
/**************************************************************************************************/

// dc모터 속도 제어
void SetSpeed(uint32_t speed){
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    TIM_OCInitStructureDC.TIM_Pulse = speed; // us
    TIM_OC3Init(TIM4, &TIM_OCInitStructureDC);
}
// dc모터 정지
void stopMotor(){ 
    TIM_OCInitStructureDC.TIM_Pulse = 0; // us
    TIM_OC3Init(TIM4, &TIM_OCInitStructureDC);
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}

/***************************************************************************************************/

// putty로 data 전송
void sendDataUART1(uint16_t data) { 
    USART_SendData(USART1, data);
}

void USART1_Init(void){
    USART_InitTypeDef USART1_InitStructure;
    USART_Cmd(USART1, ENABLE);

    USART1_InitStructure.USART_BaudRate = 9600; // baudrate = 9600
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART1_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

// uart1 handler
void USART1_IRQHandler() { 
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(USART1);
        sendDataUART2(word);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// phone bluetooth data 전송
void sendDataUART2(uint16_t data) { 
    USART_SendData(USART2, data);
}

void USART2_Init(void) {
    USART_InitTypeDef USART2_InitStructure;
    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_BaudRate = 9600; // baudrate = 9600
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART2_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

/**************************************************************************************************/
/********************************************초음파센서********************************************/
/**************************************************************************************************/

// 초음파센서 pin setting
void USW_Set_Pin(GPIO_TypeDef *GPIOx, uint16_t TRIG_PIN, uint16_t ECHO_PIN){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN; // TRIG 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
        
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN; // ECHO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

// 초음파센서 거리측정
int ReadDistance(GPIO_TypeDef* GPIOx, uint16_t TRIG_PIN, uint16_t ECHO_PIN) {
    uint32_t prev_time = 0;
    USW_Set_Pin(GPIOx, TRIG_PIN, ECHO_PIN);
    GPIO_SetBits(GPIOx, TRIG_PIN); // trig pin set
    GPIO_ResetBits(GPIOx, ECHO_PIN); // echo pin reset
    Delay(10);
    GPIO_ResetBits(GPIOx, TRIG_PIN); // trig pin reset
    uint8_t value = GPIO_ReadInputDataBit(GPIOx, ECHO_PIN); // echo pin data 읽어옴
    prev_time = current_time; // 시간 임시 저장

    while (value == RESET) { // 초기 echo pin 값이 reset, 아직 물체가 감지되지 않음
        if (current_time - prev_time >= 5000) break; // 5ms 초과시 error return
        else {
            value = GPIO_ReadInputDataBit(GPIOx, ECHO_PIN);
        }
    }

    if (value == SET) // 5ms 안에 물체를 감지했을 경우
    {
        prev_time = current_time;
        while (GPIO_ReadInputDataBit(GPIOx, ECHO_PIN) != RESET);

        return (current_time - prev_time) * 34 / 1000;
        // 측정 전 시간과 측정 후 시간을 이용해 거리 측정
    }
    else {
        return -1; // error return
    }
}

void delay(uint16_t time){ // 온습도 센서를 위한 delay
    TIM_SetCounter(TIM6, 0);
    while(TIM_GetCounter(TIM6)<time);
}

void delay_s(uint16_t time){ // 1초 세는 delay
    time = time*1000;
    for(int i = 0; i< 1000; i++){
      TIM_SetCounter(TIM6, 0);
      while(TIM_GetCounter(TIM6)<time);
    }
}

void Delay(uint32_t delayTime){ // 초음파센서 딜레이
  uint32_t prev_time = current_time;
  while(1)
  {
    if(current_time - prev_time > delayTime) break;
  }
}

