#include "fan.h"

float Temperature = 0;
float Humidity = 0;
int temp = 0;

uint16_t word;



volatile uint32_t distanceL, distanceM, distanceR = 0;                                // 왼쪽, 중앙, 오른쪽 초음파 센서

Fan f; // 선풍기  state
Direct d; // 선풍기 direction

void checkStatus(void){
    if(word == 'x')                                                     f = stop;     // phone에 x 입력시 선풍기 정지
    else if (distanceL < 5 || distanceM < 5 || distanceR<5)             f = stop;     // 거리가 너무 가까울시 (5cm) 선풍기 정지
    else if (distanceL > 50  && distanceM > 50  && distanceR> 50)       f = stop;     // 거리가 너무 멀어져도 (50cm) 선풍기 정지
    else if (distanceL < 15 || distanceM < 15 || distanceR<15)          f = low;      // 거리가 15cm 이내면 선풍기 속도 1단계
    else if (distanceL < 30 || distanceM < 30 || distanceR<30)          f = medium;   // 거리가 30cm 이내면 선풍기 속도 2단계
    else if (distanceL < 50 || distanceM < 50 || distanceR<50)          f = high;     // 거리가 50cm 이내면 선풍기 속도 3단계
    
    if(distanceL > 5 && distanceR > 5&& distanceL < 50 && distanceR < 50) d = all;    // 2개이상의 초음파 센서에서 인식될경우 선풍기 회전
    else if (distanceL > 5&& distanceL < 50 )                             d = left;   // 왼쪽에서 인식될 경우
    else if  (distanceR > 5&& distanceR < 50 )                            d = right;  // 오른쪽에서 인식될 경우
    else if  (distanceM > 5 && distanceM < 50)                            d = front;  // 중앙에서 인식될 경우

}

//USART 2 Handler
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
      char msg[10];
      int i ;
      for(i=0; i<strlen(msg); i++){
        msg[i] = USART_ReceiveData(USART2);
        if(msg[0] == 'o' || msg[0] == 'x'){     // o or x 가 입력될경우
          word = msg[i];                        // 명령어를 전역변수에 저장
          sendDataUART1(msg[i]);
        }
      }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

int main(void) {
   SystemInit();
   RCC_Configure();
   GPIO_Configure();
   NVIC_Configure();
   Time_Configure();
   USART1_Init();
   USART2_Init();
   
   LCD_Init();
   Touch_Configuration();
   LCD_Clear(WHITE);
   stopMotor();
   SetPulse(500);
   delay_s(3);

      while (1) {

            distanceL = ReadDistance(USW_PORT, USW1_TRIG_PIN, USW1_ECHO_PIN);   // 왼쪽 초음파 
            distanceM = ReadDistance(USW_PORT, USW2_TRIG_PIN, USW2_ECHO_PIN);   // 중간 초음파
            distanceR = ReadDistance(USW_PORT, USW3_TRIG_PIN, USW3_ECHO_PIN);   // 오른쪽 초음파
            GetValue(&Temperature, &Humidity);                                  //온습도
            
            LCD_ShowString(1, 20, "Left Ultrasound: ", BLACK, WHITE);
            LCD_ShowNum(150, 20 , distanceL, 4, BLACK, WHITE);
            LCD_ShowString(1,40, "Middle Ultrasound: ", BLACK, WHITE);
            LCD_ShowNum(150, 40 , distanceM, 4, BLACK, WHITE);
            LCD_ShowString(1,60, "Right Ultrasound: ", BLACK, WHITE);
            LCD_ShowNum(150, 60 , distanceR, 4, BLACK, WHITE);
            
            LCD_ShowString(1, 80, "Temperature: ", BLACK, WHITE);
            LCD_ShowNum(150,80 , Temperature,4, BLACK, WHITE);
            LCD_ShowString(1, 100, "Humidity: ", BLACK, WHITE);
            LCD_ShowNum(150, 100 , Humidity,4, RED, WHITE);
            LCD_ShowString(1, 150, "Remote Input: ", BLACK, WHITE);
            if(word== 'o') LCD_ShowString(150, 150, "ON ", BLACK, WHITE);
            // o가 입력될 경우 Remote Input : ON 출력 후 선풍기 ON
            else if (word == 'x') LCD_ShowString(150, 150, "OFF", BLACK, WHITE);
            // x가 입력될 경우 Remote Input : OFF 출력 후 선풍기 OFF

            checkStatus();        // 현재 선풍기 상태 측정
            if(Temperature>25)    // 온도가 25도 이상일 경우 선풍기 속도 200 증가
              temp = 200; 
            else
              temp = 0;
            
            switch(f){
            case stop:              // 정지
                SetSpeed(0);        //dc모터
                break;
            case low:               // 속도 1단계
                SetSpeed(500+temp); // dc모터 속도 설정 
                switch(d){
                case left:          // 왼쪽 감지
                  SetPulse(2000);
                  break;
                case front:         // 중앙 감지
                  SetPulse(1000);
                  break;
                case right:         // 오른쪽 감지
                  SetPulse(450);
                  break;
                case all:           // 선풍기 회전
                  SetPulse(450);
                  delay_s(1);
                  SetPulse(2000);

                  break;
                }
                break;    
            case medium:            // 속도 2단계
                SetSpeed(1200+temp);//dc모터
                switch(d){
                case left:
                  SetPulse(2000);
                  break;
                case front:
                  SetPulse(1000);
                  break;
                case right:
                  SetPulse(450);
                  break;
                case all:
                  SetPulse(450);
                  delay_s(1);
                  SetPulse(2000);

                  break;
                }
                break;
            case high:              // 속도 3단계
                SetSpeed(1800+temp);//dc모터
                switch(d){
                case left:
                  SetPulse(2000);
                  break;
                case front:
                  SetPulse(1000);
                  break;
                case right:
                  SetPulse(450);
                  break;
                case all:
                  SetPulse(450);
                  delay_s(1);
                  SetPulse(2000);
                  break;
                }
                break;
            }
            delay_s(2);
      }
}
