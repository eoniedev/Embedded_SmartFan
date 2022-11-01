#include "fan.h"

float Temperature = 0;
float Humidity = 0;
int temp = 0;

uint16_t word;



volatile uint32_t distanceL, distanceM, distanceR = 0;                                // ����, �߾�, ������ ������ ����

Fan f; // ��ǳ��  state
Direct d; // ��ǳ�� direction

void checkStatus(void){
    if(word == 'x')                                                     f = stop;     // phone�� x �Է½� ��ǳ�� ����
    else if (distanceL < 5 || distanceM < 5 || distanceR<5)             f = stop;     // �Ÿ��� �ʹ� ������ (5cm) ��ǳ�� ����
    else if (distanceL > 50  && distanceM > 50  && distanceR> 50)       f = stop;     // �Ÿ��� �ʹ� �־����� (50cm) ��ǳ�� ����
    else if (distanceL < 15 || distanceM < 15 || distanceR<15)          f = low;      // �Ÿ��� 15cm �̳��� ��ǳ�� �ӵ� 1�ܰ�
    else if (distanceL < 30 || distanceM < 30 || distanceR<30)          f = medium;   // �Ÿ��� 30cm �̳��� ��ǳ�� �ӵ� 2�ܰ�
    else if (distanceL < 50 || distanceM < 50 || distanceR<50)          f = high;     // �Ÿ��� 50cm �̳��� ��ǳ�� �ӵ� 3�ܰ�
    
    if(distanceL > 5 && distanceR > 5&& distanceL < 50 && distanceR < 50) d = all;    // 2���̻��� ������ �������� �νĵɰ�� ��ǳ�� ȸ��
    else if (distanceL > 5&& distanceL < 50 )                             d = left;   // ���ʿ��� �νĵ� ���
    else if  (distanceR > 5&& distanceR < 50 )                            d = right;  // �����ʿ��� �νĵ� ���
    else if  (distanceM > 5 && distanceM < 50)                            d = front;  // �߾ӿ��� �νĵ� ���

}

//USART 2 Handler
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
      char msg[10];
      int i ;
      for(i=0; i<strlen(msg); i++){
        msg[i] = USART_ReceiveData(USART2);
        if(msg[0] == 'o' || msg[0] == 'x'){     // o or x �� �Էµɰ��
          word = msg[i];                        // ��ɾ ���������� ����
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

            distanceL = ReadDistance(USW_PORT, USW1_TRIG_PIN, USW1_ECHO_PIN);   // ���� ������ 
            distanceM = ReadDistance(USW_PORT, USW2_TRIG_PIN, USW2_ECHO_PIN);   // �߰� ������
            distanceR = ReadDistance(USW_PORT, USW3_TRIG_PIN, USW3_ECHO_PIN);   // ������ ������
            GetValue(&Temperature, &Humidity);                                  //�½���
            
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
            // o�� �Էµ� ��� Remote Input : ON ��� �� ��ǳ�� ON
            else if (word == 'x') LCD_ShowString(150, 150, "OFF", BLACK, WHITE);
            // x�� �Էµ� ��� Remote Input : OFF ��� �� ��ǳ�� OFF

            checkStatus();        // ���� ��ǳ�� ���� ����
            if(Temperature>25)    // �µ��� 25�� �̻��� ��� ��ǳ�� �ӵ� 200 ����
              temp = 200; 
            else
              temp = 0;
            
            switch(f){
            case stop:              // ����
                SetSpeed(0);        //dc����
                break;
            case low:               // �ӵ� 1�ܰ�
                SetSpeed(500+temp); // dc���� �ӵ� ���� 
                switch(d){
                case left:          // ���� ����
                  SetPulse(2000);
                  break;
                case front:         // �߾� ����
                  SetPulse(1000);
                  break;
                case right:         // ������ ����
                  SetPulse(450);
                  break;
                case all:           // ��ǳ�� ȸ��
                  SetPulse(450);
                  delay_s(1);
                  SetPulse(2000);

                  break;
                }
                break;    
            case medium:            // �ӵ� 2�ܰ�
                SetSpeed(1200+temp);//dc����
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
            case high:              // �ӵ� 3�ܰ�
                SetSpeed(1800+temp);//dc����
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
