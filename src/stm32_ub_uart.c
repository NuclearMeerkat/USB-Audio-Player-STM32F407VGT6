//--------------------------------------------------------------
// File     : stm32_ub_uart.c
// Datum    : 10.09.2014
// Version  : 1.4
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.4
// GCC      : 4.7 2012q4
// Module   : GPIO, USART, MISC
// Funktion : UART (RS232) In und OUT
//            Receive wird per Interrupt behandelt
//
// Hinweis  : moegliche Pinbelegungen
//            UART1 : TX:[PA9,PB6] RX:[PA10,PB7]
//            UART2 : TX:[PA2,PD5] RX:[PA3,PD6]
//            UART3 : TX:[PB10,PC10,PD8] RX:[PB11,PC11,PD9]
//            UART4 : TX:[PA0,PC10] RX:[PA1,PC11]
//            UART5 : TX:[PC12] RX:[PD2]
//            UART6 : TX:[PC6,PG14] RX:[PC7,PG9]
//
// Vorsicht : Als Endekennung beim Empfangen, muss der Sender
//            das Zeichen "0x0D" = Carriage-Return
//            an den String anhaengen !!
//--------------------------------------------------------------


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_ub_uart.h"




//--------------------------------------------------------------
// ����������� ���� UART��
// ����������� � UART_NAME_t
//--------------------------------------------------------------
//UART_t UART;

UART_t UART[] = {
		// Name, Clock               , AF-UART      ,UART  , Baud , Interrupt
		  {COM2,RCC_APB1Periph_USART2,GPIO_AF_USART2,USART2,115200,USART2_IRQn, // UART2 со скоростью 115200 бод
		// PORT , PIN      , Clock              , Source
		  {GPIOA,GPIO_Pin_2,RCC_AHB1Periph_GPIOA,GPIO_PinSource2},  // TX на PA2
		  {GPIOA,GPIO_Pin_3,RCC_AHB1Periph_GPIOA,GPIO_PinSource3}}, // RX на PA3

		// Name, Clock               , AF-UART      ,UART  , Baud, Interrupt
		  {COM3,RCC_APB1Periph_USART3,GPIO_AF_USART3,USART3,115200,USART3_IRQn, // UART3 со скоростью 115200 бод
		// PORT , PIN      , Clock              , Source
		  {GPIOD,GPIO_Pin_8,RCC_AHB1Periph_GPIOD,GPIO_PinSource8},  // TX на PD8
		  {GPIOD,GPIO_Pin_9,RCC_AHB1Periph_GPIOD,GPIO_PinSource9}}, // RX на PD9
};





//--------------------------------------------------------------
// ������������� UART��
//--------------------------------------------------------------
void UB_Uart_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  UART_NAME_t nr;

  for(nr=0;nr<UART_ANZ;nr++) {

    // ��������� ������������ ��� TX � RX �����
    RCC_AHB1PeriphClockCmd(UART[nr].TX.CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(UART[nr].RX.CLK, ENABLE);

    // ��������� ������������ UART
    if((UART[nr].UART==USART1) || (UART[nr].UART==USART6)) {
      RCC_APB2PeriphClockCmd(UART[nr].CLK, ENABLE);
    }
    else {
      RCC_APB1PeriphClockCmd(UART[nr].CLK, ENABLE);
    }

    // ��������� �������������� ������� UART ��� IO-�����
    GPIO_PinAFConfig(UART[nr].TX.PORT,UART[nr].TX.SOURCE,UART[nr].AF);
    GPIO_PinAFConfig(UART[nr].RX.PORT,UART[nr].RX.SOURCE,UART[nr].AF);

    // UART � �������� �������������� �������
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    // TX-Pin
    GPIO_InitStructure.GPIO_Pin = UART[nr].TX.PIN;
    GPIO_Init(UART[nr].TX.PORT, &GPIO_InitStructure);
    // RX-Pin
    GPIO_InitStructure.GPIO_Pin =  UART[nr].RX.PIN;
    GPIO_Init(UART[nr].RX.PORT, &GPIO_InitStructure);

    // �������
    USART_OverSampling8Cmd(UART[nr].UART, ENABLE);

    // ������������� ��������, 8 ��� ������, 1 �������, ��� ��������, ��� RTS+CTS
    USART_InitStructure.USART_BaudRate = UART[nr].BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART[nr].UART, &USART_InitStructure);

    // ��������� UART
    USART_Cmd(UART[nr].UART, ENABLE);

    // RX-���������� ��������
    USART_ITConfig(UART[nr].UART, USART_IT_RXNE, ENABLE);

    // ��������� ����������� UART �� �������
    NVIC_InitStructure.NVIC_IRQChannel = UART[nr].INT;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ���������� RX-������
    UART_RX[nr].rx_buffer[0]=RX_END_CHR;
    UART_RX[nr].wr_ptr=0;
    UART_RX[nr].rd_ptr=0;
    UART_RX[nr].status=RX_EMPTY;
  }
}

//--------------------------------------------------------------
// ��������� ���� ����� UART
//--------------------------------------------------------------
void UB_Uart_SendByte(UART_NAME_t uart, uint16_t wert)
{
  // ���������, ���� ����� ��������� ���������� ����
  while (USART_GetFlagStatus(UART[uart].UART, USART_FLAG_TXE) == RESET);
  USART_SendData(UART[uart].UART, wert);
}

//--------------------------------------------------------------
// ��������� ������ ����� UART
//--------------------------------------------------------------
void UB_Uart_SendString(UART_NAME_t uart, char *ptr, UART_LASTBYTE_t end_cmd)
{
  // ��������� ������ ������
  while (*ptr != 0) {
    UB_Uart_SendByte(uart,*ptr);
    ptr++;
  }
  // �������� �������������� ����� 
  if(end_cmd==LFCR) {
    UB_Uart_SendByte(uart,0x0A); // �������� �������� ������
    UB_Uart_SendByte(uart,0x0D); // �������� �������� �������
  }
  else if(end_cmd==CRLF) {    
    UB_Uart_SendByte(uart,0x0D); // �������� �������� �������
    UB_Uart_SendByte(uart,0x0A); // �������� �������� ������
  }
  else if(end_cmd==LF) {    
    UB_Uart_SendByte(uart,0x0A); // �������� �������� ������
  }
  else if(end_cmd==CR) {    
    UB_Uart_SendByte(uart,0x0D); // �������� �������� �������   
  }
}

//--------------------------------------------------------------
// ��������� ������ ����� UART
// (����� ���������� � �������������� ����������)
// ����������� ����� �������
// ���������� ��������:
//  -> ���� ������ �� �������� = RX_EMPTY
//  -> ���� �������� ������ = RX_READY -> String steht in *ptr
//  -> ���� ����� �����      = RX_FULL
//--------------------------------------------------------------
UART_RXSTATUS_t UB_Uart_ReceiveString(UART_NAME_t uart, char *ptr)
{
  UART_RXSTATUS_t ret_wert=RX_EMPTY;
  uint16_t rd_pos,wr_pos,n;
  uint8_t wert,ok;

  // ������ ������� ���������
  rd_pos=UART_RX[uart].rd_ptr;
  wr_pos=UART_RX[uart].wr_ptr;
  // �������� ������ � ������
  ok=0;
  if(rd_pos!=wr_pos) {
    // ����� ����� ��������������
    while(rd_pos!=wr_pos) {
      wert=UART_RX[uart].rx_buffer[rd_pos];
      if(wert==RX_END_CHR) {
        // ����� ������
        ok=1;
        break;
      }
      // ������� ���������
      rd_pos++;
      if(rd_pos>=RX_BUF_SIZE) rd_pos=0;
    }

    // ���� ������������� ����� ������, ������ ������
    if(ok==1) {
      // ������������� ����� ������
      ret_wert=RX_READY;
      // ������ ������ ������
      rd_pos=UART_RX[uart].rd_ptr;
      n=0;
      while(rd_pos!=wr_pos) {
        wert=UART_RX[uart].rx_buffer[rd_pos];
        // ������ ��������� ����������� ��������
        rd_pos++;
        if(rd_pos>=RX_BUF_SIZE) rd_pos=0;
        // ���������� ������
        if(wert!=RX_END_CHR) {
          ptr[n]=wert;
          n++;
        }
        else {
          break;
        }
      }
      // ������ � ������������� �����
      ptr[n]=0x00;
      // ������ ����������� ���������
      UART_RX[uart].rd_ptr=rd_pos;
    }
    else {
      // ������������� ����� �� ������
      ret_wert=RX_EMPTY;
    }
  }
  else {
    // ��������, ��������� �� ������������
    if(UART_RX[uart].status==RX_FULL) {
      ret_wert=RX_FULL;
      UART_RX[uart].status=RX_EMPTY;
    }
  }

  return(ret_wert);
}

//--------------------------------------------------------------
// ���������� �������
// ���������� ����������� ������� � �����
//--------------------------------------------------------------
void P_UART_Receive(UART_NAME_t uart, uint16_t wert)
{
  uint16_t wr_pos;

  // ��������� ���� � ���������� �������
  wr_pos=UART_RX[uart].wr_ptr;
  // ���������� AscII-�������� (� ��� ����� ������)
  if(((wert>=RX_FIRST_CHR) && (wert<=RX_LAST_CHR)) || (wert==RX_END_CHR)) {
    UART_RX[uart].rx_buffer[wr_pos]=wert;
    // ������ ��������� ����������� ��������
    wr_pos++;
    if(wr_pos>=RX_BUF_SIZE) wr_pos=0;
    UART_RX[uart].wr_ptr=wr_pos;

    // ���������, ���� �� ������������
    if(UART_RX[uart].wr_ptr==UART_RX[uart].rd_ptr) {
      UART_RX[uart].status=RX_FULL;
    }
  }
}


//--------------------------------------------------------------
// ���������� �������
// ������� ���������� UART
// ������ ���� ������� ����� ����������
//--------------------------------------------------------------
void P_UART_RX_INT(uint8_t int_nr, uint16_t wert)
{
  UART_NAME_t nr;

  // �������� ����� �������
  for(nr=0;nr<UART_ANZ;nr++) {
    if(UART[nr].INT==int_nr) {
      // ������� ������, ��������� ����
      P_UART_Receive(nr,wert);
      break;
    }
  }
}

//--------------------------------------------------------------
// UART1-����������
//--------------------------------------------------------------
void USART1_IRQHandler(void) {
  uint16_t wert;

  if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
    wert=USART_ReceiveData(USART1);
    // ��������� ����
    P_UART_RX_INT(USART1_IRQn,wert);
  }
}

//--------------------------------------------------------------
// UART2-����������
//--------------------------------------------------------------
void USART2_IRQHandler(void) {
  uint16_t wert;

  if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
        wert=USART_ReceiveData(USART2);
    // ��������� ����
    P_UART_RX_INT(USART2_IRQn,wert);
  }
}
//--------------------------------------------------------------
// UART3-����������
//--------------------------------------------------------------
void USART3_IRQHandler(void) {
  uint16_t wert;

  if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
        wert=USART_ReceiveData(USART3);
    // ��������� ����
    P_UART_RX_INT(USART3_IRQn,wert);
  }
}

//--------------------------------------------------------------
// UART4-����������
//--------------------------------------------------------------
void UART4_IRQHandler(void) {
  uint16_t wert;

  if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
        wert=USART_ReceiveData(UART4);
    // ��������� ����
    P_UART_RX_INT(UART4_IRQn,wert);
  }
}

// UART5 interrupt handler using the standard peripheral library
void UART5_IRQHandler(void) {
	uint16_t data;

    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        data = USART_ReceiveData(UART5);  // Read received data
        P_UART_RX_INT(UART5_IRQn, data);
    }
}

//--------------------------------------------------------------
// UART5-����������
//--------------------------------------------------------------
//void UART5_IRQHandler(void) {
//  uint16_t wert;

//  if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
//        wert=USART_ReceiveData(UART5);
    // ��������� ����
//    P_UART_RX_INT(UART5_IRQn,wert);
//  }
//}

//--------------------------------------------------------------
// UART6-����������
//--------------------------------------------------------------
void USART6_IRQHandler(void) {
  uint16_t wert;

  if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET) {
    // ���� ���� � ������ ������ ����
        wert=USART_ReceiveData(USART6);
    // ��������� ����
    P_UART_RX_INT(USART6_IRQn,wert);
  }
}
