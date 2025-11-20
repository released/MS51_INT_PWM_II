/*_____ I N C L U D E S ____________________________________________________*/
#include "numicro_8051.h"

#include "misc_config.h"

#include "detect_pulse.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

#define TIMER_DIV12_1ms_FOSC_240000  			(65536-2000)
#define TH0_INIT        						(HIBYTE(TIMER_DIV12_1ms_FOSC_240000)) 
#define TL0_INIT        						(LOBYTE(TIMER_DIV12_1ms_FOSC_240000))

#define TIMER_DIV12_100us_FOSC_240000  			(65536-200)
#define TH1_INIT        						(HIBYTE(TIMER_DIV12_100us_FOSC_240000)) 
#define TL1_INIT        						(LOBYTE(TIMER_DIV12_100us_FOSC_240000))

//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

volatile struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_500MS                   	(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_REVERSE2                 				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


#define UDIV_ROUND_NEAREST(a,b) 						( ((unsigned long)(a) + ((unsigned long)(b)/2u)) / (unsigned long)(b) )

/*
	100 Hz
	24MHz / 128 = 187,500 Hz; 
	187,500 / 100 Hz = 1,875 ticks → PERIOD=1875-1


	60 Hz
	24MHz / 128 = 187,500 Hz; 
	187,500 / 60 Hz = 3125 ticks → PERIOD=3125-1

	50 Hz
	24MHz / 128 = 187,500 Hz; 
	187,500 / 50 Hz = 3750 ticks → PERIOD=3750-1
*/

#define PWM_BASE_FREQ_HZ        						(100u)
// #define PWM_BASE_FREQ_HZ        						(120u)
#define PWM_DIV_FOR_FREQ      							(128u)
#define PWM_PERIOD_TICKS        						((SYS_CLOCK / PWM_DIV_FOR_FREQ) / PWM_BASE_FREQ_HZ)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile uint32_t counter_tick = 0;

/*_____ M A C R O S ________________________________________________________*/
#define SYS_CLOCK 										(24000000ul)


/*_____ F U N C T I O N S __________________________________________________*/


static uint32_t get_tick(void)
{
	return (counter_tick);
}

static void set_tick(uint32_t t)
{
	counter_tick = t;
}

static void tick_counter(void)
{
	counter_tick++;
}

#if defined (REDUCE_CODE_SIZE)
void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
}
void send_UARTHex(uint16_t u16Temp)
{
    uint8_t print_buf[16];
    uint32_t i = 15;
    uint32_t temp;

    *(print_buf + i) = '\0';
    do
    {
        i--;
        temp = u16Temp % 16;
        if(temp < 10)
            *(print_buf + i) = '0' + temp;
        else
            *(print_buf + i) = 'a' + (temp - 10) ;
        u16Temp = u16Temp / 16;
    }
    while(u16Temp != 0);
    send_UARTString(print_buf + i);
}

#endif

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


/*
	ch : channel index
	frequency : target frequency (unit:Hz) , ex:100 , 1000
	duty : percent
	resolution : 100 ~ 10000
	
	ex1 : 
	target duty = 95% => duty = 95,resolution = 100
	target duty = 87.5% => duty = 875,resolution = 1000
	target duty = 97.75% => duty = 9775,resolution = 10000
*/
void pwm_channel_Init(unsigned char ch,unsigned int duty,unsigned int resolution)
{
	unsigned int period = PWM_PERIOD_TICKS;
	unsigned int res = 0;

	/*
		set target channel and pin define
	*/
	switch(ch)
	{
		// case 0:
		// 	P12_PUSHPULL_MODE;	//Add this to enhance MOS output capability
		// 	ENABLE_PWM0_CH0_P12_OUTPUT;	
		// 	break;
		case 2:
			P10_PUSHPULL_MODE;	//Add this to enhance MOS output capability
			ENABLE_PWM0_CH2_P10_OUTPUT;	
			break;
	}
		
	/*
		24M/2^7 = 24000000/128 = 187500
		187500/freq = unsigned int	
		FIX freq to 100Hz
	*/
	PWM0_IMDEPENDENT_MODE;		
	PWM0_CLOCK_DIV_128;
    PWMPH = HIBYTE(PWM_PERIOD_TICKS -1u);
    PWMPL = LOBYTE(PWM_PERIOD_TICKS -1u);

	/*
		set duty
	*/
	// res = (unsigned long) duty*(MAKEWORD(PWMPH,PWMPL)+1)/resolution;
	res = (unsigned int)UDIV_ROUND_NEAREST((unsigned long)duty * period, (unsigned long)resolution);

	switch(ch)
	{
		// case 0:
		// 	PWM0H = HIBYTE(res);
		// 	PWM0L = LOBYTE(res);
		// 	break;
		case 2:
			PWM2H = HIBYTE(res);
			PWM2L = LOBYTE(res);
			break;		
	}

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void loop(void)
{
	// static uint16_t LOG = 0;	
	if (FLAG_PROJ_TIMER_PERIOD_1000MS)
	{
		FLAG_PROJ_TIMER_PERIOD_1000MS = 0;	
		// printf("LOG : %4d\r\n",LOG++);
		// P12 ^= 1;		
	}

	if (FLAG_PROJ_TIMER_PERIOD_500MS)
	{
		FLAG_PROJ_TIMER_PERIOD_500MS = 0;
		Detect_GetFreq_log();
	}	
}

void GPIO_Init(void)
{
	P12 = 0;
	// P17 = 0;
	P30 = 0;
	
	P12_PUSHPULL_MODE;		
	// P17_QUASI_MODE;		
	P30_PUSHPULL_MODE;	
}

void Timer1_ISR(void) interrupt 3        // Vector @  0x1B
{
    _push_(SFRS);	
	
    clr_TCON_TF1;
	TH1 = TH1_INIT;
	TL1 = TL1_INIT;	
		
	// P12 ^= 1;	// for debug period
	
	output_pulse_irq();

    _pop_(SFRS);	
}

void TIMER1_Init(void)
{
	/*
		formula : 16bit 
		(0xFFFF+1 - target)  / (24MHz/psc) = time base 
	*/	
	
    TIMER1_FSYS_DIV12;
	ENABLE_TIMER1_MODE1;	// Timer 1 as 16-bits mode

	TH1 = TH1_INIT;
	TL1 = TL1_INIT;
	clr_TCON_TF1;
    set_TCON_TR1;                                  //Timer1 run
    ENABLE_TIMER1_INTERRUPT;                       //enable Timer1 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts  

	// SET_INT_Timer1_LEVEL0;
	SET_INT_Timer1_LEVEL3;
}

void Timer0_IRQHandler(void)
{

	tick_counter();

	if ((get_tick() % 1000) == 0)
	{
		FLAG_PROJ_TIMER_PERIOD_1000MS = 1;

	}

	if ((get_tick() % 500) == 0)
	{
		FLAG_PROJ_TIMER_PERIOD_500MS = 1;

	}

	if ((get_tick() % 50) == 0)
	{

	}		
}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    _push_(SFRS);	
	
    clr_TCON_TF0;
	TH0 = TH0_INIT;
	TL0 = TL0_INIT;	
	
	Timer0_IRQHandler();

    _pop_(SFRS);	
}

void TIMER0_Init(void)
{
	/*
		formula : 16bit 
		(0xFFFF+1 - target)  / (24MHz/psc) = time base 
	*/	
	
	ENABLE_TIMER0_MODE1;	// mode 0 : 13 bit , mode 1 : 16 bit
    TIMER0_FSYS_DIV12;

	TH0 = TH0_INIT;
	TL0 = TL0_INIT;
	clr_TCON_TF0;

    set_TCON_TR0;                                  //Timer0 run
    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
}

void Serial_ISR (void) interrupt 4 
{
    _push_(SFRS);

    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }

    _pop_(SFRS);	
}

void UART0_Init(void)
{
	#if 1
	const unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	
	set_T3CON_TR3;         //Trigger Timer3
	
	/*
		set UART priority
		IPH / EIPH / EIPH1 	, IP / EIP / EIP2 
		0  						0  					Level 0 (lowest) 
		0  						1  					Level 1 
		1  						0  					Level 2 
		1  						1  					Level 3 (highest) 

		SET_INT_UART0_LEVEL0;	//clr_IP_PS; clr_IPH_PSH; //0
		SET_INT_UART0_LEVEL1;	//clr_IP_PS; set_IPH_PSH; //1
		SET_INT_UART0_LEVEL2;	//set_IP_PS; clr_IPH_PSH; //2
		SET_INT_UART0_LEVEL3;	//set_IP_PS; set_IPH_PSH; //3
	*/
	
	ENABLE_UART0_INTERRUPT;
	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


void MODIFY_HIRC_24(void)
{
	unsigned char u8HIRCSEL = HIRC_24;
    unsigned char data hircmap0,hircmap1;
//    unsigned int trimvalue16bit;
    /* Check if power on reset, modify HIRC */
    set_CHPCON_IAPEN;
    SFRS = 0 ;
	#if 1
    IAPAL = 0x38;
	#else
    switch (u8HIRCSEL)
    {
      case HIRC_24:
        IAPAL = 0x38;
      break;
      case HIRC_16:
        IAPAL = 0x30;
      break;
      case HIRC_166:
        IAPAL = 0x30;
      break;
    }
	#endif
	
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL++;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    // clr_CHPCON_IAPEN;

	#if 0
    switch (u8HIRCSEL)
    {
		case HIRC_166:
		trimvalue16bit = ((hircmap0 << 1) + (hircmap1 & 0x01));
		trimvalue16bit = trimvalue16bit - 15;
		hircmap1 = trimvalue16bit & 0x01;
		hircmap0 = trimvalue16bit >> 1;

		break;
		default: break;
    }
	#endif
	
    TA = 0xAA;
    TA = 0x55;
    RCTRIM0 = hircmap0;
    TA = 0xAA;
    TA = 0x55;
    RCTRIM1 = hircmap1;
    clr_CHPCON_IAPEN;
    // PCON &= CLR_BIT4;
}


void SYS_Init(void)
{
    MODIFY_HIRC_24();

    // ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

	/*
		P1.2 : test pin , to measure TIMER1 toggle period
	*/
    UART0_Init();
	GPIO_Init();
	TIMER0_Init();

	/*
		enable pin : P1.7 (EINT)
		pulse pin : P.15 (GPIO)
	*/
    TIMER1_Init();
	EINT1_Init();

	/*
		PWM pin : P1.0 (PWM0_CH2)
	*/
	pwm_channel_Init(2,50,100);

	// PWM_SetDutyPercent(20U);
	// PWM_SetDutyPercent(25);
	PWM_SetDutyPercent(50U);
	// PWM_SetDutyPercent(75U);
	// PWM_SetDutyPercent(80U);
		
    while(1)
    {
		loop();
			
    }
}



