#ifndef __DETECT_PULSE_H__
#define __DETECT_PULSE_H__

/*_____ I N C L U D E S ____________________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*  
	template
	typedef struct _peripheral_manager_t
	{
		uint16_t* pu16Far;
		uint8_t u8Cmd;
		uint8_t au8Buf[33];
		uint8_t u8RecCnt;
		uint8_t bByPass;
	}PERIPHERAL_MANAGER_T;

	volatile PERIPHERAL_MANAGER_T g_PeripheralManager = 
	{
		.pu16Far = NULL,	//.pu16Far = 0	
		.u8Cmd = 0,
		.au8Buf = {0},		//.au8Buf = {100U, 200U},
		.u8RecCnt = 0,
		.bByPass = FALSE,
	};
	extern volatile PERIPHERAL_MANAGER_T g_PeripheralManager;
*/

typedef enum {
    DETECT_STATE_HIGH = 0,             /* waiting for next LOW window */
    DETECT_STATE_LOW_PENDING,          /* saw falling edge, waiting for confirmation */
    DETECT_STATE_LOW_ACTIVE            /* confirmed LOW window in progress */
} DETECT_STATE_T;

typedef struct _output_pulse_manager_t
{
    unsigned int duty_resolution;   /* e.g. 100 => 0..100 % */
    unsigned int duty_percent;      /* current target duty (0..resolution) */
    unsigned int duty_latched;      /* duty value latched at window start */
    unsigned char mode0;            /* 1: force 0% for whole window */
    unsigned char mode100;          /* 1: force 100% for whole window */
	
}OUTPUT_PULSE_MANAGER_T;

typedef struct _detect_pulse_manager_t
{
    DETECT_STATE_T state;	        /* current state of detect pulse window detection */
    unsigned long sum_low;          /* accumulate LOW widths for calibration */
    unsigned int tick100us;         /* global 100us tick counter */

    unsigned int low_start_tick;    /* confirmed LOW start (for period and freq) */
    unsigned int duty_start_tick;   /* P1.5 went HIGH at this tick */
    unsigned int pending_start_tick;/* tick value when LOW_PENDING started */

    unsigned int high_ticks;        /* HIGH duration (in ticks) for 1..99% duty */
    unsigned int last_low_ticks;    /* last valid LOW width */
    unsigned int fixed_low_ticks;   /* averaged LOW width after calibration */

    unsigned int min_low;           /* min for min/max rejection */
    unsigned int max_low;           /* max for min/max rejection */
    unsigned char sample_cnt;       /* number of LOW windows collected */
    unsigned char calib_done;       /* 1: fixed_low_ticks valid */

    unsigned char prev_input_state; /* last sampled detect pulse value (0/1) */
}DETECT_PULSE_MANAGER_T;

#define DETECT_PULSE_SAMPLES   		(10U)  /* number of LOW windows for calibration */
#define LOW_CONFIRM_TICKS         	(1U) 
#define MIN_LOW_TICKS       		(5U)   /* 5 * 100us = 500us , if lower than 500us , regard as noise */

/* expect 100Hz / 120Hz，LOW window 4~6 ms (40~60 ticks) */
#define PERIOD_MIN_TICKS       		(40U)  /* 4.0 ms  =  40 ticks */
#define PERIOD_MAX_TICKS       		(80U)  /* 8.0 ms  =  80 ticks */
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

/* reset LOW-window calibration (re-measure Tlow when power-on or needed) */
void Reset_EINT_calibration(void);

/* set duty in percent (0..100); safe to call in main loop */
void PWM_SetDutyPercent(unsigned int duty_percent_input);

/* called from Timer1 100us ISR */
void output_pulse_irq(void);

/* called from INT1 ISR (falling edge on P1.7) */
void input_pulse_irq(void);

/* configure INT1 (P1.7) and P1.5 output */
void EINT1_Init(void);

/* return frequency*100 (e.g. 5000 = 50.00 Hz) using fixed_low_ticks when ready */
void Detect_GetFreq_log(void);

#endif //__DETECT_PULSE_H__
