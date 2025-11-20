/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>

#include "numicro_8051.h"

#include "detect_pulse.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile OUTPUT_PULSE_MANAGER_T g_OutputPulseManager = 
{
    100U,   /* duty_resolution */
    50U,    /* duty_percent default 50% */
    50U,    /* duty_latched  */
    0U,     /* mode0         */
    0U      /* mode100       */
};

volatile DETECT_PULSE_MANAGER_T g_DetectPulseManager = 
{
    DETECT_STATE_HIGH,/* state */
    0UL,              /* sum_low */
    0U,               /* tick100us */
    0U,               /* low_start_tick */
    0U,               /* duty_start_tick */
    0U,               /* pending_start_tick */
    0U,               /* high_ticks */
    0U,               /* last_low_ticks */
    0U,               /* fixed_low_ticks */
    0xFFFFU,          /* min_low */
    0U,               /* max_low */
    0U,               /* sample_cnt */
    0U,               /* calib_done */
    1U                /* prev_input_state (assume idle HIGH) */

};

#define OUTPUT_PULSE_HIGH							(P15 = 1)
#define OUTPUT_PULSE_LOW							(P15 = 0)

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/



/* local helper */
static unsigned int elapsed_ticks(unsigned int now, unsigned int start)
{
    unsigned int dt;

    dt = (unsigned int)(now - start);
    return dt;
}

void Reset_EINT_calibration(void)
{
    EA = 0;

    g_DetectPulseManager.sum_low            = 0UL;
    g_DetectPulseManager.low_start_tick     = 0U;
    g_DetectPulseManager.duty_start_tick    = 0U;
    g_DetectPulseManager.pending_start_tick = 0U;
    g_DetectPulseManager.high_ticks         = 0U;
    g_DetectPulseManager.last_low_ticks     = 0U;
    g_DetectPulseManager.fixed_low_ticks    = 0U;

    g_DetectPulseManager.min_low            = 0xFFFFU;
    g_DetectPulseManager.max_low            = 0U;
    g_DetectPulseManager.sample_cnt         = 0U;
    g_DetectPulseManager.calib_done         = 0U;
    g_DetectPulseManager.state              = DETECT_STATE_HIGH;

    EA = 1;
}

/* duty setter: clamp 0..100, atomic vs ISR */
void PWM_SetDutyPercent(unsigned int duty_percent_input)
{
    unsigned int d;

    if (duty_percent_input > g_OutputPulseManager.duty_resolution)
    {
        d = g_OutputPulseManager.duty_resolution;
    }
    else
    {
        d = duty_percent_input;
    }

    EA = 0;
    g_OutputPulseManager.duty_percent = d;
    EA = 1;
}

// Put under timer : 100us irq
void output_pulse_irq(void)
{
    unsigned int dt;
    unsigned int now;
    unsigned int dt_low;
    unsigned char curr_input_state;

    g_DetectPulseManager.tick100us++;

    curr_input_state = (P17 == 1) ? 1U : 0U;
    now  = g_DetectPulseManager.tick100us;

    /* 1) handle LOW_PENDING -> LOW_ACTIVE confirmation */
    if (g_DetectPulseManager.state == DETECT_STATE_LOW_PENDING)
    {
        if (curr_input_state == 0U)
        {
            dt = (unsigned int)(now - g_DetectPulseManager.pending_start_tick);

            if (dt >= LOW_CONFIRM_TICKS)
            {
                /* confirmed LOW window start */
                g_DetectPulseManager.state           = DETECT_STATE_LOW_ACTIVE;
                g_DetectPulseManager.low_start_tick  = g_DetectPulseManager.pending_start_tick;
                g_DetectPulseManager.duty_start_tick = now; /* P1.5 will be HIGH from now */

                /* latch duty at window start */
                g_OutputPulseManager.duty_latched = g_OutputPulseManager.duty_percent;
                g_OutputPulseManager.mode0 =
                    (g_OutputPulseManager.duty_latched == 0U) ? 1U : 0U;
                g_OutputPulseManager.mode100 =
                    (g_OutputPulseManager.duty_latched >=
                     g_OutputPulseManager.duty_resolution) ? 1U : 0U;

                if (g_OutputPulseManager.mode0 != 0U)
                {
                    OUTPUT_PULSE_LOW;
                }
                else if (g_OutputPulseManager.mode100 != 0U)
                {
                    OUTPUT_PULSE_HIGH;
                }
                else
                {
                    /* 1..99% duty: compute HIGH length (ticks) */
                    if (g_DetectPulseManager.calib_done != 0U)
                    {
                        dt_low = g_DetectPulseManager.fixed_low_ticks;
                    }
                    else if (g_DetectPulseManager.last_low_ticks != 0U)
                    {
                        dt_low = g_DetectPulseManager.last_low_ticks;
                    }
                    else
                    {
                        /* default LOW window ~5.0 ms = 50 ticks for first few cycles */
                        dt_low = 50U;
                    }

                    g_DetectPulseManager.high_ticks =
                        (unsigned int)(((unsigned long)dt_low *
                                        (unsigned long)g_OutputPulseManager.duty_latched) /
                                       (unsigned long)g_OutputPulseManager.duty_resolution);

                    if (g_DetectPulseManager.high_ticks == 0U)
                    {
                        g_DetectPulseManager.high_ticks = 1U;  /* avoid 0 tick HIGH */
                    }

                    OUTPUT_PULSE_HIGH;
                }
            }
        }
        else
        {
            /* pulse returned HIGH before confirmation -> treat as noise */
            g_DetectPulseManager.state = DETECT_STATE_HIGH;
        }
    }

    /* 2) handle LOW_ACTIVE window (duty + end-of-window) */
    if (g_DetectPulseManager.state == DETECT_STATE_LOW_ACTIVE)
    {
        if (curr_input_state == 0U)
        {
            /* still LOW: handle duty timing */
            if ((g_OutputPulseManager.mode0 == 0U) &&
                (g_OutputPulseManager.mode100 == 0U))
            {
                dt = (unsigned int)(now - g_DetectPulseManager.duty_start_tick);
                if (dt >= g_DetectPulseManager.high_ticks)
                {
                    OUTPUT_PULSE_LOW;
                }
            }
            else
            {
                /* mode0 / mode100 force output */
                if (g_OutputPulseManager.mode0 != 0U)
                {
                    OUTPUT_PULSE_LOW;
                }
                else
                {
                    OUTPUT_PULSE_HIGH;
                }
            }
        }
        else
        {
            /* LOW window ended (rising edge) */
            OUTPUT_PULSE_LOW;
            g_OutputPulseManager.mode0   = 0U;
            g_OutputPulseManager.mode100 = 0U;

            dt_low = (unsigned int)(now - g_DetectPulseManager.low_start_tick);

            if (dt_low >= MIN_LOW_TICKS)
            {
                /* accept as valid LOW window for statistics */
                g_DetectPulseManager.last_low_ticks = dt_low;

                if ((dt_low >= PERIOD_MIN_TICKS) && (dt_low <= PERIOD_MAX_TICKS))
                {
                    if (g_DetectPulseManager.sample_cnt == 0U)
                    {
                        g_DetectPulseManager.min_low = dt_low;
                        g_DetectPulseManager.max_low = dt_low;
                    }

                    g_DetectPulseManager.sum_low += (unsigned long)dt_low;

                    if (dt_low < g_DetectPulseManager.min_low)
                    {
                        g_DetectPulseManager.min_low = dt_low;
                    }
                    if (dt_low > g_DetectPulseManager.max_low)
                    {
                        g_DetectPulseManager.max_low = dt_low;
                    }

                    g_DetectPulseManager.sample_cnt++;
                    if (g_DetectPulseManager.sample_cnt >= DETECT_PULSE_SAMPLES)
                    {
                        g_DetectPulseManager.sum_low -=
                            (unsigned long)g_DetectPulseManager.min_low;
                        g_DetectPulseManager.sum_low -=
                            (unsigned long)g_DetectPulseManager.max_low;

                        g_DetectPulseManager.fixed_low_ticks =
                            (unsigned int)(g_DetectPulseManager.sum_low /
                                           (unsigned long)(DETECT_PULSE_SAMPLES - 2U));

                        g_DetectPulseManager.calib_done = 1U;

                        g_DetectPulseManager.sum_low    = 0UL;
                        g_DetectPulseManager.sample_cnt = 0U;
                        g_DetectPulseManager.min_low    = 0xFFFFU;
                        g_DetectPulseManager.max_low    = 0U;
                    }
                }
            }

            g_DetectPulseManager.state = DETECT_STATE_HIGH;
        }
    }

    g_DetectPulseManager.prev_input_state = curr_input_state;
}

void input_pulse_irq(void)
{
    unsigned int now;

    now = g_DetectPulseManager.tick100us;

    /* only start pending when not already inside a LOW window */
    if ((g_DetectPulseManager.state == DETECT_STATE_HIGH) && (P17 == 0))
    {
        g_DetectPulseManager.pending_start_tick = now;
        g_DetectPulseManager.state = DETECT_STATE_LOW_PENDING;
    }
}

void INT1_ISR(void) interrupt 2          // Vector @  0x03
{
    _push_(SFRS);	
	
    input_pulse_irq();

    clr_TCON_IE1;          //clr int flag wait next falling edge

    _pop_(SFRS);
}

void EINT1_Init(void)
{
    /* INT1 pin P1.7 as Quasi mode with internal pull-high */
    P17_QUASI_MODE;
    P17 = 1;
    INT1_FALLING_EDGE_TRIG;             //setting trig condition level or edge
    set_IE_EX1;                         //INT1_Enable;
    ENABLE_GLOBAL_INTERRUPT;            //Global interrupt enable

    // init P15 as GPIO output    
    P15_PUSHPULL_MODE;
    OUTPUT_PULSE_LOW;

    g_DetectPulseManager.prev_input_state = (P17 == 0) ? 0U : 1U;
    g_DetectPulseManager.state            = DETECT_STATE_HIGH;
}

/* debug helper:
 * print LOW window ticks and an approximate frequency
 * (assuming LOW window = half period and tick≈100us)
 */
void Detect_GetFreq_log(void)
{
    unsigned int Tlow;
    unsigned int approx_freq;

    Tlow = g_DetectPulseManager.fixed_low_ticks;
    
    if (Tlow == 0U)
    {
        printf("Detect: not ready yet\r\n");
    }
    else
    {
        /* approximate: freq ≈ 10000 / (2 * Tlow)
           - 10000 comes from: 1 / (Tlow * 2 * 100us) = 10000 / (2*Tlow)
           - this assumes LOW ≈ half period and 1 tick ≈ 100us */
        approx_freq = (unsigned int)(10000U / (2U * Tlow));
        printf("Detect: Tlow=%u ticks, approx freq=%u Hz (assume 50%% duty, 100us tick)\r\n",
               Tlow,
               approx_freq);    
    }

}
