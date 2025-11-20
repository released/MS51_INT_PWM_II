# MS51_INT_PWM_II
MS51_INT_PWM_II

update @ 2025/11/20

1. base on https://github.com/released/MS51_INT_PWM , 

add filter noise flow , 

when trig stae : DETECT_STATE_LOW_PENDING under input_pulse_irq in EINT irq

base on output_pulse_irq in 100us timer irq , will determine noise base on LOW_CONFIRM_TICKS , 

to decide P15 will be low or continue high

