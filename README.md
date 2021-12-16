# Electronic-control-of-MCC
Handling control with STM32 of motor
## Shell definition

## Setting PWM 
We need frequency of PWM <= 16 kHz with a resolution of 10 bits. For that we set the ARR to 1023 and PSC to 10. We set DEADTIMES TO 203, to realise a 2 us break between the complementary channels. We use TIM1 CH1 and CH2 to realise the PWM generation in complementary channel.

