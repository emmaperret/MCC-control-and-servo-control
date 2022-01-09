# MCC-control-and-servo-control
Handling control with Nucleo-G431 of motor.

## Shell definition
As we have already done for other projects, we have created a shell with information and control functions for the motor.
These functions are: 
- Help to display all functions and their commands.
- Pinout to display active pinouts.
- Timer to display active timers.
- Go-Start to turns on the motor power stage.
- Stop to turns off the motor power stage.
- Speed to controls the speed of the motor.

## Setting PWM 
We need frequency of PWM <= 16 kHz with a resolution of 10 bits. For that we set the ARR to 1023 and PSC to 10. We set DEADTIMES TO 203, to realise a 2 us break between the complementary channels. We use TIM1_CH1 and TIM1_CH2 to realise the PWM generation in complementary channel.

## Start and speed command
### Start command
To turn on the motor power stage, we need to initialize the PWM command in the GPIO by resetting the sequence and then we need to start the timers that control the PWM signals.

### Speed command
We know the maximal CCR is 1024. By choosing a CCR of 512, the duty cycle is set to 50% which is the stop position of the motor. If a CCR between 512 and 1024 is chosen, the motor will run in one direction. If a CCR between 0 and 512 is chosen, it will rotate in the opposite direction.<br />
All we need to do is create a command that takes a number between 0 and 1024 as an argument and set the CCR of channel 1 of the timer to that value and channel 2 to 1024 minus the requested value.

## Speed and current measurement
The aim of this part is to determine the different parameters of the motor in order to control it.

### Speed measurement
To measure the motor speed, the encoder must be used. The Z output of the encoder is 1 when the motor has made one revolution and 0 the rest of the time. With this command and the measurement of the time between two transitions from state 0 to state 1, we can deduce the motor speed.

### Current measurement
To measure the current, we are interested in the chopper pins that return the yellow phase hall current sensor feedback and the red phase hall current sensor feedback (pins 16 and 35). We also need an ADC.


## Servo control
We have to make two servo-controls: one in speed and one in current.
### Speed feedback
To control the speed of the motor, it is necessary to be able to measure the speed in real time and adjust it so that it tends towards the set value. This is done using the equations of the MCC and the constants found in the measurement part.
