# MCC-control-and-servo-control
Handling electronic control with Nucleo-G431 of motor.

## Shell definition
As we have already done for other projects, we have created a shell with information and control functions for the motor.
These functions are: 
- Help to display all functions and their commands.
- Pinout to display active pinouts.
- Timer to display active timers.
- Go-Start to turns on the motor power stage.
- Stop to turns off the motor power stage.
- Speed to controls the duty cycle of the PWM driving the motor.
- Pid_auto to allow PID speed control.

## Setting PWM 
We need frequency of PWM <= 16 kHz with a resolution of 10 bits. For that we set the ARR to 1023 and PSC to 10. We set DEADTIMES TO 203, to realise a 2 us break between the complementary channels. We use TIM1_CH1 and TIM1_CH2 to realise the PWM generation in complementary channel.

## Start and speed command
### Start command
To turn on the motor power stage, we need to set the ResetPin low of the H full-bridge to allow us to sent signals to the motor driver.Afterwards we have to start the timers which are controlling the PWM signals.

### Speed command
We know the maximal CCR is 1024. By choosing a CCR of 512, the duty cycle is set to 50% which is the stop position of the motor. If a CCR between 512 and 1024 is chosen, the motor will run in one direction. If a CCR between 0 and 512 is chosen, it will rotate in the opposite direction.<br />
All we need to do is create a command that takes a number between 0 and 1024 as an argument and set the CCR of channel 1 of the timer to that value and channel 2 to 1024 minus the requested value.

## Speed and current measurement
The aim of this part is to determine the different parameters of the motor in order to control it.

### Speed measurement
To measure the motor speed, the encoder must be used. The Z output of the encoder is 1 when the motor has made one revolution and 0 the rest of the time. With this command and the measurement of the time between two transitions from state 0 to state 1, we can deduce the motor speed. We will use Z, A and B to determine the pulses per revolution. This will serve to normalize the value of the speed in the PID calculus.

### Current measurement
To measure the current, we are interested in the chopper pins that return the yellow phase hall current sensor feedback and the red phase hall current sensor feedback (pins 16 and 35). We also need an ADC.


## Servo control
We have to make two servo-controls: one in speed and one in current.
### Speed servo control
To control the motor in speed. We will memorize the positions data of the encoder and use them to calculate the speed with the pulses per revolution value. What needs to be done is that we are comparing the actual speed value to the targeted one. We are multipling the error by a proportionnal gain **KP**, adding a integral constant and derivative one (**KI** and **KD**). All of those values could not havec been determined in proper working conditions, therefore they may not be optimised. The memory of the system is ensured thanks to buffers and reused to calculate the speed to n+2 values.
The working principle of this control has been tested on a much smaller motor. It may be possible that a great amount of constants are not correct.
Once the PID has given us a value of speed to be sent, we need to convert this speed to a duty cycle. We used the fact that the 100% duty cycle was leading to 3000 rpm, and that 50% leads to 0 rpm. The PID is probably not fitted to go in negative rpm values, we did not have time to adapt to this type of use.
### Current pseudo servo control
The working principle is simple, we have used a  TIMER to realise interruptions every 100ms. At each interruption the values of the TIM1->CCR1 and TIM2->CCR2 are going up and down of one unit. This principle assure us that the motor will not be subject to important overcurrent situations. Eventhough we managed (as usual) to pick the worst motor of those available and getting overcurrent to each command with this principle almost.

## Conclusion
The servo control principle was realised on another scale and was working fine, the transposition for the motor is made almost blindly. The principle is probably correct. We realiseed in this course that float are sometimes an issue since it uses a great amount of memomry. We have achieved the speed electronic control but due to a lack of luck mostly we could not realise the current servo control. If we did, we should have been watching closely for the sampling frequencies between speed and current. If the current is sampled slower than the speed, the result is quite bad in simulation.
