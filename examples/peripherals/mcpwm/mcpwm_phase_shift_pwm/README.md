| Supported Targets | ESP32 |
| ----------------- | ----- |

# MCPWM Phase-Shift PWM example

This example will show you how to use MCPWM module to generate a Phase-Shift PWM signal between
two pairs of hardware pins, e.g. for controlling a phase-shifted full-bridge power converter.
 
This example sets frequency, duty cycle and dead-time values for leading and
lagging half-bridge leg to fixed values.

MCPWM unit can be [0,1]
 

## Preparation
* Copy this examples folder to an empty working directory
* Copy (or clone) esp32_ps_pwm into the components/ subfolder


## Pin assignment
* PWM outputs  
  GPIO_NUM_27 // PWM0A output for LEAD leg, Low Side  
  GPIO_NUM_26 // PWM0B output for LEAD leg, High Side  
  GPIO_NUM_25 // PWM1A output for LAG leg, Low Side  
  GPIO_NUM_33 // PWM1B output for LAG leg, High Side  
* Optional, shutdown/fault input for PWM outputs: disables output when pulled low  
  GPIO_NUM_4 // hardware shutdown input signal for PWM output

## Operation
* On the four output pins, you will see two complementary, phase-shifted PWM waveforms
* When PWM outputs are connected to a full-bridge circuit using an appropriate driver,  
  using a differential voltage probe, you will see a phase-shifted PWM waveform.

```
                            VDD
                     .---------------.
                     |               |
                  ||-+               +-||
      To  GPIO 26 ||<-               ->||To  GPIO 33
      ------------||-+               +-||-----------
                     |               |
                     |     LOAD      |
                     |      ___      |
  LEAD half-bridge   o-----|___|-----o   LAG half-bridge
                     |               |
                     |               |
                     |               |
                  ||-+               +-||
      To  GPIO 27 ||<-               ->||To  GPIO 25
      ------------||-+               +-||-----------
                     |               |
                     '---------------'
                            GND
```
