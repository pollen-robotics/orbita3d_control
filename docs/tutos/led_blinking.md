---
title: Debug LED blinking
layout: default
---

## LED blinking patterns

The blinking of the LED on the board is used to indicate the state of the board. There are two colors of the LED - green and red. The LED can be solid or blinking. The LED is blinking with a period of 500ms. The pattern of blinking is as follows:


 state           | description | CiA402 state | green         | red
 ----------------|--------------|---------------|---------|---------
 init            | initialisaiotn in process | `NotReadyToSwitchOn`  | blinks        | blinks
 preop           | waiting to connect to the master to start the operation|`SwitchOnDisabled`,`ReadyToSwitchOn`,`SwitchedOn` | solid         | off
 preop  + warning |  + a warning |`SwitchOnDisabled`,`ReadyToSwitchOn`,`SwitchedOn`| solid         | blinks
 op               | connected to the master and oprational |`OperationEnabled`| solid         | off
 op  + warning    |  + a warning |`OperationEnabled`| solid         | blinks
 fault            | error state - actuator turned off ( not recoverable ) |`Fault`| off           | solid
 fault_reaction   | turning off the actuator on error |`FaultReactionActive`| off           | blinks
 quick_stop_reaction   | turning off the actuator after the emergency stop actiavated |`QuickStopActive`| solid           | solid

## Example of LED led states


 <img src="../../img/blink1.jpg" alt="LED blinking pattern" width="300"/> | <img src="../../img/blink2.jpg" alt="LED blinking pattern" width="300"/>
    ---|---
    Operational state | Quick (emergency) stop reaction state
    Green LED is ON | Both LEDs are ON