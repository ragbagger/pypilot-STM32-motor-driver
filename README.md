# pypilot-STM32-motor-driver
Tinypilot motor driver ported to STM32 from original work with Arduino Nano
   This is based on the original work by:
   Sean D'Epagnier <seandepagnier@gmail.com>
   modified by:
   Timo Birnschein (timo.birnschein@googlemail.com)

   The previous versions were for Arduino Nano boards.

   This version has been rebuilt into an STM32CubeIDE project to run on an
   STM32F103C8 such as found on the popular "blue pill". All Nano specific code
   has been removed. The original communication with Pypilot from Sean has been retained.
   Timo followed on with work to use an off the shelf IBT-2 motor controller. This code
   has been simplified to ONLY apply to the IBT-2 type controller.
   I have also retained Timo's work on using exponential filters on analog inputs.

   This is currently aimed at a boat with hydraulic steering so no clutch has been
   implemented. It would be easy to add.

   The pinout for the STM32 is as follows:

   Motor controller---STM32

   
        RPWM          PB3
        LPWM          PA15
        R_EN          PB12
        L_EN          PB13
        R_IS          See notes on current sensing
        L_IS          See notes on current sensing
        VCC           3.3
        Gnd           G

        RPi-----------STM32
        Tx            PA10
        Rx            PA9
        Gnd           G
        3.3V          3.3

Analog Sensors- All are 0.0-3.3V analog signals

        Voltage                  PA2
        Current                  PA1
        Rudder Position          PA0
        Motor Temperature        PA3
        Controller Temperature   PA4

In the current version The analog inputs are all read and filtered but only Rudder
Position is used for control. Other inputs will be implemented later

Notes on current sensing:
The IS pins on the IBT-2 boards are a current source based on a linear ratio to the load current.
This current source needs to be converted to voltage by a resistor to ground. Furthermore each IS
pin is only active for current in one direction. The IBT-2 board supplies a resistor to ground
for each IS pin but it will not give a 0-3.3V signal. By wiring the two IS pins together and
supplying an additional resister to ground the signal can be conditioned. The resistor to ground
is dependant on the motor voltage being used and which driver chip is on the IBT-2. I have
seen two different chips used. I will provide more on this when I implement it in this code.
