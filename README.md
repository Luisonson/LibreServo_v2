# <img src="https://www.libreservo.com/sites/libreservo.com/files/imagenes/LibreServo_logo_xs.png">
An Open source controller to convert any servo motor to the best smart servo.

This project was born as a necessity of mine to build a biped robot with intelligent servos to be able to “feel the muscles” to walk more human like. In the past we had OpenServo, and I think this project inherits something from it, but OpenServo died many years ago and it didn't reach my expectations anyway (power, communications...), and the commercial alternatives are way too expensive (Dynamixel, Herkulex, Lynxmotion...).

My goal with LibreServo is to make any standard servo the “smartest” one in the market. The "gold standard" nowadays is Robotis-Dynamixel, LibreServo should be better than that, that’s my goal at least.

A few characteristics of LibreServo:

    Compatible with standard servo motors (No need to change the bottom cover of them!)
    Voltage: From 4.5V up to 18V (Recommended: 5-14V)
    Communications: RS-485 half-duplex. Max Speed 9Mbps. Daisy chained. CRC-16
    Amp: Up to 16A continuous (WSD3069DN56) (Version >2.3)
    Micro-Controller: STM32F301k8 (cortex-M4@72MHz)
    Position sensor: Magnetic encoder, 16 bits of resolution! 360 degrees (AEAT-8800). Using the servo motor potentiometer will be possible to lower the cost but will lost precision and some characteristics.
    For the encoder I have designed 3D parts to substitute the potentiometer and used the same hole/space than the original.
    LibreServo will generate their own curves (sine ramps, trapezoidal ramps, hermitian curves...)
    Current sensor: +-15A ACS711
Communication Protocol: <a href="https://www.libreservo.com/en/articulo/libreservo-commands-part-one">LibreServo Commands</a><BR>
<img src="https://www.libreservo.com/sites/libreservo.com/files/imagenes/Main-Encoder-PCB.jpg" width="550" height="412">


More info in <a href="https://www.libreservo.com/en">LibreServo</a>.

<p align="center">
<a href="http://creativecommons.org/licenses/by-sa/4.0/"><img src="https://user-images.githubusercontent.com/12425566/219942224-9a3bc76d-cd19-4feb-99f2-25e79919dd3e.png" alt="cc-by-sa" width="200px" height="69px"></a></p>
