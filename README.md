# RC car 

This project consists of a very crude "RC car" plus a 433 Mhz remote control (both driven by Arduino Nanos).

Since this is a fun project I didn't use any ready-made libraries at all (apart from the AVR libc) and the project builds with old-fashioned 'make' and does not use the Arduino IDE. You'll need a working linux avr-gcc installation and some ISP programmer to upload the firmware.


Parts list 

- some cardboard as base plate of the car (I had to glue together 2-3 layers to make it stiff enough)
- 2x Arduino Nano (~3€/pc)
- 1x 433 Mhz sender module with antenna (PCB has HFY-J1B printed on it, 1.40 €/pc))
- 1x 433 Mhz receiver module with antenna (RXB6 SuperHet, 2.50 €/pc)
- 2x helical 433 Mhz antenna ( 0.50€/pc)
- 2xAA battery holder  (~1€/pc)
- 4xAA battery holder (~1€/pc)
- 2x JGA25-370 6V DC motors, 133 RPM, with built-in encoders ( hard to find with encoders, ~11€/pc)
- 2x wheels, ball caster, 2x shaft coupling, 2x DC motor mounting brackets, screws (I bought a kit that included all of this plus 2 JGA25-370 motors for 15€, didn't use the motors though...)
- 2x 5V Step-Up/Step-Down converter (Pololu S9V11F5 , 4.79€/pc)
- 2-axis joystick module ( 3€/pc )
- DRV8833 module for driving the DC motors (3.00€/pc)

Total cost: A whopping 65€, ouch!

I think you can get this down to around ~40€ (go for cheaper motors without encoders,use no or lower-power step-up/step down converters and find cheap wheels, ball castermshaft couplings+mounting brackets) but given the fact that you can grab RC cars at your local toy store for much less it's obvious that this is never going to compete cost-wise.
