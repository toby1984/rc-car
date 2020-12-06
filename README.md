# tvremove-car
This is a fun little arduino project I build for my 3yo where you control a little 2-wheeled robot using a IR tv remote. A couple years ago I got gifted a more-or-less obscure robotics kit (made by Franzis in 2010) that had been sitting idly on a shelf since then. Since St Nichola's day was coming up and I was looking for a small gift for my son, I came up with the idea to revive this kit and use a spare IR TV remote for controlling it.

Since I like to find-out stuff all by myself, I didn't use any ready-made Arduino libraries nor Arduino IDE (but I did use some of the include files) but instead relied on make, avr-gcc and sublime text edit.

![car](https://raw.githubusercontent.com/toby1984/tvremote-car/main/screenshot.png)

Software used

* avr-gcc 
* Make
* Sublime Text 3
* avr-dude
* minicom (for serial debugging)

Hardware used

* ISP programmer
* Franzis robotics kit (no longer sold)
* spare IR sensor, some resistors, 2 blue LEDs (any project can be improved with the addition of more LEDs)
* Oscilloscope (for figuring out what protocol my IR was using and debugging the motor control)

Features:

- controlling of two DGS S04NF servos (continuous rotation) using hardware PWM
- decoding of NEC IR protocol
- blinking LEDs
