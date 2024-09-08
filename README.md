# Arduino Stepper Motor-Driven Physical Music Visualizer

# Intro
This is a music visualizer that captures the frequencies of music playing through an audio jack output. It uses an Arduino for FFT processing and motor commands. Stepper motors are geared into a module that moves a bar up and down, according to their corresponding frequency.

A demo of the project is available [here](https://www.youtube.com/watch?v=0NpDhRlfrKM).

# Materials
 - Access to a 3D printer (I used an [Ender 3 Neo](https://www.creality.com/products/ender-3-neo-3d-printer), but other precise 3D printers should print fine.)
 - One Arduino MEGA R3 Board (I used [this one](https://www.amazon.com/ELEGOO-Compatible-Arduino-Projects-Compliant/dp/B01H4ZLZLQ))
 - One Breadboard (I used [this hat](https://www.amazon.com/HiLetgo-Prototype-Expansion-Breadboard-ProtoShield/dp/B00HHYBWPO/) to make it more compact, but a separate breadboard will work as well.)
 - 7 28BYJ-48 Motors ([This pack](https://www.amazon.com/28BYJ-48-ULN2003-Stepper-Driver-Arduino/dp/B07YRHX73L/) is cheap for 6. You will have to buy two to get enough for the visualizer.)
 - 7 4mm x 10mm x 4mm Bearings ([This pack](https://www.amazon.com/uxcell-MR104ZZ-Groove-Bearings-Shielded/dp/B082PSQZMX/) is good.)
 - 7 Micro Limit Switches (Get [this pack](https://www.amazon.com/HiLetgo-KW12-3-Roller-Switch-Normally/dp/B07X142VGC/).)
 - Soldering Iron and Solder ([This one](https://www.amazon.com/Soldering-Interchangeable-Adjustable-Temperature-Enthusiast/dp/B087767KNW/) should be fine.)
 - Multimeter (For testing connections. [This one](https://www.amazon.com/AstroAI-Digital-Multimeter-Voltage-Tester/dp/B01ISAMUA6/) would be fine.)
 - Wire Cutter and Stripper ([This one](https://www.amazon.com/WGGE-Professional-crimping-Multi-Tool-Multi-Function/dp/B073YG65N2/) is fine.)
 - 22 Gauge Solid-core Wire ([This pack](https://www.amazon.com/FIRMERST-Gauge-Solid-Tinned-Copper/dp/B0CCJ5YT3V/) should be enough.)
 - 1 Micro USB Power Adapter (I used [this one](https://www.amazon.com/WMYCONGCONG-Interface-Adapter-2-54mm-Breakout/dp/B082PDD79D/).)
 - 3.5 mm Audio Jack (I used [this one](https://www.amazon.com/Monoprice-3-5mm-Stereo-Plug-Cable/dp/B003NN1XZM/).)
 - 0.22 Micro-farad Capacitor ([This one](https://www.amazon.com/10pcs-0-22uF-Metallized-Polyester-Capacitors/dp/B00TX42OMQ/) should be fine.)
 - 2 100K Ohm Resistor ([This pack](https://www.amazon.com/100K-Resistor-Tolerance-Resistors-Resistance/dp/B0B4JDHXC9/) has enough.)

# Module Construction
Print out at least 7 of each component in 3DModels/basic_module. (I would recommend some spares of the two interface gears and the motor case. These snap somewhat easily.)

Put each of them together as shown in [this guide](https://github.com/ElementalAlly/physical_visualizer/raw/main/docs/moduleBuildingGuide.pdf).

# Wiring Tutorial
There are a few sections of wiring: Module Power, Module signals, and Input Reading.

## Input Reading
This circuit is the most complex out of all of them. While the rest power or signal components where the circuitry is already made, this is a circuit you will have to build yourself. The circuit looks like this:

![Input circuit, check link below if it doesn't load](https://github.com/ElementalAlly/physical_visualizer/raw/main/docs/InputCircuit.png)

If the image doesn't load, check [this link](https://forum.arduino.cc/t/how-to-read-data-from-audio-jack/458301/3), and exclude R1.

This shifts the input from centering at 0 volts to centering at 2.5 volts, which allows the Arduino to read the entire input from its analog pins.

We use the left audio signal because it's most commonly used as mono.

The best way to get the audio signal and ground are to cut the audio extension cable in half and solder the two halves facing the same direction up on a prototype board, like the one seen on the end of the arduino hat I recommended. Then, solder the matching connections back together, and solder another two wires, one attaching to ground and the other attaching to the left signal. These can then be wired in the way of the diagram, and audio signal can be both received from a computer, and duplicated onto speakers with an audio jack.

From there, use the components from the materials to build the circuit!

## Module Power
This contains two major power circuits we need to take into account: Motor power and Limit Switch Power. There are two power sources for the entire system because 7 motors draw too much power and can disable the arduino accidentally. This is where the micro-USB power module comes into practice.

Power each motor using the Micro USB power adapter, as shown in the diagram. Use the same power source for all motors.

![Motor is powered through the Micro USB Adapter](https://github.com/ElementalAlly/physical_visualizer/raw/main/docs/MotorPower.png)

Connect each limit switch's ground to the ground of the arduino, as shown in this diagram:

![Limit switch ground is connected to the Arduino Ground](https://github.com/ElementalAlly/physical_visualizer/raw/main/docs/LimitSwitchPower.png)

## Module Signals
When a motor is declared like this:

```
stepper stepper1(a, b, c, d, e)
```

Wire your motor and limit switch signal of module 1 like this:

![a to IN1, b to IN2, c to IN3, d to IN4, e to Limit Switch Signal](https://github.com/ElementalAlly/physical_visualizer/raw/main/docs/MotorSignal.png)

For example, when you encounter:

```
stepper(7, 6, 5, 4, 3)
```

Wire IN1 to 7, IN2 to 6, IN3 to 5, IN4 to 4, and the Limit Switch Signal to 3.

All the motors' declarations are these:

```
stepper stepper1(7, 6, 5, 4, 3);
stepper stepper2(22, 24, 26, 28, 30);
stepper stepper3(23, 25, 27, 29, 31);
stepper stepper4(32, 34, 36, 38, 40);
stepper stepper5(33, 35, 37, 39, 41);
stepper stepper6(42, 44, 46, 48, 50);
stepper stepper7(43, 45, 47, 49, 51);
```

# Coding Tutorial
Download [Arduino IDE](https://www.arduino.cc/en/software) and open the physical_visualizer/physical_visualizer.ino file in it. Once everything has been wired, upload it and run it! Your modules should go up and down with the music that's played through the audio jack!

For the Approx_FFT Section, credit goes to abhilash_patel for the algorithm, and Klafyvel for some modifications and writing a blog post to show the algorithm compared to others. abhilash's original instructable with this code is here: <https://www.instructables.com/ApproxFFT-Fastest-FFT-Function-for-Arduino/>. Klafyvel's article is here: <https://klafyvel.me/blog/articles/fft-arduino/>, and their repo is here: <https://github.com/Klafyvel/AVR-FFT/tree/main>.

The sampling frequency, the number of samples per FFT, and the way the code groups the frequency buckets, make the individual bars react to approximately these following frequencies:

```
stepper1: 60 Hz -> 120 Hz
stepper2: 120 Hz -> 250 Hz
stepper3: 250 Hz -> 500 Hz
stepper4: 500 Hz -> 1000 Hz
stepper5: 1000 Hz -> 2000 Hz
stepper6: 2000 Hz -> 4000 Hz
stepper7: 4000 Hz -> 8000 Hz
```

This is limited to 8000 Hz because of the arduino platform having a cpu that isn't fast enough to sample at a fast enough rate to detect higher frequencies.
