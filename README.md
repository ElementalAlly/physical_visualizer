# Arduino Stepper Motor-Driven Physical Music Visualizer

# Intro
This is a music visualizer that captures the frequencies of music playing through an audio jack output. It uses an Arduino for FFT processing and motor commands. Stepper motors are geared into a module that moves a bar up and down, according to their corresponding frequency.

A demo of the project is available [here]().

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
 - 2 100K Ohm Resistor ([This pack has enough](https://www.amazon.com/100K-Resistor-Tolerance-Resistors-Resistance/dp/B0B4JDHXC9/).)

# Coding Tutorial

Download [Arduino IDE](https://www.arduino.cc/en/software) and open the physical_visualizer/physical_visualizer.ino file in it. Once everything has been wired, upload it and run it! Your modules should go up and down with the music that's played through the audio jack!

For the Approx_FFT Section, credit goes to abhilash_patel for the algorithm, and Klafyvel for some modifications and writing a blog post to show the algorithm compared to others. Klafyvel's article is here: <https://klafyvel.me/blog/articles/fft-arduino/>, and their repo is here: <https://github.com/Klafyvel/AVR-FFT/tree/main>. The original instructable with this code is here: <https://www.instructables.com/ApproxFFT-Fastest-FFT-Function-for-Arduino/>.
