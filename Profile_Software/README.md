This folder contains firmware for the Owl Prototypes used in Profile's STEAM classroom. 

For our quickstart guide visit goo.gl/jXzwo4.

To change the firmware on your Owl prototype, download this folder from the command line by using the command:

svn checkout https://github.com/wmsi/owl_dev/trunk/Profile_Software

Then you can choose a firmware version (right now only v1 and v2 are operational) and open it with the Arduino IDE. If you do not yet have the Arduino IDE on your computer, you can download it for free from https://www.arduino.cc/en/main/software. Each firmware version contains comments indicating the purpose of each variable and function. For those unfamiliar with programming microcontrollers, a good starting point may be to change the SENDDELAY constant near the top of the .ino file. This changes the wait time in between transmissions and it will be immediately obvious if your intended change took effect. Then you can move on from here to making more ambitious changes in the firmware. If you mess something up beyond repair never fear! The original firmware will remaine on github for you to download over and over again :)

Both Profile Owl Prototypes are currently loaded with profile_v2.ino. This version builds on profile_v1.ino to introduce a faster send rate and real time averaging of incoming signal strength (in receive mode). These changes are meant to help with radio direction finding (RDF) activities. For applications that focus on datalogging or sensor networks profile_v1.ino may still be a better fit. 


Do you have more questions about programming the Owls? Check out our starter guide at goo.gl/jXzwo4 or email mbucchieri@whitemountainscience.org.

Happy tinkering!