# Introduction

PICTURE OF THE VENTILATOR IN SITU

We were motivated by the work of [Al Husseini et al.](https://www.researchgate.net/publication/245374049_Design_and_Prototyping_of_a_Low-Cost_Portable_Mechanical_Ventilator) who proposed a machine to regularly compress a bag-valve mask (BVM).  This design leverages widely available commodity medical equipment while requiring only a relatively simple machine to compress the bag by a settable amount on a settable time schedule. We compress the bag by pushing it on just one side using an arm which rotates about a fixed pivot point.  The arm is driven by a computer controlled motor. This enables the machine to be very simple with just two moving parts: the motor shaft and the arm.  The motion of the arm would require the bag to slide across the surface of the arm, however the high friction of the bag's surface means that sliding does not occur but it results in increased motor torque as well as some wear and tear of the bag.  While this is not ideal, in practice the bags have lasted for ??? hours.

Our design aims were:
 * For the machine to be manufacturable quickly and cheaply with as few components as possible. Components had to be easily sourced or substituted.  Our second design can be from a sheet of 12mm Nylon or other engineering plastic using a water jet or CNC router. The mechanical parts can be cut in a single operation and assembled it in less than 3 hours.
 * To accommodate a wide variety of electric motors. We used a stepper mode but other types of motors could be used. Brushed DC motors, for example, are widely used for windscreen wiper motors and could be easily substituted.
 * To use low-end commodity computing hardware and sensors. We used an Arduino Mega with a simple display panel and low-cost potentiometers.

We built our first machine in one week using 3D printed components, a high-power stepper motor that we had to hand, an Arduino microcontroller, an TFT display, three potentiometers to control its operation using parameters familiar to clinicians and one potentiometer for motor feedback. The machine is unashamedly bare bones and includes no instrumentation for flow or for pressure.  

# Control software

The software allows control of three ventilation parameters: breaths per minute (BPM), inspiration/expiration ratio (IE), and breath volume.  We implement an inspiration time but this is fixed in the program and not currently adjustable by the clinician. We control the pusher arm to rotate at constant velocity during the inspiration time upto a maximum stroke which is a function of the required breath volume.  The arm is held for a constant time, and then retracted quickly allowing the bag to reinflate by itself.

PICTURE OF THE LCD DISPLAY

The control parameters are set by three potentiometers connected to analog inputs of the microcontroller.  They are digitized, rescaled and displayed continuously on the LCD panel. At the start of a cycle a snapshot of these parameters is made and used to compute the inspiration time, expiration time and the maximum stroke.  The controller then drives the pusher at the required speed for the required time, holds, and then retracts.

The inclusion of an inspiration hold time means that for some settings of IE and BPM the inspiration time can become negative.  The firmware detects this and displays a warning on the LCD panel.  If this condition exists at the start of the cycle those parameters are ignored and those for the previous cycle continue to be used.

The relationship between maximum rotation of the arm and supplied volume was calibrated for five points and incorporated into a lookup table in the control firmware.

An additional potentiometer on the shaft of the pusher arm provides an absolute and continuous measurement of the rotation angle of the pusher arm.  We use a stepper motor but for large volume and high PEEP we observe that the motor may lose steps so we cannot rely simply on step counting.


# References
Husseini, Abdul & Lee, Heon Ju & Negrete, Justin & Powelson, Stephen & Servi, Amelia & Slocum, Alexander & Saukkonen, Jussi. (2010). Design and Prototyping of a Low-Cost Portable Mechanical Ventilator. Journal of Medical Devices-transactions of The Asme. 4. 10.1115/1.3442790. 
