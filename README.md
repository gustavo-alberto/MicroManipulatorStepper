# Micro Manipulator Stepper
A sub-micrometer 3D motion control plattform.

This project contains an open source low-cost, easy-to-build motorized **XYZ Micro-Manipulator** motion controll platform achieving sub micrometer precision.
It's designed for applications such as optical alignment, probing electronic components, and microscopy.

Thanks to its parallel kinematic structure and miniature ball joints, it achieves good mechanical stiffness and a large range of motion.
The motors are off the shelf stepper motors driven by a 30kHz closed loop controller and a very precise PWM signal.
A 'magnetic gearing' approach increases the resolution of the low-cost megnetic rotary encoders by a factor of 30 allowing for steps down to 50nm.

The device can be controlled via simple G-Code commands over an USB serial interface and is thus easily integrated into other projects.
The firmware implements a complete motion planning stack with look-ahead for smooth and accurate path following capabilities. 

<div style="display: flex; gap: 1%;">
  <img src="images/MicroManipulator.jpg" alt="Image 1" style="flex: 1; object-fit: contain; height: 10vw;">
  <img src="images/ControllerPCB.jpg" alt="Image 2" style="flex: 1; object-fit: contain; height: 10vw;">
</div>

### ⚙ CAD-Files

All CAD models are made in FreeCAD, to allow everyone to view and modify the design without subscribing or paying for a propriatary CAD solution.

<div style="display: flex;">
    <img src="images/FreeCAD-Model.jpg" alt="FreeCAD Model" width="50%">
</div>

<br>

The files for the metric version can be found here: [CAD Models](construction).
Please note that FreeCAD version **1.1.0dev** was used and the files might not work with older versions.

### ⚙ Electronics

### ⚙ Firmware
