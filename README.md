# DASA Framework

The latest version of this tool can be found [here](https://github.com/sajid-mohamed/cppVrepLKAS).

If you are interested in more research along this direction: [look here](http://www.es.ele.tue.nl/~dgoswami/test/).

If you have any questions contact: s.mohamed@tue.nl

# What is DASA?

DASA is an open-source design, analysis, and simulation framework for automotive IBC systems that can consider the change in vehicle dynamics in real-time and produces real-time dynamic image stream as per the control algorithm. Our framework models the 3D environment in 3ds Max, simulates the vehicle dynamics, camera position, environment and traffic in V-REP and computes the control output in Matlab. Our framework runs Matlab as a server and V-REP as a client in synchronous mode.
Effectively, this is a software-in-the-loop simulator using Vrep and Matlab.
This work was presented in:
```
S. Mohamed, D. Zhu, D. Goswami, T. Basten, "DASA: an open-source design, analysis and simulation framework for automotive image-based control systems," in: MCAA Annual Conference, 2019.
```
-------------------------------------------------------------------------------------------------------------------------------------------------------

# How do I install it?

DASA consists of 3 tools:
1. 3ds Max
	Install this only if you want to model the 3D environment not already included in the release.
	You can download it from https://www.autodesk.eu/products/3ds-max/overview
2. V-REP
	You should download it from http://www.coppeliarobotics.com/downloads.html.
	Follow their instructions to install it.
	
3. Matlab
	You should download it from https://nl.mathworks.com/products/matlab.html.
	Follow their instructions to install it.	

-------------------------------------------------------------------------------------------------------------------------------------------------------

# Which software dependencies does DASA have?

DASA needs openCV library to be installed in your system.
The current framework uses "mexopencv". You can install it from
https://github.com/kyamagu/mexopencv. 
The path to the installation should be added to Matlab path.

-------------------------------------------------------------------------------------------------------------------------------------------------------

# Is there any documentation for DASA?

Documentation on the framework can be found in `DASA Tutorial.pdf`. 
This website provides also a command-line reference guide and documentation of
the API.

-------------------------------------------------------------------------------------------------------------------------------------------------------

# AUTHOR CONTACT INFORMATION:
   Sajid Mohamed <s.mohamed@tue.nl>
   
 # License
 This work is licensed under the Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-nd/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.


