--Neon Cave Lights
By Charlotte Roux
--Readme
Requirements to run:
--Running in VR
Access to Macalester VR Cave. Including 3D glasses, head tracking apparatus and Aimon joystick, also with tracking apparatus attached. A correct configuration command is also needed to make sure that the project runs on all screens.
Code:-c mac-cave-multithreaded

--Running on Desktop
MinVr must be installed on your desktop computer along with the basic graphics package. OpenGL must also be included when attempting to run this code.
Configuration code for desktop: -c desktop

Steps To Run in VR Cave:
Turn on the cave via the power-on.sh file on the desktop. Ensure that the motion tracking software is running on the desktop computer as well as make sure that the aimon joystick server is up and running. Check that the 3D glasses are charged and that the joystick is able to power on. It helps to also make sure the server is connected to the joystick. Sometimes it may require a reset of the aimon joystick server in order to correct effectively. Also, when running in the cave, the program will occasionally not display until you press a key on the desktop. If the code for some reason does not execute as expected, try pressing a key on the desktop to see if that will activate the display.

--User Controls:
Aimon Joystick:
Press Up on the joystick D-pad in order to change the color to white.
Press Down on the joystick D-pad in order to change the color to Red. 
Press Left on the joystick D-pad in order to change the color to Blue.
Press Right on the joystick D-pad in order to change the color to Green.
Press the lower trigger button on the joystick in order to draw a line.
Press the upper trigger button on the joystick in order to activate the stroke aggregation function.

--Tips for stroke aggregation function.
Upon pressing the button to activate this feature, the program will work to aggregate all lines of the color that is currently selected by the user. You can determine this by checking what color the ball pointer attached to the joystick in VR is. When using this function, it is also recommended that you lean into its best use case. This means that to see good results, it is best if you draw a number of similar lines in the same color in a close proximity to one another. It is also very important to draw the lines in the same direction as the averaging function goes based on the inherent order of the vertices.

Code Architecture:
This code is entirely written in the App.cpp and the App.h files. There has not been any modifications to object structure. There has been the addition of a new method for line aggregation. Also, when looking at other work (IE a botched attempt at spline integration) there has been a number of modifications to the Catmull Rom Spline.H file in order to try to get it to run successfully.

Performance Issues:
After drawing a number of lines the program begins to slow down considerably, given the lines are drawn based on the framerate of the system, this can become a problem. In order to fix this currently, one should hit the spacebar key on the computer in order to clear all lines and colors from the display. This will improve performance and allow you to draw in a much more comfortable way. In the future, figuring out how to clear the _Linearray of redundant lines after aggregation should do a great deal to improve performance. With some restructuring, it should also be possible to not have to reset them based on the eye. Given the current nature of the program and how the _Line and _Color arrays maintain synchronisation with each other, it is currently simpler to keep it where it is. 

