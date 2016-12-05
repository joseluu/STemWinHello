# STemWinHello

Minimal dialog based program using STemWin on a STM32F747-DISCO
compiled under the visualGDB IDE (uses GCC)
This can be the starting point for more complex applications

The main dialog has one button
Pressing the button makes the subdialog appear which also has one button
Pressing the subdialog button closes it

The application can be used to check the proper STemWin configuration
for LCD display and touch sensor

Touch driver is in k_touch.c, it uses the BSP_ts support of the 747-DISCO.
Touch driver is called under interrupt routine from TIM3

The STemWin is a "middleware" handling UI
the Windowing and widgets are similar to the 
traditional Windows API

The makefile and build system expects it to be installed somewhere
on the local disk for instance 
C:\Users\username\STM32Cube\Repository\STM32Cube_FW_F1_V1.4.0\Middlewares\ST\STemWin\

As this is using the GCC toolchain, the library must be
renamed to begin with the 3 letters "lib" as in libSTemWin528_CM3_GCC.a


------------------- Notes for the F103 version

The F103 works with a bare minimum system such as the "Blue pill"
The display is a 2.4" LCD using an ILI9251 controller
connected in parallel as below:
8 bit Data:  PortA, 
CS: PB_7, 
reset: PB_8, 
RS: PB_6, 
WR: PB_5, 
RD: PB_4