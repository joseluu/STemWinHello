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

