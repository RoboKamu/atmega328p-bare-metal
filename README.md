# ATMEGA328P HAL (WIP)
This project is a work-in-progress. Meant to continously add HAL/driver libraries for the ATMEGA328P when needed while working on projects that need the relevant features. A add-upon-relevant library. 

I use this as a learning objective and only use the avr/io.h file for pin definitions, minimal usage of AVR-libC functions may be present in the early iterations. The late stage goal is to minimize dependency on AVR-libC as well when I implement my own interrupt handling. I write everything myself while also explaining the code and forcing myself to document the datasheet usage and underlying theory of *why* it works. 

Find Guides explaining *why* the code works in "detail" in the Guides/ folder with Markdown files.  

The code for the microcontroller is flashed using AVRDUDE, compiled with GCC (avr-GCC), and using arduino UNO REV3 board as programmer. Development is done using toolchain + Makefile, no IDE except for VSCODE as a text editor. 
