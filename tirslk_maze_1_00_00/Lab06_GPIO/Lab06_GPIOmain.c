// Lab06_GPIOmain.c
// Runs on MSP432
// Solution to GPIO lab
// Daniel and Jonathan Valvano
// May 21, 2017
// Provide test main program for QTR-8RC reflectance sensor array
// Pololu part number 961.

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// reflectance LED illuminate connected to P5.3
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp.h"
#include "../inc/Reflectance.h"
#include "../inc/Clock.h"

uint8_t Data; // QTR-8RC

void Debug_LED_Init() {
    //P2.5 is for driving an oscilloscope to reflect either P5.3 or P7.3
    //P2.0-2.2 will allow viewing the position...
    P2->SEL0 &= ~0x27;
    P2->SEL1 &= ~0x27;
    P2->OUT &= ~0x27;
    P2->DIR |= 0x27;
    P2->DS |= 0x07;
}

// Test main for part1
// 1. Initialize the robot
// 2. Every 10mS, read the line sensor and store the result in Data
//    - if b7 or b6 of Data are set, turn on LED2.BLUE
//    - if b5, b4, b3, or b2 of Data are set, turn on LED2.GREEN
//    - if b1 or b0 of Data are set, turn on LED2.RED
int main(void){
  Clock_Init48MHz();
  Debug_LED_Init();
  Reflectance_Init(); // your initialization
  while(1){
    Data = Reflectance_Read(1000); // your measurement
    // turn on LED2.RGB as described in the comments
    // write this code


    Clock_Delay1ms(10);
  }
}

int32_t Position; // 332 is right, and -332 is left of center
int main1(void){
  Clock_Init48MHz();
  Reflectance_Init();
  while(1){
    Data = Reflectance_Read(1000);
    Position = Reflectance_Position(Data);
    Clock_Delay1ms(10);
  }
}
