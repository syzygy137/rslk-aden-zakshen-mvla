// Lab07_FSMmain.c
// Runs on MSP432
// Student version of FSM lab, FSM with 2 inputs and 2 outputs.
// Rather than real sensors and motors, it uses LaunchPad I/O
// Daniel and Jonathan Valvano
// March 17, 2017

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

#include <stdint.h>
#include "msp.h"
#include "../inc/clock.h"
#include "../inc/LaunchPad.h"
#include "../inc/Reflectance.h"

// Linked data structure
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

// Create User-Friendly ways to reference the FSM states
#define Center       &fsm[0]
#define LeftOff1     &fsm[1]   // Left Sensor - bit 4 is off
#define LeftOff2     &fsm[2]   // Left Sensor - bit 4 is off
#define RightOff1    &fsm[3]   // Right Sensor - bit 3 is off
#define RightOff2    &fsm[4]   // Right Sensor - bit 3 is off
#define LostLeft     &fsm[5]   // Both Sensors Off, previous state is RightOff*
#define LostRight    &fsm[6]   // Both Sensors Off, previous state is LeftOff*
#define Fwd5         &fsm[7]
#define Stop         &fsm[8]

// State       Output   Delay   Next_0     Next_1     Next_2     Next_3
// Center      Both     500     RightOff1  LeftOff1   RightOff1  Center

State_t fsm[9]={
  {0x03, 500,  { RightOff1, LeftOff1,   RightOff1,  Center }}, // Center
/* TODO: fill in the rest of the states
  {0x02, 500,  { , , , }},  // LeftOff1
  {0x03, 500,  { , , , }},  // LeftOff2
  {0x01, 500,  { , , , }},  // RightOff1
  {0x03, 500,  { , , , }},  // RightOff2
  {0x02, 5000, { , , , }},  // LostLeft
  {0x01, 5000, { , , , }},  // LostRight
  {0x03, 5000, { , , , }},  // Fwd5
  {0x00, 500,  { , , , }},  // Stop
*/
};


State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
/*Run FSM continuously
1) Output depends on State (display state on LED1, LED2 per slides)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */
int main(void){
  Clock_Init48MHz();
  LaunchPad_Init();
  Spt = Center;
  while(1){
    Output = Spt->out;            // set output from FSM
    LaunchPad_LED(0x0);     // display state information per slides
    LaunchPad_Output(0x0);
    Clock_Delay1ms(Spt->delay);   // wait
    //Input = LaunchPad_Input();    // read sensors
    Input = Reflectance_Center(1000);
    Spt = Spt->next[Input];       // next depends on input and current state
  }
}

