// CNC pendant interface to Duet
// Configuration constants
const int PinP = A1;
const int PinR = A0;
const int PinY = A2;

const int PinOP1 = 9;
const int PinOP2 = 8;

const unsigned long BaudRate = 57600;
const unsigned long MinCommandInterval = 20;

// Table of commands we send, one entry for each axis
const char* MoveCommands = "G91 G0 ";

#include "GCodeSerial.h"
#include "PassThrough.h"
PassThrough passThrough;

#include <avdweb_Switch.h>
Switch button_op1 = Switch(PinOP1);
Switch button_op2 = Switch(PinOP2);

const char* const ToggleOutCommands[] = {"M42 P0 S0.0\n","M42 P0 S1.0\n"};

bool toggle_out = false;

int serialBufferSize;
int distanceMultiplier;
int axis;
uint32_t whenLastCommandSent = 0;

const int axisPins[] = { PinR, PinP, PinY};

int pre_p = 0;
int pre_r = 0;
int pre_y = 0;

int mid = 512;

int c_p = 0;
int c_r = 0;
int c_y = 0;

bool go_axis[3] = {false,false,false}; 
int distance[3] = {0,0,0};

const char* const axisName[] = {" X"," Y"," Z"};

#define UartSerial   Serial1

GCodeSerial output(UartSerial);

const int div_input = 200;
const int div_move = 1000;

void setup()
{
  output.begin(BaudRate);

  //Serial.begin(BaudRate);//debug

  serialBufferSize = output.availableForWrite();

#if defined(__AVR_ATmega32U4__)     // Arduino Leonardo or Pro Micro
  TX_RX_LED_INIT;
#endif
}

// Check for received data from PanelDue, store it in the pass through buffer, and send it if we have a complete command
void checkPassThrough()
{
  unsigned int commandLength = passThrough.Check(UartSerial);
  if (commandLength != 0 && UartSerial.availableForWrite() == serialBufferSize)
  {
    output.write(passThrough.GetCommand(), commandLength);
  }
}

int tmp = 0;
void loop()
{

  c_r = analogRead( PinR );
  c_p = analogRead( PinP );
  c_y = analogRead( PinY );        

  tmp = ((c_r - mid)/div_input);
  if(abs(tmp)>=1){
    distance[0] = tmp;
  }else{
    distance[0] = 0;
  }

  tmp = ((c_p - mid)/div_input);
  if(abs(tmp)>=1){
    distance[1] = tmp;
  }else{
    distance[1] = 0;
  }

  tmp = ((c_y - mid)/div_input);
  if(abs(tmp)>=1){
    distance[2] = tmp;
  }else{
    distance[2] = 0;
  }
  
  // If the serial output buffer is empty, send a G0 command for the accumulated encoder motion.
  if (output.availableForWrite() == serialBufferSize)
  {
#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
    TXLED1;                         // turn off transmit LED
#endif
    const uint32_t now = millis();
    if (now - whenLastCommandSent >= MinCommandInterval)
    {

#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
        TXLED0;                     // turn on transmit LED
#endif
        button_op1.poll();
        button_op2.poll();
        whenLastCommandSent = now;

        if(button_op1.pushed()){
          output.write("G28\n");
        }

        if(button_op2.pushed()){
          toggle_out = !toggle_out;
          output.write(ToggleOutCommands[(int)toggle_out]);
          //Serial.println(ToggleOutCommands[(int)toggle_out]);
        }

        if((distance[0] != 0)||(distance[1] != 0)||(distance[2] != 0)){
          output.write(MoveCommands);

          for(int i=0;i<3;i++){
          
            if(distance[i] != 0){
              output.write(axisName[i]);
              if (distance[i] < 0)
              {
                output.write('-');
                distance[i] = -distance[i];
              }
              output.print(distance[i]/div_move);
              output.write('.');
              output.print(distance[i] % div_move);
              
            }
          }
          output.write('\n');
        }
        
        
    }
  }

  checkPassThrough();
}

// End
