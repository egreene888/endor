/*
Dimension Engineering Sabertooth 2X10
Sample code for packetized serial mode
This code contains some basic functions to control two
different Saberteeth on the same serial bus.
This code is for CCS PCWH and compiles to run on a PIC12F675
*/

#include "Sabertooth packetized test.h"

//Essential packetized serial mode functions
Void DriveForward1(char address, char speed)
{
Putc(address);
Putc(0); //drive motor 1 forward command
Putc(speed);
Putc((address + 0 + speed) & 0b01111111);
}

Void DriveForward2(char address, char speed)
{
Putc(address);
Putc(4); //drive motor 2 forward command
Putc(speed);
Putc((address + 4 + speed) & 0b01111111);
}

Void DriveBackward1(char address, char speed)
{
Putc(address);
Putc(1); //drive motor 1 backwards command
Putc(speed);
Putc((address + 1 + speed) & 0b01111111);
}

Void DriveBackward2(char address, char speed)
{
Putc(address);
Putc(5);//drive motor 2 backwards command
Putc(speed);
Putc((address + 5 + speed) & 0b01111111);
}



void main()
{
   //variables declared here
   char i;

   //Configuring the PIC on startup. Analog input is not used here.
   setup_adc_ports(NO_ANALOGS|VSS_VDD);
   setup_adc(ADC_OFF);
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
   setup_timer_1(T1_DISABLED);
   setup_comparator(NC_NC);
   setup_vref(VREF_LOW|-2);

   delay_ms(2000); //power up delay

   putc(0xAA); //Send the autobauding character to Sabertooth first!

   delay_ms(100);
   while(1)
   {

      //commands both motors forward - starting slow, then speeding up
      for(i=0;i<127;i++)
      {
         DriveForward1(132,i);  //Commands a Sabertooth at address 132
         delay_ms(10); //short delay in between each command
         DriveForward2(132,i);
         delay_ms(100);//longer delay to slow the speed of the ramping function
      }

      //commands both motors backwards - starting slow, then speeding up
      for(i=0;i<127;i++)
      {
         DriveBackward1(132,i); //Commands a Sabertooth at address 132
         delay_ms(10);
         DriveBackward2(132,i);
         delay_ms(100);
      }

      for(i=0;i<127;i++)
      {
         DriveForward1(128,i);  //Commands a different Sabertooth at address 128
         delay_ms(10);
         DriveForward2(128,i);
         delay_ms(100);
      }

      for(i=0;i<127;i++)
      {
         DriveBackward1(128,i); //Commands a different Sabertooth at address 128
         delay_ms(10);
         DriveBackward2(128,i);
         delay_ms(100);
      }

   }

}
