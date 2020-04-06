#include <12F675.h>
#device adc=8

#FUSES NOWDT                 	//No Watch Dog Timer
#FUSES INTRC_IO              	//Internal RC Osc, no CLKOUT
#FUSES NOCPD                 	//No EE protection
#FUSES NOPROTECT             	//Code not protected from reading
#FUSES NOMCLR                	//Master Clear pin used for I/O
#FUSES NOPUT                 	//No Power Up Timer
#FUSES NOBROWNOUT            	//No brownout reset

#use delay(clock=4000000)
#use rs232(baud=9600,parity=N,xmit=PIN_A0,rcv=PIN_A2,bits=8)

