/*
 * systemtest.c
 * 
 * Copyright 2018 Endredra <endredra@parrot>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


//#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define modo 0

int main () {
   char command[50], /*command0[50], */command1[50], command2[50], command3[50];
   //int mode;
   char mode, mode2, mode3;
   strcpy( command, "pkill screen" );
//strcpy( command0, "cd Documents/Arduino/testeusart/usart_teste/" );
   strcpy( command1, "make" );
   strcpy( command2, "make upload" );
   strcpy( command3, "screen -l /dev/ttyACM0 -s 9600" );

   puts("kill screen? y/n");
   //scanf("%d", &mode);
   	scanf("%c", &mode);

//   mode=getchar();
   if (mode=='y'||mode=='Y' || mode=='1'){
	   system(command);
   } else{
	    puts("Show on terminal? y/n");
		scanf(" %c", &mode3);
		if (mode3=='y'||mode3=='Y' || mode3=='1'){
		system(command3);

		}
	}
   
	puts("Compile and Upload Arduino? y/n");
	scanf(" %c", &mode2);
	if (mode2=='y'||mode2=='Y'|| mode2=='1'){
		system(command1);
		system(command2);

		}
		
	   
   puts("Done");

   return(0);
} 
