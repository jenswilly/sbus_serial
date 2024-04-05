/*
* Copyright 2018-2024 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* SBUS test application for reading raw values
*/

#include "sbus_serial_driver.h"
#include <iostream>

int main( int argc, char *argv[] )
{
	// We must have exactly one command-line argument: the port name
	if( argc != 2 ) {
		std::cerr << "ERROR: Specify UART port to use as argument.\n";
		return 1;
	}

	std::string portName = argv[1];
	sbus_serial::SBusSerialPort *sbusPort;
	try {
		sbusPort = new sbus_serial::SBusSerialPort( portName, true );
	}
	catch( ... ) {
		std::cerr << "Unable to initalize SBUS port: " << portName << "\n";
		return 1;
	}

	auto callback = [&]( const sbus_serial::SBusMsg sbusMsg ) {
		printf( "Ch1: %04d, Ch2: %04d, Ch3: %04d, Ch4: %04d, Ch5: %04d, Ch6: %04d, Ch7: %04d, Ch8: %04d, Ch9: %04d, Ch10: %04d, Ch11: %04d, Ch12: %04d, Ch13: %04d, Ch14: %04d, Ch15: %04d, Ch16: %04d, FS: %d, FL: %d\n",
			sbusMsg.channels[0],
			sbusMsg.channels[1],
			sbusMsg.channels[2],
			sbusMsg.channels[3],
			sbusMsg.channels[4],
			sbusMsg.channels[5],
			sbusMsg.channels[6],
			sbusMsg.channels[7],
			sbusMsg.channels[8],
			sbusMsg.channels[9],
			sbusMsg.channels[10],
			sbusMsg.channels[11],
			sbusMsg.channels[12],
			sbusMsg.channels[13],
			sbusMsg.channels[14],
			sbusMsg.channels[15],
			sbusMsg.failsafe,
			sbusMsg.frame_lost );
	};
	sbusPort->setCallback( callback );

	// Loop until terminated
	while( true )
		;
}
