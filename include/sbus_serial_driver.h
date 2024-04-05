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
* SBUS driver
* Based on code from:
*	https://github.com/uzh-rpg/rpg_quadrotor_control
*/

#pragma once

#include <atomic>
#include <thread>
#include <functional>
#include <array>

namespace sbus_serial
{
	struct SBusMsg
	{
		// Raw 11 bit channels
		std::array<uint16_t, 16> channels;

		// Flags
		bool frame_lost;
		bool failsafe;
	};

	class SBusSerialPort
	{
	  public:
		using SBusCallback = std::function<void( const SBusMsg )>;      // Callback functor/lambda (using std::function allows capturing lambdas)

		SBusSerialPort();
		SBusSerialPort( const std::string& port, const bool start_receiver_thread );
		~SBusSerialPort();

		void setCallback( SBusCallback );       // Set callback to be invoked every time an SBUS packet is received

	  private:

		bool setUpSBusSerialPort( const std::string& port,  const bool start_receiver_thread );
		bool connectSerialPort( const std::string& port );
		void disconnectSerialPort();
		bool startReceiverThread();
		bool stopReceiverThread();

		static constexpr int kSbusFrameLength_ = 25;
		static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
		static constexpr uint8_t kSbusFooterByte_ = 0x00;
		static constexpr int kPollTimeoutMilliSeconds_ = 500;

		bool configureSerialPortForSBus() const;
		void serialPortReceiveThread();
		SBusMsg parseSbusMessage( uint8_t sbus_msg_bytes[kSbusFrameLength_] ) const;

		std::thread receiver_thread_;
		std::atomic_bool receiver_thread_should_exit_;
		int serial_port_fd_;
		SBusCallback callback_;
	};

} // namespace sbus_serial
