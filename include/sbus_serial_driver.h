#pragma once

#include <atomic>
#include <thread>

namespace sbus_serial
{
	struct SBusMsg
	{
		// Normal 11 bit channels
		uint16_t channels[ 16 ];

		// Flags
		bool frame_lost;
		bool failsafe;
	};

	class SBusSerialPort
	{
	  public:

		SBusSerialPort();
		SBusSerialPort( const std::string& port, const bool start_receiver_thread );
		virtual ~SBusSerialPort();

	  protected:

		bool setUpSBusSerialPort( const std::string& port,
					  const bool start_receiver_thread );

		bool connectSerialPort( const std::string& port );
		void disconnectSerialPort();

		bool startReceiverThread();
		bool stopReceiverThread();

		void handleReceivedSbusMessage( const sbus_bridge::SBusMsg& received_sbus_msg );

	  private:

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
	};

} // namespace sbus_serial
