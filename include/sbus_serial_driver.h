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
		SBusCallback callback_;
		int serial_port_fd_;
	};

} // namespace sbus_serial
