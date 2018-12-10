#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sbus_serial_driver.h>
#include <sbus_serial/Sbus.h>
#include <algorithm>
#include <boost/algorithm/clamp.hpp>

int main( int argc, char **argv )
{
	ros::init( argc, argv, "sbus_serial_node" );
	ros::NodeHandle nh;
	ros::NodeHandle param_nh( "~" );

	// Read/set parameters
	std::string frame_id;
	std::string port;
	int refresh_rate_hr;
	int rxMinValue;
	int rxMaxValue;
	int outMinValue;
	int outMaxValue;
	param_nh.param( "frame_id", frame_id, std::string( "base" ));   // frame_id isn't really used for SBUS messages
	param_nh.param( "port", port, std::string( "/dev/ttyTHS2" ));     // /dev/ttyTHS2 is UART on J17
	param_nh.param( "refresh_rate_hz", refresh_rate_hr, 5 );
	param_nh.param( "rxMinValue", rxMinValue, 172 );
	param_nh.param( "rxMaxValue", rxMaxValue, 1811 );
	param_nh.param( "outMinValue", outMinValue, 0 );
	param_nh.param( "outMaxValue", outMaxValue, 255 );

	// Used for mapping raw values
	float rawSpan = static_cast<float>(rxMaxValue-rxMinValue);
	float outSpan = static_cast<float>(outMaxValue-outMinValue);

	ros::Publisher pub = nh.advertise<sbus_serial::Sbus>( "sbus", 100 );
	ros::Rate loop_rate( refresh_rate_hr );

	// Initialize SBUS port (using pointer to have only the initialization in the try-catch block)
	sbus_serial::SBusSerialPort *sbusPort;
	try {
		sbusPort = new sbus_serial::SBusSerialPort( port, true );
	}
	catch( ... ) {
		// TODO: add error message in exception and report
		ROS_ERROR( "Unable to initalize SBUS port" );
		return 1;
	}

	// Create Sbus message instance and set invariant properties. Other properties will be set in the callback lambda
	sbus_serial::Sbus sbus;
	sbus.header.frame_id = frame_id;
	sbus.header.stamp = ros::Time( 0 );

	// Callback (auto-capture by reference)
	auto callback = [&]( const sbus_serial::SBusMsg sbusMsg ) {
		sbus.header.stamp = ros::Time::now();

		// Assign raw channels
		std::transform( sbusMsg.channels.begin(), sbusMsg.channels.end(), sbus.rawChannels.begin(), [&]( uint16_t rawChannel ) {
					return boost::algorithm::clamp( rawChannel, rxMinValue, rxMaxValue );   // Clamp to min/max raw values
				} );

		// Map to min/max values
		std::transform( sbusMsg.channels.begin(), sbusMsg.channels.end(), sbus.mappedChannels.begin(), [&]( uint16_t rawChannel ) {
					int16_t mappedValue = (rawChannel - rxMinValue) / rawSpan * outSpan + outMinValue;
					return boost::algorithm::clamp( mappedValue, outMinValue, outMaxValue );        // Clamp to min/max output values
				} );
	};
	sbusPort->setCallback( callback );

	ROS_INFO( "SBUS node started..." );

	ros::Time lastPublishedTimestamp( 0 );
	while( ros::ok())
	{
		// Only publish if we have a new sample
		if( lastPublishedTimestamp != sbus.header.stamp ) {
			pub.publish( sbus );
			lastPublishedTimestamp = sbus.header.stamp;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	delete sbusPort;        // Cleanup
	return 0;
}
