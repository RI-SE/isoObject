#include <boost/program_options.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/systems/si/length.hpp>
#include <cstdint>
#include <iostream>
#include <string>

#include "conversions/NcomToMonr.hpp"
#include "iso22133object.hpp"
#include "oxts-utils/NComRxC.hpp"
#include "printUtil.hpp"

using namespace boost::units::degree;
using namespace boost::units::si;
namespace po = boost::program_options;

static po::variables_map parse_arguments(int argc, char** argv) {

	po::variables_map vm;
	po::options_description options{"Allowed options"};
	options.add_options()("help,h", "Prints this message");
	options.add_options()("atos_ip",
						  po::value<std::string>()->default_value("0.0.0.0"),
						  "The IP for listening for communications with ATOS");
	options.add_options()(
	  "ncom_port", po::value<std::uint16_t>()->default_value(3000U), "The port for receiving NCOM data");
	options.add_options()("ncom_crs",
						  po::value<std::string>()->default_value("EPSG:4326"),
						  "The coordinate reference system for the NCOM data");
	options.add_options()(
	  "monr_crs", po::value<std::string>()->required(), "The coordinate reference system for the MONR data");

	po::store(po::parse_command_line(argc, argv, options), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << options << std::endl;
		exit(0);
	}
	return vm;
}

class Disarmed : public ISO22133::Disarmed {
public:
	void onEnter(ISO22133::TestObject& obj) override {
		std::cout << "Entering disarmed" << std::endl;
	}

	void onExit(ISO22133::TestObject&) override {
		std::cout << "Leaving disarmed" << std::endl;
	}
};

class PreArming : public ISO22133::PreArming {
public:
	void onEnter(ISO22133::TestObject& obj) override {
		std::cout << "Entering Pre-Arming" << std::endl;
		try {
			this->handleEvent(obj, ISO22133::Events::N);
		} catch (const std::runtime_error& e) {
			std::cerr << e.what() << '\n';
		}
	}
};

class IsoObject : public ISO22133::TestObject {
public:
	IsoObject(const std::string& ip) :
	  ISO22133::TestObject(ip) {
		ObjectSettingsType osem;
		osem.testMode = TEST_MODE_UNAVAILABLE;
		set_monr(iso22133::Monr{});
		setObjectSettings(osem);
	}

	std::vector<TrajectoryWaypointType> trajectory;

	void set_monr(const iso22133::Monr monr) {
		CartesianPosition position;
		position.xCoord_m		 = monr.x_position.value();
		position.yCoord_m		 = monr.y_position.value();
		position.zCoord_m		 = monr.z_position.value();
		position.isXcoordValid	 = (static_cast<std::int32_t>(monr.x_position.value()) != -2147483648);
		position.isYcoordValid	 = (static_cast<std::int32_t>(monr.y_position.value()) != -2147483648);
		position.isZcoordValid	 = (static_cast<std::int32_t>(monr.z_position.value()) != -2147483648);
		position.isPositionValid = (position.isXcoordValid && position.isYcoordValid && position.isZcoordValid);
		position.heading_rad	 = monr.yaw.value();
		position.isHeadingValid	 = (static_cast<std::uint16_t>(monr.yaw.value()) != 65535);

		SpeedType speed;
		speed.longitudinal_m_s	  = monr.longitudinal_speed.value();
		speed.isLongitudinalValid = (static_cast<std::int16_t>(monr.longitudinal_speed.value()) != -32768);
		speed.lateral_m_s		  = monr.lateral_speed.value();
		speed.isLateralValid	  = (static_cast<std::int16_t>(monr.lateral_speed.value()) != -32768);

		AccelerationType acceleration;
		acceleration.longitudinal_m_s2 = monr.londitudinal_acceleration.value();
		acceleration.isLongitudinalValid =
		  (static_cast<std::int16_t>(monr.londitudinal_acceleration.value()) != -32768);
		acceleration.lateral_m_s2	= monr.lateral_acceleration.value();
		acceleration.isLateralValid = (static_cast<std::int16_t>(monr.lateral_acceleration.value()) != -32768);

		DriveDirectionType drive_direction;
		if (monr.drive_direction == 0) {
			drive_direction = DriveDirectionType::OBJECT_DRIVE_DIRECTION_BACKWARD;
		} else if (monr.drive_direction == 1) {
			drive_direction = DriveDirectionType::OBJECT_DRIVE_DIRECTION_FORWARD;
		} else {
			drive_direction == DriveDirectionType::OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
		}

		this->setPosition(position);
		this->setSpeed(speed);
		this->setAcceleration(acceleration);
		this->setDriveDirection(drive_direction);
	}

	void handleAbort() {
		std::cout << "Bromsa!" << std::endl;
	}

	ISO22133::Disarmed* createDisarmed() const override {
		return dynamic_cast<ISO22133::Disarmed*>(new Disarmed);
	}

	ISO22133::PreArming* createPreArming() const override {
		return dynamic_cast<ISO22133::PreArming*>(new PreArming);
	}

	void onOSEM(ObjectSettingsType& osem) override {
		std::cout << "Object Settings Received" << std::endl;
		setObjectSettings(osem);
		PRINT_STRUCT(ObjectSettingsType, &osem, PRINT_FIELD(TestModeType, testMode))
	}

	void onTRAJ() override {
		std::cout << "Got onTRAJ signal, fetching new traj segments" << std::endl;
		std::vector<TrajectoryWaypointType> newTraj;
		newTraj = this->getTrajectory();
		if (this->getObjectSettings().testMode == TEST_MODE_ONLINE) {
			std::cout << "Test mode is online planned, appending new trajectory to existing" << std::endl;
			this->trajectory.insert(this->trajectory.end(), newTraj.begin(), newTraj.end());

			// We might receive trajectories that overlap, we remove the duplicate points by checking the time
			std::sort(this->trajectory.begin(),
					  this->trajectory.end(),
					  [](const TrajectoryWaypointType& t1, const TrajectoryWaypointType& t2) {
						  return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec <
								 t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec;
					  });
			this->trajectory.erase(std::unique(this->trajectory.begin(),
											   this->trajectory.end(),
											   [](const TrajectoryWaypointType& t1, const TrajectoryWaypointType& t2) {
												   return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec ==
														  t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec;
											   }),
								   this->trajectory.end());
		} else {
			std::cout << "Test mode is preplanned, replacing existing trajectory" << std::endl;
			this->trajectory = newTraj;
		}
		std::cout << "Trajectory size: " << this->trajectory.size() << std::endl;
	}

	void onSTRT(StartMessageType&) override {
		std::cout << "Object Starting" << std::endl;
	}

	int handleVendorSpecificMessage(const int msgType, const std::vector<char>& data) override {
		int handledBytes = 0;
		RemoteControlManoeuvreMessageType DCMMmsg;
		switch (msgType) {
			case MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM:
				handledBytes = decodeDCMMMessage(data.data(), data.size(), &DCMMmsg, 0);
				if (handledBytes < 0) {
					throw std::invalid_argument("Error decoding DCMM");
				} else {
					std::cout << "Handled DCMM Message" << std::endl;
				}
				break;

			default:
				break;
		}

		return handledBytes;
	}
};

int main(int argc, char** argv) {
	const auto args			   = parse_arguments(argc, argv);
	const std::string ncom_crs = args["ncom_crs"].as<std::string>();
	const std::string monr_crs = args["monr_crs"].as<std::string>();
	conversions::NcomToMonr ncom_to_monr_conversion(ncom_crs, monr_crs);

	IsoObject iso_object{args["atos_ip"].as<std::string>()};
	UdpServer udp_server{"0.0.0.0", 3000};
	auto ncom = std::shared_ptr<NComRxC>(NComCreateNComRxC(), [](NComRxC* ptr) { NComDestroyNComRxC(ptr); });

	while (1) {
		const std::vector<char> res = udp_server.receive();
		const auto ncom_res = NComNewChars(ncom.get(), reinterpret_cast<const unsigned char*>(res.data()), res.size());

		if (ncom_res == COM_NEW_UPDATE) {
			auto monr = ncom_to_monr_conversion.ncom_to_monr(*ncom);
			iso_object.set_monr(monr);
		}
	}
	return 0;
}
