/*
  * Author: Christian Llanes, Georgia Institute of Technology, USA
  */

#include "crazysim_plugin.h"
#include <algorithm>
#include <charconv>
#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>

#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(crazyflie_interface::GzCrazyflieInterface,
			  gz::sim::System,
			  crazyflie_interface::GzCrazyflieInterface::ISystemConfigure,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPreUpdate,
			  crazyflie_interface::GzCrazyflieInterface::ISystemPostUpdate)
using namespace crazyflie_interface;

namespace {
uint32_t readHeaderSequence(const gz::msgs::Header& header, bool& found) {
  for (int index = 0; index < header.data_size(); index++) {
    const auto& item = header.data(index);
    if (item.key() != "seq" || item.value_size() == 0) {
      continue;
    }

    const std::string& value = item.value(0);
    uint64_t parsed = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec == std::errc{} && result.ptr == value.data() + value.size() &&
        parsed <= std::numeric_limits<uint32_t>::max()) {
      found = true;
      return static_cast<uint32_t>(parsed);
    }
  }

  found = false;
  return 0;
}

uint64_t readSimulationTimeNs(const gz::msgs::IMU& imu) {
  const auto& stamp = imu.header().stamp();
  if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1000000000) {
    return 0;
  }
  return static_cast<uint64_t>(stamp.sec()) * 1000000000ULL +
         static_cast<uint64_t>(stamp.nsec());
}

bool waitForDatagram(int socketFd) {
	struct pollfd descriptor = {socketFd, POLLIN, 0};
	int result;
	do {
		result = poll(&descriptor, 1, 100);
	} while (result < 0 && errno == EINTR);
	return result > 0 && (descriptor.revents & POLLIN) != 0;
}
} // namespace

GzCrazyflieInterface::GzCrazyflieInterface() :
	namespace_(kDefaultNamespace),
	motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
	imu_sub_topic_(kDefaultImuTopic),
	magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
	barometer_sub_topic_(kDefaultFluidPressureTopic),
	odom_sub_topic_(kDefaultOdomTopic),
	cf_prefix(kDefaultCfPrefix),
	isPluginOn(true){
	const char* traceMotor = std::getenv("NCS_CRAZYSIM_TRACE_MOTOR");
	traceMotorEnabled_ = traceMotor != nullptr && std::string(traceMotor) == "1";
#ifdef NCS_SITL_MOTOR_TRACE
	traceMotorAllEnabled_ = traceMotor != nullptr && std::string(traceMotor) == "all";
	if (traceMotorAllEnabled_) {
		const char* tracePath = std::getenv("NCS_CRAZYSIM_TRACE_MOTOR_PATH");
		if (tracePath == nullptr || tracePath[0] == '\0') {
			std::cerr << "NCS_SITL motor_trace=all requires NCS_CRAZYSIM_TRACE_MOTOR_PATH"
			          << std::endl;
			traceMotorAllEnabled_ = false;
		} else {
			motorTracePath_ = tracePath;
			motorTraceRecords_.reserve(kMotorTraceCapacity);
			std::cerr << "NCS_SITL motor_trace=all capacity=" << kMotorTraceCapacity
			          << " path=" << motorTracePath_ << std::endl;
		}
	}
#endif
	const char* disableActuators = std::getenv("NCS_CRAZYSIM_DISABLE_ACTUATORS");
	disableActuators_ = disableActuators != nullptr &&
	                    std::string(disableActuators) == "1";
	const char* imuTracePath = std::getenv("NCS_CRAZYSIM_IMU_TRACE_FILE");
#ifdef NCS_SITL_IMU_PROVENANCE
	if (imuTracePath != nullptr && imuTracePath[0] != '\0') {
		std::error_code error;
		if (std::filesystem::exists(imuTracePath, error) || error) {
			std::cerr << "NCS_SITL refusing existing or inaccessible IMU send trace: "
				  << imuTracePath << std::endl;
		} else {
			imuSendTracePath_ = imuTracePath;
			imuSendTraceRecords_.resize(kImuSendTraceCapacity);
		}
	}
#else
	if (imuTracePath != nullptr && imuTracePath[0] != '\0') {
		std::cerr << "NCS_SITL IMU send trace requires provenance-enabled build"
			  << std::endl;
	}
#endif
	if (disableActuators_) {
		std::cerr << "NCS_SITL actuator_bridge=disconnected" << std::endl;
	}
}

GzCrazyflieInterface::~GzCrazyflieInterface() {
	isPluginOn.store(false, std::memory_order_release);
	cfLibAddrInitialized_.store(false, std::memory_order_release);
	socketInit.store(false, std::memory_order_release);
	if (fd >= 0)
		shutdown(fd, SHUT_RDWR);
	if (fd_cfLib >= 0)
		shutdown(fd_cfLib, SHUT_RDWR);
	isInit = false;
	if (senderCfFirmwareThread.joinable()) senderCfFirmwareThread.join();
	if (senderCfLibThread.joinable()) senderCfLibThread.join();
	if (receiverCfFirmwareThread.joinable()) receiverCfFirmwareThread.join();
	if (receiverCfLibThread.joinable()) receiverCfLibThread.join();
	if (fd >= 0) close(fd);
	if (fd_cfLib >= 0) close(fd_cfLib);
	dumpImuSendTrace();
#ifdef NCS_SITL_MOTOR_TRACE
	dumpMotorTrace();
#endif
#ifdef NCS_SITL_IMU_PROVENANCE
	std::cerr << "NCS_SITL imu_queue_enqueue_failures="
	          << imuQueueEnqueueFailures_.load(std::memory_order_relaxed)
	          << std::endl;
#endif
}

void GzCrazyflieInterface::Configure(const gz::sim::Entity &_entity,
						const std::shared_ptr<const sdf::Element> &_sdf,
						gz::sim::EntityComponentManager &_ecm,
						gz::sim::EventManager &_eventMgr) {
	gzdbg << __FUNCTION__ << "() called." << std::endl;
		
  	auto sdfClone = _sdf->Clone();

	/* Get Crazylfie SDF parameters */
	cffirm_addr = sdfClone->Get<std::string>("cffirm_addr");
	cffirm_port = sdfClone->Get<std::string>("cffirm_port");
	cflib_addr = sdfClone->Get<std::string>("cflib_addr");
	cflib_port = sdfClone->Get<std::string>("cflib_port");
	cf_prefix = sdfClone->Get<std::string>("cfPrefix");
	imu_sub_topic_ = sdfClone->Get<std::string>("imuSubTopic");
	magnetic_field_sub_topic_ = sdfClone->Get<std::string>("magSubTopic");
	barometer_sub_topic_ = sdfClone->Get<std::string>("baroSubTopic");
	cf_id_ = sdfClone->Get<int>("cfId");

	// ---------- Setup socket for Crazyflie firmware -----------
	std::string dest_addr_cfFirm = cffirm_addr;
	std::string dest_port_cfFirm = cffirm_port;
	// open socket in UDP mode
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfFirm socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	if (dest_addr_cfFirm == "INADDR_ANY")
		myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfFirm.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfFirm invalid !" << std::endl;
		return;
	}else
		myaddr.sin_addr.s_addr = inet_addr(dest_addr_cfFirm.c_str());
	port = std::stoi(dest_port_cfFirm);
	myaddr.sin_port = htons(port);
	// Bind the socket
	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0){
		gzerr << "Bind socket CfFirm failed " << std::endl;
		return;
	}
	addrlen_rcv = sizeof(remaddr_rcv);
	

	// ---------- Setup socket for Crazyflie Python api -----------
	std::string dest_addr_cfLib = cflib_addr;
	std::string dest_port_cfLib = cflib_port;
	// open CfLib socket in UDP mode
	if ((fd_cfLib = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		gzerr << "create CfLib socket failed " << std::endl;
		return;
	}
	memset((char *)&myaddr_CfLib, 0, sizeof(myaddr_CfLib));
	myaddr_CfLib.sin_family = AF_INET;
	if (dest_addr_cfLib == "INADDR_ANY")
		myaddr_CfLib.sin_addr.s_addr = htonl(INADDR_ANY);
	else if (inet_addr(dest_addr_cfLib.c_str()) == INADDR_NONE){
		gzerr << "Adress IP CfLib invalid !" << std::endl;
		return;
	}else
		myaddr_CfLib.sin_addr.s_addr = inet_addr(dest_addr_cfLib.c_str());
	port_cfLib = std::stoi(dest_port_cfLib);
	myaddr_CfLib.sin_port = htons(port_cfLib);
	// Bind the socket
	if (bind(fd_cfLib, (struct sockaddr *)&myaddr_CfLib, sizeof(myaddr_CfLib)) < 0){
		gzerr << "Bind socket CfLib failed " << std::endl;
		return;
	}
	addrlen_rcv_cfLib = sizeof(remaddr_rcv_cfLib);

	imu_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	barometer_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	odom_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	cflib_to_firmware_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);
	firmware_to_cflib_queue = moodycamel::BlockingConcurrentQueue<crtpPacket_t>(20);

	addrlen = sizeof(remaddr);
	addrlen_cfLib = sizeof(remaddr_cfLib);
	isInit = false;
	socketInit = false;
	cfLibAddrInitialized_ = false;
	m_motor_command_.m1 = 0;
	m_motor_command_.m2 = 0;
	m_motor_command_.m3 = 0;
	m_motor_command_.m4 = 0;

	receiverCfFirmwareThread = std::thread(&GzCrazyflieInterface::recvCfFirmwareThread, this);
	receiverCfLibThread = std::thread(&GzCrazyflieInterface::recvCfLibThread, this);
	senderCfFirmwareThread = std::thread(&GzCrazyflieInterface::sendCfFirmwareThread, this);
	senderCfLibThread = std::thread(&GzCrazyflieInterface::sendCfLibThread, this);
	initializeSubsAndPub();
}

void GzCrazyflieInterface::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) {
	writeMotors();
}

void GzCrazyflieInterface::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) {

}

void GzCrazyflieInterface::ImuCallback(const gz::msgs::IMU& imu_msg) {
	#ifdef NCS_SITL_IMU_PROVENANCE
	bool hasGazeboSequence = false;
	uint32_t sequence = readHeaderSequence(imu_msg.header(), hasGazeboSequence);
	if (!hasGazeboSequence) {
		sequence = imuSequenceFallback_.fetch_add(1, std::memory_order_relaxed);
	}
	#endif
	struct imu_s m_imu_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_GYRO_ACC_SIM,
		.acc = { static_cast<int16_t>(imu_msg.linear_acceleration().x()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().y()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF),
				 static_cast<int16_t>(imu_msg.linear_acceleration().z()  	/ 	SENSORS_G_PER_LSB_CFG 		/ GRAVITY_MAGNITUDE_CF)},
		.gyro = {static_cast<int16_t>(imu_msg.angular_velocity().x()	   	/ 	SENSORS_DEG_PER_LSB_CFG	 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().y()    	/ 	SENSORS_DEG_PER_LSB_CFG 	/ DEG_TO_RAD_CF),
				 static_cast<int16_t>(imu_msg.angular_velocity().z()    	/	SENSORS_DEG_PER_LSB_CFG		/ DEG_TO_RAD_CF)}};

	#ifdef NCS_SITL_IMU_PROVENANCE
	encodeImuProvenance(m_imu_info.sequenceLE, sequence,
	                    readSimulationTimeNs(imu_msg));
	m_imu_info.sequenceIsGazeboHeader = hasGazeboSequence ? 1U : 0U;
	#endif

	crtpPacket_t msg;
	msg.size = sizeof(m_imu_info) - 1;
	memcpy(msg.raw, (const uint8_t*) &m_imu_info , sizeof(m_imu_info));
	if (!imu_queue.enqueue(msg)) {
		imuQueueEnqueueFailures_.fetch_add(1, std::memory_order_relaxed);
	}
}

void GzCrazyflieInterface::BarometerCallback(const gz::msgs::FluidPressure& air_pressure_msg) {
	double temperature_at_altitude_kelvin = kSeaLevelTempKelvin * exp(- log(air_pressure_msg.pressure() / kPressureOneAtmospherePascals)/kAirConstantDimensionless);
	double height_geopotential_m = (kSeaLevelTempKelvin - temperature_at_altitude_kelvin)/kTempLapseKelvinPerMeter;
	double height_geometric_m = height_geopotential_m * kEarthRadiusMeters / (kEarthRadiusMeters - height_geopotential_m);
	
	struct baro_s m_baro_info = {
		.header = crtp(CRTP_PORT_SETPOINT_SIM, 0),
		.type = SENSOR_BARO_SIM,
		.pressure = static_cast<float>(air_pressure_msg.pressure() * 0.01), // Convert in mbar
		.temperature = static_cast<float>(temperature_at_altitude_kelvin - 273.15),
		.asl = static_cast<float>(height_geometric_m)
	};

	crtpPacket_t msg;
	msg.size = sizeof(m_baro_info) - 1;
	memcpy(msg.raw, (const uint8_t*) &m_baro_info , sizeof(m_baro_info));
	barometer_queue.enqueue(msg);
}

void GzCrazyflieInterface::OdomCallback(const gz::msgs::Odometry& odom_msg) {
	// Just need to transfer to the fcu the external position
	struct CrtpExtPose_s ext_pose  = {
		.header = crtp(CRTP_PORT_LOCALIZATION, CRTP_LOC_CHANNEL_GEN_LOC),
		.id = CRTP_GEN_LOC_ID_EXT_POS,
		.x = static_cast<float>(odom_msg.pose().position().x()),
		.y = static_cast<float>(odom_msg.pose().position().y()), 
		.z = static_cast<float>(odom_msg.pose().position().z()),
		.qx = static_cast<float>(odom_msg.pose().orientation().x()),
		.qy = static_cast<float>(odom_msg.pose().orientation().y()),
		.qz = static_cast<float>(odom_msg.pose().orientation().z()),
		.qw = static_cast<float>(odom_msg.pose().orientation().w())
	}; 
	
	crtpPacket_t msg;
	msg.size = sizeof(ext_pose) - 1;
	memcpy(msg.raw, (const uint8_t*) &ext_pose , sizeof(ext_pose));
	odom_queue.enqueue(msg);
}

bool GzCrazyflieInterface::sendCfFirmware(const uint8_t* data , uint32_t length) {	
	struct sockaddr_in peer;
	socklen_t peerLength;
	{
		std::lock_guard<std::mutex> lock(firmware_peer_mutex_);
		if (!socketInit.load(std::memory_order_acquire))
			return false;
		peer = remaddr;
		peerLength = addrlen;
	}
	const auto sendTime = std::chrono::steady_clock::now().time_since_epoch();
	const uint64_t hostMonotonicSendNs =
		std::chrono::duration_cast<std::chrono::nanoseconds>(sendTime).count();
	int transferred = sendto(fd, data, length, 0, (struct sockaddr *) &peer, peerLength);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
	recordImuSend(data, static_cast<uint32_t>(transferred), hostMonotonicSendNs);
    return true;
}

void GzCrazyflieInterface::recordImuSend(const uint8_t* data, uint32_t length,
										 uint64_t hostMonotonicSendNs) {
#ifdef NCS_SITL_IMU_PROVENANCE
	if (imuSendTracePath_.empty() || length != sizeof(imu_s) ||
	    !(crtp(data[0]) == crtp(CRTP_PORT_SETPOINT_SIM, 0)) ||
	    data[1] != SENSOR_GYRO_ACC_SIM) {
		return;
	}

	ImuSendTraceRecord& record =
		imuSendTraceRecords_[imuSendTraceCount_ % kImuSendTraceCapacity];
	const uint8_t* metadata = data + sizeof(crtp) + sizeof(uint8_t) +
							  2 * sizeof(Axis3i16);
	record.sequence = decodeImuSequence(metadata);
	record.simulationTimeNs = decodeImuSimTimeNs(metadata + 4);
	record.hostMonotonicSendNs = hostMonotonicSendNs;
	std::copy(data, data + length, record.packet.begin());
	imuSendTraceCount_++;
#else
	(void)data;
	(void)length;
	(void)hostMonotonicSendNs;
#endif
}

void GzCrazyflieInterface::dumpImuSendTrace() {
	if (imuSendTracePath_.empty()) {
		return;
	}
	std::ofstream output(imuSendTracePath_, std::ios::out | std::ios::trunc);
	if (!output) {
		std::cerr << "NCS_SITL failed to open IMU send trace: "
			  << imuSendTracePath_ << std::endl;
		return;
	}
	output << "sequence,simulation_time_ns,host_monotonic_send_ns,packet_hex\n";
	const uint64_t recordCount =
		std::min<uint64_t>(imuSendTraceCount_, imuSendTraceRecords_.size());
	const uint64_t firstIndex =
		imuSendTraceCount_ > imuSendTraceRecords_.size()
			? imuSendTraceCount_ % imuSendTraceRecords_.size()
			: 0;
	for (uint64_t offset = 0; offset < recordCount; offset++) {
		const ImuSendTraceRecord& record = imuSendTraceRecords_[
			(firstIndex + offset) % imuSendTraceRecords_.size()];
		output << record.sequence << ',' << record.simulationTimeNs << ','
		       << record.hostMonotonicSendNs << ',' << std::hex
		       << std::setfill('0');
		for (uint8_t byte : record.packet) {
			output << std::setw(2) << static_cast<unsigned int>(byte);
		}
		output << std::dec << '\n';
	}
	std::cerr << "NCS_SITL imu_send_trace_records=" << recordCount
		  << " overwritten=" << (imuSendTraceCount_ - recordCount)
		  << " path=" << imuSendTracePath_ << std::endl;
}

#ifdef NCS_SITL_MOTOR_TRACE
void GzCrazyflieInterface::dumpMotorTrace() {
	if (!traceMotorAllEnabled_) return;
	std::ofstream output(motorTracePath_);
	if (!output) {
		std::cerr << "NCS_SITL motor_trace=all failed to open " << motorTracePath_
		          << std::endl;
		return;
	}
	output << std::setprecision(9);
	for (size_t sequence = 0; sequence < motorTraceRecords_.size(); sequence++) {
		const auto& record = motorTraceRecords_[sequence];
		output << "{\"capture_monotonic_time_ns\":" << record.monotonicTimeNs
		       << ",\"sequence\":" << sequence << ",\"packet_bytes\":[";
		for (size_t index = 0; index < record.packet.size(); index++) {
			if (index != 0) output << ',';
			output << static_cast<unsigned>(record.packet[index]);
		}
		output << "],\"motor_ratio\":[";
		for (size_t index = 0; index < record.ratios.size(); index++) {
			if (index != 0) output << ',';
			output << record.ratios[index];
		}
		output << "],\"omega_rad_s\":[";
		for (size_t index = 0; index < record.omegaRadPerSecond.size(); index++) {
			if (index != 0) output << ',';
			output << record.omegaRadPerSecond[index];
		}
		output << "]}\n";
	}
	std::cerr << "NCS_SITL motor_trace=all records=" << motorTraceRecords_.size()
	          << " dropped=" << motorTraceDropped_ << " path=" << motorTracePath_
	          << std::endl;
}
#endif

bool GzCrazyflieInterface::sendCfLib(const uint8_t* data , uint32_t length) {	
	int transferred = sendto(fd_cfLib, data, length, 0, (struct sockaddr *) &(remaddr_cfLib), addrlen_cfLib);
	if (transferred <= 0)
		throw std::runtime_error(strerror(errno));
    return true;
}

void GzCrazyflieInterface::recvCfFirmwareThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		if (!waitForDatagram(fd))
			continue;
		int len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv, &addrlen_rcv);
		if (len <= 0 )
			continue;
		if (buf[0] == 0xF3 && len == 1){
			const uint8_t response = 0xF3;
			bool responseSent = false;
			{
				std::lock_guard<std::mutex> lock(firmware_peer_mutex_);
				socketInit.store(false, std::memory_order_release);
				remaddr = remaddr_rcv;
				addrlen = addrlen_rcv;
				const int transferred = sendto(
					fd, &response, sizeof(response), 0,
					(struct sockaddr *) &remaddr, addrlen);
				responseSent = transferred == static_cast<int>(sizeof(response));
				if (responseSent)
					socketInit.store(true, std::memory_order_release);
			}
			if (responseSent)
				gzmsg << "Accepted firmware handshake message..." << std::endl;
			else
				gzerr << "Failed to reply to firmware handshake: "
				      << strerror(errno) << std::endl;
		}
		else if (socketInit && (crtp(buf[0]) == crtp(0x09,0)) && (len == 9)) // Motor command message
			handleMotorsMessage(&buf[0]);
		else if (socketInit && cfLibAddrInitialized_){
			crtpPacket_t packet;
			packet.size = len - 1;
			memcpy(&packet.raw[0] , &buf[0], sizeof(uint8_t) * len);
			bool succeeded  = firmware_to_cflib_queue.enqueue(packet);
			if(!succeeded)
				gzmsg << "Failed to enqueue firmware_to_cflib_queue in recvCfFirmwareThread()." << std::endl;
		}
	}
}

void GzCrazyflieInterface::recvCfLibThread() {
	uint8_t buf[32];
	while(isPluginOn)
	{
		if (!waitForDatagram(fd_cfLib))
			continue;
		int len = recvfrom(fd_cfLib, buf, sizeof(buf), 0, (struct sockaddr *) &remaddr_rcv_cfLib, &addrlen_rcv_cfLib);
		if (len <= 0 )
			continue;
		if (!socketInit.load(std::memory_order_acquire))
			continue;

		// Update cflib address on every received packet (activity-based detection)
		remaddr_cfLib = remaddr_rcv_cfLib;
		addrlen_cfLib = addrlen_rcv_cfLib;
		cfLibAddrInitialized_ = true;

		// Echo back null CRTP packets (0xFF) for scan/ping
		if (buf[0] == 0xFF && len == 1) {
			uint8_t echo[1] = {0xFF};
			sendCfLib(echo, sizeof(echo));
		}

		// Forward all packets to firmware
		recvCfLib(&buf[0], len);
	}
}

void GzCrazyflieInterface::sendCfFirmwareThread() {
	crtpPacket_t msgs[10];
	while(isPluginOn){
		if(!socketInit.load(std::memory_order_acquire)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		// IMU is the high-rate clock for the SITL bridge. Block until it arrives
		// instead of spinning continuously between 500 Hz sensor callbacks.
		size_t imu_msg_count = 0;
		if (imu_queue.wait_dequeue_timed(msgs[0], std::chrono::milliseconds(10)))
			imu_msg_count = 1 + imu_queue.try_dequeue_bulk(msgs + 1, 9);
		for (size_t i = 0; i != imu_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t baro_msg_count = barometer_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != baro_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t odom_msg_count = odom_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != odom_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}

		size_t cflib_msg_count = cflib_to_firmware_queue.try_dequeue_bulk(msgs, 10);
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfFirmware(msgs[i].raw, msgs[i].size+1);
		}
	}
}

void GzCrazyflieInterface::sendCfLibThread() {
	crtpPacket_t msgs[10];
	while(isPluginOn){
		if(!socketInit.load(std::memory_order_acquire) ||
		   !cfLibAddrInitialized_.load(std::memory_order_acquire)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}

		size_t cflib_msg_count = 0;
		if (firmware_to_cflib_queue.wait_dequeue_timed(
				msgs[0], std::chrono::milliseconds(10))) {
			cflib_msg_count = 1 + firmware_to_cflib_queue.try_dequeue_bulk(msgs + 1, 9);
		}
		for (size_t i = 0; i != cflib_msg_count; i++) {
			sendCfLib(msgs[i].raw, msgs[i].size+1);
		}
	}
} 


void GzCrazyflieInterface::initializeSubsAndPub() {
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + imu_sub_topic_, &GzCrazyflieInterface::ImuCallback , this);
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + barometer_sub_topic_, &GzCrazyflieInterface::BarometerCallback, this );
	node_.Subscribe(cf_prefix + "_" + std::to_string(cf_id_) + odom_sub_topic_, &GzCrazyflieInterface::OdomCallback, this );
	
	motor_velocity_reference_pub_ = node_.Advertise<gz::msgs::Actuators>(cf_prefix + "_" + std::to_string(cf_id_) + motor_velocity_reference_pub_topic_);
	
	isInit = true;
	gzmsg << "Init subs and Pubs done : " << std::endl;
}

void GzCrazyflieInterface::handleMotorsMessage(const uint8_t* data) {
	crtpMotorsDataResponse* motorsData = (crtpMotorsDataResponse *) data;
	const float m1 = PWM2OMEGA(motorsData->m1);
	const float m2 = PWM2OMEGA(motorsData->m2);
	const float m3 = PWM2OMEGA(motorsData->m3);
	const float m4 = PWM2OMEGA(motorsData->m4);
	{
		std::unique_lock<std::mutex> mlock(motors_mutex);
		m_motor_command_.m1 = m1;
		m_motor_command_.m2 = m2;
		m_motor_command_.m3 = m3;
		m_motor_command_.m4 = m4;
	}

	if (traceMotorEnabled_ &&
	    (motorsData->m1 != 0 || motorsData->m2 != 0 || motorsData->m3 != 0 ||
	     motorsData->m4 != 0) &&
	    !motorTraceLogged_.exchange(true, std::memory_order_relaxed)) {
		std::cerr << "TRACE_MOTOR_NONZERO NCS_SITL_ACTUATOR source=firmware"
		          << " ratio=" << motorsData->m1 << ',' << motorsData->m2 << ','
		          << motorsData->m3 << ',' << motorsData->m4 << " omega_rad_s="
		          << m1 << ',' << m2 << ',' << m3 << ',' << m4 << std::endl;
	}
#ifdef NCS_SITL_MOTOR_TRACE
	if (traceMotorAllEnabled_) {
		if (motorTraceRecords_.size() < kMotorTraceCapacity) {
			auto& record = motorTraceRecords_.emplace_back();
			record.monotonicTimeNs = static_cast<uint64_t>(
				std::chrono::duration_cast<std::chrono::nanoseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count());
			std::copy(data, data + record.packet.size(), record.packet.begin());
			record.ratios = {motorsData->m1, motorsData->m2,
			                 motorsData->m3, motorsData->m4};
			record.omegaRadPerSecond = {m1, m2, m3, m4};
		} else {
			motorTraceDropped_++;
		}
	}
#endif
}

void GzCrazyflieInterface::recvCfLib(uint8_t* data , int len) {
	crtpPacket_t packet;
	packet.size = len - 1;
	memcpy(&packet.raw[0] , data , sizeof(uint8_t) * len);
	bool succeeded  = cflib_to_firmware_queue.enqueue(packet);
	if(!succeeded)
		gzmsg << "Failed recvCfLib" << std::endl;
}


void GzCrazyflieInterface::writeMotors() {
	m_motor_speed.clear_velocity();
	motors_mutex.lock();
	m_motor_speed.add_velocity(disableActuators_ ? 0.0f : m_motor_command_.m1); // 0
	m_motor_speed.add_velocity(disableActuators_ ? 0.0f : m_motor_command_.m2); // 1
	m_motor_speed.add_velocity(disableActuators_ ? 0.0f : m_motor_command_.m3); // 2
	m_motor_speed.add_velocity(disableActuators_ ? 0.0f : m_motor_command_.m4); // 3
	motors_mutex.unlock();
	motor_velocity_reference_pub_.Publish(m_motor_speed);
}
