/*
OpenTherm.cpp - OpenTherm Communication Library For Arduino, ESP8266
Copyright 2018, Ihor Melnyk
*/

#include "OpenTherm.h"

OpenTherm::OpenTherm(int inPin, int outPin, bool isSlave):
	status(OpenThermStatus::NOT_INITIALIZED),
	inPin(inPin),
	outPin(outPin),
	isSlave(isSlave),
	response(0),
	responseStatus(OpenThermResponseStatus::NONE),
	responseTimestamp(0),
	handleInterruptCallback(NULL),
	processResponseCallback(NULL)
{
}

void OpenTherm::begin(void(*handleInterruptCallback)(void), void(*processResponseCallback)(unsigned long, OpenThermResponseStatus))
{
	pinMode(inPin, INPUT);
	pinMode(outPin, OUTPUT);
	if (handleInterruptCallback != NULL) {
		this->handleInterruptCallback = handleInterruptCallback;
		attachInterrupt(digitalPinToInterrupt(inPin), handleInterruptCallback, CHANGE);
	}
	activateBoiler();
	status = OpenThermStatus::READY;
	this->processResponseCallback = processResponseCallback;
}

void OpenTherm::begin(void(*handleInterruptCallback)(void))
{
	begin(handleInterruptCallback, NULL);
}

bool ICACHE_RAM_ATTR OpenTherm::isReady()
{
	return status == OpenThermStatus::READY;
}

int ICACHE_RAM_ATTR OpenTherm::readState() {
	return digitalRead(inPin);
}

void OpenTherm::setActiveState() {
	digitalWrite(outPin, LOW);
}

void OpenTherm::setIdleState() {
	digitalWrite(outPin, HIGH);
}

void OpenTherm::activateBoiler() {
	setIdleState();
	delay(1000);
}

void OpenTherm::sendBit(bool high) {
	if (high) setActiveState(); else setIdleState();
	delayMicroseconds(500);
	if (high) setIdleState(); else setActiveState();
	delayMicroseconds(500);
}

bool OpenTherm::sendRequestAync(unsigned long request)
{
	//Serial.println("Request: " + String(request, HEX));
	noInterrupts();
	const bool ready = isReady();
	interrupts();

	if (!ready)
	  return false;

	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();

	status = OpenThermStatus::RESPONSE_WAITING;
	responseTimestamp = micros();
	return true;
}

unsigned long OpenTherm::sendRequest(unsigned long request)
{
	if (!sendRequestAync(request)) return 0;
	while (!isReady()) {
		process();
		yield();
	}
	return response;
}

bool OpenTherm::sendResponse(unsigned long request)
{
	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();
	status = OpenThermStatus::READY;
	return true;
}

unsigned long OpenTherm::getLastResponse()
{
	return response;
}

OpenThermResponseStatus OpenTherm::getLastResponseStatus()
{
	return responseStatus;
}

void ICACHE_RAM_ATTR OpenTherm::handleInterrupt()
{
	if (isReady())
	{
		if (isSlave && readState() == HIGH) {
		   status = OpenThermStatus::RESPONSE_WAITING;
		}
		else {
			return;
		}
	}

	unsigned long newTs = micros();
	if (status == OpenThermStatus::RESPONSE_WAITING) {
		if (readState() == HIGH) {
			status = OpenThermStatus::RESPONSE_START_BIT;
			responseTimestamp = newTs;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_START_BIT) {
		if ((newTs - responseTimestamp < 750) && readState() == LOW) {
			status = OpenThermStatus::RESPONSE_RECEIVING;
			responseTimestamp = newTs;
			responseBitIndex = 0;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_RECEIVING) {
		if ((newTs - responseTimestamp) > 750) {
			if (responseBitIndex < 32) {
				response = (response << 1) | !readState();
				responseTimestamp = newTs;
				responseBitIndex++;
			}
			else { //stop bit
				status = OpenThermStatus::RESPONSE_READY;
				responseTimestamp = newTs;
			}
		}
	}
}

void OpenTherm::process()
{
	noInterrupts();
	OpenThermStatus st = status;
	unsigned long ts = responseTimestamp;
	interrupts();

	if (st == OpenThermStatus::READY) return;
	unsigned long newTs = micros();
	if (st != OpenThermStatus::NOT_INITIALIZED && st != OpenThermStatus::DELAY && (newTs - ts) > 1000000) {
		status = OpenThermStatus::READY;
		responseStatus = OpenThermResponseStatus::TIMEOUT;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_INVALID) {
		status = OpenThermStatus::DELAY;
		responseStatus = OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_READY) {
		status = OpenThermStatus::DELAY;
		responseStatus = (isSlave ? isValidRequest(response) : isValidResponse(response)) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus);
		}
	}
	else if (st == OpenThermStatus::DELAY) {
		if ((newTs - ts) > 100000) {
			status = OpenThermStatus::READY;
		}
	}
}

bool OpenTherm::parity(unsigned long frame) //odd parity
{
	byte p = 0;
	while (frame > 0)
	{
		if (frame & 1) p++;
		frame = frame >> 1;
	}
	return (p & 1);
}

OpenThermMessageType OpenTherm::getMessageType(unsigned long message)
{
	OpenThermMessageType msg_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
	return msg_type;
}

OpenThermMessageID OpenTherm::getDataID(unsigned long frame)
{
	return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

unsigned long OpenTherm::buildRequest(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long request = data;
	if (type == OpenThermMessageType::WRITE_DATA) {
		request |= 1ul << 28;
	}
	request |= ((unsigned long)id) << 16;
	if (parity(request)) request |= (1ul << 31);
	return request;
}

unsigned long OpenTherm::buildResponse(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long response = data;
	response |= type << 28;
	response |= ((unsigned long)id) << 16;
	if (parity(response)) response |= (1ul << 31);
	return response;
}

bool OpenTherm::isValidResponse(unsigned long response)
{
	if (parity(response)) return false;
	byte msgType = (response << 1) >> 29;
	return msgType == READ_ACK || msgType == WRITE_ACK;
}

bool OpenTherm::isValidRequest(unsigned long request)
{
	if (parity(request)) return false;
	byte msgType = (request << 1) >> 29;
	return msgType == READ_DATA || msgType == WRITE_DATA;
}

void OpenTherm::end() {
	if (this->handleInterruptCallback != NULL) {
		detachInterrupt(digitalPinToInterrupt(inPin));
	}
}

const char *OpenTherm::statusToString(OpenThermResponseStatus status)
{
	switch (status) {
		case NONE:	return "NONE";
		case SUCCESS: return "SUCCESS";
		case INVALID: return "INVALID";
		case TIMEOUT: return "TIMEOUT";
		default:	  return "UNKNOWN";
	}
}

const char *OpenTherm::messageTypeToString(OpenThermMessageType message_type)
{
	switch (message_type) {
		case READ_DATA:	   return "READ_DATA";
		case WRITE_DATA:	  return "WRITE_DATA";
		case INVALID_DATA:	return "INVALID_DATA";
		case RESERVED:		return "RESERVED";
		case READ_ACK:		return "READ_ACK";
		case WRITE_ACK:	   return "WRITE_ACK";
		case DATA_INVALID:	return "DATA_INVALID";
		case UNKNOWN_DATA_ID: return "UNKNOWN_DATA_ID";
		default:			  return "UNKNOWN";
	}
}

//building requests

unsigned long OpenTherm::buildSetBoilerStatusRequest(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2) {
	unsigned int data = enableCentralHeating | (enableHotWater << 1) | (enableCooling << 2) | (enableOutsideTemperatureCompensation << 3) | (enableCentralHeating2 << 4);
	data <<= 8;
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Status, data);
}

unsigned long OpenTherm::buildSetBoilerTemperatureRequest(float temperature) {
	unsigned int data = temperatureToData(temperature);
	return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TSet, data);
}

unsigned long OpenTherm::buildGetBoilerTemperatureRequest() {
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tboiler, 0);
}

//parsing responses
bool OpenTherm::isFault(unsigned long response) {
	return response & 0x1;
}

bool OpenTherm::isCentralHeatingActive(unsigned long response) {
	return response & 0x2;
}

bool OpenTherm::isHotWaterActive(unsigned long response) {
	return response & 0x4;
}

bool OpenTherm::isFlameOn(unsigned long response) {
	return response & 0x8;
}

bool OpenTherm::isCoolingActive(unsigned long response) {
	return response & 0x10;
}

bool OpenTherm::isDiagnostic(unsigned long response) {
	return response & 0x40;
}

uint16_t OpenTherm::getUInt(const unsigned long response) const {
	const uint16_t u88 = response & 0xffff;
	return u88;
}

float OpenTherm::getFloat(const unsigned long response) const {
	const uint16_t u88 = getUInt(response);
	const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
	return f;
}

unsigned int OpenTherm::temperatureToData(float temperature) {
	if (temperature < 0) temperature = 0;
	if (temperature > 100) temperature = 100;
	unsigned int data = (unsigned int)(temperature * 256);
	return data;
}

//requests

//READ ONLY REQUESTS
//0 = Status, // flag8 / flag8  Master and Slave Status flags.

//3 = SConfigSMemberIDcode, // flag8 / u8  Slave Configuration Flags /  Slave MemberID Code

//5 = ASFflags, // / OEM-fault-code  flag8 / u8  Application-specific fault flags and OEM fault code
unsigned char OpenTherm::getFault() {
    return ((sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0)) >> 8) & 0xff);
}

//6 = RBPflags, // flag8 / flag8  Remote boiler parameter transfer-enable & read/write flags

//9 = TrOverride, // f8.8  Remote override room setpoint

//10 = TSP, // u8 / u8  Number of Transparent-Slave-Parameters supported by slave

//12 = FHBsize, // u8 / u8  Size of Fault-History-Buffer supported by slave

//13 = FHBindexFHBvalue, // u8 / u8  Index number / Value of referred-to fault-history buffer entry.

//15 = MaxCapacityMinModLevel, // u8 / u8  Maximum boiler capacity (kW) / Minimum boiler modulation level(%)

//17 = RelModLevel, // f8.8  Relative Modulation Level (%)
float OpenTherm::getModulation() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//18 = CHPressure, // f8.8  Water pressure in central heating circuit  (bar)
float OpenTherm::getPressure() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//19 = DHWFlowRate, // f8.8  Water flow rate in domestic hot water circuit. (litres/minute)
float OpenTherm::getDHWFlowrate() {
	unsigned long response = sendRequest(buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::DHWFlowRate, 0));
	return isValidResponse(response) ? getFloat(response) : 0;
}

//25 = Tboiler, // f8.8  Boiler flow water temperature (°C)
float OpenTherm::getBoilerTemperature() {
	unsigned long response = sendRequest(buildGetBoilerTemperatureRequest());
	return isValidResponse(response) ? getFloat(response) : 0;
}

//26 = Tdhw, // f8.8  domestic hot water temperature (°C)
float OpenTherm::getDHWTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//27 = Toutside, // f8.8  Outside temperature (°C)
float OpenTherm::getOutsideTemperature() {
	unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Toutside, 0));
	return isValidResponse(response) ? getFloat(response) : 0;
}

//28 = Tret, // f8.8  Return water temperature (°C)
float OpenTherm::getReturnTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//29 = Tstorage, // f8.8  Solar storage temperature (°C)
float OpenTherm::getSolarStorageTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tstorage, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//30 = Tcollector, // f8.8  Solar collector temperature (°C)
float OpenTherm::getSolarCollectorTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tcollector, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//31 = TflowCH2, // f8.8  Flow water temperature CH2 circuit (°C)
float OpenTherm::getCH2Temperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::TflowCH2, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//32 = Tdhw2, // f8.8  Domestic hot water temperature 2 (°C)
float OpenTherm::getDHW2Temperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tdhw2, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//33 = Texhaust, // s16  Boiler exhaust temperature (°C)
float OpenTherm::getExhaustTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Texhaust, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//48 = TdhwSetUBTdhwSetLB,  // s8 / s8  domestic hot water setpoint upper & lower bounds for adjustment  (°C)

//49 = MaxTSetUBMaxTSetLB, // s8 / s8  Max central heating water setpoint upper & lower bounds for adjustment  (°C)

//50 = HcratioUBHcratioLB, // s8 / s8  OTC heat curve ratio upper & lower bounds for adjustment

//100 = RemoteOverrideFunction, // flag8 / -  Function of manual and program changes in master and remote room setpoint.

//115 = OEMDiagnosticCode , // u16  OEM-specific diagnostic/service code

//125 = OpenThermVersionSlave, // f8.8  The implemented version of the OpenTherm Protocol Specification in the slave.
float OpenTherm::getOTSlaveVersion() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::OpenThermVersionSlave, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

//127 = SlaveVersion, // u8 / u8  Slave product version number and type
float OpenTherm::getSlaveVersion() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::SlaveVersion, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}




//WRITE ONLY REQUESTS
//1 = TSet, // f8.8  Control setpoint  ie central heating  water temperature setpoint (°C)
bool OpenTherm::setBoilerTemperature(float temperature) {
	unsigned long response = sendRequest(buildSetBoilerTemperatureRequest(temperature));
	return isValidResponse(response);
}

//2 = MConfigMMemberIDcode, // flag8 / u8  Master Configuration Flags /  Master MemberID Code

//4 = Command, // u8 / u8  Remote Command

//7 = CoolingControl, // f8.8  Cooling control signal (%)

//8 = TsetCH2, // f8.8  Control setpoint for 2e central heating circuit (°C)

//14 = MaxRelModLevelSetting, // f8.8  Maximum relative modulation level setting (%)

//16 = TrSet, // f8.8  Room Setpoint (°C)

//23 = TrSetCH2, // f8.8  Room Setpoint for 2nd central heating circuit (°C)

//24 = Tr, // f8.8  Room temperature (°C)

//124 = OpenThermVersionMaster, // f8.8  The implemented version of the OpenTherm Protocol Specification in the master.

//126 = MasterVersion, // u8 / u8  Master product version number and type



//READ-WRITE REQUESTS
//11 = TSPindexTSPvalue, // u8 / u8  Index number / Value of referred-to transparent slave parameter.

//20 = 	DayTime, // special / u8  Day of Week and Time of Day

//21 = Date, // u8 / u8  Calendar date

//22 = Year, // u16  Calendar year

//56 = TdhwSet = 56, // f8.8  domestic hot water setpoint (°C)    (Remote parameter 1)
bool OpenTherm::setDHWSetpoint(float temperature) {
    unsigned int data = temperatureToData(temperature);
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data));
    return isValidResponse(response);
}

//57= MaxTSet, // f8.8  Max central heating water setpoint (°C)  (Remote parameters 2)

//58 = Hcratio, // f8.8  OTC heat curve ratio (°C)  (Remote parameter 3) 6mm(3,15mm);2,5mm

//116 = BurnerStarts, // u16  Number of starts burner

//117 = CHPumpStarts, // u16  Number of starts central heating pump

//118 = DHWPumpValveStarts, // u16  Number of starts domestic hot water pump/valve

//119 = DHWBurnerStarts, // u16  Number of starts burner during domestic hot water mode

//120 = BurnerOperationHours, // u16  Number of hours that burner is in operation (i.e. flame on)

//121 = CHPumpOperationHours, // u16  Number of hours that central heating pump has been running

//122 = DHWPumpValveOperationHours, // u16  Number of hours that domestic hot water pump has been running or domestic hot water valve has been opened

//123 = DHWBurnerOperationHours, // u16  Number of hours that burner is in operation during domestic hot water mode




unsigned long OpenTherm::setBoilerStatus(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2) {
	return sendRequest(buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, enableCooling, enableOutsideTemperatureCompensation, enableCentralHeating2));
}
