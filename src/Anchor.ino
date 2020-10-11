#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

#define Anchor1 11
#define Anchor2	12
#define Anchor3	13
#define Anchor4	14
#define Anchor5 15

#define TAG		5

#define this_device	Anchor4

#define scanId	99

#define antenna_delay	16442

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3

#define TIMEOUT_ERROR	253
#define MESSAGE_ERROR	254
#define RANGE_FAILED	255
// message flow state
volatile byte expectedMsgId = this_device;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;

// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 30;		//원래 250
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

bool reset_flag = false;

device_configuration_t DEFAULT_CONFIG = {
	false,
	true,
	true,
	true,
	false,
	SFDMode::STANDARD_SFD,
	Channel::CHANNEL_5,
	DataRate::RATE_850KBPS,
	PulseFrequency::FREQ_16MHZ,
	PreambleLength::LEN_256,
	PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
	true,
	true,
	true,
	false,
	true
};

void setup() {
	// DEBUG monitoring
	Serial.begin(57600);
	Serial.print(F("### DW1000Ng-arduino-ranging-Anchor ###"));
	Serial.println(this_device);
	// initialize the driver
	DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
	Serial.println("DW1000Ng initialized ...");
	// general configuration
	DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

	DW1000Ng::setNetworkId(10);

	DW1000Ng::setAntennaDelay(antenna_delay);

	Serial.println(F("Committed configuration ..."));
	// DEBUG chip info and registers pretty printed
	char msg[128];
	DW1000Ng::getPrintableDeviceIdentifier(msg);
	Serial.print("Device ID: "); Serial.println(msg);
	DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
	Serial.print("Unique ID: "); Serial.println(msg);
	DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
	Serial.print("Network ID & Device Address: "); Serial.println(msg);
	DW1000Ng::getPrintableDeviceMode(msg);
	Serial.print("Device mode: "); Serial.println(msg);
	// attach callback for (successfully) sent and received messages
	DW1000Ng::attachSentHandler(handleSent);
	DW1000Ng::attachReceivedHandler(handleReceived);
	// anchor starts by transmitting a POLL message
	//DW1000Ng::startReceive();
	receiver();
	noteActivity();
}

void loop() {

	bool status = false;
	/*	if (reset_flag) {
			reset_flag = false;
			expectedMsgId = this_device;
			sentAck = false;
			receivedAck = false;
			Serial.println("reset");
		}
		*/
	if (!sentAck && !receivedAck) {
		if (millis() - lastActivity > resetPeriod) {
			resetInactive();
			receiver();
			noteActivity();
		}
		return;

		//Serial.println("HI");
		//delay(100);

	}
	if (sentAck) {
		sentAck = false;
		//DW1000Ng::startReceive();

	}
	if (receivedAck) {
		receivedAck = false;
		// get message and parse
		DW1000Ng::getReceivedData(data, LEN_DATA);
		byte msgId = data[0];
		Serial.println(msgId);
		if (msgId != expectedMsgId) {
			// unexpected message, start over again
			//Serial.print("Received wrong message # "); Serial.println(msgId);
			expectedMsgId = this_device;
			//Serial.println("RESET");
			//return;
		}
		if (msgId == scanId) {
			Serial.println("Sent ID!");
			transmitID();
			noteActivity();
		}
		if (msgId == this_device) {
			//Serial.println("HI!");
			expectedMsgId = POLL_ACK;
			transmitPoll();
			//receiver();
			noteActivity();
			status = matching_anchor();
			if (!status) {
				noteActivity();
				//resetInactive();
				//DW1000Ng::reset();
				//Serial.println("error");
				//return;
			}
		}
	}
	//delay(10);
}

bool matching_anchor(void) {
	//Serial.println("HI");
	while (1) {
		if (!sentAck && !receivedAck) {
			if (millis() - lastActivity > resetPeriod) {
				Serial.println("timeout");
				transmitERROR(TIMEOUT_ERROR);
				resetInactive();
				return false;
			}

		}
		// continue on any success confirmation
		if (sentAck) {
			sentAck = false;
			//DW1000Ng::startReceive();
			receiver();
			Serial.println("sent");
		}
		if (receivedAck) {
			receivedAck = false;
			// get message and parse
			DW1000Ng::getReceivedData(data, LEN_DATA);
			byte msgId = data[0];
			Serial.println(msgId);
			if (msgId != expectedMsgId) {
				// unexpected message, start over again
				//Serial.print("Received wrong message # "); Serial.println(msgId);
				//expectedMsgId = POLL_ACK;
				//transmitPoll();
				transmitERROR(MESSAGE_ERROR);
				return;
			}
			if (msgId == POLL_ACK) {
				timePollSent = DW1000Ng::getTransmitTimestamp();
				timePollAckReceived = DW1000Ng::getReceiveTimestamp();
				expectedMsgId = RANGE_REPORT;
				transmitRange();
				noteActivity();
			}
			else if (msgId == RANGE_REPORT) {
				expectedMsgId = this_device;
				float curRange;
				memcpy(&curRange, data + 1, 4);
				//transmitPoll();
				//noteActivity();
				return true;
			}
			else if (msgId == RANGE_FAILED) {
				//expectedMsgId = POLL_ACK;
				//transmitPoll();
				//noteActivity();
				return false;
			}
		}
	}
}

void noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	lastActivity = millis();
}

void resetInactive() {
	// tag sends POLL and listens for POLL_ACK
	reset_flag = true;
	expectedMsgId = this_device;
	DW1000Ng::forceTRxOff();
	//transmitPoll();
	//noteActivity();
}

void handleSent() {
	// status change on sent success
	sentAck = true;
}

void handleReceived() {
	// status change on received success
	receivedAck = true;
}

void transmitID() {
	data[0] = this_device;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
}

void transmitPoll() {
	data[0] = POLL;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
}

void transmitRange() {
	data[0] = RANGE;

	/* Calculation of future time */
	byte futureTimeBytes[LENGTH_TIMESTAMP];

	timeRangeSent = DW1000Ng::getSystemTimestamp();
	timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
	DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
	DW1000Ng::setDelayedTRX(futureTimeBytes);
	timeRangeSent += DW1000Ng::getTxAntennaDelay();

	DW1000NgUtils::writeValueToBytes(data + 1, timePollSent, LENGTH_TIMESTAMP);
	DW1000NgUtils::writeValueToBytes(data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
	DW1000NgUtils::writeValueToBytes(data + 11, timeRangeSent, LENGTH_TIMESTAMP);
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit(TransmitMode::DELAYED);
	//Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}
void receiver() {
	DW1000Ng::forceTRxOff();
	// so we don't need to restart the receiver manually
	DW1000Ng::startReceive();
}
void transmitERROR(bool type) {
	data[0] = type;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
}
