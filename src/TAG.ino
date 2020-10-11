#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

#define Calibration 0
 // connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

#define Anchor1 11
#define Anchor2	12
#define Anchor3	13
#define Anchor4	14

#define TAG		5

#define this_device	TAG

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3

#define TIMEOUT_ERROR	253
#define MESSAGE_ERROR	254
#define RANGE_FAILED	255

//Antena Delay values
#define EXPECTED_RANGE 7.94 // Recommended value for default values, refer to chapter 8.3.1 of DW1000 User manual
#define EXPECTED_RANGE_EPSILON 0.05
#define ACCURACY_THRESHOLD 5
#define ANTENNA_DELAY_STEPS 1

// Antenna calibration variables
int accuracyCounter = 0;
uint16_t antenna_delay = 16456;

// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

uint64_t timeComputedRange;
// last computed range/time
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 50;			//¿ø·¡ 250
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
float ReceivePower = 0;
bool reset_flag = false;
//float distance = 0;

#define filter_size	10

float antennaDelay_Tag = 0;

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
	delay(1000);
	//Serial.println(F("### DW1000Ng-arduino-ranging-TAG ###"));
	// initialize the driver
	DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
	//Serial.println(F("DW1000Ng initialized ..."));
	// general configuration
	DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

	DW1000Ng::setDeviceAddress(1);

	DW1000Ng::setAntennaDelay(antenna_delay);

	//Serial.println(F("Committed configuration ..."));
	// DEBUG chip info and registers pretty printed
	char msg[128];
	DW1000Ng::getPrintableDeviceIdentifier(msg);
	Serial.print("Device ID: "); Serial.println(msg);
	DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
	//Serial.print("Unique ID: "); Serial.println(msg);
	DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
	//Serial.print("Network ID & Device Address: "); Serial.println(msg);
	DW1000Ng::getPrintableDeviceMode(msg);
	//Serial.print("Device mode: "); Serial.println(msg);
	// attach callback for (successfully) sent and received messages
	DW1000Ng::attachSentHandler(handleSent);
	DW1000Ng::attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message
	
	//receiver();
	//noteActivity();
	// for first time ranging frequency computation
	
	rangingCountPeriod = millis();
}

void loop() {
	float all_distance[filter_size][4] = { 0, };
	float temp_distance[4][filter_size] = { 0, };
	float large_distance = 0, small_distance = 0;
	float temp = 0;
	int temp_anchor = 0, temp_filter = 0;
	float filtered_distance[4] = { 0, };
	//Serial.println("START");
	//DW1000Ng::reset();
	bool status = false;
	char command[10] = { 0, };
	int command_int = 0;
	//DW1000Ng::startTransmit();
/*	if (reset_flag) {
		reset_flag = false;
		sentAck = false;
		receivedAck = false;
		Serial.println("reset");
	}
	*/
	if (Serial.available()) { 
		Serial.readBytesUntil('\n', command, sizeof(command));
		command_int = atoi(command);
	}

	if (command_int == 1) {
		for (int j = 0; j < filter_size; j++) {
			for (int i = Anchor1; i <= Anchor4; i++) {
				//Serial.println(i);
				status = rainging_anchor(all_distance[j], i);
				//delay(10);

				if (status) {
					//all_distance[i - Anchor1] = distance;
					//Serial.print("anchor : ");
					//Serial.println(i-Anchor1);

				}
				else
				{
					all_distance[j][i - Anchor1] = 0;
				}
				//else {
				//	transmitERROR(transmitRangeFailed);
				//}
				//delay(100);
				//transmitERROR(TIMEOUT_ERROR);

			}

			/*
				for (int i = 0; i < 4; i++) {
					Serial.print("Anchor "); Serial.print(i + 1); Serial.print(" : ");
					Serial.print(all_distance[j][i]); Serial.print('\t');
				}
				Serial.println();
			}
			*/

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < filter_size; j++) {
					temp_distance[i][j] = all_distance[j][i];
				}
			}

			for (int anchor_count = 0; anchor_count < 4; anchor_count++) {
				for (int i = 0; i < filter_size; i++) {
					for (int filter_count = 0; filter_count < filter_size - 1; filter_count++) {
						if ((temp_distance[anchor_count][filter_count]) > temp_distance[anchor_count][filter_count + 1]) {
							temp = temp_distance[anchor_count][filter_count];
							temp_distance[anchor_count][filter_count] = temp_distance[anchor_count][filter_count + 1];
							temp_distance[anchor_count][filter_count + 1] = temp;
						}
					}
				}
				filtered_distance[anchor_count] = temp_distance[anchor_count][4] + temp_distance[anchor_count][5] + temp_distance[anchor_count][6];
				filtered_distance[anchor_count] /= 3;
			}

			/*	for (int i = 0; i < 4; i++) {
					for (int j = 0; j < filter_size; j++) {
						Serial.print(temp_distance[i][j]); Serial.print(" ");
					}
					Serial.println();
			*/
		}


		for (int i = 0; i < 4; i++) {
			if (i == 0 || i == 3) {
				Serial.print(filtered_distance[i]); Serial.println("!");
			}
			else {
				Serial.print(filtered_distance[i]); Serial.println(",");
			}
		}
		//Serial.println();
	}
}


bool rainging_anchor(float *range, byte anchor) {
	float distance = 0;
	transmitAnchor(anchor);
	noteActivity();
	uint32_t curMillis;
	bool count = 0;
	while (1) {
		curMillis = millis();
		if (!sentAck && !receivedAck) {
			// check if inactive
			
			if (curMillis - lastActivity > resetPeriod) {
				transmitERROR(TIMEOUT_ERROR);
				resetInactive();
				//Serial.println("timeout");
				//DW1000Ng::startTransmit();
				//delay(3);
				return false;
			}
			if ((curMillis - lastActivity > resetPeriod / 2) && (expectedMsgId == POLL) && (count == 0)) {
				resetInactive();
				//Serial.println("retry");
				count = true;
				transmitAnchor(anchor);
				noteActivity();
			}

		}
		//Serial.print("!");
		// continue on any success confirmation
		if (sentAck) {
			sentAck = false;
			//DW1000Ng::startReceive();
			receiver();
			byte msgId = data[0];
			//Serial.print("SENT : ");	Serial.println(msgId);
			if (msgId == POLL_ACK) {
				timePollAckSent = DW1000Ng::getTransmitTimestamp();
				noteActivity();
			}
			if (msgId == RANGE_REPORT) {
				curMillis = millis();
				//delay(50);
				resetInactive();
				range[anchor - Anchor1] = distance;
				return true;
			}
		
		}
		if (receivedAck) {
			receivedAck = false;
			// get message and parse
			DW1000Ng::getReceivedData(data, LEN_DATA);
			byte msgId = data[0];
			//Serial.print("Recived : "); Serial.println(msgId);
			if (msgId != expectedMsgId) {
				// unexpected message, start over again (except if already POLL)
				protocolFailed = true;
				transmitERROR(MESSAGE_ERROR);
				return false;
			}
			if (msgId == POLL) {
				// on POLL we (re-)start, so no protocol failure
				protocolFailed = false;
				timePollReceived = DW1000Ng::getReceiveTimestamp();
				expectedMsgId = RANGE;
				transmitPollAck();
				noteActivity();
			}
			else if (msgId == RANGE) {
				timeRangeReceived = DW1000Ng::getReceiveTimestamp();
				expectedMsgId = POLL;
				if (!protocolFailed) {
					timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
					timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
					timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
					// (re-)compute range as two-way ranging is done


					distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
						timePollReceived,
						timePollAckSent,
						timePollAckReceived,
						timeRangeSent,
						timeRangeReceived);
					//Serial.println("HI");
					//Serial.println(distance);
					//char temp[300] = { 0, };
					//Serial.println();

					/* Apply simple bias correction */
					distance = DW1000NgRanging::correctRange(distance);
					//all_distance[anchor - Anchor1] = distance;
					//Serial.println(distance);
					ReceivePower = DW1000Ng::getReceivePower();
					//String rangeString = "Range: "; rangeString += distance; rangeString += " m";
					//rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
					//rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
					//Serial.println(rangeString);
					//Serial.print("FP power is [dBm]: "); Serial.print(DW1000Ng::getFirstPathPower());
					//Serial.print("RX power is [dBm]: "); Serial.println(DW1000Ng::getReceivePower());
					//Serial.print("Receive quality: "); Serial.println(DW1000Ng::getReceiveQuality());
					// update sampling rate (each second)
					transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
					
					successRangingCount++;
					if (curMillis - rangingCountPeriod > 1000) {
						samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
						rangingCountPeriod = curMillis;
						successRangingCount = 0;
					}
					
				}
				else {
					transmitRangeFailed();
					return false;
				}

				noteActivity();
			}
		}
	}
}


void noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	lastActivity = millis();
}

void resetInactive() {
	reset_flag = true;
	// anchor listens for POLL
	DW1000Ng::forceTRxOff();
	expectedMsgId = POLL;
	//receiver();
	noteActivity();
}

void handleSent() {
	// status change on sent success
	sentAck = true;
}

void handleReceived() {
	// status change on received success
	receivedAck = true;
}

void transmitAnchor(byte Anchor) {
	data[0] = Anchor;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
	//Serial.print("trans Anchor");
}

void transmitPollAck() {
	data[0] = POLL_ACK;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange) {
	data[0] = RANGE_REPORT;
	// write final ranging result
	memcpy(data + 1, &curRange, 4);
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
}

void transmitRangeFailed() {
	data[0] = RANGE_FAILED;
	DW1000Ng::setTransmitData(data, LEN_DATA);
	DW1000Ng::startTransmit();
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