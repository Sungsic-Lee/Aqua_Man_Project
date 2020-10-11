#include <SPI.h>
#include <Wire.h>
//NRF24
#include <RF24_config.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define SW  9
#define SPEAKER 41

//TF
#define I2C_SLAVE_ADDR1  0x10

//NRF24
RF24 radio(42, 43); // SPI 버스에 nRF24L01 라디오를 설정하기 위해 CE, CSN를 선언.
const byte address[][6] = { {0xe7, 0xe7, 0xe7, 0xe7, 0xe7}, {0xff, 0xff, 0xff, 0xff, 0xff} }; //주소값을 5가지 문자열로 변경할 수 있으며, 송신기와 수신기가 동일한 주소로 해야됨.
byte message[] = { 0x0A, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAE, 0x57 };
//TF




void setup() {
	char temp[30] = { 0, };
	Serial.begin(57600);    //DEBUG 시리얼
	Serial1.begin(57600);   //TAG 시리얼
	Serial2.begin(115200);  //LIDAR 시리얼
	Serial3.begin(19200);      //MPU 시리얼
	NRF24_SET();
	SPI.begin(); // SPI 통신 초기화
	digitalWrite(SS, HIGH); // 슬레이브가 선택되지 않은 상태로 유지
	SPI.setClockDivider(SPI_CLOCK_DIV16); // 안정적인 전송을 위해 분주비를 높여 전송 속도를 낮춤
	//radio.startListening();
	//GET_MPU(temp);
	pinMode(SW, INPUT_PULLUP);
	pinMode(SPEAKER, OUTPUT);

	MPU_SET();
}

void loop() {
	int switch_count = 0;
	unsigned long start_time = 0;
	unsigned long trigger_time = 0;
	volatile int live_anchor_count = 0;
	bool tag_flag = false;
	char MPU_DATA[30] = { 0, };   //Format : Z-Axis,X-Axis
	char TAG_DATA[4][10] = { 0, };    //Format : Anchor1,Anchor2,Anchor3,Anchor4,
	char LIDAR_DATA[10] = { 0, }; //Format : distance
	char SEND_DATA1[28] = { 0, };
	char SEND_DATA2[28] = { 0, };
	char SEND_DATA3[28] = { 0, };
	int Re[3] = { 0, };


	while (1) {
		if (digitalRead(SW) == HIGH)
		{
			delay(3);
			trigger_time = millis();
			if (digitalRead(SW) == LOW)
			{
				//Serial.println("HI");
				switch_count++;
				start_time = millis();

				while (digitalRead(SW) == LOW)
					delayMicroseconds(10);
				delay(50);
			}
			trigger_time = millis() - trigger_time;
			//Serial.println(switch_count);
		}
		if (trigger_time > 3000) {
			switch_count = -1;
			Serial.println("get bias");
		}
		else if (millis() - start_time > 500 && start_time != 0) {
			Serial.print("MODE : ");
			Serial.println(switch_count);
			Serial.println();
			break;
		}
	}



	if (switch_count == -1) {
		mpu_yawcalibration();
	}

	else if (switch_count == 1)
	{
		SPI.setClockDivider(SPI_CLOCK_DIV16);
		//Serial.println(MPU_DATA);
		live_anchor_count = GET_TAG(TAG_DATA);
		//Serial.println(live_anchor_count);
		GET_LIDAR(LIDAR_DATA);
		//Serial.println("hi");
		GET_MPU(MPU_DATA);

		MAKE_DATA(MPU_DATA, TAG_DATA, live_anchor_count, LIDAR_DATA, SEND_DATA1, SEND_DATA2, SEND_DATA3, tag_flag);
		Re[0] = NRF24_SEND(SEND_DATA1);
		Re[1] = NRF24_SEND(SEND_DATA2);
		Re[2] = NRF24_SEND(SEND_DATA3);
		Serial.println(Re[0]);
		Serial.println(Re[1]);
		Serial.println(Re[2]);
		//    Serial.println(NRF24_SEND(SEND_DATA1));
		//    Serial.println(NRF24_SEND(SEND_DATA2));
		//    Serial.println(NRF24_SEND(SEND_DATA3));
		send_sound(Re);
		delay(10);
	}

	else if (switch_count == 2) {
		bool result = 0;
		start_time = millis();
		while (result == 0) {

			result = NRF24_SEND("ssss");

			if ((millis() - start_time > 1000) || result == true) {
				break;
			}

		}
	}
}

void parsing(float* angle, String str) {
	String str_angle[3] = { "\0", "\0" , "\0" };
	int first = str.indexOf(',');
	int second = str.indexOf(',', first + 1);
	str_angle[0] = str.substring(0, first);
	str_angle[1] = str.substring(first + 1, second);
	str_angle[2] = str.substring(second + 1, str.length());

	for (int i = 0; i < 3; i++) {
		char temp[10] = "0, ";
		strcpy(temp, str_angle[i].c_str());
		angle[i] = atof(temp);
		//Serial.print(angle[i]);
		//Serial.print(", ");
	}
	//Serial.println();
}

void mpu_yawcalibration() {
	char nrf_sendData[28] = { 0, };
	String nrf_sendData_string = "\0";
	String cali_mpu_Raw = "\0";
	bool calibration_status = true;
	float cali_angle[3] = { 0, };
	unsigned long time_t = millis();
	float initial = 0, before = 0, rotate = 0;
	unsigned int final_count = 0;

	String error_data = "YAW Calibration ERROR:0";
	Serial.println("YAW Calibration START");

	Serial3.write('*');
	while (1) {
		if (Serial3.available()) {
			cali_mpu_Raw = Serial3.readStringUntil('\n');
			cali_mpu_Raw = &cali_mpu_Raw[1];
			break;
		}
	}
	if (cali_mpu_Raw.length() > 3) {
		parsing(cali_angle, cali_mpu_Raw);
		initial = cali_angle[2];
		Serial.print("initial YAW : ");
		Serial.println(initial);
		before = initial;
	}
	else {
		Serial.println("YAW calibration initial ERROR!");
		calibration_status = false;
	}

	tone(SPEAKER, 262, 1000);		//캘리 시작
	delay(1000);

	while (1) {
		if (!calibration_status) {
			strcpy(nrf_sendData, error_data.c_str());
			return ;
		}
		Serial3.write('*');
		time_t = millis();
		while (1) {
			if (Serial3.available()) {
				cali_mpu_Raw = Serial3.readStringUntil('\n');
				cali_mpu_Raw = &cali_mpu_Raw[1];
				break;
			}
			if ((millis() - time_t) > 500) {
				calibration_status = false;
				cali_mpu_Raw = "\0";
				Serial.println("YAW calibration timeout");
				break;
			}
		}

		if (cali_mpu_Raw.length() > 3) {
			parsing(cali_angle, cali_mpu_Raw);
			rotate = cali_angle[2] - before;
			before = cali_angle[2];

			nrf_sendData_string = "*";
			nrf_sendData_string.concat(initial);
			nrf_sendData_string.concat(" -> ");
			nrf_sendData_string.concat(cali_angle[2]);
			nrf_sendData_string.concat(", ");
			nrf_sendData_string.concat(rotate);
			nrf_sendData_string.concat("; ");
			//Serial.print(initial);
			//Serial.print(" -> ");
			//Serial.print(cali_angle[2]);
			//Serial.print(", ");
			//Serial.print(rotate);
			if (rotate < 0) {		// 반시계방향 회전
				if (((initial - cali_angle[2]) > -10) && (initial - cali_angle[2]) < 0) {
					//nrf_sendData_string.concat(" _ ");
					//nrf_sendData_string.concat(initial - cali_angle[2]);
					//Serial.print(" _ ");
					//Serial.print(initial - cali_angle[2]);

					if (((initial - cali_angle[2]) > -0.2) && ((initial - cali_angle[2]) < 0.2)) {	//초기값과 현재값 오차가 0.2도 미만일때
						final_count++;
						if (final_count > 3) {

							tone(SPEAKER, 262, 1000);
							delay(2000);
							calibration_status = false;
							nrf_sendData_string = "YAW Calibration data:";
							nrf_sendData_string.concat(initial);
							nrf_sendData_string.concat("^");
							strcpy(nrf_sendData, nrf_sendData_string.c_str());
							NRF24_SEND(nrf_sendData);
							return;
						}
					}

					else {
						tone(SPEAKER, 262, 50);
						delay(100);
						tone(SPEAKER, 262, 50);
						delay(100);
					}
				}
				else {
					tone(SPEAKER, 262, 10);
					delay(100);
				}
			}
			strcpy(nrf_sendData, nrf_sendData_string.c_str());
			bool status = NRF24_SEND(nrf_sendData);
			Serial.println(status);
		}

		else {
			Serial.println("YAW calibration ERROR!");
			calibration_status = false;
		}
	}
}

void NRF24_SET(void)
{
	Wire.begin(); //TF
	delay(100);
	radio.begin();
	radio.setChannel(108);
	radio.setPayloadSize(32);
	radio.openWritingPipe(address[0]); //이전에 설정한 5글자 문자열인 데이터를 보낼 수신의 주소를 설정
	radio.setPALevel(RF24_PA_MAX); //전원공급에 관한 파워레벨을 설정합니다. 모듈 사이가 가까우면 최소로 설정합니다.

	printf_begin();
	radio.printDetails();
	radio.startListening();
	radio.stopListening();  //모듈을 송신기로 설정

}

void MPU_SET(void) {
	String read = "\0";
	delay(1);

	Serial.print("MPU_SET : ");
	Serial3.write("<reset>");
	while (!(Serial3.available())) {

	}
	read = Serial3.readStringUntil('\n');
	Serial.println(read);
	read = "\0";
	delay(100);

	Serial.println();
	Serial.print("MPU_DATA : ");
	Serial3.write("*");
	while (!(Serial3.available())) {

	}
	read = Serial3.readStringUntil('\n');
	Serial.println(read);
	Serial.println();
	delay(1);
	/*
	  Serial3.write("<cas>");
	  while (!(Serial3.available())) {

	  }

	  Serial3.readBytesUntil('>', received, sizeof(received));
	  Serial.println(received);
	  Serial3.write("<cmco>");
	  while (!(Serial3.available())) {

	  }
	  Serial3.readBytesUntil('>', received, sizeof(received));
	  Serial.println(received);
	  Serial3.write("<cmo>");
	  while (!(Serial3.available())) {

	  }
	  Serial3.readBytesUntil('>', received, sizeof(received));
	  Serial.println(received);
	*/


	/*
	  Serial3.write("<caf>");
	  delay(5000);
	  Serial3.write("<caf>");

	  while (!(Serial3.read() == '<')) {
	  }

	  Serial3.readBytesUntil('>', received, sizeof(received));
	  Serial.println(received);
	*/
}

int GET_TAG(char received_data[][10])   //살아있는 앵커 개수 반환
{
	int count[4] = { 0, };
	double temp[4] = { 0, };
	double temp_dw[4] = { 0, };
	double number_distance[4] = { 0, };
	double min_distance[4] = { 10, 10, 10, 10 };
	double max_distance[4] = { 0, };
	double count_distance[4][3] = { 0, };
	char buffer[30] = { 0, };
	char not_dw[4][3] = { "#1","#2","#3","#4" };
	char temp_distance[4][10] = { 0, };
	int char_count = 0, anchor_count = 0;
	int counting = 0;
	int live_anc = 4;
	//  for (int k = 0; k < 5; k++)
	//  {

	Serial.print("GET_TAG : ");
	Serial1.write('1');

	while (!Serial1.available()) {
	}
	for (int i = 0; i < 4; i++) {
		Serial1.readBytesUntil('\n', buffer, 28);
		Serial.print(buffer);
		Serial.print(" ");
		number_distance[i] = atof(buffer);
		if (number_distance[i] > 0 && number_distance[i] < 10) {
			temp_dw[i] = number_distance[i];
			count[i]++;
		}
	}
	Serial.println();
	Serial.println();
	/*    for (int i = 0; i < 4; i++)
		{
		  Serial1.readBytesUntil('\n', buffer, 28);
		  //      Serial.print(i + 1);
		  //      Serial.print(": ");
		  //      Serial.println(buffer);

		  number_distance[k][i] = atof(buffer);
		  //Serial.print(" num : "); Serial.println(number_distance);

		  if (number_distance[k][i] > 0 && number_distance[k][i] < 10) {
			//        Serial.println(number_distance[k][i]);
			temp_dw[i] += number_distance[k][i];
			count[i]++;
			if (min_distance[i] > number_distance[k][i]) {
			  min_distance[i] = number_distance[k][i];
			}
			if (max_distance[i] < number_distance[k][i]) {
			  max_distance[i] = number_distance[k][i];
			}
		  }
		  //strcpy(all[i], buffer);
		  for (int j = 0; j < 30; j++)
		  {
			buffer[j] = 0;
		  }
		}
	  }

	*/
	for (int i = 0; i < 4; i++)
	{
		if (count[i] == 1) {
			dtostrf(temp_dw[i], 4, 2, temp_distance[i]);
			//      dtostrf(((temp_dw[i] - (min_distance[i] + max_distance[i])) / (count[i] - 2)), 5, 2, temp_distance[i]);
		}
		else {
			live_anc--;
			strcpy(temp_distance[i], not_dw[i]);
			//Serial.println(temp_distance[i]);
		}
	}

	for (int k = 0; k < 4; k++) {         //앵커 ++
		for (int j = 0; j < 10; j++) {        //문자 복사
			if (!(temp_distance[k][j] == 0)) {
				received_data[anchor_count][char_count++] = temp_distance[k][j];
			}
			else {

			}
		}

		//received_data[anchor_count++][char_count] = ',';
		anchor_count++;
		char_count = 0;
	}

	//  Serial.println(received_data[0]);

	return live_anc;
}

void GET_LIDAR(char* lidar_distance)
{
	char received[45] = { 0, };
	uint8_t data[45] = { 0. };
	float distance = 0;
	float average = 0;
	String final = "\0";
	unsigned int loop = 5;

	Serial.print("GET_LIDAR : ");

	for (int i = 0; i < loop; i++) {
		for (int i = 0; i < sizeof(received); i++) {
			received[i] = 0;
		}

		Serial2.write(message, sizeof(message));
		while (!(Serial2.available())) {
		}

		Serial2.readBytes(received, 15);
		for (int i = 0; i < sizeof(received); i++) {
			data[i] = (unsigned int)received[i];
		}
		distance = 0.001 * (data[5] * 256 + data[6]);
		//dtostrf(distance, 5, 1, lidar_distance);
		Serial.print(distance);
		Serial.print(" ");
		average += distance;

	}
	Serial.println();

	final = average / (float)loop;
	final.toCharArray(lidar_distance, final.length() + 1);
	Serial.print("AVG : ");
	Serial.println(lidar_distance);
	Serial.println();
}

void GET_MPU(char* received)
{
	unsigned int loop = 5;
	String mpu_raw = "\0";
	String X_angle_string = "\0", Z_angle_string = "\0";
	String final_data = "\0";
	float angle[3] = { 0, };
	float X_angle = 0, Z_angle = 0;
	float average = 0;
	//Serial.print("MPU Response : ");
	unsigned long time_t = millis();

	Serial.println("GET_MPU :");

	for (int i = 0; i < loop; i++) {
		Serial3.write('*');
		while (1) {
			if (Serial3.available()) {
				mpu_raw = Serial3.readStringUntil('\n');
				mpu_raw = &mpu_raw[1];
				break;
			}
			if ((millis() - time_t) > 200) {
				mpu_raw = "\0";
				break;
			}
		}
		if (mpu_raw.length() < 2) {
			Serial.println("timeout");
			
		}
		else if (mpu_raw.length() > 3) {
			//Serial.println(mpu_raw);
			parsing(angle, mpu_raw);
			X_angle += angle[0];
			Z_angle += angle[2];
			Serial.print(angle[0]);
			Serial.print(", ");
			Serial.println(angle[2]);
		}

	}
	X_angle /= (float)loop;
	Z_angle /= (float)loop;

	X_angle_string.concat(X_angle);
	Z_angle_string.concat(Z_angle);
	Serial.print("X : ");
	Serial.print(X_angle_string);
	Serial.print(", Y : ");
	Serial.println(Z_angle_string);
	final_data = X_angle_string;
	final_data.concat(",");
	final_data.concat(Z_angle_string);
	final_data.toCharArray(received, final_data.length() + 1);
}

void MAKE_DATA(char* MPU, char TAG[][10], int live_anchor, char* LIDAR, char* return1, char* return2, char* return3, bool tag_flag) {
	if (tag_flag == true) {
		return1[0] = '$';
		sprintf(LIDAR, "%s", "-3");
	}
	else {
		return1[0] = '@';
	}
	itoa(live_anchor, &return1[1], DEC);
	sprintf(&return1[2], ",%s,%s%c", LIDAR, TAG[0], '!');
	Serial.println();

	sprintf(return2, "%s,%s,%s%c", TAG[1], TAG[2], TAG[3], '!');

	sprintf(return3, "%s%c", MPU, '^');

}

bool NRF24_SEND(char* send_data)
{
	int send_result = 0;

	radio.stopListening();

	send_result = radio.write(send_data, strlen(send_data));
	Serial.println(send_data);
	radio.startListening();
	return send_result;
}
void send_sound(int* re) {
	if ((re[0] && re[1] && re[2]) == 1) {
		tone(SPEAKER, 262, 200);
	}
	delay(500);
}