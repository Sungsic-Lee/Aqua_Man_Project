#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <wire.h>
#include <math.h>

SoftwareSerial mySerial(10, 9);  //RX TX
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 단위는 무조건 mm, sec
#define ENCODER_PIN1 3 //1번모터 엔코더

#define STACK_SIZE 50
#define CUT_MOUNTAIN 10

#define GAP_MAX 180
#define GAP_MIN (-180)
#define LOC_OFFSET 3
#define LOC_GAP 20
#define MAX_desired_pwm 100
#define MIN_desired_pwm 0

#define GEAR_R 15.1
#define DEGREE2RADIAN 0.0174533

#define MP1 5
#define MP2 6
#define Direction 2
#define hollSensor1 1  //R 홀센서
#define hollSensor2 2
#define solenoidPin1 7
#define solenoidPin2 8

#define GAP_BIAS 0
#define MOTOR2_BIAS 0
#define PID_GAP 3
#define STR_ARR_SIZE 30
#define DIFF 1//좌우

/*엔코더*/
float encoder_gap = 0;
float degree, angle_raw, location, velocity, angle_bias = 0;
float loc_arr[STACK_SIZE];
unsigned long time_arr[STACK_SIZE];
float angle_gap, gap_aver;
int loc = 0;


/***********************
 *   low pass filter   *
 ***********************/
unsigned long thisMicros_old[2];
// {angle_raw, velocity}
float fc[2] = { 100, 1 }; // cutoff frequency
float x[2] = { 0.0, 0.0 };
float x_f[2] = { 0.0, 0.0 };
float x_fold[2] = { 0.0, 0.0 };

/***********************
 *   average filter   *
 ***********************/
// {angle_raw, velocity}
#define AVERAGE_SIZE 10
int average_loc[3] = { 0, };
float x_average[3][AVERAGE_SIZE] = { 0.0, };
float average_sum[3] = { 0.0 , };

float desired_pwm = 0; //설정값
float trans_pwm_data = 0;
int current_PWM = 0;
int PWM_output = 0;
unsigned long PWM_count = 0;

float reduction_rate = 0;
bool Dir = LOW;   //Right to Left
int time_check = micros();

/*시간 측정*/
float now = 0;
float pre = 0;

/*서보모터 데이터*/
float angle_data[82] = { 0, 1.2, 4.8, 7.2, 9.2, 11.9, 14.2, 16.5, 18.9, 21, 22.5, 26, 27.5, 30, 32.5, 34.5, 37.9, 38.9, 40.9, 42.9, 45, 47.9, 49.9, 51.9, 53.9,
    56, 58.2, 60.2, 62.9, 65.5, 67.3, 70.3, 72.3, 73.9, 76.9, 78, 80.5, 82.6, 84.5, 86.5, 88.5, 90.5, 92.5, 94.2, 96.2, 98.2, 100.5, 102.5, 104.9, 108.3,
    110.5, 111.5, 114.5, 117, 119.5, 122.5, 124, 125.5, 127.5, 130.5, 132.9, 135, 137, 140.3, 142.3, 144.5, 145.9, 149, 151, 153.3, 155.3, 156.9, 160, 162.5,
    164.5, 166, 167.8, 170.9, 173.7, 175.9, 177.5, 181 };
float desired_angle_x = 0;
float desired_angle_y = 45; //214; // 214 = 45도, 246 = 60도? 310 = 약 90도.
bool head_direction = true;

/*serial통신*/
int data_count = 0;
char receive[30] = { 0, };
int start_t = 0;
bool serial_flag = false, data_complete = false;
float xyv[3];

bool working = false;
bool wait = false;

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    mySerial.setTimeout(1000);
    
    pinMode(ENCODER_PIN1, INPUT);
    pinMode(MP1, OUTPUT);
    pinMode(MP2, OUTPUT);
    pinMode(solenoidPin1, OUTPUT);
    pinMode(solenoidPin2, OUTPUT);
    pinMode(Direction, OUTPUT);
    pinMode(MISO, OUTPUT);
    
    //init//
    pwm.begin();
    pwm.setPWMFreq(55);
    setting_motor();
    setting_encoder();
}

void loop() {
    receive_data();
    
    if (data_complete) {
        initial_status();
        read_data();            // 속도, 각도 2개 읽기
        //desired_angle_y = 214; // 45도
        servo_control();
        delay(1000);
    }
    
    if (working) {
        read_encoder();       // 속도, 위치 계산
        if (millis() - PWM_count > 5) {
            motor_control();        // 메인모터 제어
            PWM_count = millis();
            //print_plotter();
            //print_state();
        }
        holl_solenoid_handle(); // 홀센서 체크
    }
    else {
        doridori();
    }
    delay(1);
}

void receive_data(void) {
    if (mySerial.available()) {
        char tok = mySerial.read();       //mySerial -> Serial
        if (tok == '@') {
            reload();   // 소방차 멈췄을 때 1초간 역회전 주어 물을 빨아들인다.
            desired_pwm = 0;
            serial_flag = false;
            working = false;
            wait = true;
        }
        if (tok == '#') {
            serial_flag = true;
            data_complete = false;
            data_count = 0;
        }
        if (serial_flag) {
            receive[data_count++] = tok;
        }
        if (tok == ';') {
            serial_flag = false;
            data_complete = true;
            working = true;
            
            convert_data();
            
            for (int i = 0; i < 30; i++) {
                receive[i] = 0;
            }
        }
    }
}

void read_data(void) {       //desired_pwm, servo_x, servo_y 값 불러오기
    if (data_complete) {
        desired_angle_x = xyv[0];
        
        desired_angle_y = xyv[1];
       desired_pwm = pow(xyv[2]/1000 + 1.67495, 2) * 0.563452  + 8.54087;
        //desired_pwm = xyv[2];
        if (desired_pwm > MAX_desired_pwm) {
            Serial.println("OVER MAX_desired_pwm!");
            working = false;
        }
        else if (desired_pwm < MIN_desired_pwm) {
            Serial.println("UNDER MIN_desired_pwm!");
            working = false;
        }
        else {
            Serial.println("\nconvert data");
             Serial.print("angle_x : "); Serial.println(xyv[0]);
             Serial.print("angle_y : "); Serial.println(xyv[1]);
             Serial.print("water velocity : "); Serial.print(xyv[2]); Serial.println("mm/sec");
             Serial.print("desired_pwm : "); Serial.print(desired_pwm); Serial.println("mm/sec");
             
        }
        
        data_complete = false;
    }
}

void convert_data(void) {
    int index = 1, i, j, count;
    float data;
    bool flag;
    
    for (i = 0; i < 3; i++) {
        count = data = 0;
        flag = false;
        while (!(receive[index] == ',' || receive[index] == ';')) {
            if (receive[index] == '.') {
                flag = true;
            }
            else {
                data *= 10;
                data += receive[index] - '0';
            }
            if (flag) {
                count++;
            }
            index++;
        }
        index++;
        for (j = 1; j < count; j++) {
            data /= 10;
        }
        
        xyv[i] = data;
    }
}

void servo_control() {
    int i = 0;
    int now_index_x, now_index_y = 0;
    float analog_value_x, analog_value_y = 0;
    if (desired_angle_x == 0)
    {
        analog_value_x = 103;
        //Serial.print("now index_x : "); Serial.println(now_index_x);
    }
    else {
        while (angle_data[i] < desired_angle_x) {
            i++;
            
            if (angle_data[i] >= desired_angle_x) {
                now_index_x = i;
                //Serial.print("now index_x : "); Serial.println(now_index_x);
                break;
            }
        }
        analog_value_x = round((((desired_angle_x - angle_data[now_index_x - 1]) / (angle_data[now_index_x] - angle_data[now_index_x - 1])) * 5) + 103 + 5 * (now_index_x - 1));
    }

    
    i = 0;
    desired_angle_y = 90 - desired_angle_y;
    
    if (desired_angle_y == 0)
    {
        analog_value_y = 103;
        //Serial.print("now index_y : "); Serial.println(now_index_y);
    }
    else {
        while (angle_data[i] < desired_angle_y) {
            i++;
            
            if (angle_data[i] >= desired_angle_y) {
                now_index_y = i;
                //Serial.print("now index_y : "); Serial.println(now_index_y);
                break;
            }
        }
        analog_value_y = round((((desired_angle_y - angle_data[now_index_y - 1]) / (angle_data[now_index_y] - angle_data[now_index_y - 1])) * 5) + 103 + 5 * (now_index_y - 1));
    }
    //Serial.print("analog_value : "); Serial.println(analog_value);
    
    pwm.setPWM(3, 0, (int)analog_value_x + 6);
    pwm.setPWM(2, 0, (int)analog_value_y + 11);
    desired_angle_y = 90 - desired_angle_y;
}

void doridori(void) {
    if (wait) {
        initial_status();
        delay(5000);
        wait = false;
    }
    else {
        desired_pwm = 0;
        do {
            holl_solenoid_handle();
            motor_control();
        } while (current_PWM != 0);
        
        if (head_direction)
            desired_angle_y += 1;
        
        else
            desired_angle_y -= 1;
        
        if (desired_angle_y > 55)
            head_direction = false;
        
        else if (desired_angle_y < 35)
            head_direction = true;

        desired_angle_x = 0;
        servo_control();
    }
    delay(20);
}

void read_encoder(void) {   // 속도, 위치 계산
    cal_encoder();
}

void motor_control(void) {  // 메인모터 제어
    int temp_pwm;
    if(Dir){
        temp_pwm = trans_pwm(desired_pwm * DIFF);
    }
    else{
        temp_pwm = -trans_pwm(desired_pwm);
    }
    if (temp_pwm - current_PWM > PID_GAP) {         //PID값이 급격하게 증가할때 PID_GAP만큼만 증가시킨다.
        PWM_output = current_PWM + PID_GAP;
    }
    else if (temp_pwm - current_PWM < -PID_GAP) {  //PID값이 급격하게 감소할때 PID_GAP만큼만 감소시킨다.
        PWM_output = current_PWM - PID_GAP;
    }
    else {
        PWM_output = temp_pwm;
    }
    motor_act(PWM_output, PWM_output);
    current_PWM = PWM_output;
}

int trans_pwm(float input_pwm){
    float temp_pwm = input_pwm, val = (float)analogRead(6);
    temp_pwm *= 502.91 / constrain(val, 100.0, 502.91);
    trans_pwm_data += (temp_pwm - floor(temp_pwm));
    if (trans_pwm_data > 1) {
        trans_pwm_data -= 1;
        return (int)(temp_pwm + 1);
    }
    else {
        return (int)(temp_pwm);
    }
}

void initial_status(void) {
    desired_pwm = 0;
    do {
        holl_solenoid_handle();
        motor_control();
    } while (current_PWM != 0);
    desired_angle_x = 0;
}

void print_state(void) {
    Serial.println("-------------------------------------");
    //Serial.print(" degree1  : "); Serial.println(degree);
    //Serial.print("location1 : "); Serial.print(location); Serial.println(" mm");
    Serial.print("velocity1 : "); Serial.print(velocity); Serial.println(" mm/s\n");
    //Serial.print(" current_PWM  : "); Serial.println(current_PWM);
    //Serial.println();
    //Serial.print("hollSensor1 : "); Serial.println(analogRead(hollSensor1));
    //Serial.print("hollSensor2 : "); Serial.println(analogRead(hollSensor2));
    Serial.print("Direction : "); Serial.println(Dir);
}

void print_plotter(void) {
    Serial.print(velocity);
    Serial.print(" ");
    Serial.print(current_PWM);
    Serial.print(" ");
    if (Dir == HIGH) {
        Serial.print(desired_pwm);
    }
    else {
        Serial.print(-desired_pwm);
    }
    Serial.println(" 0.0 ");
}

void motor_act(int pwm1, int pwm2) {
    if (pwm1 > 0) {
        digitalWrite(Direction, LOW); // pin입력 LOW일때 시계방향 (양의 방향)
    }
    else {
        digitalWrite(Direction, HIGH);
    }
    analogWrite(MP1, constrain(abs(pwm1), 0, 255));
    analogWrite(MP2, constrain(abs(pwm2) - MOTOR2_BIAS, 0, 255));
}

void holl_solenoid_handle() {
    if (location < LOC_GAP) {//중간판과 끝과의 갭이 LOC_GAP(mm) 이면 방향바꿔 회전한다.
        Dir = HIGH;
    }
    if (location > 140 - LOC_GAP) {
        Dir = LOW;
    }
    if (analogRead(hollSensor1) < 50 && Dir == LOW) //홀센서 1이나 2에 닿으면 모터회전 멈춘다. (왼쪽 센서)
    {
        motor_act(0, 0);
        desired_pwm = 0;
    }
    if (analogRead(hollSensor2) < 50 && Dir == HIGH)
    {
        motor_act(0, 0);
        desired_pwm = 0;
    }
    if (velocity > 1){
        digitalWrite(solenoidPin1, LOW);// 오른쪽 솔밸브 두개가 pin1에 결합되어야 한다
        digitalWrite(solenoidPin2, HIGH);
    }
    else if (velocity < -1){
        digitalWrite(solenoidPin1, HIGH);
        digitalWrite(solenoidPin2, LOW);
    }
}

void setting_motor(void) {
    desired_pwm = 30;
    Dir = LOW;
    digitalWrite(solenoidPin1, LOW);
    digitalWrite(solenoidPin2, LOW);//세팅 단계에서 물 발사되지않도록.
    while (analogRead(hollSensor1) > 50) {
        motor_control();
    }
    desired_pwm = 0;
}

void setting_encoder(void) {
    for (int i = 0; i < STACK_SIZE; i++) {
        cal_encoder();
        angle_bias += angle_raw;
    }
    for (int i = 0; i < STACK_SIZE; i++) {
        loc_arr[i] = LOC_OFFSET;
        
    }
    angle_bias /= STACK_SIZE;
    location = LOC_OFFSET;
}

void cal_encoder(void) {
    float last_degree = degree;
    float temp_gap;
    
    cal_degree();
    temp_gap = degree - last_degree;
    Serial.println(temp_gap);
    if (temp_gap > GAP_MAX) {
        temp_gap -= 360;
    }
    else if (temp_gap < GAP_MIN) {
        temp_gap += 360;
    }
    if (-30 < temp_gap && temp_gap < 30) {
        angle_gap = temp_gap;
        angle_raw += angle_gap;
    }
    else {
        angle_raw += angle_gap;
    }
    //angle_raw = low_pass_filter(angle_raw, 0);
    location = GEAR_R * DEGREE2RADIAN * (angle_raw - angle_bias) + LOC_OFFSET;
    loc_arr[loc] = location;
    
    velocity = 1000000;
    velocity *= loc_arr[loc] - loc_arr[mod(loc + 1, STACK_SIZE)];
    velocity /= (float)(time_arr[loc] - time_arr[mod(loc + 1, STACK_SIZE)]);
    
    //    velocity = 1000000 * (loc_arr[loc] - loc_arr[mod(loc + 1, STACK_SIZE)]);
    //    velocity /= (float)(time_arr[loc] - time_arr[mod(loc + 1, STACK_SIZE)]);
    
    //velocity = low_pass_filter(velocity, 1);
    velocity = average_filter(velocity, 1);
    velocity = low_pass_filter(velocity, 1);
    
    loc = (loc + 1) % STACK_SIZE;
}
void cal_degree(void) {
    float encoder_gap_near_zero, encoder_gap_near_circle = 0;
    encoder_gap_near_zero = (float)pulseIn(ENCODER_PIN1, HIGH, 3000);
    encoder_gap_near_circle = 879.8 - (float)pulseIn(ENCODER_PIN1, LOW, 3000);
    
    if (encoder_gap_near_zero < 400) {
        degree = constrain(360 * (encoder_gap_near_zero - 4) / 873, 0, 360);
    }
    else {
        degree = constrain(360 * (encoder_gap_near_circle - 4) / 873, 0, 360);
    }
    time_arr[loc] = micros();
}

float low_pass_filter(float analog_data, int data_num) {
    unsigned long deltaMicros = 0; // clear last result
    unsigned long thisMicros = micros();
    float lambda;
    
    if (thisMicros != thisMicros_old[data_num]) {
        deltaMicros = thisMicros - thisMicros_old[data_num];
        thisMicros_old[data_num] = thisMicros;
    }
    
    lambda = 2 * PI * fc[data_num] * deltaMicros / 1000000.0;
    x[data_num] = analog_data; // 아날로그값 읽기
    x_f[data_num] = lambda / (1 + lambda) * x[data_num] + 1 / (1 + lambda) * x_fold[data_num]; //필터된 값
    x_fold[data_num] = x_f[data_num]; // 센서 필터 이전값 업데이트
    return x_f[data_num];
}

float average_filter(float analog_data, int data_num) {
    average_sum[data_num] += analog_data;
    average_sum[data_num] -= x_average[data_num][average_loc[data_num]];
    x_average[data_num][average_loc[data_num]] = analog_data;
    
    average_loc[data_num] = (average_loc[data_num] + 1) % AVERAGE_SIZE;
    return average_sum[data_num] / AVERAGE_SIZE;
}

int mod(int a, int b) {
    int c = a;
    while (c < 0) {
        c += b;
    }
    while (c >= b) {
        c -= b;
    }
    return c;
}

void reload(void){
  if(Dir == LOW){
    Dir = HIGH;
    desired_pwm = 30;
    if(analogRead(hollSensor1) > 50 && analogRead(hollSensor2) > 50){
       motor_control();
       delay(100);
    }
   
    Dir = LOW;
    desired_pwm = 0;
    
  }else if(Dir == HIGH){
    Dir = LOW;
    desired_pwm = 30;
    if(analogRead(hollSensor1) > 50 && analogRead(hollSensor2) > 50){
       motor_control();
       delay(100);
    }

    Dir = HIGH;
    desired_pwm = 0;
  }
}
