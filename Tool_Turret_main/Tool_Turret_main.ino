//#define ENABLEPIN 9 // must be LOW in order for driver to be turned on
#define M0PIN 8
#define M1PIN 7
#define M2PIN 6
#define RESETPIN 5  // must be HIGH in order for driver to be turned on
#define SLEEPPIN 4  // must be HIGH in order for driver to be turned on
#define STEPPIN 3   // moves the motor one microstep per pulse
#define DIRPIN 2    // specifies which direction the motor will turn 

#define HALLPIN1 0
#define HALLPIN2 1

#define TOOLPIN1 9
#define TOOLPIN2 10
#define TOOLPIN3 11
#define TOOLPIN4 12

#define ALLCLEARPIN A2

int step_size = 6;
int res = 8;

int M0_state;
int M1_state;
int M2_state;

float pos = 0;
float vel = 0;

float tool_accel = 1000;   // the constant acceleration during the trapezoidal velocity profile in deg/s^2
float tool_vel_max = 270;    // the max velocity during the trapezoidal velocity profile in deg/s^2

float backstep = 5.4;    // the amount in degrees to step backwards into the pawl.  Make sure it lies on an exact multiple of the step size. e.g. if resolution is 2 make sure is a multiple of 0.9
float backstep_accel = 50;
float backstep_vel_max = 100;

float pos_target = 0;
float pulse_timer = 0;
long pulse_count = 0;
float pulse_target = 0;

float trap_count = 0;

int tool_num1 = 1;
int tool_tot1 = 6;
int tool_angle1 = 60;

int tool_num2 = 9;
int tool_tot2 = 6;
int tool_angle2 = 60;

bool dir_toggle = 1;    // enter a zero or a one to switch the direction.  Or you can switch two wires on the stepper  around.  Don't let me tell you what to do.  Live your lfe man.


float home_accel = 1000;    
float home_vel_max = 540;   // note that the max velocity will be limited due to the analog read function taking 100 microseconds
float home_backdrive = 0.45;

float hall_angle = 8.1;
long hall_pos[2] = {0, 0};
int hall_max_1 = 0;


void setup() {

  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println("Connected");
  
//  pinMode(ENABLEPIN, OUTPUT);
  pinMode(M0PIN, OUTPUT);
  pinMode(M1PIN, OUTPUT);
  pinMode(M2PIN, OUTPUT);
  pinMode(RESETPIN, OUTPUT);
  pinMode(SLEEPPIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(ALLCLEARPIN, OUTPUT);
  pinMode(TOOLPIN1, INPUT_PULLUP);
  pinMode(TOOLPIN2, INPUT_PULLUP);
  pinMode(TOOLPIN3, INPUT_PULLUP);
  pinMode(TOOLPIN4, INPUT_PULLUP);
  
  digitalWrite(DIRPIN, HIGH);
//  digitalWrite(ENABLEPIN, LOW);
  digitalWrite(RESETPIN, HIGH);
  digitalWrite(SLEEPPIN, HIGH);
  digitalWrite(ALLCLEARPIN, LOW);
  
  M0_state = bitRead(step_size, 2);
  M1_state = bitRead(step_size, 1);
  M2_state = bitRead(step_size, 0);
  
  digitalWrite(M0PIN, M0_state);
  digitalWrite(M1PIN, M1_state);
  digitalWrite(M2PIN, M2_state);

  tool_accel *= res;
  tool_vel_max *= res;
  backstep_accel *= res;
  backstep_vel_max *= res;

  home_turret();
}

void loop() {
//  serial_read();
  int tool_target = tool_read();

  if (tool_target != tool_num1 && tool_target != tool_num2){  
    int rot = 0;
    digitalWrite(ALLCLEARPIN, LOW);   // write all clear pin low while moving tools
   
    if (tool_target <= tool_tot1 && tool_target >= 1){    // if tool is meant for first turret
      rot = tool_target - tool_num1;    // calculate number of tool rotations to do
      if (rot < 0){
        rot += tool_tot1;
      }
    
 
    pulse_target += rot * tool_angle1 * res / 1.8;    // calculate new target position
    
    pulse_target += backstep*res/1.8;     // move to new position
    trapezoid(abs((long)pulse_target-pulse_count), 1, tool_accel, tool_vel_max);
    pulse_target -= backstep*res/1.8;
    trapezoid(abs((long)pulse_target-pulse_count), 0, backstep_accel, backstep_vel_max);   

    tool_num1 += rot;   // update the tool position of the turret
    if (tool_num1 > tool_tot1){
      tool_num1 -= tool_tot1;        
    }
    
    Serial.print("Tool number: ");
    Serial.println(tool_num1);
    digitalWrite(ALLCLEARPIN, HIGH);    // write all clear pin high when move is completed
    }
    
  }
  if ((long)pulse_target != pulse_count){

    pulse_target += backstep*res/1.8;
    trapezoid(abs((long)pulse_target-pulse_count), 1, tool_accel, tool_vel_max);
    pulse_target -= backstep*res/1.8;
    trapezoid(abs((long)pulse_target-pulse_count), 0, backstep_accel, backstep_vel_max);   
  }


  
  if (pulse_count >= 2000000000){
    pulse_count -= 2000000000;
    pulse_target -= 2000000000;
  }
  
//  delayMicroseconds(2);
//  digitalWrite(STEPPIN, LOW);
//  Serial.println("step");
//  delayMicroseconds(2);
//  digitalWrite(STEPPIN, HIGH);
//  Serial.println("step");

//  int a.println(a);
//  delay(5 = analogRead(0);
//  Serial0);
}

void trapezoid(long p3, bool dir, float accel, float vel_max){

  trap_count = 0;
  
  float t1 = vel_max/accel;
  float p1 = accel*pow(t1,2)/2;
  float p2 = p3 - p1;

  if (p1 > p3/2){
    p1 = p3/2;
    p2 = p3/2;
  }
  
  long timer = micros();
  long pulse_timer = micros();

  pulse(dir);
  
  while(trap_count < p3){

    if (trap_count < p1){
      vel = accel  * sqrt(2.00*trap_count/accel);
    }
    if (trap_count > p2){
      vel = accel * sqrt(2.00*(p3-trap_count)/accel);
    }

    if (micros() - pulse_timer > 1000000/vel){
      pulse(dir);   
      pulse_timer = micros();
  
//        Serial.print("position: ");
//        Serial.print(pulse_count);
//        Serial.print("  velocity: ");
//        Serial.print(vel);
//        Serial.print('\n');
    }
  }
  
 Serial.print(pulse_count);  Serial.print(' '); Serial.println(pulse_target); 
  
}

int tool_read(){
  int t1 = digitalRead(TOOLPIN1)==0;
  int t2 = digitalRead(TOOLPIN2)==0;
  int t3 = digitalRead(TOOLPIN3)==0;
  int t4 = digitalRead(TOOLPIN4)==0;
  int tool_num = t1 + (t2<<1) + (t3<<2) + (t4<<3) + 1;
  
//  Serial.print(t1);
//  Serial.print(t2);
//  Serial.print(t3);
//  Serial.print(t4);Serial.print(' ');
//  Serial.println(tool_num);

  return tool_num;
}

void pulse(bool dir){
  digitalWrite(DIRPIN, dir != dir_toggle);
  delayMicroseconds(4);
  digitalWrite(STEPPIN, HIGH);
  delayMicroseconds(4);
  digitalWrite(STEPPIN, LOW);
  delayMicroseconds(4);

  trap_count += 1;
  if (dir == 1){
//    pos += 1.8/res;
    pulse_count += 1;
  }
  else{
//    pos -= 1.8/res;
    pulse_count -= 1;
  }
}

void serial_read(){
  
  
  while (Serial.available() > 0) {      // if there are bytes available in the serial port
    String incomingByte = Serial.readStringUntil('\n');     // read the line in the serial port as a string until '\n'       
    int spacelocation = incomingByte.indexOf(' ');      // find the location of the comma
    String command = incomingByte.substring(0,spacelocation);      // assign the command to a string
    String value = incomingByte.substring(spacelocation+1);      // assign the value to a float
    Serial.print(command);
    Serial.print(" ");
    Serial.print(value);
    Serial.print('\n');

    if (command == "step"){
      step_size = value.toFloat();
      M0_state = bitRead(step_size, 2);
      M1_state = bitRead(step_size, 1);
      M2_state = bitRead(step_size, 0);
  
      digitalWrite(M0PIN, M0_state);
      digitalWrite(M1PIN, M1_state);
      digitalWrite(M2PIN, M2_state);
    }

    if (command == "pos"){
      pos_target = value.toFloat();
      pulse_target += pos_target * res / 1.8;
    }

    if (command == "t"){

      int rot = 0;
      int tool_target = value.toInt();
      if (tool_target <= tool_tot1 && tool_target >= 1){
        rot = tool_target - tool_num1;
        if (rot < 0){
          rot += tool_tot1;
        }
      }
   
      tool_num1 = tool_num1 + rot;
        if (tool_num1 > tool_tot1){
          tool_num1 -= tool_tot1;        
        }
      Serial.print("Tool number: ");
      Serial.println(tool_num1);
      
      pos_target = rot * 60;
      pulse_target += pos_target * res / 1.8;
    }

    if (command == "home"){
      home_turret();
    }
  }
    

}

void home_turret(){
  
  pulse_count = 0;    // reset pulse count
  home_trapezoid((long)360*res/1.8, 1, home_accel, home_vel_max);    // do a full circle and find the position that has the max hall sensor reading

  if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 185){
    trapezoid(hall_pos[1], 1, home_accel, home_vel_max);   // move to the hall sensor position
    Serial.println("option 1");
  }
  else{
    trapezoid(hall_pos[0], 1, home_accel, home_vel_max);   // move to the hall sensor position
    Serial.println("option 2");
  }
  
  trapezoid((hall_angle+home_backdrive)*res/1.8, 0, home_accel, home_vel_max);    // move backwards the hall sensor offset plus an additional backdrive parameter
  pulse_count = 0;
  pulse_target = 0;

  if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 55 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 65){
    tool_angle1 = 60;
  }
  else if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 40 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 50){
    tool_angle1 = 45;
  }
  else if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 85 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 95){
    tool_angle1 = 90;
  }
  
}

void home_trapezoid(long p3, bool dir, float accel, float vel_max){

  int hall_flag = 0;
  trap_count = 0;
  
  float t1 = vel_max/accel;
  float p1 = accel*pow(t1,2)/2;
  float p2 = p3 - p1;

  if (p1 > p3/2){
    p1 = p3/2;
    p2 = p3/2;
  }
  
  long timer = micros();
  long pulse_timer = micros();

  pulse(dir);
  
  while(trap_count < p3){

    if (trap_count < p1){
      vel = accel  * sqrt(2.00*trap_count/accel);
    }
    if (trap_count > p2){
      vel = accel * sqrt(2.00*(p3-trap_count)/accel);
    }

    if (micros() - pulse_timer > 1000000/vel){
      pulse(dir);   
      pulse_timer = micros();  
    }
    
      int hall_1 = analogRead(HALLPIN1);
      if (hall_1 < 10 && hall_flag == 0){
        hall_pos[0] = pulse_count;
        hall_flag = 1;
        Serial.println(hall_pos[0]);
      }
      else if (hall_flag == 1 && hall_1 > 10){
        hall_flag = 2;
      }
      else if (hall_1 < 10 && hall_flag == 2){
        hall_pos[1] = pulse_count;
        hall_flag = 3;
        Serial.println(hall_pos[1]);
      }
      else if (hall_flag == 3 && hall_1 > 10){
        hall_flag = 4;
      }
//      Serial.print(pulse_count); Serial.print(' '); Serial.println(hall_1);
  }
  Serial.print("hall sensor locations: ");
  Serial.print(hall_pos[0]*1.8/res);Serial.print(' ');
  Serial.println(hall_pos[1]*1.8/res);  
}

