//#define ENABLEPIN 9 // must be LOW in order for driver to be turned on
#define M0PIN 7// determines microstep size
#define M1PIN 6 // determines microstep size
#define M2PIN 5 // determines microstep size
#define RESETPIN 2  // must be HIGH in order for driver to be turned on
#define SLEEPPIN 2 // must be HIGH in order for driver to be turned on

#define DIRPIN1 3    // specifies which direction the first turret will turn 
#define STEPPIN1 4   // moves the motor one microstep per pulse
#define DIRPIN2 8  // specifies which direction the first turret will turn
#define STEPPIN2 9 // moves the motor one microstep per pulse

#define HALLPIN1 A2  // analog input pin for the first turret hall sensor
#define HALLPIN2 A4  // analog input pin for the first turret hall sensor

#define TOOLPIN1 A1 // binary input pins
#define TOOLPIN2 A3
#define TOOLPIN3 A5
#define TOOLPIN4 13

#define SIGNALPIN A7  // signal pin to MASSO

int step_size = 6;  // this is the value corresponding to microstep size, see the table in the README file
int res = 8;  //  microstep resolution

int M0_state;
int M1_state;
int M2_state;


float pos = 0;
float vel = 0;
float pos_target = 0; // keeps track of position
float pulse_timer = 0;  // keeps track of timer during trapezoidal
long pulse_count = 0;   // keeps track of total pulse count for first turret
float pulse_target = 0;   // keeps track of the target pulse count for first turret

long pulse_count_2 = 0;   // keeps track of pulse count for second turret
float pulse_target_2 = 0;  // keeps track of pulse target for second turret

float trap_count = 0;   // used internally during the trapezoidal profile movement

long hall_pos[2] = {0, 0};    // stores the locations of the hall sensors for the first turret
long hall_pos_2[2] = {0, 0};    // stores the locations of the hall sensors for the second turret

// VARIABLES TO MESS AROUND WITH
int tool_num1 = 1;  // initial tool number for turret 1
int tool_tot1 = 6;  // total number of tools in turret 1
int tool_angle1 = 60; // angle between tools in turret 1  

int tool_num2 = 7;  // initial tool number for turret 2 (must be tool_tot1 + 1)
int tool_tot2 = 6;  // total number of tools in turret 2
int tool_angle2 = 60;   // angle between tools in turret 2

int tool_target = 1;  // keeps track of the target tool to be at

bool dir_toggle = 1;    // enter a zero or a one to switch the direction on first turret.  Or you can switch two wires on the stepper  around.  Don't let me tell you what to do.  Live your lfe man.
bool dir_toggle_2 = 0;    // enter a zero or a one to switch the direction on second turret.  Or you can switch two wires on the stepper  around.  Don't let me tell you what to do.  Live your lfe man.

float tool_accel = 1000;   // the constant acceleration during the trapezoidal velocity profile in deg/s^2
float tool_vel_max = 540;    // the max velocity during the trapezoidal velocity profile in deg/s

float backstep = 5.4;    // the amount in degrees to step backwards into the pawl during tool changes.
float backstep_accel = 50;  // the acceleration during the backstep in deg/s^2
float backstep_vel_max = 100; // velocity during the backstep in deg/s

float home_accel = 1000;    // the acceleration during the homing sequence in deg/s^2
float home_vel_max = 1000;   // note that the max velocity will be limited due to the analog read function taking 100 microseconds

// after doing the homing 360 degrees, the turret will go to the hall sensor location plus this home_offset value, than backdrive by hall_angle value
// takes some tuning to get the stepper to backdrive into the pawl the proper amount
float home_offset_1 = 35;    
float home_offset_2 = 0;   
float hall_angle_2 = 10.7;
float hall_angle_1 = 12.5;

int turret_mode = 1;    // the turret mode.  Set to 1 for reading the binary output from the MASSO.  Set to 2 for cycling the binary output to the MASSO and waiting for a signal to stop

void setup() {

  // set up serial communication
  Serial.begin(115200);   
  Serial.setTimeout(10);
  Serial.println("Connected");
  
//  pinMode(ENABLEPIN, OUTPUT);

  // set up pins for stepper driver
  pinMode(M0PIN, OUTPUT);
  pinMode(M1PIN, OUTPUT);
  pinMode(M2PIN, OUTPUT);
  pinMode(RESETPIN, OUTPUT);
  pinMode(SLEEPPIN, OUTPUT);
  pinMode(STEPPIN1, OUTPUT);
  pinMode(STEPPIN2, OUTPUT);
  pinMode(DIRPIN1, OUTPUT);
  pinMode(DIRPIN2, OUTPUT);
  
  // set binary tool pins to input for mode 1
  if (turret_mode == 1){
    pinMode(TOOLPIN1, INPUT_PULLUP);
    pinMode(TOOLPIN2, INPUT_PULLUP);
    pinMode(TOOLPIN3, INPUT_PULLUP);
    pinMode(TOOLPIN4, INPUT_PULLUP);
    pinMode(SIGNALPIN, OUTPUT);
  }

  // set binary tool pins to output for mode 2
  if (turret_mode == 2){
    pinMode(TOOLPIN1, OUTPUT);
    pinMode(TOOLPIN2, OUTPUT);
    pinMode(TOOLPIN3, OUTPUT);
    pinMode(TOOLPIN4, OUTPUT);
    pinMode(SIGNALPIN, INPUT);
  }

  // set up pins for stepper driver
  
  digitalWrite(DIRPIN1, HIGH);
  digitalWrite(DIRPIN2, HIGH);
//  digitalWrite(ENABLEPIN, LOW);
  digitalWrite(RESETPIN, HIGH);
  digitalWrite(SLEEPPIN, HIGH);
  digitalWrite(SIGNALPIN, LOW);

  // determine the binary orientation for the desired microstep size
  M0_state = bitRead(step_size, 2);
  M1_state = bitRead(step_size, 1);
  M2_state = bitRead(step_size, 0);

  // set the microstep setting pins to the desired microstep position
  digitalWrite(M0PIN, M0_state);
  digitalWrite(M1PIN, M1_state);
  digitalWrite(M2PIN, M2_state);

  // change all values to microsteps instead of degrees for the trapezoid function
  tool_accel *= res/1.8;
  tool_vel_max *= res/1.8;
  backstep_accel *= res/1.8;
  backstep_vel_max *= res/1.8;

  // home both turrets
  home_turret(1);
  home_turret(2);

}

void loop() {
  
  serial_read();  // read serial input

  if (turret_mode == 1){
    tool_target = tool_read();  // read desired tool from binary pins
  }
  if (turret_mode == 2) {
    tool_target = turret_cycle(100);  // cycle desired tool from binary pins
  }

    if (tool_target != tool_num1 && tool_target != tool_num2){  // if the tool target has changed, move to the new tool location

      if (turret_mode == 1){
      digitalWrite(SIGNALPIN, LOW);   // write all clear pin low while moving tools for mode 1
      }
      int rot = 0;
      
      if (tool_target <= tool_tot1 && tool_target >= 1){    // if tool is meant for first turret
        rot = tool_target - tool_num1;    // calculate number of tool rotations to do
        if (rot < 0){
          rot += tool_tot1;  
        }
      
      pulse_target += rot * tool_angle1 * res / 1.8;    // calculate new target position
      
      pulse_target += backstep*res/1.8;     // add on the overshoot amount
      trapezoid(abs((long)pulse_target-pulse_count), 1, tool_accel, tool_vel_max, 1);    // move to the target position + overshoot amount
      pulse_target -= backstep*res/1.8;   // subtract the overshoot amount
      trapezoid(abs((long)pulse_target-pulse_count), 0, backstep_accel, backstep_vel_max, 1);    // move back the overshoot amount  
  
      tool_num1 += rot;   // update the tool position of the turret
      if (tool_num1 > tool_tot1){
        tool_num1 -= tool_tot1;        
      }
      
      Serial.print("Tool turret 1: ");
      Serial.println(tool_num1);  
      }
  
      if (tool_target <= tool_tot2+tool_tot1 && tool_target > tool_tot1){    // if tool is meant for second turret
        rot = tool_target - tool_num2;    // calculate number of tool rotations to do
        if (rot < 0){
          rot += tool_tot2;
        } 
      pulse_target_2 += rot * tool_angle2 * res / 1.8;    // calculate new target position
      
      pulse_target_2 += backstep*res/1.8;     // add on the overshoot amount
      trapezoid(abs((long)pulse_target_2-pulse_count_2), 1, tool_accel, tool_vel_max, 2);    // move to the target position + overshoot amount
      pulse_target_2 -= backstep*res/1.8;   // subtract the overshoot amount
      trapezoid(abs((long)pulse_target_2-pulse_count_2), 0, backstep_accel, backstep_vel_max, 2);    // move back the overshoot amount  
  
      tool_num2 += rot;   // update the tool position of the turret
      if (tool_num2 > tool_tot2 + tool_tot1){
        tool_num2 -= tool_tot2;        
      }
      
      Serial.print("Tool turret 2: ");
      Serial.println(tool_num2 - tool_tot1);  
      }
      
    if (turret_mode == 1){
      digitalWrite(SIGNALPIN, HIGH);   // write all clear pin low while moving tools
    }
    }

  
}

int turret_cycle(int delay_time){
  // cycle the desired tool number via four pins acting as a single binary number (High = 1, Low = 0) for the MASSO controller to read.  When the
  // MASSO controller turns the SIGNALPIN to High, the arduino stops cycling and moves the turret to the desired location
  
  static int t=0;
  while(digitalRead(SIGNALPIN) == LOW){
    t = (t+1)%(tool_tot1 + tool_tot2);   // cycle tool number 
    bool t1 = bitRead(t, 0);    // convert tool number to binary
    bool t2 = bitRead(t, 1);
    bool t3 = bitRead(t, 2);
    bool t4 = bitRead(t, 3);
    digitalWrite(TOOLPIN1, t1);   // write binary to tool pins
    digitalWrite(TOOLPIN2, t2);
    digitalWrite(TOOLPIN3, t3);
    digitalWrite(TOOLPIN4, t4);
    
    delay(delay_time);  // add delay
  }
  int tool_num = t + 1;
  return tool_num;
}

void trapezoid(long p3, bool dir, float accel, float vel_max, int turret_no){
  // move turret in a trapezoidal velocity profile
  
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

  pulse(dir, turret_no);
  
  while(trap_count < p3){

    if (trap_count < p1){
      vel = accel  * sqrt(2.00*trap_count/accel);
    }
    if (trap_count > p2){
      vel = accel * sqrt(2.00*(p3-trap_count)/accel);
    }

    if (micros() - pulse_timer > 1000000/vel){
      pulse(dir, turret_no);   
      pulse_timer = micros();
    }
  }
  
// Serial.print(pulse_count);  Serial.print(' '); Serial.println(pulse_target); 
  
}


void pulse(bool dir, int turret_no){
  // pulse the desired turret in the desired direction

  if (turret_no == 1){
    digitalWrite(DIRPIN1, dir != dir_toggle);
    delayMicroseconds(4);
    digitalWrite(STEPPIN1, HIGH);
    delayMicroseconds(4);
    digitalWrite(STEPPIN1, LOW);
    delayMicroseconds(4);
    
    trap_count += 1;
    if (dir == 1){
      pulse_count += 1;
    }
    else{
      pulse_count -= 1;
    }
  }
  if (turret_no == 2){
    digitalWrite(DIRPIN2, dir != dir_toggle_2);
    delayMicroseconds(4);
    digitalWrite(STEPPIN2, HIGH);
    delayMicroseconds(4);
    digitalWrite(STEPPIN2, LOW);
    delayMicroseconds(4);
    trap_count += 1;
    if (dir == 1){
      pulse_count_2 += 1;
    }
    else{
      pulse_count_2 -= 1;
    }
  }
}


int tool_read(){
  // this is for mode 1 where the arduino will read the tool number from the MASSO controller by converting 4 digital pins into a binary tool number value
  
  int t1 = digitalRead(TOOLPIN1)==0;
  int t2 = digitalRead(TOOLPIN2)==0;
  int t3 = digitalRead(TOOLPIN3)==0;
  int t4 = digitalRead(TOOLPIN4)==0;
  int tool_num = t1 + (t2<<1) + (t3<<2) + (t4<<3) + 1;
  Serial.print(t1);
  Serial.print(t2);
  Serial.print(t3);
  Serial.println(t4);

  if (tool_num != tool_num1 && tool_num != tool_num2){
    delay(5);
    t1 = digitalRead(TOOLPIN1)==0;
    t2 = digitalRead(TOOLPIN2)==0;
    t3 = digitalRead(TOOLPIN3)==0;
    t4 = digitalRead(TOOLPIN4)==0;
    tool_num = t1 + (t2<<1) + (t3<<2) + (t4<<3) + 1;
  }

  return tool_num;
}


void serial_read(){
  // read the serial input for testing purposes
  
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
      tool_target = value.toInt();
    }

    if (command == "home"){
      
      home_turret(constrain(value.toInt(), 1, 2));
    }
  }
    

}


void home_turret(int turret_no){
  // home the desired turret using two hall sensors
  
  pulse_count = 0;    // reset pulse count
  pulse_count_2 = 0;
  pulse_target = 0;
  pulse_target_2 = 0;
  
  home_trapezoid((long)360*res/1.8, 1, home_accel, home_vel_max, turret_no);    // do a full circle and find the position that has the max hall sensor reading

  // switch between the two offsets depending on turret
  int offset = 0;
  if (turret_no ==1){
    offset = home_offset_1;    
  }
  if (turret_no == 2){
    offset = home_offset_2;
  }
  if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 185){
    trapezoid((hall_pos[1] + offset*res/1.8), 1, home_accel, home_vel_max, turret_no);   // move to the hall sensor position + the home offset
//    trapezoid((hall_pos[1] + home_offset_1*res/1.8), 1, home_accel, home_vel_max, turret_no);   // move to the hall sensor position + the home offset
    Serial.println("option 1");
    Serial.println(hall_pos[1] + home_offset_1*res/1.8);
  }
  else{
    trapezoid((hall_pos[0] + offset*res/1.8), 1, home_accel, home_vel_max, turret_no);   // move to the hall sensor position + the home offset
//  trapezoid(hall_pos[0], 1, home_accel, home_vel_max, turret_no);   // move to the hall sensor position + the home offset
    Serial.println("option 2");
  }

  if (turret_no == 1){
  trapezoid((hall_angle_1)*res/1.8, 0, home_accel, home_vel_max, turret_no);    // move backwards the hall sensor offset
  }
  if (turret_no == 2){
  trapezoid((hall_angle_2)*res/1.8, 0, home_accel, home_vel_max, turret_no);    // move backwards the hall sensor offset
  }

  // reset variables
  pulse_count = 0;
  pulse_target = 0;
  pulse_count_2 = 0;
  pulse_target_2 = 0;
  
  if (turret_no == 1){
    tool_num1 = 1;
  }
  if (turret_no == 2){
    tool_num2 = tool_tot1 + 1;
  }

  // detect number of tools in turret, currently supports 4, 6, and 8 (this is untested)
  if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 55 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 65){
    if (turret_no == 1){
      tool_angle1 = 60;
      tool_tot1 = 6;
    }
    if (turret_no == 2){
      tool_angle2 = 60;
      tool_tot2 = 6;
    }
  }
  else if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 40 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 50){
    if (turret_no == 1){
      tool_angle1 = 45;
      tool_tot1 = 8;
    }
    if (turret_no == 2){
      tool_angle2 = 45;
      tool_tot2 = 8;
    }
  }
  else if (abs(hall_pos[0] - hall_pos[1])*1.8/res > 85 && abs(hall_pos[0] - hall_pos[1])*1.8/res < 95){
    if (turret_no == 1){
      tool_angle1 = 90;
      tool_tot1 = 4;
    }
    if (turret_no == 2){
      tool_angle2 = 90;
      tool_tot2 = 4;
    }
  }

  delay(1000);
}


void home_trapezoid(long p3, bool dir, float accel, float vel_max, int turret_no){
  // trapezoidal motion profile adapted for reading analog input in order to detect hall sensors

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

  pulse(dir, turret_no);
  
  while(trap_count < p3){

    if (trap_count < p1){
      vel = accel  * sqrt(2.00*trap_count/accel);
    }
    if (trap_count > p2){
      vel = accel * sqrt(2.00*(p3-trap_count)/accel);
    }

    if (micros() - pulse_timer > 1000000/vel){
      pulse(dir, turret_no);   
      pulse_timer = micros();  
    }
    
      int hall_1 = 0;
      
      if (turret_no == 1){
        hall_1 = analogRead(HALLPIN1);
      }    
      else if (turret_no == 2){
        hall_1 = analogRead(HALLPIN2);
      }
      
      if (hall_1 < 30 && hall_flag == 0){
        if (turret_no == 1){
        hall_pos[0] = pulse_count;
        }
        else if (turret_no == 2){
        hall_pos[0] = pulse_count_2;
        }
        hall_flag = 1;
        
        Serial.println(hall_pos[0]);
      }
      else if (hall_flag == 1 && hall_1 > 30){
        hall_flag = 2;
      }
      else if (hall_1 < 30 && hall_flag == 2){
        if (turret_no == 1){
        hall_pos[1] = pulse_count;
        }
        else if (turret_no == 2){
        hall_pos[1] = pulse_count_2;
        }
        hall_flag = 3;
        Serial.println(hall_pos[1]);
      }
      else if (hall_flag == 3 && hall_1 > 10){
        hall_flag = 4;
      }
//      Serial.print(pulse_count); Serial.print(' '); Serial.println(hall_1);
  }
  Serial.print("hall sensor locations for turret no "); Serial.print(turret_no); Serial.print(": ");
  Serial.print(hall_pos[0]*1.8/res);Serial.print(' ');
  Serial.println(hall_pos[1]*1.8/res);
}

