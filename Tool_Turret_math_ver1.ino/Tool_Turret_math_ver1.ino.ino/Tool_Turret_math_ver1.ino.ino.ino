#define ENABLEPIN 9 // must be LOW in order for driver to be turned on
#define M0PIN 8
#define M1PIN 7
#define M2PIN 6
#define RESETPIN 5  // must be HIGH in order for driver to be turned on
#define SLEEPPIN 4  // must be HIGH in order for driver to be turned on
#define STEPPIN 3   // moves the motor one microstep per pulse
#define DIRPIN 2    // specifies which direction the motor will turn 

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

int tool_num = 1;
int tool_tot = 6;

bool dir_toggle = 1;    // enter a zero or a one to switch the direction.  Or you can switch two wires around.  Don't let me tell you what to do.  Live your lfe man.

void setup() {

  
  Serial.begin(115200);
  Serial.setTimeout(10);
  Serial.println("Connected");
  
  pinMode(ENABLEPIN, OUTPUT);
  pinMode(M0PIN, OUTPUT);
  pinMode(M1PIN, OUTPUT);
  pinMode(M2PIN, OUTPUT);
  pinMode(RESETPIN, OUTPUT);
  pinMode(SLEEPPIN, OUTPUT);
  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  
  digitalWrite(DIRPIN, HIGH);
  digitalWrite(ENABLEPIN, LOW);
  digitalWrite(RESETPIN, HIGH);
  digitalWrite(SLEEPPIN, HIGH);
  
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

}

void loop() {
  serial_read();

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

void pulse(bool dir){
  digitalWrite(DIRPIN, dir != dir_toggle);
  delayMicroseconds(4);
  digitalWrite(STEPPIN, HIGH);
  delayMicroseconds(4);
  digitalWrite(STEPPIN, LOW);
  delayMicroseconds(4);

  trap_count += 1;
  if (dir == 1){
    pos += 1.8/res;
    pulse_count += 1;
  }
  else{
    pos -= 1.8/res;
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
      if (tool_target <= tool_tot && tool_target >= 1){
        rot = tool_target - tool_num;
        if (rot < 0){
          rot += tool_tot;
        }
      }
   
      tool_num = tool_num + rot;
        if (tool_num > tool_tot){
          tool_num -= tool_tot;        
        }
      Serial.print("Tool number: ");
      Serial.println(tool_num);
      
      pos_target = rot * 60;
      pulse_target += pos_target * res / 1.8;
    }
  }



  

}
