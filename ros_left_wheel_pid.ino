#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
geometry_msgs::Twist left_wheel;
ros::Publisher pub("left_encoder", &left_wheel);

double left_vel=0.0;
const int dir_pin=8;
const int pwm_pin=10;
const int enc_chA=2;
const int enc_chB=3;
double wheel_pulse = 0;
double wheel_radius = 0.167;
double width_robot = 0.684;
double rps = 0;
double vel_sec = 0;
double sensed_output, control_signal, setpoint;
double Kp=1.0; //proportional gain 1.0 1.0
double Ki=1.0; //integral gain 4.0 0.1
double Kd=0.0; //derivative gain 0.01 1.8
double T=1; //sample time in milliseconds (ms)
double last_time, old_time;
double total_error, last_error;
double max_control=255;
double min_control=-255;

double newpositionL = 0;
double oldpositionL = 0;
double newtime = 0;
double oldtime = 0;
double rpm = 0;
double ticks_sec = 0;
double distance = 0;
double pi=3.14159265358979323846; 

void velocityCb( const geometry_msgs::Twist &cmd_msg){

  left_vel=((cmd_msg.linear.x*2)-(cmd_msg.angular.z*width_robot))/(2);
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel",velocityCb);

void setup(){ 
  Serial.begin(9600);
  pinMode(dir_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
  pinMode(enc_chA,INPUT_PULLUP);
  pinMode(enc_chB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_chA),do_encoder,RISING);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop(){
  PID_Control(); //calls the PID function every T interval and outputs a control signal  
  if(control_signal>=0){
    digitalWrite(dir_pin,HIGH);
    analogWrite(pwm_pin,control_signal);}
  else{
    digitalWrite(dir_pin,LOW);
    analogWrite(pwm_pin,abs(control_signal));
  }
  Serial.print(sensed_output);
  Serial.print("\t");
  Serial.print(rps);
  Serial.print("\t");
  Serial.println(setpoint);
  nh.spinOnce();
}

void PID_Control(){

  unsigned long current_time = micros(); //returns the number of milliseconds passed since the Arduino started running the program
  double delta_time = current_time - last_time; //delta time interval 
  if (delta_time >= T){
    last_time = current_time;
    newpositionL = wheel_pulse;
    ticks_sec = ((newpositionL-oldpositionL)*1000000)/(delta_time);
    oldpositionL = newpositionL;
    left_wheel.linear.x = ticks_sec;   
    pub.publish(&left_wheel);
    rpm = (60*ticks_sec)/6000;
    /*rps = (ticks_sec/6000);
    vel_sec = rps * 2 * pi * wheel_radius;
    distance = (wheel_pulse/6000) * 2 * pi * wheel_radius;*/
    sensed_output = rpm;
    setpoint = (left_vel*60)/(2*pi*wheel_radius); 
    double error = setpoint - sensed_output;
    
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    double delta_error = error - last_error; //difference of error for derivative term

    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;
    
    last_error = error;  
    }  
}

void do_encoder(){
if(digitalRead(enc_chA)==digitalRead(enc_chB)){
  wheel_pulse++;
}
else{
  wheel_pulse--;
} 
}
