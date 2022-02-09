#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/camera.hpp>
#include <webots/Display.hpp>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define TIME_STEP 32
#define MAX_SPEED 10
#define LINE_TROSHOLD 446
#define WALL_TRESHOLD 690
#define DELTA 0.1
#define SPEED 7
#define TURN90 5.45
#define TURN_SPEED 2
#define FRONT 0
#define RIGHT 1
#define LEFT 2
#define CELL_WIDTH 10

using namespace webots;
using namespace std;

////////////////////////////////////STAGES////////////////////////////////////////////////////

enum Stages
    {
        LINE_FOLLOW,
        WALL_FOLLOW
    };
Stages currentStage = LINE_FOLLOW;


////////////////////////////////////DEFINE OBJECTS////////////////////////////////////////////////////
Robot *robot;
Motor *right_motor;
Motor *left_motor;
Motor *servo;
DistanceSensor *ds[10];
PositionSensor *ps[3];
PositionSensor *servoEncoder;
Camera *camera;
Display *display;


///////////////////////////////////DEFINE VARIABLES////////////////////////////////////////////////////
double psValue[2];
char dsName[10][5] =  {"ir_1", "ir_2", "ir_3", "ir_4", "ir_5", "ir_6", "ir_7", "sn_f", "sn_r", "sn_l"};
bool dsDigitalValue[7];
double dsValue[7];
double snValue[3];
bool snDigitalValue[3];
double kp; double ki; double kd; double error; double previousError; double accumulateError;
double PValue; double IValue; double DValue; double correction;
double rightVelocity; double leftVelocity;
int PIDcoefficient[7] = {-3, -2, -1, 0, 1, 2, 3};
double servoValue;
int width;
int height;
const unsigned char *image;


///////////////////////////////////WHEEL MOTORS/////////////////////////////////////////////////////////

void initialize_wheel_motors(){
    robot = new Robot();
    right_motor = robot->getMotor("right_motor");
    left_motor = robot->getMotor("left_motor");
}

void enable_velocity_control(){
  right_motor->setPosition(INFINITY);
  left_motor->setPosition(INFINITY);
  //right_motor->setVelocity(0);
  //left_motor->setVelocity(0);
}

void set_velocity(double right, double left){
    enable_velocity_control();
    right_motor->setVelocity(right);
    left_motor->setVelocity(left);
}

///////////////////////////////////ENCODERS/////////////////////////////////////////////////////////

void initialize_encoders(){
  ps[0] = robot->getPositionSensor("ps_r");ps[0]->enable(TIME_STEP);
  ps[1] = robot->getPositionSensor("ps_l");ps[1]->enable(TIME_STEP);
}

void get_encoder_value(){
  psValue[0] = ps[0]->getValue();
  psValue[1] = ps[1]->getValue();
}



///////////////////////////////////DISTANCE SENSOR/////////////////////////////////////////////////////////

void initialize_ds(){
  for (int i = 0; i < 10; i++){
    ds[i] = robot->getDistanceSensor(dsName[i]);
    ds[i]->enable(TIME_STEP);
  }
}

void ds_digital_value(){
  for (int i = 0; i < 7; i++){
    if (dsValue[i] < LINE_TROSHOLD){
      dsDigitalValue[i] = true;
    }
    else {
      dsDigitalValue[i] = false;
    }
  }
}

void read_ds(){
  for (int i = 0; i < 7; i++){
    dsValue[i] = ds[i]->getValue();
  }
  ds_digital_value();
}

void read_sn(){
  for (int i = 7; i < 10; i++){
    snValue[i - 7] = ds[i]->getValue();
  }
}

void initialize_PID(){
  kp = 2;            //2
  ki = 0;
  kd = 1;            //1
  previousError = 0.0;
  accumulateError = 0.0;
}

void follow_line(){
  read_ds();
  error = 0;
  for (int i = 0; i < 7; i++){
    error += dsDigitalValue[i] * PIDcoefficient[i];
  }
  //cout << error << endl;
  
  PValue = kp * error;
  IValue = ki * (accumulateError + error);
  DValue = kd * (error - previousError);

  previousError = error;
  accumulateError += error;

  correction = (PValue + IValue + DValue);
  rightVelocity = SPEED - correction; leftVelocity = SPEED + correction;
  //cout << correction << endl;

  if (rightVelocity < 0){rightVelocity = 0;}
  if (leftVelocity < 0){leftVelocity = 0;}
  if (rightVelocity > MAX_SPEED){rightVelocity = MAX_SPEED;}
  if (leftVelocity > MAX_SPEED){leftVelocity = MAX_SPEED;}

  set_velocity(rightVelocity, leftVelocity);
}

///////////////////////////////////WALL FOLLOW/////////////////////////////////////////////////////////
void move_specific_distance(double right, double left){
  get_encoder_value();
  double rightTarget = psValue[0] + right;
  double leftTarget = psValue[1] + left;
  //cout << psValue[0] << endl;
  double rightSpeed = SPEED; double leftSpeed = SPEED;
  if (right < 0){
    rightSpeed = -TURN_SPEED;
    leftSpeed = TURN_SPEED;
    }
  if (left < 0){
    rightSpeed = TURN_SPEED;
    leftSpeed = -TURN_SPEED;
    }

  set_velocity(rightSpeed, leftSpeed);
  
  while ((fabs(rightTarget - psValue[0])> DELTA) || (fabs(leftTarget - psValue[1])> DELTA)){
    robot->step(TIME_STEP);
    get_encoder_value();
    //cout << psValue[0] << endl;
  }
  set_velocity(0, 0);
}

void turn(int dir){
  if (dir == RIGHT){
    move_specific_distance(-TURN90, TURN90);
  }
  if (dir == LEFT){
    move_specific_distance(TURN90, -TURN90);
  }
}

void sn_digital_value(){
  read_sn();
  for (int i = 0; i<3; i++){
    if (snValue[i] > 350){
      snDigitalValue[i] = true;
    }
    else{
      snDigitalValue[i] = false;
    }
  }
}

void initialize_wall_PID(){
  kp = 6;            //2
  ki = 0;
  kd = 2;            //1
  previousError = 0.0;
  accumulateError = 0.0;
}

void follow_wall(){
  sn_digital_value();
  error = 0;
  error = (WALL_TRESHOLD - snValue[LEFT]);

  // if (snDigitalValue[RIGHT]){
  //   error += (snValue[LEFT] - WALL_TRESHOLD);
  // }
  // error /=2;

  PValue = kp * error;
  IValue = ki * (accumulateError + error);
  DValue = kd * (error - previousError);

  previousError = error;
  accumulateError += error;
  //cout << snValue[LEFT] << endl;
  correction = (PValue + IValue + DValue)/1000;
  rightVelocity = SPEED + correction; leftVelocity = SPEED - correction;
  //cout << correction << endl;

  if (rightVelocity < 0){rightVelocity = 0;}
  if (leftVelocity < 0){leftVelocity = 0;}
  if (rightVelocity > MAX_SPEED){rightVelocity = MAX_SPEED;}
  if (leftVelocity > MAX_SPEED){leftVelocity = MAX_SPEED;}
  //cout <<snValue[1]<<"   "<<snValue[2]<<endl;


  set_velocity(rightVelocity, leftVelocity);
}

void follow_maze(){
  sn_digital_value();
  //cout << snDigitalValue[0]<<snDigitalValue[1]<<snDigitalValue[2]<<endl;

  if (snDigitalValue[LEFT]){
    if (!(snDigitalValue[FRONT])){
      follow_wall();
      //set_velocity(SPEED, SPEED);
    }
    else{
      move_specific_distance(4, 4);
      if (snDigitalValue[RIGHT]){
        turn(RIGHT);
      }
      turn(RIGHT);
    }
  }
  else{
    if (snDigitalValue[FRONT]){
      turn(LEFT);
      move_specific_distance(3, 3);
    }
    else{
      move_specific_distance(10, 10);
      turn(LEFT);
      move_specific_distance(4, 4);
    }

    }
  }


///////////////////////////////////ARM/////////////////////////////////////////////////////////

void initialize_servo(){
    servo = robot->getMotor("servo");
    servoEncoder = robot->getPositionSensor("servo_en");
    servoEncoder->enable(TIME_STEP);
}

void pull_down(){
  servo->setControlPID(2, 0, 0);
  servo->setVelocity(1);
  servo->setPosition(-0.57);
  double currentPosition;
  do
  { 
    currentPosition = servoEncoder->getValue();
    robot->step(TIME_STEP);
  } while (fabs(currentPosition + 0.57 ) > DELTA);
  
}

void pull_up(){
  servo->setControlPID(2, 0, 0);
  servo->setVelocity(1);
  servo->setPosition(1);
  double currentPosition;
  do
  { 
    currentPosition = servoEncoder->getValue();
    robot->step(TIME_STEP);
  } while (fabs(currentPosition - 1 ) > DELTA);
}

///////////////////////////////////CAMERA/////////////////////////////////////////////////////////
void initialize_camera(){
  camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  display = robot->getDisplay("display");
  width = camera->getWidth();
  height = camera->getHeight();
}

void display_image(){
      //ImageRef *ir = display->imageNew(width, height, data, Display::ARGB);
      // display->imagePaste(ir, 0, 0, false);
      // display->imageDelete(ir);
}
///////////////////////////////////MAIN/////////////////////////////////////////////////////////

int main(int argc, char **argv) {
      initialize_wheel_motors();
      initialize_ds();
      initialize_PID();
      initialize_encoders();
      initialize_servo();
      initialize_camera();
      

  while (robot->step(TIME_STEP) != -1) {
    
    switch (currentStage)
    {
    case LINE_FOLLOW:
      follow_line();
      sn_digital_value();
      if (snDigitalValue[LEFT]){
        currentStage = WALL_FOLLOW;
        initialize_wall_PID();
      }
      break;
    
    case WALL_FOLLOW:
      
      follow_maze();
      break;
    
    }
    //image = camera->getImage();
  
  

  };
  
  delete robot;
  return 0;
}



