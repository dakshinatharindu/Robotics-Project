#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/camera.hpp>
#include <webots/Display.hpp>
#include <math.h>
#include <stdio.h>
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
#define WIDTH 640
#define HEIGHT 640
#define CYAN 0
#define OBJ 1
#define YELLOW 2

using namespace webots;
using namespace std;
using namespace cv;

////////////////////////////////////STAGES////////////////////////////////////////////////////

// enum Stages
//     {
//         LINE_FOLLOW,
//         WALL_FOLLOW
//     };
// Stages currentStage = LINE_FOLLOW;

////////////////////////////////////MOSAIC SUB-STAGES////////////////////////////////////////////////////

enum Mosaic
    {
        FIND_CYAN,
        CUBE
    };
Mosaic currentMosaic = FIND_CYAN;


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
double kp; double ki; double kd; double errorPID; double previousError; double accumulateError;
double PValue; double IValue; double DValue; double correction;
double rightVelocity; double leftVelocity;
int PIDcoefficient[7] = {-3, -2, -1, 0, 1, 2, 3};
double servoValue;
Mat frame, frame_HSV, frame_threshold, frameBGR;




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
  errorPID = 0;
  for (int i = 0; i < 7; i++){
    errorPID += dsDigitalValue[i] * PIDcoefficient[i];
  }
  //cout << errorPID << endl;
  
  PValue = kp * errorPID;
  IValue = ki * (accumulateError + errorPID);
  DValue = kd * (errorPID - previousError);

  previousError = errorPID;
  accumulateError += errorPID;

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
  errorPID = 0;
  errorPID = (WALL_TRESHOLD - snValue[LEFT]);

  // if (snDigitalValue[RIGHT]){
  //   errorPID += (snValue[LEFT] - WALL_TRESHOLD);
  // }
  // errorPID /=2;

  PValue = kp * errorPID;
  IValue = ki * (accumulateError + errorPID);
  DValue = kd * (errorPID - previousError);

  previousError = errorPID;
  accumulateError += errorPID;
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
}

void display_image(Mat img){
      Mat im;
      cvtColor(img, im, COLOR_BGR2BGRA);
      ImageRef *ir = display->imageNew(WIDTH, HEIGHT, im.data, Display::BGRA);
      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
}

Mat get_image(){
  Mat im;
  const unsigned char *image = camera->getImage();
  Mat img = Mat(Size(WIDTH, HEIGHT), CV_8UC4);
  img.data = (uchar *)image;
  cvtColor(img, im, COLOR_BGRA2BGR);
  return im;
}

Mat get_mask(Mat im, int color){
    cvtColor(im, frame_HSV, COLOR_BGR2HSV);
    if (color == CYAN){
      inRange(frame_HSV, Scalar(80, 50, 50), Scalar(100, 255, 255), frame_threshold);
    }
    if (color == OBJ)
    {
      inRange(frame_HSV, Scalar(22, 50, 50), Scalar(25, 255, 255), frame_threshold);
    }
    if (color == YELLOW)
    {
      inRange(frame_HSV, Scalar(27, 50, 50), Scalar(35, 255, 255), frame_threshold);
    }
    
    //cvtColor(frame_threshold, frameBGR, COLOR_GRAY2BGR);
    return frame_threshold;
}

bool compareContourAreas (vector<Point> contour1, vector<Point> contour2) {
    double i = fabs(contourArea(Mat(contour1)) );
    double j = fabs(contourArea(Mat(contour2)) );
    return (i > j );
}

vector<vector<Point>> get_contours(Mat mask){
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
    sort(contours.begin(), contours.end(), compareContourAreas);
    return contours;
}

Point2f get_centroid(vector<Point> contour){
  Point2f mc;
  Moments mu;
  mu = moments( contour);      
  mc = Point2f( static_cast<float>(mu.m10 / (mu.m00 + 1e-5)), static_cast<float>(mu.m01 / (mu.m00 + 1e-5))); 
  //cout <<  mc.x << endl;
  return mc;
}

void find_floor(int color){
  Mat frame = get_image();
  Mat out = get_mask(frame, color);
  vector<vector<Point>> contours = get_contours(out);
  
  if (contours.size() == 0){
    set_velocity(TURN_SPEED, -TURN_SPEED);
  }
  else{
    double x = get_centroid(contours[0]).x;
    if ( (x < 250) || (x > 340)){
      set_velocity(TURN_SPEED, -TURN_SPEED);
    }
    else{
      double y = get_centroid(contours[0]).y;
      //cout << y << endl;
      if ( y < 300){
        set_velocity(TURN_SPEED, TURN_SPEED);
      }
      else{
      set_velocity(0, 0);
      if (currentMosaic == FIND_CYAN){
        currentMosaic = CUBE;
      }
      }
    }
  }
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
}

void follow_contour(vector<Point> contour){
  double x = get_centroid(contour).x;
}

void find_object(){
  Mat frame = get_image();
    Mat out = get_mask(frame, OBJ);
    vector<vector<Point>> contours = get_contours(out);
    vector<Point> approx;
    approxPolyDP(contours[0], approx, arcLength(contours[0], true)*0.02, true);
}

void mosaic_area(){
  switch (currentMosaic)
    {
    case FIND_CYAN:
      find_floor(CYAN);
      break;
    
    case CUBE:




      break;
    
    
    }
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
    
    // switch (currentStage)
    // {
    // case LINE_FOLLOW:
    //   follow_line();
    //   sn_digital_value();
    //   if (snDigitalValue[LEFT]){
    //     currentStage = WALL_FOLLOW;
    //     initialize_wall_PID();
    //   }
    //   break;
    
    // case WALL_FOLLOW:
      
    //   follow_maze();
    //   break;
    
    // }
    //find_floor(CYAN);
    mosaic_area();
    
    
    // cout << approx.size() << endl;
    

    
  

  };
  
  delete robot;
  return 0;
}



