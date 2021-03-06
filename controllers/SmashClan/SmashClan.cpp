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

#define TIME_STEP 16
#define MAX_SPEED 20
#define LINE_TROSHOLD 446
#define WALL_TRESHOLD 690
#define DELTA 0.1
#define SPEED 8
#define TURN90 5.69
#define TURN_SPEED 2
#define MOSAIC_SPEED 5
#define FRONT 0
#define RIGHT 1
#define LEFT 2
#define CELL_WIDTH 10
#define OBJ_CLOSE 0
#define BALL_CLOSE 0.005
#define CMPL_CLOSE 0.024
#define CMPL_OPEN 0.016
#define OBJ_OPEN 0.005
#define BALL_OPEN 0.001
#define WIDTH 640
#define HEIGHT 640
#define CYAN 0
#define OBJ 1
#define YELLOW 2
#define MAGENTA 3
#define WHITE 4
#define TEMP_BALL 7
#define RED 5
#define BLUE 6
#define BALL BLUE


using namespace webots;
using namespace std;
using namespace cv;

////////////////////////////////////STAGES////////////////////////////////////////////////////

enum Stages
    {
        LINE_FOLLOW,
        WALL_FOLLOW,
        MOSAIC_AREA,
        DASH_LINE,
        SHOOTING,
        KICKING,
        FINISHED
    };
Stages currentStage = LINE_FOLLOW;

// ////////////////////////////////////MOSAIC SUB-STAGES////////////////////////////////////////////////////

enum Mosaic
    {
        FIND_CYAN,
        GOTO_CUBE,
        FIND_MAGENTA_C,
        FIND_YELLOW_C,
        FIND_SQUARE,
        FIND_MAGENTA_RESET,
        FIND_CYAN_2,
        GOTO_CYLINDER,
        FIND_MAGENTA_CY,
        FIND_YELLOW_CY,
        FIND_RING,
        FIND_MAGENTA_P,
        FIND_BALL,
        FIND_MAGENTA_P2,
        FIND_RING_P
        
    };
Mosaic currentMosaic = FIND_CYAN;


////////////////////////////////////DEFINE OBJECTS////////////////////////////////////////////////////
Robot *robot;
Motor *right_motor;
Motor *left_motor;
Motor *servo;
Motor *right_linear;
Motor *left_linear;
DistanceSensor *ds[10];
PositionSensor *ps[3];
PositionSensor *servoEncoder;
PositionSensor *rightLinearEncoder;
PositionSensor *leftLinearEncoder;
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
double rightLinearValue;
double leftLinearValue;
Mat frame, frame_HSV, frame_threshold, frameBGR;
double mosaicError, veloCorrection;
int temp = 1;
int cylinder_pos = LEFT;
int temp_2 = 1;




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
  kp = 1;            //2
  ki = 0;
  kd = 0.5;            //1
  previousError = 0.0;
  accumulateError = 0.0;
}

void follow_line(){
  read_ds();
  //cout << dsDigitalValue[0]<< dsDigitalValue[1]<< dsDigitalValue[2]<< dsDigitalValue[3]<< dsDigitalValue[4]<< dsDigitalValue[5]<< dsDigitalValue[6]<< endl;
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
    if (snValue[i] > 400){
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
    //cout << "b" << endl;
    if (!(snDigitalValue[FRONT])){
      follow_wall();
      //set_velocity(SPEED, SPEED);
    }
    else{
      //cout << "C" << endl;
      move_specific_distance(4.5, 4.5);
      if (snDigitalValue[RIGHT]){
        turn(RIGHT);
      }
      turn(RIGHT);
    }
  }
  else{
    if (snDigitalValue[FRONT]){
      //cout << "S" << endl;
      //move_specific_distance(2, 2);
      turn(LEFT);
      move_specific_distance(3, 3);
      
    }
    else{
      //cout << "A" << endl;
      move_specific_distance(11.3, 11.3);
      turn(LEFT);
      move_specific_distance(3.7, 3.7);
    }

    }
  }


///////////////////////////////////ARM/////////////////////////////////////////////////////////

void initialize_servo(){
    servo = robot->getMotor("servo");
    servoEncoder = robot->getPositionSensor("servo_en");
    servoEncoder->enable(TIME_STEP);

    right_linear = robot->getMotor("r_linear");
    rightLinearEncoder = robot->getPositionSensor("r_linear_en");
    rightLinearEncoder->enable(TIME_STEP);

    left_linear = robot->getMotor("l_linear");
    leftLinearEncoder = robot->getPositionSensor("l_linear_en");
    leftLinearEncoder->enable(TIME_STEP);
}

void pull_down(){
  servo->setControlPID(2, 0, 0);
  servo->setVelocity(1);
  servo->setPosition(-0.55);
  double currentPosition;
  do
  { 
    currentPosition = servoEncoder->getValue();
    robot->step(TIME_STEP);
  } while (fabs(currentPosition + 0.55 ) > 0.01);
  robot->step(TIME_STEP);
  
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
  } while (fabs(currentPosition - 1 ) > 0.05);
  robot->step(TIME_STEP);
}

void open_grabber(double target){
  right_linear->setControlPID(2, 0, 0);
  right_linear->setPosition(target);
  right_linear->setVelocity(0.01);

  left_linear->setControlPID(2, 0, 0);
  left_linear->setPosition(-target);
  left_linear->setVelocity(0.01);
  double currentPosition;
  do
  { 
    currentPosition = rightLinearEncoder->getValue();
    robot->step(TIME_STEP);
    //cout << currentPosition << endl;
  } while (fabs(currentPosition - target ) > 0.002);
}

void close_grabber(double target){
  right_linear->setControlPID(2, 0, 0);
  right_linear->setPosition(-target);
  right_linear->setVelocity(0.01);

  left_linear->setControlPID(2, 0, 0);
  left_linear->setPosition(target);
  left_linear->setVelocity(0.01);

  int time = 270;
  if ((currentMosaic == GOTO_CUBE) || (currentMosaic == GOTO_CYLINDER) || (currentMosaic == FIND_BALL)){
    time = 160;
  }
  for (int i = 0; i < time; i++){
    robot->step(TIME_STEP);
  }
//   double currentPosition;
//   do
//   { 
//     currentPosition = rightLinearEncoder->getValue();
//     robot->step(TIME_STEP);
//     //cout << currentPosition << endl;
//   } while (fabs(currentPosition + 0.02 ) > 0.001);
}

void move_distance(double x){
  get_encoder_value();
  double rightTarget = psValue[0] + x;
  double leftTarget = psValue[1] + x;
  double velo = SPEED;
  if ((currentMosaic == GOTO_CUBE) || (currentMosaic == GOTO_CYLINDER) || (currentMosaic == FIND_BALL) || (currentMosaic == FIND_SQUARE) || (currentMosaic == FIND_RING)){
    velo = 3;
  }
  if (x > 0){
    set_velocity(velo, velo);
  }
  else {
    set_velocity(-velo, -velo);
  }
  
  
  while ((fabs(rightTarget - psValue[0])> DELTA) || (fabs(leftTarget - psValue[1])> DELTA)){
    robot->step(TIME_STEP);
    get_encoder_value();
  }
  set_velocity(0, 0);
}

void shoot(double x){
  get_encoder_value();
  double rightTarget = psValue[0] + x;
  double leftTarget = psValue[1] + x;
  double velo = MAX_SPEED;
  set_velocity(velo, velo);
  while (((rightTarget - psValue[0])> DELTA) || ((leftTarget - psValue[1]) > DELTA)){
    //cout << rightTarget << "    " << psValue[0] << endl;
    robot->step(TIME_STEP);
    get_encoder_value();
  }
  set_velocity(0, 0);
}

void pick_object(){
    open_grabber(CMPL_OPEN);
    pull_down();
    move_distance(4.5);
    close_grabber(OBJ_CLOSE);
    open_grabber(OBJ_OPEN);
    move_distance(1);
    close_grabber(OBJ_CLOSE);
    pull_up();
    //set_velocity(3, 3);
}

void pick_ball(){
    open_grabber(CMPL_OPEN);
    pull_down();
    move_distance(4.5);
    close_grabber(BALL_CLOSE);
    pull_up();
    //set_velocity(3, 3);
}

void drop_object(){
  pull_down();
  open_grabber(OBJ_OPEN);
  if (currentMosaic == FIND_RING){
    move_distance(-1);
    close_grabber(OBJ_CLOSE);
    move_distance(1);
    open_grabber(CMPL_OPEN);
  }
  move_distance(-2.4);
  close_grabber(CMPL_CLOSE);
    //set_velocity(3, 3);
}

void drop_ball(){
  pull_down();
  open_grabber(BALL_OPEN);
  close_grabber(BALL_CLOSE);
  open_grabber(CMPL_OPEN);
  move_distance(-2.4);
  close_grabber(CMPL_CLOSE);
    //set_velocity(3, 3);
}

void drop_wrong_ball(){
  pull_down();
  open_grabber(BALL_OPEN);
  pull_up();
  close_grabber(CMPL_CLOSE);
    //set_velocity(3, 3);
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
    
    if (color == CYAN){
      cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(80, 50, 50), Scalar(100, 255, 255), frame_threshold);
    }
    else if (color == OBJ)
    { cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(15, 10, 10), Scalar(25, 255, 255), frame_threshold);
    }
    else if (color == YELLOW)
    { cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(27, 50, 50), Scalar(35, 255, 255), frame_threshold);
    }
    else if (color == MAGENTA)
    { cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(135, 50, 50), Scalar(155, 255, 255), frame_threshold);
    }
    else if (color == WHITE)
    { cvtColor(im, frame_HSV, COLOR_BGR2HLS);
      inRange(frame_HSV, Scalar(0, 140,0), Scalar(179, 255, 255), frame_threshold);
    }
    else if (color == RED)
    { Mat mask1, mask2;
      cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(0, 20, 20), Scalar(10, 255, 255), mask1);
      inRange(frame_HSV, Scalar(170, 20, 20), Scalar(179, 255, 255), mask2);
      bitwise_or(mask1, mask2, frame_threshold);
    }
    else if (color == BLUE)
    { cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(115, 30, 30), Scalar(125, 255, 255), frame_threshold);
    }
    else if (color == TEMP_BALL)
    { Mat mask1, mask2, mask3, mask4;
      cvtColor(im, frame_HSV, COLOR_BGR2HSV);
      inRange(frame_HSV, Scalar(0, 20, 20), Scalar(10, 255, 255), mask1);
      inRange(frame_HSV, Scalar(170, 20, 20), Scalar(179, 255, 255), mask2);
      bitwise_or(mask1, mask2, mask3);
      inRange(frame_HSV, Scalar(115, 30, 30), Scalar(125, 255, 255), mask4);
      bitwise_or(mask3, mask4, frame_threshold);
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
    set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
  }
  else{
    double x = get_centroid(contours[0]).x;
    double treshold = 10;
    if (color == CYAN){treshold = 10;}
    if (currentMosaic == FIND_MAGENTA_P2){treshold = 20;}
    if ( (x < (320 - treshold)) || (x > (320 + treshold))){
      set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
      // mosaicError = x - 320;
      // veloCorrection = 0.005 * mosaicError;

      // set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
    }
    else{
      double y = get_centroid(contours[0]).y;
      //cout << y << endl;
      double tresh = 50;
      if (color == MAGENTA){tresh = 500;}
      if (currentMosaic == FIND_MAGENTA_P){tresh = 600;}
      if ((currentMosaic == FIND_MAGENTA_RESET) && (cylinder_pos == LEFT)){tresh = 400;}
      else if ((currentMosaic == FIND_MAGENTA_RESET) && (cylinder_pos == RIGHT)){tresh = 600;}
      if ( y < tresh){
        //cout << y << endl;
      mosaicError = x - 320;
      veloCorrection = 0.005 * mosaicError;

      set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
        //cout << y << endl;
      }
      else{
        set_velocity(0, 0);
        if (currentMosaic == FIND_CYAN){
          currentMosaic = GOTO_CUBE;
        }
        else if (currentMosaic == FIND_MAGENTA_C){ 
          move_distance(5);
          currentMosaic = FIND_YELLOW_C;
        }
        else if (currentMosaic == FIND_YELLOW_C){ 
          currentMosaic = FIND_SQUARE;
        }
        else if (currentMosaic == FIND_MAGENTA_RESET){ 
          currentMosaic = FIND_CYAN_2;
        }
        else if (currentMosaic == FIND_CYAN_2){ 
          currentMosaic = GOTO_CYLINDER;
        }
        else if (currentMosaic == FIND_MAGENTA_CY){ 
          currentMosaic = FIND_YELLOW_CY;
        }
        else if (currentMosaic == FIND_YELLOW_CY){ 
          currentMosaic = FIND_RING;
        }
        else if (currentMosaic == FIND_MAGENTA_P){ 
          currentMosaic = FIND_BALL;
        }
        else if (currentMosaic == FIND_MAGENTA_P2){ 
          currentMosaic = FIND_RING_P;
        }
      }
    }
  }
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
}

void follow_contour(vector<Point> contour){
  double x = get_centroid(contour).x;
  double y = get_centroid(contour).y;
  if (y < 320){
      mosaicError = x - 320;
      veloCorrection = 0.005 * mosaicError;
      set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
      //set_velocity(MOSAIC_SPEED, MOSAIC_SPEED);
    
  }
  else{
    if (currentMosaic == GOTO_CUBE){
      move_distance(6);
      set_velocity(0, 0);
      pick_object();
      currentMosaic = FIND_MAGENTA_C;
    }
    else if (currentMosaic == FIND_BALL) {
      move_distance(4);
      set_velocity(0, 0);
      pick_ball();
      currentMosaic = FIND_MAGENTA_P2;
    }
  }
}

void find_cylinder(){
  Mat frame = get_image();
  Mat out = get_mask(frame, OBJ);
  vector<vector<Point>> contours = get_contours(out);
  vector<Point> approx;
  if (contours.size() == 0){
      if (cylinder_pos == LEFT){
            set_velocity(MOSAIC_SPEED*0.5, -MOSAIC_SPEED*0.5);
          }
          else{
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
  }
  for (size_t i = 0; i < contours.size(); i++){
      if ( i == 2){break;}
      approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.003, true);
      Rect minRect = boundingRect(contours[i]);
      double aspectRatio = ((double)minRect.width / (double)minRect.height);
      double areaRatio = ((double)minRect.width * (double)minRect.height) / fabs(contourArea(Mat(contours[i])));
      
      //cout << (double)minRect.width / (double)minRect.height << "    " <<areaRatio << endl;
      if (((double)minRect.width / (double)minRect.height < 1.5) && (areaRatio < 1.5) && (fabs(contourArea(Mat(contours[i]))) > 500)){
        //cout <<  approx.size() << "  " << aspectRatio << endl;
        double x = get_centroid(contours[i]).x;
        double y = get_centroid(contours[i]).y;

        if (y < 320){
          if ((x > 300) && (x <340)){
            mosaicError = x - 320;
            veloCorrection = 0.005 * mosaicError;
            set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
          }
          else{
            //cout << i << "A" << (fabs(contourArea(Mat(contours[i])))) <<endl;
            if (cylinder_pos == LEFT){
              set_velocity(MOSAIC_SPEED*0.5, -MOSAIC_SPEED*0.5);
            }
            else{
              set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
            }
          }
          
          //set_velocity(MOSAIC_SPEED, MOSAIC_SPEED);
        }
        else if ((x > 300) && (x <340)){
          if ((aspectRatio < 1) && (approx.size() > 8)){
            //cout << aspectRatio <<"  "<<  y << endl;
            move_distance(5);
            set_velocity(0, 0);
            pick_object();
            currentMosaic = FIND_MAGENTA_CY;
          }
          else{
            if (cylinder_pos == LEFT){
              turn(RIGHT);
              move_distance(5);
              turn(LEFT);
            }
            else{
              turn(LEFT);
              move_distance(5);
              turn(RIGHT);
            }
            
          }
          
        }
        else {
          //cout << "B" << endl;
          if (cylinder_pos == LEFT){
            set_velocity(MOSAIC_SPEED*0.5, -MOSAIC_SPEED*0.5);
          }
          else{
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
         
        }
        break;
      }
      else{
        //cout << i << "C" << (fabs(contourArea(Mat(contours[i])))) << endl;
        if (cylinder_pos == LEFT){
            set_velocity(MOSAIC_SPEED*0.5, -MOSAIC_SPEED*0.5);
          }
          else{
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
      }
  }
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
}

void find_cube(){
  Mat frame = get_image();
  Mat out = get_mask(frame, OBJ);
  vector<vector<Point>> contours = get_contours(out);
  vector<Point> approx;
  if (contours.size() == 0){
      set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
  }
  for (size_t i = 0; i < contours.size(); i++){
      if ( i == 2){break;}
      approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.003, true);
      Rect minRect = boundingRect(contours[i]);
      //double aspectRatio = ((double)minRect.width / (double)minRect.height);
      double areaRatio = ((double)minRect.width * (double)minRect.height) / fabs(contourArea(Mat(contours[i])));
      //cout << (double)minRect.width / (double)minRect.height << "    " <<areaRatio << endl;
      if (((double)minRect.width / (double)minRect.height < 1.5) && (areaRatio < 1.5) && (fabs(contourArea(Mat(contours[i]))) > 500)){
        //cout <<  approx.size() << "  " << aspectRatio << endl;
        double x = get_centroid(contours[i]).x;
        double y = get_centroid(contours[i]).y;
        if (y < 320){
          if ((x > 300) && (x <340)){
            mosaicError = x - 320;
            veloCorrection = 0.005 * mosaicError;
            set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
            //cout << "H  "<< i << endl;
            break;
          }
          else{
            //cout << i << endl;
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
          
          //set_velocity(MOSAIC_SPEED, MOSAIC_SPEED);
        }
        else if ((x > 310) && (x <330)){
          if (approx.size() < 8){
            //cout << aspectRatio <<"  "<<  y << endl;
            move_distance(6); //changed
            set_velocity(0, 0);
            pick_object();
            currentMosaic = FIND_MAGENTA_C;
          }
          else{
            cylinder_pos = RIGHT;
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
        }
      }
      else{
        set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
      }
  }
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
}

// void find_object(){
//     Mat frame = get_image();
//     Mat out = get_mask(frame, OBJ);
//     vector<vector<Point>> contours = get_contours(out);
//     vector<Point> approx;

//     if (contours.size() == 0){
//       set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
//     }
    
//     for (size_t i = 0; i < contours.size(); i++){
//       if ( i == 2){break;}
//       approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.005, true);
//       cout << i << "    " <<approx.size() << endl;
//       if ((approx.size() <= 10) && (currentMosaic == GOTO_CUBE) && (fabs(contourArea(Mat(contours[i]))) > 500)){
//         follow_contour(contours[i]);
//         break;
//       }
//       else{
//         set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
//       }
//     } 
    
    
    
//     cvtColor(out, frameBGR, COLOR_GRAY2BGR);
//     display_image(frameBGR);
// }

void find_path(){
  sn_digital_value();
  if (snDigitalValue[LEFT]){
    move_distance(15);
    turn(LEFT);
    move_distance(6);

    initialize_PID();
    currentStage = DASH_LINE;
  }
  else{
    follow_wall();
  }
}

void follow_hole(vector<Point> contour){
  double x = get_centroid(contour).x;
  double y = get_centroid(contour).y;
  if ((y < 350)){

    mosaicError = x - 320;
    veloCorrection = 0.005 * mosaicError;
    set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
    //cout << y << endl;
    // if ((50 < x) && (x < 315)){
    //   set_velocity(MOSAIC_SPEED*0.5, -MOSAIC_SPEED*0.5);
    // }
    // else if ((x > 325) && (x < 600)){
    //   set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
    // }
    // else{
    //   set_velocity(MOSAIC_SPEED, MOSAIC_SPEED);
    // }
  }
  else{
    if (currentMosaic == FIND_SQUARE){
      move_distance(3.9);
      set_velocity(0, 0);
      drop_object();
      move_distance(3);
      move_distance(-3);
      pull_up();
      currentMosaic = FIND_MAGENTA_RESET;
    }
    else if (currentMosaic == FIND_RING)
    {
      move_distance(4.5);
      set_velocity(0, 0);
      drop_object();
      move_distance(3);
      move_distance(-3);
      pull_up();
      currentMosaic = FIND_MAGENTA_P;
    }
    else if (currentMosaic == FIND_RING_P)
    { 
      move_distance(5);
      turn(RIGHT);
      find_path();
      //currentMosaic = FIND_MAGENTA_P;
    }
    
  }
}

void find_hole(){
    Mat frame = get_image();
    Mat out = get_mask(frame, WHITE);
    vector<vector<Point>> contours = get_contours(out);
    vector<Point> approx;

    if (contours.size() == 0){
      set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
    }
    
    for (size_t i = 0; i < contours.size(); i++){
      if (i == 3){break;}
      approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.005, true);
      Rect minRect = boundingRect(contours[i]);
      //cout << i << "   "<< fabs((double)minRect.width / (double)minRect.height) << endl;
      if (fabs((double)minRect.width / (double)minRect.height) < 1.4){
        if ((approx.size() < 20) && currentMosaic == FIND_SQUARE){
          follow_hole(contours[i]);
          //cout << i << "   "<< approx.size() << endl;
          break;
          
        }
        else if ((approx.size() >= 20) && currentMosaic == FIND_RING ){
          follow_hole(contours[i]);
          //cout << i << "   "<< approx.size() << endl;
          break;  
        }
        else if ((approx.size() >= 20) && currentMosaic == FIND_RING_P){
          follow_hole(contours[i]);
          //cout << i << "   "<< approx.size() << endl;
          break;
          //cout << i << "   "<< approx.size() << endl;
        }
        else{
          set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
        }
      }
      else{
          set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
        }
      
    }
    cvtColor(out, frameBGR, COLOR_GRAY2BGR);
    display_image(frameBGR);
}

void find_ball(){
    Mat frame = get_image();
    Mat out = get_mask(frame, BALL);
    vector<vector<Point>> contours = get_contours(out);
    Point2f center;
    float radius;

    if (contours.size() == 0){
      set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
    }
    
    for (size_t i = 0; i < contours.size(); i++){
      if ( i == 2){break;}
      minEnclosingCircle(contours[i], center, radius);
      double conArea = fabs(contourArea(Mat(contours[i])));
      double boundArea = 3.14 * radius * radius;

      if ((boundArea / conArea) < 1.3 ){
        follow_contour(contours[i]);
        break;
      }
      else{
        set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
      }
    }
    cvtColor(out, frameBGR, COLOR_GRAY2BGR);
    display_image(frameBGR);
}

// double get_areaRatio(vector<Point> contour){
//   double conArea = fabs(contourArea(Mat(contour)));
//   double boundArea = 3.14 * radius * radius;
//   return boundArea / conArea;
// }

void find_close_ball(){
  Mat frame = get_image();
  Mat out = get_mask(frame, TEMP_BALL);
  vector<vector<Point>> contours = get_contours(out);
  Point2f center;
  float radius;

  if (contours.size() == 0){
    set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
  }
  
  for (size_t i = 0; i < contours.size(); i++){
    if ( i == 2){break;}
    minEnclosingCircle(contours[i], center, radius);
    double conArea = fabs(contourArea(Mat(contours[i])));
    double boundArea = 3.14 * radius * radius;

    if ((boundArea / conArea) < 1.3 ){
      double x = get_centroid(contours[i]).x;
      double y = get_centroid(contours[i]).y;
      if (y < 320){
        mosaicError = x - 320;
        veloCorrection = 0.005 * mosaicError;
        set_velocity(MOSAIC_SPEED - veloCorrection, MOSAIC_SPEED + veloCorrection);
        //set_velocity(MOSAIC_SPEED, MOSAIC_SPEED);
      }
      else{
        set_velocity(0, 0);
        //cout << fabs(contourArea(Mat(contours[i]))) << endl;
        Mat frame1 = get_image();
        Mat out1 = get_mask(frame1, BALL);
        vector<vector<Point>> con = get_contours(out1);
        if (con.size() != 0){
          double xx = get_centroid(con[i]).x;
          //cout << xx << endl;
          if ((fabs(contourArea(Mat(con[0]))) > 9000) && (xx > 310) && (xx < 330)){
            move_distance(4);
            set_velocity(0, 0);
            pick_ball();
            currentMosaic = FIND_MAGENTA_P2;
            break;
          }
          else if (temp_2 == 1){
            move_distance(4);
            set_velocity(0, 0);
            pick_ball();
            turn(RIGHT); turn(RIGHT);
            drop_wrong_ball();
            turn(RIGHT);
            temp_2 = 0;
          }
          else{
            set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
          }
        }
      }
      break;
    }
    else{
      set_velocity(-MOSAIC_SPEED*0.5, MOSAIC_SPEED*0.5);
    }
  }



  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);

}

void is_mosaic(){
  Mat frame = get_image();
  Mat out = get_mask(frame, MAGENTA);
  vector<vector<Point>> contours = get_contours(out);
  if (contours.size() != 0){
    double y = get_centroid(contours[0]).y;
    if ((y > 320) && (fabs(contourArea(Mat(contours[0]))) > 20000)){
      //cout << "T" << endl;
      move_distance(15);
      move_specific_distance(2, -2);
      currentStage = MOSAIC_AREA;
      
    }
  }
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
}

bool is_junction(){
  Mat frame = get_image();
  Mat out = get_mask(frame, BLUE);
  cvtColor(out, frameBGR, COLOR_GRAY2BGR);
  display_image(frameBGR);
  vector<vector<Point>> contours = get_contours(out);
  if (contours.size() != 0){
    if (fabs(contourArea(Mat(contours[0]))) > 25000){
      read_ds();
      if (dsDigitalValue[0] == 1){
        return true;
      }
      
    }
  }
  return false;
}

void is_white_line(){
  read_ds();
  if ((dsDigitalValue[0]) & (dsDigitalValue[1]) & (dsDigitalValue[2]) & (dsDigitalValue[3]) & (dsDigitalValue[4]) & (dsDigitalValue[5]) & (dsDigitalValue[6])){
    set_velocity(0, 0);
    if (BALL == RED){
      move_specific_distance(-0.85, 0.85);
      currentStage = KICKING;
    }
    else{
      move_specific_distance(0.85, -0.85);
      currentStage = KICKING;
    }

  }
}

void red_path(){
  if (temp == 1){
    move_distance(5.1);
    turn(LEFT);
    temp = 0;
  }
  follow_line();
  is_white_line();
}

void blue_path(){
  if (temp == 1){
    move_distance(20);
    move_specific_distance(2.9, -2.9);
    set_velocity(SPEED, SPEED);
    temp = 0;
  }
  is_white_line();
}

void mosaic_area(){
  switch (currentMosaic)
    {
    case FIND_CYAN:
      find_floor(CYAN);
      break;
    
    case GOTO_CUBE:
      find_cube();
      break;
    
    case FIND_MAGENTA_C:
      find_floor(MAGENTA);
      break;
    
    case FIND_YELLOW_C:
      find_floor(YELLOW);
      break;

    case FIND_SQUARE:
      find_hole();
      break;

    case FIND_MAGENTA_RESET:
      find_floor(MAGENTA);
      break;

    case FIND_CYAN_2:
      find_floor(CYAN);
      break;
    
    case GOTO_CYLINDER:
      find_cylinder();
      break;

    case FIND_MAGENTA_CY:
      find_floor(MAGENTA);
      break;
    
    case FIND_YELLOW_CY:
      find_floor(YELLOW);
      break;
    
    case FIND_RING:
      find_hole();
      break;
    
    case FIND_MAGENTA_P:
      find_floor(MAGENTA);
      break;

    case FIND_BALL:
      find_close_ball();
      break;

    case FIND_MAGENTA_P2:
      find_floor(MAGENTA);
      break;

    case FIND_RING_P:
      find_hole();
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
    
    switch (currentStage)
    {
    case LINE_FOLLOW:
      follow_line();
      sn_digital_value();
      if ((snDigitalValue[LEFT]) || (snDigitalValue[RIGHT])){
        move_specific_distance(0.8, 0.8);
        currentStage = WALL_FOLLOW;
        initialize_wall_PID();
      }
      break;

    case WALL_FOLLOW:
      follow_maze();
      is_mosaic();
      break;

    case MOSAIC_AREA:
      mosaic_area();
      break;
    
    case DASH_LINE:
      if (is_junction()) {
         currentStage = SHOOTING;
         break;
      }
      follow_line();
      break;

    case SHOOTING:
      if (BALL == RED){
        red_path();
      }
      else{
        blue_path();
      }
      break;
    
    case KICKING:
      set_velocity(0, 0);
      drop_ball();
      shoot(6);
      currentStage = FINISHED;
      break;
    
    case FINISHED:
      //robot->step(TIME_STEP) = -1;
      cout << "FINISHED" << endl;
      break;

    }

    //find_close_ball();
    // find_floor(CYAN);
    // if (t){
    // turn(RIGHT);
    // t=0;
    // }
    //set_velocity(-10, -10);
    //follow_line();
    //cout << currentMosaic << endl;
  

  };
  
  delete robot;
  return 0;
}



