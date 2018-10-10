/* -----------------------------------------------------------------------------
  - Original Project: Body Movement Crawling robot
  - Original Author:  panerqiang@sunfounder.com
  - Date:  2015/1/27

  - Remix Project by Wilmar
  - Date: 2018/09/25
  - Change log: 
      > Add more movement and combine demo and controlled move in one
      > Using PCA9685 16 channel Servo controller for Arduino for motor driver
      > Add NodeMCU for Wifi Controller as AP
   -----------------------------------------------------------------------------
  - Overview
  - This project was written for the Crawling robot desigened by Sunfounder.
    This version of the robot has 4 legs, and each leg is driven by 3 servos.
  This robot is driven by a Ardunio Nano Board with an expansion Board.
  We recommend that you view the product documentation before using.
  - Request
  - This project requires some library files, which you can find in the head of
    this file. Make sure you have installed these files.
  - How to
  - Before use,you must to adjust the robot,in order to make it more accurate.
    - Adjustment operation
    1.uncomment ADJUST, make and run
    2.comment ADJUST, uncomment VERIFY
    3.measure real sites and set to real_site[4][3], make and run
    4.comment VERIFY, make and run
  The document describes in detail how to operate.

/* Includes ------------------------------------------------------------------*/

#define DEBUG

#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266mDNS.h>

#include <Adafruit_PWMServoDriver.h>
#include <Ticker.h>
#include <SerialCommand.h>

SerialCommand SCmd; // The demo SerialCommand object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Ticker ticker;

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs

enum LegServoIndex : uint8_t
{
  LSI_FR = 0, //Front Right
  LSI_RR = 1, //Rear Right
  LSI_FL = 2, //Front Left
  LSI_RL = 3 // Rear Left
};

enum JointServoIndex : uint8_t
{
  JSI_Elbow = 0,
  JSI_Foot = 1,
  JSI_Shoulder = 2
};

//define servos' ports
static const int NUM_LEGS = 4;
static const int NUM_JOINTS_PER_LEG = 3;
const uint8_t servo_pin[NUM_LEGS][NUM_JOINTS_PER_LEG] = {{0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14}};
bool servo_move_enabled[NUM_LEGS][NUM_JOINTS_PER_LEG];

const float SERVO_FREQUENCY = 50.0f;
const float SERVO_PWM_MIN = 90;
const float SERVO_PWM_MAX = 540;
const float SERVO_PWM_RANGE = SERVO_PWM_MAX - SERVO_PWM_MIN;

const int servo_update_rate_ms = 1000 / SERVO_FREQUENCY;
volatile unsigned long last_servo_update_time = 0;

/* Size of the robot ---------------------------------------------------------*/
//TODO - set these properly
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 71;
const float z_absolute = -28;

/* Constants for movement ----------------------------------------------------*/
const float z_default = -50, z_up = -30, z_boot = z_absolute;
const float x_default = 62, x_offset = 0;
const float y_start = 0, y_step = 40;
const float y_default = x_default;

/* variables for movement ----------------------------------------------------*/
enum Axis
{
  XAxis = 0,
  YAxis = 1,
  ZAxis = 2,
  NUM_AXIS = 3
};

float current_foot_pos[NUM_LEGS][NUM_AXIS];     //real-time coordinates of the end of each leg
volatile float target_foot_pos[NUM_LEGS][NUM_AXIS];      //expected coordinates of the end of each leg
float foot_move_speed[NUM_LEGS][NUM_AXIS];               //each axis' speed, needs to be recalculated before each movement
int joint_offsets_deg[NUM_LEGS][NUM_JOINTS_PER_LEG];
int joint_offset_scale[NUM_LEGS][NUM_JOINTS_PER_LEG];

float move_speed = 1.4;           //movement speed
float speed_multiple = 1.0f / servo_update_rate_ms;         //movement speed multiple
const float spot_turn_speed = 4;
const float leg_move_speed = 8;
const float body_move_speed = 3;
const float stand_seat_speed = 1;

const float MAX_FOOT_POS = 255.0f;
const float KEEP_CURRENT_FOOT_POS = 256.0f;

//define PI for calculation
const float pi = 3.1415926;

/* Constants for turn --------------------------------------------------------*/
//temp length
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_elbowDeg = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);

//site for turn
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_elbowDeg);
const float turn_y0 = temp_b * sin(temp_elbowDeg) - turn_y1 - length_side;

/* ---------------------------------------------------------------------------*/
String lastComm = "";
int ledPulse = 0;

// w 0 2: body init
// w 0 1: stand
// w 0 0: sit
// w 1 x: forward x step
// w 2 x: back x step
// w 3 x: right turn x step
// w 4 x: left turn x step
// w 5 x: hand shake x times
// w 6 x: hand wave x times
// w 7 : dance
// w 8 x: head up x times
// w 9 x: head down x times
// w 10 x: body right x times
// w 11 x: body left x times
// w 12: Body init pose
// w 13: body up
// w 14: body down
// w 15: reset pos
// w 16: body twist right
// w 17: body twist left

#define W_STAND_SIT 0
#define W_FORWARD 1
#define W_BACKWARD 2
#define W_LEFT 3
#define W_RIGHT 4
#define W_SHAKE 5
#define W_WAVE 6
#define W_DANCE 7
#define W_HEAD_UP 8
#define W_HEAD_DOWN 9
#define W_B_RIGHT 10
#define W_B_LEFT 11
#define W_B_INIT 12
#define W_HIGHER 13
#define W_LOWER 14
#define W_SET 15
#define W_TW_R 16
#define W_TW_L 17

void offset_cmd(void)
{
  char *arg;
  int leg, joint, offset;
  
  arg = SCmd.next();
  leg = atoi(arg);
  arg = SCmd.next();
  joint = atoi(arg);
  arg = SCmd.next();
  offset = atoi(arg);
  Serial.printf("Leg: %d Joint: %d Offset: %d\r\n", leg, joint, offset);

  joint_offsets_deg[leg][joint] = offset;
}

void servo_cmd(void)
{
  char *arg;
  int leg, joint, angle;
  
  arg = SCmd.next();
  leg = atoi(arg);
  arg = SCmd.next();
  joint = atoi(arg);
  arg = SCmd.next();
  angle = atoi(arg);
  Serial.printf("Leg: %d Joint: %d Angle: %d\r\n", leg, joint, angle);

  servo_move_enabled[leg][joint] = false;
  const int turnOffTime = max(SERVO_PWM_MIN, min(SERVO_PWM_MAX, angle / 180.0f * SERVO_PWM_RANGE + SERVO_PWM_MIN));
  pwm.setPWM(servo_pin[leg][joint], 0, turnOffTime);  
}

void pwm_cmd(void)
{
  char *arg;
  int leg, joint, _pwm;
  
  arg = SCmd.next();
  leg = atoi(arg);
  arg = SCmd.next();
  joint = atoi(arg);
  arg = SCmd.next();
  _pwm = atoi(arg);
  Serial.printf("Leg: %d Joint: %d PWM: %d\r\n", leg, joint, _pwm);

  servo_move_enabled[leg][joint] = false;
  pwm.setPWM(servo_pin[leg][joint], 0, _pwm);  
}

void action_cmd(void)
{
  char *arg;
  int action_mode, n_step;
  Serial.print("Action: ");
  arg = SCmd.next();
  action_mode = atoi(arg);
  arg = SCmd.next();
  n_step = atoi(arg);
  do_action(action_mode, n_step);  
}

void do_action(int action_mode, int n_step)
{
  switch (action_mode)
  {
  case W_FORWARD:
    Serial.print("Step forward ");
    Serial.println(n_step);
    lastComm = "FWD";
    if (!is_stand())
      stand();
    step_forward(n_step);
    break;

  case W_BACKWARD:
    Serial.print("Step back ");
    Serial.println(n_step);
    lastComm = "BWD";
    if (!is_stand())
      stand();
    step_back(n_step);
    break;

  case W_LEFT:
    Serial.print("Turn left ");
    Serial.println(n_step);
    lastComm = "LFT";
    if (!is_stand())
      stand();
    turn_left(n_step);
    break;

  case W_RIGHT:
    Serial.print("Turn right ");
    Serial.println(n_step);
    lastComm = "RGT";
    if (!is_stand())
      stand();
    turn_right(n_step);
    break;

  case W_STAND_SIT:
    Serial.print("Stand or Sit: ");
    lastComm = "";
    if (n_step)
    {
      Serial.println("Stand");
      stand();
    }
    else
    {
      Serial.println("Sit");
      sit();
    }
    break;

  case W_SHAKE:
    Serial.print("Hand shake ");
    Serial.println(n_step);
    lastComm = "";
    hand_shake(n_step);
    break;

  case W_WAVE:
    Serial.print("Hand wave ");
    Serial.println(n_step);
    lastComm = "";
    hand_wave(n_step);
    break;

  case W_DANCE:
    Serial.print("Dance ");
    Serial.println(n_step);
    lastComm = "";
    body_dance(n_step);
    break;

  case W_SET:
    Serial.println("Set");
    for(int i = 0; i < NUM_LEGS; ++i)
    {
      for(int j = 0; j < NUM_JOINTS_PER_LEG; ++j)
      {
        joint_offsets_deg[i][j] = 0;
      }
    }
    stand();
    break;

  case W_HIGHER:
    Serial.println("Higher");
    joint_offsets_deg[LSI_FL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] += 4;
    stand();
    break;

  case W_LOWER:
    Serial.println("Lower");
    joint_offsets_deg[LSI_FL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] -= 4;
    stand();
    break;

  case W_HEAD_UP:
    Serial.println("Head up");
    joint_offsets_deg[LSI_FL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] -= 4;
    stand();
    break;

  case W_HEAD_DOWN:
    Serial.println("Head down");
    joint_offsets_deg[LSI_FL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] += 4;
    stand();
    break;

  case W_B_RIGHT:
    Serial.println("body right");
    if (!is_stand())
      stand();
      joint_offsets_deg[LSI_FL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] -= 4;
    stand();
    break;

  case W_B_LEFT:
    Serial.println("body left");
    if (!is_stand())
      stand();
    joint_offsets_deg[LSI_FL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_FR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Elbow] += 4;
    joint_offsets_deg[LSI_RR][JSI_Elbow] -= 4;
    joint_offsets_deg[LSI_FL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Foot] += 4;
    joint_offsets_deg[LSI_RL][JSI_Foot] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Foot] += 4;
    stand();
    break;

  case W_B_INIT:
    Serial.println("Body init");
    lastComm = "";
    sit();
    b_init();
    for(int i = 0; i < NUM_LEGS; ++i)
    {
      for(int j = 0; j < NUM_JOINTS_PER_LEG; ++j)
      {
        joint_offsets_deg[i][j] = 0;
      }
    }
    stand();
    break;

  case W_TW_R:
    Serial.println("Body twist right");
    joint_offsets_deg[LSI_FL][JSI_Shoulder] -= 4;
    joint_offsets_deg[LSI_FR][JSI_Shoulder] += 4;
    joint_offsets_deg[LSI_RL][JSI_Shoulder] += 4;
    joint_offsets_deg[LSI_RR][JSI_Shoulder] -= 4;
    stand();
    break;

  case W_TW_L:
    Serial.println("Body twist left");
    joint_offsets_deg[LSI_FL][JSI_Shoulder] += 4;
    joint_offsets_deg[LSI_FR][JSI_Shoulder] -= 4;
    joint_offsets_deg[LSI_RL][JSI_Shoulder] -= 4;
    joint_offsets_deg[LSI_RR][JSI_Shoulder] += 4;
    stand();
    break;

  default:
    Serial.println("Error");
    break;
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command)
{
  Serial.println("What?");
}

/*
  - is_stand
   ---------------------------------------------------------------------------*/
bool is_stand(void)
{
  return (current_foot_pos[LSI_FR][ZAxis] == z_default);
}

/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < NUM_LEGS; leg++)
  {
    set_site(leg, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < NUM_LEGS; leg++)
  {
    set_site(leg, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, z_default);
  }
  wait_all_reach();
}

/*
  - Body init
  - blocking function
   ---------------------------------------------------------------------------*/
void b_init(void)
{
  Serial.println("b_init start");
  set_site(0, x_default, y_default, z_default);
  set_site(1, x_default, y_default, z_default);
  set_site(2, x_default, y_default, z_default);
  set_site(3, x_default, y_default, z_default);
  wait_all_reach();
  Serial.println("b_init end");
}

/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (current_foot_pos[LSI_RL][YAxis] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (current_foot_pos[LSI_FL][YAxis] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (current_foot_pos[LSI_FL][YAxis] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //      leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (current_foot_pos[LSI_RL][YAxis] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void body_left(int i)
{
  set_site(0, current_foot_pos[0][0] + i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(1, current_foot_pos[1][0] + i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(2, current_foot_pos[2][0] - i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(3, current_foot_pos[3][0] - i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  wait_all_reach();
}

void body_right(int i)
{
  set_site(0, current_foot_pos[0][0] - i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(1, current_foot_pos[1][0] - i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(2, current_foot_pos[2][0] + i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  set_site(3, current_foot_pos[3][0] + i, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS);
  wait_all_reach();
}

void hand_wave(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (current_foot_pos[LSI_RL][YAxis] == y_start)
  {
    body_right(15);
    x_tmp = current_foot_pos[2][0];
    y_tmp = current_foot_pos[2][1];
    z_tmp = current_foot_pos[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = current_foot_pos[0][0];
    y_tmp = current_foot_pos[0][1];
    z_tmp = current_foot_pos[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void hand_shake(int i)
{
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (current_foot_pos[LSI_RL][YAxis] == y_start)
  {
    body_right(15);
    x_tmp = current_foot_pos[2][0];
    y_tmp = current_foot_pos[2][1];
    z_tmp = current_foot_pos[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15);
    x_tmp = current_foot_pos[0][0];
    y_tmp = current_foot_pos[0][1];
    z_tmp = current_foot_pos[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i)
{
  set_site(0, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[0][2] - i);
  set_site(1, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[1][2] + i);
  set_site(2, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[2][2] - i);
  set_site(3, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[3][2] + i);
  wait_all_reach();
}

void head_down(int i)
{
  set_site(0, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[0][2] + i);
  set_site(1, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[1][2] - i);
  set_site(2, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[2][2] + i);
  set_site(3, KEEP_CURRENT_FOOT_POS, KEEP_CURRENT_FOOT_POS, current_foot_pos[3][2] - i);
  wait_all_reach();
}

void body_dance(int i)
{
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP_CURRENT_FOOT_POS);
  set_site(1, x_default, y_default, KEEP_CURRENT_FOOT_POS);
  set_site(2, x_default, y_default, KEEP_CURRENT_FOOT_POS);
  set_site(3, x_default, y_default, KEEP_CURRENT_FOOT_POS);
  wait_all_reach();
  stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP_CURRENT_FOOT_POS, y_default - 20, KEEP_CURRENT_FOOT_POS);
    set_site(1, KEEP_CURRENT_FOOT_POS, y_default + 20, KEEP_CURRENT_FOOT_POS);
    set_site(2, KEEP_CURRENT_FOOT_POS, y_default - 20, KEEP_CURRENT_FOOT_POS);
    set_site(3, KEEP_CURRENT_FOOT_POS, y_default + 20, KEEP_CURRENT_FOOT_POS);
    wait_all_reach();
    set_site(0, KEEP_CURRENT_FOOT_POS, y_default + 20, KEEP_CURRENT_FOOT_POS);
    set_site(1, KEEP_CURRENT_FOOT_POS, y_default - 20, KEEP_CURRENT_FOOT_POS);
    set_site(2, KEEP_CURRENT_FOOT_POS, y_default + 20, KEEP_CURRENT_FOOT_POS);
    set_site(3, KEEP_CURRENT_FOOT_POS, y_default - 20, KEEP_CURRENT_FOOT_POS);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
  b_init();
}

/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - foot_move_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void)
{
  sei();
  const unsigned long prev_servo_update_time_ms = last_servo_update_time;
  last_servo_update_time = millis();
  const unsigned long time_delta_ms = last_servo_update_time - prev_servo_update_time_ms;

  for (int i = 0; i < NUM_LEGS; i++)
  {
    bool changed = false;
    for (int j = 0; j < NUM_AXIS; j++)
    {
      float &now = current_foot_pos[i][j];
      const float expect = target_foot_pos[i][j];

      if (now != expect)
      {
        changed = true;
        const float delta = foot_move_speed[i][j] * servo_update_rate_ms;//time_delta_ms;
        if (fabs(expect - now) >= fabs(delta))
        {
          now += delta;
        }
        else
        {
          now = expect;
        }
      }
    }

    //if (changed)
    {
      float elbowDeg, footDeg, shoulderDeg;
      cartesian_to_polar(elbowDeg, footDeg, shoulderDeg, current_foot_pos[i][XAxis], current_foot_pos[i][YAxis], current_foot_pos[i][ZAxis]);
      polar_to_servo(i, elbowDeg, footDeg, shoulderDeg);
    }
  }
}

bool is_valid_pos(float val)
{
  return fabs(val) <= MAX_FOOT_POS;
}

/*
  - set one of end points' expect site
  - this function will set foot_move_speed[NUM_LEGS][NUM_JOINTS_PER_LEG] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (is_valid_pos(x))
  {
    length_x = x - current_foot_pos[leg][XAxis];
    target_foot_pos[leg][XAxis] = x;
  }
  if (is_valid_pos(y))
  {
    length_y = y - current_foot_pos[leg][YAxis];
    target_foot_pos[leg][YAxis] = y;
  }
  if (is_valid_pos(z))
  {
    length_z = z - current_foot_pos[leg][ZAxis];
    target_foot_pos[leg][ZAxis] = z;
  }

  float length = sqrt(length_x * length_x + length_y * length_y + length_z * length_z);
  if (length <= 0.0f)
  {
    length = 0.1f;
  }

  foot_move_speed[leg][XAxis] = length_x / length * move_speed * speed_multiple;
  foot_move_speed[leg][YAxis] = length_y / length * move_speed * speed_multiple;
  foot_move_speed[leg][ZAxis] = length_z / length * move_speed * speed_multiple;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_reach(int leg)
{
  while (true)
  {
    int waitingOnAxis = -1;
    for (int axis = 0; axis < NUM_AXIS; ++axis)
    {
      if (current_foot_pos[leg][axis] != target_foot_pos[leg][axis])
      {
        waitingOnAxis = axis;
        break;
      }
    }

    if (waitingOnAxis < 0)
    {
#ifdef DEBUG      
      Serial.printf("wait_reach finished leg %d\r\n", leg);
#endif
      break;
    }
    else
    {
      const unsigned long timeSinceLastServoUpdate = millis() - last_servo_update_time;
      const unsigned long timeUntilNextServoUpdate = (servo_update_rate_ms >= timeSinceLastServoUpdate) ? servo_update_rate_ms - timeSinceLastServoUpdate : 0UL;
      const unsigned long remainingMovementTimeMS = ceil(fabs(target_foot_pos[leg][waitingOnAxis] - current_foot_pos[leg][waitingOnAxis]) / fabs(foot_move_speed[leg][waitingOnAxis]));
      
      unsigned long waitTimeMS = timeUntilNextServoUpdate;
      if(remainingMovementTimeMS > timeUntilNextServoUpdate)
      {
        waitTimeMS = remainingMovementTimeMS;
        waitTimeMS += servo_update_rate_ms - ((remainingMovementTimeMS - timeUntilNextServoUpdate) % servo_update_rate_ms);
      }

#ifdef DEBUG      
      Serial.print("wait_reach waiting on leg ");
      Serial.print(leg);  
      Serial.print(" axis ");
      Serial.print(waitingOnAxis);
      Serial.print(" for ");
      Serial.print(waitTimeMS);
      Serial.println(" ms");
#endif      
      delay(max(1UL, waitTimeMS));
    }
  }
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void wait_all_reach(void)
{
  for (int i = 0; i < NUM_LEGS; i++)
  {
    wait_reach(i);
  }    
}

/*
  - trans site from cartesian to polar
  - mathematical model 2/2
   ---------------------------------------------------------------------------*/
void cartesian_to_polar(float &elbowDeg, float &footDeg, float &shoulderDeg, float x, float y, float z)
{
  //calculate w-z angles
  const float w = (x >= 0 ? 1 : -1) * sqrt(x * x + y * y);
  const float v = w - length_c;
  const float zSquared = z * z;
  const float vSquared = v * v;
  const float lengthASquared = length_a * length_a;
  const float lengthBSquared = length_b * length_b;
  
  elbowDeg = atan2(z, v) + acos((lengthASquared - lengthBSquared + vSquared + zSquared) * 0.5f / length_a / sqrt(vSquared + zSquared));
  footDeg = acos((lengthASquared + lengthBSquared - vSquared - zSquared) * 0.5f / length_a / length_b);
  //calculate x-y-z angles
  shoulderDeg = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  
  //convert radians to degrees
  elbowDeg = elbowDeg / pi * 180;
  footDeg = footDeg / pi * 180;
  shoulderDeg = shoulderDeg / pi * 180;
}

void polar_to_servo(int leg, float elbowDeg, float footDeg, float shoulderDeg)
{
  switch (leg)
  {
  case LSI_FR: //Front Right
  {
    elbowDeg = 90 - elbowDeg + joint_offset_scale[leg][JSI_Elbow] * joint_offsets_deg[leg][JSI_Elbow];//elbow (- is up)
    footDeg = 0 + footDeg + joint_offset_scale[leg][JSI_Foot] * joint_offsets_deg[leg][JSI_Foot];//foot (- is up)
    shoulderDeg = 90 - shoulderDeg + joint_offset_scale[leg][JSI_Shoulder] * joint_offsets_deg[leg][JSI_Shoulder];//shoulder (- is left)
    break;
  }
  case LSI_RR: //Rear Right
  {
    elbowDeg = 90 + elbowDeg + joint_offset_scale[leg][JSI_Elbow] * joint_offsets_deg[leg][JSI_Elbow];//elbow (+ is up)
    footDeg = 180 - footDeg + joint_offset_scale[leg][JSI_Foot] * joint_offsets_deg[leg][JSI_Foot];//foot (+ is up)
    shoulderDeg = 90 + shoulderDeg + joint_offset_scale[leg][JSI_Shoulder] * joint_offsets_deg[leg][JSI_Shoulder];//shoulder (+ is left)
    break;
  }
  case LSI_FL: //Front Left
  {
    elbowDeg = 90 + elbowDeg + joint_offset_scale[leg][JSI_Elbow] * joint_offsets_deg[leg][JSI_Elbow];//elbow (+ is up)
    footDeg = 180 - footDeg + joint_offset_scale[leg][JSI_Foot] * joint_offsets_deg[leg][JSI_Foot];//foot (+ is up)
    shoulderDeg = 90 + shoulderDeg + joint_offset_scale[leg][JSI_Shoulder] * joint_offsets_deg[leg][JSI_Shoulder];//shoulder (+ is left)
    break;
  }
  case LSI_RL: // Rear Left
  {
    elbowDeg = 90 - elbowDeg + joint_offset_scale[leg][JSI_Elbow] * joint_offsets_deg[leg][JSI_Elbow];//elbow (- is up)
    footDeg = 0 + footDeg + joint_offset_scale[leg][JSI_Foot] * joint_offsets_deg[leg][JSI_Foot];//foot; (- is up)
    shoulderDeg = 90 - shoulderDeg + joint_offset_scale[leg][JSI_Shoulder] * joint_offsets_deg[leg][JSI_Shoulder];//shoulder (- is left)
    break;
  }
  }

  if(servo_move_enabled[leg][JSI_Elbow])
  {
    pwm.setPWM(servo_pin[leg][JSI_Elbow], 0, max(SERVO_PWM_MIN, min(SERVO_PWM_MAX, elbowDeg / 180.0f * SERVO_PWM_RANGE + SERVO_PWM_MIN)));
  }

  if(servo_move_enabled[leg][JSI_Foot])
  {
    pwm.setPWM(servo_pin[leg][JSI_Foot], 0, max(SERVO_PWM_MIN, min(SERVO_PWM_MAX, footDeg / 180.0f * SERVO_PWM_RANGE + SERVO_PWM_MIN)));
  }
  
  if(servo_move_enabled[leg][JSI_Shoulder])
  {
    pwm.setPWM(servo_pin[leg][JSI_Shoulder], 0, max(SERVO_PWM_MIN, min(SERVO_PWM_MAX, shoulderDeg / 180.0f * SERVO_PWM_RANGE + SERVO_PWM_MIN)));
  }  
}














/* Set these to your desired credentials. */
const char *ssid = "SpiderRobo";
const char *password = "12345678";

const int led = 13;
MDNSResponder mdns;
ESP8266WiFiMulti WiFiMulti;
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

//=================================================================================

static const PROGMEM char INDEX_HTML[] = "";/* = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ESP8266 Spider Robot</title>
<style>
"body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }"
#JD {
  text-align: center;
}
#JD {
  text-align: center;
  font-family: "Lucida Sans Unicode", "Lucida Grande", sans-serif;
  font-size: 24px;
}
.foot {
  text-align: center;
  font-family: "Comic Sans MS", cursive;
  font-size: 9px;
  color: #F00;
}
.button {
    border: none;
    color: white;
    padding: 20px;
    text-align: center;
    text-decoration: none;
    display: inline-block;
    font-size: 16px;
    margin: 4px 2px;
    cursor: pointer;
    border-radius: 12px;
  width: 100%;
}
.red {background-color: #F00;}
.green {background-color: #090;}
.yellow {background-color:#F90;}
.blue {background-color:#03C;}
</style>
<script>
var websock;
function start() {
  websock = new WebSocket('ws://' + window.location.hostname + ':81/');
  websock.onopen = function(evt) { console.log('websock open'); };
  websock.onclose = function(evt) { console.log('websock close'); };
  websock.onerror = function(evt) { console.log(evt); };
  websock.onmessage = function(evt) {
    console.log(evt);
    var e = document.getElementById('ledstatus');
    if (evt.data === 'ledon') {
      e.style.color = 'red';
    }
    else if (evt.data === 'ledoff') {
      e.style.color = 'black';
    }
    else {
      console.log('unknown event');
    }
  };
}
function buttonclick(e) {
  websock.send(e.id);
}
</script>
</head>
<body onload="javascript:start();">
&nbsp;
<table width="100%" border="1">
  <tr>
    <td bgcolor="#FFFF33" id="JD">Quadruped Controller</td>
  </tr>
</table>
<table width="100" height="249" border="0" align="center">
  <tr>
    <td>&nbsp;</td>
    <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 1 1"  type="button" onclick="buttonclick(this);" class="button green">Forward</button> 
      </label>
    </form></td>
    <td>&nbsp;</td>
  </tr>
  <tr>
    <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 3 1"  type="button" onclick="buttonclick(this);" class="button green">Turn_Left</button> 
      </label>
    </form></td>
    <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 0 1"  type="button" onclick="buttonclick(this);" class="button red">Stop_all</button> 
      </label>
    </form></td>
    <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 4 1"  type="button" onclick="buttonclick(this);" class="button green">Turn_Right</button> 
      </label>
    </form></td>
  </tr>
  <tr>
    <td>&nbsp;</td>
    <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 2 1"  type="button" onclick="buttonclick(this);" class="button green">Backward</button> 
      </label>
    </form></td>
    <td>&nbsp;</td>
  </tr>
  <tr>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 5 3"  type="button" onclick="buttonclick(this);" class="button yellow">Shake </button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 8 5"  type="button" onclick="buttonclick(this);" class="button blue">Head_up</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 6 3"  type="button" onclick="buttonclick(this);" class="button yellow">Wave</button> 
      </label>
    </form></td>
  </tr>
  <tr>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 16"  type="button" onclick="buttonclick(this);" class="button blue">Twist_Left</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 9 5"  type="button" onclick="buttonclick(this);" class="button blue">Head_down</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 17"  type="button" onclick="buttonclick(this);" class="button blue">Twist_Right</button> 
      </label>
    </form></td>
  </tr>
  <tr>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 11 5"  type="button" onclick="buttonclick(this);" class="button blue">Body_left</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 13"  type="button" onclick="buttonclick(this);" class="button blue">Body_higher</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 10 5"  type="button" onclick="buttonclick(this);" class="button blue">Body_right</button>
      </label>
    </form></td>
  </tr>

  <tr>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
         <button id="w 12"  type="button" onclick="buttonclick(this);" class="button yellow">Service</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 14"  type="button" onclick="buttonclick(this);" class="button blue">Body_lower</button> 
      </label>
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 15"  type="button" onclick="buttonclick(this);" class="button yellow">Reset_Pose</button> 
      </label>
    </form></td>
  </tr>
  
    <tr>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 0 0"  type="button" onclick="buttonclick(this);" class="button yellow">Sit</button> 
      </label>
    </form></td>
      <td align="center" valign="middle"><form name="form1" method="post" action="">
&nbsp;
    </form></td>
        <td align="center" valign="middle"><form name="form1" method="post" action="">
      <label>
        <button id="w 7 1 "  type="button" onclick="buttonclick(this);" class="button yellow">Dance</button> 
      </label>
    </form></td>
  </tr>
  
</table>
<p class="foot">this application requires Mwilmar Quadruped platform.</p>
</body>
</html>
)rawliteral";*/

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);
  switch(type) 
  {
    case WStype_DISCONNECTED:
      {
        Serial.printf("[%u] Disconnected!\r\n", num);
        break;
      }
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        break;
      }      
    case WStype_TEXT:
      {
        Serial.printf("%s\r\n", payload);
        
        const char* command = strtok(reinterpret_cast<char*>(payload), " ");
        if(*command == 'w')
        {
          int action_mode = atoi(strtok(nullptr, " "));
          int n_step = atoi(strtok(nullptr, " "));
          do_action(action_mode, n_step);
        }

        digitalWrite(LED_BUILTIN, LOW);
        // send data to all connected clients
        webSocket.broadcastTXT(payload, length);
        break;
      }
    case WStype_BIN:
      {
        hexdump(payload, length);
        // echo data back to browser
        webSocket.sendBIN(num, payload, length);
        break;
      }
  }
}

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}





void setup()
{
  //start serial for debug

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); 
  for(uint8_t t = 4; t > 0; t--) {
    Serial.flush();
    delay(1000);
  }
  delay(1000);
  Serial.begin(115200);
  Serial.println("Robot starts initialization");

  pinMode(LED_BUILTIN, OUTPUT);
  pwm.begin();  
  pwm.setPWMFreq(SERVO_FREQUENCY);
  
  SCmd.addCommand("w", action_cmd);
  SCmd.addCommand("s", servo_cmd);
  SCmd.addCommand("p", pwm_cmd);
  SCmd.addCommand("o", offset_cmd);
  SCmd.setDefaultHandler(unrecognized);

  for(int i = 0; i < NUM_LEGS; ++i)
  {
    const int direction = (i == LSI_FR || i == LSI_RL) ? -1 : 1;
    for(int j = 0; j < NUM_JOINTS_PER_LEG; ++j)
    {
      servo_move_enabled[i][j] = true;
      joint_offsets_deg[i][j] = 0;
      joint_offset_scale[i][j] = direction;
    }
  }

  //initialize default parameter
  set_site(LSI_FR, x_default - x_offset, y_start + y_step, z_boot);
  set_site(LSI_RR, x_default - x_offset, y_start + y_step, z_boot);
  set_site(LSI_FL, x_default + x_offset, y_start, z_boot);
  set_site(LSI_RL, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < NUM_LEGS; i++)
  {
    for (int j = 0; j < NUM_AXIS; j++)
    {
      current_foot_pos[i][j] = target_foot_pos[i][j];
    }
  }

  //start servo service
  last_servo_update_time = millis();
  ticker.attach_ms(servo_update_rate_ms, servo_service);
  Serial.println("Servo service started");

  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  if (mdns.begin("espWebSock", WiFi.localIP())) 
  {
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  sit();
  b_init();
}

void loop()
{
  //-----------led blink status
  if (ledPulse <= 500)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (ledPulse > 1000)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (ledPulse >= 1500)
  {
    ledPulse = 0;
  }
  ledPulse++;
  //-------------------

  SCmd.readSerial();
  if (lastComm == "FWD")
  {
    step_forward(1);
  }
  if (lastComm == "BWD")
  {
    step_back(1);
  }
  if (lastComm == "LFT")
  {
    turn_left(1);
  }
  if (lastComm == "RGT")
  {
    turn_right(1);
  }

  if (lastComm != "")
  {
    Serial.println(lastComm);
  }

  digitalWrite(LED_BUILTIN, HIGH); 
  webSocket.loop();
  server.handleClient();
}

