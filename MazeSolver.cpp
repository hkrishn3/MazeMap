/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/********************************************************************************************************
 * 
 *  
 * MAZE SOLVER
 * ------------
 * MAPS AND SOLVES A MAZE USING A LEFT WALL FOLLOWING AND FLOOD FILL ALGORITHMS
 * FINDS AN OPTIMAL SOLUTION TO RETURN BACK TO THE STARTING POINT
 * SOLVES THE MAZE USING THE OPTIMAL SOLUTION 
 * 
 ********************************************************************************************************/
#include <Arduino_FreeRTOS.h>
#include <MeAuriga.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio



#define NODEBUG                  0
#define DEBUG_ULTRASONIC_SENSORS 1
#define DEBUG_TURNS              2
#define DEBUG_DIRECTION_QUEUE    3
#define DEBUG_ENCODER_DIFF       4
#define DEBUG_FORWARD_COUNTER    5

#define DEBUG             NODEBUG

//direction definitions
#define NORTH              0
#define WEST               1
#define SOUTH              2
#define EAST               3

//clockwise/counterclockwise
#define CW                 0   
#define CCW                1

//sensors
#define FRONT_US           0 //front ultrasonic sensor
#define LEFT_US            1 //left ultrasonic sensor
#define COMPASS            2 //compass sensor
#define RIGHT_US           3 //right ultrasonic sensor

//compass values for the different directions
#define EAST_VALUE         106.5
#define NORTH_VALUE          3.8
#define SOUTH_VALUE        187.6
#define WEST_VALUE         270.4

#define RGBLED_PORT         44  //port addresses for the RGB LED ring and buzzer
#define BUZZER_PORT         45  
#define BUF_SIZE           256  //maze buffer size

//maze solution array size definition
#define SEARCH_BUF_SIZE   99
#define SEARCH_SOL_SIZE   99

//state machine states for the different tasks
#define FOREVER_BLINK         0   
#define SOLVE_MAZE            1
#define ASSESS_SENSORS        2
#define LEFT_WALL_FIND        3
#define DELAY_HALF_SECOND     4
#define DELAY_ONE_SECOND      5
#define ROTATE_ROBOT          6
#define DELAY_STOP            7
#define GO_SIGNAL             8
#define POSITION_ROBOT        9
#define READ_ARRAY           10
#define R_ROTATE_ROBOT       11
#define F_ROTATE_ROBOT       12
#define SWITCH_FORWARD       13

#define ONE_BRICK_LENGTH    560 //length of one brick in encoder ticks
#define SENSOR_DELAY        5   //sampling delay for ultrasonic sensor values

//ultrasonic sensor distances for the different sensors in centimeters
#define FRONT_US_DISTANCE  20.0
#define FOPEN_US_DISTANCE  40.0
#define LEFT_US_DISTANCE   40.0
#define RIGHT_US_DISTANCE  40.0

//speed for the fast run
#define RUN_SPEED_FAST     100.0

//maze definitions
#define AREA 121 //total array size
#define COLS  11 //number of columns
#define ROWS  11 //number of rows

//maze mapping and solving variables
int maze[AREA];
int current_x;
int current_y;
int current_distance_count = 0;

int search_run_buf[SEARCH_BUF_SIZE];  //circular tone buffer
int search_run_head = 0; //head of the buffer
int search_run_tail = 0; //tail
int search_run = 0;

int maze_solution_array[SEARCH_SOL_SIZE];
int maze_solution_array_reverse[SEARCH_SOL_SIZE];
int maze_solution_array_forward[SEARCH_SOL_SIZE];

//software usb serial via bluetooth for debugging and printing
SoftwareSerial mb_bluetooth(0, 1, false); //RX, TX

//define the two encoders, LED and the buzzer
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
static MeRGBLed led;
static MeBuzzer buzzer;

//sensors used in solving the maze
MeCompass Compass(PORT_6);
MeLineFollower lineFinder(PORT_7);
MeUltrasonicSensor rside_sensor(PORT_8);
MeUltrasonicSensor front_sensor(PORT_9);
MeUltrasonicSensor lside_sensor(PORT_10);

//FreeRTOS tasks
void TaskSearchRun(void *pvParameters);
void TaskBlink( void *pvParameters );
void TaskPrintMaze(void *pvParameters);
void TaskReverseSolve(void *pvParameters);
void TaskForwardSolve(void *pvParameters);

//task handles
static TaskHandle_t xTaskSearchRun = NULL, xTaskBlink = NULL, xTaskPrintMaze = NULL, xTaskReverseSolve = NULL, xTaskForwardSolve = NULL;

//variables for mapping the maze
volatile unsigned long forward_time_counter = 0, forward_timestamp = 0; //keeps track of the the time robot is traveling in a straight line
volatile unsigned long last_milli_sec = 0;
volatile unsigned long current_encoder_position = 0, previous_encoder_position = 0; //encoder position

//solve the maze from the exit point back to the start point after the search run
void TaskReverseSolve(void *pvParameters) {
  (void) pvParameters;
  uint8_t current_state = READ_ARRAY, previous_direction = SOUTH, current_direction, target_direction;
  boolean turn_direction, state_flag = 0, task_solve_forward = 0;
  int i, reverse_array_index = 0, previous_ts_count = 0, ts_count = 0;
  unsigned long ts_timestamp = 0, time_elapsed;
  double front_distance, lside_distance, rside_distance;
  const double north_value = NORTH_VALUE, east_value = EAST_VALUE, south_value = SOUTH_VALUE, west_value = WEST_VALUE;
  double current_angle = south_value, rotation_angle, target_angle;
  
  /* 
   * Reads each direction entry in the solution array and moves forward in that direction until
   * the sensors detect other open pathways in the maze. the next direction is chosen based on 
   * the sensor reading
   */
  
  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      //block until task notification is received
    for(;;) {
      switch(current_state) {
        case READ_ARRAY:
          if(!state_flag) {
            previous_ts_count = ts_count;
            state_flag = 1;
          }
          time_elapsed =  ts_count - previous_ts_count;
          if(time_elapsed > 10) {
            current_direction = maze_solution_array_reverse[reverse_array_index];
            for(i = reverse_array_index; i < SEARCH_BUF_SIZE; i++) {
              if(maze_solution_array_reverse[i] != current_direction) {
                if(maze_solution_array_reverse[i] != 99)
                  target_direction = maze_solution_array_reverse[i];
                else
                  target_direction = current_direction;
                reverse_array_index = i;
                break;
              }
            }
            current_state = current_direction;
            state_flag = 0;
          }
          break;
        
        //states to position the robot for solving the maze from start to finish  
        case SWITCH_FORWARD:
          if(analogRead(A2) < 10) {
            xTaskNotifyGive(xTaskSearchRun);          //restore context to the main task
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  //block this task
          }
          break;
        case POSITION_ROBOT:
          if(!state_flag) {
            previous_ts_count = ts_count;
            state_flag = 1;
          }
          time_elapsed =  ts_count - previous_ts_count;
          if(time_elapsed > 5) {
            move_forward(EAST_VALUE, EAST, 50.0);
            if(time_elapsed > 20) {
              stop_motors();
              state_flag = 0;
              current_state = SWITCH_FORWARD;
            }
          }
          break;
        
        //rotates the robot in the desired direction
        case R_ROTATE_ROBOT:
          if(!state_flag) {
            previous_ts_count = ts_count;
            state_flag = 1;
          }
          time_elapsed =  ts_count - previous_ts_count;
          if(time_elapsed > 10) {
            rotate(turn_direction);
            rotation_angle = read_sensor(COMPASS);
            if(abs(rotation_angle - target_angle) < 0.5) {
              stop_motors();
              previous_direction = current_direction;
              current_direction = target_direction;
              current_angle = target_angle;
              state_flag = 0;
              if(task_solve_forward == 1)
                current_state = POSITION_ROBOT;
              else
                current_state = READ_ARRAY;
            }
          }
          break;
        
        //move in the SOUTH direction
        case SOUTH:
          move_forward(south_value, SOUTH, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          rside_distance = read_sensor(RIGHT_US);
          if(rside_distance > RIGHT_US_DISTANCE  && front_distance < FRONT_US_DISTANCE) {
            stop_motors();
            if(target_direction == EAST) {
              turn_direction = CCW;
              target_angle = east_value;
            } else if(target_direction == WEST) {
              turn_direction = CW;
              target_angle = west_value;
            }
            current_state = R_ROTATE_ROBOT;
          }
          break;
        
        //move in the WEST direction
        case WEST:
          move_forward(west_value, WEST, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          rside_distance = read_sensor(RIGHT_US);
          if(rside_distance > RIGHT_US_DISTANCE  && front_distance < FRONT_US_DISTANCE) {
            stop_motors();
            if(target_direction == NORTH) {
              turn_direction = CW;
              target_angle = north_value;
            } else if(target_direction == SOUTH) {
              turn_direction = CCW;
              target_angle = south_value;
            }
            current_state = R_ROTATE_ROBOT;
          }
          break;
        //move in the NORTH direction
        case NORTH:
          move_forward(north_value, NORTH, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          rside_distance = read_sensor(RIGHT_US);
          lside_distance = read_sensor(LEFT_US);
          if((front_distance < FRONT_US_DISTANCE) || (rside_distance > RIGHT_US_DISTANCE  && front_distance > FOPEN_US_DISTANCE && lside_distance > LEFT_US_DISTANCE)) {
            if((rside_distance > RIGHT_US_DISTANCE  && front_distance > FOPEN_US_DISTANCE && lside_distance > LEFT_US_DISTANCE)) {
              delay(500);
            }
            stop_motors();
            if(target_direction == EAST) {
              turn_direction = CW;
              target_angle = east_value;
            } else if(target_direction == WEST) {
              turn_direction = CCW;
              target_angle = west_value;
            }
            current_state = R_ROTATE_ROBOT;
          }
          break;
        //move in the EAST direction
        case EAST:
          move_forward(east_value, EAST, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          lside_distance = read_sensor(LEFT_US);
          if(lside_distance > LEFT_US_DISTANCE  && front_distance < FRONT_US_DISTANCE) {
            stop_motors();
            if(target_direction == NORTH) {
              turn_direction = CCW;
              target_angle = north_value;
            } else if(target_direction == SOUTH) {
              turn_direction = CW;
              target_angle = south_value;
            }
            current_state = R_ROTATE_ROBOT;
          }
          break;
      }
      
      //check the line sensor if the robot has reached the end
      int sensorState = lineFinder.readSensors();
      if(task_solve_forward == 0) {
        if(sensorState == S1_IN_S2_IN) {
          stop_motors();
          task_solve_forward = 1;
          switch(current_direction) {
            case NORTH:
              target_direction = SOUTH;
              target_angle = south_value;
              break;
            case EAST:
              target_direction = WEST;
              target_angle = west_value;
              break;
            case SOUTH:
              target_direction = NORTH;
              target_angle = north_value;
              break;
            case WEST:
              target_direction = EAST;
              target_angle = east_value;
              break;
          }
          turn_direction = CW;
          current_state = R_ROTATE_ROBOT;
          state_flag = 0;
        }
      }
      Encoder_1.loop();         //refresh the encoder position values
      Encoder_2.loop();
      //increment the timer counter
      if(millis() - ts_timestamp >= 100) {
        ts_count++;
        ts_timestamp = millis();
      }
    }
  }
}

//solve the maze from the start point to the exit point after solving it in the reverse direction
void TaskForwardSolve(void *pvParameters) {
  (void) pvParameters;
  uint8_t current_state = READ_ARRAY, previous_direction = EAST, current_direction, target_direction;
  boolean turn_direction, state_flag = 0, end_solve = 0;
  int i, forward_array_index = 1, previous_ts_count = 0, ts_count = 0;
  unsigned long ts_timestamp = 0, time_elapsed;
  double front_distance, lside_distance, rside_distance;
  const double north_value = NORTH_VALUE, east_value = EAST_VALUE, south_value = SOUTH_VALUE, west_value = WEST_VALUE;
  double current_angle = east_value, rotation_angle, target_angle;

  /* 
   * Reads each direction entry in the solution array and moves forward in that direction until
   * the sensors detect other open pathways in the maze. the next direction is chosen based on 
   * the sensor reading
   */

  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      //block until task notification is received
    for(;;) {
      switch(current_state) {
        case READ_ARRAY:
          if(!state_flag) {
            previous_ts_count = ts_count;
            state_flag = 1;
          }
          time_elapsed =  ts_count - previous_ts_count;
          if(time_elapsed > 10) {
            current_direction = maze_solution_array[forward_array_index];
            for(i = forward_array_index; i < SEARCH_BUF_SIZE; i++) {
              if(maze_solution_array[i] != current_direction) {
                if(maze_solution_array[i] != 99)
                  target_direction = maze_solution_array[i];
                else
                  target_direction = current_direction;
                forward_array_index = i;
                break;
              }
            }
            current_state = current_direction;
            state_flag = 0;
          }
          break;
       //rotates the robot in the desired direction
        case F_ROTATE_ROBOT:
          if(!state_flag) {
            previous_ts_count = ts_count;
            state_flag = 1;
          }
          time_elapsed =  ts_count - previous_ts_count;
          if(time_elapsed > 10) {
            rotate(turn_direction);
            rotation_angle = read_sensor(COMPASS);
            if(abs(rotation_angle - target_angle) < 0.5) {
              stop_motors();
              previous_direction = current_direction;
              current_direction = target_direction;
              current_angle = target_angle;
              state_flag = 0;
              if(end_solve == 1) {
                xTaskNotifyGive(xTaskBlink);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
              } else
                current_state = READ_ARRAY;
            }
          }
          break;
          
        //move in the SOUTH direction         
        case SOUTH:
          move_forward(south_value, SOUTH, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          lside_distance = read_sensor(LEFT_US);
          rside_distance = read_sensor(RIGHT_US);
          if((lside_distance > LEFT_US_DISTANCE || rside_distance > RIGHT_US_DISTANCE)  && front_distance < FRONT_US_DISTANCE) {
            stop_motors();
            if(target_direction == EAST) {
              turn_direction = CCW;
              target_angle = east_value;
            } else if(target_direction == WEST) {
              turn_direction = CW;
              target_angle = west_value;
            }
            current_state = F_ROTATE_ROBOT;
          }
          break;
        
        //move in the WEST direction         
        case WEST:
          move_forward(west_value, WEST, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          lside_distance = read_sensor(LEFT_US);
          if(lside_distance > LEFT_US_DISTANCE  && front_distance < FRONT_US_DISTANCE) {
            stop_motors();
            if(target_direction == NORTH) {
              turn_direction = CW;
              target_angle = north_value;
            } else if(target_direction == SOUTH) {
              turn_direction = CCW;
              target_angle = south_value;
            }
            current_state = F_ROTATE_ROBOT;
          }
          break;
        
        //move in the NORTH direction
        case NORTH:
          move_forward(north_value, NORTH, RUN_SPEED_FAST);
          front_distance = read_sensor(FRONT_US);
          lside_distance = read_sensor(LEFT_US);
          if(front_distance < FRONT_US_DISTANCE && lside_distance > LEFT_US_DISTANCE ) {
            stop_motors();
            if(target_direction == EAST) {
              turn_direction = CW;
              target_angle = east_value;
            } else if(target_direction == WEST) {
              turn_direction = CCW;
              target_angle = west_value;
            }
            current_state = F_ROTATE_ROBOT;
          }
          break;
        
        //move in the EAST direction          
        case EAST:
          move_forward(east_value, EAST, RUN_SPEED_FAST);
          mb_bluetooth.println("EAST");
          front_distance = read_sensor(FRONT_US);
          rside_distance = read_sensor(RIGHT_US);
          lside_distance = read_sensor(LEFT_US);
          if((lside_distance > LEFT_US_DISTANCE  && front_distance < FRONT_US_DISTANCE) || (rside_distance > RIGHT_US_DISTANCE  && front_distance > FOPEN_US_DISTANCE && lside_distance > LEFT_US_DISTANCE)) {
            if((rside_distance > RIGHT_US_DISTANCE  && front_distance > FOPEN_US_DISTANCE && lside_distance > LEFT_US_DISTANCE)) {
              delay(500); // wait for one second
            }
            stop_motors();
            if(target_direction == NORTH) {
              turn_direction = CCW;
              target_angle = north_value;
            } else if(target_direction == SOUTH) {
              turn_direction = CW;
              target_angle = south_value;
            }
            current_state = F_ROTATE_ROBOT;
          }
          break;
      }

      //check the line sensor if the robot has reached the end
      int sensorState = lineFinder.readSensors();
      if(end_solve == 0) {
        if(sensorState == S1_IN_S2_IN) {
          stop_motors();
          end_solve = 1;
          switch(current_direction) {
            case NORTH:
              target_direction = SOUTH;
              target_angle = south_value;
              break;
            case EAST:
              target_direction = WEST;
              target_angle = west_value;
              break;
            case SOUTH:
              target_direction = NORTH;
              target_angle = north_value;
              break;
            case WEST:
              target_direction = EAST;
              target_angle = east_value;
              break;
          }
          turn_direction = CW;
          current_state = F_ROTATE_ROBOT;
          state_flag = 0;
        }
      }
      Encoder_1.loop();         //refresh the encoder position values
      Encoder_2.loop();

      //increment the timer counter
      if(millis() - ts_timestamp >= 100) {
        ts_count++;
        ts_timestamp = millis();
      }
    }
  }
}

/*  
 *  Creates the map of the maze using the Flood fill algorithm and calculates the optimal
 *  solution. The solution is then printed out via the Bluetooth software serial interface
 *  
 */
void TaskPrintMaze(void *pvParameters) { // This is a task.
  (void) pvParameters;
  int min_array[4];
  int total_brick_count = 0, direction_count = 0;
  int i, last_cell, home_value, home_x, home_y, value_north, value_east, value_south, value_west, next_min, num_steps = 0;
  int maze_solution_size, previous_direction = EAST;
  
  for (;;) { // A Task shall never return or exit.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      //block until task notification is received
    initialize_maze(-1);
   
   //read the solution array and insert it into the maze array using 1D->2D transformation
    for(i = 0; i < SEARCH_BUF_SIZE; i++) {
      if(search_run_buf[i] != 99) {
        set_cell(search_run_buf[i]);
      }
    }
    //calcualtes the total number of steps taken
    total_brick_count = 0;
    for(i = 0; i < AREA; i++) {
      if(maze[i] > 0) total_brick_count++;
    }
    maze_solution_size = -99;
    
    //find the exit point of the maze by searching for the cell with the largest visit number
    for(i = 0; i < 99; i++) {
      if (maze[i] > maze_solution_size) {
        maze_solution_size = maze[i];
        last_cell = i;
      }
    }
    int solution_index = maze_solution_size;
    int reverse_index = 0;
    float home_xf = last_cell % COLS;
    float home_yf = last_cell / COLS;
    
    //convert the cell index into x and y coordinates
    home_x = (int)home_xf;
    home_y = (int)home_yf;

    //find the cells adjacent to the exit
    value_north = get_north(home_x, home_y);
    value_east = get_east(home_x, home_y);
    value_south = get_south(home_x, home_y);
    value_west = get_west(home_x, home_y);

    //initialize the solution arrays for solving the maze in reverse and forward direction
    clear_buffer(maze_solution_array, SEARCH_SOL_SIZE, 99);
    clear_buffer(maze_solution_array_reverse, SEARCH_SOL_SIZE, 99);
    
    //build the solution array by traversing the maze array and finding the lowest visit number next
    home_value = maze[(home_x) + (home_y*COLS)];
    int max_value = home_value;
    for(int j=0; j<max_value; j++) {
      value_north = get_north(home_x, home_y);
      if(value_north < 0) value_north = 99;
      value_east = get_east(home_x, home_y);
      if(value_east < 0) value_east = 99;
      value_south = get_south(home_x, home_y);
      if(value_south < 0) value_south = 99;
      value_west = get_west(home_x, home_y);
      if(value_west < 0) value_west = 99;
      if((value_north >= 0) && (value_north < value_east && value_north < value_west && value_north < value_south)) {
        next_min = value_north;
        home_x = home_x - 1;
        maze_solution_array[solution_index] = SOUTH;
        maze_solution_array_reverse[reverse_index] = NORTH;
      } else if((value_east >= 0) && (value_east < value_north && value_east < value_west && value_east < value_south)) {
        next_min = value_east;
        home_y = home_y - 1;
        maze_solution_array[solution_index] = WEST;
        maze_solution_array_reverse[reverse_index] = EAST;
      } else if((value_south >= 0) && (value_south < value_east && value_south < value_west && value_south < value_north)) {
        next_min = value_south;
        home_x = home_x + 1;
        maze_solution_array[solution_index] = NORTH;
        maze_solution_array_reverse[reverse_index] = SOUTH;
      } else if((value_west >= 0) && (value_west < value_east && value_west < value_north && value_west < value_south)) {
        next_min = value_west;
        home_y = home_y + 1;
        maze_solution_array[solution_index] = EAST;
        maze_solution_array_reverse[reverse_index] = WEST;
      }
      num_steps++;
      solution_index--;
      reverse_index++;
      home_value = maze[(home_x) + (home_y*COLS)];
    }
    
    initialize_maze(-1);
    current_distance_count = 0;
    total_brick_count = 0;
    
    //print the solution array
    for(i = 0; i < num_steps; i++) {
      if((maze_solution_array[i] >= 0) && maze_solution_array[i] < 99) {
        set_cell(maze_solution_array[i]);
        mb_bluetooth.print(maze_solution_array[i]);
        mb_bluetooth.print(" ");
      }
    }
    mb_bluetooth.println("\n");
    num_steps = 0;
    
    xTaskNotifyGive(xTaskSearchRun);          //restore context to the main task
  }
}
/* 
 *  Task that controls the LED array and the buzzer
 *  Task is called by TaskSearchRun each time it finds a wall
 *  
 */
void TaskBlink(void *pvParameters) { // This is a task.
  (void) pvParameters;
  led.setpin(RGBLED_PORT);
  led.setColor(0,0,0,0);
  led.show();
  for (;;) { // A Task shall never return or exit.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      //block until task notification is received
    led.setColor(20, 16, 16); // turn the LEDs on
    led.show();
    buzzer.tone(800,150);  //sound buzzer
    vTaskDelay(300 / portTICK_PERIOD_MS ); // wait for one second
    buzzer.tone(400,150);  //sound buzzer
    led.setColor(0, 0, 0); // turn the LEDs off
    led.show();
    vTaskDelay(300 / portTICK_PERIOD_MS ); // wait for one second
    xTaskNotifyGive(xTaskSearchRun);          //restore context to the main task
  }
}

/*
 * Primary task that solves the maze using the left wall following algorithm and builds the maze array
 */
void TaskSearchRun(void *pvParameters) {
  (void) pvParameters;
  //variables to control the state during execution
  int encoder2_previous_position = 0;
  unsigned long time_elapsed;
  unsigned long ts_timestamp = 0;
  long ts_count = 0;
  long previous_ts_count = 0;
  int state_flag = 0;
  int left_turn = 0;
  double front_distance;
  double lside_distance;
  const double north_value = NORTH_VALUE, east_value = EAST_VALUE, south_value = SOUTH_VALUE, west_value = WEST_VALUE;
  int orientation_error;
  int target_direction, turn_direction, going_forward = 0, task_solve_reverse = 0;
  double target_angle;
  int current_direction = EAST;
  int prior_direction = EAST;
  double current_angle = east_value;
  double rotation_angle;
  uint8_t current_state = GO_SIGNAL;
  uint8_t last_state = 0;
  
  for (;;) {
    switch(current_state) {
      //signals the start of the search run when the light sensor is activated
      case GO_SIGNAL:
        if(analogRead(A2) < 10) {
          current_state = ASSESS_SENSORS;
        }
        break;
      //state at the end of all runs
      case FOREVER_BLINK:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 10) {
          xTaskNotifyGive(xTaskBlink);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          state_flag = 0;
        }
        break;
      //solves maze using the optimal solution in both directions  
      case SOLVE_MAZE:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 10) {
          search_run = 0;
          xTaskNotifyGive(xTaskPrintMaze);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          xTaskNotifyGive(xTaskBlink);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          xTaskNotifyGive(xTaskReverseSolve);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          xTaskNotifyGive(xTaskBlink);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          xTaskNotifyGive(xTaskForwardSolve);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          xTaskNotifyGive(xTaskBlink);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          vTaskDelay(1000 / portTICK_PERIOD_MS ); // wait for one second
          current_state = FOREVER_BLINK;
          state_flag = 0;
        }
        break;
      //positions the robot  
      case POSITION_ROBOT:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 5) {
          move_forward(current_angle, current_direction, 30);
          if(time_elapsed > 20) {
            stop_motors();
            state_flag = 0;
            current_state = SOLVE_MAZE;
          }
        }
        break;
      //delay half second  
      case DELAY_HALF_SECOND:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 5) {
          state_flag = 0;
          current_state = ASSESS_SENSORS;
          last_state = DELAY_ONE_SECOND;
        }
        break;
      //delay one second  
      case DELAY_ONE_SECOND:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 10) {
          stop_motors();
          state_flag = 0;
          current_state = DELAY_HALF_SECOND;
          last_state = DELAY_ONE_SECOND;
        }
        break;
      //find the left wall after a left turn  
      case LEFT_WALL_FIND:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 10) {
          if(going_forward == 0) {
            going_forward = 1;
            forward_timestamp = millis();
            forward_time_counter = 0;
          }
          move_forward(current_angle, current_direction, 30.0);
          lside_distance = read_sensor(LEFT_US);
          front_distance = read_sensor(FRONT_US);
          if(lside_distance < LEFT_US_DISTANCE || front_distance < FRONT_US_DISTANCE) {
            add_data(current_direction);
            add_data(current_direction);
            forward_time_counter = 0;
#ifdef DEBUG
#if (DEBUG > 2) && (DEBUG < 4)
            mb_bluetooth.print("Curent Forward Direction ");
            mb_bluetooth.println(current_direction);
            mb_bluetooth.print("Curent Forward Direction ");
            mb_bluetooth.println(current_direction);
#endif
#endif
            vTaskDelay(500 /portTICK_PERIOD_MS);
            state_flag = 0;
            current_state = ASSESS_SENSORS;
            last_state = LEFT_WALL_FIND;
          }
        }
        break;
      //turn robot in the desired direction  
      case ROTATE_ROBOT:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
#ifdef DEBUG
#if (DEBUG > 1) && (DEBUG < 3)
          mb_bluetooth.print(current_direction);
          mb_bluetooth.print(" -->");
          mb_bluetooth.println(target_direction);
#endif
#endif
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 10) {
          going_forward = 0;
          rotate(turn_direction);
          rotation_angle = read_sensor(COMPASS);
          if(abs(rotation_angle - target_angle) < 0.5) {
            if(stop_motors() == 1) {
              add_data(current_direction);
              forward_time_counter = 0;
#ifdef DEBUG
#if (DEBUG > 2) && (DEBUG < 4)
              mb_bluetooth.print("Curent Forward Direction ");
              mb_bluetooth.println(current_direction);
#endif
#endif
            }
            previous_encoder_position = Encoder_2.getCurPos();
            prior_direction = current_direction;
            current_direction = target_direction;
            current_angle = target_angle;
            state_flag = 0;
            current_state = DELAY_HALF_SECOND;
            if(left_turn == 1) {
              current_state = LEFT_WALL_FIND;
              left_turn = 0;
            }
            if(task_solve_reverse == 1) {
              current_state = POSITION_ROBOT;
            }
          }
        }
        break;
      //checks the ultrasonic sensors to decide if the robot should move forward or perform a turn  
      case ASSESS_SENSORS:
        if(!state_flag) {
          search_run = 1;
          xTaskNotifyGive(xTaskBlink);
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          state_flag = 1;
        }
        front_distance = read_sensor(FRONT_US);
        lside_distance = read_sensor(LEFT_US);
#ifdef DEBUG
#if (DEBUG > 0) && (DEBUG < 2)
        mb_bluetooth.print("F Distance : ");
        mb_bluetooth.print(read_sensor(FRONT_US));
        mb_bluetooth.print(" cm");
        mb_bluetooth.print(" L Distance : ");
        mb_bluetooth.print(read_sensor(LEFT_US));
        mb_bluetooth.println(" cm");
#endif
#endif
        if((front_distance > 0) && (front_distance < FRONT_US_DISTANCE && lside_distance < LEFT_US_DISTANCE)) {
          switch(current_direction) {
            case NORTH:
              target_direction = EAST;
              target_angle = east_value;
              break;
            case EAST:
              target_direction = SOUTH;
              target_angle = south_value;
              break;
            case SOUTH:
              target_direction = WEST;
              target_angle = west_value;
              break;
            case WEST:
              target_direction = NORTH;
              target_angle = north_value;
              break;
          }
          turn_direction = CW;
          current_state = DELAY_STOP;
          state_flag = 0;
        } else if(lside_distance > 45.0 && (lside_distance > 0 && lside_distance < 200.0)) {
          left_turn = 1;
          vTaskDelay(500 /portTICK_PERIOD_MS);
          switch(current_direction) {
            case NORTH:
              target_direction = WEST;
              target_angle = west_value;
              break;
            case EAST:
              target_direction = NORTH;
              target_angle = north_value;
              break;
            case SOUTH:
              target_direction = EAST;
              target_angle = east_value;
              break;
            case WEST:
              target_direction = SOUTH;
              target_angle = south_value;
              break;
          }
          turn_direction = CCW;
          current_state = ROTATE_ROBOT;
          state_flag = 0;
        } else if((front_distance > 0) && (front_distance >= FRONT_US_DISTANCE && lside_distance < LEFT_US_DISTANCE)) {
          if(going_forward == 0) {
            going_forward = 1;
            forward_timestamp = millis();
            forward_time_counter = 0;
          }
          if(move_forward(current_angle, current_direction, 30) == 1)  {
            add_data(current_direction);
#ifdef DEBUG
#if (DEBUG > 2) && (DEBUG < 4)
            mb_bluetooth.print("Curent Forward Direction ");
            mb_bluetooth.println(current_direction);
#endif
#endif
          }
          current_state = ASSESS_SENSORS;
        }
        break;
      case DELAY_STOP:
        if(!state_flag) {
          previous_ts_count = ts_count;
          state_flag = 1;
        }
        time_elapsed =  ts_count - previous_ts_count;
        if(time_elapsed > 5) {
          state_flag = 0;
          stop_motors();
          current_state = ROTATE_ROBOT;
          last_state = DELAY_STOP;
        }
        break;
    }
    //checks the line following sensor if the end of the maze has been reached
    int sensorState = lineFinder.readSensors();
    if(task_solve_reverse == 0) {
      if(sensorState == S1_IN_S2_IN) {
        stop_motors();
        task_solve_reverse = 1;
        switch(current_direction) {
          case NORTH:
            target_direction = SOUTH;
            target_angle = south_value;
            break;
          case EAST:
            target_direction = WEST;
            target_angle = west_value;
            break;
          case SOUTH:
            target_direction = NORTH;
            target_angle = north_value;
            break;
          case WEST:
            target_direction = EAST;
            target_angle = east_value;
            break;
        }
        turn_direction = CW;
        current_state = ROTATE_ROBOT;
        state_flag = 0;
      }
    }
    Encoder_1.loop();         //refresh the encoder position values
    Encoder_2.loop();
    if(millis() - ts_timestamp >= 100) {
      ts_count++;
      ts_timestamp = millis();
    }
  }
}

void loop() {
// Empty. Things are done in Tasks.
// put your main code here, to run repeatedly:
}
