#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "crtp.h"
#include "vl53l5cx_api.h"
#include "I2C_expander.h"

#include "log.h"
#include "param.h"
#include "stabilizer_types.h"
#include "commander.h"
#include "usec_time.h"


#define DEBUG_MODULE "TOFMATRIX"
#define NR_OF_SENSORS 4
#define TEN_NR_OF_SENSORS 40
#define NR_OF_PIXELS 64
// in position_controller_pid.c max velocity CHANGED to 2.0f! 
#define MAXVEL 1.0f
#define MINRANGE 300
#define MAXRANGE 2000
#define MAXRANGESPEED 1200 
#define MINMOVE 600 // don't go lower than 300
#define YAWSPEED 75 //deg/s
#define MinGap 4 //min gap to move


static VL53L5CX_Configuration tof_dev[NR_OF_SENSORS];
static VL53L5CX_ResultsData tof_data;

void send_command(uint8_t command, uint8_t arg);
void send_data_packet(uint8_t *data, uint16_t data_len);
void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index);

uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);

#define TOF_I2C_ADDR 0x56
uint8_t tof_i2c_addresses[NR_OF_SENSORS];
uint32_t min_dis_col[TEN_NR_OF_SENSORS];

static const float height_fly = 0.9f;

typedef struct {
    uint8_t max_len;
    int max_start_pos;
    int end_pos;
    uint8_t middle_pos;
}GapInfo;

// Swap function for find median
void swap(uint32_t* a, uint32_t* b) {
    uint32_t temp = *a;
    *a = *b;
    *b = temp;
}

// Function to partition the array and return the pivot index
static uint32_t partition(uint32_t arr[], uint32_t low, uint32_t high) {
    uint32_t pivot = arr[high];
    uint32_t i = (low - 1);

    for (uint32_t j = low; j <= high - 1; j++) {
        if (arr[j] < pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }
    swap(&arr[i + 1], &arr[high]);
    return (i + 1);
}

// Function to perform quickselect
static uint32_t quickselect(uint32_t arr[], uint32_t low, uint32_t high, uint32_t k) {
    if (low < high) {
        uint32_t pivotIndex = partition(arr, low, high);

        // If the pivot is the kth element, return it
        if (pivotIndex == k)
            return arr[pivotIndex];

        // If the pivot is greater than kth element, search in the left subarray
        else if (pivotIndex > k)
            return quickselect(arr, low, pivotIndex - 1, k);

        // Else, search in the right subarray
        else
            return quickselect(arr, pivotIndex + 1, high, k);
    }
    // Return the element if the array has only one element
    return arr[low];
}

// Get primary velocity from secondary
static float getPrimeVel(float vel, float velA)
{
   if (velA >= 0.0f) {
      return vel - velA;
   } else {
      return vel + velA;
   }
}

// Setting setpoint
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate) 
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

// Translating desired point to fly to into setpoint 
static void midSetpoint(setpoint_t *setpoint, uint8_t middle_pos, float vel)
{
   uint8_t direction = middle_pos/10;
   uint8_t semi_direction = middle_pos%10;
   float semi_vel = 0;
   float velFront = 0;
   float velSide = 0;
   float yawrate = 0;

   if (semi_direction < 3){
      yawrate = -10.0f*(3-semi_direction);
   }else if (semi_direction > 6){
      yawrate = 10.0f*(semi_direction % 6);
   }

   if (semi_direction < 3){
      semi_vel = -0.1f*(4-semi_direction);
   }else if (semi_direction > 6){
      semi_vel = 0.1f*(semi_direction % 6);
   }

   switch (direction)
   {
      case 0:
         velSide = semi_vel*vel;
         velFront = getPrimeVel(vel,velSide);
         break;
      case 1:
         velFront = -semi_vel*vel;
         velSide = getPrimeVel(vel,velFront);
         break;   
      case 2:
         velSide = -semi_vel*vel;
         velFront = - getPrimeVel(vel,velSide);        
         break;
      case 3:
         velFront = semi_vel*vel;
         velSide = - getPrimeVel(vel,velFront);
         break;   
   }
   setHoverSetpoint(setpoint, velFront, velSide, height_fly, yawrate);   
}

// Find Consecutive Falses
GapInfo findMaxConsecFalse(bool threat[]) {
   GapInfo result;
   result.max_len = 0;
   uint8_t curr_len = 0;
   int start_pos = -1;
   result.max_start_pos = -1;
   result.end_pos = -1;

   for (uint8_t i = 0; i < 2 * TEN_NR_OF_SENSORS; i++) {
      uint8_t j = i % TEN_NR_OF_SENSORS; // Wrap around to the beginning of the array
      if (threat[j] == false) {
         if (curr_len == 0) {
               start_pos = j;
         }
         curr_len++;
      } else {
         if (curr_len > result.max_len) {
               result.max_len = curr_len;
               result.max_start_pos = start_pos;
               result.end_pos = (j - 1 + TEN_NR_OF_SENSORS) % TEN_NR_OF_SENSORS; // Wrap around to the end of the array
         }
         curr_len = 0;
      }
   }

   if (result.max_len == 0 && !threat[0]) {
      // If the array is all false
      result.max_len = 40;
      result.max_start_pos = 0;
      result.end_pos = 39;
   }
   result.middle_pos = 4;
   if (result.max_len != 0) { //calculate middle_pos if there is a gap
      if (result.end_pos < result.max_start_pos) {
         result.middle_pos = (result.max_start_pos + result.end_pos + TEN_NR_OF_SENSORS) / 2;
         result.middle_pos %= TEN_NR_OF_SENSORS;
      } else {
         result.middle_pos = (result.max_start_pos + result.end_pos) / 2;
         result.middle_pos %= TEN_NR_OF_SENSORS;
      }
      //DEBUG_PRINT("Middle position 1: %d\n", middle_pos);
   }


   return result;
}

//find median and average of lowest 2 pixels
static void findAverages(const uint32_t array[], uint32_t *median, uint32_t *avgSmallest)
{
   // Initialize variables
   uint32_t sumAll = 0;
   uint32_t min1 = MAXRANGE;
   uint32_t min2 = MAXRANGE;

   // Calculate the sum of all values and find the two smallest values
   for (uint8_t i = 0; i < TEN_NR_OF_SENSORS; i++)
   {
      sumAll += array[i];

      if (array[i] < min1)
      {
         min2 = min1;
         min1 = array[i];
      }
      else if (array[i] < min2)
      {
         min2 = array[i];
      }
   }

   // Calculate the averages
   //*avgAll = sumAll / TEN_NR_OF_SENSORS;
   *avgSmallest = (min1 + min2) / 2;

   // Create a temporary array to store the values for finding the median
    uint32_t tempArray[TEN_NR_OF_SENSORS];
    for (uint8_t i = 0; i < TEN_NR_OF_SENSORS; i++) {
        tempArray[i] = array[i];
    }

    // Find the median using quickselect
   *median = quickselect(tempArray, 0, TEN_NR_OF_SENSORS - 1, (TEN_NR_OF_SENSORS-1)/2);

}

//calculate velocity of drone
static float findVel(int min_average, float last_vel)
{
   float current_vel;
   if (min_average < MINRANGE)
   {
      //current_vel = MAXVEL;
      current_vel = 0.6f * MAXVEL;
   }
   else if (min_average > MAXRANGESPEED)
   {
      current_vel = 0.2f;
   }
   else
   {  
      if (min_average < MINRANGE + 200){
         //current_vel = current_vel - (1 + (MINRANGE-min_average)/300) * 0.4f * MAXVEL;
         current_vel = MAXVEL- ((200-(min_average-MINRANGE-1)) / (float)(200)) * (0.4f * MAXVEL);
      }/*else if (min_average < MINRANGE + 400){
         current_vel = MAXVEL;
      }*/else{
         current_vel = MAXVEL - ((min_average-(MINRANGE+199)) / (float)(MAXRANGESPEED - (MINRANGE+200))) * (MAXVEL - 0.2f);
      }
   }
   float average_vel;
   /*if (last_vel< current_vel){
      average_vel = (current_vel + 2*last_vel) / 3;
   }else{
      average_vel = (2*current_vel + last_vel) / 3;
   }*/
   average_vel = (current_vel + 2*last_vel) / 3;

   return average_vel;
}

//calculate range to move
static uint32_t calculateRange(uint32_t Range, uint32_t average)
{
   float weighted_average = 0.98 * Range + 0.02 * average;
   uint32_t result = (uint32_t)weighted_average;
   return result > MINRANGE ? result : MINRANGE;
}

//check if opposite side
static bool notOppositeDirection(uint8_t middle_position, uint8_t old_position){
   uint8_t new = middle_position/10;
   uint8_t old = old_position/10;
   if(new > old){
      return (new - old != 2);
   }
   return (old - new != 2);
}

void appMain() {

   DEBUG_PRINT("Size of configuration %d \n", sizeof(VL53L5CX_Configuration));

   DEBUG_PRINT("Configured for %d ToF sensor(s) \n", NR_OF_SENSORS);
   // Configure GPIO expander pins modes
   I2C_expander_initialize();

   // Define the address of each ToF matrix sensor
   for(uint8_t i=0; i<NR_OF_SENSORS; i++)
      tof_i2c_addresses[i] = TOF_I2C_ADDR + 2 + 2*i;

   // Configure the ToF sensor(s)
   for(uint8_t i=0; i<NR_OF_SENSORS; i++) {
      I2C_expander_set_pin(i, 1); 

      // Configure the current sensor
      uint8_t status = config_sensors(&tof_dev[i], tof_i2c_addresses[i]);
      DEBUG_PRINT("Sensor %d conf. status: %d  (0 means ok) \n", i, status);

      // Start ranging
      status = vl53l5cx_start_ranging(&tof_dev[i]);
      DEBUG_PRINT("Sensor %d ranging status: %d  (0 means ok) \n", i, status);

      if (status == 0)
         I2C_expander_set_pin(LED0, 1);
   }

   uint8_t reg_value;
   i2cdevReadByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS, OUTPUT_REG_ADDRESS, &reg_value);
   DEBUG_PRINT("Sensor reg_value: %d \n", reg_value);

   uint8_t ranging_ready = 255;
   uint8_t get_data_success = 255;
   uint8_t to_send_buffer[4*NR_OF_PIXELS];

   static setpoint_t setpoint;

   DEBUG_PRINT("Wait 2s.\n");
   vTaskDelay(M2T(2000));
   DEBUG_PRINT("Start flying.\n");

   //time limit of fly time
   uint64_t start_time = usecTimestamp(); // Record the start time of the loop

   // Fly to correct height for 2s
   setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
   commanderSetSetpoint(&setpoint, 3); //set setpoint

   while (usecTimestamp() - start_time < 3000000)
   {
      vTaskDelay(M2T(100));
      commanderSetSetpoint(&setpoint, 3); //set setpoint
   }

   //DEBUG_PRINT("Start using sensors.\n");
  
   // reset timer
   start_time = usecTimestamp();

   float vel = 0;
   uint32_t range = MINRANGE+400;
   uint8_t old_pos = 4;

   while(1) {
      // Initialize min_dis_col with UINT32_MAX
      for (uint8_t i = 0; i < TEN_NR_OF_SENSORS; i++) {
         min_dis_col[i] = MAXRANGE;
      }

      // Read ToF Data
      for (uint8_t p=0; p<NR_OF_SENSORS; p++) {
         // Change index as ToF sensores are numbered 0 2 1 3 counter clockwise
         uint8_t i = p;
         if(i==1){
            i=2;
         } else if(i==2){
            i=1;
         }

         //wait untill sensor is ready
         vl53l5cx_check_data_ready(&tof_dev[i], &ranging_ready);
         while (ranging_ready != 1){ 
            vl53l5cx_check_data_ready(&tof_dev[i], &ranging_ready);  // poll for data-ready
            commanderSetSetpoint(&setpoint, 3); //set setpoint
            vTaskDelay(M2T(5));
         }

         //uint64_t sensor_time_start = usecTimestamp();

         // Read Tof Data and fill min_dis_col
         if (ranging_ready == 1) {
            get_data_success = vl53l5cx_get_ranging_data(&tof_dev[i], &tof_data);
            if (get_data_success == VL53L5CX_STATUS_OK) {
               // pixel indexes are horizontally and vertically flipped (see datasheet of sensor)
               for(uint8_t j=0; j<NR_OF_PIXELS-16; j++){
                  uint8_t k = j % 8;
                  if ((tof_data.distance_mm[j] < min_dis_col[p*10+8-k]) && (tof_data.distance_mm[j] > 30) && ((tof_data.target_status[j] == 5) || (tof_data.target_status[j] == 10)|| (tof_data.target_status[j] == 4))) {
                     //DEBUG_PRINT("Sensor %d, pixel %d: %ld\n", i, j, (int32_t)tof_data.distance_mm[j]);
                     min_dis_col[p*10+8-k] = tof_data.distance_mm[j];
                  }
               }  
               //set blind zone to edge value
               min_dis_col[p*10] = min_dis_col[p*10+1];
               min_dis_col[p*10+9] = min_dis_col[p*10+8];
               
               memcpy(&to_send_buffer[0], (uint8_t *)(&tof_data.distance_mm[0]), 2*NR_OF_PIXELS);
               memcpy(&to_send_buffer[2*NR_OF_PIXELS], (uint8_t *)(&tof_data.nb_target_detected[0]), NR_OF_PIXELS);
               memcpy(&to_send_buffer[3*NR_OF_PIXELS], (uint8_t *)(&tof_data.target_status[0]), NR_OF_PIXELS);
               
               send_command(1, (4*NR_OF_PIXELS)/28 + 1);
               send_data_packet(&to_send_buffer[0], 4*NR_OF_PIXELS);
            }
            else{
               DEBUG_PRINT("Critical ERROR VL53L5CX_STATUS_OK NOT OK");
            }
         }
         //DEBUG_PRINT("Sensor %d Time: %lld  , ", p, usecTimestamp()-sensor_time_start);
         ranging_ready = 2;
      }
      //uint64_t loop_time_start = usecTimestamp();
      vTaskDelay(M2T(10));
      //Calculate Velocity and Range
      uint64_t algorithm_time_start = usecTimestamp();
      uint32_t min_average, median;
      findAverages(min_dis_col, &median, &min_average);
      vel = findVel(min_average, vel);
      range = calculateRange(range, median);
      //range = 2500;


      //Objects Arrays
      bool obj_2100[TEN_NR_OF_SENSORS] = {false};
      bool obj_1900[TEN_NR_OF_SENSORS] = {false};
      bool obj_1700[TEN_NR_OF_SENSORS] = {false};
      bool obj_1500[TEN_NR_OF_SENSORS] = {false};
      bool obj_1300[TEN_NR_OF_SENSORS] = {false};
      bool obj_1100[TEN_NR_OF_SENSORS] = {false};
      bool obj_900[TEN_NR_OF_SENSORS] = {false};
      bool obj_750[TEN_NR_OF_SENSORS] = {false};
      bool obj_600[TEN_NR_OF_SENSORS] = {false};
      bool obj_450[TEN_NR_OF_SENSORS] = {false};
      bool obj_300[TEN_NR_OF_SENSORS] = {false};
      bool obj_200[TEN_NR_OF_SENSORS] = {false};

      //Change to True if Obj Detected
      for (uint8_t i = 0; i < TEN_NR_OF_SENSORS; i++) {
         uint32_t min_dis = min_dis_col[i];
         if (min_dis < 2100){
            obj_2100[i] = true;
            obj_1900[i] = (min_dis < 1900);
            obj_1700[i] = (min_dis < 1700);
            obj_1500[i] = (min_dis < 1500);
            if (min_dis < 1300){
               obj_1300[i] = true;
               obj_1100[i] = (min_dis < 1100);
               obj_900[i] = (min_dis < 900);
               obj_750[i] = (min_dis < 750);
               if (min_dis < 600){
                  obj_600[i] = true;
                  obj_450[i] = (min_dis < 450);
                  obj_300[i] = (min_dis < 300);
                  obj_200[i] = (min_dis < 200);
               }
            }
         }
      }

      //find gap
      GapInfo info_2100;
      if(min_average < MINMOVE || range > 2100){
         info_2100 = findMaxConsecFalse(obj_2100);
      }
      if (range > 2100 && min_average > MINMOVE && info_2100.max_len > 36){
         setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
         old_pos = 80;
         vel = 0;
      }else if ((min_average < MINMOVE || range > 2100) && info_2100.max_len > MinGap){
         if(notOppositeDirection(info_2100.middle_pos, old_pos)){
            midSetpoint(&setpoint, info_2100.middle_pos, vel); //set position
            old_pos = info_2100.middle_pos;
         }else{
            if(old_pos >= TEN_NR_OF_SENSORS || obj_2100[old_pos] || obj_2100[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_2100[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
               setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
               old_pos = 80;
               vel = 0;
            }else{
               midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
               old_pos = old_pos;
               //old_pos = 80;
            }
            /*
            if (old_pos < TEN_NR_OF_SENSORS ){
               //if left right and center of old pos is still clear
               if(!(obj_2100[old_pos] || obj_2100[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_2100[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS])){
                  midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                  old_pos = old_pos;
               }
               else {
                  setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                  old_pos = 80;
               }
            }
            else {
               setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
               old_pos = 80;
            }*/

         }
      }else{
         GapInfo info_1900;
         if(min_average < MINMOVE || range > 1900){
            info_1900 = findMaxConsecFalse(obj_1900); //reset and rerun on threat
         }
         if (range > 1900 && min_average > MINMOVE && info_1900.max_len > 36){
            setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
            old_pos = 80;
            vel = 0;
         }else if ((min_average < MINMOVE || range > 1900) && info_1900.max_len > MinGap ){
            if(notOppositeDirection(info_1900.middle_pos, old_pos)){
               midSetpoint(&setpoint, info_1900.middle_pos, vel); //set position
               old_pos = info_1900.middle_pos;
            }else{
               if(old_pos >= TEN_NR_OF_SENSORS || obj_2100[old_pos] || obj_2100[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_2100[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                  setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                  old_pos = 80;
                  vel = 0;
               }else{
                  midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                  old_pos = old_pos;
                  //old_pos = 80;
               }
            }
         }else{
            GapInfo info_1700;
            if(min_average < MINMOVE || range > 1700){
               info_1700 = findMaxConsecFalse(obj_1700); //reset and rerun on threat
            }

            if (range > 1700 && min_average > MINMOVE && info_1700.max_len > 36){
               setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
               old_pos = 80;
               vel = 0;
            }else if ((min_average < MINMOVE || range > 1700) && info_1700.max_len > MinGap ){
               if(notOppositeDirection(info_1700.middle_pos, old_pos)){
                  midSetpoint(&setpoint, info_1700.middle_pos, vel); //set position
                  old_pos = info_1700.middle_pos;
               }else{
                  if(old_pos >= TEN_NR_OF_SENSORS || obj_2100[old_pos] || obj_2100[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_2100[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                     setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                     old_pos = 80;
                     vel = 0;
                  }else{
                     midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                     old_pos = old_pos;
                     //old_pos = 80;
                  }
               }
            }else{
               GapInfo info_1500;
               if(min_average < MINMOVE || range > 1500){
                  info_1500 = findMaxConsecFalse(obj_1500); //reset and rerun on threat
               }
               if (range > 1500 && min_average > MINMOVE && info_1500.max_len > 36){
                  setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                  old_pos = 80;
                  vel = 0;
               }else if ((min_average < MINMOVE || range > 1500) && info_1500.max_len > MinGap ){
                  if(notOppositeDirection(info_1500.middle_pos, old_pos)){
                     midSetpoint(&setpoint, info_1500.middle_pos, vel); //set position
                     old_pos = info_1500.middle_pos;
                  }else{
                     if(old_pos >= TEN_NR_OF_SENSORS || obj_1900[old_pos] || obj_1900[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_1900[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                        setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                        old_pos = 80;
                        vel = 0;
                     }else{
                        midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                        old_pos = old_pos;
                        //old_pos = 80;
                     }
                  }
               }else{
                  GapInfo info_1300;
                  if(min_average < MINMOVE || range > 1300){
                     info_1300 = findMaxConsecFalse(obj_1300); //reset and rerun on threat
                  }
                  if (range > 1300 && min_average > MINMOVE && info_1300.max_len > 36){
                     setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                     old_pos = 80;
                     vel = 0;
                  }else if ((min_average < MINMOVE || range > 1300) && info_1300.max_len > MinGap){
                     if(notOppositeDirection(info_1300.middle_pos, old_pos)){
                        midSetpoint(&setpoint, info_1300.middle_pos, vel); //set position
                        old_pos = info_1300.middle_pos;
                     }else{
                        if(old_pos >= TEN_NR_OF_SENSORS || obj_1700[old_pos] || obj_1700[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_1700[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                           setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                           old_pos = 80;
                           vel = 0;
                        }else{
                           midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                           old_pos = old_pos;
                           //old_pos = 80;
                        }
                     }
                  }else{
                     GapInfo info_1100;
                     if(min_average < MINMOVE || range > 1100){
                        info_1100 = findMaxConsecFalse(obj_1100); //reset and rerun on threat
                     }
                     if (range > 1100 && min_average > MINMOVE && info_1100.max_len > 36){
                        setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                        old_pos = 80;
                        //vel = 0;
                     }else if ((min_average < MINMOVE || range > 1100) && info_1100.max_len > MinGap){
                        if(notOppositeDirection(info_1100.middle_pos, old_pos)){
                           midSetpoint(&setpoint, info_1100.middle_pos, vel); //set position
                           old_pos = info_1100.middle_pos;
                        }else{
                           if(old_pos >= TEN_NR_OF_SENSORS || obj_1500[old_pos] || obj_1500[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_1500[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                              setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                              old_pos = 80;
                              vel = 0;
                           }else{
                              midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                              old_pos = old_pos;
                              //old_pos = 80;
                           }
                        }
                     }else{
                        GapInfo info_900;
                        if(min_average < MINMOVE || range > 900){
                           info_900 = findMaxConsecFalse(obj_900); //reset and rerun on threat
                        }
                        if (range > 900 && min_average > MINMOVE && info_900.max_len > 36){
                           setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                           old_pos = 80;
                           vel = 0;
                        }else if ((min_average < MINMOVE || range > 900) && info_900.max_len > MinGap){
                           if(notOppositeDirection(info_900.middle_pos, old_pos)){
                              midSetpoint(&setpoint, info_900.middle_pos, vel); //set position
                              old_pos = info_900.middle_pos;
                           }else{
                              if(old_pos >= TEN_NR_OF_SENSORS || obj_1300[old_pos] || obj_1300[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_1300[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                                 setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                                 old_pos = 80;
                                 //vel = 0;
                              }else{
                                 midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                 old_pos = old_pos;
                                 //old_pos = 80;
                              }
                           }
                        }else{
                           GapInfo info_750;
                           if(range > 750 || min_average < MINMOVE){
                              info_750 = findMaxConsecFalse(obj_750); //reset and rerun on threat
                           }
                           if (range > 750 && min_average > MINMOVE && info_750.max_len > 36){
                              setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                              old_pos = 80;
                              vel = 0;
                           }else if ((range > 750 || min_average < MINMOVE) && info_750.max_len > MinGap){
                              if(notOppositeDirection(info_750.middle_pos, old_pos)){
                                 midSetpoint(&setpoint, info_750.middle_pos, vel); //set position
                                 old_pos = info_750.middle_pos;
                              }else{
                                 if(old_pos >= TEN_NR_OF_SENSORS || obj_1100[old_pos] || obj_1100[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_1100[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                                    setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                                    old_pos = 80;
                                    vel = 0;
                                 }else{
                                    midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                    old_pos = old_pos;
                                    //old_pos = 80;
                                 }
                              }
                           }else{
                              GapInfo info_600;
                              if(range > 600 || min_average < MINMOVE){
                                 info_600 = findMaxConsecFalse(obj_600); //reset and rerun on threat
                              }
                              if (range > 600 && min_average > MINMOVE && info_600.max_len > 36){
                                 setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                                 old_pos = 80;
                                 vel = 0;
                              }else if ((range > 600 || min_average < MINMOVE) && info_600.max_len > MinGap){
                                 if(notOppositeDirection(info_600.middle_pos, old_pos)){
                                    midSetpoint(&setpoint, info_600.middle_pos, vel); //set position
                                    old_pos = info_600.middle_pos;
                                 }else{
                                    if(old_pos >= TEN_NR_OF_SENSORS || obj_900[old_pos] || obj_900[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_900[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                                       setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                                       old_pos = 80;
                                       vel = 0;
                                    }else{
                                       midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                       old_pos = old_pos;
                                       //old_pos = 80;
                                    }
                                 }
                              }else{
                                 GapInfo info_450;
                                 if(range > 450 || min_average < MINMOVE){
                                    info_450 = findMaxConsecFalse(obj_450); //reset and rerun on threat
                                 }
                                 if (range > 450 && info_450.max_len > 39){
                                    setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                                    old_pos = 80;
                                    vel = 0;
                                 }else if ((range > 450 || min_average < MINMOVE) && info_450.max_len > MinGap){
                                    if(notOppositeDirection(info_450.middle_pos, old_pos)){
                                       midSetpoint(&setpoint, info_450.middle_pos, vel); //set position
                                       old_pos = info_450.middle_pos;
                                    }else{
                                       if(old_pos >= TEN_NR_OF_SENSORS || obj_750[old_pos] || obj_750[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_750[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                                          setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                                          old_pos = 80;
                                          vel = 0;
                                       }else{
                                          midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                          old_pos = old_pos;
                                          //old_pos = 80;
                                       }
                                    }
                                 }else{
                                    GapInfo info_300 = findMaxConsecFalse(obj_300); //reset and rerun on threat
                                    if (info_300.max_len > 39){
                                       setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                                       old_pos = 80;
                                       vel = 0;
                                       DEBUG_PRINT("300 yaw\n");
                                    }else if (info_300.max_len > MinGap){
                                       if(notOppositeDirection(info_300.middle_pos, old_pos)){
                                          midSetpoint(&setpoint, info_300.middle_pos, vel); //set position
                                          old_pos = info_300.middle_pos;
                                       }else{
                                          if(old_pos >= TEN_NR_OF_SENSORS || obj_600[old_pos] || obj_600[(old_pos+1)%TEN_NR_OF_SENSORS] || obj_600[(old_pos+TEN_NR_OF_SENSORS-1)%TEN_NR_OF_SENSORS]){
                                             setHoverSetpoint(&setpoint, 0, 0, height_fly, 0);
                                             old_pos = 80;
                                             vel = 0;
                                          }else{
                                             midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                             //old_pos = old_pos;
                                             old_pos = 80;
                                          }
                                       }
                                       DEBUG_PRINT("300 fly\n");
                                    }else{
                                       //No Gap Found Yawing
                                       /*setHoverSetpoint(&setpoint, 0, 0, height_fly, 100);
                                       old_pos = 80;
                                       vel = 0;
                                       DEBUG_PRINT("300 no gap\n");*/

                                       GapInfo info_200 = findMaxConsecFalse(obj_200); //reset and rerun on threat
                                       vel = 0.3f;
                                       if (info_200.max_len > 39){
                                          setHoverSetpoint(&setpoint, 0, 0, height_fly, YAWSPEED); // no threat yawing
                                          old_pos = 80;
                                          vel = 0;
                                          DEBUG_PRINT("200 yaw\n");
                                       }else if (info_200.max_len > MinGap){
                                          if(notOppositeDirection(info_200.middle_pos, old_pos)){
                                             midSetpoint(&setpoint, info_200.middle_pos, vel); //set position
                                             old_pos = info_200.middle_pos;
                                          }else{
                                             midSetpoint(&setpoint, old_pos, vel); //continue flying towards old pos
                                             //old_pos = old_pos;
                                             old_pos = 80;
                                          }
                                          DEBUG_PRINT("200 fly\n");
                                       }else{
                                          //No Gap Found Yawing
                                          setHoverSetpoint(&setpoint, 0, 0, height_fly, 100);
                                          old_pos = 80;
                                          vel = 0;
                                          DEBUG_PRINT("200 no gap\n");
                                       }
                                    }
                                 }
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
      }
      //DEBUG_PRINT("\n Move towards %d \n Range is %ld \n Velocity is %f \n \n", old_pos, range, (double)vel);
      commanderSetSetpoint(&setpoint, 3); //set setpoint
      uint64_t now_time = usecTimestamp();
      DEBUG_PRINT("%lld\n", now_time - algorithm_time_start);

      //land after 20s
      if ((usecTimestamp() - start_time) > 90000000){
         break;
      }

      //uint64_t loop_time = usecTimestamp() - loop_time_start;
      //DEBUG_PRINT("Loop Time %lld \n", loop_time);
      vTaskDelay(M2T(30));
   }
   
   //landing
   DEBUG_PRINT("Landing\n");
   setHoverSetpoint(&setpoint, 0, 0, 0, 0);
   commanderSetSetpoint(&setpoint, 3);
   vTaskDelay(M2T(750));
   commanderSetSetpoint(&setpoint, 3);
}

void send_data_packet(uint8_t *data, uint16_t data_len) {
   uint8_t packets_nr = 0;
   if (data_len%28 > 0)
      packets_nr = data_len/28 + 1;
   else
      packets_nr = data_len/28;

   for (uint8_t idx=0; idx<packets_nr; idx++)
      if(data_len - 28*idx >= 28)
         send_data_packet_28b(&data[28*idx], 28, idx);
      else
         send_data_packet_28b(&data[28*idx], data_len - 28*idx, idx);
}


void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index) {
   CRTPPacket pk;
   pk.header = CRTP_HEADER(1, 0); // first arg is the port number
   pk.size = size + 2;
   pk.data[0] = 'D';
   pk.data[1] = index;
   memcpy(&(pk.data[2]), data, size);
   crtpSendPacketBlock(&pk);
}

void send_command(uint8_t command, uint8_t arg) {
   CRTPPacket pk;
   pk.header = CRTP_HEADER(1, 0); // first arg is the port number
   pk.size = 5;
   pk.data[0] = 'C';
   pk.data[1] = command;
   pk.data[2] = arg;
   crtpSendPacketBlock(&pk);
}


uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address) {
   p_dev->platform = VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

   uint8_t status = 0;
   // Initialize the sensor
   status += vl53l5cx_init(p_dev); 

   // Change I2C address
   status += vl53l5cx_set_i2c_address(p_dev, new_i2c_address);
   status += vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);

   // 15Hz frame rate
   status += vl53l5cx_set_ranging_frequency_hz(p_dev, 15);
   status += vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
   status += vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

   return status;
}