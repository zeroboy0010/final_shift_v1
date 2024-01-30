#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Firebase_ESP_Client.h>
#include <EEPROM.h>
#include <addons/TokenHelper.h>
#include <ShiftRegister74HC595.h>
#include "esp_heap_caps.h"

//
/* 1. Define the WiFi credentials */
String Input;
String Output;
String Password;
String SSID;
String PageID;
String Password_Read;
String SSID_Read;
String PageID_Read;

char WIFI_SSID[20];
char WIFI_PASSWORD[20];

bool Wifi_status = true;
bool taskCompleted = false;

/* 2. Define the API Key */
#define API_KEY "AIzaSyBCsGv1SVYiKzgnKmEZSgx9ZbEc_A6gN6w"

/* 3. Define the project ID */
#define FIREBASE_PROJECT_ID "like-counter-3d752"

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "zeroeverything001@gmail.com"
#define USER_PASSWORD "11112222"


// Replace with your network credentials
const char* ssid     = "Like_counter_demo";
const char* password = "12345678";
// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;
// Auxiliar variables to store the current output state
String LED_state = "off";
// Assign output variables to GPIO pins
const int output26 = LED_BUILTIN;

unsigned long dataMillis = 0;

int connected_wifi;
#define Input_Button 4

int writeStringToEEPROM(int addrOffset, const String &strToWrite);
int readStringFromEEPROM(int addrOffset, String *strToRead);

void connect();
void running_ap();

//freertos
// Define two tasks for Blink & AnalogRead.
void TaskMotor( void *pvParameters );
void TaskHalSensor  ( void *pvParameters );
void Task_Data  ( void *pvParameters );
void running_ap  ( void *pvParameters );
//
SemaphoreHandle_t homed_1;
SemaphoreHandle_t homed_2;
SemaphoreHandle_t homed_3;
SemaphoreHandle_t homed_4;
SemaphoreHandle_t homed_5;
SemaphoreHandle_t homed_6;
SemaphoreHandle_t homed_7;
QueueHandle_t like_num;



// put function declarations here:
#define MotorInterfaceType 8

int speed_ = 300;
int accel_ = 150;
int __accel = 150;
int __speed = 300;

//
int arr[7]; 
void extract_digit(int N) 
{ 
    // To store the digit 
    // of the number N 
    
    int i = 0; 
    int r; 
  
    // Till N becomes 0 
    while (N != 0) { 
  
        // Extract the last digit of N 
        r = N % 10; 
  
        // Put the digit in arr[] 
        arr[i] = r; 
        i++; 
  
        // Update N to N/10 to extract 
        // next last digit 
        N = N / 10; 
    } 
} 

xTaskHandle taskHandle_delete;
  

void setup() {
  Serial.begin(9600);
  /////////////////////////////////
  EEPROM.begin(150);
  pinMode(Input_Button,INPUT_PULLUP);

  for (int i=0; i<50;i++){
    Serial.print(".");
    if (digitalRead(Input_Button) == 0){
      connected_wifi = 0;
      break;
    }
    else {
      connected_wifi = 1;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  Serial.println("---------------------");
  if(connected_wifi == 1){
    connect();
    vTaskDelay(pdMS_TO_TICKS(3000));
    /* Assign the api key (required) */

    // while (!Firebase.ready()){


    // }
    //     String documentPath = "project_like_counter/device_001";
    //     if (!taskCompleted)
    //     {
    //         taskCompleted = true;

    //         // For the usage of FirebaseJson, see examples/FirebaseJson/BasicUsage/Create.ino
    //         FirebaseJson content;

    //         content.set("fields/like/integerValue", "0");
    //         content.set("fields/pageID/stringValue",PageID_Read);
    //         content.set("fields/state/stringValue","running");

    //         // info is the collection id, countries is the document id in collection info.
            

    //         Serial.print("Create document... ");

    //         if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw()))
    //             Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    //         else
    //             Serial.println(fbdo.errorReason());

    //         if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw(),"pageID"))
    //             Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    //         else
    //             Serial.println(fbdo.errorReason());
    //         vTaskDelay(pdMS_TO_TICKS(3000));
    //     }

    homed_1 = xSemaphoreCreateBinary();
    homed_2 = xSemaphoreCreateBinary();
    homed_3 = xSemaphoreCreateBinary();
    homed_4 = xSemaphoreCreateBinary();
    homed_5 = xSemaphoreCreateBinary();
    homed_6 = xSemaphoreCreateBinary();
    homed_7 = xSemaphoreCreateBinary();

    like_num = xQueueCreate(2,sizeof(int));
    /////
    xTaskCreatePinnedToCore(
    Task_Data
    ,  "Task_Data" // A name just for humans
    ,  2048 * 10      // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  3  // Priority
    ,  NULL
    ,  0 // Task handle is not used here - simply pass NULL
    );
  xTaskCreatePinnedToCore(
    TaskMotor
    ,  "TaskMotor" // A name just for humans
    ,  2048 * 4       // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  5  // Priority
    ,  NULL
    , 1 // Task handle is not used here - simply pass NULL
    );
  xTaskCreatePinnedToCore(
    TaskHalSensor
    ,  "TaskHalSensor" // A name just for humans
    ,  2048 * 3       // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  4  // Priority
    ,  &taskHandle_delete
    , 1 // Task handle is not used here - simply pass NULL
    );
  }
  else {
    xTaskCreatePinnedToCore(
    running_ap
    ,  "running_ap" // A name just for humans
    ,  2048 * 4       // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  3  // Priority
    ,  NULL
    ,  0 // Task handle is not used here - simply pass NULL
    );
  }

}

void loop() {
  // put your main code here, to run repeatedly
}


void TaskHalSensor(void *pvParameters){
  // pin for shiftin
  const uint8_t dataPin = 16;   /* Q7 */
  const uint8_t clockPin = 18;  /* CP */
  const uint8_t latchPin = 17;  /* PL */
  const uint8_t numBits = 8;   /* Set to 8 * number of shift registers */
  pinMode(dataPin,INPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(latchPin,OUTPUT);
  uint8_t array_bit[8] = {0,0,0,0,0,0,0,0};
  uint8_t old_array_bit[8] = {0,0,0,0,0,0,0,0};
  uint8_t hal_sensor_home[8] = {1,1,1,1,1,1,1,1};
  vTaskDelay(pdMS_TO_TICKS(3000));
  for(;;){
    // Step 1: Sample
    digitalWrite(latchPin, LOW);
    digitalWrite(latchPin, HIGH);

    // Step 2: Shift
    // Serial.print("Bits: ");
    for (int i = 0; i < numBits; i++) {
      int bit = digitalRead(dataPin);
      array_bit[i] = bit;
      if (bit == HIGH) {
        // Serial.print("1");
      } else {
        // Serial.print("0");
      }
      
      digitalWrite(clockPin, HIGH); // Shift out the next bit
      digitalWrite(clockPin, LOW);
    }
    // Serial.println();
    // part read from shift in
    for(int i = 0;i<=7;i++){
      if(array_bit[i] < old_array_bit[i]){
        if(i == 1){
          xSemaphoreGive(homed_7);
        }
        else if(i == 2){
          xSemaphoreGive(homed_6);
        }
        else if(i == 3){
          xSemaphoreGive(homed_5);
        }
        else if(i == 4){
          xSemaphoreGive(homed_4);
        }
        else if(i == 5){
          xSemaphoreGive(homed_3);
        }
        else if(i == 6){
          xSemaphoreGive(homed_2);
        }
        else if(i == 7){
          xSemaphoreGive(homed_1);
        }
      }
      else{
        old_array_bit[i] = array_bit[i];
      }
    }
    // Serial.println();
    vTaskDelay(pdMS_TO_TICKS(400));

  }
  vTaskDelete(NULL);
}



// put function definitions here:
void TaskMotor(void *pvParameters)
{
  ShiftRegister74HC595 <4> sr(15,13,14);  // (data pin, clock pin, latch pin)
  // direction of every motor
  int dir1 = -1;
  int dir2 = 1;
  int dir3 = -1;
  int dir4 = -1;
  int dir5 = 1;
  int dir6 = -1;
  int dir7 = -1;
  int dir8 = -1;

  uint8_t step1;
  uint8_t step2;
  uint8_t step3;
  uint8_t step4;
  uint8_t step5;
  uint8_t step6;
  uint8_t step7;
  uint8_t step8;

  uint16_t tick = 409;
  // stepper1
  uint8_t stepper1_new_number = 0;
  uint8_t stepper1_old_number = 0;
  uint8_t stepper1_finish_homing = 0; 
  uint8_t stepper1_Homing_sensor = 1;
  // stepper2
  uint8_t stepper2_new_number = 9;
  uint8_t stepper2_old_number = 9;
  uint8_t stepper2_finish_homing = 0; 
  uint8_t stepper2_Homing_sensor = 1;
  // stepper3
  uint8_t stepper3_new_number = 9;
  uint8_t stepper3_old_number = 9;
  uint8_t stepper3_finish_homing = 0; 
  uint8_t stepper3_Homing_sensor = 1;
  // stepper4
  uint8_t stepper4_new_number = 8;
  uint8_t stepper4_old_number = 8;
  uint8_t stepper4_finish_homing = 0; 
  uint8_t stepper4_Homing_sensor = 1;
    // stepper5
  uint8_t stepper5_new_number = 1;
  uint8_t stepper5_old_number = 1;
  uint8_t stepper5_finish_homing = 0; 
  uint8_t stepper5_Homing_sensor = 1;
    // stepper6
  uint8_t stepper6_new_number = 6;
  uint8_t stepper6_old_number = 6;
  uint8_t stepper6_finish_homing = 0; 
  uint8_t stepper6_Homing_sensor = 1;
    // stepper7
  uint8_t stepper7_new_number = 6;
  uint8_t stepper7_old_number = 6;
  uint8_t stepper7_finish_homing = 0; 
  uint8_t stepper7_Homing_sensor = 1;

  AccelStepper myStepper1(MotorInterfaceType);
  AccelStepper myStepper2(MotorInterfaceType);
  AccelStepper myStepper3(MotorInterfaceType);
  AccelStepper myStepper4(MotorInterfaceType);
  AccelStepper myStepper5(MotorInterfaceType);
  AccelStepper myStepper6(MotorInterfaceType);
  AccelStepper myStepper7(MotorInterfaceType);
  // 
  myStepper1.setMaxSpeed(speed_);
  myStepper1.setAcceleration(accel_);
  myStepper1.moveTo(dir1 * 10000000);
  myStepper2.setMaxSpeed(speed_);
  myStepper2.setAcceleration(accel_);
  myStepper2.moveTo(dir2 * 10000000);
  myStepper3.setMaxSpeed(speed_);
  myStepper3.setAcceleration(accel_);
  myStepper3.moveTo(dir3 * 10000000);
  myStepper4.setMaxSpeed(speed_);
  myStepper4.setAcceleration(accel_);
  myStepper4.moveTo(dir4 * 10000000);
  myStepper5.setMaxSpeed(speed_);
  myStepper5.setAcceleration(accel_);
  myStepper5.moveTo(dir5 * 10000000);
  myStepper6.setMaxSpeed(speed_);
  myStepper6.setAcceleration(accel_);
  myStepper6.moveTo(dir6 * 10000000);
  myStepper7.setMaxSpeed(speed_);
  myStepper7.setAcceleration(accel_);
  myStepper7.moveTo(dir7 * 10000000);

  int like_num_;
  int tick_wait =  5;
  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.println("  runnnn !!!!");
  for (;;)
  { 
    if(stepper1_finish_homing && stepper2_finish_homing && stepper3_finish_homing && stepper4_finish_homing
      && stepper5_finish_homing && stepper6_finish_homing && stepper7_finish_homing){
      vTaskDelete(taskHandle_delete);
    }

    if (xQueueReceive(like_num,&like_num_,2) == pdTRUE){
      Serial.print("Queue recieve!!");
      Serial.println(like_num_);
      extract_digit(like_num_);
      // stepper7_new_number  = (int) arr[0]; //3
      // stepper6_new_number  = (int) arr[1]; //3
      // stepper5_new_number  = (int) arr[2]; //3
      // stepper4_new_number  = (int) arr[3]; //3
      // stepper3_new_number  = (int) arr[4]; //3
      // stepper2_new_number  = (int) arr[5]; //3
      // stepper1_new_number  = (int) arr[6]; //3
    };

    if(stepper1_finish_homing == 0){
      if(xSemaphoreTake(homed_1, tick_wait) == pdTRUE){
        stepper1_finish_homing = 1;
        myStepper1.setMaxSpeed(__speed);
        myStepper1.setAcceleration(__accel);
        myStepper1.stop();
        myStepper1.setCurrentPosition(0);
        myStepper1.moveTo(0);
      }
    }

    if(stepper2_finish_homing == 0){
      if(xSemaphoreTake(homed_2, tick_wait) == pdTRUE){
        stepper2_finish_homing = 1;
        myStepper2.setMaxSpeed(__speed);
        myStepper2.setAcceleration(__accel);
        myStepper2.stop();
        myStepper2.setCurrentPosition(0);
        myStepper2.moveTo(0);
      }
    }

    if(stepper3_finish_homing == 0){
      if(xSemaphoreTake(homed_3, tick_wait) == pdTRUE){
        stepper3_finish_homing = 1;
        myStepper3.setMaxSpeed(__speed);
        myStepper3.setAcceleration(__accel);
        myStepper3.stop();
        myStepper3.setCurrentPosition(0);
        myStepper3.moveTo(0);
      }
    }

    if(stepper4_finish_homing == 0){
      if(xSemaphoreTake(homed_4, tick_wait) == pdTRUE){
        stepper4_finish_homing = 1;
        myStepper4.setMaxSpeed(__speed);
        myStepper4.setAcceleration(__accel);
        myStepper4.stop();
        myStepper4.setCurrentPosition(0);
        myStepper4.moveTo(0);
      }
    }
    if(stepper5_finish_homing == 0){
      if(xSemaphoreTake(homed_5, tick_wait) == pdTRUE){
        stepper5_finish_homing = 1;
        myStepper5.setMaxSpeed(__speed);
        myStepper5.setAcceleration(__accel);
        myStepper5.stop();
        myStepper5.setCurrentPosition(0);
        myStepper5.moveTo(0);
      }
    }
    if(stepper6_finish_homing == 0){
      if(xSemaphoreTake(homed_6, tick_wait) == pdTRUE){
        stepper6_finish_homing = 1;
        myStepper6.setMaxSpeed(__speed);
        myStepper6.setAcceleration(__accel);
        myStepper6.stop();
        myStepper6.setCurrentPosition(0);
        myStepper6.moveTo(0);
      }
    }
    if(stepper7_finish_homing == 0){
      if(xSemaphoreTake(homed_7, tick_wait) == pdTRUE){
        stepper7_finish_homing = 1;
        Serial.println("homed!!!!!!!");
        myStepper7.setMaxSpeed(__speed);
        myStepper7.setAcceleration(__accel);
        myStepper7.stop();
        myStepper7.setCurrentPosition(0);
        myStepper7.moveTo(0);
      }
    }

  if(stepper1_finish_homing == 1 && myStepper1.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper1_new_number != stepper1_old_number){
        int distant_to_move = stepper1_new_number - stepper1_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper1.moveTo(myStepper1.currentPosition() + dir1 * distant_to_move *(int) tick);
        stepper1_old_number = stepper1_new_number;
    }
  }

    if(stepper2_finish_homing == 1 && myStepper2.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper2_new_number != stepper2_old_number){
        int distant_to_move = stepper2_new_number - stepper2_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper2.moveTo(myStepper2.currentPosition() + dir2 * distant_to_move *(int) tick);
        stepper2_old_number = stepper2_new_number;
    }
  }

    if(stepper3_finish_homing == 1 && myStepper3.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper3_new_number != stepper3_old_number){
        int distant_to_move = stepper3_new_number - stepper3_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper3.moveTo(myStepper3.currentPosition() + dir3 * distant_to_move *(int) tick);
        stepper3_old_number = stepper3_new_number;
    }
  }

  if(stepper4_finish_homing == 1 && myStepper4.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper4_new_number != stepper4_old_number){
        int distant_to_move = stepper4_new_number - stepper4_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper4.moveTo(myStepper4.currentPosition() + dir4 * distant_to_move *(int) tick);
        stepper4_old_number = stepper4_new_number;
    }
  }
  if(stepper5_finish_homing == 1 && myStepper5.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper5_new_number != stepper5_old_number){
        int distant_to_move = stepper5_new_number - stepper5_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper5.moveTo(myStepper5.currentPosition() + dir5 * distant_to_move *(int) tick);
        stepper5_old_number = stepper5_new_number;
    }
  }
  if(stepper6_finish_homing == 1 && myStepper6.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper6_new_number != stepper6_old_number){
        int distant_to_move = stepper6_new_number - stepper6_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper6.moveTo(myStepper6.currentPosition() + dir6 * distant_to_move *(int) tick);
        stepper6_old_number = stepper6_new_number;
    }
  }
  if(stepper7_finish_homing == 1 && myStepper7.isRunning() != 1) {
    // Serial.print("current_state = ");
    // Serial.println(myStepper.currentPosition());
    
    if(stepper7_new_number != stepper7_old_number){
        int distant_to_move = stepper7_new_number - stepper7_old_number;
        
        while(distant_to_move < 0){
          distant_to_move += 10;
          }
        // Serial.println(distant_to_move);
        myStepper7.moveTo(myStepper7.currentPosition() + dir7 * distant_to_move *(int) tick);
        stepper7_old_number = stepper7_new_number;
    }
  }

  // run
  if (myStepper1.distanceToGo() != 0){
      myStepper1.run();
      if (myStepper1.StepReturn() != step1){
        step1 = myStepper1.StepReturn();
        sr.setNoUpdate(4,(step1 >> 0 & 1));
        sr.setNoUpdate(5,(step1 >> 1 & 1));
        sr.setNoUpdate(6,(step1 >> 2 & 1));
        sr.setNoUpdate(7,(step1 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step1 = 0;
    sr.setNoUpdate(4, 0);
    sr.setNoUpdate(5, 0);
    sr.setNoUpdate(6, 0);
    sr.setNoUpdate(7, 0);
    // sr.updateRegisters();
  }

  if (myStepper2.distanceToGo() != 0){

      myStepper2.run();
      if (myStepper2.StepReturn() != step2){
        step2 = myStepper2.StepReturn();
        sr.setNoUpdate(0,(step2 >> 0 & 1));
        sr.setNoUpdate(1,(step2 >> 1 & 1));
        sr.setNoUpdate(2,(step2 >> 2 & 1));
        sr.setNoUpdate(3,(step2 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step2 = 0;
    sr.setNoUpdate(0, 0);
    sr.setNoUpdate(1, 0);
    sr.setNoUpdate(2, 0);
    sr.setNoUpdate(3, 0);
    // sr.updateRegisters();
  }

    if (myStepper3.distanceToGo() != 0){
      myStepper3.run();
      if (myStepper3.StepReturn() != step3){
        step3 = myStepper3.StepReturn();
        sr.setNoUpdate(8,(step3 >> 0 & 1));
        sr.setNoUpdate(9,(step3 >> 1 & 1));
        sr.setNoUpdate(10,(step3 >> 2 & 1));
        sr.setNoUpdate(11,(step3 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step3 = 0;
    sr.setNoUpdate(8, 0);
    sr.setNoUpdate(9, 0);
    sr.setNoUpdate(10, 0);
    sr.setNoUpdate(11, 0);
    // sr.updateRegisters();
  }

    if (myStepper4.distanceToGo() != 0){
      myStepper4.run();
      if (myStepper4.StepReturn() != step4){
        step4 = myStepper4.StepReturn();
        sr.setNoUpdate(12,(step4 >> 0 & 1));
        sr.setNoUpdate(13,(step4 >> 1 & 1));
        sr.setNoUpdate(14,(step4 >> 2 & 1));
        sr.setNoUpdate(15,(step4 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step4 = 0;
    sr.setNoUpdate(12, 0);
    sr.setNoUpdate(13, 0);
    sr.setNoUpdate(14, 0);
    sr.setNoUpdate(15, 0);
    // sr.updateRegisters();
  }

    if (myStepper5.distanceToGo() != 0){
      myStepper5.run();
      if (myStepper5.StepReturn() != step5){
        step5 = myStepper5.StepReturn();
        sr.setNoUpdate(20,(step5 >> 0 & 1));
        sr.setNoUpdate(21,(step5 >> 1 & 1));
        sr.setNoUpdate(22,(step5 >> 2 & 1));
        sr.setNoUpdate(23,(step5 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step5 = 0;
    sr.setNoUpdate(20, 0);
    sr.setNoUpdate(21, 0);
    sr.setNoUpdate(22, 0);
    sr.setNoUpdate(23, 0);
    // sr.updateRegisters();
  }

      if (myStepper6.distanceToGo() != 0){
      myStepper6.run();
      if (myStepper6.StepReturn() != step6){
        step6 = myStepper6.StepReturn();
        sr.setNoUpdate(16,(step6 >> 0 & 1));
        sr.setNoUpdate(17,(step6 >> 1 & 1));
        sr.setNoUpdate(18,(step6 >> 2 & 1));
        sr.setNoUpdate(19,(step6 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step6 = 0;
    sr.setNoUpdate(16, 0);
    sr.setNoUpdate(17, 0);
    sr.setNoUpdate(18, 0);
    sr.setNoUpdate(19, 0);
    // sr.updateRegisters();
  }

      if (myStepper7.distanceToGo() != 0){
      myStepper7.run();
      if (myStepper7.StepReturn() != step7){
        step7 = myStepper7.StepReturn();
        sr.setNoUpdate(28,(step7 >> 0 & 1));
        sr.setNoUpdate(29,(step7 >> 1 & 1));
        sr.setNoUpdate(30,(step7 >> 2 & 1));
        sr.setNoUpdate(31,(step7 >> 3 & 1));
        // sr.updateRegisters();
      } 
    }
  else {
    step7 = 0;
    sr.setNoUpdate(28, 0);
    sr.setNoUpdate(29, 0);
    sr.setNoUpdate(20, 0);
    sr.setNoUpdate(31, 0);
    // sr.updateRegisters();
  }

  sr.updateRegisters();
  // vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL);
}

void Task_Data  ( void *pvParameters ){
  // Define Firebase Data object
  FirebaseData fbdo;
  FirebaseAuth auth;
  FirebaseConfig config;
  config.api_key = API_KEY;
  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h
  // Limit the size of response payload to be collected in FirebaseData
  fbdo.setBSSLBufferSize(4096 , 1024);
  fbdo.setResponseSize(2048);
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  char Like_str[8];
  vTaskDelay(pdMS_TO_TICKS(5000));
  // DynamicJsonDocument doc(2048);
  for(;;){
    
    // Firebase.ready() should be called repeatedly to handle authentication tasks.
    if (Firebase.ready() && Wifi_status == true)
    {
        String documentPath = "project_like_counter/device_001";
        if (!taskCompleted)
        {
            taskCompleted = true;

            // For the usage of FirebaseJson, see examples/FirebaseJson/BasicUsage/Create.ino
            FirebaseJson content;

            content.set("fields/like/integerValue", "0");
            content.set("fields/pageID/stringValue",PageID_Read);
            content.set("fields/state/stringValue","running");

            // info is the collection id, countries is the document id in collection info.
            

            Serial.print("Create document... ");

            if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw()))
                // Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
                Serial.printf("ok\n");
            else
                Serial.println(fbdo.errorReason());

            if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw(),"pageID"))
                Serial.printf("ok\n");
                // Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
            else
                Serial.println(fbdo.errorReason());
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        String mask = "";

        // If the document path contains space e.g. "a b c/d e f"
        // It should encode the space as %20 then the path will be "a%20b%20c/d%20e%20f"

        Serial.println("Get a document... ");

        if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), mask.c_str())){
            
            // Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());

            JsonDocument doc;
            deserializeJson(doc, fbdo.payload().c_str());
            int Like = doc["fields"]["like"]["integerValue"];
            xQueueSend(like_num, ( void * ) &Like, portMAX_DELAY);

            // String Name = doc["fields"]["pageID"]["stringValue"];
            // (String(Like)).toCharArray(Like_str,(String(Like)).length()+1);
            // Serial.print("Like = ");
            // Serial.println(Like);
            // Serial.print("Name = ");
            // Serial.println(Name);
            // Serial.println("----------------------------------------");

        }
        else
            Serial.println(fbdo.errorReason());
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    // test
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    Serial.print("Free heap: ");
    Serial.println(freeHeap);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

void running_ap (void *pvParameters){
  WiFi.softAP(ssid, password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());
  pinMode(output26, OUTPUT);
  digitalWrite(output26, LOW);
  server.begin();
  for(;;){
    running_ap();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


int writeStringToEEPROM(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);

  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
  
  return addrOffset + 1 + len;
}

int readStringFromEEPROM(int addrOffset, String *strToRead)
{
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];

  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0'; // !!! NOTE !!! Remove the space between the slash "/" and "0" (I've added a space because otherwise there is a display bug)

  *strToRead = String(data);
  return addrOffset + 1 + newStrLen;
}

void connect(){
  readStringFromEEPROM(0, &SSID_Read);
  readStringFromEEPROM(50, &Password_Read);
  readStringFromEEPROM(100, &PageID_Read);
  // // convert to char from string
  String(SSID_Read).toCharArray(WIFI_SSID,SSID_Read.length() + 1);
  String(Password_Read).toCharArray(WIFI_PASSWORD,Password_Read.length() + 1);
  //
  Serial.println("SSID == " + SSID_Read);
  Serial.println("Password == " + Password_Read);
  Serial.println("PageID == " + PageID_Read);
  // connect wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      vTaskDelay(pdMS_TO_TICKS(300));
      if (millis() - ms >= 10000){
        break;
        Serial.println("cant connect wifi");
        connected_wifi = 0;
      }
      else {
        connected_wifi = 1;
      }
  }
  if (Wifi_status == true && connected_wifi == 1){
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
  }
}


void running_ap(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            
            if (header.indexOf("GET /zero/on") >= 0) {
              Serial.println("GPIO 26 on");
              LED_state = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /zero/off") >= 0) {
              Serial.println("GPIO 26 off");
              LED_state = "off";
              digitalWrite(output26, LOW);
            }
            else {
              Serial.println("-----------------------");
              Serial.println(header.c_str());
              String SSID = header.substring(header.indexOf("SSID=") + 5 ,header.indexOf("&Password"));
              String Password = header.substring(header.indexOf("Password=") + 9,header.indexOf("&PageID"));
              String PageID = header.substring(header.indexOf("PageID=") + 7,header.indexOf(" HTTP"));
              
              SSID.replace("+"," ");

              Serial.print("SSID = ");
              Serial.println(SSID);
              Serial.print("Password = ");
              Serial.println(Password);
              Serial.print("PageID = ");
              Serial.println(PageID);
              
              int Offset = writeStringToEEPROM(0, SSID);  // 50 bytes
              Serial.println("SSID = " + SSID + "\n length = " + Offset);
              Offset = writeStringToEEPROM(50, Password); // 50 bytes
              Serial.println("Password = " + Password + "\n length = " + int(Offset - 50));
              Offset = writeStringToEEPROM(100, PageID); // 50 bytes
              Serial.println("PageID = " + PageID + "\n length = " + int(Offset - 50));
              EEPROM.commit();
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");  
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Testing ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO zero - State " + LED_state + "</p>");
            // If the LED_state is off, it displays the ON button       
            if (LED_state=="off") {
              client.println("<p><a href=\"/zero/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/zero/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            client.println("<form action=\"/page_like_counter\">");

            client.println("<label for=\"SSID\">SSID:</label>");
            client.println("<input type=\text\" id=\"SSID\" name=\"SSID\" placeholder=\"wifi name\"><br><br>");

            client.println("<label for=\"Password\">Password:</label>");
            client.println("<input type=\text\" id=\"Password\" name=\"Password\" placeholder=\"wifi password\"><br><br>");

            client.println("<label for=\"PageID\">PageID:</label>");
            client.println("<input type=\text\" id=\"PageID\" name=\"PageID\" placeholder=\"pageid\"><br><br>");

            client.println("<input type=\"submit\" value=\"Submit\">");
            client.println("</form>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}


