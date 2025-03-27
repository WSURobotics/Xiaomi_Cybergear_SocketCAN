#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h
#include <bits/stdc++.h>
#include <map>
#include <string>

// =======================================================================================================
// README - WIPE THE ESP32 BEFORE RESTARTING! OTHERWISE THE HIGH STRENGTH VALUES MAY SEND
//          MOTORS TO POSITIONS AT INCREDIBLY DANGEROUS SPEEDS.
//          WHENEVER POSSIBLE, SEND COMMAND '0' OVER SERIAL TO STOP MOTOR LOOP BEFORE END!
//          WIPE: HOLD BOOT BTN, PRESS EN BTN, RELEASE BOOT BTN
//                I ALSO DO IT THE OTHER WAY (hold EN, press Boot)
//                I ALSO DO BOTH WAYS MULTIPLE TIMES!
// =======================================================================================================
// How to add the custom 'all_motor_pos_to_zero()' function (only works for MODE_POSITION):
// 1) Copy the following
//      lib_deps = https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino
//    Navigate to the project > platformio.ini
//    Paste at the bottom of the file
//
// 2) Copy the following
//      void motor_pos_to_zero();
//    Navigate to the project > .pio > libdeps > Xiaomi_CyberGear_Arduino > xioami_cybergear_driver.h
//    Paste anywhere in the 'public' section of class XiaomiCyberGearDriver
//
// 3) Copy the following
//      void XiaomiCyberGearDriver::motor_pos_to_zero(){
//      uint8_t data[8] = {0x01};
//      _send_can_package(_cybergear_can_id, CMD_SET_MECH_POSITION_TO_ZERO,_master_can_id, 8, data);
//      }
//    Navigate to the project > .pio > libdeps > Xiaomi_CyberGear_Arduino > xioami_cybergear_driver.cpp
//    Paste at the very bottom of the file
// =======================================================================================================

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 4
#define TX_PIN 5

// Intervals:
#define TRANSMIT_RATE_MS 1000 // 30 for MODE_CURRENT
#define POLLING_RATE_MS 1000 // 100 for MODE_CURRENT

// Static globals
static bool initStall = false; // If we want an initial stall before starting the loop
static bool driver_installed = false; // Checks if the motor has been initialized
static bool loopStart = false; // Will run the walking commands themselves while true
static bool printStart = false; // Will print motor status while true
unsigned long previousMillis = 0;  // will store last time a message was send
static int timeSlice = 30; // The delay in void loop()

static int readCanID = 3; // ie 0 = read motor 1 (the motor to print status to serial)

// Define the CAN ID's for each motor
uint8_t CYBERGEAR_CAN_ID1 = 0x65;
uint8_t CYBERGEAR_CAN_ID2 = 0x66;
uint8_t CYBERGEAR_CAN_ID3 = 0x67;
uint8_t CYBERGEAR_CAN_ID4 = 0x68;
uint8_t CYBERGEAR_CAN_ID5 = 0x69;
uint8_t CYBERGEAR_CAN_ID6 = 0x6A;
uint8_t CYBERGEAR_CAN_ID7 = 0x71; // SWAPPED 7 FOR 13 - MOTOR IS NOW 13!
uint8_t CYBERGEAR_CAN_ID8 = 0x6C;
uint8_t CYBERGEAR_CAN_ID9 = 0x6D;
uint8_t CYBERGEAR_CAN_ID10 = 0x6E;
uint8_t CYBERGEAR_CAN_ID11 = 0x6F;
uint8_t CYBERGEAR_CAN_ID12 = 0x70;
uint8_t MASTER_CAN_ID = 0x00;

// array of all 12 cybergears
XiaomiCyberGearDriver gearArr[12] = {
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID1, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID2, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID3, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID4, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID5, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID6, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID7, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID8, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID9, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID10, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID11, MASTER_CAN_ID),
  XiaomiCyberGearDriver(CYBERGEAR_CAN_ID12, MASTER_CAN_ID)
};

// WORKING STEPS - MARCH (50 timeSlice)
// // The 'knee' values for a step, in order
// float kneeArray[22] = {
//   -1.12, -1.14, -1.00, -0.89, -0.89, -0.89, -0.89, -0.89, -0.89, -0.91, -0.92, -0.94, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.08, -1.09, -1.10
// };

// // The 'ankle' values for a step, in order
// float ankleArray[22] = {
//   1.61, 2.2, 2.2, 1.74, 1.74, 1.72, 1.70, 1.69, 1.69, 1.68, 1.68, 1.68, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.61, 1.62, 1.62
// };

// WORKING STEPS - WOBBLY (50 timeSlice)
// // The 'knee' values for a step, in order
// float kneeArray[22] = {
//   -1.12, -1.14, -1.00, -0.89, -0.89, -0.89, -0.89, -0.89, -0.89, -0.91, -0.92, -0.94, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.08, -1.09, -1.10
// };

// // The 'ankle' values for a step, in order
// float ankleArray[22] = {
//   1.61, 2.2, 2.2, 2.1, 1.94, 1.80, 1.70, 1.69, 1.69, 1.68, 1.68, 1.68, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.61, 1.62, 1.62
// };

// WORKING STEPS - FASTER (30 timeSlice)
// The 'knee' values for a step, in order
float kneeArray[22] = {
  -1.12, -1.00, -0.89, -0.89, -0.89, -0.89, -0.89, -0.89, -0.89, -0.91, -0.92, -0.94, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.07, -1.08, -1.09, -1.10
};

// The 'ankle' values for a step, in order
float ankleArray[22] = {
  1.61, 2.35, 2.35, 1.74, 1.74, 1.72, 1.70, 1.69, 1.69, 1.68, 1.68, 1.68, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.60, 1.61, 1.62, 1.62
};

// FL Polarity: 1
// FR Polarity: -1
// BL Polarity: 1
// BR Polarity: -1

// For setting walk positions in loop
static int group1 = 0; // Tracks index of group 1
static int group2 = 13; // Tracks index of group 2

// function declarations
static void initialize_all_motors(); // Initializes all motors
static void all_motor_pos_to_zero(); // Sets all motor positions to zero (MODE_POSITION ONLY)
static void normalize_stance(); // Brings motors to (current) default stance, assuming 0.0 is straight leg orientation
static void strengthen_motors(); // Strengthens motors to dangerously high speed (ie, 20 or 30) for walk strength
static void weaken_motors(); // Lowers motor speeds back down to safe value
static void handle_rx_message(twai_message_t& message); // Configures status messages
static void check_alerts(); // Checks for status messages

// Runs once when the code is re-pushed to microcontroller
void setup() {
  delay(1000);
  initialize_all_motors(); // Initialize all 12 cybergears
  delay(1000);
  all_motor_pos_to_zero(); // Set the current motor positions to zero
  delay(1000);
}

// Runs ongoing while the microntroller is powered
void loop() {
  if (!driver_installed) { // Checks for motor initialization
    delay(5000);
    Serial.printf("Driver not installed - bool is false");
    return;
  }

  if (printStart)
  {
    // Prints target motor status
    check_alerts(); // Read incoming messages
    XiaomiCyberGearStatus cybergear_status = gearArr[readCanID].get_status(); // Gets the status
    Serial.printf("Motor: POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
      previousMillis = currentMillis;
      gearArr[readCanID].request_status(); // Prompt for status
    }
  }

  // Controlled void loop() timeslice
  delay(timeSlice); // Controls the actual speed of the walking cycle (lower delay = faster the steps happen)

  // ==================================================================================
  // PERFORMS ONCE - INITIAL POSITIONS AND RAISES SPEED LIMIT
  // ==================================================================================
  if (!initStall)
  {
    Serial.println("Moving to default stance");
    delay(2000);
    // Set to a standing stance
    normalize_stance();

    delay(3000);
    Serial.println("Waiting for Serial Command '1' before starting walk cycle");
    initStall = true;
  }

  // ==================================================================================
  // START AT STRAIGHT LEG ORIENTATION IF EVER USING WALK CYCLE (all legs straight out)
  // ==================================================================================
  if (loopStart)
  {
    gearArr[10].set_position_ref(kneeArray[group1] * -1);
    gearArr[9].set_position_ref(ankleArray[group1] * -1);
    gearArr[1].set_position_ref(kneeArray[group1] * 1);
    gearArr[0].set_position_ref(ankleArray[group1] * 1);
    group1++;
    if (group1 == 22) // reset if at max
      group1 = 0;

    gearArr[7].set_position_ref(kneeArray[group2] * 1);
    gearArr[6].set_position_ref(ankleArray[group2] * 1);
    gearArr[4].set_position_ref(kneeArray[group2] * -1);
    gearArr[3].set_position_ref(ankleArray[group2] * -1);
    group2++;
    if (group2 == 22) // reset if at max
      group2 = 0;
  }

  // ==================================================================================
  // EDIT FOR SERIAL INPUT ACTIONS AS REQUIRED
  // ==================================================================================
  if (Serial.available()) {  // Check if data is available
        String command = Serial.readStringUntil('\n');  // Read input until newline
        command.trim();  // Remove any whitespace or newlines

        int commandID = -1;
        
        if (sscanf(command.c_str(), "%d", &commandID) == 1) {  // Parse input
            switch (commandID) {
              case 0:
                  Serial.println("CommandID 0 - Lazy Stop");
                  loopStart = false;
                  break;
              case 1:
                  Serial.println("CommandID 1 - Lazy Start");
                  loopStart = true;
                  break;
              case 2:
                  Serial.println("CommandID 2 - Printing Status Disabled");
                  printStart = false;
                  break;
              case 3:
                  Serial.println("CommandID 3 - Printing Status Enabled");
                  printStart = true;
                  break;
              case 4:
                  Serial.println("CommandID 4 - Decrease Step");
                  timeSlice -= 5;
                  break;
              case 5:
                  Serial.println("CommandID 5 - Increase Step");
                  timeSlice += 5;
                  break;
              case 6:
                  Serial.println("CommandID 6 - Normalizing Stance");
                  normalize_stance();
                  break;
              case 7: 
                  Serial.println("CommandID 7 - Strengthening Motors to 'dangerous speed'");
                  strengthen_motors();
                  break;
              case 8:
                Serial.println("CommandID8 - Weakening Motors back down");
                weaken_motors();
                break;
              default:
                  Serial.println("Invalid command ID");
                  break;
            }
        } else {
            Serial.println("Invalid input format");
        }
    }
}


// Initializes all motors
static void initialize_all_motors()
{
  gearArr[0].init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true); // once for any cybergear
  //gearArr[0].init_can("can0", /*serial_debug=*/true);


  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} init\n", i);
    gearArr[i].init_motor(MODE_POSITION); 
    gearArr[i].set_limit_speed(1.0f); /* set the maximum speed of the motor */
    gearArr[i].set_limit_current(12.0f); /* current limit allows faster operation */
    gearArr[i].set_limit_torque(12.0f);
    gearArr[i].enable_motor(); /* turn on the motor */
  }
  driver_installed = true;
}

// Sets all motor current positions to zero
static void all_motor_pos_to_zero()
{
  for (int i = 0; i < 12; i++)
  {
    Serial.printf("cybergear{%d} zero pos\n", i);
    // Set current position to zero, and then send motor to new zero (otherwise it will spin!)
    gearArr[i].motor_pos_to_zero();
    gearArr[i].set_position_ref(0.0f);
    delay(50);
  }
  Serial.println("All motor positions set.");
}

// Sets the legs to the default stance, assuming motors have been zeroed at the straight orientation
static void normalize_stance()
{
  group1 = 0;
  group2 = 11;

  gearArr[10].set_position_ref(kneeArray[0] * -1); 
  gearArr[9].set_position_ref(ankleArray[0] * -1);
  gearArr[7].set_position_ref(kneeArray[11] * 1);
  gearArr[6].set_position_ref(ankleArray[11] * 1);
  gearArr[4].set_position_ref(kneeArray[11] * -1);
  gearArr[3].set_position_ref(ankleArray[11] * -1);
  gearArr[1].set_position_ref(kneeArray[0] * 1);
  gearArr[0].set_position_ref(ankleArray[0] * 1);
}

// Initializes motor speed values to very high value to give it enough strength to hold position when walking
static void strengthen_motors()
{
  Serial.println("Raising speed limit - strength increased");
    for (int i = 0; i < 12; i++)
    {
      gearArr[i].set_limit_speed(30.0f);
    }
}

// Initializes motor speed values to lower, safer value
static void weaken_motors()
{
  Serial.println("Lowering speed limit - strength decreased");
    for (int i = 0; i < 12; i++)
    {
      gearArr[i].set_limit_speed(1.0f);
    }
}

// The following functions came with the Xioami Cybergear Library - only for motor error/status communications

// Processes error message to be read in Serial (only set up for a single targeted motor)
static void handle_rx_message(twai_message_t& message) {
  if (((message.identifier & 0xFF00) >> 8) == gearArr[readCanID].get_motor_can_id()){
    gearArr[readCanID].process_message(message);
  }
}

// Checks for error messages within the POLLING_RATE_MS timeslice
static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
    Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN)
  {
    Serial.println("Alert: Error counters exceeded limit!");
  }
  if (alerts_triggered & TWAI_ALERT_ERR_ACTIVE)
  {
    Serial.println("Alert: Error-active!");
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}
