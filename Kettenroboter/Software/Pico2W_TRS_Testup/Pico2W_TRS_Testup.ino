#include <Wire.h>
//#include <FreeRTOS.h>
//#include "MySoftwareWire.h"
#include "board_pins.h"
bool core1_separate_stack = true;
// enable FreeRTOS SMP with Tools: Operating System: FreeRTOS SMP
// compile with acli compile -j 0 -v -e -b rp2040:rp2040:rpipico2w
// (in windows compile pio asm with pioasm.exe pio_code.pio pio_code.pio.h
// find port: sudo dmesg | egrep -i --color 'ttyACM'
// arduino-cli upload I2Csketch.ino -p /dev/ttyACM0 -b rp2040:rp2040:rpipico2w
// serial: sudo screen /dev/ttyACM0 115200          (ctrl-d to detach)

// ========================= MOTOR A/B CONFIG =========================
#include "motor_config.h"

// ========================= MD03 CONFIG flipper 1 =========================
#include "md03_config.h"

/*
volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;
volatile long encoderCount1B = 0;
volatile long encoderCount2B = 0;
*/
/*
long targetAngle = 0;
bool homed = false;
const float countsPerDegree = 24600.0 / 360.0; // 68,3333333333333
long homePosition1 = 0;
long homePosition2 = 0;
*/
#define ENCODER_COUNTS_FULL_ROT 24600.0
#define COUNTS_PER_DEGREE (ENCODER_COUNTS_FULL_ROT / 360.0)

// ========================= SOFTWARE I2C SRF02 =========================
//MySoftwareWire sw(10, 11);
const byte frontSensors[] = {0x78, 0x76, 0x7E};
const byte backSensors[] = {0x70, 0x71};
const byte Sensors[] = {0x78, 0x76, 0x7E, 0x70, 0x71};

// ========================= FLIPPER CONTROL =========================
//#include "flipper_control.h"

// ========================= I2C HELPERS =========================
#include "i2c_helpers.h"

// ========================= SRF02 READ =========================
//#include "srf02.h"

// ========================= PIO-QEI =========================
#include "hardware/pio.h"
#include "pio_qei.h"
const int pio_pin = 15;
const float pio_freq = 2500;


// ========================= MCP2515-CAN =========================

#include <Adafruit_MCP2515.h>
#include "can_db.h"
#ifdef ARDUINO_ADAFRUIT_FEATHER_RP2040
   #define ADA_CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define ADA_CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define ADA_CS_PIN    20
#else
    // Anything else, defaults!
   #define ADA_CS_PIN    5
#endif
#define AFAIK_CS_PIN 20
// Set CAN bus baud rate
#define CAN_BAUDRATE (500000)
Adafruit_MCP2515 mcp(AFAIK_CS_PIN); //, 19, 16, 18);
//*/

// ========================= FLAGS =========================
#include "flags.h"
uint32_t c1_flags;

//*
// ========================= Postbox / Pico-SDK Mutex =========================
//#include <FreeRTOS.h>
//#include "semphr.h"
#include "pico/mutex.h"
#include "icc.h"
#define POSTBOXSIZE 32

mutex_t  pb_mutex; 
uint32_t pb_owner;
volatile bool newcontent = false;
volatile uint32_t postbox[POSTBOXSIZE] = {0};
volatile long ct_pb = 0;
volatile byte motor_pb[2][7] = {0};
volatile byte flipper_pb[2][7] = {0};
volatile uint32_t flags = 0;

volatile bool i2c_flag = false;
volatile bool i2c_timeout = false;

mutex_t serialmutex; 
uint32_t serial_owner;

//*

// ========================= Pico-SDK Queue =========================
#include "pico/util/queue.h"
typedef struct {
    byte values[4][7] = {0};
} MotorData;

MotorData C1_Motor_Data;
MotorData C0_Motor_Data;

queue_t motor_queue;

// ========================= SETUP =========================
bool success;
byte error = 10;
unsigned long c0_lastExecutedMillis, c1_lastExecutedMillis = 0;
unsigned long c0_currentMillis, c1_currentMillis = 10; //millis();
uint8_t swcase = 0;
byte motor_add[] = {MOTOR_A_ADDR, MOTOR_B_ADDR, MD03_ADDR2, MD03_ADDR};
byte regs[] = {DIR_REG, STATUS_REG, SPEED_REG, ACCEL_REG, TEMP_REG, CURR_REG, VERSION_REG};
byte motor = 0;
bool can_available = false;
bool printvalues = false;
//message_t message = stop;
char serialread;

unsigned long c0_current_micros, c1_current_micros, c0_cycletime, c1_cycletime, ct_buf = 0;

byte motor_c0_readvalues[4][7] = {0};
byte motor_c1_readvalues[4][7] = {0};

bool sensor_presence[5] = {false};

// SINGLECORE-MODE (runs both loops sequentially on core0)
const bool singlecore = true;

const bool request_queue_output = false;

bool test;


void setup() {

    // Serial Setup
    Serial.begin(1000000);
    while(!Serial){delay(10);}
    Serial.println("Serial Ready ----------------------   ");

    //Mutex für postbox
    initMutex(&pb_mutex);
    /*
    if(&pb_mutex != NULL){
        Serial.println("Mutex created postbox up and working ");
    }else{
        Serial.println("!Error: Could not create Mutex!");
    }
    */
    if(mutexTake(&pb_mutex, &pb_owner)){
        Serial.println("Mutex created postbox up and working ");
        delay(1);
        mutexGive(&pb_mutex);
    }

    // PIO-Setup
    if(pio_qei_setup(15, 2500)){
        Serial.println(" PIO-Setup successfull");
    }else{
        Serial.println(" ! PIO-ERROR ! ");
    };

    // Initialize a queue to hold up to 10 MotorData structs
    queue_init(&motor_queue, sizeof(MotorData), 10);

    Serial.println("Setup on core0 finished");
    Serial.print(" MCU running at "); Serial.print(rp2040.f_cpu()/1000); Serial.println(" kHz");

    //Serial.println(" Running MCP2515 Sender test!");

    ///*
    //CAN.setSPIFrequency(frequency);
    SPI.setSCK(18);
    SPI.setRX(16); // MISO
    SPI.setTX(19); // MOSI
    SPI.begin();
    if (mcp.begin(CAN_BAUDRATE) != 0) {
        Serial.println("MCP2515 chip found");
        can_available = true;
    }else{
        Serial.println("Error initializing MCP2515.");
        can_available = false;
    }
    //*/

    // Sensor Bus Initialisierung
    if(Wire1.setSCL(3) && Wire1.setSDA(2)){ //WireX.serXXX returns 1 / true if successful
        Serial.println("starting Sensor-I2C");
        Wire1.begin();
        for(int i = 0; i<5; i++){
            if(wireCheckPresence(Wire1, Sensors[i])){
                sensor_presence[i] = true;
                Serial.print(" | Found device on Wire1 : 0x"); Serial.print(Sensors[i], HEX);
            }else{
                Serial.println(); Serial.print("ERROR: Wire1 device 0x "); Serial.print(Sensors[i], HEX); Serial.println(" not showing ");
            }
        } 
        Serial.println();       
    }else{
        Serial.println("Could not start Sensor-I2C");
    }
    
   if(singlecore==true){
        //rp2040.idleOtherCore();
        setupCore1(); 
        Serial.println("Program running on Core0 only (Singlecore)");
    }else{
        Serial.println("Program running on both Cores (Dualcore)");
    }
}

void setupCore1(){ // Core1: Motor control, QEI & Control loops
    pinMode(LED_BUILTIN, OUTPUT);

    /*
    success = Wire.setSCL(1) && Wire.setSDA(0);
    */
    //digitalWrite(LED_BUILTIN, Wire.setSCL(1) && Wire.setSDA(0));
    // Initialisierung Motor Bus
    if(Wire.setSCL(1) && Wire.setSDA(0)){
        Wire.setTimeout(10, true);
        Wire.setClock(1000);
    }
    //*/
    if(singlecore == true){Serial.println("starting Motor-I2C");}
    Wire.begin();
    for(int j=0; j<4; j++){
        if(wireCheckPresence(Wire, motor_add[j])){
                if(singlecore == true){Serial.print(" | Found device on Wire0 : 0x"); Serial.print(Sensors[j], HEX);}
            }else{
                if(singlecore == true){Serial.println(); Serial.print("ERROR: Wire0 device 0x "); Serial.print(Sensors[j], HEX); Serial.println(" not showing ");}
            }
    }
    if(singlecore == true){Serial.println();}

    

    digitalWrite(LED_BUILTIN, LOW);
}

void setup1(){ if(singlecore==false){setupCore1();}}

// ========================= LOOP =========================
void loop() {

    c0_currentMillis = millis();
    if (c0_currentMillis - c0_lastExecutedMillis >= 471) 
    {
    c0_lastExecutedMillis = c0_currentMillis; // save the last executed time
    c0_current_micros = micros();

    //test=!test;
    //if(test=true){rp2040.idleOtherCore();}else{rp2040.resumeOtherCore();}

    Serial.println("    Ping from Core0");
    Serial.print(" core0 cycle time: [ c0_CT: "); Serial.print(c0_cycletime); Serial.print(" ] microseconds,");
    Serial.print(" core1 cycle time: [ c1_CT: "); Serial.print(ct_pb); Serial.println(" ] microseconds");

    if(false){//mutexTake(&pb_mutex, &pb_owner)){
        Serial.println(" Mutex availabe for Core0, copy OP started");
        if(newcontent == true){
            newcontent = false;
            Serial.println(" new content available:");

            // Kopiert die 28 Bytes aus der Postbox zurück in das lokale 2D-Array
            memcpy(motor_c0_readvalues, (void*)&postbox[0], sizeof(motor_c0_readvalues));
            
            printvalues = true;
        }else{
            Serial.println("! no new content available :(");
            printvalues = false;
        }
        ct_buf = ct_pb;

        if(i2c_flag==true){Serial.println("! I2C-Error");}
        if(i2c_timeout==true){Serial.println("! I2C-Time-oout !");}

        mutexGive(&pb_mutex);

        if(printvalues==true){

        for(int i=0; i<2; i++){
            Serial.print("   -> Data from Motor "); Serial.print(i); Serial.println(": "); Serial.print(" { ");
            for(int ii=0; ii<7; ii++){
                Serial.print(" [ Value "); Serial.print(ii); Serial.print(": "); Serial.print(motor_c0_readvalues[i][ii]); Serial.print(" ] ");
            }
            Serial.println(" } ");
        }
        printvalues = false;
        
        }
    }else{
        //Serial.println("<- Mutex taken ");
    }

    // Queue

    Serial.println("QUEUE-OUTPUT:");
    if (queue_try_remove(&motor_queue, &C0_Motor_Data) && request_queue_output) {
        // Access the copied array data directly
        for(int i=0; i<2; i++){
            Serial.print("   -> Data from Motor "); Serial.print(i); Serial.println(": "); Serial.print(" { ");
            for(int ii=0; ii<7; ii++){
                Serial.print(" [ Value "); Serial.print(ii); Serial.print(": "); Serial.print(C0_Motor_Data.values[i][ii]); Serial.print(" ] ");
            }
            Serial.println(" } ");
        }
    }else{
        Serial.println("QUEUE: no output");
    }




    
    // CAN-Comms:
    if(can_available == true){
        Serial.print("CAN-MCP-Test:");
        Serial.println("Test Message ");
        mcp.beginPacket(CAN_PING);
        mcp.write(0x5);
        mcp.endPacket();
        /*
        Serial.println("CAN-MCP-Test:");
        mcp.beginPacket(0x65);
        mcp.write('h');
        mcp.write('e');
        mcp.write('l');
        mcp.write('l');
        mcp.write('o');
        mcp.endPacket();

        mcp.beginPacket(CAN_MOTOR_A);
        for(int i=0; i>=6; i++){
            mcp.write(motor_c0_readvalues[0][i]);
        }
        mcp.endPacket();
        */
    }

    // Serial Input
    Serial.println("SERIAL INPUT:");
    for(int i=Serial.available(); i>=0; i--){
        serialread = Serial.read();
        Serial.print(" Recieved char: "); Serial.print(serialread); Serial.print(" "); Serial.print((uint8_t)serialread); Serial.print(" ");
        if(((uint8_t)serialread < 48) || ((uint8_t)serialread) > 126){
            Serial.println(" No accepted character");
            }else{
            switch (serialread){  //original f b l r s
                case 'f':
                    Serial.println("recieved command f");
                    //message = forward;
                    //rp2040.fifo.push_nb(message);
                    break;
                case 'b':
                    Serial.println("recieved command b");
                    break;
                case 'l':
                    Serial.println("recieved command l");
                    break;
                case 'r':
                    Serial.println("recieved command r");
                    break;
                case 's':
                    Serial.println("recieved command s");
                    break;
                case 0x0:
                    Serial.println("(No Input)");
                default:
                    Serial.println("! Serial Read ERROR, could not parse");
                    break;
            }
        }
    }


    Serial.println("loop0 finished");
    //rp2040.resumeOtherCore();
    c0_cycletime = micros() - c0_current_micros; 
    }
    if(singlecore==true){loopCore1();}
}

void loopCore1(){ // Core1: Motor control, QEI & Control loops
    delay(360);
    c1_currentMillis = millis();
    if ((c1_currentMillis - c1_lastExecutedMillis >= 300)  || (singlecore==true) /* */) {
    c1_lastExecutedMillis = c1_currentMillis; // save the last executed time
    c1_current_micros = micros();

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    for(int j=0; j<4; j++){
    
    //Wire.begin();
    for(int jj=0; jj<7; jj++){
    Wire.beginTransmission(motor_add[j]);
    delay(1);
    Wire.write(regs[jj]);
    Wire.endTransmission(true);
    error = false;
    // Wenn der Bus sich aufhängt oder der Slave NACKt
            if(error != 0){
                Wire.end();        // Hardware-I2C deinitialisieren
                delay(100);
                Wire.begin();      // Hardware-I2C neu starten
                //Wire.setTimeout(100); 
                Wire.setTimeout(10, true);
                delay(200);
                i2c_flag = true;
                continue;          // Versuch für dieses Register überspringen
            }
    /*Serial.print(error);
    switch (error) 
    {case 1: Serial.println("Success"); break; case 2: Serial.println("Data too long"); break; case 3: Serial.println("NACK on transmit of address"); break; case 4: Serial.println("NACK on transmit of data"); break; case 5: Serial.println("Timeout"); break;}
    */
    Wire.requestFrom(motor_add[j], 1); //Vollständige Transaktion kein begin / end nötig

    if(Wire.available()){
        motor_c1_readvalues[j][jj] = Wire.read();
    }
    if(jj>=6){
        Wire.endTransmission(true);
    }else{
        Wire.endTransmission(false);
    }
    }
    //Wire.end();
    }
    
    // Queue-Exchange:
    for(int j=0; j<2; j++){
        for(int jj=0; jj<7; jj++){
            C1_Motor_Data.values[j][jj] = motor_c1_readvalues[j][jj];
        }
    }
    // Non-blocking add to the queue
    queue_try_add(&motor_queue, &C1_Motor_Data); 


    // Postbox-Exchange:
    // Mutex sperren (z. B. xSemaphoreTake / mutex_enter_blocking)
    if(mutexTake(&pb_mutex, &pb_owner)){
    // Kopiert die 28 Bytes ab dem gewünschten START_INDEX in die Postbox
    memcpy((void*)&postbox[0], motor_c1_readvalues, sizeof(motor_c1_readvalues));

    ct_pb = c1_cycletime;

    newcontent = true;
    if(!(error != 0)){
        i2c_flag = false;
    }else{
        i2c_flag = true;
    }

    /*
    if(Wire.getTimeoutFlag()){
        Wire.clearTimeoutFlag();
        i2c_timeout = true;
    }else{
        i2c_timeout = false;
    }
    */

    // Mutex entsperren (z. B. xSemaphoreGive / mutex_exit)
    mutexGive(&pb_mutex);
    }
    
    c1_cycletime = micros() - c1_current_micros;
    ///*
    }
    //*/
}

void loop1(){ if(singlecore==false){loopCore1();}}