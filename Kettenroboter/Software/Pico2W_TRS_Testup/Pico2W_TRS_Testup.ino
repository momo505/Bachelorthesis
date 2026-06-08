#include <SPI.h>
#include <Wire.h>

bool core1_separate_stack = true;
// (in windows compile pio asm with pioasm.exe pio_code.pio pio_code.pio.h
// find port: sudo dmesg | egrep -i --color 'ttyACM'
// arduino-cli upload I2Csketch.ino -p /dev/ttyACM0 -b rp2040:rp2040:rpipico2w
// serial: sudo screen /dev/ttyACM0 115200          (ctrl-d to detach)

// ========================= Pinout =========================
#include "board_pins.h"

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
/*
#include "hardware/pio.h"
#include "pio_qei.h"
const int pio_pin = 15;
const float pio_freq = 2500;
*/

// ========================= Pico-QEI =========================
// https://github.com/pmarques-dev/PicoEncoder
#include <PicoEncoder.h>
PicoEncoder Encoder_LD;
PicoEncoder Encoder_RD;

// ========================= PID =========================
#include "PID.h"
#include "PID_config.h"
PID LD_Controller(false);
PID RD_Controller(false);

byte float_to_byte(float in) {
    // Clamp between 0 and 255
    if (in < 0) return 0;
    if (in > 255) return 255;
    return (byte)in;
}


// ========================= MCP2515-CAN =========================

#include "can_db.h"


// ========================= FLAGS =========================
#include "flags.h"
uint32_t c1_flags;

// ========================= CoreToCore-FIFO =========================

typedef enum{
    stop,               // Hard Stop Befehl 
    forward,            // vorwärts
    backward,           // rückwärts
    left, right,        // links bzw rechts
    none,               // leer Befehl noch nicht zugewiesen
    manual,             // manueller Modus vorerst als langsames vorwärts definiert
    test_1,
    test_2,
    test_3,
    set_flag_and,
    set_flag_xor,
    set_motor_speed,    // bedeutet nächster Wert ist enthät Motor-Geschwingkeits-Werte
    set_speeds,         // nächster Wert  enthält uint32 mit neuen Geschwindigkeitsvorgaben für PID-Regler (noch implementieren)
    motors_connected    // Statusmeldung ob alle Motoren an I2C-Bus detektiert (noch implementieren)
} command_t;

typedef union { 
    uint32_t uint32;    
    command_t command;
    } fifo_command_t;

const char input_chars[] = 		    {'s',   'f',        'b',        'l',    'r',    'n',    'm',    '1',    '2',    '3'}; //original f b l r s
const command_t char_meaning[] = 	{stop,  forward,    backward,   left,   right,  none,   manual, test_1, test_2, test_3};


//const fifo_command_t char_dict[];


const /*union*/ fifo_command_t char_dict[] = {{.command = stop},{.command = forward},{.command = backward},{.command = left},{.command = right},{.command = none},{.command = manual},{.command = test_1},{.command = test_2}
};






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

command_t curr_command;

//*

// ========================= Pico-SDK Queue =========================
#include "pico/util/queue.h"
typedef struct {
    byte values[4][7] = {0};
} MotorData;

MotorData C1_Motor_Data;
MotorData C0_Motor_Data;

queue_t motor_queue;

// ========================= MySerial ========================= 
#include "my_serial.h"
MySerialUSB MySerial(1); //legt fest welcher Core in serial schreiben kann

// ========================= CONFIG =========================
// SINGLECORE-MODE (runs both loops sequentially on core0)
const bool singlecore = false;

const bool debug = true;

const bool request_queue_output = true;

// Serielle Schnittstelle (choose which Core the seriel port gets connected to)
//const bool core0_serial = true;


// ========================= SETUP =========================
//Flagge auf Core 1
uint32_t c1_control_flag = (C1_MOVEMENT_ALLOWED); // | C1_GATHER_MOTORDATA);

uint rd_step, ld_step; // Steps des linken & Rechten Antriebsmotors
int r_position, l_position;
byte ld_speed_target = 0xA0;
byte rd_speed_target = 0xA0; //0;
byte ld_speed_output, rd_speed_output = 0;
byte rd_curr_speed, ld_curr_speed = 0;

bool success;
byte error = 10;
unsigned long c0_lastExecutedMillis, c1_lastExecutedMillis = 0;
unsigned long c0_currentMillis, c1_currentMillis = 10; // millis();
long unsigned int controller_timestamp = 1000;
long unsigned int controller_last_timestamp = 0;

uint8_t swcase = 0;
byte motor_add[] = {MOTOR_L_ADDR, MOTOR_R_ADDR, MD03_ADDR2, MD03_ADDR};
byte regs[] = {DIR_REG, STATUS_REG, SPEED_REG, ACCEL_REG, TEMP_REG, CURR_REG, VERSION_REG};
byte motor = 0;
bool can_available = false;
bool printvalues = false;
//message_t message = stop;
bool motor_connected = false;
char serialread;

unsigned long c0_current_micros, c1_current_micros, c0_cycletime, c1_cycletime, ct_buf = 0;

byte motor_c0_readvalues[4][7] = {0};
byte motor_c1_readvalues[4][7] = {0};

bool sensor_presence[5] = {false};

bool test;

fifo_command_t testcommand = {.command = manual};

// ========================= SPECIAL_COMMAND_HANDLER =========================
bool special_command_handler(uint32_t input){
    fifo_command_t read;
    read.uint32 = input;
    if(read.uint32 > 4){
        switch(input){
            case test_1:
                wireWrite2Regs(Wire, MOTOR_R_ADDR, DIR_REG , 0x01, SPEED_REG, 0xFF);
                wireWrite2Regs(Wire, MOTOR_L_ADDR, DIR_REG , 0x01, SPEED_REG, 0xFF);
                ld_speed_target = 0xFF;
                ld_speed_target = 0xFF;
                break;
            case test_2:
                wireWrite2Regs(Wire, MOTOR_R_ADDR, DIR_REG , 0x01, SPEED_REG, 0x00);
                wireWrite2Regs(Wire, MOTOR_L_ADDR, DIR_REG , 0x01, SPEED_REG, 0x00);
                ld_speed_target = 0x00;
                ld_speed_target = 0x00;
                break;
            case test_3:
                MySerial.core = ((MySerial.core +1)%2);
                break;
            default:
                break;
            }
        return true;
    }else{
        return false;
    }
}

void setup() {
    // Serial Setup
    MySerial.begin(1000000);
    while(!Serial){delay(10);}
    MySerial.println("Serial Ready ----------------------   ");

    //Mutex für postbox
    initMutex(&pb_mutex);
    /*
    if(&pb_mutex != NULL){
        MySerial.println("Mutex created postbox up and working ");
    }else{
        MySerial.println("!Error: Could not create Mutex!");
    }
    */
    if(mutexTake(&pb_mutex, &pb_owner)){
        MySerial.println("Mutex created postbox up and working ");
        delay(1);
        mutexGive(&pb_mutex);
    }
    
    /* PIO-Setup 
    if(pio_qei_setup(15, 2500)){
        MySerial.println(" PIO-Setup successfull");
    }else{
        MySerial.println(" ! PIO-ERROR ! ");
    };
    */

    // Initialize a queue to hold up to 10 MotorData structs
    queue_init(&motor_queue, sizeof(MotorData), 10);

    MySerial.println("Setup on core0 finished");
    MySerial.print(" MCU running at "); MySerial.print(rp2040.f_cpu()/1000); MySerial.println(" kHz");

    // Sensor Bus Initialisierung
    if(Wire1.setSCL(I2C1_SCL) && Wire1.setSDA(I2C1_SDA)){ //WireX.serXXX returns 1 / true if successful
        MySerial.println("starting Sensor-I2C");
        Wire1.begin();
        for(int i = 0; i<5; i++){
            if(wireCheckPresence(Wire1, Sensors[i])){
                sensor_presence[i] = true;
                MySerial.print(" | Found device on Wire1 : 0x"); MySerial.print(Sensors[i], HEX);
            }else{
                MySerial.println(); MySerial.print("ERROR: Wire1 device 0x "); MySerial.print(Sensors[i], HEX); MySerial.println(" not showing ");
            }
        } 
        MySerial.println();       
    }else{
        MySerial.println("Could not start Sensor-I2C");
    }
    
   if(singlecore==true){
        //rp2040.idleOtherCore();
        setupCore1(); 
        MySerial.println("Program running on Core0 only (Singlecore)");
    }else{
        MySerial.println("Program running on both Cores (Dualcore)");
    }
}

void setupCore1(){ // Core1: Motor control, QEI & Control loops
    pinMode(LED_BUILTIN, OUTPUT);

    // Serial Setup
    MySerial.begin(1000000);
    while(!Serial){delay(10);}
    MySerial.println("Serial Ready ----------------------   ");

    
    if(Wire.setSCL(I2C0_SCL) && Wire.setSDA(I2C0_SDA)){MySerial.println("starting Motor-I2C");}
    Wire.begin();
    for(int j=0; j<4; j++){
        if(wireCheckPresence(Wire, motor_add[j])){
            MySerial.print(" | Found device on Wire0 : 0x"); MySerial.print(Sensors[j], HEX);
            motor_connected = true;
        }else{
            MySerial.println(); MySerial.print("ERROR: Wire0 device 0x "); MySerial.print(Sensors[j], HEX); MySerial.println(" not showing ");
            motor_connected = false;
        }
    }

    // Pico-QEI Setup
    Encoder_LD.begin(QEI0_A);
    Encoder_RD.begin(QEI1_A);

    // PID Setup
    LD_Controller.set_parameters(Kp, Ki, Kd);
    LD_Controller.set_parameters(Kp, Ki, Kd);
    //LD_Controller.calculate_out(float e_new, float timediff);
    //LD_Controller.calculate_out(float e_new, float timediff);

    if(singlecore == true){MySerial.println();}

    digitalWrite(LED_BUILTIN, LOW);
}

void setup1(){ if(singlecore==false){setupCore1();}}

// ========================= LOOP =========================
void loop() {
    c0_currentMillis = millis();
    if (c0_currentMillis - c0_lastExecutedMillis >= 471){
    c0_lastExecutedMillis = c0_currentMillis; // save the last executed time
    c0_current_micros = micros();

    MySerial.println("    Ping from Core0");
    MySerial.print(" core0 cycle time: [ c0_CT: "); MySerial.print(c0_cycletime); MySerial.print(" ] microseconds,");
    MySerial.print(" core1 cycle time: [ c1_CT: "); MySerial.print(ct_pb); MySerial.println(" ] microseconds");

    if(false){//mutexTake(&pb_mutex, &pb_owner)){
        MySerial.println(" Mutex availabe for Core0, copy OP started");
        if(newcontent == true){
            newcontent = false;
            MySerial.println(" new content available:");

            // Kopiert die 28 Bytes aus der Postbox zurück in das lokale 2D-Array
            memcpy(motor_c0_readvalues, (void*)&postbox[0], sizeof(motor_c0_readvalues));
            
            printvalues = true;
        }else{
            MySerial.println("! no new content available :(");
            printvalues = false;
        }
        ct_buf = ct_pb;

        if(i2c_flag==true){MySerial.println("! I2C-Error");}
        if(i2c_timeout==true){MySerial.println("! I2C-Time-oout !");}

        mutexGive(&pb_mutex);

        if(printvalues==true){

        for(int i=0; i<2; i++){
            MySerial.print("   -> Data from Motor "); MySerial.print(i); MySerial.println(": "); MySerial.print(" { ");
            for(int ii=0; ii<7; ii++){
                MySerial.print(" [ Value "); MySerial.print(ii); MySerial.print(": "); MySerial.print(motor_c0_readvalues[i][ii]); MySerial.print(" ] ");
            }
            MySerial.println(" } ");
        }
        printvalues = false;
        
        }
    }else{
        //MySerial.println("<- Mutex taken ");
    }

    // Queue

    MySerial.println("QUEUE-OUTPUT:");
    if (queue_try_remove(&motor_queue, &C0_Motor_Data) && request_queue_output) {
        // Access the copied array data directly
        for(int i=0; i<2; i++){
            MySerial.print("   -> Data from Motor "); MySerial.print(i); MySerial.println(": "); MySerial.print(" { ");
            for(int ii=0; ii<7; ii++){
                MySerial.print(" [ Value "); MySerial.print(ii); MySerial.print(": "); MySerial.print(C0_Motor_Data.values[i][ii]); MySerial.print(" ] ");
            }
            MySerial.println(" } ");
        }
    }else{
        MySerial.println("QUEUE: no output");
    }

    // Serial Input
    MySerial.println("MySerial INPUT:");
    while (MySerial.available() > 0) {
        int serialread = MySerial.read();
    
        // Prüfe auf gültigen Bereich
        if (serialread < 48 || serialread > 126) {
            MySerial.print(" No accepted character: "); MySerial.println(serialread);
            continue;
        }

        bool found = false;
        for (size_t ii = 0; ii < (sizeof(input_chars) / sizeof(char)); ii++) {
            if ((char)serialread == input_chars[ii]) {
                MySerial.print("recieved command "); 
                MySerial.println(input_chars[ii]);
                if(rp2040.fifo.push_nb(char_meaning[ii])){
                    MySerial.println("Successfully pushed to FIFO");
                }
                found = true;
                break; // Suche abbrechen, wenn gefunden
            }
        }

        if (!found) {
            switch(serialread){
                case 'x':
                    rp2040.reboot();
                    break;
                default:
                    break;
            }
        }else{
            MySerial.print("! Serial Read ERROR: "); 
            MySerial.print((char)serialread);
            MySerial.println(" could not parse !");
        }
    
    }
    // Test:
    rp2040.fifo.push_nb(testcommand.uint32);
    // Zu testen:
    MySerial.print(" FEEDBACK pops available:"); MySerial.println(rp2040.fifo.available());
    while(rp2040.fifo.available()>0){
        union {
            uint32_t uint32;
            command_t command;
        } recieved2;

        //command_t recieved;
        rp2040.fifo.pop_nb(&recieved2.uint32);
        // Test:
        MySerial.println(" Commmand-Feedback-Loop:");
        switch (recieved2.command){
            case forward:
                MySerial.println("acknowledged command forward");
                break;
            case backward:
            MySerial.println("acknowledged command backward");
                break;
            default:
                //MySerial.print("ERROR, not understood "); MySerial.print(recieved2.command); MySerial.print(" , "); MySerial.println(recieved2.uint32);
                break;
            }
    }

    MySerial.println("loop0 finished");

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

    // Read newest flag values 
    if(c1_control_flag || 0xf /*checkt alle unter 8 bits*/){
        // Notstoproutine (noch zu ergänzen)
    }

    //
    controller_last_timestamp = controller_timestamp;  
    controller_timestamp = millis();


    ld_curr_speed = ld_step;
    //Pico-QEI:
    MySerial.print("Pico-QEI:");
    Encoder_LD.update();
    Encoder_RD.update();
    ld_step = Encoder_LD.step;
    rd_step = Encoder_RD.step;
    MySerial.println(" ");
    MySerial.print(" LD Position: ");
    MySerial.print(Encoder_LD.position);
    MySerial.print(", LD step: ");
    MySerial.print(Encoder_LD.step);
    MySerial.print(", RD Position: ");
    MySerial.print(Encoder_RD.position);
    MySerial.print(", RD step: ");
    MySerial.println(Encoder_RD.step);

    ld_curr_speed -= ld_step;
    ld_curr_speed = ld_curr_speed / (controller_timestamp - controller_last_timestamp);
    

    ///*
    // Read Command from FIFO
    //MySerial.println(" >>>>>>>||| FIFO-CHECK ||| <<<<<<<");
    if(rp2040.fifo.available() > 0){
        fifo_command_t recieved;

        //uint32_t* pointer;

        //command_t recieved;
        rp2040.fifo.pop_nb(&recieved.uint32);
        // Test:
        //MySerial.print(" ");
        if(rp2040.fifo.push_nb(recieved.command)){
            //MySerial.println(" >>>>>>>||| FIFO-Feedback Push ||| <<<<<<<");
        };
        curr_command = recieved.command;
        switch(curr_command){
            case stop:
                // Not-Stop
                break;
            case test_1:
            case test_2:
                break;
            default:
                if(special_command_handler(recieved.uint32)){
                    MySerial.println("Special Command handled");
                }
                break;
        }
    }
    //*/
    
    // Read Motor Sensors
    //Geminis Vorschlag:
    // Read Motor Sensors
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
    if(c1_control_flag || C1_GATHER_MOTORDATA){
        for (int j = 0; j < 4; j++) {
            for (int jj = 0; jj < 7; jj++) {
        
            // 1. Registeradresse schreiben
            Wire.beginTransmission(motor_add[j]);
            Wire.write(regs[jj]);
            byte transmission_error = Wire.endTransmission(false); // RESTART anfordern
            //transmission_error = 1; //
            // Fehlerbehandlung beim Schreiben
            if (transmission_error != 0) {
                // Reset des Busses bei schwerem Fehler
                Wire.end();
                delay(10);
                Wire.begin();
                Wire.setTimeout(10, true);
                i2c_flag = true;
                continue; // Nächstes Register versuchen
            }  

            // 2. Daten anfordern
            byte bytesReceived = Wire.requestFrom(motor_add[j], (byte)1);
        
            if (bytesReceived > 0 && Wire.available()) {
                motor_c1_readvalues[j][jj] = Wire.read();
            } else {
                // Device hat nicht geantwortet
                motor_c1_readvalues[j][jj] = 0;
            } 
            }
        }
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

        // Mutex entsperren (z. B. xSemaphoreGive / mutex_exit)
        mutexGive(&pb_mutex);
    }

    // Control Loops:
    MySerial.println("[]> Controller: ");
    MySerial.print("LD: current speed: "); MySerial.print(ld_curr_speed); MySerial.print(" , speed target: "); MySerial.print(ld_speed_target);
    ld_speed_output = float_to_byte(LD_Controller.calculate_out((float)(0xFF - ld_speed_target), (float)(controller_last_timestamp - controller_timestamp)));
    //ld_speed_output = float_to_byte(LD_Controller.calculate_out((float)(ld_curr_speed-ld_speed_target), (float)(controller_last_timestamp - controller_timestamp)));
    //rd_speed_output = float_to_byte(RD_Controller.calculate_out((float)(rd_curr_speed-rd_speed_target), (float)(controller_last_timestamp - controller_timestamp)));
    MySerial.print(" , new speed: "); MySerial.println(ld_speed_output);
    MySerial.print("Controller Output: ld out: "); MySerial.print(ld_speed_output); MySerial.print(", rd out: "); MySerial.println(rd_speed_output);

    // Send Motor Commands:
    for(int i = 0; i < 2; i++){ 
        wireWrite2Regs(Wire, motor_add[i], DIR_REG, 0x01, SPEED_REG, 0xFF);
    }
    
    c1_cycletime = micros() - c1_current_micros;
    ///*
    }
    //*/
}

void loop1(){ if(singlecore==false){loopCore1();}}