//  On Arduino Uno, the PWM pins are 3, 5, 6, 9, 10 and 11. 
#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <AltSoftSerial.h> // TX 9	RX 8

// ===== Bluetooth =====
char bluetoothByte;

AltSoftSerial SerialBluetooth; // 8 is RX, 9 is TX
// ===== Bluetooth =====

// ===== QTR Below Sensors =====
QTRSensors qtr;
const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
// ===== QTR Below Sensors =====

// ===== Motors =====
SoftwareSerial SWSerial(NOT_A_PIN, 11);
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

#define fullspeedPWM 127
#define secondSpeedPWM 70

#define halfSpeedPWM 64
// ===== Motors =====

// sensors is 1 when no detection, 0 when detected
#define IR_SENSOR_OPPONENT_F_LEFT 10
#define IR_SENSOR_OPPONENT_F_RIGHT 5
#define IR_SENSOR_OPPONENT_F_MID 6

#define IR_SENSOR_OPPONENT_RIGHT 12
#define IR_SENSOR_OPPONENT_LEFT 7

int battleMode = 1; // battle mode initialize to mode 1 first
int runMode = 1; // initialize to run

// === Initializations ===
void setup() {
    Serial.begin(9600);
    SerialBluetooth.begin(9600);
    SWSerial.begin(9600);

    // LED ONBOARD
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW); 

    // === IR Sensors Setup ===
    pinMode  (IR_SENSOR_OPPONENT_F_LEFT ,INPUT);
    pinMode  (IR_SENSOR_OPPONENT_F_RIGHT ,INPUT);
    pinMode  (IR_SENSOR_OPPONENT_F_MID ,INPUT);

    pinMode  (IR_SENSOR_OPPONENT_RIGHT ,INPUT);
    pinMode  (IR_SENSOR_OPPONENT_LEFT ,INPUT);
    // === IR Sensors Setup ===

    // Stop it from moving before initialization is done
    stop();

    // === QTR Setup ===
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A4, A5}, SensorCount); // A4 is Left, A5 is Right
    // === QTR Setup ===

    while(SerialBluetooth.available() == 0){
        Serial.println("Awaiting bluetooth battlemode command");
    }
    bluetoothByte = SerialBluetooth.read();

    if (bluetoothByte=='Y' || bluetoothByte=='y'){
        battleMode = 1; // switch 2
    }else if (bluetoothByte=='W' || bluetoothByte=='w'){
        battleMode = 2; // switch 3
    }else if (bluetoothByte=='V' || bluetoothByte=='v'){
        battleMode = 3; // switch 3
    }else if (bluetoothByte=='U' || bluetoothByte=='u'){
        battleMode = 4; // switch 4
    }

    while(SerialBluetooth.available() == 0){
        Serial.println("Awaiting bluetooth switch mode");
    }
    bluetoothByte = SerialBluetooth.read();
    if (bluetoothByte=='Z' || bluetoothByte=='z'){
        runMode = 1;
    }
    digitalWrite(13, HIGH); 
    delay(4850);
}

boolean bluetoothCheckOff(){
     if(SerialBluetooth.available() > 0){
        bluetoothByte = SerialBluetooth.read();

        if (bluetoothByte=='Z'){
            runMode = 1;
            return false;
        }else if(bluetoothByte=='z'){
            runMode = 0;
            return true;
        }
    }
    return false;
}

// ===== Movement =====

// MOTOR1 is RIGHT. MOTOR2 is LEFT
// -127 - 0 - 127 (-VE is forward. VE is backward) [255]
void turnLeft(){
    ST.motor(1, -fullspeedPWM);
    ST.motor(2, fullspeedPWM);
}

void turnRight(){
    ST.motor(1, fullspeedPWM);
    ST.motor(2, -fullspeedPWM);
}

void forward(){
    ST.motor(1, -halfSpeedPWM);
    ST.motor(2, -halfSpeedPWM);
}

void fastFoward(){
    ST.motor(1, -fullspeedPWM);
    ST.motor(2, -fullspeedPWM);
}

void backward(){
    ST.motor(1, fullspeedPWM);
    ST.motor(2, fullspeedPWM);
}

void forwardLeft(){
    ST.motor(1, -fullspeedPWM);
    ST.motor(2, -secondSpeedPWM);
}

void forwardRight(){
    ST.motor(1, -secondSpeedPWM);
    ST.motor(2, -fullspeedPWM);
}

void stop(){
    ST.motor(1, 0);
    ST.motor(2, 0);
}

// === Variables ===
bool enemyFoundFront = false; // assume enemy infront not found at first
// === Variables ===

bool checkEnemyDetectedFront(){
    // Check 3 sensors at the front for any detection
    if(!digitalRead(IR_SENSOR_OPPONENT_F_MID)){
        return true;
    }else if(!digitalRead(IR_SENSOR_OPPONENT_F_LEFT)){
        return true;
    }else if(!digitalRead(IR_SENSOR_OPPONENT_F_RIGHT)){
        return true;
    }
    return false;
}
bool checkEnemySide(){
    // Check 2 sensors at the side for any detection
    if(!digitalRead(IR_SENSOR_OPPONENT_RIGHT)){
        return true;
    }else if(!digitalRead(IR_SENSOR_OPPONENT_LEFT)){
        return true;
    }
    return false;

}
bool checkEnemyAllSensor(){
    // Check all 5 sensors for any detection
    if(checkEnemyDetectedFront()){
        return true;
    }else if(checkEnemySide()){
        return true;
    }
    return false;
}
bool checkIfAllFrontSimultaneouslyDetect(){
    // to check if all three detects at the same time
    if(digitalRead(IR_SENSOR_OPPONENT_F_MID)){
        return false;
    }else if(digitalRead(IR_SENSOR_OPPONENT_F_LEFT)){
        return false;
    }else if(digitalRead(IR_SENSOR_OPPONENT_F_RIGHT)){
        return false;
    }
    return true;
}

// ----- White Line Detection ----- //
// Speed is 127
#define LINE_TIME_DIFF 300 // FOR 180 DEGREE TURN
#define LINE_TIME_DIFF_HALF 120 // FOR 90 DEGREE TURN

unsigned long prevTime_LineCheckTimer = 0;
unsigned long currentTime_LineCheckTimer = 0;

#define BACKWARD_WHITE 250
#define STOP_TIME_WHITE 100
int threshold = 90;
// ----- White Line Detection ----- //
void checkWhiteLine(){
    qtr.read(sensorValues);

    // Left is 0. Check first pin sensor (left) if got white line (0 is white)
    if(sensorValues[0] <= threshold){
        if(sensorValues[1] <= threshold){
            // If both left and right detects whiteline, turn left still
            stop();
            delay(STOP_TIME_WHITE);

            backward();
            delay(BACKWARD_WHITE);

            prevTime_LineCheckTimer = millis();
            currentTime_LineCheckTimer = prevTime_LineCheckTimer;

            // if got, run code to move back
            while(currentTime_LineCheckTimer - prevTime_LineCheckTimer < LINE_TIME_DIFF_HALF){
                if(checkEnemyAllSensor()){
                    // if any of the sensor pick up something stop the moving motion
                    break;
                }

                currentTime_LineCheckTimer = millis();
                turnLeft();
                Serial.println("Whiteline Both detected, turning left full");
            }
            stop();
        }else{
            stop();
            delay(STOP_TIME_WHITE);

            backward();
            delay(BACKWARD_WHITE);

            prevTime_LineCheckTimer = millis();
            currentTime_LineCheckTimer = prevTime_LineCheckTimer;

            while(currentTime_LineCheckTimer - prevTime_LineCheckTimer < LINE_TIME_DIFF_HALF){
                if(checkEnemyAllSensor()){
                    break;
                }

                currentTime_LineCheckTimer = millis();
                turnRight();
                Serial.println("Whiteline Left detected, turning right");
            }
            stop();
        }

    }else if(sensorValues[1] <= threshold){
        stop();
        delay(STOP_TIME_WHITE);

        backward();
        delay(BACKWARD_WHITE);

        prevTime_LineCheckTimer = millis();
        currentTime_LineCheckTimer = prevTime_LineCheckTimer;

        // if got, run code to move back
        while(currentTime_LineCheckTimer - prevTime_LineCheckTimer < LINE_TIME_DIFF_HALF){
            if(checkEnemyAllSensor()){
                break;
            }

            currentTime_LineCheckTimer = millis();
            turnLeft();
            Serial.println("Whiteline Right detected, turning left");
        }
        stop();
    }else{
        forward();
        Serial.println("White line nothing, forward");
    }
}

void trackingRun(bool enemyFoundFront){
    if(enemyFoundFront){
        int sensorStatusFrontL = digitalRead (IR_SENSOR_OPPONENT_F_LEFT);
        int sensorStatusFrontR = digitalRead (IR_SENSOR_OPPONENT_F_RIGHT);
        int sensorStatusFrontM = digitalRead (IR_SENSOR_OPPONENT_F_MID);

        if(!sensorStatusFrontM){
            if(!sensorStatusFrontL){
                if(!sensorStatusFrontR){
                    // go straight all three detected
                    Serial.println("FORWARD");
                    fastFoward();
                }else{
                    // turn right Middle and left only
                    forwardRight();
                    Serial.println("Forward Right");
                }

            }else if(!sensorStatusFrontR){
                // turn left Middle and right only
                forwardLeft();
            }else{
                // middle only
                Serial.println("FORWARD");
                fastFoward();
            }

        }else if(!sensorStatusFrontL){
            // turn right
            forwardRight();

            Serial.println("Forward Right");
        }else if(!sensorStatusFrontR){
            // turn left
            forwardLeft();

            Serial.println("Forward Left");
        }
    }
}

unsigned long prevTime_PushNoDetectTimer = 0;
unsigned long currentTime_PushNoDetectTimer = 0;

unsigned long prevTime_ChangeTurnTimer = 0;
unsigned long currentTime_ChangeTurnTimer= 0;

bool firstTimeTurn = true;

#define TURNCHANGE_TIME_LEFT_HALF 175
#define TURNCHANGE_TIME_LEFT 250

#define TURNCHANGE_TIME_FORWARD_LEFT TURNCHANGE_TIME_LEFT * 2
#define TURNCHANGE_TIME_RIGHT TURNCHANGE_TIME_LEFT * 3
#define TURNCHANGE_TIME_FORWARD_RIGHT TURNCHANGE_TIME_LEFT * 4

#define NODETECT_TIME_DIFF 300
void trackingRam(bool enemyFoundFront){
    if(enemyFoundFront){
        if(checkIfAllFrontSimultaneouslyDetect()){
            firstTimeTurn = true;
            bool continuePushing = true;
            int diff = 0;

            prevTime_PushNoDetectTimer = 0;
            currentTime_PushNoDetectTimer = 0;

            prevTime_ChangeTurnTimer = millis();
            currentTime_ChangeTurnTimer= prevTime_ChangeTurnTimer;

            while(continuePushing){
                if(bluetoothCheckOff()){
                    break;
                }
                currentTime_ChangeTurnTimer = millis();
                diff = currentTime_ChangeTurnTimer - prevTime_ChangeTurnTimer;

                if(firstTimeTurn){
                    if(diff < TURNCHANGE_TIME_LEFT_HALF){
                        firstTimeTurn = false;
                        turnLeft();
                        Serial.println("Turn Left");
                    }else{
                        firstTimeTurn = false;
                    }
                }else{
                    if(diff < TURNCHANGE_TIME_LEFT){
                        turnLeft();
                        Serial.println("Turn Left");
                    }else if(diff < TURNCHANGE_TIME_FORWARD_LEFT){
                        fastFoward();
                        Serial.println("Forward From Left");
                    }else if(diff < TURNCHANGE_TIME_RIGHT){
                        turnRight();
                        Serial.println("Turn Right");
                    }else if(diff < TURNCHANGE_TIME_FORWARD_RIGHT){
                        fastFoward();
                        Serial.println("Forward From Right");
                    }else{
                        // way pass the cycle timer, reset.
                        prevTime_ChangeTurnTimer = millis();
                    }
                }

                if(!checkIfAllFrontSimultaneouslyDetect()){
                    Serial.println("All front not detected");
                    if(prevTime_PushNoDetectTimer == 0){
                        prevTime_PushNoDetectTimer = millis();
                    }else{
                        currentTime_PushNoDetectTimer = millis();
                        if(currentTime_PushNoDetectTimer - prevTime_PushNoDetectTimer > NODETECT_TIME_DIFF){
                            continuePushing = false;
                        }
                    }
                }else{
                    Serial.println("All front detected");
                    prevTime_PushNoDetectTimer = 0;
                }
            }
            stop(); // if no more simultaneous detection
        }else{
            // Since check all front is not right, we need to keep turning it till it is. So check L and R here only
            int sensorStatusFrontL = digitalRead (IR_SENSOR_OPPONENT_F_LEFT);
            int sensorStatusFrontR = digitalRead (IR_SENSOR_OPPONENT_F_RIGHT);
            int sensorStatusFrontM = digitalRead (IR_SENSOR_OPPONENT_F_MID);

            if(!sensorStatusFrontM){
                Serial.println("FORWARD TRACK");
                fastFoward();
            }
            if(!sensorStatusFrontL){
                Serial.println("TURN RIGHT TRACK");
                forwardRight();
            }else if(!sensorStatusFrontR){
                Serial.println("TURN LEFT TRACK");
                forwardLeft();
            }
        }
    }
}

// If can't find enemy, move forward but keep track of them. Push normally
void strategyOne(){
    enemyFoundFront = checkEnemyDetectedFront(); 

    // Check if left or right side sensors detects something
    if(!enemyFoundFront){
        // enemy at right side
        if(!digitalRead (IR_SENSOR_OPPONENT_RIGHT)){
            while(true){
                
                if(bluetoothCheckOff()){
                    break;
                }
                if(checkEnemyDetectedFront()){
                    enemyFoundFront = true;
                    break;
                }
                
                Serial.println("ALWAYS RIGHT STATIONARY");
                turnRight();
            }
        }else if(!digitalRead (IR_SENSOR_OPPONENT_LEFT)){
            while(true){
                                
                if(bluetoothCheckOff()){
                    break;
                }
                if(checkEnemyDetectedFront()){
                    enemyFoundFront = true;
                    break;
                }
                Serial.println("ALWAYS LEFT STATIONARY");
                turnLeft();
            }
        }
    }
    if(enemyFoundFront){
        trackingRun(enemyFoundFront);
    }else{
        checkWhiteLine();
        stop();
    }
}

void strategyTwo(){
    enemyFoundFront = checkEnemyDetectedFront(); 

    if(!enemyFoundFront){
         // enemy at right side
        if(!digitalRead (IR_SENSOR_OPPONENT_RIGHT)){
            while(true){
                
                if(bluetoothCheckOff()){
                    break;
                }
                if(checkEnemyDetectedFront()){
                    enemyFoundFront = true;
                    break;
                }
                
                Serial.println("ALWAYS RIGHT STATIONARY");
                turnRight();
            }
        }else if(!digitalRead (IR_SENSOR_OPPONENT_LEFT)){
            while(true){
                                
                if(bluetoothCheckOff()){
                    break;
                }
                if(checkEnemyDetectedFront()){
                    enemyFoundFront = true;
                    break;
                }
                Serial.println("ALWAYS LEFT STATIONARY");
                turnLeft();
            }
        }
    }

    if(enemyFoundFront){
        trackingRam(enemyFoundFront);
    }else{
        checkWhiteLine();
        stop();
    }
}


unsigned long prevTime_ChangeTurnTimerST = 0;
unsigned long currentTime_ChangeTurnTimerST = 0;
bool firstTimeTurnST = true;
int diffST = true;

#define WAIT_TIMER 3000

#define TURNCHANGE_TIME_MIDDLESTOP 7000
#define TURNCHANGE_TIME_LEFT_ST 50 + TURNCHANGE_TIME_MIDDLESTOP
#define TURNCHANGE_TIME_STOPLEFT_ST WAIT_TIMER + TURNCHANGE_TIME_LEFT_ST
#define TURNCHANGE_TIME_MIDDLELEFT_ST 62 + TURNCHANGE_TIME_STOPLEFT_ST
#define TURNCHANGE_TIME_STOPMIDDLELEFT_ST WAIT_TIMER + TURNCHANGE_TIME_MIDDLELEFT_ST
#define TURNCHANGE_TIME_RIGHT_ST 62 + TURNCHANGE_TIME_STOPMIDDLELEFT_ST
#define TURNCHANGE_TIME_STOPRIGHT_ST WAIT_TIMER + TURNCHANGE_TIME_RIGHT_ST
#define TURNCHANGE_TIME_MIDDLERIGHT_ST 45 + TURNCHANGE_TIME_STOPRIGHT_ST 

void strategyThree(){
    if(checkEnemySide()){
        firstTimeTurnST = true;
         // enemy at right side
        if(!digitalRead (IR_SENSOR_OPPONENT_RIGHT)){
            turnRight();
            enemyFoundFront = checkEnemyDetectedFront();
            Serial.println("TURN RIGHT STATIONARY");
            
        }else if(!digitalRead (IR_SENSOR_OPPONENT_LEFT)){
            turnLeft();
            enemyFoundFront = checkEnemyDetectedFront();
            Serial.println("TURN LEFT STATIONARY");
        }
    }else if(checkEnemyDetectedFront()){
        firstTimeTurnST = true;
        trackingRun(true);
        checkWhiteLine();
    }else{
        if(firstTimeTurnST){
            currentTime_ChangeTurnTimerST = millis();
            prevTime_ChangeTurnTimerST = millis();
            firstTimeTurnST = false;
            diffST = 0;
        }

        currentTime_ChangeTurnTimerST = millis();
        diffST = currentTime_ChangeTurnTimerST - prevTime_ChangeTurnTimerST;

        if(diffST <= TURNCHANGE_TIME_MIDDLESTOP){
            stop();
            Serial.println("Start stop");
        }else if(diffST <= TURNCHANGE_TIME_LEFT_ST){
            turnLeft();
            Serial.println("Turning Left");
        }else if(diffST <= TURNCHANGE_TIME_STOPLEFT_ST){
            stop();
            Serial.println("Stopping on Left");
        }else if(diffST <= TURNCHANGE_TIME_MIDDLELEFT_ST){
            turnRight();
            Serial.println("Turn Right to middle");
        }else if(diffST <= TURNCHANGE_TIME_STOPMIDDLELEFT_ST){
            stop();
            Serial.println("Stopping on middle");
        }else if(diffST <= TURNCHANGE_TIME_RIGHT_ST){
            turnRight();
            Serial.println("Turning right");
        }else if(diffST <= TURNCHANGE_TIME_STOPRIGHT_ST){
            stop();
            Serial.println("Stopping on Right");
        }else if(diffST <= TURNCHANGE_TIME_MIDDLERIGHT_ST){
            turnLeft();
            Serial.println("Turning left to middle");
        }else{
            // way pass the cycle timer, reset.
            prevTime_ChangeTurnTimerST = millis();
        }
    }
}

void strategyFour(){
    if(checkEnemySide()){
        firstTimeTurnST = true;
         // enemy at right side
        if(!digitalRead (IR_SENSOR_OPPONENT_RIGHT)){
            turnRight();
            enemyFoundFront = checkEnemyDetectedFront();
            Serial.println("TURN RIGHT STATIONARY");
            
        }else if(!digitalRead (IR_SENSOR_OPPONENT_LEFT)){
            turnLeft();
            enemyFoundFront = checkEnemyDetectedFront();
            Serial.println("TURN LEFT STATIONARY");
        }
    }else if(checkEnemyDetectedFront()){
        firstTimeTurnST = true;
        trackingRam(true);
        checkWhiteLine();
    }else{
        if(firstTimeTurnST){
            currentTime_ChangeTurnTimerST = millis();
            prevTime_ChangeTurnTimerST = millis();
            firstTimeTurnST = false;
            diffST = 0;
        }

        currentTime_ChangeTurnTimerST = millis();
        diffST = currentTime_ChangeTurnTimerST - prevTime_ChangeTurnTimerST;

        if(diffST <= TURNCHANGE_TIME_MIDDLESTOP){
            stop();
            Serial.println("Start stop");
        }else if(diffST <= TURNCHANGE_TIME_LEFT_ST){
            turnLeft();
            Serial.println("Turning Left");
        }else if(diffST <= TURNCHANGE_TIME_STOPLEFT_ST){
            stop();
            Serial.println("Stopping on Left");
        }else if(diffST <= TURNCHANGE_TIME_MIDDLELEFT_ST){
            turnRight();
            Serial.println("Turn Right to middle");
        }else if(diffST <= TURNCHANGE_TIME_STOPMIDDLELEFT_ST){
            stop();
            Serial.println("Stopping on middle");
        }else if(diffST <= TURNCHANGE_TIME_RIGHT_ST){
            turnRight();
            Serial.println("Turning right");
        }else if(diffST <= TURNCHANGE_TIME_STOPRIGHT_ST){
            stop();
            Serial.println("Stopping on Right");
        }else if(diffST <= TURNCHANGE_TIME_MIDDLERIGHT_ST){
            turnLeft();
            Serial.println("Turning left to middle");
        }else{
            // way pass the cycle timer, reset.
            prevTime_ChangeTurnTimerST = millis();
        }
    }
}
 
void loop(){
    if(bluetoothCheckOff()){
        stop();
    }

    if (runMode == 1){
        if(battleMode == 1){
            strategyOne();
            Serial.println("Strat 1");
        }else if(battleMode == 2){
            strategyTwo();
            Serial.println("Strat 2");
        }else if(battleMode == 3){
            strategyThree();
            Serial.println("Strat 3");
        }else if(battleMode == 4){
            strategyFour();
            Serial.println("Strat 3");
        }
    }else if (runMode==0){
        // if switched off, won't run
        Serial.println("Switched off");
        stop();
    } 
}