#include <Arduino.h> // from Roboruka
#include <SmartLeds.h>
#include "RBControl.hpp" // for encoders 
#include "roboruka.h"
using namespace rb;

#include <driver/spi_master.h> // from pixy2
#include <driver/uart.h>
#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"
#include "pixy2/i2c.hpp"
using namespace pixy2;

// startuje z pravého zadního horního rohu, dva palce od zadní steny
// tlačítko SW1 spouští přípravu, tlačítko SW2 vybírá strany; startovací lanko rozjíždí - SW3

// nastaveni paze roboruky: https://roboticsbrno.github.io/RB3201-RBControl-Roboruka-library/group__arm.html
//todo nastaveni konstant - maximální hodnoty poloh pro serva 
//todo dokončení komunikace mezi nano a esp32 - hlavička 1 bajt, zprávu pošlu 3x za sebou a vyhodnotím, if přišla pokaždé stejně 
// ukládám si minulou hodnotu, if nová hodnota nepřišla a minulá hodnota dost velká - v pohodě, if ne, brzdím až stojím  
//todo musí zastavit za nastavený čas - max. 126 sekund (a rozsvítí všechny LED vzadu) 
//todo přesné řízení 
//todo iLED pásek, který indikuje, co měří ultrazvuky - zelená/bílá - volno, červená -stát 
//todo předstartovní taneček - po stisku  tl 1 
//todo start na vytažení lanka -> rozpojení tl. 3 
// dotaz: na jakém napětí běží data pro LX16A? 

// kalibrace pixy2: s klepetem na zemi a SetLedAll(127, 127, 127); 
// potom pro klepeto ve výšce cca 10 cm a víc nastavit SetLedAll(255, 255, 255);
// příjezd do prostoru kostek, kameru více nahoru, pootočit vpravo a vlevo, najít nejblžší kostku, 
// poodjet, aby se dalo klepeto položit na zem - nesmí být jiná kostka těsně kolem klepeta 
// dojet ke kostce, vzít ji, odvést 

bool LORRIS = false; // varianty debugu:  true - data UART0 jdou do Lorris, false - data jdou do Serial monitoru 

const int LED_COUNT = 16; // zacatek nastaveni iLED  -------------------
const int DATA_PIN = 32;
const int CHANNEL = 0;

bool red = true;
const byte readSize = 8;
const byte header = 250; //hlavicka zpravy ma tvar: {250, 250+k}, k = 1 ... 3    
constexpr byte msgHeader[3] = {251, 252, 253};
const byte minVzdal = 60; // minimalni vzdalenost, na ktere se sousedni robot muze priblizit, if se priblizi vic, tak abort();

byte readData0[readSize]= {0}; //The character array is used as buffer to read into.
byte readData1[readSize]= {0};
byte readData2[readSize]= {0};
byte state = 1; // stav programu
bool startState = false; // if je odstartovano vytazenim lanka 
byte speed = 50; // obvykla rychlost robota
byte speedSlow = 35; // pomala = zataceci rychlost robota  
float coefSpeed = 1.17; // pravy motor je pomalejsi, takze se jeho rychlost musi nasobit touto konstantou 
float ticksToMm = -1.33; // prepocet z tiku v enkoderech na mm 
int stopCount = 0; 
long timeOfGame = 120000; // pocet milisekund hry/souteze


/**
 * @brief Funkce na vyhledání nejmenší hodnoty z byte pole. !!! Nulu to nebere jako nejmenší !!!
 * 
 * @param arr   Ukazatel na pole hodnot
 * @param index Adresa kam má funkce vrátit pozici nejmenší hodnoty
 * @return byte Vrací nejmenší hodnotu pole 
 */
byte min_arr(byte *arr, int &index){
    byte tmp = 255;
    index = -1;
    for (size_t i = 0; i < 8; i++) {
        if (arr[i] < tmp && arr[i] != 0) {
            tmp = arr[i];
            index = i;
        }
    }
    return tmp;
}

// SmartLed -> RMT driver (WS2812/WS2812B/SK6812/WS2813)
SmartLed leds(LED_WS2812, LED_COUNT, DATA_PIN, CHANNEL, DoubleBuffer);

void SetLedAll(uint8_t R, uint8_t G, uint8_t B) // iLED pro přisvícení kamery 
{
    for (int i = 0; i < LED_COUNT; i++)
        leds[i] = Rgb{R, G, B};
    leds.wait();
    leds.show();
}       // konec nastaveni iLED ---------------------------------------

void setup() {
    SetLedAll(0, 0, 0);  
    // SetLedAll(127, 127, 127);  
    // SetLedAll(255, 255, 255);

    rkConfig cfg;
    cfg.arm_bone_trims[0] = 25;  // nastaveni posunu serva robopaze vuci nulove poloze
    cfg.arm_bone_trims[1] = 30;

    cfg.motor_enable_failsafe = false;
    cfg.rbcontroller_app_enable = true;
    cfg.motor_max_power_pct = 100;
    cfg.motor_polarity_switch_left = true;
    cfg.motor_polarity_switch_right = true;
    auto &man = Manager::get();
    man.initSmartServoBus(5, (gpio_num_t)cfg.pins.arm_servos); // nastaveni poctu serv na roboruce 
    rkSetup(cfg);
    
    uart_config_t uart_config = {  
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));

    rkLedGreen(false);
    rkLedBlue(true); // cekani na stisk SW1, take po resetu stop tlacitkem, aby se zase hned nerozjela
    fmt::print("{}'s roboruka '{}' started!\n", cfg.owner, cfg.name);
    fmt::print("Battery at {}%, {}mV\n", rkBatteryPercent(), rkBatteryVoltageMv());
    printf("cekani na stisk SW1\n");
    while(true) {   
        if(!man.expander().digitalRead(SW1)) { // nula - stisknuto 
            break;
        }
        delay(10);
    }
    rkLedBlue(false);

    Serial2.begin(115200, SERIAL_8N1, 4, 14); // Rx = 4 Tx = 14



    // rkArmSetServo(3, 60); // parkovaci pozice

    rkLedYellow(true); // robot je připraven
    if(red) {  
        rkLedRed(true);
        rkLedBlue(false);
    }
    else {
        rkLedRed(false);
        rkLedBlue(true);               
    }

    rkArmSetGrabbing(false); // otevrit ruku 
    printf("vyber strany - SW2\n");
    while(true) {   
        if(man.expander().digitalRead(SW3)) { // vytazeni startovaciho lanka na tl. Left rozjede robota  
            startState = true;
            printf("Rozjizdim se\n");
            break;
        }
        if(!man.expander().digitalRead(SW2)) {
            red = !red;
            if(red) {
                rkLedRed(true);
                rkLedBlue(false);
            }
            else {
                rkLedRed(false);
                rkLedBlue(true);               
            }
        delay(300);
        }
        
    }
    delay(1000); // robot se pri vytahovani lanka musi pridrzet -> mel by pockat, nez oddelam ruku, ale je lepsi drzet se pod urovni ultrazvuku  
    
    long startTime = millis();
    printf("startTime: %i ", startTime);
    long currentTime = millis();
    

    while(true) {  // hlavni smycka 
        //vTaskDelay(pdMS_TO_TICKS(500));  //************************* todo ZMENSIT   
        zacatekHlavniSmycky:
        currentTime = millis(); 
        printf("currentTime: %i ", currentTime);
        if ((currentTime - startTime) > timeOfGame ) {
            printf("cas: %i vyprsel ... koncim", (currentTime - startTime));
            abort();
        }
            

        delay(10);
        int pozice0;
            if (Serial2.available() > 0)  {  // reseni ultrazvuku 
                int temp = Serial2.read();
                if(temp == header) {
                    // printf("bytes: %i \n", header); 
                    if (Serial2.available() > 0) {
                        int head = Serial2.read();
                        printf("head: %i ", head); 
                        switch (head) {
                        case msgHeader[0]: 
                            Serial2.readBytes(readData0, readSize); //It require two things, variable name to read into, number of bytes to read.
                            // senzor 6 čte občas falešné údaje kolem hodnoty 30-40 
                            if ((readData0[6]) < 70 ) 
                                readData0[6] = readData0[6] + 70;
                            for(int i = 0; i<8; i++) { printf("%i: %i, ", i, readData0[i]); } printf("\n ");
                            break;        
                        case msgHeader[1]:
                            Serial2.readBytes(readData1, readSize); 
                            // senzor 6 čte občas falešné údaje kolem hodnoty 30-40 
                             if ((readData1[6]) < 70 ) 
                                readData1[6] = readData1[6] + 70;
                            for(int i = 0; i<8; i++) { printf("%i: %i, ", i, readData1[i]); } printf("\n ");
                            break;        
                        case msgHeader[2]:
                            Serial2.readBytes(readData2, readSize); 
                            for(int i = 0; i<8; i++) { printf("%i: %i, ", i, readData2[i]); } printf("\n ");
                            break;
                        default:
                            printf("Nenasel druhy byte hlavicky !! "); 
                        }
                    }
                    int min0 = min_arr(readData0, pozice0); 
                    int min1 = min_arr(readData1, pozice0); 
                    if ( (min0 == min1) && (min0 < minVzdal) ) {
                        printf("Souper blizi...");
                        if(startState) {
                            man.motor(MotorId::M1).stop();
                            man.motor(MotorId::M2).stop();
                            rkLedRed(true);   // rozsviti vsechny LED, if vidi soupere
                            rkLedYellow(true); 
                            rkLedGreen(true); 
                            rkLedBlue(true); 

                            int enc2 = man.motor(MotorId::M2).enc()->value();
                            printf("enc2: %d \n",enc2);

                            stopCount++;
                            printf("Souper se prilis priblizil... %i \n", stopCount);
                            delay(500);
                            goto zacatekHlavniSmycky;
                            if (stopCount > 10 ) {
                              abort();
                            }
                        }
                    }
                }
                //printf("test1: %i ", state);
            } // reseni ultrazvuku - konec
              // todo dodělat ************ 

        int encP = man.motor(MotorId::M1).enc()->value(); //vypis enkoderu
        printf("encP: %d \n",encP);
        int encL = man.motor(MotorId::M2).enc()->value();
        printf("encL: %d \n",encL);

        if(state == 1) { // jizda rovne ze startu 
            state = 2;
            man.motor(MotorId::M1).drive(500*ticksToMm, speed*coefSpeed, [&](rb::Encoder& _){printf("ze startu\n"); state = 3;});	// right motor <lze callback>
            man.motor(MotorId::M2).drive(500*ticksToMm, speed);	// left motor
        }
        //printf("test2: ");

        if(state == 3) {
            state = 4;
            if (red){
                man.motor(MotorId::M1).drive(-150*ticksToMm, speedSlow*coefSpeed, [&](rb::Encoder& _){printf("zatocil k nakladaku\n"); state = 5;});	// right motor <lze callback>
                man.motor(MotorId::M2).drive( 150*ticksToMm, speedSlow);	// left motor
                delay(500);
            } 
            else {
                man.motor(MotorId::M1).drive( 170*ticksToMm, speedSlow*coefSpeed, [&](rb::Encoder& _){printf("zatocil k nakladaku\n"); state = 5;});	// right motor <lze callback>
                man.motor(MotorId::M2).drive(-170*ticksToMm, speedSlow);	// left motor
            }
        }

        if(state == 5) {
            state = 6;
            // rkMotorsDriveAsync(1010, 1010, speed, [&](){printf("vytlacil\n"); state = 7;}); // ************ bez couvani - state 9 
            man.motor(MotorId::M1).drive(1210*ticksToMm, speed*coefSpeed, [&](rb::Encoder& _){printf("vytlacil\n"); state = 7;});	// right motor <lze callback>
            man.motor(MotorId::M2).drive(1210*ticksToMm, speed);
        }

        if(state == 7) { // couvani zpet 
            state = 8;
            man.motor(MotorId::M1).drive(-1310*ticksToMm, speed*coefSpeed, [&](rb::Encoder& _){printf("vratil se dlouhy usek\n"); state = 9;});	// right motor <lze callback>
            man.motor(MotorId::M2).drive(-1310*ticksToMm, speed);
        }

        if(state == 9) {
            state = 10;
            if (red){
                man.motor(MotorId::M1).drive(150*ticksToMm, speedSlow*coefSpeed, [&](rb::Encoder& _){printf("zatocil na start\n"); state = 11;});	// right motor <lze callback>
                man.motor(MotorId::M2).drive(-150*ticksToMm, speedSlow);	// left motor

            } 
            else {
                man.motor(MotorId::M1).drive(-170*ticksToMm, speedSlow*coefSpeed, [&](rb::Encoder& _){printf("zatocil na start\n"); state = 11;});	// right motor <lze callback>
                man.motor(MotorId::M2).drive(170*ticksToMm, speedSlow);	// left motor
            }
        }

        if(state == 11) { // jizda rovne ze startu 
            state = 12;
            man.motor(MotorId::M1).drive(-750*ticksToMm, speed*coefSpeed, [&](rb::Encoder& _){printf("vratil se na start\n"); state = 13;});	// right motor <lze callback>
            man.motor(MotorId::M2).drive(-750*ticksToMm, speed);	// left motor
        }

    }
}


