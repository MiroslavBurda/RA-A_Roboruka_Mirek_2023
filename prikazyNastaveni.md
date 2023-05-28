    // rkArmSetGrabbing(false);
    // fmt::print("open: {}\n", rkArmGetServo(2) );
    // delay(1000);
    // rkArmSetGrabbing(true);
    // fmt::print("close: {}\n", rkArmGetServo(2) );
    
    // rkArmSetServo(3, 60);
    //rkArmSetServo(4, 60);

    // rkArmMoveTo(145, -45);   // je uprostred v prostoru
    // rkArmSetGrabbing(false); // open: 87 deg
    // delay(1000);
    // rkArmMoveTo(145, 65);   // je tesne nad zemi 
    // rkArmSetGrabbing(true); // close: 160 deg 
    
           //rkArmMoveTo(145, -45);   // je uprostred v prostoru
            //rkArmSetGrabbing(false); // open: 87 deg
            //delay(1000);
            //rkArmMoveTo(145, 65);   // je tesne nad zemi 
            //rkArmSetGrabbing(true); // close: 160 deg 

#ifdef AAA

rkArmSetServo(3, k);  // ID =3 -> leve klepeto dole cca 12 nahoře 80 smerem nahoru pozice roste 
rkArmSetServo(4, k);  // ID =4 -> prave klepeto dole 140 nahoře 60 smerem nahoru pozice klesa


#endif 

       

            // //rkMotorsSetPower(50, 50);
            // man.motor(MotorId::M1).drive(-500*110/100, 50);	// right motor 
            // man.motor(MotorId::M2).drive(-500, 50);	// left motor
            // delay(100);
            // //man.motor(MotorId::M1).drive(0, 0);	// nezastavi, dokud nedojede minuly drive  
            // //man.motor(MotorId::M2).drive(0, 0);
            // rkMotorsSetPower(0, 0); // zastavi ihned, i kdyz probiha drive  	
            // int32_t enR = man.motor(MotorId::M1).enc()->value();  // reading encoder
            // int32_t enL = man.motor(MotorId::M2).enc()->value();
            // fmt::print("{}: {},  {}\n", k, enL, enR);
              
            //rkArmSetServo(2, 120+k);  // 124 deg - rovnoběžná klepeta

    /*gpio_num_t SerialRx1 = GPIO_NUM_4;
    gpio_num_t SerialTx1 = GPIO_NUM_14;
    pinMode(SerialRx1, INPUT);
    pinMode(SerialTx1, OUTPUT);
    digitalWrite(SerialRx1, HIGH);
    digitalWrite(SerialTx1, HIGH);*/

int k = 120; 
int IDservo = 4;


        // // zacatek nastavovani serv ****************************************************************************************
        // if (rkButtonIsPressed(1)) {
        //     k += 10;
        //     rkArmSetServo(IDservo, k);
        // }
        // if (rkButtonIsPressed(2)) { 
        //     k-=10;
        //     rkArmSetServo(IDservo, k);      
        // }
        // if (rkButtonIsPressed(3)) {
        //     k += 1;
        //     rkArmSetServo(IDservo, k);
        // }
        // if(!LORRIS) {
        //     fmt::print("vision: {}, position: {}\n", k, rkArmGetServo(IDservo) ); //konec nastavovani serv *****************
        // }

    spi_bus_config_t busCfg;        // zacatek nastaveni kamery +++++++++++++++++++++++++++++++++
    busCfg.mosi_io_num = GPIO_NUM_25; 
    busCfg.miso_io_num = GPIO_NUM_26; 
    busCfg.sclk_io_num = GPIO_NUM_27; 
    busCfg.quadwp_io_num = -1;
    busCfg.quadhd_io_num = -1;
    busCfg.flags = SPICOMMON_BUSFLAG_MASTER;
    busCfg.intr_flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &busCfg, 0)); // HSPI_HOST je typ enum, nepsat číslo (1), ale tu danou konstantu z výběru  (HSPI_HOST)

    auto linkRes = LinkSpi::addSpiDevice(HSPI_HOST, 2000000); // nastavení rychlost komunikace SPI na běžných pinech, aby to stíhaly RBC i zařízení - nutno vyzkoušet 
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());
    GetBlocksContext blocksCtx;     // konec nastaveni kamery +++++++++++++++++++++++++++++++++++++

        auto err = pixy.getColorBlocks(1|2|4|8, 8, blocksCtx); // cervena | modra | zelena | cerna
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error1: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if(LORRIS) {
            const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
            uart_write_bytes(UART_NUM_0, (const char*)lorrisHeader, sizeof(lorrisHeader));
            if(blocksCtx.blocks.size() > 0) {
                uart_write_bytes(UART_NUM_0, (const char*)blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
            }
        
            vTaskDelay(1000); // aby Lorris nepadala, musí tady být pauza 

            uint8_t orderTest[] = { 0xFF,  0x02, 0x02, 0x03, 15  };  // test exportu jiných dat než z kamery do Lorris 
            uart_write_bytes(UART_NUM_0, (const char*)orderTest, sizeof(orderTest));
        }
        else  {
            int poziceX = blocksCtx.blocks[0]->x;
            int poziceY = blocksCtx.blocks[0]->y;
            int poziceX2 = blocksCtx.blocks[1]->x;
            int poziceY2 = blocksCtx.blocks[1]->y;

           printf("i: %i, %i, %i, %i\n", poziceX, poziceY, poziceX2, poziceY2);
        }

        if (Serial2.available() > 0) { 
            byte readData[10]= { 1 }; //The character array is used as buffer to read into.
            int x = Serial2.readBytes(readData, 8); //It require two things, variable name to read into, number of bytes to read.
            Serial.println(x); //display number of character received in readData variable.
            for(int i = 0; i<10; i++) {
                if(!LORRIS) { 
                    printf("i: %i, ", readData[i]); 
                }
            printf("\n");    
            }               
        }