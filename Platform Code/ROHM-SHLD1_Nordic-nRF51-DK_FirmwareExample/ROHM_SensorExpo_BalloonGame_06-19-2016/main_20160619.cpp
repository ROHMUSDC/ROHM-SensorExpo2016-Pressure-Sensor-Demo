/* 
 * mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
 //BALLOON GAME!
 /*
 * 	Code Example for ROHM Sensor Expo Balloon Game
 * 
 *  Description: This Application interfaces ROHM's BM1383AGLV with Nordic's nRF51-DK
 *  This Code supports the following sensor devices:
 *      > BM1383AGLV Pressure Sensor
 *
 *  This Code allows the user to configure two known pressure distances and save pressure readings
 *	onto the application.  Then it will automatically extrapolate these values and allow the user to see
 *	the height of the board.  When connected to a balloon, greater heights can be achieved and the board
 *	will return the current height of the board.
 *   
 *  Additional information about the this Balloon Game can be found at the following link:
 *      asdfasdfasdf
 * 
 *  Last Upadtaed: 6/19/2016 
 *  Author: ROHM USDC
 *  Contact Information: engineering@rohmsemiconductor.com
 */

#define nRF52DevKit
#define Pressure            //BM1383, Barometric Pressure Sensor

#include "mbed.h"
#include "BLEDevice.h"
#include "UARTService.h"
#include "nrf_temp.h"
#include "I2C.h"
#include <string>

#define MAX_REPLY_LEN           (UARTService::BLE_UART_SERVICE_MAX_DATA_LEN)    //Actually equal to 20
#define SENSOR_READ_INTERVAL_S  (1.0F) 
#define ADV_INTERVAL_MS         (1000UL)
#define UART_BAUD_RATE          (19200UL)
#define DEVICE_NAME             ("ROHM DEMO  ") // This can be read AFTER connecting to the device.
#define SHORT_NAME              ("BALLOON1")    // Keep this short: max 8 chars if a 128bit UUID is also advertised.
#define DEBUG(...)              { m_serial_port.printf(__VA_ARGS__); }

// Function Prototypes
void PBTrigger();               //Interrupt function for PB4
void BTLE_DataWrittenHandler();

// Global Variables
BLEDevice   m_ble;
Serial      m_serial_port(p9, p11);  // TX pin, RX pin Original
//Serial      m_serial_port(p8, p10);  // TX pin, RX pin 
DigitalOut  m_cmd_led(LED1);
DigitalOut  m_error_led(LED2);
UARTService *m_uart_service_ptr;
DigitalIn   testButton(p20);
InterruptIn sw4Press(p20);
I2C         i2c(p30,p7);
bool        RepStart = true;
bool        NoRepStart = false;
int         i = 1;
unsigned char   printQue = 0;
string      ReceivedValue;
char        FormattedData[30];
uint8_t  	buf[MAX_REPLY_LEN];
uint32_t 	len = 0;

//Sensor Variables
#ifdef Pressure
int         Press_addr_w = 0xBA;
int         Press_addr_r = 0xBB;

char        PWR_ON[2] = {0x12, 0x01};
char        PWR_OFF[2] = {0x12, 0x00};
char        SLEEP_OFF[2] = {0x13, 0x01};
char        SLEEP_ON[2] = {0x13, 0x00};
char        Mode_Control[2] = {0x14, 0xCA};

char        Press_Content_ReadData[6];
char        Press_Addr_ReadData =0x1A;

int         BM1383_Temp_highByte;
int         BM1383_Temp_lowByte;
int         BM1383_Pres_highByte;
int         BM1383_Pres_lowByte;
int         BM1383_Pres_leastByte; 

short int   BM1383_Temp_Out;
float       BM1383_Temp_Conv_Out;
float       BM1383_Pres_Conv_Out;

float       BM1383_Var;
float       BM1383_Deci;

uint32_t    BM1383_TempPressure;


#endif

//Balloon Game Variables
float       BM1383_Pres_0Level;
float       BM1383_Pres_KnownLevel;
float       HeightBase = 0;
float       HeightKnown_Total = 68;
float       HeightKnown_Foot = 60;
float       HeightKnown_Inches = 8;
float       slope;
float       yInt;
float       pressureCurr;
float       HeightExtrapolated; //Assuming linear pressure curve
float       HeightExtrapolated_Foot = 60;
float       HeightExtrapolated_Inches = 8;


/**
 * This callback is used whenever a disconnection occurs.
 */
void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    switch (reason) {
    case Gap::REMOTE_USER_TERMINATED_CONNECTION:
        DEBUG("Disconnected (REMOTE_USER_TERMINATED_CONNECTION)\n\r");
        break;
    case Gap::LOCAL_HOST_TERMINATED_CONNECTION:
        DEBUG("Disconnected (LOCAL_HOST_TERMINATED_CONNECTION)\n\r");
        break;
    case Gap::CONN_INTERVAL_UNACCEPTABLE:
        DEBUG("Disconnected (CONN_INTERVAL_UNACCEPTABLE)\n\r");
        break;
    }

    DEBUG("Restarting the advertising process\n\r");
    m_ble.startAdvertising();
}

/**
 * This callback is used whenever the host writes data to one of our GATT characteristics.
 */
void dataWrittenCallback(const GattCharacteristicWriteCBParams *params)
{
    // Ensure that initialization is finished and the host has written to the TX characteristic.
    if ((m_uart_service_ptr != NULL) && (params->charHandle == m_uart_service_ptr->getTXCharacteristicHandle())) {
        int i;
        ReceivedValue.clear();
        for(i = 0; i < params->len; i++)
        {
            ReceivedValue += params->data[i];
        }
        printQue = 1;
    }
}

/**
 * This callback is used whenever a write to a GATT characteristic causes data to be sent to the host.
 */
void dataSentCallback(unsigned count)
{
    // NOTE: The count always seems to be 1 regardless of data.
    DEBUG("%d bytes sent to host\n\r", count);
}


/**
 * This callback is scheduled to be called periodically via a low-priority interrupt.
 */
void periodicCallback(void)
{
    //uint8_t  buf[MAX_REPLY_LEN];
    //uint32_t len = 0;
}

void error(ble_error_t err, uint32_t line)
{
    m_error_led = 1;
    DEBUG("Error %d on line number %d\n\r", err, line);
}

void PBTrigger()
{
    uint8_t  buf[MAX_REPLY_LEN];
    uint32_t len = 0;
    
    m_cmd_led = !m_cmd_led;
    
    if (m_ble.getGapState().connected) {
        len = snprintf((char*) buf, MAX_REPLY_LEN, "Button Pressed!");
        m_ble.updateCharacteristicValue(m_uart_service_ptr->getRXCharacteristicHandle(), buf, len);
    }
}

int main(void)
{
    ble_error_t err;
    Ticker      ticker;

    m_serial_port.baud(UART_BAUD_RATE);

    DEBUG("Initialising...\n\r");

    m_cmd_led      = 0;
    m_error_led    = 0;

    ticker.attach(periodicCallback, SENSOR_READ_INTERVAL_S);

    sw4Press.fall(&PBTrigger);

    #ifdef Pressure  //no Initialization because we keep in low power mode until we need to measure pressure
    //i2c.write(Press_addr_w, &PWR_OFF[0], 2, false);
    //i2c.write(Press_addr_w, &SLEEP_ON[0], 2, false);
    //i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
    #endif

    //Start BTLE Initialization Section
    m_ble.init();
    m_ble.onDisconnection(disconnectionCallback);
    m_ble.onDataWritten(dataWrittenCallback);
    m_ble.onDataSent(dataSentCallback);

    // Set the TX power in dBm units.
    // Possible values (in decreasing order): 4, 0, -4, -8, -12, -16, -20.
    err = m_ble.setTxPower(4);
    if (BLE_ERROR_NONE != err) {
        error(err, __LINE__);
    }

    // Setup advertising (GAP stuff).
    err = m_ble.setDeviceName(DEVICE_NAME);
    if (BLE_ERROR_NONE != err) {
        error(err, __LINE__);
    }

    err = m_ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    if (BLE_ERROR_NONE != err) {
        error(err, __LINE__);
    }

    m_ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);

    err = m_ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                                (const uint8_t *)SHORT_NAME,
                                                (sizeof(SHORT_NAME) - 1));
    if (BLE_ERROR_NONE != err) {
        error(err, __LINE__);
    }

    err = m_ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                                (const uint8_t *)UARTServiceUUID_reversed,
                                                sizeof(UARTServiceUUID_reversed));
    if (BLE_ERROR_NONE != err) {
        error(err, __LINE__);
    }

    m_ble.setAdvertisingInterval(Gap::MSEC_TO_ADVERTISEMENT_DURATION_UNITS(ADV_INTERVAL_MS));
    m_ble.startAdvertising();

    // Create a UARTService object (GATT stuff).
    UARTService uartService(m_ble);
    m_uart_service_ptr = &uartService;

    while (true) {
        m_ble.waitForEvent();
        
        #ifdef Pressure
        /*
        //Read color Portion from the IC
        i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
        i2c.read(Press_addr_r, &Press_Content_ReadData[0], 3, NoRepStart);
        
        BM1383_Var  = (Press_Content_ReadData[0]<<3) | (Press_Content_ReadData[1] >> 5);
        BM1383_Deci = ((Press_Content_ReadData[1] & 0x1f) << 6 | ((Press_Content_ReadData[2] >> 2)));
        BM1383_Deci = (float)BM1383_Deci* 0.00048828125;  //0.00048828125 = 2^-11
        BM1383_Pres_Conv_Out = (BM1383_Var + BM1383_Deci);   //question pending here...
        */
        #endif
        
        
        if(printQue)    //Handle Data Written Interrupt
        {
            BTLE_DataWrittenHandler();
            printQue = 0;
        }
    }
}

void BTLE_DataWrittenHandler(){    
    int i;
    if (ReceivedValue.length() == 1) {
        switch (ReceivedValue[0]) { 
            case 'F':
                len = snprintf((char*) buf, MAX_REPLY_LEN, "  Pres= %0.2f hPa", BM1383_Pres_Conv_Out);
                break;
            
            case 'r': //System Reset
                NVIC_SystemReset(); //Q. What is this? no break statement?
                                    //A. SoftReset... No need for break because this resets the program...
                break;
                
            default:
                len = snprintf((char*) buf, MAX_REPLY_LEN, "1b,ERROR");
                break;
        }
    }
    else if (ReceivedValue.length() > 1) {
        
        //BTLE Interface Code, Added 5/29/2016
        for(i = 0; i < 29; i++)
        {
            FormattedData[i] = '\0';
        }
        
        if(ReceivedValue.compare(0,4,"CAL0") == 0)
        {
			pressureCurr = 0;
			i2c.write(Press_addr_w, &PWR_ON[0], 2, false);
			i2c.write(Press_addr_w, &SLEEP_OFF[0], 2, false);
			i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
			for(i = 0; i < 10; i++)
			{
				#ifdef Pressure
				wait_ms(200);
				i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
				i2c.read(Press_addr_r, &Press_Content_ReadData[0], 3, NoRepStart);
				BM1383_TempPressure = (Press_Content_ReadData[0]<<14)|(Press_Content_ReadData[1]<<6)|(Press_Content_ReadData[2]);
				BM1383_Pres_Conv_Out = (float)BM1383_TempPressure / (float)2048;
				#endif
				pressureCurr += BM1383_Pres_Conv_Out;
			}
			i2c.write(Press_addr_w, &PWR_OFF[0], 2, false);
			i2c.write(Press_addr_w, &SLEEP_ON[0], 2, false);
			BM1383_Pres_0Level = pressureCurr/10;
			len = snprintf((char*) buf, MAX_REPLY_LEN, "BaseLvl=%.3f",BM1383_Pres_0Level);  
        }
        
        else if(ReceivedValue.compare(0,4,"CAL1") == 0)
        {
            pressureCurr = 0;
            i2c.write(Press_addr_w, &PWR_ON[0], 2, false);
            i2c.write(Press_addr_w, &SLEEP_OFF[0], 2, false);
            i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
            for(i = 0; i < 10; i++)
            {
                #ifdef Pressure
                wait_ms(200);
				i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
				i2c.read(Press_addr_r, &Press_Content_ReadData[0], 3, NoRepStart);
				BM1383_TempPressure = (Press_Content_ReadData[0]<<14)|(Press_Content_ReadData[1]<<6)|(Press_Content_ReadData[2]);
				BM1383_Pres_Conv_Out = (float)BM1383_TempPressure / (float)2048;
                #endif
                pressureCurr += BM1383_Pres_Conv_Out;
            }
            i2c.write(Press_addr_w, &PWR_OFF[0], 2, false);
            i2c.write(Press_addr_w, &SLEEP_ON[0], 2, false);
            BM1383_Pres_KnownLevel = pressureCurr/10;
            len = snprintf((char*) buf, MAX_REPLY_LEN, "KnownLv=%.3f",BM1383_Pres_KnownLevel);              
        }
        
        
        else if(ReceivedValue.compare(0,4,"HEI?") == 0)
        {
            HeightKnown_Total = HeightKnown_Foot + HeightKnown_Inches;
            slope = (HeightKnown_Total - HeightBase) / (BM1383_Pres_KnownLevel - BM1383_Pres_0Level);
            yInt = HeightBase - (slope * BM1383_Pres_0Level);
            pressureCurr = 0;
            i2c.write(Press_addr_w, &PWR_ON[0], 2, false);
            i2c.write(Press_addr_w, &SLEEP_OFF[0], 2, false);
            i2c.write(Press_addr_w, &Mode_Control[0], 2, false);
            for(i = 0; i < 10; i++)
            {
                #ifdef Pressure
                wait_ms(200);
				i2c.write(Press_addr_w, &Press_Addr_ReadData, 1, RepStart);
				i2c.read(Press_addr_r, &Press_Content_ReadData[0], 3, NoRepStart);
				BM1383_TempPressure = (Press_Content_ReadData[0]<<14)|(Press_Content_ReadData[1]<<6)|(Press_Content_ReadData[2]);
				BM1383_Pres_Conv_Out = (float)BM1383_TempPressure / (float)2048;
                #endif
                pressureCurr += BM1383_Pres_Conv_Out;
            }
            i2c.write(Press_addr_w, &PWR_OFF[0], 2, false);
            i2c.write(Press_addr_w, &SLEEP_ON[0], 2, false);
            pressureCurr = pressureCurr/10;
            
			HeightExtrapolated = ((pressureCurr * slope) + yInt)/12;
			//len = snprintf((char*) buf, MAX_REPLY_LEN, "Height=%f FT",HeightExtrapolated);
			//m_ble.updateCharacteristicValue(m_uart_service_ptr->getRXCharacteristicHandle(), buf, len);
			
			HeightExtrapolated_Foot = int(HeightExtrapolated);
			HeightExtrapolated_Inches = (HeightExtrapolated - HeightExtrapolated_Foot)* 12;
			len = snprintf((char*) buf, MAX_REPLY_LEN, "H=%.0fFT, %.0fIN",HeightExtrapolated_Foot,HeightExtrapolated_Inches);
        }
        
        else if(ReceivedValue.compare(0,2,"FT") == 0)
        {
            HeightKnown_Foot = (ReceivedValue[2]-48) * 12;  //Convert ASCII to INT, then Convert to Inches
            sprintf(FormattedData, "%2.0f", HeightKnown_Foot);
            len = snprintf((char*) buf, MAX_REPLY_LEN, "FT = %s inchs", FormattedData);
			wait_ms(200);
        }
        
        else if(ReceivedValue.compare(0,2,"IN") == 0)
        {
            if((ReceivedValue[3]>= 48) && (ReceivedValue[3]<=57)){
                HeightKnown_Inches = ((ReceivedValue[2]-48)*10) + (ReceivedValue[3]-48);    //Convert ASCII to INT, then Convert to Inches  
                
            }
            else{
                HeightKnown_Inches = (ReceivedValue[2]-48); //Convert ASCII to INT
            }
            sprintf(FormattedData, "%2.0f", HeightKnown_Inches);
            len = snprintf((char*) buf, MAX_REPLY_LEN, "IN = %s inchs", FormattedData);
			wait_ms(200);
        }
        
        else
        {
            len = snprintf((char*) buf, MAX_REPLY_LEN, "??? = %s", ReceivedValue.c_str());
            //m_ble.updateCharacteristicValue(m_uart_service_ptr->getRXCharacteristicHandle(), buf, len);
        }

    }
    else
    {
        len = snprintf((char*) buf, MAX_REPLY_LEN, "ERR:NUL");
    }
    m_ble.updateCharacteristicValue(m_uart_service_ptr->getRXCharacteristicHandle(), buf, len);
}