#include "TouchControl.h"

QueueHandle_t incomingMsg;

void feedWatchdog();

TouchControl::TouchControl(HardwareSerial &serial) : _port(serial) {

	this->customShake.powder_1 = 0;
	this->customShake.powder_2 = 0;
	this->customShake.water = 0;

	linker.serial = &this->_port;
	linker._customShake = &this->customShake;
}

void TouchControl::init()
{
	this->_port.begin(DGUS_SERIAL_BAUDRATE, SERIAL_8N1, TOUCHSCREEN_TX, TOUCHSCREEN_RX);

	incomingMsg = xQueueCreate(10, sizeof(command_t));

	xTaskCreatePinnedToCore((TaskFunction_t)this->msgProcessor, "UART MSG Processor", 5000, 
							&this->linker, 1, 
							&this->msgProcessorHandle, 1);

	xTaskCreatePinnedToCore((TaskFunction_t)this->uartRxProcess,
							"UART RX Receiver Controller",
							5000, &this->_port,
							1, &this->uartRxQueueHandle, 0);
}
void TouchControl::resetDisplay()
{
	delay(300);
	// Byte array needed to trigger a reset on the touch screen
	/* RESET DWIN SCREEN */
	// 5A A5 - 04 - 80 - EE - 5A A5
	// Reg-addr = 0xEE
	// Data to write = 5AA5
	/*********************/
	byte reset[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x04, DGUS_WRITE_REG, REG_RESET_TRIGGER, 0x5A, 0xA5};
	this->_port.write(reset, sizeof(reset));
	// if ESP32- Core Debug Level is set to "Verbose", ESP_LOGI will print information through the uart
	//... without the need of the "Serial.println()" function
	ESP_LOGI("resetDisplay()", "Display has been reset");
}



TouchControl& TouchControl::setCallback(callback_t callback)
{
	this->callback = callback;
	this->linker.cb = callback;
	return *this;
}

void TouchControl::setDisplayBrightness(uint8_t brightnessValue)
{
	if (brightnessValue > 0x40)
	{
		brightnessValue = 0x40;
	}
	// 5AA5 03 80 01 0A
	byte brightness[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x03, DGUS_WRITE_REG, REG_SCREEN_BRIGHT, brightnessValue};
	this->_port.write(brightness, sizeof(brightness));
}

void TouchControl::loadWifiInfo(char *ssid, char* ipAddress)
{
	_clearTextField(WIFI_SSID_ADDR);
	_clearTextField(WIFI_IP_ADDR);
	_setTextField(WIFI_SSID_ADDR, ssid);
	_setTextField(WIFI_IP_ADDR, ipAddress);
}
void TouchControl::loadFirmwareInfo(char *version)
{
	_clearTextField(FIRMWARE_VERSION_ADDR);
	_setTextField(FIRMWARE_VERSION_ADDR, version);
}


/*(Write)change RTC Time */
// 5A A5 - 0A 	   - 	80 	   - 	1F    -   	     5A 		-   21    11     1D       30   23  48 00
//  FH	 - 10 byte - Write Reg - RTC- Register - Flag to set RTC - Year Month Week-number Day Hour Min Sec
void TouchControl::loadRtcInfo(uint8_t year, uint8_t month, uint8_t week_number, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
	byte time_and_date[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x0A, DGUS_WRITE_REG, REG_RTC_COM_ADJ, 0x5A, 
							year, month, week_number, day, hour, min, sec};

	this->_port.write(time_and_date, sizeof(time_and_date));
}

customShake_t TouchControl::getCustomShakeValues()
{
	return this->customShake;
}

void TouchControl::returnToMainMenu()
{
	uint32_t page = PAGE_MAIN_MENU;
	uint8_t page_H = (page >> 8);
	uint8_t page_L = page;

	byte pageBuff[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x04, DGUS_WRITE_REG, REG_PIC_ID, page_H, page_L};
	this->_port.write(pageBuff, sizeof(pageBuff));
}


void TouchControl::uartRxProcess(HardwareSerial *serial)
{
	dgusPacket_t msg;
	command_t command;
	TickType_t _wait = pdMS_TO_TICKS(5);

	while (true)
	{
		feedWatchdog();
		if (serial->available())
		{
			// preprare the msg.data array by clearing them with 0x00
			memset(msg.data, 0x00, sizeof(msg.data));
			memset(command.data, 0x00, sizeof(command.data));

			// read a 16 byte block into msg.data array
			serial->readBytes(msg.data, 4);

			if(msg.fields.byteCount < 11){
				serial->readBytes(command.data, msg.fields.byteCount - 1);
				command.command = msg.fields.command;
				xQueueSend(incomingMsg, &command, _wait);
			}
			else{
				serial->readBytes(command.data, sizeof(command.data));
				command.command = msg.fields.command;
				xQueueSend(incomingMsg, &command, _wait);
			}
		}
	}
}


void TouchControl::msgProcessor(varLinker_t * varLinker)
{
	command_t payload;
	BaseType_t qStatus;
	TickType_t _wait = pdMS_TO_TICKS(10);

	formatted_command_t toProcessCommand;

	while (true)
	{
		qStatus = xQueueReceive(incomingMsg, &payload, _wait);
		if (qStatus == pdPASS)
		{
			switch (payload.command)
			{
			case DGUS_WRITE_REG:
				ESP_LOGI("", "Write Reg: %02X, %02X, data...", payload.data[0], payload.data[1]);
				break;
			case DGUS_READ_REG:
				memset(toProcessCommand.data, 0x00, sizeof(toProcessCommand.data));				
				memcpy(toProcessCommand.fields.comm_data, payload.data+2, sizeof(toProcessCommand.fields.comm_data) - 2);
				if(payload.data[0] == 0x01){
					toProcessCommand.fields.comm_source = brightness_value;
				}
				else{
					toProcessCommand.fields.comm_source = payload.data[0];
				}
				
				toProcessCommand.fields.comm_length = payload.data[1];
				/*ESP_LOGI("", "%02X, %02X, %02X%02X%02X%02X%02X%02X%02X%02X%02X", 
							toProcessCommand.fields.comm_source, toProcessCommand.fields.comm_length, 
							toProcessCommand.fields.comm_data[0], toProcessCommand.fields.comm_data[1], toProcessCommand.fields.comm_data[2],
							toProcessCommand.fields.comm_data[3], toProcessCommand.fields.comm_data[4], toProcessCommand.fields.comm_data[5],
							toProcessCommand.fields.comm_data[6], toProcessCommand.fields.comm_data[7], toProcessCommand.fields.comm_data[8]
							);*/
				break;
			case DGUS_WRITE_VP:
				ESP_LOGI("", "Write VP: %02X%02X, data...", payload.data[0], payload.data[1]);
				break;

			case DGUS_READ_VP:
				memset(toProcessCommand.data, 0x00, sizeof(toProcessCommand.data));
				memcpy(toProcessCommand.fields.comm_data, payload.data+3, sizeof(toProcessCommand.fields.comm_data)-3);
				toProcessCommand.fields.comm_source = (payload.data[0]<<8) + payload.data[1];
				toProcessCommand.fields.comm_length = payload.data[2];
				/*ESP_LOGI("", "%04X, %02X, %02X%02X%02X%02X%02X%02X%02X%02X%02X", 
							toProcessCommand.fields.comm_source, toProcessCommand.fields.comm_length, 
							toProcessCommand.fields.comm_data[0], toProcessCommand.fields.comm_data[1], toProcessCommand.fields.comm_data[2],
							toProcessCommand.fields.comm_data[3], toProcessCommand.fields.comm_data[4], toProcessCommand.fields.comm_data[5],
							toProcessCommand.fields.comm_data[6], toProcessCommand.fields.comm_data[7], toProcessCommand.fields.comm_data[8]
							);*/
				break;
			
			default:
				break;
			}


			switch(toProcessCommand.fields.comm_source)
			{
				case brightness_value: //0x000A
					_loadDisplayBrightness(toProcessCommand.fields.comm_data[0], varLinker->serial);
				break;
				
				case update_brightness: //0x0320
					_setDisplayBrightness(toProcessCommand.fields.comm_data[1], varLinker->serial);
				break;

				case save_powder_1_g: // 0x0080
				{
					varLinker->_customShake->powder_1 = atoi((const char*)toProcessCommand.fields.comm_data);
				}
				break;
				case save_powder_2_g: // 0x0120
					varLinker->_customShake->powder_2 = atoi((const char*)toProcessCommand.fields.comm_data);
				break;
				case save_water_ml: // 0x0160
					varLinker->_customShake->water = atoi((const char*)toProcessCommand.fields.comm_data);
				break;

				case normal_command: // 0x0020
					uint16_t cmd = (toProcessCommand.fields.comm_data[0]<<8) + toProcessCommand.fields.comm_data[1];
					ESP_LOGI("","%04X", cmd);
					switch(cmd){
						case load_brightness: // 0x0009
							readBrightnessValue(varLinker->serial);
						break;

						case clean: // 0x0001
							_changePageto(varLinker->serial, PAGE_CLEAN);
							varLinker->cb("clean start");
						break;

						case clean_abort: // 0x0002
							_changePageto(varLinker->serial, PAGE_MAIN_MENU);
							varLinker->cb("clean abort");
						break;

						case express_shake_1: // 0x0003
							_changePageto(varLinker->serial, PAGE_SHAKE_PREPARING);
							varLinker->cb("express shake 1");
						break;

						case express_shake_2: // 0x0004
							_changePageto(varLinker->serial, PAGE_SHAKE_PREPARING);
							varLinker->cb("express shake 2");
						break;

						case custom_shake_start: // 0x0005
							_changePageto(varLinker->serial, PAGE_SHAKE_PREPARING);
							varLinker->cb("custom shake start");
						break;

						case stop_shake_prep: // 0x0006
							_changePageto(varLinker->serial, PAGE_MAIN_MENU);
							varLinker->cb("stop shake prep");
						break;

						case load_wifi_info: // 0x0007
							varLinker->cb("load wifi info");
						break;

						case load_firmware_info: // 0x0008
							varLinker->cb("load firmware info");
						break;
						case rtc_info:
							varLinker->cb("rtc info");
						break;
					}
				break;
			}

			memset(toProcessCommand.data, 0x00, sizeof(toProcessCommand.data));
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

uint8_t TouchControl::readBrightnessValue(HardwareSerial *serial)
{
	// 5AA5 03 81 01 01
	byte brightness[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x03, DGUS_READ_REG, REG_SCREEN_BRIGHT, 0x01};
	serial->write(brightness, sizeof(brightness));
	return 0x00;
}

// 5A A5 - 04 - 83 - 0160 - 05
// read 10 bytes from 0x0160(water in ml)


void TouchControl::_clearTextField(uint16_t addr)
{
	uint8_t addr_H = (addr >> 8);
	uint8_t addr_L = addr; 

	byte ssid_txtfield[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x17, DGUS_WRITE_VP, addr_H, addr_L, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	this->_port.write(ssid_txtfield, sizeof(ssid_txtfield));
}

void TouchControl::_setTextField(uint16_t addr, char *text)
{

	uint8_t addr_H = (addr >> 8);
	uint8_t addr_L = addr; 
	uint8_t textLength = strlen(text);
	if(textLength > 20){
		textLength = 20;
	}
	char buff[20];

	memset(buff, 0x00, sizeof(buff));
	memcpy(buff, text, textLength);


	byte ssid_txtfield[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x17, DGUS_WRITE_VP, addr_H, addr_L, 
	buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7], buff[8], buff[9], buff[10],
	buff[11], buff[12], buff[13], buff[14], buff[15], buff[16], buff[17], buff[18], buff[19]
	};

	this->_port.write(ssid_txtfield, sizeof(ssid_txtfield));

}

/***** Switch Page *********/

// 		5A A5 	- 	04 		 - 		80 - 				03 	 - 	0001
// Frame Header - Byte count - Command(REG WRITE) - REG ADDR - PAGE
/***********************/
void TouchControl::_changePageto(HardwareSerial *serial, uint16_t page)
{
	uint8_t page_H = (page >> 8);
	uint8_t page_L = page;

	byte pageBuff[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x04, DGUS_WRITE_REG, REG_PIC_ID, page_H, page_L};
	serial->write(pageBuff, sizeof(pageBuff));
}

void TouchControl::_setDisplayBrightness(uint8_t brightnessValue, HardwareSerial *serial)
{
	uint8_t new_value = map(brightnessValue, 0x00, 0x64, 0x00, 0x40);

	// 5AA5 03 80 01 0A
	byte brightness[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x03, DGUS_WRITE_REG, REG_SCREEN_BRIGHT, new_value};
	serial->write(brightness, sizeof(brightness));
}

// 5AA5 05 82 0320 0009
void TouchControl::_loadDisplayBrightness(uint8_t brightnessValue, HardwareSerial *serial)
{
	if (brightnessValue > 0x40)
	{
		brightnessValue = 0x40;
	}
	uint8_t new_value = map(brightnessValue, 0x00, 0x40, 0x00, 0x64);

	ESP_LOGI("", "value: %02X", new_value);

	byte brightness[] = {FRAME_HEADER_H, FRAME_HEADER_L, 0x05, DGUS_WRITE_VP, 0x03, 0x20, 0x00, new_value};
	serial->write(brightness, sizeof(brightness));
}

void feedWatchdog()
{
	// Deze 3 commandos worden gebruikt om de "Wachtdog" te resetten
	TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
	TIMERG0.wdt_feed = 1;
	TIMERG0.wdt_wprotect = 0;
	// Anders zal de "Wachtdog" getriggeerd worden en de ESP32 resetten
}