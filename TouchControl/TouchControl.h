#ifndef TOUCHCONTROL_H
#define TOUCHCONTROL_H

#include <HardwareSerial.h>
#include <Arduino.h>
#include <esp_task.h>
#include <soc/timer_group_struct.h>
#include <soc/timer_group_reg.h>

// Modifiable pointer with const data (Read-only data)
// uint8_t const * pData = (uint8_t*) 0x400;
// You can read this from the pData name:
// pData is a POINTER(*) with CONSTANT(const) data of type UINT8_T

// Constant pointer with modifiable data (Read-only pointer)
// uint8_t * const pData = (uint8_t*) 0x400;
// The same rule applies here:
// pData is a CONSTANT(const) POINTER(*) with data of type UINT8_T

// Constant pointer with const data (read-only pointer with read-only data)
// uint8_t const * const pData = (uint8_t) 0x400;
// use case of this => Status register of MCU; The pointer and the data must not be change but it can be read.

#include <functional>


typedef struct 
{
	uint32_t powder_1;
	uint32_t powder_2;
	uint32_t water;
}customShake_t;




class TouchControl
{

private:

typedef void (*callback_t)(char*);
callback_t callback; //private "callback" function declared;

#define TOUCHSCREEN_TX 35U
#define TOUCHSCREEN_RX 23U
#define FRAME_HEADER 0x5AA5U
#define FRAME_HEADER_H 0x5AU
#define FRAME_HEADER_L 0xA5U
#define DGUS_WRITE_REG 0x80U
#define DGUS_READ_REG 0x81U
#define DGUS_WRITE_VP 0x82U
#define DGUS_READ_VP 0x83U
#define DGUS_SERIAL_BAUDRATE 115200U
#define REG_VERSION 0x00U
#define REG_SCREEN_BRIGHT 0x01U
#define REG_BUZZ_TIME 0x02U
#define REG_PIC_ID 0x03U // 2 bytes
#define REG_TP_FLAG 0x05U
#define REG_TP_STATUS 0x06U
#define REG_TP_POSITION 0x07U // 4 byes
#define REG_TPC_ENABLE 0x0BU
#define REG_RUN_TIME_L 0x0CU // 4 bytes from 0x0C to 0x0F
#define REG_RUN_TIME_H 0x0FU
#define REG_RTC_COM_ADJ 0x1FU
#define REG_RTC_NOW 0x20U	   // 16 bytes
#define REG_RESET_TRIGGER 0xEE // 2 bytes -> Reset Screen by writing 0x5AA5 to this address

#define POWDER_1_ADDR 0x0080
#define POWDER_2_ADDR 0x0120
#define WATER_ADDR 0x0160
#define WIFI_SSID_ADDR 0x0200
#define WIFI_IP_ADDR 0x0240
#define FIRMWARE_VERSION_ADDR 0x0280
#define BRIGHTNESS_ADDR 0x0320

#define PAGE_MAIN_MENU 0x0000
#define PAGE_SHAKE_SELECT 0x0001
#define PAGE_SHAKE_PREPARING 0x0002
#define PAGE_CUSTOM 0x0003
#define PAGE_SHAKE_EXPRESS 0x0004
#define PAGE_NUMBERPAD 0x0005
#define PAGE_CLEAN 0x0006
#define PAGE_SETTING 0x0007
#define PAGE_WIFI 0x0008
#define PAGE_DATE_TIME 0x0009
#define PAGE_FIRMWARE 0x000A
#define PAGE_BRIGHTNESS 0x000B
#define PAGE_NEW_FIRMWARE 0x000C


	typedef union
	{
		struct
		{
			uint16_t H : 8;
			uint16_t L : 8;
		};
		uint16_t value;
	} dgusFrameHeader_t;

	typedef union
	{
		struct
		{
			uint16_t H : 8;
			uint16_t L : 8;
		};
		uint16_t value;
	} dgusVpAddress_t;

	typedef union
	{
		struct
		{
			dgusFrameHeader_t frameHeader;
			uint8_t byteCount;
			uint8_t command;
		}fields;
		byte data[sizeof(fields)];
	} dgusPacket_t;

	typedef enum
	{
		clean 				= 0x0001,
		clean_abort 		= 0x0002,
		express_shake_1 	= 0x0003,
		express_shake_2 	= 0x0004,
		custom_shake_start	= 0x0005,
		stop_shake_prep		= 0x0006,
		load_wifi_info		= 0x0007,
		load_firmware_info	= 0x0008,
		load_brightness		= 0x0009,
		normal_command		= 0x0020,
		save_powder_1_g		= 0x0080,
		save_powder_2_g		= 0x0120,
		save_water_ml		= 0x0160,
		update_brightness	= 0x0320,
		brightness_value	= 0x000A,
		rtc_info			= 0x000B
	}screenCommands_t;

	typedef union
	{
		struct
		{
			uint8_t command;
			uint8_t data[12];
		};
	}command_t;

	typedef union 
	{
		struct
		{
			uint16_t comm_source;
			uint8_t comm_length;
			uint8_t comm_data[9];
		}fields;
		uint8_t data[sizeof(fields)];
	}formatted_command_t;

	//used to store values for the Custom Shake
	customShake_t customShake;
	
	// This struct is used to pass multiple private variables into 
	// the static "processes" as a "pvParameter"
	typedef struct 
	{
		HardwareSerial *serial;
		customShake_t *_customShake;
		callback_t cb;
	}varLinker_t;
	

	HardwareSerial _port;
	dgusPacket_t _dgusMsg;
	TaskHandle_t uartRxQueueHandle;
	TaskHandle_t uartTxQueueHandle;
	TaskHandle_t msgProcessorHandle;
	varLinker_t linker;

	static void uartRxProcess(HardwareSerial * serial);
	static void msgProcessor(varLinker_t * varLinker);
	static uint8_t readBrightnessValue(HardwareSerial * serial);
	static void _setDisplayBrightness(uint8_t brightnessValue, HardwareSerial * serial);
	static void _loadDisplayBrightness(uint8_t brightnessValue, HardwareSerial * serial);
	void _clearTextField(uint16_t addr);
	void _setTextField(uint16_t addr, char *text);
	static void _changePageto(HardwareSerial *serial, uint16_t page);

public:

	
	/**
	 * @brief Construct a new Touch Control object, one parameter is needed.
	 * 
	 * @param serial Pass the current HardwareSerial object to this constructor
	 */
	TouchControl(HardwareSerial &serial);

	/**
	 * @brief Initializes communication with the screen and processing of messages
	 * 
	 */
	void init();

	/**
	 * @brief Resets the Touch Display immediately when this function is called
	 * 
	 */
	void resetDisplay();

	/**
	 * @brief Set the Display Brightness by passing an uint8_t value
	 * 
	 * @param brightnessValue Pass an unsigned value from 1 to 100 presenting 1% to 100%
	 */
	void setDisplayBrightness(uint8_t brightnessValue);

	/**
	 * @brief Set the text to display under the WiFi settings
	 * 
	 * @param ssid is a char array containing a maximum of 20 characters. 
	 * For example: "AP WiFi"
	 * @param ipAddress is a char array containing a maximum of 20 characters.
	 * For example: "192.168.1.1"
	 */
	void loadWifiInfo(char *ssid, char* ipAddress);

	/**
	 * @brief Set the text to display under the Firmware settings
	 * 
	 * @param version is a char array containing a maximum of 20 characters.
	 * For example: "v1.1.3"
	 */
	void loadFirmwareInfo(char *version);

	/**
	 * @brief Configure the RTC with a new Date and Time
	 * All of the parameters must be in hexadecimal form.
	 * For example: Year 2021 should be pass as => 0x21
	 * 
	 * @param year is a unsigned int
	 * @param month is a unsigned int
	 * @param week_number is a unsigned int
	 * @param day is a unsigned int
	 * @param hour is a unsigned int
	 * @param min is a unsigned int
	 * @param sec is a unsigned int
	 * 
	 */
	void loadRtcInfo(uint8_t year, uint8_t month, uint8_t week_number, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

	/**
	 * @brief Get the Custom Shake Values object
	 * 
	 * @return customShake_t
	 */
	customShake_t getCustomShakeValues();

	/**
	 * @brief Calling this function will switch the curren page to the Main menu in the LCD Display
	 * 
	 */
	void returnToMainMenu();

	/**
	 * @brief Sets a callback function to which commands will be passed.
	 * 
	 * @param callback This is a declared function of type void with 1 argument of type char*
	 */
	TouchControl& setCallback(callback_t callback);
};

#endif