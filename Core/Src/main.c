/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	HAT_IDX_UP = 0,
	HAT_IDX_UR = 1,
	HAT_IDX_R = 2,
	HAT_IDX_DR = 3,
	HAT_IDX_D = 4,
	HAT_IDX_DL = 5,
	HAT_IDX_L = 6,
	HAT_IDX_UL = 7
} HatIndex;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WINWING_MAX_PACKET_SIZE  8
#define VIRPIL_PACKET_SIZE	14
#define VIRPIL_TIMEOUT_MS 1000
#define CALIBRATION_CHECK_INTERVAL 500 //ms
#define DIAL_HANDLE_INTERVAL_MS 10 //ms
#define MIN_CALIBRATION_DURATION 5000 //ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// Calibration data structure
typedef struct {
    uint16_t MinThumbStickX;
    uint16_t MaxThumbStickX;
    uint16_t MinThumbStickY;
    uint16_t MaxThumbStickY;
    uint16_t MinTwistZ;
    uint16_t MaxTwistZ;
    uint16_t MinSlider;
    uint16_t MaxSlider;
    uint16_t SliderDetent1;
    uint16_t SliderDetent2;
    uint32_t crc32; // For data integrity
} CalibrationData;

// Flash storage address (last 16KB sector for STM32F411, 0x08060000)
#define CALIBRATION_FLASH_ADDR ((uint32_t)0x08060000)

CalibrationData calibData = {
    .MinThumbStickX = 10,
    .MaxThumbStickX = 4086,
    .MinThumbStickY = 10,
    .MaxThumbStickY = 4086,
    .MinTwistZ = 10,
    .MaxTwistZ = 4086,
    .MinSlider = 16,
    .MaxSlider = 4086,
	.SliderDetent1 = 1250,
	.SliderDetent2 = 3400,
    .crc32 = 0
};

uint16_t CurrThumbStickX = 2048,
        CurrThumbStickY = 2048,
        CurrTwistZ = 2048,
        CurrSlider = 2086;

//        PrevSlider = 0, PrevTwistZ = 0, PrevThumbStickX = 0, PrevThumbStickY = 0;


uint8_t PrevDialBits = 0;
uint8_t RedDown = 0;
uint8_t BlackDown = 0;
uint8_t DialDown = 0;
uint32_t lastBlink = 0;
uint32_t lastCheck = 0;
uint32_t blinkDelay = 0;
uint32_t DialHandleTime = 0;
uint8_t DialHandleTicks = 0;

typedef struct
{
	uint8_t ReqSize;
	uint8_t RespSize;
	uint8_t Data[WINWING_MAX_PACKET_SIZE];
} tCache;

uint8_t BS_Hat = 0, BS_Center = 0, BS_Side = 0, BS_Thumb = 0;

typedef struct
{
	const char *Name;
	uint8_t VP_Byte;
	uint8_t VP_Bit;
	uint8_t WW_Byte;
	uint8_t WW_Mask;
	uint8_t WW_Push;
	uint8_t *pBS;
	uint8_t *pButton;
} tButtonMap;

uint8_t VP_Buffer[VIRPIL_PACKET_SIZE] =
{ 0xFF };
uint8_t WW_Request[WINWING_MAX_PACKET_SIZE] =
{ 0 };
volatile uint16_t WWRequestLen = 0;

#define SERIAL_NO_1 0x7f
#define SERIAL_NO_2 0xdb
#define SERIAL_NO_3 0x0c
#define SERIAL_NO_4 0x26

#define WW_A0_INDEX 20
#define WW_THUMBX_INDEX 22
#define WW_THUMBY_INDEX 23
#define WW_ZAXIS_INDEX 24
#define WW_SLIDER_INDEX 25

tCache WW_Cache[] =
		{
// Ping pong?
				{ 1, 1,
				{ 0x00 } },

				// Serial and verisons
				{ 1, 5,
				{ 0x01, 0x0D, 0x01, 0x30, 0x51 } }, //HW ver in hex, sw ver in dec??
				{ 1, 5,
				{ 0x02, 0x07, 0xBE, 0x14, 0x01 } }, //SW ver in hex,
				{ 1, 7,
				{ 0x03, SERIAL_NO_4, SERIAL_NO_3, SERIAL_NO_2, SERIAL_NO_1,
						0x12, 0x30 } }, //serial parts

				{ 4, 8,
				{ 0x05, 0x00, 0x10, 0x01, 0x55, 0x55, 0x55, 0x55 } }, //EXTension enable?
				{ 4, 8,
				{ 0x05, 0x04, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 } }, //??
				{ 4, 8,
				{ 0x05, 0x91, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00 } }, //ADD_ONFunction testUnqualified - triggers firmware update! when changed
				{ 4, 8,
				{ 0x05, 0x98, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00 } }, //ADD_ONOnboard testUnqualified
				{ 4, 8,
				{ 0x05, 0x9c, 0x00, 0x00, SERIAL_NO_1, SERIAL_NO_2, SERIAL_NO_3,
				SERIAL_NO_4 } }, //serial part 1
				{ 4, 8,
				{ 0x05, 0xa0, 0x00, 0x00, 0x4E, 0x62, 0x30, 0x12 } }, //serial part 2
				{ 4, 8,
				{ 0x05, 0xa4, 0x00, 0x00, 0x4b, 0x64, 0x06, 0x25 } }, //serial part 3
				{ 4, 8,
				{ 0x05, 0xac, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00 } },
				{ 4, 8,
				{ 0x05, 0xf0, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff } },

//0x05 0xF8 0x00 0x01 //modify trigger pos??
//0x06 0xF8 0x00 0x01 //modify trigger pos??
//0x49 0x00 0x00-0xFF //Vibration intensity

				{ 4, 8,
				{ 0x05, 0x91, 0x01, 0x00, 0xFF, 0x01, 0xFF, 0xFF } },
				{ 4, 8,
				{ 0x05, 0xAC, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 } },
				{ 4, 8,
				{ 0x05, 0xA8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x11 } },
				{ 4, 8,
				{ 0x05, 0xB8, 0x00, 0x00, 0xA3, 0x07, 0x00, 0x00 } },
				{ 4, 8,
				{ 0x05, 0x98, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 } },

				// No idea what these are
				{ 1, 0,
				{ 0xB0 } },
				{ 1, 2,
				{ 0x18, 0x00 } },

				// Digital buttons
				{ 1, 8,
				{ 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
				{ 1, 8,
				{ 0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },

				// Analog axes
				{ 2, 6,
				{ 0x46, 0x03, 0x00, 0x08, 0xFF, 0x0F } }, // thumb x
				{ 2, 6,
				{ 0x46, 0x04, 0x00, 0x08, 0xFF, 0x0F } }, // thumb y
				{ 2, 6,
				{ 0x46, 0x05, 0x00, 0x00, 0xFF, 0x0F } }, // z-axis
				{ 2, 6,
				{ 0x46, 0x06, 0x00, 0x00, 0xFF, 0x0F } }, // slider

				// Analog raw ???
				{ 2, 8,
				{ 0x45, 0x03, 0x7c, 0x08, 0x00, 0x00, 0x00, 0x00 } },
				{ 2, 8,
				{ 0x45, 0x04, 0xd2, 0x07, 0x00, 0x00, 0x00, 0x00 } },
				{ 2, 8,
				{ 0x45, 0x05, 0xbc, 0xaf, 0x01, 0x00, 0x00, 0x00 } },
				{ 2, 8,
				{ 0x45, 0x06, 0x86, 0x0e, 0x00, 0x00, 0x00, 0x00 } }, };

size_t cache_count = sizeof(WW_Cache) / sizeof(tCache);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		;  // wait for the counter to reach the us input in the parameter
}

int _write(int file, char *ptr, int len)
{
#ifdef DEBUG
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
		ITM_SendChar(*ptr++);
#endif //DEBUG
	return len;
}

static inline void printHexFrame(const char *prefix, const uint8_t *b, size_t n)
{
	printf("%s", prefix);
	for (size_t i = 0; i < n; i++)
	{
		printf(" 0x%02X", b[i]);
	}
	printf("\n");
}

static inline void printBinByte(const uint8_t *b, size_t n)
{
	printf(" ");
	for (size_t i = 0; i < n; i++)
	{
		for (int j = 7; j >= 0; j--)
		{
			printf("%d", (b[i] >> j) & 0x01);
		}
	}
}
//#define DEBUG_VIRPIL

#ifdef DEBUG_VIRPIL
#ifdef DEBUG
uint8_t prev_VP_Button_Buffer[14] = {0};
//uint8_t virpil_byte = 0xff;
#endif //DEBUG
#endif //DEBUG_VIRPIL

uint32_t LastInvalidVirpilFrameTime = 0;

static uint8_t readVirpilDataSync()
{
	uint8_t ret = 1;
	static const uint8_t DUMMY = 0xFF;
	HAL_GPIO_WritePin(VIRPIL_NSS_GPIO_Port, VIRPIL_NSS_Pin, GPIO_PIN_RESET);
	delay_us(10);
//	ret = HAL_SPI_Receive(&hspi2, VP_Buffer, VIRPIL_PACKET_SIZE, 100);
	for (uint16_t i = 0; i < VIRPIL_PACKET_SIZE; i++)
	{
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &DUMMY, &VP_Buffer[i], 1, 2);
		delay_us(10);
	}
	delay_us(10);
	HAL_GPIO_WritePin(VIRPIL_NSS_GPIO_Port, VIRPIL_NSS_Pin, GPIO_PIN_SET);

	
	if (VP_Buffer[0] == 0 || VP_Buffer[1] == 0 || VP_Buffer[2] == 0 || VP_Buffer[3] == 0 || VP_Buffer[13] != 0)
	{
		if (LastInvalidVirpilFrameTime == 0)
		{
			printf("Invalid Virpil frame received, pause processing.\n");
			printHexFrame("[VIRPIL INVALID]", VP_Buffer, VIRPIL_PACKET_SIZE);
		}
		LastInvalidVirpilFrameTime = HAL_GetTick();
		blinkDelay = 50;
		ret = 0;
	}
	else if (LastInvalidVirpilFrameTime != 0)
	{
		if (HAL_GetTick() - LastInvalidVirpilFrameTime > VIRPIL_TIMEOUT_MS)
		{
			printf("Valid Virpil frame received, resuming.\n");
			LastInvalidVirpilFrameTime = 0;
			blinkDelay = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		}
		else
			ret = 0;
	}

	//printHexFrame("[VIRPIL]", VP_Buffer, VIRPIL_PACKET_SIZE);
#ifdef DEBUG_VIRPIL
#ifdef DEBUG
	if (memcmp(prev_VP_Button_Buffer, VP_Buffer, sizeof(prev_VP_Button_Buffer)) != 0)
	{
		memcpy(prev_VP_Button_Buffer, VP_Buffer, sizeof(prev_VP_Button_Buffer));
		printHexFrame("[VIRPIL BB]", prev_VP_Button_Buffer, sizeof(prev_VP_Button_Buffer));
//		printf("[VIRPIL Slider] 0x%02X 0x%02X" , VP_Buffer[5], VP_Buffer[4]);
//		printBinByte(&VP_Buffer[5], 1);
//		printBinByte(&VP_Buffer[4], 1);
//		printf("\n");
		//if (virpil_byte != VP_Buffer[3])
		//{
		//	virpil_byte = VP_Buffer[3];
		//	printf("VP3=0x%02X\n", virpil_byte);
		//}
	}
#endif //DEBUG
#endif //DEBUG_VIRPIL
	return ret;
}

tButtonMap ButtonMap[] =
{
{ "Trigger Second Detent", 0, 3, 3, 0x10 },
{ "Trigger First Detent", 0, 2, 3, 0x08 },
{ "Trigger Guard Active", 0, 1, 3, 0x01 },

{ "Black Button Press", 0, 5, 7, 0x08, 0, 0, &BlackDown },
{ "Red Button Press", 1, 3, 5, 0x08, 0, 0, &RedDown },
{ "Pinky Button Press", 3, 5, 3, 0x20 },

//	{"Hat Up"   , 1, 5, 1, 0x00, 0, 0},
//	{"Hat Right", 1, 6, 1, 0x02, 0, 0},
//	{"Hat Down" , 1, 7, 1, 0x04, 0, 0},
//	{"Hat Left" , 2, 0, 1, 0x06, 0, 0},
		{ "Hat Press", 1, 4, 5, 0x04, 1, &BS_Hat },

		{ "Thumbstick Press", 0, 4, 6, 0x02 },

		{ "Dial Press", 2, 4, 5, 0x10, 0, 0, &DialDown },

		{ "Center Up", 0, 7, 6, 0x80, 0, &BS_Center },
		{ "Center Down", 1, 1, 7, 0x02, 0, &BS_Center },
		{ "Center Left", 1, 2, 7, 0x04, 0, &BS_Center },
		{ "Center Right", 1, 0, 7, 0x01, 0, &BS_Center },
		{ "Center Press", 0, 6, 6, 0x40, 1, &BS_Center },

		{ "Thumb Fwd", 3, 1, 4, 0x10, 0, &BS_Thumb },
		{ "Thumb Back", 3, 3, 4, 0x04, 0, &BS_Thumb },
		{ "Thumb Left", 3, 4, 4, 0x02, 0, &BS_Thumb },
		{ "Thumb Right", 3, 2, 4, 0x08, 0, &BS_Thumb },
		{ "Thumb Press", 3, 0, 4, 0x01, 1, &BS_Thumb },

		{ "Side Fwd", 2, 2, 4, 0x40, 0, &BS_Side },
		{ "Side Back", 2, 3, 5, 0x01, 0, &BS_Side },
		{ "Side Press", 2, 1, 4, 0x20, 1, &BS_Side } };

size_t ButtonCount = sizeof(ButtonMap) / sizeof(tButtonMap);

#define VP_ISSET(a,b) ((VP_Buffer[a] >> b) & 0x01) == 0
#define WW_SET(a,b) WW_Cache[WW_A0_INDEX].Data[a] |= b
#define WW_RESET(a,b) WW_Cache[WW_A0_INDEX].Data[a] &= ~b

uint32_t CalibrationEnterTime = 0;
uint8_t CalibrationJustEntered = 0;
uint8_t CalibrationActive = 0;

static inline uint8_t ww_hat_code_from_index(int idx)
{
	if (idx < 0)
		return 0x0F;               // neutral
	// high nibble: one (or two) cardinal bits based on idx
	uint8_t hi = 0;
	if (idx == 0 || idx == 1 || idx == 7)
		hi |= 0x10; // Up
	if (idx == 1 || idx == 2 || idx == 3)
		hi |= 0x20; // Right
	if (idx == 3 || idx == 4 || idx == 5)
		hi |= 0x40; // Down
	if (idx == 5 || idx == 6 || idx == 7)
		hi |= 0x80; // Left
	return (uint8_t) (hi | (idx & 0x0F));
}

static inline uint8_t ww_hat_code_from_bools(int up, int right, int down,
		int left)
{
	// cancel opposites (if both pressed, treat as neither)
	int v = (up && !down) ? 1 : (down && !up) ? -1 : 0;
	int h = (right && !left) ? 1 : (left && !right) ? -1 : 0;

	int idx = -1; // neutral by default
	if (v == 0 && h == 0)
		idx = -1;
	else if (v > 0 && h == 0)
		idx = HAT_IDX_UP;
	else if (v > 0 && h > 0)
		idx = HAT_IDX_UR;
	else if (v == 0 && h > 0)
		idx = HAT_IDX_R;
	else if (v < 0 && h > 0)
		idx = HAT_IDX_DR;
	else if (v < 0 && h == 0)
		idx = HAT_IDX_D;
	else if (v < 0 && h < 0)
		idx = HAT_IDX_DL;
	else if (v == 0 && h < 0)
		idx = HAT_IDX_L;
	else if (v > 0 && h < 0)
		idx = HAT_IDX_UL;

	return ww_hat_code_from_index(idx);
}

static inline void ww_write_hat(uint8_t code)
{
	WW_Cache[WW_A0_INDEX].Data[1] = code;   // POV code
	WW_Cache[WW_A0_INDEX].Data[2] = 0x00;   // per your captures
	WW_Cache[WW_A0_INDEX].Data[3] = 0x42;   // constant in your captures
}

static inline uint16_t map_vp_to_ww(uint16_t cur, uint16_t minv, uint16_t maxv)
{
	if (minv == maxv)
		return 0;                 // avoid /0; treat as 0
	if (minv > maxv)
	{
		uint16_t t = minv;
		minv = maxv;
		maxv = t;
	} // handle reversed cal

	// clamp
	if (cur <= minv)
		return 0;
	if (cur >= maxv)
		return 0x0FFF;

	// round-to-nearest: ((cur-min)/(span)) * 4095
	const uint32_t span = (uint32_t) maxv - (uint32_t) minv;
	const uint32_t num = ((uint32_t) cur - (uint32_t) minv) * 0x0FFFu
			+ (span >> 1);
	return (uint16_t) (num / span);
}

int8_t DialDirection = 0;
static const int8_t DialDirections[16] =
{
/* prev<<2 | curr: 00,01,10,11 */
/* 00-> */0, +1, -1, 0,
/* 01-> */-1, 0, 0, +1,
/* 10-> */+1, 0, 0, -1,
/* 11-> */0, -1, +1, 0 };

void updateWWCache(void)
{
	// 2) Reset group counters each frame
	BS_Hat = BS_Center = BS_Side = BS_Thumb = 0;

	//process Hat first. hat is different from buttons

	int vp_up = VP_ISSET(1, 5);
	int vp_down = VP_ISSET(1, 7);
	int vp_left = VP_ISSET(2, 0);
	int vp_right = VP_ISSET(1, 6);

	uint8_t hatCode = ww_hat_code_from_bools(vp_up, vp_right, vp_down, vp_left);
	ww_write_hat(hatCode);

	if (vp_up || vp_down || vp_left || vp_right)
		BS_Hat = 1;

	// 3) First pass: compute group occupancy (boolean)
	for (size_t i = 0; i < ButtonCount; i++)
	{
		tButtonMap *m = &ButtonMap[i];
		if (m->pButton)
			*(m->pButton) = 0;
		if (!m->pBS)
			continue;
		if (m->WW_Push)
			continue;                 // only non-push members contribute
		if (VP_ISSET(m->VP_Byte, m->VP_Bit))
		{
			*m->pBS = 1;                   // boolean “someone in group is held”
		}
	}

	// 4) Second pass: write WW cache
	for (size_t i = 0; i < ButtonCount; i++)
	{
		tButtonMap *m = &ButtonMap[i];
		const uint8_t mask = m->WW_Mask;

		if (!m->WW_Push)
		{
			if (VP_ISSET(m->VP_Byte, m->VP_Bit))
			{
				WW_SET(m->WW_Byte, mask);
				if (m->pButton)
					*(m->pButton) = 1;
			}
			else
			{
				WW_RESET(m->WW_Byte, mask);
			}
		}
		else
		{
			// Push in same group is falsely reported as pressed by Virpil when any dir is held.
			// Suppress push if any other in group is down.
			if (VP_ISSET(m->VP_Byte, m->VP_Bit) && m->pBS && *m->pBS == 0)
			{
				WW_SET(m->WW_Byte, mask);
				if (m->pButton)
					*(m->pButton) = 1;
			}
			else
			{
				WW_RESET(m->WW_Byte, mask);
			}
		}
	}

	WW_RESET(3, 0x02); //get rid of z-axis detetn buttons

	uint8_t CurrDialBits = (VP_Buffer[2] >> 6) & 0x03;
	int8_t dir = DialDirections[(PrevDialBits << 2) | CurrDialBits];
	PrevDialBits = CurrDialBits;

	DialDirection += dir;

	//analog axes
	//sometimes MSB byte get corrupted and is copied from previous byte. in cese both are equal, juıst skip updating that axis

	//if (VP_Buffer[8] != VP_Buffer[7] && VP_Buffer[6] != VP_Buffer[5])
		CurrSlider = VP_Buffer[6] + ((VP_Buffer[8] & 0x3F) << 8);
	//if (VP_Buffer[5] != VP_Buffer[4] && VP_Buffer[4] != VP_Buffer[3])
		CurrTwistZ = VP_Buffer[4] + ((VP_Buffer[5] & 0x3F) << 8);
	//if (VP_Buffer[12] != VP_Buffer[11]) {
		CurrThumbStickX = ((uint16_t) (((VP_Buffer[12] >> 2) & 0x03) << 8) | VP_Buffer[11]);
		CurrThumbStickY = ((uint16_t) ((VP_Buffer[12] & 0x03) << 8) | VP_Buffer[10]);
	//}


//#ifdef DEBUG
//#define LARGE_DIFF(a,b) ((((a)>(b))?((a)-(b)):((b)-(a)))>5)
//	if (LARGE_DIFF(CurrSlider,PrevSlider) || LARGE_DIFF(CurrTwistZ, PrevTwistZ) || LARGE_DIFF(CurrThumbStickX, PrevThumbStickX) || LARGE_DIFF(CurrThumbStickY, PrevThumbStickY))
//	{
//		printf("Slider=%d (0x%02X 0x%02X) - TwistZ=%d (0x%02X 0x%02X) - ThumbX=%d (0x%02X 0x%02X) - ThumbY=%d (0x%02X 0x%02X) ",
//				CurrSlider, VP_Buffer[8], VP_Buffer[6],
//				CurrTwistZ, VP_Buffer[5], VP_Buffer[4],
//				CurrThumbStickX, VP_Buffer[12], VP_Buffer[11],
//				CurrThumbStickY, VP_Buffer[12], VP_Buffer[10]
//				);
//		printHexFrame("[RAW] ", VP_Buffer, sizeof(VP_Buffer));
//	}
//	PrevSlider = CurrSlider;
//	PrevTwistZ = CurrTwistZ;
//	PrevThumbStickX = CurrThumbStickX;
//	PrevThumbStickY = CurrThumbStickY;
//#endif //DEBUG

#define SET_MIN_MAX(a,b,c,d) if (b > calibData.c) {calibData.c = b; printf("%s max = %d\n", a, b);} else if (b < calibData.d) {calibData.d = b; printf("%s min = %d\n", a, b);}

	if (CalibrationActive)
	{
		SET_MIN_MAX("Slider", CurrSlider, MaxSlider, MinSlider);
		SET_MIN_MAX("TwistZ", CurrTwistZ, MaxTwistZ, MinTwistZ);
		SET_MIN_MAX("ThumbStickX", CurrThumbStickX, MaxThumbStickX, MinThumbStickX);
		SET_MIN_MAX("ThumbStickY", CurrThumbStickY, MaxThumbStickY, MinThumbStickY);
		if (RedDown)
			calibData.SliderDetent1 = CurrSlider;
		if (BlackDown)
			calibData.SliderDetent2 = CurrSlider;
	}
	else
	{

#define WW_WRITE_AXIS(a,b,c) WW_Cache[a].Data[b] = c & 0xff; WW_Cache[a].Data[b+1] = (c >> 8) & 0xff

		uint16_t val = 4096 - map_vp_to_ww(CurrSlider, calibData.MinSlider, calibData.MaxSlider);
		WW_WRITE_AXIS(WW_SLIDER_INDEX, 2, val);
		WW_WRITE_AXIS(WW_SLIDER_INDEX+4, 2, CurrSlider);

		WW_RESET(3, 0xC0);
		if (val > 3400) //trigger secont pinky detent
			WW_SET(3, 0xC0);
		else if (val > 1250)
			WW_SET(3, 0x40);


		val = 4096 - map_vp_to_ww(CurrTwistZ, calibData.MinTwistZ, calibData.MaxTwistZ);
		WW_WRITE_AXIS(WW_ZAXIS_INDEX, 2, val);
		WW_WRITE_AXIS(WW_ZAXIS_INDEX+4, 2, CurrTwistZ);

		val = 4096 - map_vp_to_ww(CurrThumbStickX, calibData.MinThumbStickX, calibData.MaxThumbStickX);
		WW_WRITE_AXIS(WW_THUMBX_INDEX, 2, val);
		WW_WRITE_AXIS(WW_THUMBX_INDEX+4, 2, CurrThumbStickX);

		val = 4096 - map_vp_to_ww(CurrThumbStickY, calibData.MinThumbStickY, calibData.MaxThumbStickY);
		WW_WRITE_AXIS(WW_THUMBY_INDEX, 2, val);
		WW_WRITE_AXIS(WW_THUMBY_INDEX+4, 2, CurrThumbStickY);
	}

}

void writeWinwingDataSync(const uint8_t *data, uint16_t size)
{
	//printHexFrame("[TO WW]", data, size);
	// Enable TX (Half-Duplex Mode)
	HAL_HalfDuplex_EnableTransmitter(&huart1);
	// Send The Data You Want!
	HAL_UART_Transmit(&huart1, data, size, 100);
	// Switch Back To RX (Receiver Mode)
	HAL_HalfDuplex_EnableReceiver(&huart1);
}


static void HandleDial()
{
	//handle dial action here
	if (HAL_GetTick() - DialHandleTime < DIAL_HANDLE_INTERVAL_MS)
		return;
	DialHandleTime = HAL_GetTick();
	WW_RESET(5, 0xA0);
	if (DialHandleTicks % 2)
	{
		if (DialDirection > 0)
		{
			WW_SET(5, 0x20);
			DialDirection--;
			printf("UP %d\n", DialDirection);
		}
		else if (DialDirection < 0)
		{
			WW_SET(5, 0x80);
			DialDirection++;
			printf("DOWN %d\n", DialDirection);
		}
	}
	DialHandleTicks++;
}

static void WW_HandleData()
{
	tCache *pCache;
	for (int i = 0; i < cache_count; i++)
	{
		pCache = &WW_Cache[i];
		if (WWRequestLen == pCache->ReqSize && pCache->RespSize > 0
				&& memcmp(WW_Request, pCache->Data, pCache->ReqSize) == 0)
		{
			//printHexFrame("[FROM WW]", WW_Request, WWRequestLen);
			writeWinwingDataSync(pCache->Data, pCache->RespSize);
			return;
		}
	}
	printHexFrame("[FROM WW - UNHANDLED]", WW_Request, WWRequestLen);
	writeWinwingDataSync(WW_Request, WWRequestLen);
}

// Helper: Calculate CRC32 (simple implementation, replace with HAL if available)
uint32_t crc32_calc(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

// Save calibration data to flash
void SaveCalibrationToFlash(void) {
    HAL_FLASH_Unlock();
    // Erase sector
    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    uint32_t sectorError = 0;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.Sector = FLASH_SECTOR_7;
    eraseInitStruct.NbSectors = 1;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError);
    // Calculate CRC
    calibData.crc32 = crc32_calc((uint8_t*)&calibData, sizeof(CalibrationData) - sizeof(uint32_t));
    // Program data
    uint32_t *src = (uint32_t*)&calibData;
    uint32_t addr = CALIBRATION_FLASH_ADDR;
    for (size_t i = 0; i < sizeof(CalibrationData)/4; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        addr += 4;
    }
    HAL_FLASH_Lock();
}

// Load calibration data from flash
void LoadCalibrationFromFlash(void) {
    CalibrationData *flashData = (CalibrationData*)CALIBRATION_FLASH_ADDR;
    uint32_t crc = crc32_calc((uint8_t*)flashData, sizeof(CalibrationData) - sizeof(uint32_t));
    if (flashData->crc32 == crc) {
        calibData = *flashData;
    }
    // else keep defaults
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setvbuf(stdout, NULL, _IONBF, 0);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	LoadCalibrationFromFlash();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_HalfDuplex_EnableReceiver(&huart1);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, WW_Request, WINWING_MAX_PACKET_SIZE);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
	printf("Virpil Grip to WinWing Base Bridge\n");


	while (1)
	{
		uint32_t now = HAL_GetTick();
		uint8_t virpilOK = readVirpilDataSync();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (blinkDelay && (now - lastBlink > blinkDelay))
		{
			lastBlink = now;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}

		if (virpilOK && (now - lastCheck >= CALIBRATION_CHECK_INTERVAL))
		{
			lastCheck = now;
			if (CalibrationActive)
			{
				if (now - CalibrationEnterTime >= MIN_CALIBRATION_DURATION) //allow exit at least 5 seconds afrter enter
				{
					// Exit calibration if any of the 3 buttons is pressed
					if (DialDown)
					{
						CalibrationActive = 0;
						blinkDelay = 0;
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
						printf("Exit calibration mode\n"
								"\tMinThumbStickX=%d\n"
								"\tMaxThumbStickX=%d\n"
								"\tMinThumbStickY=%d\n"
								"\tMaxThumbStickY=%d\n"
								"\tMinTwistZ=%d\n"
								"\tMaxTwistZ=%d\n"
								"\tMinSlider=%d\n"
								"\tMaxSlider=%d\n"
								"\tSliderDetent1=%d\n"
								"\tSliderDetent2=%d\n",
								calibData.MinThumbStickX, calibData.MaxThumbStickX,
								calibData.MinThumbStickY, calibData.MaxThumbStickY,
								calibData.MinTwistZ, calibData.MaxTwistZ,
								calibData.MinSlider, calibData.MaxSlider,
								calibData.SliderDetent1, calibData.SliderDetent2
								);

						SaveCalibrationToFlash(); // save values
					}
				}
			}
			else
			{
				// Wait for combo to be held for 2 seconds
				if (RedDown && BlackDown && DialDown)
				{
					if (CalibrationEnterTime == 0)
					{
						printf("Pre-calibration enter.\n");
						CalibrationEnterTime = now;
						blinkDelay = 0;
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
					}
					else if (now - CalibrationEnterTime >= 2000)
					{
						CalibrationActive = 1;
						CalibrationEnterTime = now;
						blinkDelay = 200;
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
						printf("Enter calibration mode.\n"
							"\tMinThumbStickX=%d\n"
							"\tMaxThumbStickX=%d\n"
							"\tMinThumbStickY=%d\n"
							"\tMaxThumbStickY=%d\n"
							"\tMinTwistZ=%d\n"
							"\tMaxTwistZ=%d\n"
							"\tMinSlider=%d\n"
							"\tMaxSlider=%d\n"
							"\tSliderDetent1=%d\n"
							"\tSliderDetent2=%d\n",
							calibData.MinThumbStickX, calibData.MaxThumbStickX,
							calibData.MinThumbStickY, calibData.MaxThumbStickY,
							calibData.MinTwistZ, calibData.MaxTwistZ,
							calibData.MinSlider, calibData.MaxSlider,
							calibData.SliderDetent1, calibData.SliderDetent2
							);

						calibData.MinThumbStickX = 0xffff;
						calibData.MaxThumbStickX = 0;
						calibData.MinThumbStickY = 0xffff;
						calibData.MaxThumbStickY = 0;
						calibData.MinTwistZ = 0xffff;
						calibData.MaxTwistZ = 0;
						calibData.MinSlider = 0xffff;
						calibData.MaxSlider = 0;
						//calibData.SliderDetent1 = 0;
						//calibData.SliderDetent2 = 0;
					}
				}
				else if (CalibrationEnterTime)
				{
					printf("Pre-calibration exit.\n");
					CalibrationEnterTime = 0;
					CalibrationJustEntered = 0;
					blinkDelay = 0;
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
				}
			}
		}

		if (virpilOK)
			updateWWCache(); //update WW cache with virpil data

		if (WWRequestLen > 0)
		{
			HandleDial();
			WW_HandleData();
			WWRequestLen = 0;
			HAL_UARTEx_ReceiveToIdle_IT(&huart1, WW_Request, WINWING_MAX_PACKET_SIZE);
		}
//		else
		delay_us(400);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VIRPIL_NSS_GPIO_Port, VIRPIL_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VIRPIL_NSS_Pin */
  GPIO_InitStruct.Pin = VIRPIL_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(VIRPIL_NSS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	WWRequestLen = size;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(100);
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
