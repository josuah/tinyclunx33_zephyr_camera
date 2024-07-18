/*
 * Copyright (c) 2022 CircuitValley
 * Copyright (c) 2023-2024 tinyVision.ai Inc.
 * SPDX-License-Identifier: CC-BY-4.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imx219, LOG_LEVEL_DBG);

// For now, only support 2 lanes
#define LANES 2

#define IMX219_I2C_ADDR  0x10
#define IMX219_SENSOR_ID 0x0219

#define CAMERA_ID IMX219_SENSOR_ID

#define REG_SW_RESET      0x0103
#define REG_MODEL_ID_MSB  0x0000
#define REG_MODEL_ID_LSB  0x0001
#define REG_MODE_SEL      0x0100
#define REG_CSI_LANE      0x0114
#define REG_DPHY_CTRL     0x0128
#define REG_EXCK_FREQ_MSB 0x012A
#define REG_EXCK_FREQ_LSB 0x012B
#define REG_FRAME_LEN_MSB 0x0160
#define REG_FRAME_LEN_LSB 0x0161
#define REG_LINE_LEN_MSB  0x0162
#define REG_LINE_LEN_LSB  0x0163
#define REG_X_ADD_STA_MSB 0x0164
#define REG_X_ADD_STA_LSB 0x0165
#define REG_X_ADD_END_MSB 0x0166
#define REG_X_ADD_END_LSB 0x0167
#define REG_Y_ADD_STA_MSB 0x0168
#define REG_Y_ADD_STA_LSB 0x0169
#define REG_Y_ADD_END_MSB 0x016A
#define REG_Y_ADD_END_LSB 0x016B

#define REG_X_OUT_SIZE_MSB 0x016C
#define REG_X_OUT_SIZE_LSB 0x016D
#define REG_Y_OUT_SIZE_MSB 0x016E
#define REG_Y_OUT_SIZE_LSB 0x016F

#define REG_X_ODD_INC      0x0170
#define REG_Y_ODD_INC      0x0171
#define REG_IMG_ORIENT     0x0172
#define REG_BINNING_H      0x0174
#define REG_BINNING_V      0x0175
#define REG_BIN_CALC_MOD_H 0x0176
#define REG_BIN_CALC_MOD_V 0x0177

#define REG_CSI_FORMAT_C 0x018C
#define REG_CSI_FORMAT_D 0x018D

#define REG_DIG_GAIN_GLOBAL_MSB  0x0158
#define REG_DIG_GAIN_GLOBAL_LSB  0x0159
#define REG_ANA_GAIN_GLOBAL      0x0157
#define REG_INTEGRATION_TIME_MSB 0x015A
#define REG_INTEGRATION_TIME_LSB 0x015B
#define REG_ANALOG_GAIN          0x0157

#define REG_VTPXCK_DIV      0x0301
#define REG_VTSYCK_DIV      0x0303
#define REG_PREPLLCK_VT_DIV 0x0304
#define REG_PREPLLCK_OP_DIV 0x0305
#define REG_PLL_VT_MPY_MSB  0x0306
#define REG_PLL_VT_MPY_LSB  0x0307
#define REG_OPPXCK_DIV      0x0309
#define REG_OPSYCK_DIV      0x030B
#define REG_PLL_OP_MPY_MSB  0x030C
#define REG_PLL_OP_MPY_LSB  0x030D

#define REG_TEST_PATTERN_MSB 0x0600
#define REG_TEST_PATTERN_LSB 0x0601
#define REG_TP_RED_MSB       0x0602
#define REG_TP_RED_LSB       0x0603
#define REG_TP_GREEN_MSB     0x0604
#define REG_TP_GREEN_LSB     0x0605
#define REG_TP_BLUE_MSB      0x0606
#define REG_TP_BLUE_LSB      0x0607
#define REG_TP_X_OFFSET_MSB  0x0620
#define REG_TP_X_OFFSET_LSB  0x0621
#define REG_TP_Y_OFFSET_MSB  0x0622
#define REG_TP_Y_OFFSET_LSB  0x0623
#define REG_TP_WIDTH_MSB     0x0624
#define REG_TP_WIDTH_LSB     0x0625
#define REG_TP_HEIGHT_MSB    0x0626
#define REG_TP_HEIGHT_LSB    0x0627

typedef enum {
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
} IMGSENSOR_MODE;

enum {
	IMAGE_NORMAL = 0,
	IMAGE_H_MIRROR,
	IMAGE_V_MIRROR,
	IMAGE_HV_MIRROR
};

typedef struct imgsensor_mode_struct_s {
	uint16_t pix_clk_mul;
	uint16_t pix_clk_div;
	uint8_t mirror;
	uint16_t integration;
	uint16_t gain;
	uint16_t gain_max;

	uint16_t linelength;
	uint16_t framelength;
	uint16_t startx;
	uint16_t starty;
	uint16_t endx;
	uint16_t endy;

	uint16_t width;
	uint16_t height;
	uint16_t framerate;
	uint8_t binning;
	uint16_t fps;
	uint8_t test_pattern;
} imgsensor_mode_t;

typedef struct imx219_reg_s {
	uint16_t address;
	uint8_t val;
} imx219_reg_t;

#if LANES == 2
#define SENSOR_MODE0_WIDTH  (unsigned int)640
#define SENSOR_MODE0_HEIGHT (unsigned int)78
#define SENSOR_MODE0_FPS    (unsigned int)1000

#define SENSOR_MODE1_WIDTH  (unsigned int)640
#define SENSOR_MODE1_HEIGHT (unsigned int)480
#define SENSOR_MODE1_FPS    (unsigned int)200

#define SENSOR_MODE2_WIDTH  (unsigned int)1280
#define SENSOR_MODE2_HEIGHT (unsigned int)720
#define SENSOR_MODE2_FPS    (unsigned int)60

#define SENSOR_MODE3_WIDTH  (unsigned int)1920
#define SENSOR_MODE3_HEIGHT (unsigned int)1080
#define SENSOR_MODE3_FPS    (unsigned int)30

#define SENSOR_MODE4_WIDTH   (unsigned int)3280
#define SENSOR_MODE4_HEIGHT  (unsigned int)2464
#define SENSOR_MODE4_FPS_MIN (unsigned int)5
#define SENSOR_MODE4_FPS     (unsigned int)15

#else
#define SENSOR_MODE0_WIDTH  (unsigned int)640
#define SENSOR_MODE0_HEIGHT (unsigned int)78
#define SENSOR_MODE0_FPS    (unsigned int)2000

#define SENSOR_MODE1_WIDTH  (unsigned int)640
#define SENSOR_MODE1_HEIGHT (unsigned int)480
#define SENSOR_MODE1_FPS    (unsigned int)400

#define SENSOR_MODE2_WIDTH  (unsigned int)1280
#define SENSOR_MODE2_HEIGHT (unsigned int)720
#define SENSOR_MODE2_FPS    (unsigned int)120

#define SENSOR_MODE3_WIDTH  (unsigned int)1920
#define SENSOR_MODE3_HEIGHT (unsigned int)1080
#define SENSOR_MODE3_FPS    (unsigned int)60

#define SENSOR_MODE4_WIDTH   (unsigned int)3280
#define SENSOR_MODE4_HEIGHT  (unsigned int)2464
#define SENSOR_MODE4_FPS_MIN (unsigned int)10
#define SENSOR_MODE4_FPS     (unsigned int)30

#endif

#define I2C0 DEVICE_DT_GET(DT_NODELABEL(i2c0))

#define GET_WORD_MSB(x)  ((x >> 8) & 0xFF)
#define GET_WORD_LSB(x)  (       x & 0xFF)

static bool sensor_i2c_write(uint16_t addr, uint8_t data)
{
	uint8_t msg[] = { addr & 0xff, addr & 0xff, data };

	return i2c_write(I2C0, msg, sizeof(msg), IMX219_I2C_ADDR) == 0;
}

static bool sensor_i2c_read(uint16_t addr, uint8_t *data)
{
	addr = sys_cpu_to_be16(addr);
	return i2c_write_read(I2C0, IMX219_I2C_ADDR, &addr, sizeof(addr), &data, sizeof(data)) == 0;
}

imgsensor_mode_t *selected_img_mode;

static const imx219_reg_t mode_default[] = {
	// default register settings, Resolution and FPS specific settings will be over written
	{REG_MODE_SEL, 0x00},
	{0x30EB, 0x05}, // access sequence
	{0x30EB, 0x0C},
	{0x300A, 0xFF},
	{0x300B, 0xFF},
	{0x30EB, 0x05},
	{0x30EB, 0x09},
	{REG_CSI_LANE, 0x01},      // 3-> 4Lane 1-> 2Lane
	{REG_DPHY_CTRL, 0x00},     // DPHY timing 0-> auot 1-> manual
	{REG_EXCK_FREQ_MSB, 0x18}, // external oscillator frequncy 0x18 -> 24Mhz
	{REG_EXCK_FREQ_LSB, 0x00},
	{REG_FRAME_LEN_MSB,
	 0x06}, // frame length , Raspberry pi sends this commands continously when recording video
		// @60fps ,writes come at interval of 32ms , Data 355 for resolution 1280x720
		// command 162 also comes along with data 0DE7 also 15A with data 0200
	{REG_FRAME_LEN_LSB, 0xE3},
	{REG_LINE_LEN_MSB, 0x0d}, // does not directly affect how many bits on wire in one line does
				  // affect how many clock between lines
	{REG_LINE_LEN_LSB,
	 0x78}, // appears to be having step in value, not every LSb change will reflect on fps
	{REG_X_ADD_STA_MSB, 0x02}, // x start
	{REG_X_ADD_STA_LSB, 0xA8},
	{REG_X_ADD_END_MSB, 0x0A}, // x end
	{REG_X_ADD_END_LSB, 0x27},
	{REG_Y_ADD_STA_MSB, 0x02}, // y start
	{REG_Y_ADD_STA_LSB, 0xB4},
	{REG_Y_ADD_END_MSB, 0x06}, // y end
	{REG_Y_ADD_END_LSB, 0xEB},
	{REG_X_OUT_SIZE_MSB, 0x07}, // resolution 1280 -> 5 00 , 1920 -> 780 , 2048 -> 0x8 0x00
	{REG_X_OUT_SIZE_LSB, 0x80},
	{REG_Y_OUT_SIZE_MSB, 0x04}, // 720 -> 0x02D0 | 1080 -> 0x438  | this setting changes how
				    // many line over wire does not affect frame rate
	{REG_Y_OUT_SIZE_LSB, 0x38},
	{REG_X_ODD_INC, 0x01},       // increment
	{REG_Y_ODD_INC, 0x01},       // increment
	{REG_BINNING_H, 0x00},       // binning H 0 off 1 x2 2 x4 3 x2 analog
	{REG_BINNING_V, 0x00},       // binning H 0 off 1 x2 2 x4 3 x2 analog
	{REG_CSI_FORMAT_C, 0x0A},    // CSI Data format A-> 10bit
	{REG_CSI_FORMAT_D, 0x0A},    // CSI Data format
	{REG_VTPXCK_DIV, 0x05},      // vtpxclkd_div	5 301
	{REG_VTSYCK_DIV, 0x01},      // vtsclk _div  1	303
	{REG_PREPLLCK_VT_DIV, 0x03}, // external oscillator /3
	{REG_PREPLLCK_OP_DIV, 0x03}, // external oscillator /3
	{REG_PLL_VT_MPY_MSB, 0x00},  // PLL_VT multiplizer
	{REG_PLL_VT_MPY_LSB, 0x52},  // Changes Frame rate with , integration register 0x15a
	{REG_OPPXCK_DIV, 0x0A},      // oppxck_div
	{REG_OPSYCK_DIV, 0x01},      // opsysck_div
	{REG_PLL_OP_MPY_MSB, 0x00},  // PLL_OP
	{REG_PLL_OP_MPY_LSB,
	 0x32},         // 8Mhz x 0x57 ->696Mhz -> 348Mhz |  0x32 -> 200Mhz | 0x40 -> 256Mhz
	{0x455E, 0x00}, // magic?
	{0x471E, 0x4B},
	{0x4767, 0x0F},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47B4, 0x14},
	{0x4713, 0x30},
	{0x478B, 0x10},
	{0x478F, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0E},
	{0x479B, 0x0E},
	{REG_TP_RED_MSB, 0x00},
	{REG_TP_RED_LSB, 0x00},
	{REG_TP_GREEN_MSB, 0x00},
	{REG_TP_GREEN_LSB, 0x00},
	{REG_TP_BLUE_MSB, 0x00},
	{REG_TP_BLUE_LSB, 0x00},
	{REG_TEST_PATTERN_MSB, 0x00}, // test pattern
	{REG_TEST_PATTERN_LSB, 0x00},
	{REG_TP_X_OFFSET_MSB, 0x00}, // tp offset x 0
	{REG_TP_X_OFFSET_LSB, 0x00},
	{REG_TP_Y_OFFSET_MSB, 0x00}, // tp offset y 0
	{REG_TP_Y_OFFSET_LSB, 0x00},
	{REG_TP_WIDTH_MSB, 0x05}, // TP width 1920 ->780 1280->500
	{REG_TP_WIDTH_LSB, 0x00},
	{REG_TP_HEIGHT_MSB, 0x02}, // TP height 1080 -> 438 720->2D0
	{REG_TP_HEIGHT_LSB, 0xD0},
	{REG_DIG_GAIN_GLOBAL_MSB, 0x01},
	{REG_DIG_GAIN_GLOBAL_LSB, 0x00},
	{REG_ANA_GAIN_GLOBAL,
	 0x80}, // analog gain , raspberry pi constinouly changes this depending on scense
	{REG_INTEGRATION_TIME_MSB, 0x03}, // integration time , really important for frame rate
	{REG_INTEGRATION_TIME_LSB, 0x51},
	{REG_IMG_ORIENT, 0x03}, // image_orientation (for both direction) bit[0]: hor bit[1]: vert
	{REG_MODE_SEL, 0x01},

};

imgsensor_mode_t *sensor_config;

static imgsensor_mode_t sensor_config_2LANE[] = {
	{// 200Mhz 2Lane
	 .pix_clk_mul = 0x2E,
	 .pix_clk_div = 0x4,     // only 4 or 5 or 8 or 10
	 .integration = 258 - 4, // must be < (linelength- 4) to maintain frame rate by framelength
				 // or integration time will slow frame rate
	 .gain = 0x70,
	 .gain_max = 0xFF,
	 .linelength = 3448, // Warning! This value need to be either 0xD78 or 0xDE7 regardless of
			     // frame size and FPS, other values will result undefined and
			     // ununderstanable issues in image
	 .framelength =
		 258, // decided how long is frame, basically frame rate with pix clock, it has
		      // second priority to integration time. absolute minimum is 255 for imx219
	 .startx = 1000,
	 .starty = 750,
	 .endx = 2280,
	 .endy = 1715, // this has to odd or bayer oder will change
	 .width = 640,
	 .height = 480, // each frame will have two extra line to compensate for debayer crop
	 .binning = 2,
	 .fps = 200,
	 .test_pattern = 0},

	{// 200Mhz mipi 2 lane
	 .pix_clk_mul = 0x2E,
	 .pix_clk_div = 0x4,
	 .integration = 862 - 4,
	 .gain = 0x80,
	 .gain_max = 0xFF,
	 .linelength = 0xD78,
	 .framelength = 862,
	 .startx = 0x2A8,
	 .starty = 0x2B4,
	 .endx = 0xA27,
	 .endy = 0x6EB,
	 .width = 1280,
	 .height = 720,
	 .binning = 0,
	 .fps = 60,
	 .test_pattern = 0},

	{// 200Mhz 2Lane
	 .pix_clk_mul = 0x20,
	 .pix_clk_div = 0x4,
	 .integration = 1200 - 4,
	 .gain = 0x80,
	 .gain_max = 0xFF,
	 .linelength = 0xD78,
	 .framelength = 1200,
	 .startx = 0x2A8,
	 .starty = 0x2B4,
	 .endx = 0xA27,
	 .endy = 0x6EB,
	 .width = 1920,
	 .height = 1080,
	 .binning = 0,
	 .fps = 30,
	 .test_pattern = 0},

	{// 200Mhz 2Lane
	 .pix_clk_mul = 0x2D,
	 .pix_clk_div = 0x4,
	 .integration = 56 - 4,
	 .gain = 200,
	 .gain_max = 0xFF,
	 .linelength = 0xD78,
	 .framelength = 56,
	 .startx = 1320,
	 .starty = 990,
	 .endx = 2600,
	 .endy = 1561,
	 .width = 640,
	 .height = 80,
	 .binning = 2,
	 .fps = 900,
	 .test_pattern = 0},

	{// 200Mhz 2 Lane
	 .pix_clk_mul = 0x12,
	 .pix_clk_div = 0x4,
	 .integration = 2670 - 4,
	 .gain = 200,
	 .gain_max = 0xFF,
	 .linelength = 0xD78, // 3448
	 .framelength = 2670,
	 .startx = 0,
	 .starty = 0,
	 .endx = 3279,
	 .endy = 2463,
	 .width = 3280,
	 .height = 2464,
	 .binning = 0,
	 .fps = 5,
	 .test_pattern = 0},

	{// 200Mhz 2 Lane
	 .pix_clk_mul = 0x12,
	 .pix_clk_div = 0x4,
	 .integration = 2670 - 4,
	 .gain = 200,
	 .gain_max = 0xFF,
	 .linelength = 0xD78, // 3448
	 .framelength = 2670,
	 .startx = 0,
	 .starty = 0,
	 .endx = 3279,
	 .endy = 2463,
	 .width = 3280,
	 .height = 2464,
	 .binning = 0,
	 .fps = 7,
	 .test_pattern = 0},

};

uint8_t camera_stream_on(uint8_t on)
{
	return sensor_i2c_write(REG_MODE_SEL, on);
}

void SensorReset(void)
{
	sensor_i2c_write(REG_SW_RESET, 0x01);
	/* Wait for some time to allow proper reset. */
	k_sleep(K_USEC(10));
	/* Delay the allow the sensor to power up. */
	sensor_i2c_write(REG_SW_RESET, 0x00);
	k_sleep(K_USEC(100));
	return;
}

static void sensor_configure_mode(imgsensor_mode_t *mode)
{
	// set_mirror_flip(mode->mirror);
	camera_stream_on(false);
	sensor_i2c_write(REG_PLL_VT_MPY_MSB, GET_WORD_MSB(mode->pix_clk_mul));
	sensor_i2c_write(REG_PLL_VT_MPY_LSB, GET_WORD_LSB(mode->pix_clk_mul));

	sensor_i2c_write(REG_VTPXCK_DIV, GET_WORD_LSB(mode->pix_clk_div));

	sensor_i2c_write(REG_INTEGRATION_TIME_MSB, GET_WORD_MSB(mode->integration));
	sensor_i2c_write(REG_INTEGRATION_TIME_LSB, GET_WORD_LSB(mode->integration));

	sensor_i2c_write(REG_ANALOG_GAIN, GET_WORD_LSB(mode->gain));
	sensor_i2c_write(REG_LINE_LEN_MSB, GET_WORD_MSB(mode->linelength));
	sensor_i2c_write(REG_LINE_LEN_LSB, GET_WORD_LSB(mode->linelength));

	sensor_i2c_write(REG_FRAME_LEN_MSB, GET_WORD_MSB(mode->framelength));
	sensor_i2c_write(REG_FRAME_LEN_LSB, GET_WORD_LSB(mode->framelength));

	sensor_i2c_write(REG_X_ADD_STA_MSB, GET_WORD_MSB(mode->startx));
	sensor_i2c_write(REG_X_ADD_STA_LSB, GET_WORD_LSB(mode->startx));

	sensor_i2c_write(REG_Y_ADD_STA_MSB, GET_WORD_MSB(mode->starty));
	sensor_i2c_write(REG_Y_ADD_STA_LSB, GET_WORD_LSB(mode->starty));

	sensor_i2c_write(REG_X_ADD_END_MSB, GET_WORD_MSB(mode->endx));
	sensor_i2c_write(REG_X_ADD_END_LSB, GET_WORD_LSB(mode->endx));

	sensor_i2c_write(REG_Y_ADD_END_MSB, GET_WORD_MSB(mode->endy));
	sensor_i2c_write(REG_Y_ADD_END_LSB, GET_WORD_LSB(mode->endy));

	sensor_i2c_write(REG_X_OUT_SIZE_MSB, GET_WORD_MSB(mode->width));
	sensor_i2c_write(REG_X_OUT_SIZE_LSB, GET_WORD_LSB(mode->width));

	sensor_i2c_write(REG_Y_OUT_SIZE_MSB, GET_WORD_MSB(mode->height));
	sensor_i2c_write(REG_Y_OUT_SIZE_LSB, GET_WORD_LSB(mode->height));

	sensor_i2c_write(REG_TEST_PATTERN_LSB, (mode->test_pattern < 8) ? mode->test_pattern : 0);

	sensor_i2c_write(REG_TP_WIDTH_MSB, GET_WORD_MSB(mode->width));
	sensor_i2c_write(REG_TP_WIDTH_LSB, GET_WORD_LSB(mode->width));
	sensor_i2c_write(REG_TP_HEIGHT_MSB, GET_WORD_MSB(mode->height));
	sensor_i2c_write(REG_TP_HEIGHT_LSB, GET_WORD_LSB(mode->height));

	if (mode->binning == 2) {
		sensor_i2c_write(REG_BINNING_H, 0x03);
		sensor_i2c_write(REG_BINNING_V, 0x03);
	} else {
		sensor_i2c_write(REG_BINNING_H, 0x00);
		sensor_i2c_write(REG_BINNING_V, 0x00);
	}
	camera_stream_on(1);
}

uint8_t SensorI2cBusTest(void)
{
	uint8_t model_lsb;
	uint8_t model_msb;

	sensor_i2c_read(REG_MODEL_ID_MSB, &model_msb);
	sensor_i2c_read(REG_MODEL_ID_LSB, &model_lsb);

	if (((((uint16_t)model_msb & 0x0F) << 8) | model_lsb) == CAMERA_ID) {
		LOG_INF("I2C Sensor id: 0x%x", (((uint16_t)model_msb & 0x0F) << 8) | model_lsb);
		return 0;
	}

	return -ENODEV;
}

uint8_t SensorInit(void)
{
	if (SensorI2cBusTest() != 0) /* Verify that the sensor is connected. */
	{
		LOG_ERR("Error: Reading Sensor ID failed!");
		return -ENODEV;
	}

	for (uint16_t i = 0; i < (sizeof(mode_default) / sizeof(mode_default[0])); i++) {
		sensor_i2c_write((mode_default + i)->address, (mode_default + i)->val);
	}
	sensor_config = sensor_config_2LANE;
	sensor_configure_mode(&sensor_config_2LANE[2]);
	return 0;
}

uint8_t SensorGetBrightness(void)
{
	return selected_img_mode->gain;
}

uint16_t getMaxBrightness(void)
{
	return selected_img_mode->gain_max;
}

void SensorSetBrightness(uint8_t input)
{
	selected_img_mode->gain = input;
	sensor_i2c_write(REG_ANALOG_GAIN, input);
}

uint16_t sensor_get_min_exposure(void)
{
	return 0;
}

uint16_t sensor_get_max_exposure(void)
{
	return selected_img_mode->integration;
}

uint16_t sensor_get_def_exposure(void)
{
	return selected_img_mode->integration;
}

uint16_t sensor_get_exposure(void)
{
	return selected_img_mode->integration;
}

void sensor_set_exposure(uint16_t integration)
{
	if (integration > selected_img_mode->integration) {
		integration = selected_img_mode->integration;
	}
	sensor_i2c_write(REG_INTEGRATION_TIME_MSB, (integration >> 8) & 0xFF);
	sensor_i2c_write(REG_INTEGRATION_TIME_LSB, integration & 0xFF);
}

uint8_t sensor_get_test_pattern(void)
{
	return selected_img_mode->test_pattern;
}

void sensor_set_test_pattern(uint8_t test_pattern)
{
	if (test_pattern > 8) {
		test_pattern = 0;
	}
	selected_img_mode->test_pattern = test_pattern;
	sensor_i2c_write(REG_TEST_PATTERN_LSB, test_pattern);
}
