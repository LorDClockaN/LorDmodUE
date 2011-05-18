/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "s5k3h1gx.h"


struct s5k3h1gx_i2c_reg_conf s5k3h1gx_init_settings_array_mipi[] =
{
  /* Because we will turn off power source after probe, there is no need to set register here */
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_init_settings_array_parallel[] =
{
  /* Because we will turn off power source after probe, there is no need to set register here */
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_common_settings_array_mipi[] =
{
  { 0x0100 , 0x00 },
  { 0x3091 , 0x00 },
  { 0x310E , 0x00 },
  { 0x0111 , 0x01 },
  { 0x3098 , 0xAB },
  { 0x309A , 0x01 },
  { 0x310D , 0xC6 },
  { 0x30C3 , 0x40 },
  { 0x308E , 0x01 },
  { 0x308F , 0x8F },
  { 0x30BB , 0x02 },
  { 0x30C7 , 0x1A },
  { 0x30BC , 0x38 },
  { 0x30BD , 0x40 },
  { 0x3110 , 0x70 },
  { 0x3111 , 0x80 },
  { 0x3112 , 0x7B },
  { 0x3113 , 0xC0 },
  { 0x3000 , 0x08 },
  { 0x3001 , 0x05 },
  { 0x3002 , 0x0D },
  { 0x3003 , 0x21 },
  { 0x3004 , 0x62 },
  { 0x3005 , 0x0B },
  { 0x3006 , 0x6D },
  { 0x3007 , 0x02 },
  { 0x3008 , 0x62 },
  { 0x3009 , 0x62 },
  { 0x300A , 0x41 },
  { 0x300B , 0x10 },
  { 0x300C , 0x21 },
  { 0x300D , 0x04 },
  { 0x307E , 0x03 },
  { 0x307F , 0xA5 },
  { 0x3080 , 0x04 },
  { 0x3081 , 0x29 },
  { 0x3082 , 0x03 },
  { 0x3083 , 0x21 },
  { 0x3011 , 0x5F },
  { 0x3156 , 0xE2 },
  { 0x3027 , 0x0E },
  { 0x300f , 0x02 },
  { 0x3072 , 0x13 },
  { 0x3073 , 0x61 },
  { 0x3074 , 0x92 },
  { 0x3075 , 0x10 },
  { 0x3076 , 0xA2 },
  { 0x3077 , 0x02 },
  { 0x3078 , 0x91 },
  { 0x3079 , 0x91 },
  { 0x307A , 0x61 },
  { 0x307B , 0x18 },
  { 0x307C , 0x61 },
  { 0x3010 , 0x10 },
  { 0x3017 , 0x74 },
  { 0x3018 , 0x00 },
  { 0x3020 , 0x02 },
  { 0x3021 , 0x24 },
  { 0x3023 , 0x40 },
  { 0x3024 , 0x08 },
  { 0x3025 , 0x08 },
  { 0x301C , 0xD4 },
  { 0x315D , 0x00 },
  { 0x3053 , 0xCF },
  { 0x3054 , 0x00 },
  { 0x3055 , 0x35 },
  { 0x3062 , 0x04 },
  { 0x3063 , 0x38 },
  { 0x3016 , 0x2c },
  { 0x3157 , 0x02 },
  { 0x3158 , 0x00 },
  { 0x315B , 0x02 },
  { 0x315C , 0x00 },
  { 0x301B , 0x04 },
  { 0x301A , 0xC4 },
  { 0x302d , 0x19 },
  { 0x302b , 0x04 },
  { 0x0305 , 0x04 }, /* pre_pll_clk_div = 4 */
  { 0x0306 , 0x00 }, /* pll_multiplier */
  { 0x0307 , 0x66 }, /* pll_multiplier = 102 */
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x05 }, /* vt_pix_clk_div = 5 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x05 }, /* op_pix_clk_div = 5 */
  { 0x30CC , 0xA0 }, /* DPHY_band_ctrl 560 ~ 640Mbps */
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_common_settings_array_parallel[] =
{
  { 0x0100 , 0x00 },
  /* MIPI/CCP/Parallel Setting */
  { 0x3091 , 0x00 },
  { 0x3065 , 0x15 },  /* sync mode */
  { 0x310E , 0x08 },  /* reg_sel 08h:parallel / 04h: CCP / 00h : MIPI */
  { 0x0111 , 0x01 },  /* CCP2_signalling_mode */
  { 0x308E , 0x01 },
  { 0x308F , 0x8F },
  /* Manufacture Setting */
  { 0x3000 , 0x08 },
  { 0x3001 , 0x05 },
  { 0x3002 , 0x0D },
  { 0x3003 , 0x21 },
  { 0x3004 , 0x62 },
  { 0x3005 , 0x0B },
  { 0x3006 , 0x6D },
  { 0x3007 , 0x02 },
  { 0x3008 , 0x62 },
  { 0x3009 , 0x62 },
  { 0x300A , 0x41 },
  { 0x300B , 0x10 },
  { 0x300C , 0x21 },
  { 0x300D , 0x04 },
  { 0x307E , 0x03 },
  { 0x307F , 0xA5 },
  { 0x3080 , 0x04 },
  { 0x3081 , 0x29 },
  { 0x3082 , 0x03 },
  { 0x3083 , 0x21 },
  { 0x3011 , 0x5F },
  { 0x3156 , 0xE2 },
 /* { 0x3027 , 0x0E }, */
  { 0x300f , 0x02 },
  { 0x3072 , 0x13 },
  { 0x3073 , 0x61 },
  { 0x3074 , 0x92 },
  { 0x3075 , 0x10 },
  { 0x3076 , 0xA2 },
  { 0x3077 , 0x02 },
  { 0x3078 , 0x91 },
  { 0x3079 , 0x91 },
  { 0x307A , 0x61 },
  { 0x307B , 0x18 },
  { 0x307C , 0x61 },
  { 0x3010 , 0x10 },
  { 0x3017 , 0x74 },
  { 0x3018 , 0x00 },
  { 0x3020 , 0x02 },
  { 0x3021 , 0x24 },
  { 0x3023 , 0x40 },
  { 0x3024 , 0x08 },
  { 0x3025 , 0x08 },
  { 0x301C , 0xD4 },
  { 0x315D , 0x00 },
  { 0x3053 , 0xCF },
  { 0x3054 , 0x00 },
  { 0x3055 , 0x35 },
  { 0x3062 , 0x04 },
  { 0x3063 , 0x38 },
  { 0x3016 , 0x2c },
  { 0x3157 , 0x02 },
  { 0x3158 , 0x00 },
  { 0x315B , 0x02 },
  { 0x315C , 0x00 },
  { 0x301A , 0xC4 },
  { 0x301B , 0x04 },
  { 0x302d , 0x19 },
  { 0x302b , 0x04 },
  { 0x310d , 0xe6 },
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_qtr_settings_array_mipi[] =
{
  { 0x0344 , 0x00 }, /* Xaddrstart 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Yaddrstart 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* Xaddrend 3279d */
  { 0x0349 , 0xCf },
  { 0x034A , 0x09 }, /* Yaddrend 2463d */
  { 0x034B , 0x9F },
  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x03 }, /* y_odd_inc = 3 */
  { 0x0105 , 0x01 }, /* skip corrupted frame - for preview flash when doing hjr af */
  { 0x034C , 0x06 }, /* x_output_size = 1640 */
  { 0x034D , 0x68 },
  { 0x034E , 0x04 }, /* y_output_size = 1232 */
  { 0x034F , 0xD0 },
  { 0x0200 , 0x02 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x03 }, /* Coarse integration time */
  { 0x0203 , 0xA0 },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x04 }, /* Frame_length_lines 1248d */
  { 0x0341 , 0xE0 },
  { 0x300E , 0xED }, /* Reserved */
  { 0x3085 , 0x00 }, /* Reserved */
  { 0x301D , 0x81 }, /* Reserved */
  { 0x3086 , 0x03 }, /* Reserved */
  { 0x3087 , 0x34 }, /* Reserved */
  { 0x3065 , 0x15 }, /* Reserved */
  { 0x3028 , 0x40 }, /* Reserved */
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_qtr_settings_array_parallel[] =
{
  /* PLL settings  MCLK:24MHz,vt_pix_clk_freq_mhz=130.2MHz,op_sys_clk_freq_mhz=65.1MHz */
  { 0x0305 , 0x08 }, /* pre_pll_clk_div = 8	*/
  { 0x0306 , 0x01 }, /* pll_multiplier */
  { 0x0307 , 0x40 }, /* pll_multiplier = 320 */
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x05 }, /* vt_pix_clk_div = 5 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x0A }, /* op_pix_clk_div = 10 */
  /* DBLR Clock setting = 96Mhz = vt_pix_clk_freq_mhz/2 */
  { 0x3027 , 0x7E },

  /* Readout	H:1/2 SubSampling binning, V:1/2 SubSampling binning */
  { 0x0344 , 0x00 }, /* X addr start 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Y addr start 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* X addr end 3279d */
  { 0x0349 , 0xCF },
  { 0x034A , 0x09 }, /* Y addr end 2463d */
  { 0x034B , 0x9F },
  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x03 }, /* y_odd_inc = 3 */
  { 0x0105 , 0x01 }, /* skip corrupted frame - for preview flash when doing hjr af */
  /* ------------- */
  { 0x0401 , 0x01 }, /* Derating_en  = 1 (disable) */
  { 0x0405 , 0x10 },
  { 0x0700 , 0x05 }, /* fifo_threshold = 1328 */
  { 0x0701 , 0x30 },
  /* ------------- */
  { 0x034C , 0x06 }, /* x_output_size = 1640 */
  { 0x034D , 0x68 },
  { 0x034E , 0x04 }, /* y_output_size = 1232 */
  { 0x034F , 0xD0 },
  { 0x0200 , 0x03 }, /* fine integration time */
  { 0x0201 , 0x50 },
  /* ------------- */
  { 0x0202 , 0x03 }, /* Coarse integration time */
  { 0x0203 , 0xA0 }, /* DB */
  /* ------------- */
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x07 }, /* Frame_length_lines 1843d */
  { 0x0341 , 0x33 },
  /* Manufacture Setting */
  { 0x300E , 0xED },
  { 0x3085 , 0x00 },
  { 0x301D , 0x81 },
  { 0x3028 , 0x40 },
  { 0x3086 , 0x03 },
  { 0x3087 , 0x34 },
  { 0x3065 , 0x15 },
  /* ------------- */
  { 0x310C , 0x50 }, /* pclk invert */
  { 0x3117 , 0x0A }, /* H/V sync driving strength 6mA */
  { 0x3118 , 0xA3 }, /* parallel data driving strength 6mA */
  /* ------------- */

  /*{ 0x0100 , 0x01 },*/
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_full_settings_array_mipi[] =
{
  { 0x0344 , 0x00 }, /* X addr start 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Y addr start 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* X addr end 3279d */
  { 0x0349 , 0xCf },
  { 0x034A , 0x09 }, /* Y addr end 2463d */
  { 0x034B , 0x9F },
  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x01 }, /* y_odd_inc = 1 */
  { 0x0105 , 0x01 }, /* skip corrupted frame - for preview flash when doing hjr af */
  { 0x034C , 0x0C }, /* x_output_size = 3280 */
  { 0x034D , 0xD0 },
  { 0x034E , 0x09 }, /* y_output_size = 2464 */
  { 0x034F , 0xA0 },
  { 0x0200 , 0x02 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x04 }, /* Coarse integration time */
  { 0x0203 , 0xE7 },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x09 }, /* Frame_length_lines */
  { 0x0341 , 0xB0 },
  { 0x300E , 0xE9 }, /* Reserved */
  { 0x3085 , 0x01 }, /* Reserved */
  { 0x301D , 0x01 }, /* Reserved */
  { 0x3086 , 0x03 }, /* Reserved */
  { 0x3087 , 0x34 }, /* Reserved */
  { 0x3065 , 0x15 }, /* Reserved */
  { 0x3028 , 0x41 }, /* Reserved */
};

struct s5k3h1gx_i2c_reg_conf s5k3h1gx_full_settings_array_parallel[] =
{
  /* PLL settings  MCLK:24MHz,vt_pix_clk_freq_mhz=96MHz,op_sys_clk_freq_mhz=96MHz */
  { 0x0305 , 0x04 }, /* pre_pll_clk_div = 4 */
  { 0x0306 , 0x00 }, /* pll_multiplier */
  { 0x0307 , 0xA0 }, /* pll_multiplier  = 160 */
  { 0x0303 , 0x01 }, /* vt_sys_clk_div = 1 */
  { 0x0301 , 0x0A }, /* vt_pix_clk_div = 10 */
  { 0x030B , 0x01 }, /* op_sys_clk_div = 1 */
  { 0x0309 , 0x0A }, /* op_pix_clk_div = 10 */
  /* DBLR Clock setting = 96Mhz = vt_pix_clk_freq_mhz */
  { 0x3027 , 0x3E },
  /* Readout	Full */
  { 0x0344 , 0x00 }, /* X addr start 0d */
  { 0x0345 , 0x00 },
  { 0x0346 , 0x00 }, /* Y addr start 0d */
  { 0x0347 , 0x00 },
  { 0x0348 , 0x0C }, /* X addr end 3279d */
  { 0x0349 , 0xCF },
  { 0x034A , 0x09 }, /* Y addr end 2463d */
  { 0x034B , 0x9F },
  { 0x0381 , 0x01 }, /* x_even_inc = 1 */
  { 0x0383 , 0x01 }, /* x_odd_inc = 1 */
  { 0x0385 , 0x01 }, /* y_even_inc = 1 */
  { 0x0387 , 0x01 }, /* y_odd_inc = 1 */
  { 0x0105 , 0x01 }, /* skip corrupted frame - for preview flash when doing hjr af */
  /* ------------- */
  { 0x0401 , 0x00 }, /* Scaler OFF */
  { 0x0405 , 0x10 }, /* Scaling ratio 16/16 */
  { 0x0700 , 0x03 }, /* fifo_threshold = 818d */
  { 0x0701 , 0x32 },
  /* ------------- */
  { 0x034C , 0x0C }, /* x_output_size = 3280 */
  { 0x034D , 0xD0 },
  { 0x034E , 0x09 }, /* y_output_size = 2464 */
  { 0x034F , 0xA0 },
  { 0x0200 , 0x03 }, /* fine integration time */
  { 0x0201 , 0x50 },
  { 0x0202 , 0x04 }, /* Coarse integration time */
  { 0x0203 , 0xE7 },
  { 0x0204 , 0x00 }, /* Analog gain */
  { 0x0205 , 0x20 },
  { 0x0342 , 0x0D }, /* Line_length_pck 3470d */
  { 0x0343 , 0x8E },
  { 0x0340 , 0x09 }, /* Frame_length_lines 2480d */
  { 0x0341 , 0xB0 },
  /* Manufacture Setting */
  { 0x300E , 0xE9 },
  { 0x3085 , 0x01 },
  { 0x301D , 0x01 },
  { 0x3086 , 0x03 },
  { 0x3087 , 0x34 },
  { 0x3028 , 0x41 },
  { 0x3065 , 0x15 },
  /* ------------- */
  { 0x310C , 0x50 }, /* pclk invert */
  { 0x3117 , 0x0A }, /* H/V sync driving strength 6mA */
  { 0x3118 , 0xA3 }, /* parallel data driving strength 6mA */
  /* ------------- */

  /*{ 0x0100 , 0x01 },*/
};



struct s5k3h1gx_reg_t s5k3h1gx_regs = {
	.init_mipi = &s5k3h1gx_init_settings_array_mipi[0],
	.init_mipi_size = ARRAY_SIZE(s5k3h1gx_init_settings_array_mipi),
	.init_parallel = &s5k3h1gx_init_settings_array_parallel[0],
	.init_parallel_size = ARRAY_SIZE(s5k3h1gx_init_settings_array_parallel),

	.common_mipi = &s5k3h1gx_common_settings_array_mipi[0],
	.common_mipi_size = ARRAY_SIZE(s5k3h1gx_common_settings_array_mipi),
	.common_parallel = &s5k3h1gx_common_settings_array_parallel[0],
	.common_parallel_size = ARRAY_SIZE(s5k3h1gx_common_settings_array_parallel),

	.qtr_mipi = &s5k3h1gx_qtr_settings_array_mipi[0],
	.qtr_mipi_size = ARRAY_SIZE(s5k3h1gx_qtr_settings_array_mipi),
	.qtr_parallel = &s5k3h1gx_qtr_settings_array_parallel[0],
	.qtr_parallel_size = ARRAY_SIZE(s5k3h1gx_qtr_settings_array_parallel),

	.full_mipi = &s5k3h1gx_full_settings_array_mipi[0],
	.full_mipi_size = ARRAY_SIZE(s5k3h1gx_full_settings_array_mipi),
	.full_parallel = &s5k3h1gx_full_settings_array_parallel[0],
	.full_parallel_size = ARRAY_SIZE(s5k3h1gx_full_settings_array_parallel),
};
