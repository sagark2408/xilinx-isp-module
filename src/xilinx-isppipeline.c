// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 - 2023, Advanced Micro Devices, Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/xilinx-v4l2-controls.h>

#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "xilinx-gamma-correction.h"
#include "xilinx-isp-params.h"
#include "xilinx-vip.h"

#define XISP_AP_CTRL_REG		(0x0)
#define XISP_COMMON_CONFIG		(0x00010)
#define XISP_AEC_CONFIG			(0x00018)
#define XISP_AWB_CONFIG			(0x00020)
#define XISP_BLC_CONFIG_1		(0x00028)
#define XISP_BLC_CONFIG_2		(0x00030)
#define XISP_DEGAMMA_CONFIG_BASE		(0x01000)
#define XISP_GAIN_CONTROL_CONFIG_1	(0x00038)
#define XISP_GAIN_CONTROL_CONFIG_2	(0x00040)
#define XISP_GAMMA_CONFIG_BASE		(0x02000)
#define XISP_GTM_CONFIG_1		(0x00048)
#define XISP_GTM_CONFIG_2		(0x00050)
#define XISP_HDR_DECOM_CONFIG_BASE		(0x00100)
#define XISP_HDR_MERGE_CONFIG_BASE		(0x08000)
#define XISP_LTM_CONFIG			(0x00058)
#define XISP_RGBIR_CONFIG_BASE		(0x00200)
#define XISP_LUT3D_CONFIG_BASE		(0x80000)
#define XISP_LUT3D_CONFIG		(0x00060)
#define XISP_CLAHE_CONFIG_1		(0x00068)
#define XISP_CLAHE_CONFIG_2		(0x00070)
#define XISP_RESIZE_CONFIG		(0x00078)
#define XISP_PIPELINE_CONFIG_INFO		(0x00080)
#define XISP_MAX_SUPPORTED_SIZE		(0x00090)
#define XISP_FUNCS_AVAILABLE			(0x000a0)
#define XISP_FUNCS_BYPASSABLE		(0x000b0)
#define XISP_FUNCS_BYPASS_CONFIG		(0x000c0)

#define XISP_MIN_HEIGHT			(64)
#define XISP_MIN_WIDTH			(64)
#define XISP_MAX_HEIGHT			(4320)
#define XISP_MAX_WIDTH			(8192)
#define XISP_MIN_VALUE			(0)
#define XISP_MAX_VALUE			(65535)
#define XISP_GAMMA_LUT_LEN		(64)
#define XISP_NO_OF_PADS			(2)

#define XISP_HDR_OFFSET		(0x800)
#define XISP_RGBIR_CONFIG_BASE1	(XISP_RGBIR_CONFIG_BASE + 0x20)
#define XISP_RGBIR_CONFIG_BASE2	(XISP_RGBIR_CONFIG_BASE + 0x40)
#define XISP_RGBIR_CONFIG_BASE3	(XISP_RGBIR_CONFIG_BASE + 0x60)
#define XISP_RGBIR_CONFIG_BASE4	(XISP_RGBIR_CONFIG_BASE + 0x70)
#define XISP_RGBIR_CONFIG_BASE5	(XISP_RGBIR_CONFIG_BASE + 0x80)
#define XISP_NO_EXPS			(2)
#define XISP_WR_CV			(16384)
#define XISP_W_B_SIZE			(1024)
#define XISP_LUT3D_SIZE			(107811)
#define XISP_MAX_INTENSITY		(10 * (XISP_W_B_SIZE - 1))

#define XISP_1H				(100)
#define XISP_1K				(1000)
#define XISP_1M				(1000000)
#define XISP_1B				(1000000000)
#define XISP_10B			(10000000000)
#define XISP_1H_1B			(100000000000)
#define XISP_1T				(1000000000000)
#define XISP_CONST1			(4032000000)
#define XISP_CONST2			(3628800000000)
#define XISP_CONST3			(36288000000000)
#define XISP_CONST4			(399168000000)
#define XISP_CONST5			(62270208000000)

#define XISP_RESET_DEASSERT		(0)
#define XISP_RESET_ASSERT		(1)
#define XISP_START			BIT(0)
#define XISP_AUTO_RESTART		BIT(7)
#define XISP_STREAM_ON			(XISP_AUTO_RESTART | XISP_START)

static const u8 exposure_time[XISP_NO_EXPS] = {100, 25};

enum xisp_bayer_format {
	XISP_BGGR = 0,
	XISP_GBRG = 1,
	XISP_GRBG = 2,
	XISP_RGGB = 3
};

/*
 * struct xisp_dev - Xilinx ISP pipeline device structure
 * @xvip: Xilinx Video IP device
 * @pads: media pads
 * @formats: V4L2 media bus formats
 * @ctrl_handler: V4L2 Control Handler
 * @bayer_fmt: IP or Hardware specific video format
 * @rst_gpio: GPIO reset line to bring ISP pipeline out of reset
 * @npads: number of pads
 * @width: Current frame width
 * @height: Current frame height
 * @max_width: Maximum width supported by this instance
 * @max_height: Maximum height supported by this instance
 * @in_type: Expected input type i.e. mono or color
 * @out_type: Expected output type i.e. mono or color
 * @in_bw_mode: Expected input bit width i.e. 8, 10, 12, 14, 16, 24
 * @out_bw_mode: Expected output bit width i.e. 8, 10, 12, 14, 16, 24
 * @nppc: Expected number of pixels per clock
 * @num_streams: Expected number of streams for multi-streaming support
 * @threshold_aec: Expected threshold for auto exposure correction
 * @threshold_awb: Expected threshold for auto white balance
 * @xisp_info_reg_0_data: Extracted values from XISP_PIPELINE_CONFIG_INFO
 * @xisp_info_reg_1_data: Extracted values from XISP_MAX_SUPPORTED_SIZE
 * @xisp_info_reg_2_data: Extracted values from XISP_FUNCS_AVAILABLE
 * @xisp_info_reg_3_data: Extracted values from XISP_FUNCS_BYPASSABLE
 * @bypass_isp: Track if isp is enabled or not from user
 * @bypass_hdr: Track if hdr is enabled or not from user
 * @bypass_rgbir: Track if rgbir is enabled or not from user
 * @bypass_aec: Track if aec is enabled or not from user
 * @bypass_blc: Track if blc is enabled or not from user
 * @bypass_bpc: Track if bpc is enabled or not from user
 * @bypass_degamma: Track if degamma is enabled or not from user
 * @bypass_lsc: Track if lsc is enabled or not from user
 * @bypass_gain: Track if gain is enabled or not from user
 * @bypass_demosaic: Track if demosaic is enabled or not from user
 * @bypass_awb: Track if awb is enabled or not from user
 * @bypass_ccm: Track if ccm is enabled or not from user
 * @bypass_tm: Track if tone mapping is enabled or not from user
 * @bypass_gamma: Track if gamma is enabled or not from user
 * @bypass_lut3d: Track if 3D lut is enabled or not from user
 * @bypass_csc: Track if csc is enabled or not from user
 * @bypass_bayer_stats: Track if bayer stats is enabled or not from user
 * @bypass_luma_stats: Track if luma stats is enabled or not from user
 * @bypass_rgb_stats: Track if rgb stats is enabled or not from user
 * @bypass_clahe: Track if clahe is enabled or not from user
 * @bypass_median: Track if median is enabled or not from user
 * @bypass_resize: Track if resize is enabled or not from user
 * @isp_bypass_en: Track if isp is bypassed or not
 * @aec_bypass_en: Track if aec is bypassed or not
 * @blc_bypass_en: Track if blc is bypassed or not
 * @bpc_bypass_en: Track if bpc is bypassed or not
 * @degamma_bypass_en: Track if degamma is bypassed or not
 * @lsc_bypass_en: Track if lsc is bypassed or not
 * @gain_bypass_en: Track if gain is bypassed or not
 * @demosaic_bypass_en: Track if demosaic is bypassed or not
 * @awb_bypass_en: Track if awb is bypassed or not
 * @ccm_bypass_en: Track if ccm is bypassed or not
 * @tm_bypass_en: Track if tone mapping is bypassed or not
 * @gamma_bypass_en: Track if gamma is bypassed or not
 * @lut3d_bypass_en: Track if lut3d is bypassed or not
 * @csc_bypass_en: Track if csc is bypassed or not
 * @bayer_stats_bypass_en: Track if bayer stats is bypassed or not
 * @luma_stats_bypass_en: Track if luma stats is bypassed or not
 * @rgb_stats_bypass_en: Track if rgb stats is bypassed or not
 * @hdr_bypass_en: Track if hdr is bypassed or not
 * @rgbir_bypass_en: Track if rgbir is bypassed or not
 * @clahe_bypass_en: Track if clahe is bypassed or not
 * @median_bypass_en: Track if median is bypassed or not
 * @resize_bypass_en: Track if resize is bypassed or not
 * @aec_en: Track if aec is enabled or not
 * @awb_en: Track if awb is enabled or not
 * @ccm_en: Track if ccm is enabled or not
 * @blc_en: Track if blc is enabled or not
 * @bpc_en: Track if bpc is enabled or not
 * @degamma_en: Track if degamma is enabled or not
 * @lsc_en: Track if lsc is enabled or not
 * @gain_en: Track if gain is enabled or not
 * @demosaic_en: Track if demosaic is enabled or not
 * @gamma_en: Track if gamma is enabled or not
 * @tm_en: Track if tone mapping is enabled or not
 * @hdr_mode: Track which hdr mode is enabled
 * @hdr_en: Track if hdr is enabled or not
 * @rgbir_en: Track if rgbir is enabled or not
 * @lut3d_en: Track if 3d lut is enabled or not
 * @csc_en: Track if csc is enabled or not
 * @bayer_stats_en: Track if bayer stats is enabled or not
 * @luma_stats_en: Track if luma stats is enabled or not
 * @rgb_stats_en: Track if rgb stats is enabled or not
 * @clahe_en: Track if clahe is enabled or not
 * @median_en: Track if median is enabled or not
 * @tm_type: Flag to indicate the type of tone mapping enabled
 * @mult_factor: Expected multiplication factor
 * @black_level: Expected black level
 * @rgain: Expected red gain
 * @bgain: Expected blue gain
 * @ggain: Expected green gain
 * @luma_gain: Expected luminance gain
 * @gtm_c1: Expected gtm c1 value
 * @gtm_c2: Expected gtm c2 value
 * @lut3d_dim: Expected 3d lut dimension
 * @block_rows: Expected number of block rows
 * @block_cols: Expected number of block cols
 * @clip: Expected clip value
 * @tilesY: Expected number of tiles along y-axis
 * @tilesX: Expected number of tiles along x-axis
 * @intersec: Expected intersec value
 * @weights1: Expected weights1 values
 * @weights2: Expected weights2 values
 * @rho: Expected rho value
 * @alpha: Expected alpha value
 * @optical_black_value: Expected optical black value
 * @resize_new_height: Expected resize new height
 * @resize_new_width: Expected resize new width
 * @degamma_select: Expected degamma array values
 * @decompand_select: Expected decompand array values
 * @lut3d: Expected 3d lut values
 * @mono_lut: Pointer to the gamma coefficient as per the mono Gamma control
 * @red_lut: Pointer to the gamma coefficient as per the Red Gamma control
 * @green_lut: Pointer to the gamma coefficient as per the Green Gamma control
 * @blue_lut: Pointer to the gamma coefficient as per the Blue Gamma control
 * @gamma_table: Pointer to the table containing various gamma values
 * @degamma_lut: Pointer to the degamma coefficient as per the degamma control
 * @decompand_lut: Pointer to the decompand coefficient as per the decompand control
 */
struct xisp_dev {
	struct xvip_device xvip;
	struct media_pad pads[XISP_NO_OF_PADS];
	struct v4l2_mbus_framefmt formats[XISP_NO_OF_PADS];
	struct v4l2_ctrl_handler ctrl_handler;
	enum xisp_bayer_format bayer_fmt;
	struct gpio_desc *rst_gpio;
	u16 npads;
	u16 width;
	u16 height;
	u16 max_width;
	u16 max_height;
	bool in_type;
	bool out_type;
	u8 in_bw_mode;
	u8 out_bw_mode;
	u8 nppc;
	u8 num_streams;
	u16 threshold_aec;
	u16 threshold_awb;
	u32 xisp_info_reg_0_data;
	u32 xisp_info_reg_1_data;
	u32 xisp_info_reg_2_data;
	u32 xisp_info_reg_3_data;
	bool bypass_isp;
	bool bypass_hdr;
	bool bypass_rgbir;
	bool bypass_aec;
	bool bypass_blc;
	bool bypass_bpc;
	bool bypass_degamma;
	bool bypass_lsc;
	bool bypass_gain;
	bool bypass_demosaic;
	bool bypass_awb;
	bool bypass_ccm;
	bool bypass_tm;
	bool bypass_gamma;
	bool bypass_lut3d;
	bool bypass_csc;
	bool bypass_bayer_stats;
	bool bypass_luma_stats;
	bool bypass_rgb_stats;
	bool bypass_clahe;
	bool bypass_median;
	bool bypass_resize;
	bool isp_bypass_en;
	bool aec_bypass_en;
	bool blc_bypass_en;
	bool bpc_bypass_en;
	bool degamma_bypass_en;
	bool lsc_bypass_en;
	bool gain_bypass_en;
	bool demosaic_bypass_en;
	bool awb_bypass_en;
	bool ccm_bypass_en;
	bool tm_bypass_en;
	bool gamma_bypass_en;
	bool lut3d_bypass_en;
	bool csc_bypass_en;
	bool bayer_stats_bypass_en;
	bool luma_stats_bypass_en;
	bool rgb_stats_bypass_en;
	bool hdr_bypass_en;
	bool rgbir_bypass_en;
	bool clahe_bypass_en;
	bool median_bypass_en;
	bool resize_bypass_en;
	bool aec_en;
	bool awb_en;
	bool ccm_en;
	bool blc_en;
	bool bpc_en;
	bool degamma_en;
	bool lsc_en;
	bool gain_en;
	bool demosaic_en;
	bool gamma_en;
	bool tm_en;
	bool hdr_mode;
	bool hdr_en;
	bool rgbir_en;
	bool lut3d_en;
	bool csc_en;
	bool bayer_stats_en;
	bool luma_stats_en;
	bool rgb_stats_en;
	bool clahe_en;
	bool median_en;
	bool resize_en;
	u8 tm_type;
	u32 mult_factor;
	u32 black_level;
	u16 rgain;
	u16 bgain;
	u16 ggain;
	u16 luma_gain;
	u32 gtm_c1;
	u32 gtm_c2;
	u32 lut3d_dim;
	u16 block_rows;
	u16 block_cols;
	u16 clip;
	u16 tilesY;
	u16 tilesX;
	u32 intersec;
	u16 weights1[XISP_W_B_SIZE];
	u16 weights2[XISP_W_B_SIZE];
	u16 rho;
	u8 alpha;
	u8 optical_black_value;
	u16 resize_new_height;
	u16 resize_new_width;
	u8 degamma_select;
	u8 decompand_select;
	const u32 *lut3d;
	const u32 *mono_lut;
	const u32 *red_lut;
	const u32 *green_lut;
	const u32 *blue_lut;
	const u32 **gamma_table;
	const u32 (*degamma_lut)[8][3];
	const u32 (*decompand_lut)[4][3];
};

static inline struct xisp_dev *to_xisp(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct xisp_dev, xvip.subdev);
}

/**
 * compute_data_reliability_weight - Computes exponential value
 * @c_intersec: Expected c_intersec value
 * @mu_h: Expected mu_h value
 * @mu_l: Expected mu_l value
 * @r: Expected rv value
 */
static u16 compute_data_reliability_weight(u32 c_intersec, u32 mu_h, u32 mu_l, u32 r)
{
	u16 wr, wr1;

	if (r < mu_l) {
		u32 x = int_pow((mu_l - r), 2);
		u64 xpow = (c_intersec * x) / XISP_1M;

		wr = XISP_WR_CV / int_pow(3, xpow);

	} else if (r < mu_h) {
		wr = XISP_WR_CV;

	/* Computes exponential using taylor series expansion */
	} else {
		u32 x = int_pow((r - mu_h), 2);
		u64 xpow = c_intersec * x;
		u64 taylor_coeff_1 = xpow / XISP_1K;
		u64 int_coeff_21 = (int_pow(xpow, 2) / (XISP_1H * XISP_1K));
		u64 taylor_coeff_2 = int_coeff_21 / (20 * XISP_1K);
		u64 int_coeff_31 = xpow / (6 * XISP_1K);
		u64 taylor_coeff_3 = (int_coeff_21 * int_coeff_31) / (10 * XISP_1M);
		u64 taylor_coeff_4 = (int_coeff_21 * int_coeff_21) / (24 * XISP_1H_1B);
		u64 int_coeff_51 = (int_coeff_21 * xpow) / (120 * XISP_1M);
		u64 taylor_coeff_5 = (int_coeff_21 * int_coeff_51) / XISP_1H_1B;
		u64 int_coeff_61 = (int_coeff_21 * int_coeff_21) / (72 * XISP_1M);
		u64 taylor_coeff_6 = (int_coeff_61 * int_coeff_21) / (10 * XISP_1T);
		u64 int_coeff_71 = (int_coeff_21 * xpow) / 5040;
		u64 int_coeff_72 = (int_coeff_21 * int_coeff_21) / XISP_10B;
		u64 taylor_coeff_7 = (int_coeff_71 * int_coeff_72) / (XISP_1H * XISP_1T);
		u64 int_coeff_81 = (int_coeff_21 * int_coeff_21) / XISP_10B;
		u64 int_coeff_82 = (int_coeff_21 * int_coeff_21) / XISP_10B;
		u64 taylor_coeff_8 = (int_coeff_81 * int_coeff_82) / XISP_CONST1;
		u64 int_coeff_91 = (int_coeff_21 * int_coeff_21) / (XISP_1H * XISP_1M);
		u64 int_coeff_92 = (int_coeff_21 * xpow) / (XISP_1H * XISP_1K);
		u64 int_coeff_93 = (int_coeff_91 * int_coeff_92) / XISP_CONST2;
		u64 taylor_coeff_9 = (int_coeff_93 * int_coeff_21) / XISP_1H_1B;
		u64 int_coeff_101 = (int_coeff_21 * int_coeff_21) / XISP_10B;
		u64 int_coeff_102 = (int_coeff_21 * int_coeff_21) / XISP_10B;
		u64 int_coeff_103 = int_coeff_21 / (XISP_1H * XISP_1K);
		u64 taylor_coeff_10 = (int_coeff_101 * int_coeff_102 * int_coeff_103) /
				       XISP_CONST3;
		u64 int_coeff_111 = (int_coeff_21 * int_coeff_21) / XISP_1H_1B;
		u64 int_coeff_112 = (xpow * int_coeff_21) / XISP_1T;
		u64 int_coeff_113 = (int_coeff_111 * int_coeff_112 * int_coeff_21) /
				    XISP_CONST4;
		u64 taylor_coeff_11 = (int_coeff_113 * int_coeff_21) / XISP_1H_1B;
		u64 int_coeff_121 = (int_coeff_111 * int_coeff_21) / XISP_10B;
		u64 int_coeff_122 = (int_coeff_121 * int_coeff_21) / 479001600;
		u64 int_coeff_123 = (int_coeff_122 * int_coeff_21) / XISP_1M;
		u64 taylor_coeff_12 = (int_coeff_123 * int_coeff_21) / XISP_1T;
		u64 int_coeff_131 = (int_coeff_21 * int_coeff_21) / XISP_1H_1B;
		u64 int_coeff_132 = (int_coeff_131 * int_coeff_112 * int_coeff_21) /
				    XISP_1T;
		u64 int_coeff_133 = (int_coeff_132 * int_coeff_21) / XISP_1M;
		u64 taylor_coeff_13 = (int_coeff_133 * int_coeff_21) / XISP_CONST5;
		u64 int_coeff_141 = (int_coeff_131 * int_coeff_21) / (XISP_1H * XISP_1K);
		u64 int_coeff_142 = (int_coeff_131 * int_coeff_141) / (XISP_1H * XISP_1K);
		u64 taylor_coeff_14 = (int_coeff_131 * int_coeff_142) / (87178291200 * XISP_1K);

		wr1 = XISP_1K - taylor_coeff_1 + taylor_coeff_2 - taylor_coeff_3 + taylor_coeff_4
			- taylor_coeff_5 + taylor_coeff_6 - taylor_coeff_7 + taylor_coeff_8 -
			taylor_coeff_9 + taylor_coeff_10 - taylor_coeff_11 + taylor_coeff_12 -
			taylor_coeff_13 + taylor_coeff_14;

		wr = (wr1 * XISP_WR_CV) / XISP_1K;
	}

	return wr;
}

/**
 * HDR_merge - For computing weight value
 * @alpha:  Expected alpha value
 * @optical_black_value:  Expected optical black value
 * @intersec:  Expected intersec value
 * @rho:  Expected rho value
 * @weights1:  Computed weights1
 * @weights2:  Computed weights2
 */
static void HDR_merge(u32 alpha, u32 optical_black_value, u32 intersec,
		      u32 rho, u16 weights1[XISP_W_B_SIZE], u16 weights2[XISP_W_B_SIZE])
{
	u32 mu_h[XISP_NO_EXPS] = {0, 0};
	u32 mu_l[XISP_NO_EXPS] = {0, 0};
	u32 gamma_out[XISP_NO_EXPS] = {0, 0};
	u32 value_max = (XISP_MAX_INTENSITY - optical_black_value) / alpha;
	u32 c_inters;
	u16 wr_ocv[XISP_NO_EXPS][XISP_W_B_SIZE];
	int m = XISP_NO_EXPS;
	int i, j;

	for (i = 0; i < m - 1; i++) {
		gamma_out[i] = (10 * (rho * (XISP_MAX_INTENSITY - optical_black_value)
				- optical_black_value * (XISP_MAX_INTENSITY / 10 - rho))) /
				(exposure_time[i] * rho + exposure_time[i + 1] *
				(XISP_MAX_INTENSITY / 10 - rho));
	}

	for (i = 0; i < m - 1; i++) {
		if (i == 0) {
			u32 value = (10 * rho - optical_black_value) / alpha;

			mu_h[i] = (100 * value) / exposure_time[0];
		} else {
			mu_h[i] = gamma_out[i] - (gamma_out[i - 1] - mu_h[i - 1]);
		}
		mu_l[i + 1] = 2 * gamma_out[i] - mu_h[i];
	}

	mu_h[m - 1] = (100 * value_max) / exposure_time[m - 1];
	c_inters = (intersec / (int_pow((gamma_out[0] - mu_h[0]), 2)));

	for (i = 0; i < XISP_NO_EXPS; i++) {
		for (j = 0; j < (XISP_W_B_SIZE); j++) {
			u32 rv = (u32)((100 * j) / exposure_time[i]);

			wr_ocv[i][j] = compute_data_reliability_weight(c_inters,
								       mu_h[i], mu_l[i], rv);
		}
	}

	for (i = 0; i < XISP_W_B_SIZE; i++)
		weights1[i] = wr_ocv[0][i];

	for (i = 0; i < XISP_W_B_SIZE; i++)
		weights2[i] = wr_ocv[1][i];
}

/**
 * extract_register_bits - To extract bits from a specific position
 * and return the extracted value as integer
 * @data: register which is used for extraction
 * @n: number of bits to be extracted
 * @p: position of the data bits
 */
int extract_register_bits(int data, int n, int p)
{
	return (((1 << n) - 1) & (data >> p));
}

/*
 * xisp_set_lut_entries - Write to a field in ISP pipeline registers
 *
 * @xisp:	The xisp_dev
 * @lut:	The value to write
 * @lut_base:	The field to write to
 *
 * This function allows writing to gamma lut array.
 */

static void xisp_set_lut_entries(struct xisp_dev *xisp, const u32 *lut, const u32 lut_base)
{
	int itr;
	u32 lut_offset;

	lut_offset = lut_base;

	for (itr = 0; itr < XISP_GAMMA_LUT_LEN; itr = itr + 1) {
		xvip_write(&xisp->xvip, lut_offset, lut[itr]);
		lut_offset += 4;
	}
}

static void select_gamma(u32 value, const u32 **coeff, const u32 **xgamma_curves)
{
	*coeff = *(xgamma_curves + value - 1);
}

static void xisp_hdrmg_set_lut_entries(struct xisp_dev *xisp,
				       const u32 lut_base,
				       const u16 lut[XISP_W_B_SIZE],
				       const u16 lut1[XISP_W_B_SIZE])
{
	u32 lut_data;
	int i, j, ival;
	u32 lut_offset[8];

	for (j = 0; j < ARRAY_SIZE(lut_offset); j++)
		lut_offset[j] = lut_base + (j * XISP_HDR_OFFSET);

	for (i = 0; i < ARRAY_SIZE(lut_offset); i++) {
		for (ival = 0; ival < XISP_W_B_SIZE; ival += 2) {
			if (i % 2 == 0)
				lut_data = (lut[ival + 1] << 16) | lut[ival];
			else
				lut_data = (lut1[ival + 1] << 16) | lut1[ival];

			xvip_write(&xisp->xvip, lut_offset[i], lut_data);
			lut_offset[i] += 4;
		}
	}
}

static void xisp_set_lut3d_entries(struct xisp_dev *xisp, const u32 lut3d_offset, const u32 *lut3d)
{
	int i;

	for (i = 0; i < XISP_LUT3D_SIZE; i = i + 1)
		xvip_write(&xisp->xvip, lut3d_offset, lut3d[i]);
}

static void xisp_set_degamma_entries(struct xisp_dev *xisp, const u32 degamma_offset,
				     const u32 (*degamma)[8][3])
{
	int i, j, k;

	for (i = 0; i < 3; i = i + 1)
		for (j = 0; j < 8; j = j + 1)
			for (k = 0; k < 3; k = k + 1)
				xvip_write(&xisp->xvip, degamma_offset, degamma[i][j][k]);
}

static void xisp_set_decomp_entries(struct xisp_dev *xisp, const u32 decomp_offset,
				    const u32 (*decomp)[4][3])
{
	int i, j, k;

	for (i = 0; i < 3; i = i + 1)
		for (j = 0; j < 4; j = j + 1)
			for (k = 0; k < 3; k = k + 1)
				xvip_write(&xisp->xvip, decomp_offset, decomp[i][j][k]);
}

static void xisp_set_rgbir_entries(struct xisp_dev *xisp, const u32 rgbir_offset, const s8
				    *rgbir_param, int size)
{
	int i;

	for (i = 0; i < size; i = i + 1)
		xvip_write(&xisp->xvip, rgbir_offset, rgbir_param[i]);
}

static void xisp_set_info_reg_4(struct xisp_dev *xisp, bool bypass_isp, bool bypass_hdr,
				bool bypass_rgbir, bool bypass_aec, bool bypass_blc,
				bool bypass_bpc, bool bypass_degamma, bool bypass_lsc,
				bool bypass_gain, bool bypass_demosaic, bool bypass_awb,
				bool bypass_ccm, bool bypass_tm, bool bypass_gamma,
				bool bypass_lut3d, bool bypass_csc, bool bypass_bayer_stats,
				bool bypass_luma_stats, bool bypass_rgb_stats,
				bool bypass_clahe, bool bypass_median, bool bypass_resize)
{
	u32 data;

	data = (u32)(bypass_isp) | ((u32)(bypass_hdr) << 1) | ((u32)(bypass_rgbir) << 2) |
		((u32)(bypass_aec) << 3) | ((u32)(bypass_blc) << 4) | ((u32)(bypass_bpc) << 5) |
		((u32)(bypass_degamma) << 6) | ((u32)(bypass_lsc) << 7) |
		((u32)(bypass_gain) << 8) | ((u32)(bypass_demosaic) << 9) |
		((u32)(bypass_awb) << 10) | ((u32)(bypass_ccm) << 11) |
		((u32)(bypass_tm) << 12) | ((u32)(bypass_gamma) << 15) |
		((u32)(bypass_lut3d) << 16) | ((u32)(bypass_csc) << 17) |
		((u32)(bypass_bayer_stats) << 18) | ((u32)(bypass_luma_stats) << 19) |
		((u32)(bypass_rgb_stats) << 20) | ((u32)(bypass_clahe) << 21) |
		((u32)(bypass_median) << 22) | ((u32)(bypass_resize) << 23);
	xvip_write(&xisp->xvip, XISP_FUNCS_BYPASS_CONFIG, data);
}

static int xisp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct xisp_dev *xisp =
		container_of(ctrl->handler,
			     struct xisp_dev, ctrl_handler);
	switch (ctrl->id) {
	case V4L2_CID_XILINX_ISP_AEC_EN:
		xisp->bypass_aec = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_THRESHOLD:
		xisp->threshold_aec = ctrl->val;
		if (xisp->aec_en)
			if (!xisp->aec_bypass_en || !(xisp->aec_bypass_en & xisp->bypass_aec))
				xvip_write(&xisp->xvip, XISP_AEC_CONFIG, xisp->threshold_aec);
		break;
	case V4L2_CID_XILINX_ISP_AWB_EN:
		xisp->bypass_awb = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_AWB:
		xisp->threshold_awb = ctrl->val;
		if (xisp->awb_en)
			if (!xisp->awb_bypass_en || !(xisp->awb_bypass_en & xisp->bypass_awb))
				xvip_write(&xisp->xvip, XISP_AWB_CONFIG, xisp->threshold_awb);
		break;
	case V4L2_CID_XILINX_ISP_BLC_EN:
		xisp->bypass_blc = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_BPC_EN:
		xisp->bypass_bpc = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_RGBIR_EN:
		xisp->bypass_rgbir = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_DEGAMMA_EN:
		xisp->bypass_degamma = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_DEGAMMA_PARAMS:
		xisp->degamma_select = ctrl->val;
		if (xisp->degamma_en)
			if (!xisp->degamma_bypass_en || !(xisp->degamma_bypass_en &
			    xisp->bypass_degamma)) {
				xisp->degamma_lut = xisp_degamma_choices[xisp->degamma_select];
				xisp_set_degamma_entries(xisp, XISP_DEGAMMA_CONFIG_BASE,
							 xisp->degamma_lut);
			}
		break;
	case V4L2_CID_XILINX_ISP_LSC_EN:
		xisp->bypass_lsc = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_DEMOSAIC_EN:
		xisp->bypass_demosaic = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_CCM_EN:
		xisp->bypass_ccm = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_MULTI_FACTOR:
		xisp->mult_factor = ctrl->val;
		if (xisp->blc_en)
			if (!xisp->blc_bypass_en || !(xisp->blc_bypass_en & xisp->bypass_blc))
				xvip_write(&xisp->xvip, XISP_BLC_CONFIG_1, xisp->mult_factor);
		break;
	case V4L2_CID_BLACK_LEVEL:
		xisp->black_level = ctrl->val;
		if (xisp->blc_en)
			if (!xisp->blc_bypass_en || !(xisp->blc_bypass_en & xisp->bypass_blc))
				xvip_write(&xisp->xvip, XISP_BLC_CONFIG_2, xisp->black_level);
		break;
	case V4L2_CID_XILINX_ISP_GAIN_EN:
		xisp->bypass_gain = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_RED_GAIN:
		xisp->rgain = ctrl->val;
		if (xisp->gain_en)
			if (!xisp->gain_bypass_en || !(xisp->gain_bypass_en & xisp->bypass_gain))
				if (xisp->in_type)
					xvip_write(&xisp->xvip, XISP_GAIN_CONTROL_CONFIG_1,
						   (xisp->bgain << 16) | xisp->rgain);
		break;
	case V4L2_CID_XILINX_ISP_BLUE_GAIN:
		xisp->bgain = ctrl->val;
		if (xisp->gain_en)
			if (!xisp->gain_bypass_en || !(xisp->gain_bypass_en & xisp->bypass_gain))
				if (xisp->in_type)
					xvip_write(&xisp->xvip, XISP_GAIN_CONTROL_CONFIG_1,
						   (xisp->bgain << 16) | xisp->rgain);
		break;
	case V4L2_CID_XILINX_ISP_GREEN_GAIN:
		xisp->ggain = ctrl->val;
		if (xisp->gain_en)
			if (!xisp->gain_bypass_en || !(xisp->gain_bypass_en & xisp->bypass_gain))
				if (xisp->in_type)
					xvip_write(&xisp->xvip, XISP_GAIN_CONTROL_CONFIG_2,
						   (xisp->bayer_fmt << 16) | xisp->ggain);
		break;
	case V4L2_CID_XILINX_ISP_GAMMA_EN:
		xisp->bypass_gamma = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_RED_GAMMA:
		select_gamma(ctrl->val, &xisp->red_lut, xisp->gamma_table);
		dev_dbg(xisp->xvip.dev, "Setting Red Gamma to %d.%d",
			ctrl->val / 10, ctrl->val % 10);
		if (xisp->gamma_en)
			if (!xisp->gamma_bypass_en || !(xisp->gamma_bypass_en & xisp->bypass_gamma))
				if (xisp->in_type)
					xisp_set_lut_entries(xisp, xisp->red_lut,
							     XISP_GAMMA_CONFIG_BASE);
		break;
	case V4L2_CID_XILINX_ISP_GREEN_GAMMA:
		select_gamma(ctrl->val, &xisp->green_lut, xisp->gamma_table);
		dev_dbg(xisp->xvip.dev, "Setting Green Gamma to %d.%d",
			ctrl->val / 10, ctrl->val % 10);
		if (xisp->gamma_en)
			if (!xisp->gamma_bypass_en || !(xisp->gamma_bypass_en & xisp->bypass_gamma))
				if (xisp->in_type)
					xisp_set_lut_entries(xisp, xisp->green_lut,
							     (XISP_GAMMA_CONFIG_BASE + 0x100));
		break;
	case V4L2_CID_XILINX_ISP_BLUE_GAMMA:
		select_gamma(ctrl->val, &xisp->blue_lut, xisp->gamma_table);
		dev_dbg(xisp->xvip.dev, "Setting Blue Gamma to %d.%d",
			ctrl->val / 10, ctrl->val % 10);
		if (xisp->gamma_en)
			if (!xisp->gamma_bypass_en || !(xisp->gamma_bypass_en & xisp->bypass_gamma))
				if (xisp->in_type)
					xisp_set_lut_entries(xisp, xisp->blue_lut,
							     (XISP_GAMMA_CONFIG_BASE + 0x200));
		break;
	case V4L2_CID_XILINX_ISP_HDR_EN:
		xisp->bypass_hdr = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_ALPHA:
		xisp->alpha = ctrl->val;
		HDR_merge(xisp->alpha, xisp->optical_black_value, xisp->intersec,
			  xisp->rho, xisp->weights1, xisp->weights2);
		if (xisp->hdr_en)
			if (!xisp->hdr_bypass_en || !(xisp->hdr_bypass_en & xisp->bypass_hdr))
				if (!xisp->hdr_mode)
					xisp_hdrmg_set_lut_entries(xisp, XISP_HDR_MERGE_CONFIG_BASE,
								   xisp->weights1, xisp->weights2);
		break;
	case V4L2_CID_XILINX_ISP_OPTICAL_BLACK_VALUE:
		xisp->optical_black_value = ctrl->val;
		HDR_merge(xisp->alpha, xisp->optical_black_value, xisp->intersec,
			  xisp->rho, xisp->weights1, xisp->weights2);
		if (xisp->hdr_en)
			if (!xisp->hdr_bypass_en || !(xisp->hdr_bypass_en & xisp->bypass_hdr))
				if (!xisp->hdr_mode)
					xisp_hdrmg_set_lut_entries(xisp, XISP_HDR_MERGE_CONFIG_BASE,
								   xisp->weights1, xisp->weights2);
		break;
	case V4L2_CID_XILINX_ISP_INTERSEC:
		xisp->intersec = ctrl->val;
		HDR_merge(xisp->alpha, xisp->optical_black_value, xisp->intersec,
			  xisp->rho, xisp->weights1, xisp->weights2);
		if (xisp->hdr_en)
			if (!xisp->hdr_bypass_en || !(xisp->hdr_bypass_en & xisp->bypass_hdr))
				if (!xisp->hdr_mode)
					xisp_hdrmg_set_lut_entries(xisp, XISP_HDR_MERGE_CONFIG_BASE,
								   xisp->weights1, xisp->weights2);
		break;
	case V4L2_CID_XILINX_ISP_RHO:
		xisp->rho = ctrl->val;
		HDR_merge(xisp->alpha, xisp->optical_black_value, xisp->intersec,
			  xisp->rho, xisp->weights1, xisp->weights2);
		if (xisp->hdr_en)
			if (!xisp->hdr_bypass_en || !(xisp->hdr_bypass_en & xisp->bypass_hdr))
				if (!xisp->hdr_mode)
					xisp_hdrmg_set_lut_entries(xisp, XISP_HDR_MERGE_CONFIG_BASE,
								   xisp->weights1, xisp->weights2);
		break;
	case V4L2_CID_XILINX_ISP_DECOMPAND_PARAMS:
		xisp->decompand_select = ctrl->val;
		if (xisp->hdr_en)
			if (!xisp->hdr_bypass_en || !(xisp->hdr_bypass_en & xisp->bypass_hdr))
				if (xisp->hdr_mode) {
					xisp->decompand_lut =
					xisp_decompand_choices[xisp->decompand_select];
					xisp_set_decomp_entries(xisp, XISP_HDR_DECOM_CONFIG_BASE,
								xisp->decompand_lut);
				}
		break;
	case V4L2_CID_XILINX_ISP_TM_EN:
		xisp->bypass_tm = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_GTM_C1:
		xisp->gtm_c1 = ctrl->val;
		if (xisp->tm_en)
			if (!xisp->tm_bypass_en || !(xisp->tm_bypass_en & xisp->bypass_tm))
				if (xisp->tm_type == 1)
					xvip_write(&xisp->xvip, XISP_GTM_CONFIG_1, xisp->gtm_c1);
		break;
	case V4L2_CID_XILINX_ISP_GTM_C2:
		xisp->gtm_c2 = ctrl->val;
		if (xisp->tm_en)
			if (!xisp->tm_bypass_en || !(xisp->tm_bypass_en & xisp->bypass_tm))
				if (xisp->tm_type == 1)
					xvip_write(&xisp->xvip, XISP_GTM_CONFIG_2, xisp->gtm_c2);
		break;
	case V4L2_CID_XILINX_ISP_3DLUT_EN:
		xisp->bypass_lut3d = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_3DLUT_DIM:
		xisp->lut3d_dim = ctrl->val;
		if (xisp->lut3d_en)
			if (!xisp->lut3d_en || !(xisp->lut3d_en & xisp->bypass_lut3d))
				xvip_write(&xisp->xvip, XISP_LUT3D_CONFIG, xisp->lut3d_dim);
		break;
	case V4L2_CID_XILINX_ISP_CSC_EN:
		xisp->bypass_csc = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_BLOCK_ROWS:
		xisp->block_rows = ctrl->val;
		if (xisp->tm_en)
			if (!xisp->tm_bypass_en || !(xisp->tm_bypass_en & xisp->bypass_tm))
				if (xisp->tm_type == 0)
					xvip_write(&xisp->xvip, XISP_LTM_CONFIG, (xisp->block_rows
						   << 16) | xisp->block_cols);
		break;
	case V4L2_CID_XILINX_ISP_BLOCK_COLS:
		xisp->block_cols = ctrl->val;
		if (xisp->tm_en)
			if (!xisp->tm_bypass_en || !(xisp->tm_bypass_en & xisp->bypass_tm))
				if (xisp->tm_type == 0)
					xvip_write(&xisp->xvip, XISP_LTM_CONFIG, (xisp->block_rows
						   << 16) | xisp->block_cols);
		break;
	case V4L2_CID_XILINX_ISP_BAYER_STATS_EN:
		xisp->bypass_bayer_stats = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_LUMA_STATS_EN:
		xisp->bypass_luma_stats = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_RGB_STATS_EN:
		xisp->bypass_rgb_stats = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_LUMA_GAIN:
		xisp->luma_gain = ctrl->val;
		if (xisp->gain_en)
			if (!xisp->gain_bypass_en || !(xisp->gain_bypass_en & xisp->bypass_gain))
				if (!xisp->in_type)
					xvip_write(&xisp->xvip, XISP_GAIN_CONTROL_CONFIG_1,
						   xisp->luma_gain);
		break;
	case V4L2_CID_GAMMA:
		select_gamma(ctrl->val, &xisp->mono_lut, xisp->gamma_table);
		dev_dbg(xisp->xvip.dev, "Setting Gamma to %d.%d",
			ctrl->val / 10, ctrl->val % 10);
		if (xisp->gamma_en)
			if (!xisp->gamma_bypass_en || !(xisp->gamma_bypass_en & xisp->bypass_gamma))
				if (!xisp->in_type)
					xisp_set_lut_entries(xisp, xisp->mono_lut,
							     XISP_GAMMA_CONFIG_BASE);
		break;
	case V4L2_CID_XILINX_ISP_CLAHE_EN:
		xisp->bypass_clahe = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_CLIP:
		xisp->clip = ctrl->val;
		if (xisp->clahe_en)
			if (!xisp->clahe_bypass_en || !(xisp->clahe_bypass_en & xisp->bypass_clahe))
				xvip_write(&xisp->xvip, XISP_CLAHE_CONFIG_1, xisp->clip);
		break;
	case V4L2_CID_XILINX_ISP_TILESY:
		xisp->tilesY = ctrl->val;
		if (xisp->clahe_en)
			if (!xisp->clahe_bypass_en || !(xisp->clahe_bypass_en & xisp->bypass_clahe))
				xvip_write(&xisp->xvip, XISP_CLAHE_CONFIG_2, (xisp->tilesY << 16) |
					   xisp->tilesX);
		break;
	case V4L2_CID_XILINX_ISP_TILESX:
		xisp->tilesX = ctrl->val;
		if (xisp->clahe_en)
			if (!xisp->clahe_bypass_en || !(xisp->clahe_bypass_en & xisp->bypass_clahe))
				xvip_write(&xisp->xvip, XISP_CLAHE_CONFIG_2, (xisp->tilesY << 16) |
					   xisp->tilesX);
		break;
	case V4L2_CID_XILINX_ISP_MEDIAN_EN:
		xisp->bypass_median = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_EN:
		xisp->bypass_isp = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	case V4L2_CID_XILINX_ISP_RESIZE_EN:
		xisp->bypass_resize = ctrl->val;
		xisp_set_info_reg_4(xisp, xisp->bypass_isp, xisp->bypass_hdr, xisp->bypass_rgbir,
				    xisp->bypass_aec, xisp->bypass_blc, xisp->bypass_bpc,
				    xisp->bypass_degamma, xisp->bypass_lsc, xisp->bypass_gain,
				    xisp->bypass_demosaic, xisp->bypass_awb, xisp->bypass_ccm,
				    xisp->bypass_tm, xisp->bypass_gamma, xisp->bypass_lut3d,
				    xisp->bypass_csc, xisp->bypass_bayer_stats,
				    xisp->bypass_luma_stats, xisp->bypass_rgb_stats,
				    xisp->bypass_clahe, xisp->bypass_median, xisp->bypass_resize);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops xisp_ctrl_ops = {
	.s_ctrl = xisp_s_ctrl,
};

static struct v4l2_ctrl_config xisp_ctrls_aec[] = {
	/* AEC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_AEC_EN,
		.name = "bypass_aec",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* AEC THRESHOLD */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_THRESHOLD,
		.name = "aec_threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 20,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_awb[] = {
	/* AWB ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_AWB_EN,
		.name = "bypass_awb",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* AWB THRESHOLD */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_XILINX_ISP_AWB,
		.name = "awb_threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 512,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_blc[] = {
	/* BLC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BLC_EN,
		.name = "bypass_blc",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* MULTIPLICATION FACTOR */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_XILINX_ISP_MULTI_FACTOR,
		.name = "blc_multiplication_factor",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 131070,
		.step = 1,
		.def = XISP_MAX_VALUE,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* BLACK LEVEL */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_BLACK_LEVEL,
		.name = "blc_black_level",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 32,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_bpc[] = {
	/* BPC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BPC_EN,
		.name = "bypass_bpc",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_rgbir[] = {
	/* RGBIR ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RGBIR_EN,
		.name = "bypass_rgbir",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_lsc[] = {
	/* LSC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_LSC_EN,
		.name = "bypass_lsc",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_demosaic[] = {
	/* DEMOSAIC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_DEMOSAIC_EN,
		.name = "bypass_demosaic",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_ccm[] = {
	/* CCM ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_CCM_EN,
		.name = "bypass_ccm",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_degamma[] = {
	/* DEGAMMA ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_DEGAMMA_EN,
		.name = "bypass_degamma",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* SELECT DEGAMMA PARAMS */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_DEGAMMA_PARAMS,
		.name = "select_degamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_gain_control[] = {
	/* GAIN CONTROL ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GAIN_EN,
		.name = "bypass_gain_control",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Red Gain*/
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RED_GAIN,
		.name = "red_gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 100,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Blue Gain */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BLUE_GAIN,
		.name = "blue_gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 350,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Green Gain */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GREEN_GAIN,
		.name = "green_gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 200,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	}
};

static struct v4l2_ctrl_config xisp_ctrls_gamma_correct[] = {
	/* GAMMA ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GAMMA_EN,
		.name = "bypass_gamma",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Red Gamma */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RED_GAMMA,
		.name = "red_gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 40,
		.step = 1,
		.def = 20,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Green Gamma */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GREEN_GAMMA,
		.name = "green_gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 40,
		.step = 1,
		.def = 15,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Blue Gamma */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BLUE_GAMMA,
		.name = "blue_gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 40,
		.step = 1,
		.def = 20,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_hdr_merge[] = {
	/* HDR ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_HDR_EN,
		.name = "bypass_hdr",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Alpha */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_ALPHA,
		.name = "hdr_alpha",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 256,
		.step = 1,
		.def = 5,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Optical Black Value */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_OPTICAL_BLACK_VALUE,
		.name = "hdr_optical_black_value",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 256,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Intersec */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_INTERSEC,
		.name = "hdr_intersec",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 4000000,
		.step = 1,
		.def = 1386290,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Rho */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RHO,
		.name = "hdr_rho",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 65536,
		.step = 1,
		.def = 512,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_hdr_decom[] = {
	/* HDR DECOMPAND ARRAY SELECT */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_DECOMPAND_PARAMS,
		.name = "select_decompand",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 2,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_gtm[] = {
	/* TM ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_TM_EN,
		.name = "bypass_tm",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Parameter C1 */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GTM_C1,
		.name = "gtm_c1",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 128,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Parameter C2 */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_GTM_C2,
		.name = "gtm_c2",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 128,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_clahe[] = {
	/* CLAHE ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_CLAHE_EN,
		.name = "bypass_clahe",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* CLIP */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_XILINX_ISP_CLIP,
		.name = "clahe_clip",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 3,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* TILESY */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_XILINX_ISP_TILESY,
		.name = "clahe_tiles_y",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 4,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* TILESX */
	{
		.ops = &xisp_ctrl_ops,
		.id =  V4L2_CID_XILINX_ISP_TILESX,
		.name = "clahe_tiles_x",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 4,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_ltm[] = {
	/* Block Rows */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BLOCK_ROWS,
		.name = "ltm_block_height",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 8,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Block Cols */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BLOCK_COLS,
		.name = "ltm_block_width",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 8,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_lut3d[] = {
	/* 3DLUT ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_3DLUT_EN,
		.name = "bypass_3dlut",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_3DLUT_DIM,
		.name = "3dlut_dim",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = 4294967296,
		.step = 1,
		.def = 107811,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_csc[] = {
	/* CSC ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_CSC_EN,
		.name = "bypass_csc",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_bayer_stats[] = {
	/* BAYER STATS ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_BAYER_STATS_EN,
		.name = "bypass_bayer_stats",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_luma_stats[] = {
	/* LUMA STATS ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_LUMA_STATS_EN,
		.name = "bypass_luma_stats",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_rgb_stats[] = {
	/* RGB STATS ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RGB_STATS_EN,
		.name = "bypass_rgb_stats",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_median[] = {
	/* MEDIAN BLUR ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_MEDIAN_EN,
		.name = "bypass_median",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_resize[] = {
	/* RESIZE ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_RESIZE_EN,
		.name = "bypass_resize",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	}
};

static struct v4l2_ctrl_config xisp_ctrls_isp[] = {
	/* ISP ENABLE/DISABLE */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_EN,
		.name = "bypass_isp",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = XISP_MIN_VALUE,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_ctrl_config xisp_ctrls_mono[] = {
	/* LUMINANCE Gain*/
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_XILINX_ISP_LUMA_GAIN,
		.name = "mono_luma_gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = XISP_MIN_VALUE,
		.max = XISP_MAX_VALUE,
		.step = 1,
		.def = 128,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
	/* Gamma */
	{
		.ops = &xisp_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "mono_gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 40,
		.step = 1,
		.def = 20,
		.flags = V4L2_CTRL_FLAG_SLIDER,
	},
};

static struct v4l2_mbus_framefmt
*__xisp_get_pad_format(struct xisp_dev *xisp,
			struct v4l2_subdev_state *sd_state,
			unsigned int pad, u32 which)
{
	struct v4l2_mbus_framefmt *get_fmt;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		get_fmt = v4l2_subdev_get_try_format(&xisp->xvip.subdev,
						     sd_state, pad);
		break;
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		get_fmt = &xisp->formats[pad];
		break;
	default:
		get_fmt = NULL;
		break;
	}

	return get_fmt;
}

/*
 * xisp_reset - Reset ISP pipeline IP
 */
static void xisp_reset(struct xisp_dev *xisp)
{
	/* reset ip */
	gpiod_set_value_cansleep(xisp->rst_gpio, XISP_RESET_ASSERT);
	udelay(1);
	gpiod_set_value_cansleep(xisp->rst_gpio, XISP_RESET_DEASSERT);
}

static int xisp_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct xisp_dev *xisp = to_xisp(subdev);
	static const s8 R_IR_C1_wgts[25] = {6, 6, 6, 6, 6, 6, 6, 6, 0, 6, 6, 6,
	6, 6, 6, 6, 0, 6, 6, 6, 6, 6, 6, 6, 6};
	static const s8 R_IR_C2_wgts[25] = {6, 6, 6, 6, 6, 6, 0, 6, 6, 6, 6, 6,
	6, 6, 6, 6, 6, 6, 0, 6, 6, 6, 6, 6, 6};
	static const s8 B_at_R_wgts[25] = {6, 6, 0, 6, 6, 6, 6, 6, 6, 6, 0, 6, 6, 6,
	0, 6, 6, 6, 6, 6, 6, 6, 0, 6, 6};
	static const s8 IR_at_R_wgts[9] = {2, 6, 2, 6, 6, 6, 2, 6, 2};
	static const s8 IR_at_B_wgts[9] = {2, 6, 2, 6, 6, 6, 2, 6, 2};
	static const s8 sub_wgts[4] = {3, 1, 2, 5};

	if (!enable) {
		dev_dbg(xisp->xvip.dev, "%s : Off", __func__);
		xisp_reset(xisp);
		return 0;
	}

	xisp->width = xisp->formats[XVIP_PAD_SINK].width;
	xisp->height = xisp->formats[XVIP_PAD_SINK].height;
	xvip_write(&xisp->xvip, XISP_COMMON_CONFIG, (xisp->height << 16) | xisp->width);

	if (xisp->rgbir_en)
		if (!xisp->rgbir_bypass_en || !(xisp->rgbir_bypass_en & xisp->bypass_rgbir)) {
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE, R_IR_C1_wgts, 25);
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE1, R_IR_C2_wgts, 25);
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE2, B_at_R_wgts, 25);
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE3, IR_at_R_wgts, 9);
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE4, IR_at_B_wgts, 9);
			xisp_set_rgbir_entries(xisp, XISP_RGBIR_CONFIG_BASE5, sub_wgts, 4);
		}

	if (xisp->lut3d_en)
		if (!xisp->lut3d_bypass_en || !(xisp->lut3d_bypass_en & xisp->bypass_lut3d))
			xisp_set_lut3d_entries(xisp, XISP_LUT3D_CONFIG_BASE, xisp->lut3d);

	/* Start ISP pipeline IP */
	xvip_write(&xisp->xvip, XISP_AP_CTRL_REG, XISP_STREAM_ON);

	return 0;
}

static const struct v4l2_subdev_video_ops xisp_video_ops = {
	.s_stream = xisp_s_stream,
};

static int xisp_get_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct xisp_dev *xisp = to_xisp(subdev);
	struct v4l2_mbus_framefmt *get_fmt;

	get_fmt = __xisp_get_pad_format(xisp, sd_state, fmt->pad, fmt->which);
	if (!get_fmt)
		return -EINVAL;

	fmt->format = *get_fmt;

	return 0;
}

static bool
xisp_get_bayer_format(struct xisp_dev *xisp, u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		break;
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_Y12_1X12:
		break;
	default:
		dev_dbg(xisp->xvip.dev, "Unsupported format for Sink Pad");
		return false;
	}
	return true;
}

static void xisp_set_bayer_format(struct xisp_dev *xisp, u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		xisp->bayer_fmt = XISP_RGGB;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
		xisp->bayer_fmt = XISP_GRBG;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
		xisp->bayer_fmt = XISP_GBRG;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR16_1X16:
		xisp->bayer_fmt = XISP_BGGR;
		break;
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_Y12_1X12:
		break;
	default:
		xisp->bayer_fmt = XISP_RGGB;
	}
	xvip_write(&xisp->xvip, XISP_GAIN_CONTROL_CONFIG_2,
		   (xisp->bayer_fmt << 16) | xisp->ggain);
}

static int xisp_set_format(struct v4l2_subdev *subdev,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct xisp_dev *xisp = to_xisp(subdev);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_mbus_framefmt *__propagate1;

	__format = __xisp_get_pad_format(xisp, sd_state, fmt->pad, fmt->which);
	if (!__format)
		return -EINVAL;

	*__format = fmt->format;

	__format->width = clamp_t(unsigned int, fmt->format.width,
				  XISP_MIN_WIDTH, XISP_MAX_WIDTH);
	__format->height = clamp_t(unsigned int, fmt->format.height,
				   XISP_MIN_HEIGHT, XISP_MAX_HEIGHT);
	/* Updating current height and width to device */

	if (fmt->pad == XVIP_PAD_SOURCE) {
		if (__format->code != MEDIA_BUS_FMT_Y8_1X8 &&
		    __format->code != MEDIA_BUS_FMT_Y10_1X10 &&
		    __format->code != MEDIA_BUS_FMT_Y12_1X12 &&
		    __format->code != MEDIA_BUS_FMT_RBG888_1X24 &&
		    __format->code != MEDIA_BUS_FMT_RBG101010_1X30 &&
		    __format->code != MEDIA_BUS_FMT_RBG121212_1X36 &&
		    __format->code != MEDIA_BUS_FMT_RBG161616_1X48 &&
			__format->code != MEDIA_BUS_FMT_BGR888_1X24 &&
			__format->code != MEDIA_BUS_FMT_GBR888_1X24 &&
			__format->code != MEDIA_BUS_FMT_RGB888_1X24 &&
			__format->code != MEDIA_BUS_FMT_RGB101010_1X30) {
			dev_dbg(xisp->xvip.dev,
				"%s : Unsupported source media bus code format",
				__func__);
			__format->code = MEDIA_BUS_FMT_RBG888_1X24;
		}
		xisp->resize_new_width  = __format->width;
		xisp->resize_new_height = __format->height;
		xvip_write(&xisp->xvip, XISP_RESIZE_CONFIG,
			   (xisp->resize_new_height << 16) |
			   xisp->resize_new_width);
	}

	if (fmt->pad == XVIP_PAD_SINK) {
		if (!xisp_get_bayer_format(xisp, __format->code)) {
			dev_dbg(xisp->xvip.dev,
				"Unsupported Sink Pad Media format, defaulting to RGGB");
			__format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
		}
		xisp_set_bayer_format(xisp, __format->code);
	}

	fmt->format = *__format;
	return 0;
}

static const struct v4l2_subdev_pad_ops xisp_pad_ops = {
	.get_fmt = xisp_get_format,
	.set_fmt = xisp_set_format,
};

static const struct v4l2_subdev_ops xisp_ops = {
	.video = &xisp_video_ops,
	.pad = &xisp_pad_ops,
};

static const struct media_entity_operations xisp_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int xisp_parse_of(struct xisp_dev *xisp)
{
	struct device *dev = xisp->xvip.dev;
	struct device_node *node = dev->of_node;
	struct device_node *ports;
	struct device_node *port;

	ports = of_get_child_by_name(node, "ports");
	if (!ports)
		ports = node;

	/* Get the format description for each pad */
	for_each_child_of_node(ports, port) {
		struct device_node *endpoint;

		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		endpoint = of_get_next_child(port, NULL);
		if (!endpoint) {
			dev_err(dev, "No port at\n");
			return -EINVAL;
		}

		/* Count the number of ports. */
		xisp->npads++;
	}

	/* validate number of ports */
	if (xisp->npads > XISP_NO_OF_PADS) {
		dev_err(dev, "invalid number of ports %u\n", xisp->npads);
		return -EINVAL;
	}

	xisp->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(xisp->rst_gpio)) {
		if (PTR_ERR(xisp->rst_gpio) != -EPROBE_DEFER)
			dev_err(dev, "Reset GPIO not setup in DT");
		return PTR_ERR(xisp->rst_gpio);
	}

	return 0;
}

static int xisp_probe(struct platform_device *pdev)
{
	struct xisp_dev *xisp;
	struct v4l2_subdev *subdev;
	int rval, itr;
	u8 NUM_OF_PARAMETERS = ARRAY_SIZE(xisp_ctrls_aec) + ARRAY_SIZE(xisp_ctrls_rgbir) +
	ARRAY_SIZE(xisp_ctrls_awb) + ARRAY_SIZE(xisp_ctrls_blc) + ARRAY_SIZE(xisp_ctrls_bpc) +
	ARRAY_SIZE(xisp_ctrls_degamma) + ARRAY_SIZE(xisp_ctrls_lsc) +
	ARRAY_SIZE(xisp_ctrls_gain_control) + ARRAY_SIZE(xisp_ctrls_demosaic) +
	ARRAY_SIZE(xisp_ctrls_ccm) + ARRAY_SIZE(xisp_ctrls_gamma_correct) +
	ARRAY_SIZE(xisp_ctrls_gtm) + ARRAY_SIZE(xisp_ctrls_ltm) +
	ARRAY_SIZE(xisp_ctrls_hdr_merge) + ARRAY_SIZE(xisp_ctrls_lut3d) +
	ARRAY_SIZE(xisp_ctrls_csc) + ARRAY_SIZE(xisp_ctrls_bayer_stats) +
	ARRAY_SIZE(xisp_ctrls_luma_stats) + ARRAY_SIZE(xisp_ctrls_rgb_stats) +
	ARRAY_SIZE(xisp_ctrls_clahe) + ARRAY_SIZE(xisp_ctrls_median) +
	ARRAY_SIZE(xisp_ctrls_resize) + ARRAY_SIZE(xisp_ctrls_isp) +
	ARRAY_SIZE(xisp_ctrls_hdr_decom) + ARRAY_SIZE(xisp_ctrls_mono);

	xisp = devm_kzalloc(&pdev->dev, sizeof(*xisp), GFP_KERNEL);
	if (!xisp)
		return -ENOMEM;

	xisp->xvip.dev = &pdev->dev;

	rval = xisp_parse_of(xisp);
	if (rval < 0)
		return rval;

	rval = xvip_init_resources(&xisp->xvip);
	if (rval)
		return -EIO;

	/* Reset ISP pipeline IP */
	xisp_reset(xisp);

	/* Init V4L2 subdev */
	subdev = &xisp->xvip.subdev;
	v4l2_subdev_init(subdev, &xisp_ops);
	subdev->dev = &pdev->dev;
	strscpy(subdev->name, dev_name(&pdev->dev), sizeof(subdev->name));
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	xisp->gamma_table = xgamma_curves;
	xisp->lut3d = xisp_lut3d_table;
	xisp->xisp_info_reg_0_data = xvip_read(&xisp->xvip, XISP_PIPELINE_CONFIG_INFO);
	xisp->xisp_info_reg_1_data = xvip_read(&xisp->xvip, XISP_MAX_SUPPORTED_SIZE);
	xisp->xisp_info_reg_2_data = xvip_read(&xisp->xvip, XISP_FUNCS_AVAILABLE);
	xisp->xisp_info_reg_3_data = xvip_read(&xisp->xvip, XISP_FUNCS_BYPASSABLE);

	xisp->in_type = extract_register_bits(xisp->xisp_info_reg_0_data, 1, 0);
	xisp->in_bw_mode = extract_register_bits(xisp->xisp_info_reg_0_data, 3, 1);
	xisp->out_type = extract_register_bits(xisp->xisp_info_reg_0_data, 1, 4);
	xisp->out_bw_mode = extract_register_bits(xisp->xisp_info_reg_0_data, 3, 5);
	xisp->nppc = extract_register_bits(xisp->xisp_info_reg_0_data, 4, 8);
	xisp->num_streams = extract_register_bits(xisp->xisp_info_reg_0_data, 5, 12);

	xisp->max_width = extract_register_bits(xisp->xisp_info_reg_1_data, 16, 0);
	xisp->max_height = extract_register_bits(xisp->xisp_info_reg_1_data, 16, 16);

	xisp->hdr_mode = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 0);
	xisp->hdr_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 1);
	xisp->rgbir_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 2);
	xisp->aec_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 3);
	xisp->blc_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 4);
	xisp->bpc_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 5);
	xisp->degamma_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 6);
	xisp->lsc_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 7);
	xisp->gain_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 8);
	xisp->demosaic_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 9);
	xisp->awb_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 10);
	xisp->ccm_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 11);
	xisp->tm_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 12);
	xisp->tm_type = extract_register_bits(xisp->xisp_info_reg_2_data, 2, 13);
	xisp->gamma_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 15);
	xisp->lut3d_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 16);
	xisp->csc_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 17);
	xisp->bayer_stats_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 18);
	xisp->luma_stats_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 19);
	xisp->rgb_stats_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 20);
	xisp->clahe_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 21);
	xisp->median_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 22);
	xisp->resize_en = extract_register_bits(xisp->xisp_info_reg_2_data, 1, 23);

	xisp->isp_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 0);
	xisp->hdr_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 1);
	xisp->rgbir_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 2);
	xisp->aec_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 3);
	xisp->blc_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 4);
	xisp->bpc_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 5);
	xisp->degamma_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 6);
	xisp->gain_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 8);
	xisp->demosaic_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 9);
	xisp->awb_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 10);
	xisp->ccm_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 11);
	xisp->tm_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 12);
	xisp->gamma_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 15);
	xisp->lut3d_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 16);
	xisp->csc_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 17);
	xisp->bayer_stats_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 18);
	xisp->luma_stats_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 19);
	xisp->rgb_stats_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 20);
	xisp->clahe_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 21);
	xisp->median_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 22);
	xisp->resize_bypass_en = extract_register_bits(xisp->xisp_info_reg_3_data, 1, 23);

	/*
	 * Sink Pad can be any Bayer format.
	 * Default Sink Pad format is RGGB.
	 */
	xisp->formats[XVIP_PAD_SINK].field = V4L2_FIELD_NONE;
	xisp->formats[XVIP_PAD_SINK].colorspace = V4L2_COLORSPACE_SRGB;
	xisp->formats[XVIP_PAD_SINK].width = XISP_MIN_WIDTH;
	xisp->formats[XVIP_PAD_SINK].height = XISP_MIN_HEIGHT;
	xisp->formats[XVIP_PAD_SINK].code = MEDIA_BUS_FMT_SRGGB10_1X10;

	/* Source Pad has a fixed media bus format of RGB */
	xisp->formats[XVIP_PAD_SOURCE].field = V4L2_FIELD_NONE;
	xisp->formats[XVIP_PAD_SOURCE].colorspace = V4L2_COLORSPACE_SRGB;
	xisp->formats[XVIP_PAD_SOURCE].width = XISP_MIN_WIDTH;
	xisp->formats[XVIP_PAD_SOURCE].height = XISP_MIN_HEIGHT;
	xisp->formats[XVIP_PAD_SOURCE].code = MEDIA_BUS_FMT_RBG888_1X24;

	xisp->pads[XVIP_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	xisp->pads[XVIP_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	/* Init Media Entity */
	subdev->entity.ops = &xisp_media_ops;
	rval = media_entity_pads_init(&subdev->entity, XISP_NO_OF_PADS, xisp->pads);
	if (rval < 0)
		goto media_error;

	v4l2_ctrl_handler_init(&xisp->ctrl_handler, NUM_OF_PARAMETERS);

	if (xisp->aec_en) {
		if (xisp->aec_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_aec[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_aec); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_aec[itr], NULL);
		}
	}

	if (xisp->rgbir_en && xisp->rgbir_bypass_en)
		for (itr = 0; itr < ARRAY_SIZE(xisp_ctrls_rgbir); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_rgbir[itr], NULL);
		}

	if (xisp->blc_en) {
		if (xisp->blc_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_blc[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_blc); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_blc[itr], NULL);
		}
	}

	if (xisp->awb_en) {
		if (xisp->awb_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_awb[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_awb); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_awb[itr], NULL);
		}
	}

	if (xisp->bpc_en) {
		if (xisp->bpc_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_bpc[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_bpc); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_bpc[itr], NULL);
		}
	}

	if (xisp->degamma_en) {
		if (xisp->degamma_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_degamma[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_degamma); itr++)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_degamma[itr], NULL);
		}

	if (xisp->lsc_en) {
		if (xisp->lsc_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_lsc[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_lsc); itr++)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_lsc[itr], NULL);
		}

	if (xisp->gain_en && xisp->in_type) {
		if (xisp->gain_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gain_control[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_gain_control); itr++)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gain_control[itr], NULL);
		}

	if (xisp->demosaic_en) {
		if (xisp->demosaic_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_demosaic[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_demosaic); itr++)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_demosaic[itr], NULL);
		}

	if (xisp->ccm_en) {
		if (xisp->ccm_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_ccm[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_ccm); itr++)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_ccm[itr], NULL);
		}

	if (xisp->tm_en) {
		if (xisp->tm_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_gtm[0], NULL);
		if (xisp->tm_type == 0) {
			for (itr = 0; itr < ARRAY_SIZE(xisp_ctrls_ltm); itr++) {
				v4l2_ctrl_new_custom(&xisp->ctrl_handler,
						     &xisp_ctrls_ltm[itr], NULL);
			}
		}

		if (xisp->tm_type == 1) {
			for (itr = 1; itr < (ARRAY_SIZE(xisp_ctrls_gtm) - 1); itr++) {
				v4l2_ctrl_new_custom(&xisp->ctrl_handler,
						     &xisp_ctrls_gtm[itr], NULL);
			}
		}
	}

	if (xisp->hdr_en) {
		if (xisp->hdr_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_hdr_merge[0], NULL);
		if (!xisp->hdr_mode) {
			for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_hdr_merge) - 1; itr++) {
				v4l2_ctrl_new_custom(&xisp->ctrl_handler,
						     &xisp_ctrls_hdr_merge[itr], NULL);
			}
		}

		if (xisp->hdr_mode) {
			for (itr = 0; itr < ARRAY_SIZE(xisp_ctrls_hdr_decom); itr++) {
				v4l2_ctrl_new_custom(&xisp->ctrl_handler,
						     &xisp_ctrls_hdr_decom[itr], NULL);
			}
		}
	}

	if (xisp->lut3d_en) {
		if (xisp->lut3d_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_lut3d[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_lut3d); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_lut3d[itr], NULL);
		}
	}

	if (xisp->csc_en) {
		if (xisp->csc_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_csc[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_csc); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_csc[itr], NULL);
		}
	}

	if (xisp->bayer_stats_en) {
		if (xisp->bayer_stats_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_bayer_stats[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_bayer_stats); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_bayer_stats[itr], NULL);
		}
	}

	if (xisp->luma_stats_en) {
		if (xisp->luma_stats_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_luma_stats[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_luma_stats); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_luma_stats[itr], NULL);
		}
	}

	if (xisp->rgb_stats_en) {
		if (xisp->rgb_stats_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_rgb_stats[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_rgb_stats); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_rgb_stats[itr], NULL);
		}
	}

	if (xisp->clahe_en) {
		if (xisp->clahe_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_clahe[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_clahe); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_clahe[itr], NULL);
		}
	}

	if (xisp->median_en) {
		if (xisp->median_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_median[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_median); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_median[itr], NULL);
		}
	}

	if (xisp->resize_en) {
		if (xisp->resize_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_resize[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_resize); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_resize[itr], NULL);
		}
	}

	if (xisp->gain_en && !xisp->in_type) {
		if (xisp->gain_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gain_control[0], NULL);
		v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_mono[0], NULL);
	}

	if (xisp->gamma_en && xisp->in_type) {
		if (xisp->gamma_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gamma_correct[0], NULL);
		for (itr = 1; itr < ARRAY_SIZE(xisp_ctrls_gamma_correct); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gamma_correct[itr], NULL);
		}
	}

	if (xisp->gamma_en && !xisp->in_type) {
		if (xisp->gamma_bypass_en)
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_gamma_correct[0], NULL);
		v4l2_ctrl_new_custom(&xisp->ctrl_handler, &xisp_ctrls_mono[1], NULL);
	}

	if (xisp->isp_bypass_en)
		for (itr = 0; itr < ARRAY_SIZE(xisp_ctrls_isp); itr++) {
			v4l2_ctrl_new_custom(&xisp->ctrl_handler,
					     &xisp_ctrls_isp[itr], NULL);
		}

	if (xisp->ctrl_handler.error) {
		dev_err(&pdev->dev, "Failed to add V4L2 controls");
		rval = xisp->ctrl_handler.error;
		goto ctrl_error;
	}

	subdev->ctrl_handler = &xisp->ctrl_handler;
	rval = v4l2_ctrl_handler_setup(&xisp->ctrl_handler);
	if (rval < 0) {
		dev_err(&pdev->dev, "Failed to setup control handler");
		goto  ctrl_error;
	}

	platform_set_drvdata(pdev, xisp);
	rval = v4l2_async_register_subdev(subdev);
	if (rval < 0) {
		dev_err(&pdev->dev, "failed to register subdev");
		goto ctrl_error;
	}

	dev_dbg(&pdev->dev, "Xilinx Video ISP Pipeline Probe Successful");
	return 0;

ctrl_error:
	v4l2_ctrl_handler_free(&xisp->ctrl_handler);
media_error:
	xvip_cleanup_resources(&xisp->xvip);

	return rval;
}

static int xisp_remove(struct platform_device *pdev)
{
	struct xisp_dev *xisp = platform_get_drvdata(pdev);
	struct v4l2_subdev *subdev = &xisp->xvip.subdev;

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);
	xvip_cleanup_resources(&xisp->xvip);

	return 0;
}

static const struct of_device_id xisp_of_id_table[] = {
	{.compatible = "xlnx,isppipeline-1.0"},
	{ }
};

MODULE_DEVICE_TABLE(of, xisp_of_id_table);

static struct platform_driver xisp_driver = {
	.driver = {
		.name = "xilinx-isppipeline",
		.of_match_table = xisp_of_id_table,
	},
	.probe = xisp_probe,
	.remove = xisp_remove,

};

module_platform_driver(xisp_driver);
MODULE_DESCRIPTION("Xilinx Video ISP Pipeline IP Driver");
MODULE_LICENSE("GPL");
