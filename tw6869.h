/*
 * Copyright 2015 www.starterkit.ru <info@starterkit.ru>
 *
 * Based on:
 * Driver for Intersil|Techwell TW6869 based DVR cards
 * (c) 2011-12 liran <jli11@intersil.com> [Intersil|Techwell China]
 *
 * V4L2 PCI Skeleton Driver
 * Copyright 2014 Cisco Systems, Inc. and/or its affiliates.
 * All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TW6869_H
#define __TW6869_H

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#define FSL_G_CHIP_IDENT
#define FSL_QUERYBUF

#define TW_DEFAULT_V4L2_STD        V4L2_STD_NTSC /* V4L2_STD_PAL */
#define TW_DMA_ERR_MAX             30
#define TW_APAGE_MAX               16
#define TW_VBUF_ALLOC              6

#define TW_DEBUG                   1

#define tw_dbg(twdev, fmt, arg...) \
	do { if (TW_DEBUG) dev_info(&twdev->pdev->dev, \
		"%s: " fmt, __func__, ## arg); } while (0)

#define tw_info(twdev, fmt, arg...) \
	dev_info(&twdev->pdev->dev, "%s: " fmt, __func__, ## arg)

#define tw_err(twdev, fmt, arg...) \
	dev_err(&twdev->pdev->dev, "%s: " fmt, __func__, ## arg)

#define PCI_VENDOR_ID_TECHWELL     0x1797
#define PCI_DEVICE_ID_6869         0x6869
#define PCI_DEVICE_ID_6865         0x6865

#define TW_MMIO_BAR                0
#define TW_PAGE_SIZE               4096

#define TW_STD_NTSC                0
#define TW_STD_PAL                 1
#define TW_STD_SECAM               2
#define TW_STD_NTSC_443            3
#define TW_STD_PAL_M               4
#define TW_STD_PAL_Nc              5
#define TW_STD_PAL_60              6

#define TW_FMT_UYVY                0
#define TW_FMT_RGB565              5
#define TW_FMT_YUYV                6

#define TW_VDMA_SG_MODE            0
#define TW_VDMA_FRAME_MODE         2
#define TW_VDMA_FIELD_MODE         3

#define TW_ID_MAX                  16
#define TW_CH_MAX                  8
#define TW_VIN_MAX                 4
#define TW_BUF_MAX                 4

#define TW_VID                     0x00FF
#define TW_AID                     0xFF00

#define RADDR(idx)                 ((idx) * 0x4)
#define ID2CH(id)                  ((id) & 0x7)
#define ID2SC(id)                  ((id) & 0x3)
#define GROUP(id)                  (((id) & 0x4) * 0x40)
#define ID2DMACH8(id)              ((id) & 0x7)
#define ID2DMACH16(id)             ((id) & 0xf)
#define ID2DMACH17(id)             (((id) & 0x10) | ((id) & 0xf))
#define ID2YUVGEN(id)              ((id) & 0x7)

/**
 * Register definitions
 */
#define R32_INT_STATUS             RADDR(0x00)
#define R32_PB_STATUS              RADDR(0x01)
#define R32_DMA_CMD                RADDR(0x02)
#define R32_FIFO_STATUS            RADDR(0x03)
#define R32_VIDEO_CHANNEL_ID       RADDR(0x04)
#define R32_VIDEO_PARSER_STATUS    RADDR(0x05)
#define R32_SYS_SOFT_RST           RADDR(0x06)
#define R32_DMA_CHANNEL_ENABLE     RADDR(0x0A)
#define R32_DMA_CONFIG             RADDR(0x0B)
#define R32_DMA_TIMER_INTERVAL     RADDR(0x0C)
#define R32_DMA_CHANNEL_TIMEOUT    RADDR(0x0D)
#define R32_DMA_CHANNEL_CONFIG(id) RADDR(0x10 + ID2CH(id))
#define R32_ADMA_P_ADDR(id)        RADDR(0x18 + ID2CH(id) * 0x2)
#define R32_ADMA_B_ADDR(id)        RADDR(0x19 + ID2CH(id) * 0x2)
#define R32_VIDEO_CONTROL1         RADDR(0x2A)
#define R32_VIDEO_CONTROL2         RADDR(0x2B)
#define R32_AUDIO_CONTROL1         RADDR(0x2C)
#define R32_AUDIO_CONTROL2         RADDR(0x2D)
#define R32_PHASE_REF              RADDR(0x2E)
#define R32_INTL_HBAR_CTRL(id)     RADDR(0x30 + ID2CH(id))
#define R32_AUDIO_CONTROL3         RADDR(0x38)
#define R32_VIDEO_FIELD_CTRL(id)   RADDR(0x39 + ID2CH(id))
#define R32_HSCALER_CTRL(id)       RADDR(0x42 + ID2CH(id))
#define R32_VIDEO_SIZE(id)         RADDR(0x4A + ID2CH(id))
#define R32_VIDEO_SIZE_F2(id)      RADDR(0x52 + ID2CH(id))
#define R32_MD_CONF(id)            RADDR(0x60 + ID2CH(id))
#define R32_MD_INIT(id)            RADDR(0x68 + ID2CH(id))
#define R32_MD_MAPO(id)            RADDR(0x70 + ID2CH(id))
#define R32_VDMA_P_ADDR(id)        RADDR(0x80 + ID2CH(id) * 0x8)
#define R32_VDMA_WHP(id)           RADDR(0x81 + ID2CH(id) * 0x8)
#define R32_VDMA_B_ADDR(id)        RADDR(0x82 + ID2CH(id) * 0x8)
#define R32_VDMA_F2_P_ADDR(id)     RADDR(0x84 + ID2CH(id) * 0x8)
#define R32_VDMA_F2_WHP(id)        RADDR(0x85 + ID2CH(id) * 0x8)
#define R32_VDMA_F2_B_ADDR(id)     RADDR(0x86 + ID2CH(id) * 0x8)
#define R32_RG_NR_CTRL(id)         RADDR(0xC8 + ID2CH(id))
#define R32_PIN_CFG_CTRL           RADDR(0xFB)
#define R32_DBGPORT_CTRL           RADDR(0xFC)
#define R32_EP_REG_ADDR            RADDR(0xFE)
#define R32_EP_REG_DATA            RADDR(0xFF)

#define R32_DMA_PAGE_TABLE_ADDR(n,id)   RADDR((ID2CH(id)                     \
                                              ? (0xD0 + (ID2CH(id) - 1) * 2) \
                                              : 0x08)                        \
                                            + ((n) & 0x1))
#define R32_DMA_PAGE_TABLE0_ADDR(id)    R32_DMA_PAGE_TABLE_ADDR(0,(id))
#define R32_DMA_PAGE_TABLE1_ADDR(id)    R32_DMA_PAGE_TABLE_ADDR(1,(id))

/* BIT[31:8] are hardwired to 0 in all registers */
#define R8_VIDEO_STATUS(id)        RADDR(0x100 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_BRIGHTNESS_CTRL(id)     RADDR(0x101 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_CONTRAST_CTRL(id)       RADDR(0x102 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_SHARPNESS_CTRL(id)      RADDR(0x103 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_SAT_U_CTRL(id)          RADDR(0x104 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_SAT_V_CTRL(id)          RADDR(0x105 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_HUE_CTRL(id)            RADDR(0x106 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_CROPPING_CONTROL(id)    RADDR(0x107 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_VERTICAL_DELAY(id)      RADDR(0x108 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_VERTICAL_ACTIVE(id)     RADDR(0x109 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_HORIZONTAL_DELAY(id)    RADDR(0x10A + GROUP(id) + ID2SC(id) * 0x10)
#define R8_HORIZONTAL_ACTIVE(id)   RADDR(0x10B + GROUP(id) + ID2SC(id) * 0x10)
#define R8_STANDARD_SELECTION(id)  RADDR(0x10E + GROUP(id) + ID2SC(id) * 0x10)
#define R8_STANDARD_RECOGNIT(id)   RADDR(0x10F + GROUP(id) + ID2SC(id) * 0x10)
#define R8_VERTICAL_SCALING(id)    RADDR(0x144 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_SCALING_HIGH(id)        RADDR(0x145 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_HORIZONTAL_SCALING(id)  RADDR(0x146 + GROUP(id) + ID2SC(id) * 0x10)

#define R8_F2CROPPING_CONTROL(id)    RADDR(0x147 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2VERTICAL_DELAY(id)      RADDR(0x148 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2VERTICAL_ACTIVE(id)     RADDR(0x149 + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2HORIZONTAL_DELAY(id)    RADDR(0x14A + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2HORIZONTAL_ACTIVE(id)   RADDR(0x14B + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2VERTICAL_SCALING(id)    RADDR(0x14C + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2SCALING_HIGH(id)        RADDR(0x14D + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2HORIZONTAL_SCALING(id)  RADDR(0x14E + GROUP(id) + ID2SC(id) * 0x10)
#define R8_F2CNT(id)                 RADDR(0x14F + GROUP(id) + ID2SC(id) * 0x10)

#define R8_AVSRST(id)              RADDR(0x180 + GROUP(id))
#define R8_VERTICAL_CONTROL1(id)   RADDR(0x18F + GROUP(id))
#define R8_MISC_CONTROL1(id)       RADDR(0x194 + GROUP(id))
#define R8_MISC_CONTROL2(id)       RADDR(0x196 + GROUP(id))
#define R8_ANALOG_PWR_DOWN(id)     RADDR(0x1CE + GROUP(id))
#define R8_AIGAIN_CTRL(id)         RADDR(0x1D0 + GROUP(id) + ID2SC(id))

/*
 * Registers shifts and masks
 */
#define R32_INT_STATUS_INTSTA_DMA_SHIFT(id)             (0 + ID2DMACH16(id))
#define R32_INT_STATUS_INTSTA_DMA_MASK                  GENMASK(1, 0)
#define R32_INT_STATUS_DMA_TOUT_SHIFT                   17
#define R32_INT_STATUS_DMA_TOUT_MASK                    GENMASK(1, 0)
#define R32_INT_STATUS_BAD_FMT_SHIFT(id)                (24 + ID2CH(id))
#define R32_INT_STATUS_BAD_FMT_MASK                     GENMASK(1, 0)

#define R32_PB_STATUS_PBFLAGS_DMA_SHIFT(ch)             (0 + ID2DMACH16(id))
#define R32_PB_STATUS_PBFLAGS_DMA_MASK                  GENMASK(1, 0)
#define R32_PB_STATUS_FFLAGS_DMA_SHIFT(id)              (24 + ID2DMACH8(id))
#define R32_PB_STATUS_FFLAGS_DMA_MASK                   GENMASK(1, 0)

#define R32_DMA_CMD_RESET_DMA_SHIFT(id)                 (0 + ID2DMACH17(id))
#define R32_DMA_CMD_RESET_DMA_MASK                      GENMASK(1, 0)
#define R32_DMA_CMD_BOND_OPT_SHIFT                      29
#define R32_DMA_CMD_BOND_OPT_MASK                       GENMASK(2, 0)
#define R32_DMA_CMD_DMA_ENABLE_SHIFT                    31
#define R32_DMA_CMD_DMA_ENABLE_MASK                     GENMASK(1, 0)

#define R32_FIFO_STATUS_VDLOSS_SHIFT(id)                (0 + ID2CH(id))
#define R32_FIFO_STATUS_VDLOSS_MASK                     GENMASK(1, 0)
#define R32_FIFO_STATUS_BAD_PTR_SHIFT(id)               (16 + ID2DMACH8(id))
#define R32_FIFO_STATUS_BAD_PTR_MASK                    GENMASK(1, 0)
#define R32_FIFO_STATUS_BAD_FMT_SHIFT(id)               (24 + ID2DMACH8(id))
#define R32_FIFO_STATUS_BAD_FMT_MASK                    GENMASK(1, 0)

#define R32_VIDEO_CHANNEL_ID_SHIFT(id)                  (3 * ID2CH(id))
#define R32_VIDEO_CHANNEL_ID_MASK                       GENMASK(3, 0)

#define R32_VIDEO_PARSER_STATUS_P_BAD_SHIFT(id)         (0 + ID2CH(id))
#define R32_VIDEO_PARSER_STATUS_P_BAD_MASK              GENMASK(1, 0)
#define R32_VIDEO_PARSER_STATUS_P_OV_SHIFT(id)          (8 + ID2CH(id))
#define R32_VIDEO_PARSER_STATUS_P_OV_MASK               GENMASK(1, 0)

#define R32_SYS_SOFT_RST_RESET_EXT_PHY_SHIFT            0
#define R32_SYS_SOFT_RST_RESET_EXT_PHY_MASK             GENMASK(1, 0)
#define R32_SYS_SOFT_RST_RESET_DEC_INTF_SHIFT           1
#define R32_SYS_SOFT_RST_RESET_DEC_INTF_MASK            GENMASK(1, 0)
#define R32_SYS_SOFT_RST_RESET_DMA_CTRL_SHIFT           2
#define R32_SYS_SOFT_RST_RESET_DMA_CTRL_MASK            GENMASK(1, 0)
#define R32_SYS_SOFT_RST_RESET_AV_REG_SHIFT             3
#define R32_SYS_SOFT_RST_RESET_AV_REG_MASK              GENMASK(1, 0)

#define R32_DMA_CHANNEL_ENABLE_ENA_DMA_SHIFT(id)        (0 + ID2DMACH17(id))
#define R32_DMA_CHANNEL_ENABLE_ENA_DMA_MASK             GENMASK(1, 0)

#define R32_DMA_CONFIG_BIG_ENDIAN_SHIFT                 0
#define R32_DMA_CONFIG_BIG_ENDIAN_MASK                  GENMASK(1, 0)
#define R32_DMA_CONFIG_ENA_INTX_SHIFT                   2
#define R32_DMA_CONFIG_ENA_INTX_MASK                    GENMASK(1, 0)
#define R32_DMA_CONFIG_ENA_CPL_WAIT_SHIFT               3
#define R32_DMA_CONFIG_ENA_CPL_WAIT_MASK                GENMASK(1, 0)
#define R32_DMA_CONFIG_MASK_OVF_SHIFT(id)               (8 + ID2DMACH8(id))
#define R32_DMA_CONFIG_MASK_OVF_MASK                    GENMASK(1, 0)
#define R32_DMA_CONFIG_MASK_BAD_PTR_SHIFT(id)           (16 + ID2DMACH8(id))
#define R32_DMA_CONFIG_MASK_BAD_PTR_MASK                GENMASK(1, 0)
#define R32_DMA_CONFIG_MASK_BAD_FMT_SHIFT(id)           (24 + ID2DMACH8(id))
#define R32_DMA_CONFIG_MASK_BAD_FMT_MASK                GENMASK(1, 0)

#define R32_DMA_TIMER_INTERVAL_DMA_INT_TIMER_SHIFT      0
#define R32_DMA_TIMER_INTERVAL_DMA_INT_TIMER_MASK       GENMASK(22, 0)

#define R32_DMA_CHANNEL_TIMEOUT_DMA_VDO_CH_TIMEOUT_SHIFT    0
#define R32_DMA_CHANNEL_TIMEOUT_DMA_VDO_CH_TIMEOUT_MASK     GENMASK(12, 0)
#define R32_DMA_CHANNEL_TIMEOUT_DMA_DAT_CH_TIMEOUT_SHIFT    12
#define R32_DMA_CHANNEL_TIMEOUT_DMA_DAT_CH_TIMEOUT_MASK     GENMASK(12, 0)
#define R32_DMA_CHANNEL_TIMEOUT_PRE_TIMEOUT_OFST_SHIFT      24
#define R32_DMA_CHANNEL_TIMEOUT_PRE_TIMEOUT_OFST_MASK       GENMASK(6, 0)

#define R32_DMA_CHANNEL_CONFIG_START_IDX_DMA_SHIFT      0
#define R32_DMA_CHANNEL_CONFIG_START_IDX_DMA_MASK       GENMASK(10, 0)
#define R32_DMA_CHANNEL_CONFIG_END_IDX_DMA_SHIFT        10
#define R32_DMA_CHANNEL_CONFIG_END_IDX_DMA_MASK         GENMASK(10, 0)
#define R32_DMA_CHANNEL_CONFIG_VIDEO_OUT_FORMAT_SHIFT   20
#define R32_DMA_CHANNEL_CONFIG_VIDEO_OUT_FORMAT_MASK    GENMASK(3, 0)
#define R32_DMA_CHANNEL_CONFIG_ENA_HDECI_SHIFT          23
#define R32_DMA_CHANNEL_CONFIG_ENA_HDECI_MASK           GENMASK(1, 0)
#define R32_DMA_CHANNEL_CONFIG_ENA_VDECI_SHIFT          24
#define R32_DMA_CHANNEL_CONFIG_ENA_VDECI_MASK           GENMASK(1, 0)
#define R32_DMA_CHANNEL_CONFIG_MASTER_CHID_SHIFT        25
#define R32_DMA_CHANNEL_CONFIG_MASTER_CHID_MASK         GENMASK(2, 0)
#define R32_DMA_CHANNEL_CONFIG_ENA_MASTER_SHIFT         27
#define R32_DMA_CHANNEL_CONFIG_ENA_MASTER_MASK          GENMASK(1, 0)
#define R32_DMA_CHANNEL_CONFIG_ENA_FIELD_DROP_SHIFT     28
#define R32_DMA_CHANNEL_CONFIG_ENA_FIELD_DROP_MASK      GENMASK(1, 0)
#define R32_DMA_CHANNEL_CONFIG_FIELD_OUT_SHIFT          29
#define R32_DMA_CHANNEL_CONFIG_FIELD_OUT_MASK           GENMASK(1, 0)
#define R32_DMA_CHANNEL_CONFIG_VIN_MUX_SEL_SHIFT        30
#define R32_DMA_CHANNEL_CONFIG_VIN_MUX_SEL_MASK         GENMASK(2, 0)

#define R32_VIDEO_CONTROL1_VSCL_ENA_0_SHIFT             6
#define R32_VIDEO_CONTROL1_VSCL_ENA_0_MASK              GENMASK(1, 0)
#define R32_VIDEO_CONTROL1_HSCL_ENA_0_SHIFT             7
#define R32_VIDEO_CONTROL1_HSCL_ENA_0_MASK              GENMASK(1, 0)
#define R32_VIDEO_CONTROL1_VSCL_ENA_1_SHIFT             10
#define R32_VIDEO_CONTROL1_VSCL_ENA_1_MASK              GENMASK(1, 0)
#define R32_VIDEO_CONTROL1_SYS_MODE_DMA_SHIFT(id)       (13 + ID2DMACH8(id))
#define R32_VIDEO_CONTROL1_SYS_MODE_DMA_MASK            GENMASK(1, 0)

#define R32_VIDEO_CONTROL2_EXT_VDAT_ENA_SHIFT(id)       (ID2DMACH8(id))
#define R32_VIDEO_CONTROL2_EXT_VDAT_ENA_MASK            GENMASK(1, 0)
#define R32_VIDEO_CONTROL2_PAT_SEL_GEN_SHIFT(id)        (8 + ID2YUVGEN(id))
#define R32_VIDEO_CONTROL2_PAT_SEL_GEN_MASK             GENMASK(1, 0)
#define R32_VIDEO_CONTROL2_RST_GEN_SHIFT(id)            (16 + ID2YUVGEN(id))
#define R32_VIDEO_CONTROL2_RST_GEN_MASK                 GENMASK(1, 0)

#define R32_AUDIO_CONTROL1_EXT_ADAT_ENA_SHIFT           0
#define R32_AUDIO_CONTROL1_EXT_ADAT_ENA_MASK            GENMASK(1, 0)
#define R32_AUDIO_CONTROL1_MIX_MODE_SHIFT               1
#define R32_AUDIO_CONTROL1_MIX_MODE_MASK                GENMASK(3, 0)
#define R32_AUDIO_CONTROL1_PAT_SEL_ADO_SHIFT            4
#define R32_AUDIO_CONTROL1_PAT_SEL_ADO_MASK             GENMASK(1, 0)
#define R32_AUDIO_CONTROL1_INT_GEN_ADAT_RATE_SHIFT      5
#define R32_AUDIO_CONTROL1_INT_GEN_ADAT_RATE_MASK       GENMASK(14, 0)
#define R32_AUDIO_CONTROL1_BYTE_LENGTH_DMA8TO16_SHIFT   19
#define R32_AUDIO_CONTROL1_BYTE_LENGTH_DMA8TO16_MASK    GENMASK(13, 0)

#define R32_AUDIO_CONTROL2_AUDIO_CLK_REF_SHIFT      0
#define R32_AUDIO_CONTROL2_AUDIO_CLK_REF_MASK       GENMASK(30, 0)

#define R32_PHASE_REF_PHASE_REF_SHIFT               0
#define R32_PHASE_REF_PHASE_REF_MASK                GENMASK(14, 0)
#define R32_PHASE_REF_DMA_MODE_SHIFT(id)            (16 + ID2DMACH8(id) * 2)
#define R32_PHASE_REF_DMA_MODE_MASK                 GENMASK(2, 0)

#define R32_INTL_HBAR_CTRL_ST_LINE_HBAR_SHIFT       0
#define R32_INTL_HBAR_CTRL_ST_LINE_HBAR_MASK        GENMASK(8, 0)
#define R32_INTL_HBAR_CTRL_HEIGHT_HBAR_SHIFT        8
#define R32_INTL_HBAR_CTRL_HEIGHT_HBAR_MASK         GENMASK(8, 0)

#define R32_AUDIO_CONTROL3_OUT_BITWIDTH_SHIFT       8
#define R32_AUDIO_CONTROL3_OUT_BITWIDTH_MASK        GENMASK(1, 0)

#define R32_VIDEO_FIELD_CTRL_FLD_OUT_OPT_SHIFT      0
#define R32_VIDEO_FIELD_CTRL_FLD_OUT_OPT_MASK       GENMASK(30, 0)
#define R32_VIDEO_FIELD_CTRL_START_FLD_SHIFT        30
#define R32_VIDEO_FIELD_CTRL_START_FLD_MASK         GENMASK(1, 0)
#define R32_VIDEO_FIELD_CTRL_FLD_CTRL_ENA_SHIFT     31
#define R32_VIDEO_FIELD_CTRL_FLD_CTRL_ENA_MASK      GENMASK(1, 0)

#define R32_HSCALER_CTRL_START_POS_SHIFT            0
#define R32_HSCALER_CTRL_START_POS_MASK             GENMASK(5, 0)
#define R32_HSCALER_CTRL_END_POS_SHIFT              5
#define R32_HSCALER_CTRL_END_POS_MASK               GENMASK(10, 0)
#define R32_HSCALER_CTRL_PHASE_REF_SHIFT            15
#define R32_HSCALER_CTRL_PHASE_REF_MASK             GENMASK(16, 0)
#define R32_HSCALER_CTRL_HSCALER_ENA_SHIFT          31
#define R32_HSCALER_CTRL_HSCALER_ENA_MASK           GENMASK(1, 0)

#define R32_VIDEO_SIZE_H_SIZE_SHIFT                 0
#define R32_VIDEO_SIZE_H_SIZE_MASK                  GENMASK(11, 0)
#define R32_VIDEO_SIZE_V_SIZE_SHIFT                 16
#define R32_VIDEO_SIZE_V_SIZE_MASK                  GENMASK(9, 0)
#define R32_VIDEO_SIZE_VS_F2_EN_SHIFT               30
#define R32_VIDEO_SIZE_VS_F2_EN_MASK                GENMASK(1, 0)
#define R32_VIDEO_SIZE_VS_EN_SHIFT                  31
#define R32_VIDEO_SIZE_VS_EN_MASK                   GENMASK(1, 0)

#define R32_VIDEO_SIZE_F2_H_SIZE_F2_SHIFT           0
#define R32_VIDEO_SIZE_F2_H_SIZE_F2_MASK            GENMASK(11, 0)
#define R32_VIDEO_SIZE_F2_V_SIZE_F2_SHIFT           16
#define R32_VIDEO_SIZE_F2_V_SIZE_F2_MASK            GENMASK(9, 0)

#define R32_MD_CONF_MD_TRESHOLD_SHIFT               0
#define R32_MD_CONF_MD_TRESHOLD_MASK                GENMASK(18, 0)
#define R32_MD_CONF_MD_MODE_SHIFT                   18
#define R32_MD_CONF_MD_MODE_MASK                    GENMASK(1, 0)
#define R32_MD_CONF_MD_ACT_FLD_SHIFT                19
#define R32_MD_CONF_MD_ACT_FLD_MASK                 GENMASK(1, 0)
#define R32_MD_CONF_MD_ENABLE_SHIFT                 20
#define R32_MD_CONF_MD_ENABLE_MASK                  GENMASK(1, 0)
#define R32_MD_CONF_MD_TSCALE_SHIFT                 21
#define R32_MD_CONF_MD_TSCALE_MASK                  GENMASK(3, 0)

#define R32_MD_CONF_MASK (                      \
		(R32_MD_CONF_MD_TRESHOLD_MASK           \
			<< R32_MD_CONF_MD_TRESHOLD_SHIFT) | \
		(R32_MD_CONF_MD_MODE_MASK               \
			<< R32_MD_CONF_MD_MODE_SHIFT    ) | \
		(R32_MD_CONF_MD_ACT_FLD_MASK            \
			<< R32_MD_CONF_MD_ACT_FLD_SHIFT ) | \
		(R32_MD_CONF_MD_TSCALE_MASK             \
			<< R32_MD_CONF_MD_TSCALE_SHIFT  ) )

#define R32_MD_INIT_SHIFT                           0
#define R32_MD_INIT_MASK                            GENMASK(1, 0)

#define R32_MD_MAPO_SHIFT                           0
#define R32_MD_MAPO_MASK                            GENMASK(24, 0)

#define R32_VDMA_WHP_ACTIVE_WIDTH_SHIFT             0
#define R32_VDMA_WHP_ACTIVE_WIDTH_MASK              GENMASK(11, 0)
#define R32_VDMA_WHP_LINE_WIDTH_SHIFT               11
#define R32_VDMA_WHP_LINE_WIDTH_MASK                GENMASK(11, 0)
#define R32_VDMA_WHP_HEIGHT_SHIFT                   22
#define R32_VDMA_WHP_HEIGHT_MASK                    GENMASK(10, 0)

#define R32_VDMA_F2_WHP_F2_ACTIVE_WIDTH_SHIFT       0
#define R32_VDMA_F2_WHP_F2_ACTIVE_WIDTH_MASK        GENMASK(11, 0)
#define R32_VDMA_F2_WHP_F2_LINE_WIDTH_SHIFT         11
#define R32_VDMA_F2_WHP_F2_LINE_WIDTH_MASK          GENMASK(11, 0)
#define R32_VDMA_F2_WHP_F2_HEIGHT_SHIFT             22
#define R32_VDMA_F2_WHP_F2_HEIGHT_MASK              GENMASK(10, 0)

#define R32_RG_NR_CTRL_RG_FRONT_CTL_SHIFT           0
#define R32_RG_NR_CTRL_RG_FRONT_CTL_MASK            GENMASK(8, 0)
#define R32_RG_NR_CTRL_RG_FRONT_GEN_CTL_SHIFT       8
#define R32_RG_NR_CTRL_RG_FRONT_GEN_CTL_MASK        GENMASK(8, 0)
#define R32_RG_NR_CTRL_RG_NRDET_CTL_SHIFT           16
#define R32_RG_NR_CTRL_RG_NRDET_CTL_MASK            GENMASK(4, 0)
#define R32_RG_NR_CTRL_RGA_RST_NRDET_SHIFT          20
#define R32_RG_NR_CTRL_RGA_RST_NRDET_MASK           GENMASK(1, 0)
#define R32_RG_NR_CTRL_SKIP_CNT_SHIFT               21
#define R32_RG_NR_CTRL_SKIP_CNT_MASK                GENMASK(2, 0)
#define R32_RG_NR_CTRL_NR2D_ENA_SHIFT               23
#define R32_RG_NR_CTRL_NR2D_ENA_MASK                GENMASK(1, 0)

#define R32_PIN_CFG_CTRL_MODE_SHIFT                 0
#define R32_PIN_CFG_CTRL_MODE_MASK                  GENMASK(2, 0)
#define R32_PIN_CFG_CTRL_VCHDTEST_SHIFT(id)         (8 + ID2CH(id))
#define R32_PIN_CFG_CTRL_VCHDTEST_MASK              GENMASK(1, 0)
#define R32_PIN_CFG_CTRL_ACHDTEST_SHIFT(id)         (16 + ID2CH(id))
#define R32_PIN_CFG_CTRL_ACHDTEST_MASK              GENMASK(1, 0)

#define R32_DBGPORT_CTRL_SUBMODULE_SEL_SHIFT        0
#define R32_DBGPORT_CTRL_SUBMODULE_SEL_MASK         GENMASK(8, 0)
#define R32_DBGPORT_CTRL_BLOCK_SEL_SHIFT            8
#define R32_DBGPORT_CTRL_BLOCK_SEL_MASK             GENMASK(7, 0)
#define R32_DBGPORT_CTRL_MSB16_EN_SHIFT             15
#define R32_DBGPORT_CTRL_MSB16_EN_MASK              GENMASK(1, 0)

#define R32_EP_REG_ADDR_SHIFT                       0
#define R32_EP_REG_ADDR_MASK                        GENMASK(12, 0)

/**
 * struct tw6869_buf - instance of one DMA buffer
 */
struct tw6869_buf {
	struct vb2_buffer vb;
	struct list_head list;
	dma_addr_t dma_addr;
};

/**
 * struct tw6869_dma - instance of one DMA channel
 * @dev: parent device
 * @buf: DMA ping-pong buffers
 * @hw_on: switching DMA ON with delay
 * @lock: spinlock controlling access to channel
 * @reg: DMA ping-pong registers
 * @id: DMA id
 * @fld: field1 | field2 ping-pong state
 * @pb: P-buffer | B-buffer ping-pong state
 * @delay: delay in jiffies before switching DMA ON
 * @is_on: DMA channel status
 * @lost: video (audio) signal lost
 * @low_power: channel in a low-power state
 * @err: DMA errors counter
 * @srst: software reset the video (audio) portion
 * @ctrl: restore control state
 * @cfg: configure the DMA channel
 * @isr: DMA channel ISR
 */
struct tw6869_dma {
	struct tw6869_dev *dev;
	struct tw6869_buf * buf[TW_BUF_MAX];
	struct delayed_work hw_on;
	spinlock_t lock;
	unsigned int reg[TW_BUF_MAX];
	unsigned int id;
	unsigned int fld;
	unsigned int pb;
	unsigned int delay;
	unsigned int is_on;
	unsigned int lost;
	unsigned int low_power;
	unsigned int err;
	void (*srst)(struct tw6869_dma *);
	void (*ctrl)(struct tw6869_dma *);
	void (*cfg)(struct tw6869_dma *);
	void (*isr)(struct tw6869_dma *);
};

/**
 * struct tw6869_vch - instance of one video channel
 * @dma: DMA channel
 * @vdev: V4L2 video device
 * @mlock: ioctl serialization mutex
 * @queue: queue maintained by videobuf2 layer
 * @buf_list: list of buffers queued for DMA
 * @hdl: handler for control framework
 * @format: pixel format
 * @std: video standard (e.g. PAL/NTSC)
 * @input: input line for video signal
 * @sequence: frame sequence counter
 * @dcount: number of dropped frames
 * @fps: current frame rate
 * @brightness: control state
 * @contrast: control state
 * @sharpness: control state
 * @saturation: control state
 * @hue: control state
 * @md_mode: motion detection state (enabled | disabled)
 * @md_threshold: motion detection threshold
 */
struct tw6869_vch {
	struct tw6869_dma dma;
	struct video_device vdev;
	struct mutex mlock;
	struct vb2_queue queue;
	struct list_head buf_list;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format format;
	v4l2_std_id std;
	unsigned int input;
	unsigned int sequence;
	unsigned int dcount;
	unsigned int fps;
	unsigned int brightness;
	unsigned int contrast;
	unsigned int sharpness;
	unsigned int saturation;
	unsigned int hue;
	unsigned int md_mode;
	unsigned int md_threshold;
};

/**
 * struct tw6869_ach - instance of one audio channel
 * @dma: DMA channel
 * @snd_card: soundcard registered in ALSA layer
 * @ss: PCM substream
 * @buffers: split an contiguous buffer into chunks (DMA pages)
 * @buf_list: ring buffer consisting of DMA pages
 * @ptr: PCM buffer pointer
 * @gain: control state
 */
struct tw6869_ach {
	struct tw6869_dma dma;
	struct snd_card *snd_card;
	struct snd_pcm_substream *ss;
	struct tw6869_buf buffers[TW_APAGE_MAX];
	struct list_head buf_list;
	dma_addr_t ptr;
	unsigned int gain;
};

/**
 * struct tw6869_dev - instance of device
 * @pdev: PCI device
 * @mmio: hardware base address
 * @rlock: spinlock controlling access to the shared registers
 * @vch_max: number of the used video channels
 * @ach_max: number of the used audio channels
 * @dma: DMA channels
 * @v4l2_dev: device registered in V4L2 layer
 * @alloc_ctx: context for videobuf2
 * @vch: array of video channel instance
 * @ach: array of audio channel instance
 */
struct tw6869_dev {
	struct pci_dev *pdev;
	unsigned char __iomem *mmio;
	spinlock_t rlock;
	unsigned int vch_max;
	unsigned int ach_max;
	struct tw6869_dma * dma[TW_ID_MAX];
	struct v4l2_device v4l2_dev;
	struct vb2_alloc_ctx *alloc_ctx;
	struct tw6869_vch vch[TW_CH_MAX];
	struct tw6869_ach ach[TW_CH_MAX];
};

static inline void tw_write(struct tw6869_dev *dev,
		unsigned int reg, unsigned int val)
{
	iowrite32(val, dev->mmio + reg);
}

static inline unsigned int tw_read(struct tw6869_dev *dev,
		unsigned int reg)
{
	return ioread32(dev->mmio + reg);
}

static inline void tw_write_mask(struct tw6869_dev *dev,
		unsigned int reg, unsigned int val, unsigned int mask)
{
	unsigned int v = tw_read(dev, reg);

	v = (v & ~mask) | (val & mask);
	tw_write(dev, reg, v);
}

static inline void tw_clear(struct tw6869_dev *dev,
		unsigned int reg, unsigned int val)
{
	tw_write_mask(dev, reg, 0, val);
}

static inline void tw_set(struct tw6869_dev *dev,
		unsigned int reg, unsigned int val)
{
	tw_write_mask(dev, reg, val, val);
}

static inline unsigned int tw_dma_active(struct tw6869_dma *dma)
{
	return dma->is_on && dma->buf[0] && dma->buf[1];
}

static inline void tw_dma_set_addr(struct tw6869_dma *dma)
{
	unsigned int i;

	for (i = 0; i < TW_BUF_MAX; i++) {
		if (dma->reg[i] && dma->buf[i])
			tw_write(dma->dev, dma->reg[i], dma->buf[i]->dma_addr);
	}
}

static inline unsigned int tw_dma_is_on(struct tw6869_dma *dma)
{
	unsigned int e = tw_read(dma->dev, R32_DMA_CHANNEL_ENABLE);
	unsigned int c = tw_read(dma->dev, R32_DMA_CMD);

	return c & e & BIT(dma->id);
}

static inline void tw_dma_enable(struct tw6869_dma *dma)
{
	tw_set(dma->dev, R32_DMA_CHANNEL_ENABLE, BIT(dma->id));
	tw_set(dma->dev, R32_DMA_CMD, BIT(31) | BIT(dma->id));

	dma->fld = 0;
	dma->pb = 0;
	dma->err = 0;
	dma->lost = 0;
	dma->low_power = 0;
	dma->is_on = tw_dma_is_on(dma);
	tw_dbg(dma->dev, "DMA %u\n", dma->id);
}

static inline void tw_dma_disable(struct tw6869_dma *dma)
{
	tw_clear(dma->dev, R32_DMA_CMD, BIT(dma->id));
	tw_clear(dma->dev, R32_DMA_CHANNEL_ENABLE, BIT(dma->id));

	dma->is_on = tw_dma_is_on(dma);
	tw_dbg(dma->dev, "DMA %u\n", dma->id);
}

int tw6869_video_register(struct tw6869_dev *dev);
void tw6869_video_unregister(struct tw6869_dev *dev);
int tw6869_audio_register(struct tw6869_dev *dev);
void tw6869_audio_unregister(struct tw6869_dev *dev);

#endif /* __TW6869_H */
