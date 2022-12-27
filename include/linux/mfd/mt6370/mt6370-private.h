/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MT6370_PRIVATE_H__
#define __MT6370_PRIVATE_H__

#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* core control */
#define MT6370_PMU_REG_DEVINFO		(0x00)
#define MT6370_PMU_REG_CORECTRL1	(0x01)
#define MT6370_PMU_REG_CORECTRL2	(0x02)
#define MT6370_PMU_REG_RSTPASCODE1	(0x03)
#define MT6370_PMU_REG_RSTPASCODE2	(0x04)
#define MT6370_PMU_REG_HIDDENPASCODE1	(0x07)
#define MT6370_PMU_REG_HIDDENPASCODE2	(0x08)
#define MT6370_PMU_REG_HIDDENPASCODE3	(0x09)
#define MT6370_PMU_REG_HIDDENPASCODE4	(0x0A)
#define MT6370_PMU_REG_IRQIND		(0x0B)
#define MT6370_PMU_REG_IRQMASK		(0x0C)
#define MT6370_PMU_REG_IRQSET		(0x0D)
#define MT6370_PMU_REG_SHDNCTRL1	(0x0E)
#define MT6370_PMU_REG_SHDNCTRL2	(0x0F)
#define MT6370_PMU_REG_OSCCTRL		(0x10)
/* charger control */
#define MT6370_PMU_REG_CHGCTRL1		(0x11)
#define MT6370_PMU_REG_CHGCTRL2		(0x12)
#define MT6370_PMU_REG_CHGCTRL3		(0x13)
#define MT6370_PMU_REG_CHGCTRL4		(0x14)
#define MT6370_PMU_REG_CHGCTRL5		(0x15)
#define MT6370_PMU_REG_CHGCTRL6		(0x16)
#define MT6370_PMU_REG_CHGCTRL7		(0x17)
#define MT6370_PMU_REG_CHGCTRL8		(0x18)
#define MT6370_PMU_REG_CHGCTRL9		(0x19)
#define MT6370_PMU_REG_CHGCTRL10	(0x1A)
#define MT6370_PMU_REG_CHGCTRL11	(0x1B)
#define MT6370_PMU_REG_CHGCTRL12	(0x1C)
#define MT6370_PMU_REG_CHGCTRL13	(0x1D)
#define MT6370_PMU_REG_CHGCTRL14	(0x1E)
#define MT6370_PMU_REG_CHGCTRL15	(0x1F)
#define MT6370_PMU_REG_CHGCTRL16	(0x20)
#define MT6370_PMU_REG_CHGADC		(0x21)
#define MT6370_PMU_REG_DEVICETYPE	(0x22)
#define MT6370_PMU_REG_QCCTRL1		(0x23)
#define MT6370_PMU_REG_QCCTRL2		(0x24)
#define MT6370_PMU_REG_QC3P0CTRL1	(0x25)
#define MT6370_PMU_REG_QC3P0CTRL2	(0x26)
#define MT6370_PMU_REG_USBSTATUS1	(0x27)
#define MT6370_PMU_REG_QCSTATUS1	(0x28)
#define MT6370_PMU_REG_QCSTATUS2	(0x29)
#define MT6370_PMU_REG_CHGPUMP		(0x2A)
#define MT6370_PMU_REG_CHGCTRL17	(0x2B)
#define MT6370_PMU_REG_CHGCTRL18	(0x2C)
#define MT6370_PMU_REG_CHGDIRCHG1	(0x2D)
#define MT6370_PMU_REG_CHGDIRCHG2	(0x2E)
#define MT6370_PMU_REG_CHGDIRCHG3	(0x2F)
#define MT6370_PMU_REG_CHGHIDDENCTRL0	(0x30)
#define MT6370_PMU_REG_CHGHIDDENCTRL1	(0x31)
#define MT6370_PMU_REG_LG_CONTROL	(0x33)
#define MT6370_PMU_REG_CHGHIDDENCTRL6	(0x35)
#define MT6370_PMU_REG_CHGHIDDENCTRL7	(0x36)
#define MT6370_PMU_REG_CHGHIDDENCTRL8	(0x37)
#define MT6370_PMU_REG_CHGHIDDENCTRL9	(0x38)
#define MT6370_PMU_REG_CHGHIDDENCTRL15	(0x3E)
#define MT6370_PMU_REG_CHGSTAT		(0x4A)
#define MT6370_PMU_REG_CHGNTC		(0x4B)
#define MT6370_PMU_REG_ADCDATAH		(0x4C)
#define MT6370_PMU_REG_ADCDATAL		(0x4D)
#define MT6370_PMU_REG_ADCDATATUNEH	(0x4E)
#define MT6370_PMU_REG_ADCDATATUNEL	(0x4F)
#define MT6370_PMU_REG_ADCDATAORGH	(0x50)
#define MT6370_PMU_REG_ADCDATAORGL	(0x51)
#define MT6370_PMU_REG_ADCBATDATAH	(0x52)
#define MT6370_PMU_REG_ADCBATDATAL	(0x53)
#define MT6370_PMU_REG_CHGCTRL19	(0x60)
#define MT6370_PMU_REG_OVPCTRL		(0x61)
#define MT6370_PMU_REG_VDDASUPPLY	(0x62)
/* flashled control */
#define MT6370_PMU_REG_FLEDCFG		(0x70)
#define MT6370_PMU_REG_FLED1CTRL	(0x72)
#define MT6370_PMU_REG_FLEDSTRBCTRL	(0x73)
#define MT6370_PMU_REG_FLED1STRBCTRL	(0x74)
#define MT6370_PMU_REG_FLED1TORCTRL	(0x75)
#define MT6370_PMU_REG_FLED2CTRL	(0x76)
#define MT6370_PMU_REG_FLED2STRBCTRL2	(0x78)
#define MT6370_PMU_REG_FLED2TORCTRL	(0x79)
#define MT6370_PMU_REG_FLEDVMIDTRKCTRL1	(0x7A)
#define MT6370_PMU_REG_FLEDVMIDRTM	(0x7B)
#define MT6370_PMU_REG_FLEDVMIDTRKCTRL2	(0x7C)
#define MT6370_PMU_REG_FLEDEN		(0x7E)
/* ldo control */
#define MT6370_PMU_REG_LDOCFG		(0x80)
#define MT6370_PMU_REG_LDOVOUT		(0x81)
/* rgb control */
#define MT6370_PMU_REG_RGB1DIM		(0x82)
#define MT6370_PMU_REG_RGB2DIM		(0x83)
#define MT6370_PMU_REG_RGB3DIM		(0x84)
#define MT6370_PMU_REG_RGBEN		(0x85)
#define MT6370_PMU_REG_RGB1ISINK	(0x86)
#define MT6370_PMU_REG_RGB2ISINK	(0x87)
#define MT6370_PMU_REG_RGB3ISINK	(0x88)
#define MT6370_PMU_REG_RGB1TR		(0x89)
#define MT6370_PMU_REG_RGB1TF		(0x8A)
#define MT6370_PMU_REG_RGB1TONTOFF	(0x8B)
#define MT6370_PMU_REG_RGB2TR		(0x8C)
#define MT6370_PMU_REG_RGB2TF		(0x8D)
#define MT6370_PMU_REG_RGB2TONTOFF	(0x8E)
#define MT6370_PMU_REG_RGB3TR		(0x8F)
#define MT6370_PMU_REG_RGB3TF		(0x90)
#define MT6370_PMU_REG_RGB3TONTOFF	(0x91)
#define MT6370_PMU_REG_RGBCHRINDDIM	(0x92)
#define MT6370_PMU_REG_RGBCHRINDCTRL	(0x93)
#define MT6370_PMU_REG_RGBCHRINDTR	(0x94)
#define MT6370_PMU_REG_RGBCHRINDTF	(0x95)
#define MT6370_PMU_REG_RGBCHRINDTONTOFF	(0x96)
#define MT6370_PMU_REG_RGBOPENSHORTEN	(0x97)
#define MT6370_PMU_REG_RGBTONTOFF	(0x98)
#define MT6370_PMU_REG_RGBHIDDEN1	(0x99)
#define MT6370_PMU_REG_RGBHIDDEN2	(0x9A)
#define MT6370_PMU_REG_RESERVED1	(0x9F)
/* backlight control */
#define MT6370_PMU_REG_BLEN		(0xA0)
#define MT6370_PMU_REG_BLBSTCTRL	(0xA1)
#define MT6370_PMU_REG_BLPWM		(0xA2)
#define MT6370_PMU_REG_BLCTRL		(0xA3)
#define MT6370_PMU_REG_BLDIM2		(0xA4)
#define MT6370_PMU_REG_BLDIM1		(0xA5)
#define MT6370_PMU_REG_BLAFH		(0xA6)
#define MT6370_PMU_REG_BLFL		(0xA7)
#define MT6370_PMU_REG_BLFLTO		(0xA8)
#define MT6370_PMU_REG_BLTORCTRL	(0xA9)
#define MT6370_PMU_REG_BLSTRBCTRL	(0xAA)
#define MT6370_PMU_REG_BLAVG		(0xAB)
#define MT6370_PMU_REG_BLMODECTRL	(0xAD)
/* display bias control */
#define MT6370_PMU_REG_DBCTRL1		(0xB0)
#define MT6370_PMU_REG_DBCTRL2		(0xB1)
#define MT6370_PMU_REG_DBVBST		(0xB2)
#define MT6370_PMU_REG_DBVPOS		(0xB3)
#define MT6370_PMU_REG_DBVNEG		(0xB4)
/* irq event */
#define MT6370_PMU_REG_CHGIRQ1		(0xC0)
#define MT6370_PMU_REG_CHGIRQ2		(0xC1)
#define MT6370_PMU_REG_CHGIRQ3		(0xC2)
#define MT6370_PMU_REG_CHGIRQ4		(0xC3)
#define MT6370_PMU_REG_CHGIRQ5		(0xC4)
#define MT6370_PMU_REG_CHGIRQ6		(0xC5)
#define MT6370_PMU_REG_QCIRQ		(0xC6)
#define MT6370_PMU_REG_DICHGIRQ7	(0xC7)
#define MT6370_PMU_REG_OVPCTRLIRQ	(0xC8)
#define MT6370_PMU_REG_FLEDIRQ1		(0xC9)
#define MT6370_PMU_REG_FLEDIRQ2		(0xCA)
#define MT6370_PMU_REG_BASEIRQ		(0xCB)
#define MT6370_PMU_REG_LDOIRQ		(0xCC)
#define MT6370_PMU_REG_RGBIRQ		(0xCD)
#define MT6370_PMU_REG_BLIRQ		(0xCE)
#define MT6370_PMU_REG_DBIRQ		(0xCF)
/* status event */
#define MT6370_PMU_REG_CHGSTAT1		(0xD0)
#define MT6370_PMU_REG_CHGSTAT2		(0xD1)
#define MT6370_PMU_REG_CHGSTAT3		(0xD2)
#define MT6370_PMU_REG_CHGSTAT4		(0xD3)
#define MT6370_PMU_REG_CHGSTAT5		(0xD4)
#define MT6370_PMU_REG_CHGSTAT6		(0xD5)
#define MT6370_PMU_REG_QCSTAT		(0xD6)
#define MT6370_PMU_REG_DICHGSTAT	(0xD7)
#define MT6370_PMU_REG_OVPCTRLSTAT	(0xD8)
#define MT6370_PMU_REG_FLEDSTAT1	(0xD9)
#define MT6370_PMU_REG_FLEDSTAT2	(0xDA)
#define MT6370_PMU_REG_BASESTAT		(0xDB)
#define MT6370_PMU_REG_LDOSTAT		(0xDC)
#define MT6370_PMU_REG_RGBSTAT		(0xDD)
#define MT6370_PMU_REG_BLSTAT		(0xDE)
#define MT6370_PMU_REG_DBSTAT		(0xDF)
/* irq mask */
#define MT6370_PMU_CHGMASK1		(0xE0)
#define MT6370_PMU_CHGMASK2		(0xE1)
#define MT6370_PMU_CHGMASK3		(0xE2)
#define MT6370_PMU_CHGMASK4		(0xE3)
#define MT6370_PMU_CHGMASK5		(0xE4)
#define MT6370_PMU_CHGMASK6		(0xE5)
#define MT6370_PMU_DPDMMASK1		(0xE6)
#define MT6370_PMU_DICHGMASK		(0xE7)
#define MT6370_PMU_OVPCTRLMASK		(0xE8)
#define MT6370_PMU_FLEDMASK1		(0xE9)
#define MT6370_PMU_FLEDMASK2		(0xEA)
#define MT6370_PMU_BASEMASK		(0xEB)
#define MT6370_PMU_LDOMASK		(0xEC)
#define MT6370_PMU_RGBMASK		(0xED)
#define MT6370_PMU_BLMASK		(0xEE)
#define MT6370_PMU_DBMASK		(0xEF)


/*for 6370-not working now*/
#define MT6370_PMU_MAXREG			(MT6370_PMU_DBMASK)

/* PMU register defininition */
#define RT5081_VENDOR_ID		(0x80)
#define MT6370_VENDOR_ID		(0xE0)
#define MT6371_VENDOR_ID		(0xF0)
#define MT6372_VENDOR_ID		(0x90)
#define MT6372C_VENDOR_ID		(0xB0)

/* MT6370_PMU_IRQ_SET */
#define MT6370_PMU_IRQ_REGNUM	(MT6370_PMU_REG_DBIRQ - MT6370_PMU_REG_CHGIRQ1 + 1)
#define MT6370_IRQ_RETRIG	BIT(2)

#define CHIP_VEN_MASK				(0xF0)
#define CHIP_VEN_MT6370				(0xB0)
#define CHIP_REV_MASK				(0x0F)

/* IRQ definitions */
struct mt6370_pmu_irq_desc {
	const char *name;
	irq_handler_t irq_handler;
	int irq;
};

#define  MT6370_DT_VALPROP(name, type) \
			{#name, offsetof(type, name)}

struct mt6370_val_prop {
	const char *name;
	size_t offset;
};

static inline void mt6370_dt_parser_helper(struct device_node *np, void *data,
					   const struct mt6370_val_prop *props,
					   int prop_cnt)
{
	int i;

	for (i = 0; i < prop_cnt; i++) {
		if (unlikely(!props[i].name))
			continue;
		of_property_read_u32(np, props[i].name, data + props[i].offset);
	}
}

#define MT6370_PDATA_VALPROP(name, type, reg, shift, mask, func, base) \
			{offsetof(type, name), reg, shift, mask, func, base}

struct mt6370_pdata_prop {
	size_t offset;
	u8 reg;
	u8 shift;
	u8 mask;
	u32 (*transform)(u32 val);
	u8 base;
};

static inline int mt6370_pdata_apply_helper(void *context, void *pdata,
					   const struct mt6370_pdata_prop *prop,
					   int prop_cnt)
{
	int i, ret;
	u32 val;

	for (i = 0; i < prop_cnt; i++) {
		val = *(u32 *)(pdata + prop[i].offset);
		if (prop[i].transform)
			val = prop[i].transform(val);
		val += prop[i].base;
		ret = regmap_update_bits(context,
			     prop[i].reg, prop[i].mask, val << prop[i].shift);
		if (ret < 0)
			return ret;
	}
	return 0;
}
/*for 6370-not working now*/

#endif /* __MT6370_PRIVATE_H__ */
