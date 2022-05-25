// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Zhiyong Tao <zhiyong.tao@mediatek.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/nvmem-consumer.h>

/* Register definitions */
#define MT6577_AUXADC_CON0                    0x00
#define MT6577_AUXADC_CON1                    0x04
#define MT6577_AUXADC_CON1_SET                0x08
#define MT6577_AUXADC_CON1_CLR                0x0C
#define MT6577_AUXADC_CON2                    0x10
#define MT6577_AUXADC_STA                     BIT(0)

#define MT6577_AUXADC_DAT0                    0x14
#define MT6577_AUXADC_RDY0                    BIT(12)

#define MT6577_AUXADC_MISC                    0x94
#define MT6577_AUXADC_PDN_EN                  BIT(14)

#define MT6577_AUXADC_DAT_MASK                0xfff
#define MT6577_AUXADC_SLEEP_US                1000
#define MT6577_AUXADC_TIMEOUT_US              10000
#define MT6577_AUXADC_POWER_READY_MS          1
#define MT6577_AUXADC_SAMPLE_READY_US         25

struct mtk_auxadc_compatible {
	bool sample_data_cali;
	bool check_global_idle;
};

struct mt6577_auxadc_device {
	void __iomem *reg_base;
	struct clk *adc_clk;
	struct mutex lock;
	const struct mtk_auxadc_compatible *dev_comp;
};

static const struct mtk_auxadc_compatible mt8173_compat = {
	.sample_data_cali = false,
	.check_global_idle = true,
};

static const struct mtk_auxadc_compatible mt6765_compat = {
	.sample_data_cali = true,
	.check_global_idle = false,
};

#define MT6577_AUXADC_CHANNEL(idx) {				    \
		.type = IIO_VOLTAGE,				    \
		.indexed = 1,					    \
		.channel = (idx),				    \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	    \
				      BIT(IIO_CHAN_INFO_PROCESSED), \
}

static const struct iio_chan_spec mt6577_auxadc_iio_channels[] = {
	MT6577_AUXADC_CHANNEL(0),
	MT6577_AUXADC_CHANNEL(1),
	MT6577_AUXADC_CHANNEL(2),
	MT6577_AUXADC_CHANNEL(3),
	MT6577_AUXADC_CHANNEL(4),
	MT6577_AUXADC_CHANNEL(5),
	MT6577_AUXADC_CHANNEL(6),
	MT6577_AUXADC_CHANNEL(7),
	MT6577_AUXADC_CHANNEL(8),
	MT6577_AUXADC_CHANNEL(9),
	MT6577_AUXADC_CHANNEL(10),
	MT6577_AUXADC_CHANNEL(11),
	MT6577_AUXADC_CHANNEL(12),
	MT6577_AUXADC_CHANNEL(13),
	MT6577_AUXADC_CHANNEL(14),
	MT6577_AUXADC_CHANNEL(15),
};

/* For Voltage calculation */
#define VOLTAGE_FULL_RANGE  1500	/* VA voltage */
#define AUXADC_PRECISE      4096	/* 12 bits */

/* For calibration */
#define ADC_GE_OE_MASK          0x000003ff
#define ADC_GE_OE_EN_MASK       0x00000001

struct adc_cali_info {
	u32 efuse_en_bs;        /* dt efuse en bit shift */
	u32 efuse_ge_bs;        /* dt efuse ge bit shift */
	u32 efuse_oe_bs;        /* dt efuse oe bit shift */
	u32 efuse_reg_offset;   /* dt efuse reg offset */
	u32 efuse_reg_value;    /* efuse reg value */
	u32 efuse_en;           /* efuse en value */
	u32 efuse_ge;           /* efuse ge value */
	u32 efuse_oe;           /* efuse oe value */
	s32 cali_ge;            /* cali ge value */
	s32 cali_oe;            /* cali oe value */
};

static struct adc_cali_info adc_cali;

static int mt_auxadc_update_cali(struct device *dev)
{
	struct device_node *np = dev->of_node;
#if IS_ENABLED(CONFIG_MTK_DEVINFO)
	struct nvmem_device *nvmem_dev;
	struct device_node *efuse_node;
	struct platform_device *efuse_pdev;
	struct device_link  *link;
#endif
	u32 reg;
	int ret = 0;

	if (np) {
		ret = of_property_read_u32(np, "mediatek,cali-en-bit",
			&adc_cali.efuse_en_bs);
		if (ret)
			goto err;

		ret = of_property_read_u32(np, "mediatek,cali-ge-bit",
			&adc_cali.efuse_ge_bs);
		if (ret)
			goto err;

		ret = of_property_read_u32(np, "mediatek,cali-oe-bit",
			&adc_cali.efuse_oe_bs);
		if (ret)
			goto err;

#if IS_ENABLED(CONFIG_MTK_DEVINFO)
		efuse_node = of_find_compatible_node(NULL, NULL,
						     "mediatek,devinfo");
		if (!efuse_node) {
			dev_notice(dev, "find mediatek,devinfo fail\n");
			ret = -ENODEV;
			goto err;
		}
		efuse_pdev = of_find_device_by_node(efuse_node);
		if (!efuse_pdev) {
			dev_notice(dev, "find efuse_pdev fail\n");
			ret = -ENODEV;
			goto err;
		}
		link = device_link_add(dev, &efuse_pdev->dev,
			DL_FLAG_AUTOPROBE_CONSUMER);
		if (!link) {
			dev_notice(dev, "add efuse device_link fail\n");
			ret = -ENODEV;
			goto err;
		}
		/* supplier is not probed */
		if (link->status == DL_STATE_DORMANT) {
			ret = -EPROBE_DEFER;
			goto err;
		}

		ret = of_property_read_u32(np, "mediatek,cali-efuse-reg-offset",
			&adc_cali.efuse_reg_offset);
		if (ret)
			goto err;
		nvmem_dev = nvmem_device_get(dev, "mtk_efuse");
		if (IS_ERR(nvmem_dev)) {
			dev_notice(dev, "failed to get mtk_efuse device\n");
			ret = -ENODEV;
			goto err;
		}
		ret = nvmem_device_read(nvmem_dev,
			adc_cali.efuse_reg_offset, 4, &reg);
		if (ret != 4) {
			dev_notice(dev, "error efuse read size: %d\n", ret);
			nvmem_device_put(nvmem_dev);
			ret = -ENODEV;
			goto err;
		}
		nvmem_device_put(nvmem_dev);
#else
		ret = of_property_read_u32(np, "mediatek,cali-efuse-index",
			&adc_cali.efuse_reg_offset);
		if (ret)
			goto err;

		reg = get_devinfo_with_index(adc_cali.efuse_reg_offset);
#endif

		adc_cali.efuse_reg_value = reg;

		adc_cali.efuse_en = (reg >> adc_cali.efuse_en_bs) &
			ADC_GE_OE_EN_MASK;

		if (adc_cali.efuse_en) {
			adc_cali.efuse_oe =
				(reg >> adc_cali.efuse_oe_bs) & ADC_GE_OE_MASK;
			adc_cali.efuse_ge =
				(reg >> adc_cali.efuse_ge_bs) & ADC_GE_OE_MASK;

			/* In sw implement guide, ge should div 4096.
			 * But we don't do that now due to it
			 * will multi 4096 later
			 */
			adc_cali.cali_ge = adc_cali.efuse_ge - 512;
			adc_cali.cali_oe = adc_cali.efuse_oe - 512;
		}

		return 0;
	}
err:
	dev_notice(dev, "fail to get some dt info! ret=%d\n", ret);
	return ret;
}

static int mt_auxadc_get_cali_data(int rawdata, bool enable_cali)
{
	int data;

	/* In sw implement guide, 4096 * gain = 4096 * (1 + GE)
	 * = 4096 * (1 + cali_ge / 4096) = 4096 + cali_ge)
	 */
	if (enable_cali)
		data = (AUXADC_PRECISE * (rawdata - adc_cali.cali_oe)) /
			(AUXADC_PRECISE + adc_cali.cali_ge);
	else
		data = rawdata;

	return data;
}

static inline void mt6577_auxadc_mod_reg(void __iomem *reg,
					 u32 or_mask, u32 and_mask)
{
	u32 val;

	val = readl(reg);
	val |= or_mask;
	val &= ~and_mask;
	writel(val, reg);
}

static int mt6577_auxadc_read(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan)
{
	u32 val;
	void __iomem *reg_channel;
	int ret;
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	reg_channel = adc_dev->reg_base + MT6577_AUXADC_DAT0 +
		      chan->channel * 0x04;

	mutex_lock(&adc_dev->lock);

	writel(1 << chan->channel, adc_dev->reg_base + MT6577_AUXADC_CON1_CLR);

	/* read channel and make sure old ready bit == 0 */
	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) == 0),
				 MT6577_AUXADC_SLEEP_US,
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
			"wait for channel[%d] ready bit clear time out\n",
			chan->channel);
		goto err_timeout;
	}

	/* set bit to trigger sample */
	writel(1 << chan->channel, adc_dev->reg_base + MT6577_AUXADC_CON1_SET);

	/* we must delay here for hardware sample channel data */
	udelay(MT6577_AUXADC_SAMPLE_READY_US);

	if (adc_dev->dev_comp->check_global_idle) {
		/* check MTK_AUXADC_CON2 if auxadc is idle */
		ret = readl_poll_timeout(adc_dev->reg_base + MT6577_AUXADC_CON2,
					 val, ((val & MT6577_AUXADC_STA) == 0),
					 MT6577_AUXADC_SLEEP_US,
					 MT6577_AUXADC_TIMEOUT_US);
		if (ret < 0) {
			dev_err(indio_dev->dev.parent,
				"wait for auxadc idle time out\n");
			goto err_timeout;
		}
	}

	/* read channel and make sure ready bit == 1 */
	ret = readl_poll_timeout(reg_channel, val,
				 ((val & MT6577_AUXADC_RDY0) != 0),
				 MT6577_AUXADC_SLEEP_US,
				 MT6577_AUXADC_TIMEOUT_US);
	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
			"wait for channel[%d] data ready time out\n",
			chan->channel);
		goto err_timeout;
	}

	/* read data */
	val = readl(reg_channel) & MT6577_AUXADC_DAT_MASK;

	mutex_unlock(&adc_dev->lock);

	return val;

err_timeout:

	mutex_unlock(&adc_dev->lock);

	return -ETIMEDOUT;
}

static int mt6577_auxadc_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val,
				  int *val2,
				  long info)
{
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = mt6577_auxadc_read(indio_dev, chan);
		if (*val < 0) {
			dev_notice(indio_dev->dev.parent,
				"failed to sample data on channel[%d]\n",
				chan->channel);
			return *val;
		}
		if (adc_dev->dev_comp->sample_data_cali)
			*val = mt_auxadc_get_cali_data(*val, true);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		*val = mt6577_auxadc_read(indio_dev, chan);
		if (*val < 0) {
			dev_err(indio_dev->dev.parent,
				"failed to sample data on channel[%d]\n",
				chan->channel);
			return *val;
		}
		if (adc_dev->dev_comp->sample_data_cali)
			*val = mt_auxadc_get_cali_data(*val, true);

		/* Convert adc raw data to voltage: 0 - 1500 mV */
		*val = *val * VOLTAGE_FULL_RANGE / AUXADC_PRECISE;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info mt6577_auxadc_info = {
	.read_raw = &mt6577_auxadc_read_raw,
};

static int __maybe_unused mt6577_auxadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret;

	ret = clk_prepare_enable(adc_dev->adc_clk);
	if (ret) {
		pr_err("failed to enable auxadc clock\n");
		return ret;
	}

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      MT6577_AUXADC_PDN_EN, 0);
	mdelay(MT6577_AUXADC_POWER_READY_MS);

	return 0;
}

static int __maybe_unused mt6577_auxadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);
	clk_disable_unprepare(adc_dev->adc_clk);

	return 0;
}

static int mt6577_auxadc_probe(struct platform_device *pdev)
{
	struct mt6577_auxadc_device *adc_dev;
	unsigned long adc_clk_rate;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc_dev));
	if (!indio_dev)
		return -ENOMEM;

	adc_dev = iio_priv(indio_dev);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &mt6577_auxadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = mt6577_auxadc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt6577_auxadc_iio_channels);

	adc_dev->reg_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc_dev->reg_base)) {
		dev_err(&pdev->dev, "failed to get auxadc base address\n");
		return PTR_ERR(adc_dev->reg_base);
	}

	adc_dev->adc_clk = devm_clk_get(&pdev->dev, "main");
	if (IS_ERR(adc_dev->adc_clk)) {
		dev_err(&pdev->dev, "failed to get auxadc clock\n");
		return PTR_ERR(adc_dev->adc_clk);
	}

	ret = clk_prepare_enable(adc_dev->adc_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable auxadc clock\n");
		return ret;
	}

	adc_clk_rate = clk_get_rate(adc_dev->adc_clk);
	if (!adc_clk_rate) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "null clock rate\n");
		goto err_disable_clk;
	}

	adc_dev->dev_comp = device_get_match_data(&pdev->dev);

	if (adc_dev->dev_comp->sample_data_cali) {
		ret = mt_auxadc_update_cali(&pdev->dev);
		if (ret)
			goto err_disable_clk;
	}

	mutex_init(&adc_dev->lock);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      MT6577_AUXADC_PDN_EN, 0);
	mdelay(MT6577_AUXADC_POWER_READY_MS);

	platform_set_drvdata(pdev, indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio device\n");
		goto err_power_off;
	}

	dev_info(&pdev->dev, "%s done\n", __func__);

	return 0;

err_power_off:
	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);
err_disable_clk:
	clk_disable_unprepare(adc_dev->adc_clk);
	return ret;
}

static int mt6577_auxadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct mt6577_auxadc_device *adc_dev = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	mt6577_auxadc_mod_reg(adc_dev->reg_base + MT6577_AUXADC_MISC,
			      0, MT6577_AUXADC_PDN_EN);

	clk_disable_unprepare(adc_dev->adc_clk);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mt6577_auxadc_pm_ops,
			 mt6577_auxadc_suspend,
			 mt6577_auxadc_resume);

static const struct of_device_id mt6577_auxadc_of_match[] = {
	{ .compatible = "mediatek,mt2701-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt2712-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt7622-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt8173-auxadc", .data = &mt8173_compat},
	{ .compatible = "mediatek,mt6765-auxadc", .data = &mt6765_compat},
	{ }
};
MODULE_DEVICE_TABLE(of, mt6577_auxadc_of_match);

static struct platform_driver mt6577_auxadc_driver = {
	.driver = {
		.name   = "mt6577-auxadc",
		.of_match_table = mt6577_auxadc_of_match,
		.pm = &mt6577_auxadc_pm_ops,
	},
	.probe	= mt6577_auxadc_probe,
	.remove	= mt6577_auxadc_remove,
};
module_platform_driver(mt6577_auxadc_driver);

MODULE_AUTHOR("Zhiyong Tao <zhiyong.tao@mediatek.com>");
MODULE_DESCRIPTION("MTK AUXADC Device Driver");
MODULE_LICENSE("GPL v2");
