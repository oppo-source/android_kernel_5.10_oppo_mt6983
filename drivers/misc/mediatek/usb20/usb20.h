/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2017 MediaTek Inc.
 */

#ifndef __USB20_H__
#define __USB20_H__

#if IS_ENABLED(CONFIG_FPGA_EARLY_PORTING)
#define FPGA_PLATFORM
#endif

#include <linux/interrupt.h>
#include <musb.h>

struct mt_usb_work {
	struct delayed_work dwork;
	int ops;
};

/* ToDo: should be moved to glue */
extern struct musb *mtk_musb;
extern struct musb *musb;

struct mt_usb_glue {
	struct device *dev;
	struct platform_device *musb_pdev;
	struct musb *mtk_musb;
	/* common power & clock */
	struct clk *sys_clk;
	/* optional clock */
	struct clk *ref_clk;
	struct clk *src_clk;
	struct clk *dma_clk;
	struct clk *phy_clk;
	struct clk *mcu_clk;
#if IS_ENABLED(CONFIG_PHY_MTK_TPHY)
	struct platform_device *usb_phy;
	struct phy *phy;
	struct usb_phy *xceiv;
	enum phy_mode phy_mode;
#endif
#if IS_ENABLED(CONFIG_MTK_MUSB_DUAL_ROLE)
	struct otg_switch_mtk otg_sx;
#endif
};

extern struct mt_usb_glue *glue;

#define glue_to_musb(g)         platform_get_drvdata(g->musb)

extern int kernel_init_done;

/* specific USB fuctnion */
enum CABLE_MODE {
	CABLE_MODE_CHRG_ONLY = 0,
	CABLE_MODE_NORMAL,
	CABLE_MODE_HOST_ONLY,
	CABLE_MODE_MAX
};

enum USB_CLK_STATE {
	NO_CHANGE = 0,
	ON_TO_OFF,
	OFF_TO_ON,
};

/* specific USB operation */
enum CONNECTION_OPS {
	CONNECTION_OPS_DISC = 0,
	CONNECTION_OPS_CHECK,
	CONNECTION_OPS_CONN
};

enum VBUS_OPS {
	VBUS_OPS_OFF = 0,
	VBUS_OPS_ON
};

enum MTK_USB_SMC_CALL {
	MTK_USB_SMC_INFRA_REQUEST = 0,
	MTK_USB_SMC_INFRA_RELEASE,
	MTK_USB_SMC_INFRA_RESUME,
	MTK_USB_SMC_INFRA_SUSPEND,
	MTK_USB_SMC_NUM
};

#if IS_ENABLED(CONFIG_MTK_UART_USB_SWITCH)
enum PORT_MODE {
	PORT_MODE_USB = 0,
	PORT_MODE_UART,
	PORT_MODE_MAX
};

extern bool usb_phy_check_in_uart_mode(void);
extern void usb_phy_switch_to_usb(void);
extern void usb_phy_switch_to_uart(void);
#endif

#if IS_ENABLED(CONFIG_MTK_UART_USB_SWITCH)

#define RG_GPIO_SELECT (0x600)
#define GPIO_SEL_OFFSET (4)
#define GPIO_SEL_MASK (0x7 << GPIO_SEL_OFFSET)
#define GPIO_SEL_UART0 (0x1 << GPIO_SEL_OFFSET)
#define GPIO_SEL_UART1 (0x2 << GPIO_SEL_OFFSET)
#define GET_GPIO_SEL_VAL(x) ((x & GPIO_SEL_MASK) >> GPIO_SEL_OFFSET)

extern void __iomem *ap_gpio_base;
extern bool in_uart_mode;
#endif
extern int usb20_phy_init_debugfs(void);
#if IS_ENABLED(CONFIG_USB_MTK_OTG)
extern void mt_usb_otg_init(struct musb *musb);
extern void mt_usb_otg_exit(struct musb *musb);
extern int mt_usb_get_vbus_status(struct musb *musb);
extern void mt_usb_host_connect(int delay);
extern void mt_usb_host_disconnect(int delay);
extern void mt_usb_host_connect(int delay);
extern void mt_usb_host_disconnect(int delay);
#endif
extern void musb_platform_reset(struct musb *musb);
extern int usb_enable_clock(bool enable);
extern bool usb_prepare_clock(bool enable);
extern void usb_prepare_enable_clock(bool enable);
extern void mt_usb_dev_disconnect(void);

/* usb host mode wakeup */
#define USB_WAKEUP_DEC_CON1	0x404
#define USB1_CDEN		BIT(0)
#define USB1_CDDEBOUNCE(x)	(((x) & 0xf) << 1)
#endif

/* MUSB phy set */
void set_usb_phy_mode(int mode);
void set_usb_phy_clear(void);
