/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef I2C_HID_H
#define I2C_HID_H

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/i2c-hid.h>

struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;

struct i2c_hid_cmd {
	unsigned int registerIndex;
	__u8 opcode;
	unsigned int length;
	bool wait;
};

union command {
	u8 data[0];
	struct cmd {
		__le16 reg;
		__u8 reportTypeID;
		__u8 opcode;
	} __packed c;
};

/*
 * These definitions are not used here, but are defined by the spec.
 * Keeping them here for documentation purposes.
 *
 * static const struct i2c_hid_cmd hid_get_idle_cmd = { I2C_HID_CMD(0x04) };
 * static const struct i2c_hid_cmd hid_set_idle_cmd = { I2C_HID_CMD(0x05) };
 * static const struct i2c_hid_cmd hid_get_protocol_cmd = { I2C_HID_CMD(0x06) };
 * static const struct i2c_hid_cmd hid_set_protocol_cmd = { I2C_HID_CMD(0x07) };
 */

/* The main device structure */
struct i2c_hid {
	struct i2c_client	*client;	/* i2c client */
	struct hid_device	*hid;	/* pointer to corresponding HID dev */
	union {
		__u8 hdesc_buffer[sizeof(struct i2c_hid_desc)];
		struct i2c_hid_desc hdesc;	/* the HID Descriptor */
	};
	__le16			wHIDDescRegister; /* location of the i2c
						   * register of the HID
						   * descriptor. */
	unsigned int		bufsize;	/* i2c buffer size */
	u8			*inbuf;		/* Input buffer */
	u8			*rawbuf;	/* Raw Input buffer */
	u8			*cmdbuf;	/* Command buffer */
	u8			*argsbuf;	/* Command arguments buffer */

	unsigned long		flags;		/* device flags */
	unsigned long		quirks;		/* Various quirks */

	wait_queue_head_t	wait;		/* For waiting the interrupt */

	struct i2c_hid_platform_data pdata;

	bool			irq_wake_enabled;
	struct mutex		reset_lock;

	unsigned long		sleep_delay;
};

#define PAGESIZE 512

/* quirks to control the device */
#define I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV	BIT(0)
#define I2C_HID_QUIRK_NO_IRQ_AFTER_RESET	BIT(1)
#define I2C_HID_QUIRK_NO_RUNTIME_PM		BIT(2)
#define I2C_HID_QUIRK_DELAY_AFTER_SLEEP		BIT(3)
#define I2C_HID_QUIRK_BOGUS_IRQ			BIT(4)
#define I2C_HID_QUIRK_RESET_ON_RESUME		BIT(5)
#define I2C_HID_QUIRK_BAD_INPUT_SIZE		BIT(6)


/* flags */
#define I2C_HID_STARTED		0
#define I2C_HID_RESET_PENDING	1
#define I2C_HID_READ_PENDING	2

#define I2C_HID_PWR_ON		0x00
#define I2C_HID_PWR_SLEEP	0x01

#define I2C_VENDOR_ID_L_KEYBOARD		0x17ef
#define I2C_PRODUCT_ID_L_KEYBOARD		0x6103

#define I2C_VENDOR_ID_L_TOUCHPAD		0x04f3
#define I2C_PRODUCT_ID_L_TOUCHPAD		0x3164

static const struct i2c_hid_quirks {
	__u16 idVendor;
	__u16 idProduct;
	__u32 quirks;
} i2c_hid_quirks[] = {
	{ USB_VENDOR_ID_WEIDA,		USB_DEVICE_ID_WEIDA_8752,				I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ USB_VENDOR_ID_WEIDA,		USB_DEVICE_ID_WEIDA_8755,				I2C_HID_QUIRK_SET_PWR_WAKEUP_DEV },
	{ I2C_VENDOR_ID_HANTICK,	I2C_PRODUCT_ID_HANTICK_5288,			I2C_HID_QUIRK_NO_IRQ_AFTER_RESET | I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ I2C_VENDOR_ID_RAYDIUM,	I2C_PRODUCT_ID_RAYDIUM_4B33,			I2C_HID_QUIRK_DELAY_AFTER_SLEEP },
	{ USB_VENDOR_ID_LG,			I2C_DEVICE_ID_LG_8001,					I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ USB_VENDOR_ID_ELAN,		HID_ANY_ID,								I2C_HID_QUIRK_BOGUS_IRQ },
	{ USB_VENDOR_ID_ALPS_JP,	HID_ANY_ID,								I2C_HID_QUIRK_RESET_ON_RESUME },
	{ I2C_VENDOR_ID_SYNAPTICS,	I2C_PRODUCT_ID_SYNAPTICS_SYNA2393,		I2C_HID_QUIRK_RESET_ON_RESUME },
	{ USB_VENDOR_ID_ITE,		I2C_DEVICE_ID_ITE_LENOVO_LEGION_Y720,	I2C_HID_QUIRK_BAD_INPUT_SIZE },
	{ I2C_VENDOR_ID_L_KEYBOARD,	I2C_PRODUCT_ID_L_KEYBOARD,				I2C_HID_QUIRK_NO_IRQ_AFTER_RESET | I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ I2C_VENDOR_ID_L_TOUCHPAD,	I2C_PRODUCT_ID_L_TOUCHPAD,				I2C_HID_QUIRK_NO_IRQ_AFTER_RESET | I2C_HID_QUIRK_NO_RUNTIME_PM },
	{ 0, 0 }
};

static inline struct i2c_hid_desc
		   *i2c_hid_get_dmi_i2c_hid_desc_override(uint8_t *i2c_name)
{ return NULL; }
static inline char *i2c_hid_get_dmi_hid_report_desc_override(uint8_t *i2c_name,
							     unsigned int *size)
{ return NULL; }

#endif
