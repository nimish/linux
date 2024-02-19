// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * msi-psu.c - Linux driver for MSI PSUs with USB HID interface
 * Currently supports MSI MEG Ai1000P and MEG Ai1300P
 * Copyright (C) 2024 Nimish Telang <nimish@telang.net>
 */

#include "linux/kern_levels.h"
#include "linux/printk.h"
#include "linux/stddef.h"
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hid.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/string.h>
#include <linux/errno.h>

#define DRIVER_NAME "msi-psu"

#define CMD_BUFFER_SIZE 65
#define CMD_TIMEOUT_MS 250
#define MSI_PSU_RAIL_COUNT 8
#define MAX_ALERTS 18

#include "msi-psu.h"

struct msipsu_priv {
	struct hid_device *hdev;
	struct device *hwmon_dev;
	struct completion wait_completion;
	struct mutex lock; /* cmd buffer access */
	u8 cmd_buffer[CMD_BUFFER_SIZE];
};

/* All channels for V/I support identical features */
static const struct hwmon_channel_info *msipsu_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(in, [0 ... MSI_PSU_RAIL_COUNT - 1] =
				       (HWMON_I_INPUT | HWMON_I_LABEL |
					HWMON_I_MAX_ALARM | HWMON_I_MIN_ALARM)),
	HWMON_CHANNEL_INFO(curr, [0 ... MSI_PSU_RAIL_COUNT - 1] =
					 (HWMON_C_INPUT | HWMON_C_LABEL |
					  HWMON_C_MAX_ALARM)),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_LABEL | HWMON_P_MAX_ALARM),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT),
	NULL
};

static __pure enum msi_psu_alert_status
msipsu_alarm_to_msialert(enum hwmon_sensor_types type, u32 attr, int channel)
{
	enum msi_psu_alert_status ocp_alert_map[] = {
		MSI_PSU_ALERT_STATUS_OCP_12V,  MSI_PSU_ALERT_STATUS_OCP_12V1,
		MSI_PSU_ALERT_STATUS_OCP_12V2, MSI_PSU_ALERT_STATUS_OCP_12V3,
		MSI_PSU_ALERT_STATUS_OCP_12V4, MSI_PSU_ALERT_STATUS_OCP_12V,
		MSI_PSU_ALERT_STATUS_OCP_5V,   MSI_PSU_ALERT_STATUS_OCP_3_3V,
	};

	enum msi_psu_alert_status ovp_alert_map[] = {
		[0 ... 5] = MSI_PSU_ALERT_STATUS_OVP_12V,
		[6] = MSI_PSU_ALERT_STATUS_OVP_5V,
		[7] = MSI_PSU_ALERT_STATUS_OVP_3_3V,
	};

	enum msi_psu_alert_status uvp_alert_map[] = {
		[0 ... 5] = MSI_PSU_ALERT_STATUS_UVP_12V,
		[6] = MSI_PSU_ALERT_STATUS_UVP_5V,
		[7] = MSI_PSU_ALERT_STATUS_UVP_3_3V,
	};

	if (type == hwmon_curr && attr == hwmon_curr_max_alarm)
		return ocp_alert_map[channel];
	else if (type == hwmon_in && attr == hwmon_in_max_alarm)
		return ovp_alert_map[channel];
	else if (type == hwmon_in && attr == hwmon_in_min_alarm)
		return uvp_alert_map[channel];
	else if (type == hwmon_temp && attr == hwmon_temp_max_alarm)
		return MSI_PSU_ALERT_STATUS_OTP;
	else if (type == hwmon_power && attr == hwmon_power_max_alarm)
		return MSI_PSU_ALERT_STATUS_OPP;

	return -EOPNOTSUPP;
}

static __pure s32 linear11_to_int(u16 val, s32 scale)
{
	const int exp = ((s16)val) >> 11;
	const int mant = (((s16)(val & 0x7ff)) << 5) >> 5;
	const int result = mant * scale;

	return (exp >= 0) ? (result << exp) : (result >> -exp);
}

static int msipsu_usb_cmd(struct msipsu_priv *priv, enum msi_psu_cmd cmd0,
			  enum msi_psu_cmd cmd1, u8 *data, size_t size)
{
	unsigned long time;
	int ret;
	int jiffies = msecs_to_jiffies(CMD_TIMEOUT_MS);

	hid_dbg(priv->hdev,
		"usb command cmd0=0x%02x, cmd1=0x%02x, size=%zu, data=%p\n",
		cmd0, cmd1, size, data);

	/* clear the buffer -- report id is always 0 */
	memset(priv->cmd_buffer, 0, CMD_BUFFER_SIZE);
	priv->cmd_buffer[1] = cmd0;
	priv->cmd_buffer[2] = cmd1;

	reinit_completion(&priv->wait_completion);

	ret = hid_hw_output_report(priv->hdev, priv->cmd_buffer,
				   CMD_BUFFER_SIZE);
	if (unlikely(ret < 0))
		return ret;

	time = wait_for_completion_interruptible_timeout(&priv->wait_completion,
							 jiffies);

	/* interrupted */
	if (time < 0)
		return time;

	if (unlikely(!time))
		return -ETIMEDOUT;

	/*
	 * at the start of the reply is an echo of the send command/length
	 * in the same order it was sent, likely inherited from PMBus
	 */
	if (cmd0 != priv->cmd_buffer[0] || cmd1 != priv->cmd_buffer[1]) {
		hid_err(priv->hdev,
			"usb command mismatch (cmd0=0x%02x, cmd1=0x%02x)\n",
			priv->cmd_buffer[0], priv->cmd_buffer[1]);
		return -EIO;
	}

	/* skip echoed bytes */
	memcpy(data, priv->cmd_buffer + 2,
	       min_t(size_t, size, CMD_BUFFER_SIZE - 2));
	memset(priv->cmd_buffer, 0, CMD_BUFFER_SIZE);

	return 0;
}

static int msipsu_usb_cmd_locked(struct msipsu_priv *priv,
				 enum msi_psu_cmd cmd0, enum msi_psu_cmd cmd1,
				 void *payload, size_t size)
{
	int ret;

	ret = mutex_lock_interruptible(&priv->lock);

	if (unlikely(ret < 0))
		return ret;

	ret = msipsu_usb_cmd(priv, cmd0, cmd1, payload, size);

	mutex_unlock(&priv->lock);

	if (unlikely(ret < 0))
		return ret;

	return ret;
}

static int msipsu_handshake(struct msipsu_priv *priv)
{
	int ret;
	u8 data[CMD_BUFFER_SIZE];
	const char *expected_product;

	if (priv->hdev->product == USB_PID_MEG_AI1000P)
		expected_product = "MEG AI1000P";
	else if (priv->hdev->product == USB_PID_MEG_AI1300P)
		expected_product = "MEG AI1300P";
	else
		return -ENODEV;

	ret = msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_HANDSHAKE,
				    MSI_PSU_CMD_READ, data, sizeof(data));

	if (unlikely(ret < 0)) {
		hid_err(priv->hdev, "handshake failed (%d)\n", ret);
		return ret;
	}

	if (!strcmp(data, expected_product)) {
		hid_err(priv->hdev, "handshake failed (expected %s)\n",
			expected_product);
		print_hex_dump(KERN_ERR, "response: ", DUMP_PREFIX_NONE, 16, 1,
			       data, sizeof(data), true);
		return -EIO;
	}

	return ret;
}

static int msipsu_resume(struct hid_device *hdev)
{
	struct msipsu_priv *priv = hid_get_drvdata(hdev);

	return msipsu_handshake(priv);
}

static int msipsu_hwmon_search_alertstatus(struct msipsu_priv *priv,
					   enum hwmon_sensor_types type,
					   u32 attr, int channel,
					   long *is_alarming)
{
	u8 status[MAX_ALERTS];
	int ret;

	enum msi_psu_alert_status msialert =
		msipsu_alarm_to_msialert(type, attr, channel);

	if (unlikely(msialert < 0))
		return msialert;

	memset(status, 0, sizeof(status));

	ret = msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_READ,
				    MSI_PSU_CMD_READ_ALERTSTATUS, status,
				    sizeof(status));
	if (unlikely(ret < 0)) {
		hid_err(priv->hdev, "unable to read alert status (%d)\n", ret);
		return ret;
	}

	*is_alarming = !!memchr(status, msialert, sizeof(status));

	return ret;
}

struct msipsu_channel_data {
	int alarm_attrs;
	s32 scale;
	size_t data_offset;
	char **labels;
};

static struct msipsu_channel_data msipsu_channel_setup[hwmon_max] = {
	[hwmon_curr] = {
		.alarm_attrs = HWMON_C_MAX_ALARM,
		.scale = MILLI,
		.labels = (char *[]){"curr_12V0", "curr_12V1", "curr_12V2",
						 "curr_12V3", "curr_12V4", "curr_12V",
						 "curr_5V", "curr_3V3"},
	},
	[hwmon_in] = {
		.alarm_attrs = HWMON_I_MAX_ALARM | HWMON_I_MIN_ALARM,
		.scale = MILLI,
		.labels = (char *[]){"in_12V0", "in_12V1", "in_12V2", "in_12V3",
						 "in_12V4", "in_12V", "in_5V", "in_3V3"}
	},
	[hwmon_temp] = {
		.alarm_attrs = HWMON_T_MAX_ALARM,
		.scale = MILLIDEGREE_PER_DEGREE,
		.labels = (char *[]){ "temp" },
	},
	[hwmon_power] = {
		.alarm_attrs = HWMON_P_MAX_ALARM,
		.scale = MICROWATT_PER_WATT,
		.labels = (char *[]){ "power" },
	},
	[hwmon_pwm] = {
		.scale = 255, /* extra division by 100 to account for percentage */
	},
};

static int msipsu_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	struct msipsu_getall_reply {
		struct {
			u16 in;
			u16 curr;
		} rails[MSI_PSU_RAIL_COUNT];
		u16 pout;
		u16 eff;
		u16 temp;
		u16 pwm;
	} __packed;

	struct msipsu_getall_reply reply;

	u16 data;
	int ret;

	struct msipsu_priv *priv = dev_get_drvdata(dev);

	struct msipsu_channel_data channel_data = msipsu_channel_setup[type];

	if (BIT(attr) & channel_data.alarm_attrs)
		return msipsu_hwmon_search_alertstatus(priv, type, attr,
						       channel, val);

	ret = msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_READ,
				    MSI_PSU_CMD_READ_ALL, &reply,
				    sizeof(reply));

	if (unlikely(ret < 0))
		return ret;

	switch (type) {
	case hwmon_in:
		data = reply.rails[channel].in;
		break;
	case hwmon_curr:
		data = reply.rails[channel].curr;
		break;
	case hwmon_temp:
		data = reply.temp;
		break;
	case hwmon_power:
		data = reply.pout;
		break;
	case hwmon_pwm:
		data = reply.pwm;
		break;
	default:
		/* should never happen */
		return -EOPNOTSUPP;
	}

	*val = linear11_to_int(data, channel_data.scale);

	if (type == hwmon_pwm)
		*val = DIV_ROUND_CLOSEST(*val, 100);

	return 0;
}

static int msipsu_read_string(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, const char **str)
{
	*str = msipsu_channel_setup[type].labels[channel];
	return 0;
}

static umode_t msipsu_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	/* protocol bug: 12V OVP alarm is always set since it's 0x00 */
	if (type == hwmon_in && attr == hwmon_in_max_alarm &&
	    ((channel) >= 0 && (channel) <= 5))
		return 0;

	return (umode_t)0444;
}

static int msipsu_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct msipsu_priv *priv = hid_get_drvdata(hdev);

	if (completion_done(&priv->wait_completion))
		return 0;

	memcpy(priv->cmd_buffer, data, min_t(size_t, CMD_BUFFER_SIZE, size));
	complete(&priv->wait_completion);

	return 0;
}

static const struct hwmon_ops msipsu_hwmon_ops = {
	.is_visible = msipsu_is_visible,
	.read = msipsu_read,
	.read_string = msipsu_read_string,
};

static const struct hwmon_chip_info msipsu_chip_info = {
	.ops = &msipsu_hwmon_ops,
	.info = msipsu_info,
};

static int msipsu_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct msipsu_priv *priv;
	int ret;

	priv = devm_kzalloc(&hdev->dev, sizeof(*priv), GFP_KERNEL);
	if (unlikely(!priv))
		return -ENOMEM;

	priv->hdev = hdev;
	hid_set_drvdata(hdev, priv);

	ret = hid_parse(hdev);
	if (unlikely(ret < 0))
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (unlikely(ret < 0))
		return ret;

	ret = devm_add_action_or_reset(&hdev->dev,
				       (void (*)(void *))hid_hw_stop, hdev);
	if (unlikely(ret))
		return ret;

	ret = hid_hw_open(hdev);
	if (unlikely(ret < 0))
		return ret;

	ret = devm_add_action_or_reset(&hdev->dev,
				       (void (*)(void *))hid_hw_close, hdev);
	if (unlikely(ret))
		return ret;

	mutex_init(&priv->lock);
	init_completion(&priv->wait_completion);

	hid_device_io_start(hdev);

	ret = msipsu_handshake(priv);
	if (unlikely(ret < 0))
		return ret;

	priv->hwmon_dev = devm_hwmon_device_register_with_info(
		&hdev->dev, "msipsu", priv, &msipsu_chip_info, NULL);

	if (IS_ERR(priv->hwmon_dev))
		return PTR_ERR(priv->hwmon_dev);

	return 0;
}

static const struct hid_device_id msipsu_idtable[] = {
	{ HID_USB_DEVICE(USB_VID_MSI, USB_PID_MEG_AI1000P) },
	{ HID_USB_DEVICE(USB_VID_MSI, USB_PID_MEG_AI1300P) },
	{}
};

static void msipsu_remove(struct hid_device *hdev)
{
	hid_dbg(hdev, "removing device\n");
}

static struct hid_driver msipsu_driver = {
	.name = DRIVER_NAME,
	.id_table = msipsu_idtable,
	.probe = msipsu_probe,
	.raw_event = msipsu_raw_event,
	.remove = msipsu_remove,
#ifdef CONFIG_PM
	.resume = msipsu_resume,
	.reset_resume = msipsu_resume,
#endif
};

static int __init msipsu_init(void)
{
	return hid_register_driver(&msipsu_driver);
}

static void __exit msipsu_exit(void)
{
	hid_unregister_driver(&msipsu_driver);
}

MODULE_DEVICE_TABLE(hid, msipsu_idtable);

/* must load after HID bus */
late_initcall(msipsu_init);
module_exit(msipsu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nimish Telang <nimish@telang.net>");
MODULE_DESCRIPTION("Linux driver for MSI PSUs with USB HID interface");
