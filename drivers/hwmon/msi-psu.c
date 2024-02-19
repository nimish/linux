// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * msi-psu.c - Linux driver for MSI PSUs with USB HID interface
 * Currently supports MSI MEG Ai1000P and MEG Ai1300P
 * Copyright (C) 2024 Nimish Telang <nimish@telang.net>
 */

#include "asm-generic/errno.h"
#include "linux/printk.h"
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

/* all linear11 */
typedef __le16 linear11;

struct msipsu_getall_reply {
	linear11 in_curr[MSI_PSU_RAIL_COUNT * 2]; /* VI pairs */
	linear11 pout;
	linear11 eff;
	linear11 temp;
	linear11 pwm;
} __packed;

static const char *const msipsu_labels_in[MSI_PSU_RAIL_COUNT] = {
	"in_12V0", "in_12V1", "in_12V2", "in_12V3",
	"in_12V4", "in_12V",  "in_5V",	 "in_3V3"
};

static const char *const msipsu_labels_curr[MSI_PSU_RAIL_COUNT] = {
	"curr_12V0", "curr_12V1", "curr_12V2", "curr_12V3",
	"curr_12V4", "curr_12V",  "curr_5V",   "curr_3V3"
};

static const char *const msipsu_labels_power[] = { "power" };

static const char *const msipsu_labels_temp[] = { "temp" };

static const struct hid_device_id msipsu_idtable[] = {
	{ HID_USB_DEVICE(USB_VID_MSI, USB_PID_MEG_AI1000P) },
	{ HID_USB_DEVICE(USB_VID_MSI, USB_PID_MEG_AI1300P) },
	{}
};

static const struct hwmon_channel_info *msipsu_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM,
			   HWMON_I_INPUT | HWMON_I_LABEL | HWMON_I_MAX_ALARM |
				   HWMON_I_MIN_ALARM),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM,
			   HWMON_C_INPUT | HWMON_C_LABEL | HWMON_C_MAX_ALARM),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_LABEL | HWMON_P_MAX_ALARM),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT),
	NULL
};

MODULE_DEVICE_TABLE(hid, msipsu_idtable);

static s32 linear11_to_int(u16 val, int scale);
static int msipsu_usb_cmd(struct msipsu_priv *priv, enum msi_psu_cmd cmd0,
			  enum msi_psu_cmd cmd1, u8 *data, size_t size);
static int msipsu_getall(struct msipsu_priv *priv,
			 struct msipsu_getall_reply *reply);
static int msipsu_handshake(struct msipsu_priv *priv);

static umode_t msipsu_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel);
static int msipsu_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val);
static int msipsu_read_string(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, const char **str);
static int msipsu_probe(struct hid_device *hdev,
			const struct hid_device_id *id);
static int msipsu_resume(struct hid_device *hdev);
static int msipsu_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size);
static int msipsu_hwmon_search_alertstatus(struct msipsu_priv *priv,
					   enum hwmon_sensor_types type,
					   u32 attr, int channel,
					   long *is_alarming);

static const struct hwmon_ops msipsu_hwmon_ops = {
	.is_visible = msipsu_is_visible,
	.read = msipsu_read,
	.read_string = msipsu_read_string,
};

static const struct hwmon_chip_info msipsu_chip_info = {
	.ops = &msipsu_hwmon_ops,
	.info = msipsu_info,
};

static struct hid_driver msipsu_driver = {
	.name = DRIVER_NAME,
	.id_table = msipsu_idtable,
	.probe = msipsu_probe,
	.raw_event = msipsu_raw_event,
#ifdef CONFIG_PM
	.resume = msipsu_resume,
	.reset_resume = msipsu_resume,
#endif
};

static int msipsu_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct msipsu_priv *priv;
	int ret;

	priv = devm_kzalloc(&hdev->dev, sizeof(struct msipsu_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

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

	priv->hdev = hdev;
	hid_set_drvdata(hdev, priv);

	mutex_init(&priv->lock);
	init_completion(&priv->wait_completion);

	hid_device_io_start(hdev);

	ret = msipsu_handshake(priv);
	if (unlikely(ret < 0)) {
		hid_err(hdev, "msi psu handshake failed (%d)\n", ret);
		return ret;
	}

	priv->hwmon_dev = devm_hwmon_device_register_with_info(
		&hdev->dev, "msipsu", priv, &msipsu_chip_info, NULL);

	ret = IS_ERR_OR_NULL(priv->hwmon_dev);
	if (ret)
		hid_err(hdev, "hwmon device registration failed (%d)\n", ret);

	return ret;
}

static __pure s32 linear11_to_int(linear11 val, s32 scale)
{
	const int exp = ((s16)val) >> 11;
	const int mant = (((s16)(val & 0x7ff)) << 5) >> 5;
	const int result = mant * scale;

	return (exp >= 0) ? (result << exp) : (result >> -exp);
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

	if (time < 0)
		return time;

	if (unlikely(!time)) {
		hid_err(priv->hdev, "usb command timeout\n");
		return -ETIMEDOUT;
	}

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

	memcpy(data, priv->cmd_buffer + 2,
	       min_t(size_t, size, CMD_BUFFER_SIZE - 2));
	memset(priv->cmd_buffer, 0, CMD_BUFFER_SIZE);

	return 0;
}

static int msipsu_handshake(struct msipsu_priv *priv)
{
	int ret;
	u8 result[11];

	ret = msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_HANDSHAKE,
				    MSI_PSU_CMD_READ, result, 11);

	if (unlikely(ret < 0)) {
		hid_err(priv->hdev, "handshake failed (%d)\n", ret);
		return ret;
	}

	switch (priv->hdev->product) {
	case USB_PID_MEG_AI1000P:
		ret = strncmp((char *)result, "MEG Ai1000P", 11) ? -ENODEV : 0;
		break;
	case USB_PID_MEG_AI1300P:
		ret = strncmp((char *)result, "MEG Ai1300P", 11) ? -ENODEV : 0;
		break;
	default:
		ret = -ENODEV;
	}

	return ret;
}

static int msipsu_getall(struct msipsu_priv *priv,
			 struct msipsu_getall_reply *reply)
{
	return msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_READ,
				     MSI_PSU_CMD_READ_ALL, reply,
				     sizeof(*reply));
}

static int msipsu_resume(struct hid_device *hdev)
{
	struct msipsu_priv *priv = hid_get_drvdata(hdev);

	return msipsu_handshake(priv);
}

static int msipsu_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	struct msipsu_priv *priv = dev_get_drvdata(dev);
	struct msipsu_getall_reply reply;
	int ret;

	hid_dbg(priv->hdev,
		"HWMON request received (type=%d, attr=%d, channel=%d)\n", type,
		attr, channel);

	ret = msipsu_getall(priv, &reply);
	if (unlikely(ret < 0))
		return ret;

	switch (type) {
	case hwmon_curr:
		if (attr == hwmon_curr_max_alarm) {
			return msipsu_hwmon_search_alertstatus(priv, type, attr,
							       channel, val);
		} else if (attr == hwmon_curr_input && channel >= 0 &&
			   channel < MSI_PSU_RAIL_COUNT) {
			*val = linear11_to_int(reply.in_curr[channel * 2 + 1],
					       MILLI); /* A -> mA */
			return 0;
		}
		break;
	case hwmon_in:
		if (attr == hwmon_in_max_alarm || attr == hwmon_in_min_alarm) {
			return msipsu_hwmon_search_alertstatus(priv, type, attr,
							       channel, val);
		} else if (attr == hwmon_in_input && channel >= 0 &&
			   channel < MSI_PSU_RAIL_COUNT) {
			*val = linear11_to_int(reply.in_curr[channel * 2],
					       MILLI); /* V -> mV */
			return 0;
		}
		break;
	case hwmon_temp:
		if (attr == hwmon_temp_max_alarm) {
			return msipsu_hwmon_search_alertstatus(priv, type, attr,
							       channel, val);
		} else if (attr == hwmon_temp_input && channel == 0) {
			/* C -> mC */
			*val = linear11_to_int(reply.temp,
					       MILLIDEGREE_PER_DEGREE);
			return 0;
		}
		break;
	case hwmon_power:
		if (attr == hwmon_power_max_alarm) {
			return msipsu_hwmon_search_alertstatus(priv, type, attr,
							       channel, val);
		} else if (attr == hwmon_power_input && channel == 0) {
			/* W -> uW */
			*val = linear11_to_int(reply.pout, MICROWATT_PER_WATT);
			return 0;
		}
		break;
	case hwmon_pwm:
		if (attr == hwmon_pwm_input && channel == 0) {
			/* % -> duty cycle */
			*val = linear11_to_int(reply.pwm, 255);
			*val = DIV_ROUND_CLOSEST(*val, 100);
			return 0;
		}
		break;
	default:
		break; /* appease compiler */
	}
	hid_err(priv->hdev,
		"unsupported attribute (type=%d, attr=%d, channel=%d)\n", type,
		attr, channel);
	return -EOPNOTSUPP;
}

static int msipsu_read_string(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, const char **str)
{
	if (type == hwmon_temp && attr == hwmon_temp_label && channel == 0) {
		*str = msipsu_labels_temp[0];
	} else if (type == hwmon_power && attr == hwmon_power_label &&
		   channel == 0) {
		*str = msipsu_labels_power[0];
	} else if (type == hwmon_in && attr == hwmon_in_label && channel >= 0 &&
		   channel < MSI_PSU_RAIL_COUNT) {
		*str = msipsu_labels_in[channel];
	} else if (type == hwmon_curr && attr == hwmon_curr_label &&
		   channel >= 0 && channel < MSI_PSU_RAIL_COUNT) {
		*str = msipsu_labels_curr[channel];
	} else {
		struct msipsu_priv *priv = dev_get_drvdata(dev);

		hid_err(priv->hdev,
			"unsupported attribute (type=%d, attr=%d, channel=%d)\n",
			type, attr, channel);
		*str = NULL;
		return -EOPNOTSUPP;
	}

	return 0;
}

static umode_t msipsu_is_visible(const void *data, enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	switch (type) {
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
		case hwmon_curr_label:
		case hwmon_curr_max_alarm:
			return channel >= 0 && channel < MSI_PSU_RAIL_COUNT ?
				       (umode_t)0444 :
				       0;
		default:
			return 0;
		}
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_label:
		case hwmon_in_max_alarm:
		case hwmon_in_min_alarm:
			return channel >= 0 && channel < MSI_PSU_RAIL_COUNT ?
				       (umode_t)0444 :
				       0;
		default:
			return 0;
		}
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			return channel == 0 ? (umode_t)0444 : 0;
		default:
			return 0;
		}

	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_label:
		case hwmon_power_max_alarm:
			return channel == 0 ? (umode_t)0444 : 0;
		default:
			return 0;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_label:
		case hwmon_temp_max_alarm:
			return channel == 0 ? (umode_t)0444 : 0;
		default:
			return 0;
		}
	default:
		return 0;
	}
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

static __pure enum msi_psu_alert_status
msipsu_alarm_to_msialert(enum hwmon_sensor_types type, u32 attr, int channel)
{
	if (type == hwmon_curr && attr == hwmon_curr_max_alarm &&
	    channel >= 0 && channel < MSI_PSU_RAIL_COUNT) {
		switch (channel) {
		case 1 ... 4: /* only applicable when multirail OCP is enabled */
			return MSI_PSU_ALERT_STATUS_OCP_12V1 + channel - 1;
		case 0:
		case 5:
			return MSI_PSU_ALERT_STATUS_OCP_12V;
		case 6:
			return MSI_PSU_ALERT_STATUS_OCP_5V;
		case 7:
			return MSI_PSU_ALERT_STATUS_OCP_3_3V;
		}
	}

	else if (type == hwmon_in && attr == hwmon_in_max_alarm &&
		 channel >= 0 && channel < MSI_PSU_RAIL_COUNT) {
		switch (channel) {
		case 0 ... 5:
			/* current protocol is broken and uses 0x00 for all clear _and_ 12V OVP*/
			return -EOPNOTSUPP;
		case 6:
			return MSI_PSU_ALERT_STATUS_OVP_5V;
		case 7:
			return MSI_PSU_ALERT_STATUS_OVP_3_3V;
		}
	}

	else if (type == hwmon_in && attr == hwmon_in_min_alarm &&
		 channel >= 0 && channel < MSI_PSU_RAIL_COUNT) {
		switch (channel) {
		case 0 ... 5:
			return MSI_PSU_ALERT_STATUS_UVP_12V;
		case 6:
			return MSI_PSU_ALERT_STATUS_UVP_5V;
		case 7:
			return MSI_PSU_ALERT_STATUS_UVP_3_3V;
		}
	}

	else if (type == hwmon_temp && attr == hwmon_temp_max_alarm &&
		 channel == 0)
		return MSI_PSU_ALERT_STATUS_OTP;

	else if (type == hwmon_power && attr == hwmon_power_max_alarm &&
		 channel == 0)
		return MSI_PSU_ALERT_STATUS_OPP;

	return -EOPNOTSUPP;
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
		return -EOPNOTSUPP;

	memset(status, 0, sizeof(status));

	ret = msipsu_usb_cmd_locked(priv, MSI_PSU_CMD_READ,
				    MSI_PSU_CMD_READ_ALERTSTATUS, status,
				    sizeof(status));
	if (unlikely(ret < 0)) {
		hid_err(priv->hdev, "unable to read alert status (%d)\n", ret);
		return ret;
	}

	u8 *p = memchr(status, msialert, sizeof(status));
	*is_alarming = !!p;

	return ret;
}

static int __init msipsu_init(void)
{
	return hid_register_driver(&msipsu_driver);
}

static void __exit msipsu_exit(void)
{
	hid_unregister_driver(&msipsu_driver);
}

/*
 * With module_init() the driver would load before the HID bus when
 * built-in, so use late_initcall() instead.
 */

late_initcall(msipsu_init);
module_exit(msipsu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nimish Telang <nimish@telang.net>");
MODULE_DESCRIPTION("Linux driver for MSI PSUs with USB HID interface");
