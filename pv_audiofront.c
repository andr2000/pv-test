/*
 *  Xen para-virtual audio device
 *  Copyright (c) 2016, Oleksandr Andrushchenko
 *
 *  Based on sound/drivers/vaudio.c
 *  Based on drivers/input/misc/xen-kbdfront.c
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <asm/xen/hypervisor.h>

#include <xen/xen.h>
#include <xen/platform_pci.h>
#include <xen/xenbus.h>

#define AUDIO_DEVICE_NAME	"vaudio"
#define AUDIO_CARD_NAME_FMT	"vcard%d"

#ifdef SILENT
#define LOG(log_level, fmt, ...)
#else
#define LOG(log_level, fmt, ...) \
		do { \
			printk(AUDIO_DEVICE_NAME #log_level " (%s:%d): " fmt "\n", \
					__FUNCTION__, __LINE__ , ## __VA_ARGS__); \
		} while (0)
#endif

#define LOG0(fmt, ...) do if (debug_level >= 0) LOG(0, fmt, ## __VA_ARGS__); while (0)

int debug_level;

struct snd_dev_vaudio_info;

struct xen_drv_vaudio_info {
	struct xenbus_device *xen_bus_dev;
	char phys[32];
	/* number of virtual cards */
	int num_cards;
	/* array of virtual audio platform devices */
	struct snd_dev_vaudio_info *snd_dev_info;
};

struct snd_dev_vaudio_info {
	char name[32];
	struct xen_drv_vaudio_info *xen_drv_info;
	struct platform_device *snd_dev;
};

/*
 * Audio driver start
 */
static int snd_drv_vaudio_probe(struct platform_device *devptr)
{
	LOG0();
	return 0;
}

static int snd_drv_vaudio_remove(struct platform_device *devptr)
{
	LOG0();
	return 0;
}

static struct platform_driver snd_drv_vaudio_info = {
	.probe		= snd_drv_vaudio_probe,
	.remove		= snd_drv_vaudio_remove,
	.driver		= {
		.name	= AUDIO_DEVICE_NAME,
	},
};

static void snd_drv_vaudio_cleanup(struct xen_drv_vaudio_info *info)
{
	LOG0();
}

static int snd_drv_vaudio_init(struct xen_drv_vaudio_info *info)
{
	int i, num_cards, err;

	LOG0();
	err = platform_driver_register(&snd_drv_vaudio_info);
	if (err < 0)
		return err;

	LOG0("platform_driver_register ok");
	/* XXX: test code - start */
	num_cards = 2;
	/* XXX: test code - stop */
	info->num_cards = num_cards;
	info->snd_dev_info = kzalloc(sizeof(info->snd_dev_info[0]) * num_cards,
			GFP_KERNEL);
	if (!info->snd_dev_info)
		goto fail;
	for (i = 0; i < num_cards; i++) {
		struct platform_device *snd_drv_dev;
		struct snd_dev_vaudio_info *snd_dev_info = &info->snd_dev_info[i];
		LOG0("Adding card %d", i);
		snprintf(snd_dev_info->name, sizeof(snd_dev_info->name),
				AUDIO_CARD_NAME_FMT, i);
		snd_drv_dev = platform_device_register_data(NULL,
				snd_dev_info->name, i, &info, sizeof(info));
		if (IS_ERR(snd_drv_dev))
			goto fail;
		snd_dev_info->snd_dev = snd_drv_dev;
	}
	LOG0("Added %d cards", num_cards);
	return 0;

fail:
	LOG0("fail");
	snd_drv_vaudio_cleanup(info);
	return -ENODEV;
}

/*
 * Audio driver stop
 */

/*
 * Xen driver start
 */

static int xen_drv_talk_to_audioback(struct xenbus_device *xbdev,
				struct xen_drv_vaudio_info *info);
static void xen_drv_vaudio_connect_backend(struct xen_drv_vaudio_info *info);
static void xen_drv_vaudio_disconnect_backend(struct xen_drv_vaudio_info *info);

static int xen_drv_vaudio_remove(struct xenbus_device *dev);

static int xen_drv_vaudio_probe(struct xenbus_device *dev,
				const struct xenbus_device_id *id)
{
	struct xen_drv_vaudio_info *info;
	int ret;

	LOG0();
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		goto error_nomem;
	dev_set_drvdata(&dev->dev, info);
	info->xen_bus_dev = dev;
	snprintf(info->phys, sizeof(info->phys), "xenbus/%s", dev->nodename);
	/* connect backend now, get audio device(s) topology, then
	 *  initialize audio devices */
	ret = xen_drv_talk_to_audioback(dev, info);
	if (ret < 0)
		goto error;

	/* XXX: this is test code. must be removed - start */
	LOG0("HACK! --------------------------------------------------");
	snd_drv_vaudio_init(info);
	/* XXX: this is test code. must be removed - stop */

	return 0;

error_nomem:
	ret = -ENOMEM;
	xenbus_dev_fatal(dev, ret, "allocating device memory");
error:
	xen_drv_vaudio_remove(dev);
	return ret;
}

static int xen_drv_vaudio_remove(struct xenbus_device *dev)
{
	struct xen_drv_vaudio_info *info = dev_get_drvdata(&dev->dev);

	LOG0();
	kfree(info);
	return 0;
}

static int xen_drv_vaudio_resume(struct xenbus_device *dev)
{
	LOG0();
	return 0;
}

static void xen_drv_vaudio_backend_changed(struct xenbus_device *dev,
				enum xenbus_state backend_state)
{
	struct xen_drv_vaudio_info *info = dev_get_drvdata(&dev->dev);

	LOG0("Backend state is %d", backend_state);
	switch (backend_state) {
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateInitWait:
		break;

	case XenbusStateConnected:
		xen_drv_vaudio_connect_backend(info);
		break;

	case XenbusStateUnknown:
	case XenbusStateClosed:
		if (dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		xen_drv_vaudio_disconnect_backend(info);
		break;
	}
}

static void xen_drv_vaudio_free(struct xen_drv_vaudio_info *info)
{
	LOG0();
}

static int xen_drv_vaudio_create(struct xenbus_device *xbdev, struct xen_drv_vaudio_info *info)
{
	LOG0();
	return 0;
}

/* Common code used when first setting up, and when resuming. */
static int xen_drv_talk_to_audioback(struct xenbus_device *xbdev,
				struct xen_drv_vaudio_info *info)
{
	int ret;

	LOG0();
	ret = xen_drv_vaudio_create(xbdev, info);
	if (ret)
		goto out;
	xenbus_switch_state(xbdev, XenbusStateInitialised);
out:
	return ret;
}

static void xen_drv_vaudio_connect_backend(struct xen_drv_vaudio_info *info)
{
	LOG0();
}

static void xen_drv_vaudio_disconnect_backend(struct xen_drv_vaudio_info *info)
{
	LOG0();
}

/*
 * Xen driver stop
 */

static const struct xenbus_device_id xen_drv_vaudio_ids[] = {
	{ AUDIO_DEVICE_NAME },
	{ "" }
};

static struct xenbus_driver xen_vaudio_driver = {
	.ids = xen_drv_vaudio_ids,
	.probe = xen_drv_vaudio_probe,
	.remove = xen_drv_vaudio_remove,
	.resume = xen_drv_vaudio_resume,
	.otherend_changed = xen_drv_vaudio_backend_changed,
};

static int __init xen_drv_vaudio_init(void)
{
	if (!xen_domain())
		return -ENODEV;
	if (xen_initial_domain()) {
		LOG0(AUDIO_DEVICE_NAME " cannot run in Dom0");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	LOG0("Registering XEN PV " AUDIO_DEVICE_NAME);
	return xenbus_register_frontend(&xen_vaudio_driver);
}

static void __exit xen_drv_vaudio_cleanup(void)
{
	LOG0("Unregistering XEN PV " AUDIO_DEVICE_NAME);
	xenbus_unregister_driver(&xen_vaudio_driver);
}

module_init(xen_drv_vaudio_init);
module_exit(xen_drv_vaudio_cleanup);

MODULE_DESCRIPTION("Xen virtual audio device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"AUDIO_DEVICE_NAME);
