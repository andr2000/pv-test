/*
 *  Xen para-virtual audio device
 *  Copyright (c) 2016, Oleksandr Andrushchenko
 *
 *  Based on sound/drivers/dummy.c
 *  Based on drivers/net/xen-netfront.c
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

#define VAUDIO_DRIVER_NAME	"vaudio"

#ifdef SILENT
#define LOG(log_level, fmt, ...)
#else
#define LOG(log_level, fmt, ...) \
		do { \
			printk(VAUDIO_DRIVER_NAME #log_level " (%s:%d): " fmt "\n", \
					__FUNCTION__, __LINE__ , ## __VA_ARGS__); \
		} while (0)
#endif

#define LOG0(fmt, ...) do if (debug_level >= 0) LOG(0, fmt, ## __VA_ARGS__); while (0)

int debug_level;

struct xen_drv_vaudio_info {
	struct xenbus_device *xen_bus_dev;
	char *phys;
	/* number of virtual cards */
	int num_cards;
	/* array of virtual audio platform devices */
	struct platform_device **snd_drv_dev;
};

struct snd_dev_card_platdata {
	struct xen_drv_vaudio_info *xen_drv_info;
	int index;
	int num_streams_playback;
	int num_streams_capture;
};

struct snd_dev_card_info {
	struct xen_drv_vaudio_info *xen_drv_info;
	int index;
};

/*
 * Audio driver start
 */
static int snd_drv_vaudio_probe(struct platform_device *pdev)
{
	struct snd_dev_card_info *info;
	struct snd_dev_card_platdata *platdata = dev_get_platdata(&pdev->dev);
	LOG0("Probing Card %d", platdata->index);
	info = devm_kzalloc(&pdev->dev,	sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->xen_drv_info = platdata->xen_drv_info;
	info->index = platdata->index;
	LOG0("Will configure %d playback and %d capture streams",
			platdata->num_streams_playback, platdata->num_streams_capture);
	dev_set_drvdata(&pdev->dev, info);
	return 0;
}

static int snd_drv_vaudio_remove(struct platform_device *pdev)
{
	struct snd_dev_card_info *info = platform_get_drvdata(pdev);
	LOG0("Removing Card %d", info->index);
	return 0;
}

static struct platform_driver snd_drv_vaudio_info = {
	.probe		= snd_drv_vaudio_probe,
	.remove		= snd_drv_vaudio_remove,
	.driver		= {
		.name	= VAUDIO_DRIVER_NAME,
	},
};

static void snd_drv_vaudio_cleanup(struct xen_drv_vaudio_info *info)
{
	int i;

	LOG0("Cleaning audio driver");
	for (i = 0; i < info->num_cards; i++) {
		struct platform_device *snd_drv_dev;

		LOG0("Removing audio card %d", i);
		snd_drv_dev = info->snd_drv_dev[i];
		if (snd_drv_dev)
			platform_device_unregister(snd_drv_dev);
	}
	LOG0("Removing audio driver");
	platform_driver_unregister(&snd_drv_vaudio_info);
	LOG0("Audio driver cleanup complete");
}

static int snd_drv_vaudio_init(struct xen_drv_vaudio_info *info)
{
	int i, num_cards, ret;

	LOG0();
	ret = platform_driver_register(&snd_drv_vaudio_info);
	if (ret < 0)
		return ret;

	LOG0("platform_driver_register ok");
	/* XXX: test code - start */
	num_cards = 2;
	/* XXX: test code - stop */
	info->snd_drv_dev = devm_kzalloc(&info->xen_bus_dev->dev,
			sizeof(info->snd_drv_dev[0]) * num_cards, GFP_KERNEL);
	if (!info->snd_drv_dev)
		goto fail;
	info->num_cards = num_cards;
	for (i = 0; i < num_cards; i++) {
		struct platform_device *snd_drv_dev;
		struct snd_dev_card_platdata snd_dev_platdata;

		LOG0("Adding card %d", i);
		/* pass card configuration via platform data */
		memset(&snd_dev_platdata, 0, sizeof(snd_dev_platdata));
		snd_dev_platdata.xen_drv_info = info;
		/* XXX: the config below must be acquired from the backend */
		snd_dev_platdata.index = i;
		snd_dev_platdata.num_streams_capture = i + 1;
		snd_dev_platdata.num_streams_playback = i * 2 + 1;
		snd_drv_dev = platform_device_register_data(NULL, VAUDIO_DRIVER_NAME,
				i, &snd_dev_platdata, sizeof(snd_dev_platdata));
		if (IS_ERR(snd_drv_dev))
			goto fail;
		info->snd_drv_dev[i] = snd_drv_dev;
	}
	LOG0("Added %d cards", num_cards);
	return 0;

fail:
	LOG0("Failed to register audio driver");
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
static void xen_drv_vaudio_on_backend_connected(struct xen_drv_vaudio_info *info);
static void xen_drv_vaudio_disconnect_backend(struct xen_drv_vaudio_info *info);

static int xen_drv_vaudio_remove(struct xenbus_device *dev);

static int xen_drv_vaudio_probe(struct xenbus_device *xbdev,
				const struct xenbus_device_id *id)
{
	struct xen_drv_vaudio_info *info;
	int ret;

	LOG0();
	info = devm_kzalloc(&xbdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto fail;
	}
	dev_set_drvdata(&xbdev->dev, info);
	info->xen_bus_dev = xbdev;
	info->phys = devm_kasprintf(&xbdev->dev, GFP_KERNEL, "xenbus/%s", xbdev->nodename);
	if (!info->phys) {
		ret = -ENOMEM;
		goto fail;
	}

	/* XXX: this is test code. must be removed - start */
	LOG0("HACK! --------------------------------------------------");
	ret = snd_drv_vaudio_init(info);
	if (ret < 0)
		goto fail;
	/* XXX: this is test code. must be removed - stop */
	return 0;
fail:
	xenbus_dev_fatal(xbdev, ret, "allocating device memory");
	return ret;
}

static int xen_drv_vaudio_remove(struct xenbus_device *dev)
{
	struct xen_drv_vaudio_info *info = dev_get_drvdata(&dev->dev);

	LOG0("Removing audio driver");
	snd_drv_vaudio_cleanup(info);
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

	LOG0("Backend state is %s, front is %s", xenbus_strstate(backend_state),
			xenbus_strstate(dev->state));
	switch (backend_state) {
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateInitialising:
	case XenbusStateInitWait:
		break;

	case XenbusStateInitialised:
		if (dev->state != XenbusStateInitialising)
			break;
		if (xen_drv_talk_to_audioback(dev, info) != 0)
			break;
		break;

	case XenbusStateConnected:
		xen_drv_vaudio_on_backend_connected(info);
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

static void xen_drv_vaudio_on_backend_connected(struct xen_drv_vaudio_info *info)
{
	int ret;
	LOG0();
	ret = snd_drv_vaudio_init(info);
	xenbus_switch_state(info->xen_bus_dev, XenbusStateConnected);
}

static void xen_drv_vaudio_disconnect_backend(struct xen_drv_vaudio_info *info)
{
	LOG0();
	xen_drv_vaudio_free(info);
}

/*
 * Xen driver stop
 */

static const struct xenbus_device_id xen_drv_vaudio_ids[] = {
	{ VAUDIO_DRIVER_NAME },
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
		LOG0(VAUDIO_DRIVER_NAME " cannot run in Dom0");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	LOG0("Registering XEN PV " VAUDIO_DRIVER_NAME);
	return xenbus_register_frontend(&xen_vaudio_driver);
}

static void __exit xen_drv_vaudio_cleanup(void)
{
	LOG0("Unregistering XEN PV " VAUDIO_DRIVER_NAME);
	xenbus_unregister_driver(&xen_vaudio_driver);
}

module_init(xen_drv_vaudio_init);
module_exit(xen_drv_vaudio_cleanup);

MODULE_DESCRIPTION("Xen virtual audio device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"VAUDIO_DRIVER_NAME);
