#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <asm/xen/hypervisor.h>

#include <xen/xen.h>
#include <xen/platform_pci.h>
#include <xen/xenbus.h>

#define AUDIO_DEVICE_NAME	"vaudio"

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

struct xenaudio_info {
	struct xenbus_device *xbdev;
	char phys[32];
};

static int xenaudio_talk_to_back(struct xenbus_device *dev,
				struct xenaudio_info *info);
static void xenaudio_connect_backend(struct xenaudio_info *info);
static void xenaudio_disconnect_backend(struct xenaudio_info *info);

static int xenaudio_remove(struct xenbus_device *dev);

static int xenaudio_probe(struct xenbus_device *dev,
				const struct xenbus_device_id *id)
{
	struct xenaudio_info *info;
	int ret;

	LOG0();
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		goto error_nomem;
	dev_set_drvdata(&dev->dev, info);
	info->xbdev = dev;
	snprintf(info->phys, sizeof(info->phys), "xenbus/%s", dev->nodename);
	/* connect backend now, get audio device(s) topology, then
	 *  initialize audio devices */
	ret = xenaudio_talk_to_back(dev, info);
	if (ret < 0)
		goto error;

	return 0;

error_nomem:
	ret = -ENOMEM;
	xenbus_dev_fatal(dev, ret, "allocating device memory");
error:
	xenaudio_remove(dev);
	return ret;
}

static int xenaudio_remove(struct xenbus_device *dev)
{
	LOG0();
	return 0;
}

static int xenaudio_resume(struct xenbus_device *dev)
{
	LOG0();
	return 0;
}

static void xenaudio_backend_changed(struct xenbus_device *dev,
				enum xenbus_state backend_state)
{
	struct xenaudio_info *info = dev_get_drvdata(&dev->dev);

	LOG0("Backend state is %d", backend_state);
	switch (backend_state) {
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateInitWait:
		break;

	case XenbusStateConnected:
		xenaudio_connect_backend(info);
		break;

	case XenbusStateUnknown:
	case XenbusStateClosed:
		if (dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		xenaudio_disconnect_backend(info);
		break;
	}
}

static int xenaudio_talk_to_back(struct xenbus_device *dev,
				struct xenaudio_info *info)
{
	LOG0("Allocating resources");
	xenbus_switch_state(dev, XenbusStateConnected);
	return 0;
}

static void xenaudio_connect_backend(struct xenaudio_info *info)
{
}

static void xenaudio_disconnect_backend(struct xenaudio_info *info)
{
}

static const struct xenbus_device_id xenaudio_ids[] = {
	{ AUDIO_DEVICE_NAME },
	{ "" }
};

static struct xenbus_driver xenaudio_driver = {
	.ids = xenaudio_ids,
	.probe = xenaudio_probe,
	.remove = xenaudio_remove,
	.resume = xenaudio_resume,
	.otherend_changed = xenaudio_backend_changed,
};

static int __init xenaudio_init(void)
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
	return xenbus_register_frontend(&xenaudio_driver);
}

static void __exit xenaudio_cleanup(void)
{
	LOG0("Unregistering XEN PV " AUDIO_DEVICE_NAME);
	xenbus_unregister_driver(&xenaudio_driver);
}

module_init(xenaudio_init);
module_exit(xenaudio_cleanup);

MODULE_DESCRIPTION("Xen virtual audio device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"AUDIO_DEVICE_NAME);
