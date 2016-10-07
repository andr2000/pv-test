#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>

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

static int xenaudio_probe(struct xenbus_device *dev,
				  const struct xenbus_device_id *id)
{
	LOG0(AUDIO_DEVICE_NAME);
	return 0;
}

static int xenaudio_remove(struct xenbus_device *dev)
{
	LOG0(AUDIO_DEVICE_NAME);
	return 0;
}

static void xenaudio_backend_changed(struct xenbus_device *dev,
				   enum xenbus_state backend_state)
{
	LOG0(AUDIO_DEVICE_NAME);
}

static int xenaudio_resume(struct xenbus_device *dev)
{
	LOG0(AUDIO_DEVICE_NAME);
	return 0;
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
