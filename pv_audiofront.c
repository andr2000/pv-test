#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>


#ifdef SILENT
#define LOG(log_level, fmt, ...)
#else
#define LOG(log_level, fmt, ...) 								\
		do {													\
			printk("hello" #log_level " (%s:%d): " fmt "\n",	\
					__FUNCTION__, __LINE__ , ## __VA_ARGS__);	\
		} while (0)
#endif

#define LOG0(fmt, ...) do if (debug_level >= 0) LOG(0, fmt, ## __VA_ARGS__); while (0)

int debug_level;

static int __init xenaudio_init(void)
{
	LOG0("sss");
	return 0;
}

static void __exit xenaudio_cleanup(void)
{
}


module_init(xenaudio_init);
module_exit(xenaudio_cleanup);

MODULE_DESCRIPTION("Xen virtual audio device frontend");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("xen:vkbd");
