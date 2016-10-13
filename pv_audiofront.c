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

#include <sound/core.h>
#include <sound/pcm.h>

#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/platform_pci.h>
#include <xen/xenbus.h>

#include "vaudioif.h"

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
	/* array of virtual audio platform devices */
	struct platform_device **snd_drv_dev;

	/* XXX: this comes from back to end configuration negotiation */
	/* number of virtual cards */
	int cfg_num_cards;
	/* card configuration */
	struct vaudioif_card_config *cfg_cards;
};

struct snd_dev_card_platdata {
	struct xen_drv_vaudio_info *xen_drv_info;
	int index;
	struct vaudioif_card_config *card_config;
};

struct snd_dev_card_info {
	struct xen_drv_vaudio_info *xen_drv_info;
	struct snd_card *card;
	/* array of PCM instances of this card */
	int num_pcm_instances;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	int index;
};

/* XXX: remove me */
#include "dbg_pv_audiofront.c"
/* XXX: remove me */


/*
 * Audio driver start
 */
int snd_drv_pcm_open(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_close(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_hw_params(struct snd_pcm_substream *substream,
		 struct snd_pcm_hw_params *params)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_hw_free(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_prepare(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

snd_pcm_uframes_t snd_drv_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

snd_pcm_uframes_t snd_drv_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	LOG0("Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_playback_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	LOG0("Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
	return 0;
}

int snd_drv_pcm_capture_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	LOG0("Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
	return 0;
}

int snd_drv_pcm_playback_silence(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	LOG0("Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
	return 0;
}

/* defaults */
#define MAX_BUFFER_SIZE		(4*1024)
#define MIN_PERIOD_SIZE		64
#define MAX_PERIOD_SIZE		MAX_BUFFER_SIZE
#define USE_FORMATS 		(SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE		SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define USE_RATE_MIN		5500
#define USE_RATE_MAX		48000
#define USE_CHANNELS_MIN 	1
#define USE_CHANNELS_MAX 	2
#define USE_PERIODS_MIN 	1
#define USE_PERIODS_MAX 	1024

static struct snd_pcm_hardware snd_drv_pcm_hardware = {
		.info =			(SNDRV_PCM_INFO_MMAP |
					 SNDRV_PCM_INFO_INTERLEAVED |
					 SNDRV_PCM_INFO_RESUME |
					 SNDRV_PCM_INFO_MMAP_VALID),
		.formats =		USE_FORMATS,
		.rates =		USE_RATE,
		.rate_min =		USE_RATE_MIN,
		.rate_max =		USE_RATE_MAX,
		.channels_min =		USE_CHANNELS_MIN,
		.channels_max =		USE_CHANNELS_MAX,
		.buffer_bytes_max =	MAX_BUFFER_SIZE,
		.period_bytes_min =	MIN_PERIOD_SIZE,
		.period_bytes_max =	MAX_PERIOD_SIZE,
		.periods_min =		USE_PERIODS_MIN,
		.periods_max =		USE_PERIODS_MAX,
		.fifo_size =		0,
};

static struct snd_pcm_ops snd_drv_pcm_playback_ops = {
		.open =		snd_drv_pcm_open,
		.close =	snd_drv_pcm_close,
		.ioctl =	snd_pcm_lib_ioctl,
		.hw_params =	snd_drv_pcm_hw_params,
		.hw_free =	snd_drv_pcm_hw_free,
		.prepare =	snd_drv_pcm_prepare,
		.trigger =	snd_drv_pcm_trigger,
		.pointer =	snd_drv_pcm_playback_pointer,
		.copy =		snd_drv_pcm_playback_copy,
		.silence =	snd_drv_pcm_playback_silence,
};

static struct snd_pcm_ops snd_drv_pcm_capture_ops = {
		.open =		snd_drv_pcm_open,
		.close =	snd_drv_pcm_close,
		.ioctl =	snd_pcm_lib_ioctl,
		.hw_params =	snd_drv_pcm_hw_params,
		.hw_free =	snd_drv_pcm_hw_free,
		.prepare =	snd_drv_pcm_prepare,
		.trigger =	snd_drv_pcm_trigger,
		.pointer =	snd_drv_pcm_capture_pointer,
		.copy =		snd_drv_pcm_capture_copy,
};

static int snd_drv_vaudio_new_pcm(struct snd_dev_card_info *card_info,
		struct vaudioif_pcm_instance_config *instance_config,
		struct snd_pcm *pcm)
{
	int ret;

	LOG0("Device \"%s\" with id %d playback %d capture %d",
			instance_config->name,
			instance_config->device,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap);
	ret = snd_pcm_new(card_info->card, instance_config->name,
			instance_config->device,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap,
			&pcm);
	if (ret < 0)
		return ret;
	if (instance_config->num_streams_pb)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&snd_drv_pcm_playback_ops);
	if (instance_config->num_streams_cap)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&snd_drv_pcm_capture_ops);
	pcm->private_data = card_info;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Virtual card PCM");
	return 0;
}

static int snd_drv_vaudio_probe(struct platform_device *pdev)
{
	struct snd_dev_card_info *card_info;
	struct snd_dev_card_platdata *platdata;
	struct snd_card *card;
	struct vaudioif_card_pcm_hw_config *card_pcm_hw;
	char card_id[sizeof(card->id)];
	int ret, i;

	platdata = dev_get_platdata(&pdev->dev);
	LOG0("Creating virtual sound card %d", platdata->index);
	LOG0("Will configure %d playback/capture streams",
			platdata->card_config->num_pcm_instances);

	snprintf(card_id, sizeof(card->id), VAUDIO_DRIVER_NAME "%d",
			platdata->index);
	ret = snd_card_new(&pdev->dev, platdata->index, card_id, THIS_MODULE,
			sizeof(struct snd_dev_card_info), &card);
	if (ret < 0)
		return ret;
	/* card_info is allocated and maintained by snd_card_new */
	card_info = card->private_data;
	card_info->xen_drv_info = platdata->xen_drv_info;
	card_info->index = platdata->index;
	card_info->card = card;
	card_info->pcm = devm_kzalloc(&pdev->dev,
			platdata->card_config->num_pcm_instances *
			sizeof(struct snd_pcm), GFP_KERNEL);
	if (!card_info->pcm)
		goto fail;
	card_info->num_pcm_instances = platdata->card_config->num_pcm_instances;

	for (i = 0; i < platdata->card_config->num_pcm_instances; i++) {
		ret = snd_drv_vaudio_new_pcm(card_info,
				&platdata->card_config->pcm_instance[i],
				&card_info->pcm[i]);
		if (ret < 0)
			goto fail;
	}
	card_info->pcm_hw = snd_drv_pcm_hardware;
	card_pcm_hw = &platdata->card_config->card_pcm_hw;
	if (card_pcm_hw->formats)
		card_info->pcm_hw.formats = card_pcm_hw->formats;
	if (card_pcm_hw->buffer_bytes_max)
		card_info->pcm_hw.buffer_bytes_max =
				card_pcm_hw->buffer_bytes_max;
	if (card_pcm_hw->period_bytes_min)
		card_info->pcm_hw.period_bytes_min =
				card_pcm_hw->period_bytes_min;
	if (card_pcm_hw->period_bytes_max)
		card_info->pcm_hw.period_bytes_max =
				card_pcm_hw->period_bytes_max;
	if (card_pcm_hw->periods_min)
		card_info->pcm_hw.periods_min = card_pcm_hw->periods_min;
	if (card_pcm_hw->periods_max)
		card_info->pcm_hw.periods_max = card_pcm_hw->periods_max;
	if (card_pcm_hw->rates)
		card_info->pcm_hw.rates = card_pcm_hw->rates;
	if (card_pcm_hw->rate_min)
		card_info->pcm_hw.rate_min = card_pcm_hw->rate_min;
	if (card_pcm_hw->rate_max)
		card_info->pcm_hw.rate_max = card_pcm_hw->rate_max;
	if (card_pcm_hw->channels_min)
		card_info->pcm_hw.channels_min = card_pcm_hw->channels_min;
	if (card_pcm_hw->channels_max)
		card_info->pcm_hw.channels_max = card_pcm_hw->channels_max;
#if 0
		err = snd_card_dummy_new_mixer(dummy);
		if (err < 0)
			goto __nodev;

		dummy_proc_init(dummy);

#endif
	strncpy(card->driver, VAUDIO_DRIVER_NAME, sizeof(card->driver));
	strncpy(card->shortname, platdata->card_config->shortname,
			sizeof(card->shortname));
	strncpy(card->longname, platdata->card_config->longname,
			sizeof(card->longname));
	ret = snd_card_register(card);
	if (ret == 0) {
		platform_set_drvdata(pdev, card);
		return 0;
	}
fail:
	snd_card_free(card);
	return ret;
}

static int snd_drv_vaudio_remove(struct platform_device *pdev)
{
	struct snd_dev_card_info *info;
	struct snd_card *card = platform_get_drvdata(pdev);
	info = card->private_data;
	LOG0("Removing Card %d", info->index);
	snd_card_free(card);
	return 0;
}

static struct platform_driver snd_drv_vaudio_info = {
	.probe		= snd_drv_vaudio_probe,
	.remove		= snd_drv_vaudio_remove,
	.driver		= {
		.name	= VAUDIO_DRIVER_NAME,
	},
};

static void snd_drv_vaudio_cleanup(struct xen_drv_vaudio_info *drv_info)
{
	int i;

	LOG0("Cleaning audio driver");
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		struct platform_device *snd_drv_dev;

		LOG0("Removing audio card %d", i);
		snd_drv_dev = drv_info->snd_drv_dev[i];
		if (snd_drv_dev)
			platform_device_unregister(snd_drv_dev);
	}
	LOG0("Removing audio driver");
	platform_driver_unregister(&snd_drv_vaudio_info);
	LOG0("Audio driver cleanup complete");
}

static int snd_drv_vaudio_init(struct xen_drv_vaudio_info *drv_info)
{
	char *cur_card_config;
	int i, num_cards, ret;

	LOG0();
	ret = platform_driver_register(&snd_drv_vaudio_info);
	if (ret < 0)
		return ret;

	LOG0("platform_driver_register ok");
	/* XXX: test code - start */
	num_cards = 2;
	/* XXX: test code - stop */
	drv_info->snd_drv_dev = devm_kzalloc(&drv_info->xen_bus_dev->dev,
			sizeof(drv_info->snd_drv_dev[0]) * num_cards, GFP_KERNEL);
	if (!drv_info->snd_drv_dev)
		goto fail;
	drv_info->cfg_num_cards = num_cards;
	cur_card_config = (char *)drv_info->cfg_cards;
	for (i = 0; i < num_cards; i++) {
		struct platform_device *snd_drv_dev;
		struct snd_dev_card_platdata snd_dev_platdata;

		LOG0("Adding card %d", i);
		/* pass card configuration via platform data */
		memset(&snd_dev_platdata, 0, sizeof(snd_dev_platdata));
		snd_dev_platdata.xen_drv_info = drv_info;
		snd_dev_platdata.index = i;
		snd_dev_platdata.card_config =
				(struct vaudioif_card_config *)cur_card_config;
		snd_drv_dev = platform_device_register_data(NULL, VAUDIO_DRIVER_NAME,
				i, &snd_dev_platdata, sizeof(snd_dev_platdata));
		if (IS_ERR(snd_drv_dev))
			goto fail;
		drv_info->snd_drv_dev[i] = snd_drv_dev;
		/* advance pointer to the next card configuration */
		cur_card_config += sizeof(struct vaudioif_card_config) +
				snd_dev_platdata.card_config->num_pcm_instances *
				sizeof(struct vaudioif_pcm_instance_config);
	}
	LOG0("Added %d cards", num_cards);
	return 0;

fail:
	LOG0("Failed to register audio driver");
	snd_drv_vaudio_cleanup(drv_info);
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

static int xen_drv_vaudio_probe(struct xenbus_device *xen_bus_dev,
				const struct xenbus_device_id *id)
{
	struct xen_drv_vaudio_info *drv_info;
	int ret;

	LOG0();
	drv_info = devm_kzalloc(&xen_bus_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		ret = -ENOMEM;
		goto fail;
	}
	dev_set_drvdata(&xen_bus_dev->dev, drv_info);
	drv_info->xen_bus_dev = xen_bus_dev;

	/* FIXME: remove me */
	ret = DBG_vaudio_run(xen_bus_dev, drv_info);
	if (ret < 0)
		goto fail;
	/* FIXME: remove me */
	return 0;
fail:
	xenbus_dev_fatal(xen_bus_dev, ret, "allocating device memory");
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

static int xen_drv_vaudio_create(struct xenbus_device *xbdev,
		struct xen_drv_vaudio_info *info)
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
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");

