/*
 *  Xen para-virtual sound device
 *  Copyright (c) 2016, Oleksandr Andrushchenko
 *
 *  Based on sound/drivers/dummy.c
 *  Based on drivers/net/xen-netfront.c
 *  Based on drivers/block/xen-blkfront.c
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
#include <linux/irq.h>

#include <sound/core.h>
#include <sound/pcm.h>

#include <asm/xen/hypervisor.h>
#include <xen/xen.h>
#include <xen/platform_pci.h>
#include <xen/xenbus.h>
#include <xen/events.h>
#include <xen/grant_table.h>
#include <xen/interface/io/ring.h>

#include "xen-sndif/xen/include/public/io/sndif_linux.h"

#define GRANT_INVALID_REF	0
/* timeout in ms to wait for backend to respond */
#define VSND_WAIT_BACK_MS	5000

#ifdef SILENT
#define LOG(log_level, fmt, ...)
#else
#define LOG(log_level, fmt, ...) \
		do { \
			printk(XENSND_DRIVER_NAME #log_level " (%s:%d): " fmt "\n", \
					__FUNCTION__, __LINE__ , ## __VA_ARGS__); \
		} while (0)
#endif

#define LOG0(fmt, ...) do if (debug_level >= 0) LOG(0, fmt, ## __VA_ARGS__); while (0)

int debug_level;

enum vsndif_state {
	VSNDIF_STATE_DISCONNECTED,
	VSNDIF_STATE_CONNECTED,
	VSNDIF_STATE_SUSPENDED,
};

struct xen_drv_vsnd_info;

struct xen_vsndif_event_channel {
	struct xen_drv_vsnd_info *drv_info;
	struct xen_sndif_front_ring ring;
	int ring_ref;
	unsigned int port;
	unsigned int irq;
	struct completion completion;
	/* state of the event channel */
	enum vsndif_state state;
	/* latest response status */
	int resp_status;
};

struct xen_drv_vsnd_info {
	struct xenbus_device *xen_bus_dev;
	spinlock_t io_lock;
	struct mutex mutex;
	bool snd_drv_registered;
	/* array of virtual sound platform devices */
	struct platform_device **snd_drv_dev;

	int num_evt_channels;
	struct xen_vsndif_event_channel *evt_channel;

	/* number of virtual cards */
	int cfg_num_cards;
	struct snd_dev_card_platdata *cfg_plat_data;
};

struct snd_dev_pcm_stream_info {
	int index;
	struct snd_pcm_hardware pcm_hw;
	struct xen_vsndif_event_channel evt_channel;
};

struct vsndif_stream_config {
	int index;
	char *xenstore_path;
	struct snd_pcm_hardware pcm_hw;
};

struct vsndif_pcm_instance_config {
	char name[80];
	/* device number */
	int device;
	/* Device's PCM hardware descriptor */
	struct snd_pcm_hardware pcm_hw;
	int  num_streams_pb;
	struct vsndif_stream_config *streams_pb;
	int  num_streams_cap;
	struct vsndif_stream_config *streams_cap;
};

struct vsndif_card_config {
	/* card configuration */
	char shortname[32];
	char longname[80];
	/* number of PCM instances in this configuration */
	int num_devices;
	/* Card's PCM hardware descriptor */
	struct snd_pcm_hardware pcm_hw;

	/* pcm instance configurations */
	struct vsndif_pcm_instance_config *pcm_instances;
};

struct snd_dev_card_platdata {
	int index;
	struct xen_drv_vsnd_info *xen_drv_info;
	struct vsndif_card_config card_config;
};

struct snd_dev_card_info {
	struct xen_drv_vsnd_info *xen_drv_info;
	struct snd_card *card;
	struct snd_pcm_hardware pcm_hw;
	/* array of PCM instances of this card */
	int num_pcm_instances;
	struct snd_dev_pcm_instance_info *pcm_instance;
};

struct snd_dev_pcm_instance_info {
	struct snd_dev_card_info *card_info;
	struct snd_pcm *pcm;
	int num_pcm_streams_pb;
	struct snd_dev_pcm_stream_info *streams_pb;
	int num_pcm_streams_cap;
	struct snd_dev_pcm_stream_info *streams_cap;
};

/*
 * Sound driver start
 */

int snd_drv_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_dev_pcm_instance_info *pcm_instance =
			snd_pcm_substream_chip(substream);
	struct snd_dev_pcm_stream_info *stream_info;
	struct snd_pcm_runtime *runtime = substream->runtime;

	LOG0("TODO: mutex_lock/unlock: Substream is %s direction %d number %d", substream->name,
			substream->stream, substream->number);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream_info = &pcm_instance->streams_pb[substream->number];
	else
		stream_info = &pcm_instance->streams_cap[substream->number];

	runtime->hw = pcm_instance->card_info->pcm_hw;
	runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
			      SNDRV_PCM_INFO_MMAP_VALID |
			      SNDRV_PCM_INFO_DOUBLE |
			      SNDRV_PCM_INFO_BATCH |
			      SNDRV_PCM_INFO_NONINTERLEAVED |
			      SNDRV_PCM_INFO_RESUME |
			      SNDRV_PCM_INFO_PAUSE);
	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;

	LOG0("stream_idx == %d", stream_info->index);
	return 0;
}

int snd_drv_pcm_close(struct snd_pcm_substream *substream)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_hw_params(struct snd_pcm_substream *substream,
		 struct snd_pcm_hw_params *params)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_hw_free(struct snd_pcm_substream *substream)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_prepare(struct snd_pcm_substream *substream)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

snd_pcm_uframes_t snd_drv_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

snd_pcm_uframes_t snd_drv_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	return 0;
}

int snd_drv_pcm_playback_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
	return 0;
}

int snd_drv_pcm_capture_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
	return 0;
}

int snd_drv_pcm_playback_silence(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	LOG0("TODO: mutex_lock/unlock: Substream is %s channel %d pos %lu count %lu", substream->name, channel, pos, count);
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

static int snd_drv_vsnd_new_pcm(struct snd_dev_card_info *card_info,
		struct vsndif_pcm_instance_config *instance_config,
		struct snd_dev_pcm_instance_info *pcm_instance_info)
{
	struct snd_pcm *pcm;
	int ret;

	LOG0("Device \"%s\" with id %d playback %d capture %d",
			instance_config->name,
			instance_config->device,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap);
	pcm_instance_info->card_info = card_info;
	/* allocate info for playback streams if any */
	if (instance_config->num_streams_pb) {
		pcm_instance_info->streams_pb = devm_kzalloc(&card_info->card->card_dev,
			instance_config->num_streams_pb *
			sizeof(struct snd_dev_pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_pb)
			return -ENOMEM;
	}
	/* allocate info for capture streams if any */
	if (instance_config->num_streams_cap) {
		pcm_instance_info->streams_cap = devm_kzalloc(&card_info->card->card_dev,
			instance_config->num_streams_cap *
			sizeof(struct snd_dev_pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_cap)
			return -ENOMEM;
	}
	pcm_instance_info->num_pcm_streams_pb = instance_config->num_streams_pb;
	pcm_instance_info->num_pcm_streams_cap = instance_config->num_streams_cap;
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
	pcm->private_data = pcm_instance_info;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Virtual card PCM");
	pcm_instance_info->pcm = pcm;
	return 0;
}

static int snd_drv_vsnd_probe(struct platform_device *pdev)
{
	struct snd_dev_card_info *card_info;
	struct snd_dev_card_platdata *platdata;
	struct snd_card *card;
	struct snd_pcm_hardware *card_pcm_hw;
	char card_id[sizeof(card->id)];
	int ret, i;

	platdata = dev_get_platdata(&pdev->dev);
	LOG0("Creating virtual sound card %d", platdata->index);
	LOG0("Will configure %d playback/capture streams",
			platdata->card_config.num_devices);

	snprintf(card_id, sizeof(card->id), XENSND_DRIVER_NAME "%d",
			platdata->index);
	ret = snd_card_new(&pdev->dev, platdata->index, card_id, THIS_MODULE,
			sizeof(struct snd_dev_card_info), &card);
	if (ret < 0)
		return ret;
	/* card_info is allocated and maintained by snd_card_new */
	card_info = card->private_data;
	card_info->xen_drv_info = platdata->xen_drv_info;
	card_info->card = card;
	card_info->pcm_instance = devm_kzalloc(&pdev->dev,
			platdata->card_config.num_devices *
			sizeof(struct snd_dev_pcm_instance_info), GFP_KERNEL);
	if (!card_info->pcm_instance)
		goto fail;
	card_info->num_pcm_instances = platdata->card_config.num_devices;

	for (i = 0; i < platdata->card_config.num_devices; i++) {
		ret = snd_drv_vsnd_new_pcm(card_info,
				&platdata->card_config.pcm_instances[i],
				&card_info->pcm_instance[i]);
		if (ret < 0)
			goto fail;
	}
	card_info->pcm_hw = snd_drv_pcm_hardware;
	card_pcm_hw = &platdata->card_config.pcm_hw;
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
	strncpy(card->driver, XENSND_DRIVER_NAME, sizeof(card->driver));
	strncpy(card->shortname, platdata->card_config.shortname,
			sizeof(card->shortname));
	strncpy(card->longname, platdata->card_config.longname,
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

static int snd_drv_vsnd_remove(struct platform_device *pdev)
{
	struct snd_dev_card_info *info;
	struct snd_card *card = platform_get_drvdata(pdev);
	info = card->private_data;
	LOG0("Removing Card %d", info->card->number);
	snd_card_free(card);
	return 0;
}

static struct platform_driver snd_drv_vsnd_info = {
	.probe		= snd_drv_vsnd_probe,
	.remove		= snd_drv_vsnd_remove,
	.driver		= {
		.name	= XENSND_DRIVER_NAME,
	},
};

static void snd_drv_vsnd_cleanup(struct xen_drv_vsnd_info *drv_info)
{
	int i;

	if (!drv_info->snd_drv_registered)
		return;
	LOG0("Cleaning sound driver");
	if (drv_info->snd_drv_dev) {
		for (i = 0; i < drv_info->cfg_num_cards; i++) {
			struct platform_device *snd_drv_dev;

			LOG0("Removing sound card %d", i);
			snd_drv_dev = drv_info->snd_drv_dev[i];
			if (snd_drv_dev)
				platform_device_unregister(snd_drv_dev);
		}
	}
	LOG0("Removing sound driver");
	platform_driver_unregister(&snd_drv_vsnd_info);
	drv_info->snd_drv_registered = false;
	LOG0("Sound driver cleanup complete");
}

static int snd_drv_vsnd_init(struct xen_drv_vsnd_info *drv_info)
{
	int i, num_cards, ret;

	LOG0();
	ret = platform_driver_register(&snd_drv_vsnd_info);
	if (ret < 0)
		return ret;
	drv_info->snd_drv_registered = true;

	num_cards = drv_info->cfg_num_cards;
	drv_info->snd_drv_dev = devm_kzalloc(&drv_info->xen_bus_dev->dev,
			sizeof(drv_info->snd_drv_dev[0]) * num_cards, GFP_KERNEL);
	if (!drv_info->snd_drv_dev)
		goto fail;
	for (i = 0; i < num_cards; i++) {
		struct platform_device *snd_drv_dev;
		struct snd_dev_card_platdata *snd_dev_platdata;

		snd_dev_platdata = &drv_info->cfg_plat_data[i];
		LOG0("Adding card %d", i);
		/* pass card configuration via platform data */
		snd_drv_dev = platform_device_register_data(NULL, XENSND_DRIVER_NAME,
				snd_dev_platdata->index, snd_dev_platdata,
				sizeof(*snd_dev_platdata));
		drv_info->snd_drv_dev[i] = snd_drv_dev;
		if (IS_ERR(snd_drv_dev)) {
			drv_info->snd_drv_dev[i] = NULL;
			goto fail;
		}
	}
	LOG0("Added %d cards", num_cards);
	return 0;

fail:
	LOG0("Failed to register sound driver");
	snd_drv_vsnd_cleanup(drv_info);
	return -ENODEV;
}

/*
 * Sound driver stop
 */

/*
 * Xen driver start
 */

static int xen_drv_talk_to_soundback(struct xen_drv_vsnd_info *drv_info);
static int xen_drv_vsnd_on_backend_connected(struct xen_drv_vsnd_info *drv_info);
static void xen_drv_vsnd_on_backend_disconnected(struct xen_drv_vsnd_info *drv_info);

static int xen_drv_vsnd_remove(struct xenbus_device *dev);
static void xen_drv_vsnd_ring_free_all(struct xen_drv_vsnd_info *drv_info);

static int xen_drv_vsnd_probe(struct xenbus_device *xen_bus_dev,
				const struct xenbus_device_id *id)
{
	struct xen_drv_vsnd_info *drv_info;
	int ret;

	LOG0();
	drv_info = devm_kzalloc(&xen_bus_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		ret = -ENOMEM;
		goto fail;
	}

	/* FIXME: this is for insmod after rmmod
	 * after removing the driver it remains in XenbusStateClosed state
	 * but I would expect XenbusStateInitialising or XenbusStateUnknown */
	xenbus_switch_state(xen_bus_dev, XenbusStateInitialising);

	drv_info->xen_bus_dev = xen_bus_dev;
	spin_lock_init(&drv_info->io_lock);
	mutex_init(&drv_info->mutex);
	drv_info->snd_drv_registered = false;
	dev_set_drvdata(&xen_bus_dev->dev, drv_info);
	return 0;
fail:
	xenbus_dev_fatal(xen_bus_dev, ret, "allocating device memory");
	return ret;
}

static void xen_drv_vsnd_remove_internal(struct xen_drv_vsnd_info *drv_info)
{
	snd_drv_vsnd_cleanup(drv_info);
	xen_drv_vsnd_ring_free_all(drv_info);
}

static int xen_drv_vsnd_remove(struct xenbus_device *dev)
{
	struct xen_drv_vsnd_info *drv_info = dev_get_drvdata(&dev->dev);

	mutex_lock(&drv_info->mutex);
	xen_drv_vsnd_remove_internal(drv_info);
	mutex_unlock(&drv_info->mutex);
	return 0;
}

static int xen_drv_vsnd_resume(struct xenbus_device *dev)
{
	LOG0();
	return 0;
}

static void xen_drv_vsnd_backend_changed(struct xenbus_device *xen_bus_dev,
				enum xenbus_state backend_state)
{
	struct xen_drv_vsnd_info *drv_info = dev_get_drvdata(&xen_bus_dev->dev);
	int ret;

	LOG0("Backend state is %s, front is %s", xenbus_strstate(backend_state),
			xenbus_strstate(xen_bus_dev->state));
	switch (backend_state) {
	case XenbusStateReconfiguring:
		/* fall through */
	case XenbusStateReconfigured:
		/* fall through */
	case XenbusStateInitialising:
		/* fall through */
	case XenbusStateInitialised:
		break;

	case XenbusStateInitWait:
		if (xen_bus_dev->state != XenbusStateInitialising)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xen_drv_talk_to_soundback(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0)
			break;
		xenbus_switch_state(xen_bus_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xen_bus_dev->state != XenbusStateInitialised)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xen_drv_vsnd_on_backend_connected(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0)
			break;
		LOG0("Sound initialized");
		xenbus_switch_state(xen_bus_dev, XenbusStateConnected);
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xen_bus_dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		/* FIXME: is this check needed? */
		if (xen_bus_dev->state == XenbusStateClosing)
			break;
		mutex_lock(&drv_info->mutex);
		xen_drv_vsnd_on_backend_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xen_bus_dev, XenbusStateClosed);
		break;
	}
}

static void xen_drv_vsnd_stream_ring_free(struct xen_drv_vsnd_info *drv_info,
		struct xen_vsndif_event_channel *channel)
{
	LOG0("Cleaning up ring-ref %u at port %u",
			channel->ring_ref, channel->port);
	if (!channel->ring.sring)
		return;
	channel->state = VSNDIF_STATE_DISCONNECTED;
	/* release all who still waits for response if any */
	channel->resp_status = -XENSND_RSP_ERROR;
	complete_all(&channel->completion);
	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);
	channel->irq = 0;
	if (channel->port)
		xenbus_free_evtchn(drv_info->xen_bus_dev, channel->port);
	channel->port = 0;
	/* End access and free the pages */
	if (channel->ring_ref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->ring_ref, 0,
				(unsigned long)channel->ring.sring);
	channel->ring.sring = NULL;
}


static void xen_drv_vsnd_ring_free_all(struct xen_drv_vsnd_info *drv_info)
{
	int i;

	LOG0("Cleaning up event channels for streams");
	if (!drv_info->evt_channel)
		return;
	for (i = 0; i < drv_info->num_evt_channels; i++)
		xen_drv_vsnd_stream_ring_free(drv_info,
				&drv_info->evt_channel[i]);
	devm_kfree(&drv_info->xen_bus_dev->dev, drv_info->evt_channel);
	drv_info->evt_channel = NULL;
}

static irqreturn_t xen_drv_vsnd_stream_ring_interrupt(int irq, void *dev_id)
{
	struct xen_vsndif_event_channel *channel = dev_id;
	struct xen_drv_vsnd_info *drv_info = channel->drv_info;
	struct xensnd_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != VSNDIF_STATE_CONNECTED))
		goto out;

 again:
	rp = channel->ring.sring->rsp_prod;
	rmb(); /* Ensure we see queued responses up to 'rp'. */

	for (i = channel->ring.rsp_cons; i != rp; i++) {
		resp = (struct xensnd_resp *)RING_GET_RESPONSE(&channel->ring, i);
		switch (resp->u.data.operation) {
		case XENSND_OP_OPEN:
			LOG0("Got response on XENSND_OP_OPEN");
			channel->resp_status = resp->u.data.status;
			complete(&channel->completion);
			break;
		default:
			BUG();
		}
	}

	channel->ring.rsp_cons = i;

	if (i != channel->ring.req_prod_pvt) {
		int more_to_do;
		RING_FINAL_CHECK_FOR_RESPONSES(&channel->ring, more_to_do);
		if (more_to_do)
			goto again;
	} else
		channel->ring.sring->rsp_event = i + 1;

out:
	spin_unlock_irqrestore(&drv_info->io_lock, flags);
	return IRQ_HANDLED;
}

static int xen_drv_vsnd_stream_ring_alloc(struct xen_drv_vsnd_info *drv_info,
		struct xen_vsndif_event_channel *evt_channel, int stream_idx)
{
	struct xenbus_device *xen_bus_dev = drv_info->xen_bus_dev;
	struct xen_sndif_sring *sring;
	grant_ref_t gref;
	int ret;

	evt_channel->drv_info = drv_info;
	init_completion(&evt_channel->completion);
	evt_channel->state = VSNDIF_STATE_DISCONNECTED;
	evt_channel->ring_ref = GRANT_INVALID_REF;
	evt_channel->ring.sring = NULL;
	evt_channel->port = -1;
	evt_channel->irq = -1;
	sring = (struct xen_sndif_sring *)get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!sring) {
		ret = -ENOMEM;
		goto fail;
	}
	SHARED_RING_INIT(sring);
	FRONT_RING_INIT(&evt_channel->ring, sring, PAGE_SIZE);

	ret = xenbus_grant_ring(xen_bus_dev, sring, 1, &gref);
	if (ret < 0)
		goto fail;
	evt_channel->ring_ref = gref;

	ret = xenbus_alloc_evtchn(xen_bus_dev, &evt_channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(evt_channel->port,
			xen_drv_vsnd_stream_ring_interrupt,
			0, xen_bus_dev->devicetype, evt_channel);

	if (ret < 0)
		goto fail;
	evt_channel->irq = ret;
	return 0;

fail:
	LOG0("Failed to allocate ring with err %d", ret);
	return ret;
}

static int xen_drv_vsnd_stream_ring_create(struct xen_drv_vsnd_info *drv_info,
		struct xen_vsndif_event_channel *evt_channel, int stream_idx,
		const char *path)
{
	const char *message;
	int ret;

	LOG0("Allocating and opening event channel for stream %d", stream_idx);
	/* allocate and open control channel */
	ret = xen_drv_vsnd_stream_ring_alloc(drv_info, evt_channel, stream_idx);
	if (ret < 0) {
		message = "allocating event channel";
		goto fail;
	}
	/* Write control channel ring reference */
	ret = xenbus_printf(XBT_NIL, path, XENSND_FIELD_RING_REF, "%u",
			evt_channel->ring_ref);
	if (ret < 0) {
		message = "writing " XENSND_FIELD_RING_REF;
		goto fail;
	}

	ret = xenbus_printf(XBT_NIL, path, XENSND_FIELD_EVT_CHNL, "%u",
			evt_channel->port);
	if (ret < 0) {
		message = "writing " XENSND_FIELD_EVT_CHNL;
		goto fail;
	}
	LOG0("Allocated and opened control event channel: %s "
			XENSND_FIELD_RING_REF " %u "
			XENSND_FIELD_EVT_CHNL " %u ",
			path, evt_channel->ring_ref, evt_channel->port);
	return 0;

fail:
	LOG0("Error %s with err %d", message, ret);
	return ret;
}

static inline void xen_drv_vsnd_stream_ring_flush(
		struct xen_vsndif_event_channel *channel)
{
	int notify;

	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->ring, notify);

	if (notify)
		notify_remote_via_irq(channel->irq);
}

/* get number of nodes under the path to get number of
 * cards configured or number of devices within the card
 */
static char **xen_drv_get_num_nodes(const char *path, const char *node,
		int *num_entries)
{
	char **result;

	result = xenbus_directory(XBT_NIL, path, node, num_entries);
	if (IS_ERR(result)) {
		*num_entries = 0;
		return NULL;
	}
	return result;
}

struct PCM_HW_SAMPLE_RATE {
	const char *name;
	unsigned int mask;
};
static struct PCM_HW_SAMPLE_RATE xen_drv_pcm_hw_supported_rates[] = {
	{ .name = "5512",   .mask = SNDRV_PCM_RATE_5512 },
	{ .name = "8000",   .mask = SNDRV_PCM_RATE_8000 },
	{ .name = "11025",  .mask = SNDRV_PCM_RATE_11025 },
	{ .name = "16000",  .mask = SNDRV_PCM_RATE_16000 },
	{ .name = "22050",  .mask = SNDRV_PCM_RATE_22050 },
	{ .name = "32000",  .mask = SNDRV_PCM_RATE_32000 },
	{ .name = "44100",  .mask = SNDRV_PCM_RATE_44100 },
	{ .name = "48000",  .mask = SNDRV_PCM_RATE_48000 },
	{ .name = "64000",  .mask = SNDRV_PCM_RATE_64000 },
	{ .name = "96000",  .mask = SNDRV_PCM_RATE_96000 },
	{ .name = "176400", .mask = SNDRV_PCM_RATE_176400 },
	{ .name = "192000", .mask = SNDRV_PCM_RATE_192000 },
};

static void xen_drv_pcm_hw_parse_rates(char *list, unsigned int len,
		const char *path, struct snd_pcm_hardware *pcm_hw)
{
	char *cur_rate;
	unsigned int cur_mask;
	unsigned int rates;
	unsigned int rate_min;
	unsigned int rate_max;
	int i;

	cur_rate = NULL;
	rates = 0;
	rate_min = -1;
	rate_max = 0;
	while ((cur_rate = strsep(&list, XENSND_LIST_SEPARATOR))) {
		for (i = 0; i < ARRAY_SIZE(xen_drv_pcm_hw_supported_rates); i++)
			if (!strncasecmp(cur_rate, xen_drv_pcm_hw_supported_rates[i].name,
					XENSND_SAMPLE_RATE_MAX_LEN)) {
				cur_mask =xen_drv_pcm_hw_supported_rates[i].mask;
				rates |= cur_mask;
				if (rate_min > cur_mask)
					rate_min = cur_mask;
				if (rate_max < cur_mask)
					rate_max = cur_mask;
				LOG0("Found rate: %s", cur_rate);
			}
	}
	if (rates) {
		pcm_hw->rates = rates;
		pcm_hw->rate_min = rate_min;
		pcm_hw->rate_max = rate_max;
	}
}

struct PCM_HW_SAMPLE_FORMAT {
	const char * name;
	u64 mask;
};
static struct PCM_HW_SAMPLE_FORMAT xen_drv_pcm_hw_supported_formats[] = {
	{ .name = XENSND_SAMPLE_FMT_U8,                 .mask = SNDRV_PCM_FMTBIT_U8 },
	{ .name = XENSND_SAMPLE_FMT_S8,                 .mask = SNDRV_PCM_FMTBIT_S8 },
	{ .name = XENSND_SAMPLE_FMT_U16_LE,             .mask = SNDRV_PCM_FMTBIT_U16_LE },
	{ .name = XENSND_SAMPLE_FMT_U16_BE,             .mask = SNDRV_PCM_FMTBIT_U16_BE },
	{ .name = XENSND_SAMPLE_FMT_S16_LE,             .mask = SNDRV_PCM_FMTBIT_S16_LE },
	{ .name = XENSND_SAMPLE_FMT_S16_BE,             .mask = SNDRV_PCM_FMTBIT_S16_BE },
	{ .name = XENSND_SAMPLE_FMT_U24_LE,             .mask = SNDRV_PCM_FMTBIT_U24_LE },
	{ .name = XENSND_SAMPLE_FMT_U24_BE,             .mask = SNDRV_PCM_FMTBIT_U24_BE },
	{ .name = XENSND_SAMPLE_FMT_S24_LE,             .mask = SNDRV_PCM_FMTBIT_S24_LE },
	{ .name = XENSND_SAMPLE_FMT_S24_BE,             .mask = SNDRV_PCM_FMTBIT_S24_BE },
	{ .name = XENSND_SAMPLE_FMT_U32_LE,             .mask = SNDRV_PCM_FMTBIT_U32_LE },
	{ .name = XENSND_SAMPLE_FMT_U32_BE,             .mask = SNDRV_PCM_FMTBIT_U32_BE },
	{ .name = XENSND_SAMPLE_FMT_S32_LE,             .mask = SNDRV_PCM_FMTBIT_S32_LE },
	{ .name = XENSND_SAMPLE_FMT_S32_BE,             .mask = SNDRV_PCM_FMTBIT_S32_BE },
	{ .name = XENSND_SAMPLE_FMT_A_LAW,              .mask = SNDRV_PCM_FMTBIT_A_LAW },
	{ .name = XENSND_SAMPLE_FMT_MU_LAW,             .mask = SNDRV_PCM_FMTBIT_MU_LAW },
	{ .name = XENSND_SAMPLE_FMT_FLOAT_LE,           .mask = SNDRV_PCM_FMTBIT_FLOAT_LE },
	{ .name = XENSND_SAMPLE_FMT_FLOAT_BE,           .mask = SNDRV_PCM_FMTBIT_FLOAT_BE },
	{ .name = XENSND_SAMPLE_FMT_FLOAT64_LE,         .mask = SNDRV_PCM_FMTBIT_FLOAT64_LE },
	{ .name = XENSND_SAMPLE_FMT_FLOAT64_BE,         .mask = SNDRV_PCM_FMTBIT_FLOAT64_BE },
	{ .name = XENSND_SAMPLE_FMT_IEC958_SUBFRAME_LE, .mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE },
	{ .name = XENSND_SAMPLE_FMT_IEC958_SUBFRAME_BE, .mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_BE },
	{ .name = XENSND_SAMPLE_FMT_IMA_ADPCM,          .mask = SNDRV_PCM_FMTBIT_IMA_ADPCM },
	{ .name = XENSND_SAMPLE_FMT_MPEG,               .mask = SNDRV_PCM_FMTBIT_MPEG },
	{ .name = XENSND_SAMPLE_FMT_GSM,                .mask = SNDRV_PCM_FMTBIT_GSM },
};

static void xen_drv_pcm_hw_parse_formats(char *list, unsigned int len,
		const char *path, struct snd_pcm_hardware *pcm_hw)
{
	u64 formats;
	char *cur_format;
	int i;

	cur_format = NULL;
	formats = 0;
	while ((cur_format = strsep(&list, XENSND_LIST_SEPARATOR))) {
		for (i = 0; i < ARRAY_SIZE(xen_drv_pcm_hw_supported_formats); i++)
			if (!strncasecmp(cur_format,
				xen_drv_pcm_hw_supported_formats[i].name,
				XENSND_SAMPLE_FORMAT_MAX_LEN)) {
					formats |= xen_drv_pcm_hw_supported_formats[i].mask;
					LOG0("Found format: %s", cur_format);
			}
	}
	if (formats)
		pcm_hw->formats = formats;
}

static void xen_drv_read_pcm_hw_config(const char *path,
		struct snd_pcm_hardware *parent_pcm_hw,
		struct snd_pcm_hardware *pcm_hw)
{
	char *list;
	int val;
	unsigned int len;

	*pcm_hw = *parent_pcm_hw;
	if (xenbus_scanf(XBT_NIL, path, XENSND_FIELD_CHANNELS_MIN,
			"%d", &val) < 0)
		val = 0;
	if (val) {
		pcm_hw->channels_min = val;
		LOG0("pcm_hw->channels_min %d", val);
	}
	if (xenbus_scanf(XBT_NIL, path, XENSND_FIELD_CHANNELS_MAX,
			"%d", &val) < 0)
		val = 0;
	if (val) {
		pcm_hw->channels_max = val;
		LOG0("pcm_hw->channels_max %d", val);
	}
	list = xenbus_read(XBT_NIL, path, XENSND_FIELD_SAMPLE_RATES, &len);
	if (!IS_ERR(list)) {
		xen_drv_pcm_hw_parse_rates(list, len, path, pcm_hw);
		LOG0("Sample rates: \"%s\"", list);
		kfree(list);
	}
	list = xenbus_read(XBT_NIL, path, XENSND_FIELD_SAMPLE_FORMATS, &len);
	if (!IS_ERR(list)) {
		xen_drv_pcm_hw_parse_formats(list, len, path, pcm_hw);
		LOG0("Sample formats: \"%s\"", list);
		kfree(list);
	}
}

static void xen_drv_read_card_config_common(const char *path,
		struct vsndif_card_config *card_config)
{
	char *str;

	str = xenbus_read(XBT_NIL, path, XENSND_FIELD_CARD_SHORT_NAME, NULL);
	if (!IS_ERR(str)) {
		strncpy(card_config->shortname, str,
				sizeof(card_config->shortname));
		kfree(str);
	}
	LOG0("Card short name is \"%s\"", card_config->shortname);
	str = xenbus_read(XBT_NIL, path, XENSND_FIELD_CARD_LONG_NAME, NULL);
	if (!IS_ERR(str)) {
		strncpy(card_config->longname, str,
				sizeof(card_config->longname));
		kfree(str);
	}
	LOG0("Card long name is \"%s\"", card_config->longname);
	/* check if PCM HW configuration exists for this card and update if so */
	xen_drv_read_pcm_hw_config(path, &snd_drv_pcm_hardware,
			&card_config->pcm_hw);
}

static int xen_drv_read_card_config_get_stream_type(const char *path, int index,
		int *num_pb, int *num_cap)
{
	int ret;
	char *str = NULL;
	char *stream_path;

	*num_pb = 0;
	*num_cap = 0;
	stream_path = kasprintf(GFP_KERNEL, "%s/%s/%d", path, XENSND_PATH_STREAM, index);
	if (!stream_path) {
		ret = -ENOMEM;
		goto fail;
	}
	str = xenbus_read(XBT_NIL, stream_path, XENSND_FIELD_TYPE, NULL);
	if (IS_ERR(str)) {
		LOG0("Cannot find stream type at %s/%s", stream_path, XENSND_FIELD_TYPE);
		ret = -EINVAL;
		goto fail;
	}
	if (!strncasecmp(str, XENSND_STREAM_TYPE_PLAYBACK,
			sizeof(XENSND_STREAM_TYPE_PLAYBACK)))
		(*num_pb)++;
	else if (!strncasecmp(str, XENSND_STREAM_TYPE_CAPTURE,
			sizeof(XENSND_STREAM_TYPE_CAPTURE)))
		(*num_cap)++;
	else {
		ret = EINVAL;
		goto fail;
	}
	ret = 0;
fail:
	if (stream_path)
		kfree(stream_path);
	if (str)
		kfree(str);
	return -ret;
}

static int xen_drv_read_card_config_stream(struct xen_drv_vsnd_info *drv_info,
		struct vsndif_pcm_instance_config *pcm_instance,
		const char *path, int index, int *cur_pb, int *cur_cap,
		int *stream_idx)
{
	int ret;
	char *str = NULL;
	char *stream_path;
	struct vsndif_stream_config *stream;

	stream_path = devm_kasprintf(&drv_info->xen_bus_dev->dev,
			GFP_KERNEL, "%s/%s/%d", path, XENSND_PATH_STREAM, index);
	if (!stream_path) {
		ret = -ENOMEM;
		goto fail;
	}
	str = xenbus_read(XBT_NIL, stream_path, XENSND_FIELD_TYPE, NULL);
	if (IS_ERR(str)) {
		LOG0("Cannot find stream type at %s/%s", stream_path,
				XENSND_FIELD_TYPE);
		ret = -EINVAL;
		goto fail;
	}
	if (!strncasecmp(str, XENSND_STREAM_TYPE_PLAYBACK,
			sizeof(XENSND_STREAM_TYPE_PLAYBACK))) {
		stream = &pcm_instance->streams_pb[(*cur_pb)++];
	} else if (!strncasecmp(str, XENSND_STREAM_TYPE_CAPTURE,
			sizeof(XENSND_STREAM_TYPE_CAPTURE))) {
		stream = &pcm_instance->streams_cap[(*cur_cap)++];
	}
	else {
		ret = -EINVAL;
		goto fail;
	}
	/* assign and publish next unique stream index */
	stream->index = (*stream_idx)++;
	stream->xenstore_path = stream_path;
	ret = xenbus_printf(XBT_NIL, stream->xenstore_path,
			XENSND_FIELD_STREAM_INDEX, "%d", stream->index);
	if (ret < 0)
		goto fail;
	xen_drv_read_pcm_hw_config(stream->xenstore_path,
			&pcm_instance->pcm_hw, &stream->pcm_hw);
	LOG0("Stream %s type %s index %d", stream->xenstore_path,
			str, stream->index);
	ret = 0;
fail:
	if (str)
		kfree(str);
	return -ret;
}

static int xen_drv_read_card_config_device(struct xen_drv_vsnd_info *drv_info,
		struct vsndif_pcm_instance_config *pcm_instance,
		struct snd_pcm_hardware *parent_pcm_hw,
		const char *path, const char *device_node,
		int *stream_idx)
{
	char **stream_nodes;
	char *str;
	char *device_path;
	int ret, i, num_streams;
	int num_pb, num_cap;
	int cur_pb, cur_cap;

	device_path = kasprintf(GFP_KERNEL, "%s/%s", path, device_node);
	if (!device_path) {
		ret = -ENOMEM;
		goto fail;
	}
	str = xenbus_read(XBT_NIL, device_path, XENSND_FIELD_DEVICE_NAME, NULL);
	if (!IS_ERR(str)) {
		strncpy(pcm_instance->name, str,
				sizeof(pcm_instance->name));
		kfree(str);
	}
	if (!kstrtoint(device_node, 10, &pcm_instance->device)) {
		LOG0("Device id %d", pcm_instance->device);
	} else {
		LOG0("Wrong device id at %s", device_path);
		ret = -EINVAL;
		goto fail;
	}
	/* check if PCM HW configuration exists for this device
	 * and update if so */
	xen_drv_read_pcm_hw_config(device_path, parent_pcm_hw, &pcm_instance->pcm_hw);
	/* read streams */
	stream_nodes = xen_drv_get_num_nodes(device_path, XENSND_PATH_STREAM,
			&num_streams);
	if (stream_nodes)
		kfree(stream_nodes);
	LOG0("Reading %d stream(s)", num_streams);
	pcm_instance->num_streams_pb = 0;
	pcm_instance->num_streams_cap = 0;
	/* get number of playback and capture streams */
	for (i = 0; i < num_streams; i++) {
		ret = xen_drv_read_card_config_get_stream_type(device_path, i,
				&num_pb, &num_cap);
		if (ret < 0)
			goto fail;
		pcm_instance->num_streams_pb += num_pb;
		pcm_instance->num_streams_cap += num_cap;
	}
	if (pcm_instance->num_streams_pb) {
		pcm_instance->streams_pb = devm_kzalloc(&drv_info->xen_bus_dev->dev,
				pcm_instance->num_streams_pb *
				sizeof(struct vsndif_stream_config),
				GFP_KERNEL);
		if (!pcm_instance->streams_pb) {
			ret = -ENOMEM;
			goto fail;
		}
	}
	if (pcm_instance->num_streams_cap) {
		pcm_instance->streams_cap = devm_kzalloc(&drv_info->xen_bus_dev->dev,
				pcm_instance->num_streams_cap *
				sizeof(struct vsndif_stream_config),
				GFP_KERNEL);
		if (!pcm_instance->streams_cap) {
			ret = -ENOMEM;
			goto fail;
		}
	}
	cur_pb = 0;
	cur_cap = 0;
	for (i = 0; i < num_streams; i++) {
		ret = xen_drv_read_card_config_stream(drv_info,
				pcm_instance, device_path, i, &cur_pb, &cur_cap, stream_idx);
		if (ret < 0)
			goto fail;
	}
	ret = 0;
fail:
	if (device_path)
		kfree(device_path);
	return -ret;
}

static int xen_drv_read_card_config(struct xen_drv_vsnd_info *drv_info,
		struct snd_dev_card_platdata *plat_data,
		int *stream_idx)
{
	struct xenbus_device *xen_bus_dev = drv_info->xen_bus_dev;
	char *path;
	char **device_nodes = NULL;
	int ret, num_devices, i;

	path = kasprintf(GFP_KERNEL, "%s/" XENSND_PATH_CARD "/%d",
			xen_bus_dev->nodename, plat_data->index);
	if (!path) {
		ret = -ENOMEM;
		goto fail;
	}
	device_nodes = xen_drv_get_num_nodes(path, XENSND_PATH_DEVICE,
			&num_devices);
	if (!num_devices) {
		LOG0("No devices configured for sound card %d at %s/%s",
				plat_data->index, path, XENSND_PATH_DEVICE);
		ret = -ENODEV;
		goto fail;
	}
	xen_drv_read_card_config_common(path, &plat_data->card_config);
	/* read configuration for devices of this card */
	plat_data->card_config.pcm_instances = devm_kzalloc(
			&drv_info->xen_bus_dev->dev,
			num_devices * sizeof(struct vsndif_pcm_instance_config),
			GFP_KERNEL);
	if (!plat_data->card_config.pcm_instances) {
		ret = -ENOMEM;
		goto fail;
	}
	kfree(path);
	path = kasprintf(GFP_KERNEL,
			"%s/" XENSND_PATH_CARD "/%d/" XENSND_PATH_DEVICE,
			xen_bus_dev->nodename, plat_data->index);
	if (!path) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < num_devices; i++) {
		ret = xen_drv_read_card_config_device(drv_info,
			&plat_data->card_config.pcm_instances[i],
			&plat_data->card_config.pcm_hw,
			path, device_nodes[i], stream_idx);
		if (ret < 0)
			goto fail;
	}
	plat_data->card_config.num_devices = num_devices;
	ret = 0;
fail:
	if (device_nodes)
		kfree(device_nodes);
	if (path)
		kfree(path);
	return ret;
}

static int xen_drv_create_stream_evtchannels(struct xen_drv_vsnd_info *drv_info,
		int num_streams)
{
	int ret, c, d, s, stream_idx;

	drv_info->evt_channel = devm_kzalloc(&drv_info->xen_bus_dev->dev,
			num_streams *
			sizeof(struct xen_vsndif_event_channel), GFP_KERNEL);
	if (!drv_info->evt_channel) {
		ret = -ENOMEM;
		goto fail;
	}
	for (c = 0; c < drv_info->cfg_num_cards; c++) {
		struct snd_dev_card_platdata *plat_data;
		plat_data = &drv_info->cfg_plat_data[c];
		for (d = 0; d < plat_data->card_config.num_devices; d++) {
			struct vsndif_pcm_instance_config *pcm_instance;
			pcm_instance = &plat_data->card_config.pcm_instances[d];
			for (s = 0; s < pcm_instance->num_streams_pb; s++) {
				stream_idx = pcm_instance->streams_pb[s].index;
				ret = xen_drv_vsnd_stream_ring_create(drv_info,
						&drv_info->evt_channel[stream_idx],
						stream_idx,
						pcm_instance->streams_pb[s].xenstore_path);
				if (ret < 0)
					goto fail;
			}
			for (s = 0; s < pcm_instance->num_streams_cap; s++) {
				stream_idx = pcm_instance->streams_cap[s].index;
				ret = xen_drv_vsnd_stream_ring_create(drv_info,
						&drv_info->evt_channel[stream_idx],
						stream_idx,
						pcm_instance->streams_cap[s].xenstore_path);
				if (ret < 0)
					goto fail;
			}
		}
		if (ret < 0)
			goto fail;
	}
	drv_info->num_evt_channels = num_streams;
	return 0;
fail:
	xen_drv_vsnd_ring_free_all(drv_info);
	return ret;
}

static int xen_drv_talk_to_soundback(struct xen_drv_vsnd_info *drv_info)
{
	struct xenbus_device *xen_bus_dev = drv_info->xen_bus_dev;
	char **card_nodes;
	int stream_idx;
	int i, ret;

	LOG0("Reading number of cards to configure");
	card_nodes = xen_drv_get_num_nodes(xen_bus_dev->nodename,
			XENSND_PATH_CARD, &drv_info->cfg_num_cards);
	if (card_nodes)
		kfree(card_nodes);
	if (!drv_info->cfg_num_cards) {
		LOG0("No sound cards configured");
		return 0;
	}
	LOG0("Configuring %d sound cards", drv_info->cfg_num_cards);
	drv_info->cfg_plat_data = devm_kzalloc(&drv_info->xen_bus_dev->dev,
			drv_info->cfg_num_cards *
			sizeof(struct snd_dev_card_platdata), GFP_KERNEL);
	if (!drv_info->cfg_plat_data) {
		ret = -ENOMEM;
		goto fail;
	}
	/* stream index must be unique through all cards: pass it in to be
	 * incremented when creating streams */
	stream_idx = 0;
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		/* read card configuration from the store and
		 * set platform data structure */
		drv_info->cfg_plat_data[i].index = i;
		drv_info->cfg_plat_data[i].xen_drv_info = drv_info;
		ret = xen_drv_read_card_config(drv_info,
				&drv_info->cfg_plat_data[i], &stream_idx);
		if (ret < 0)
			goto fail;
	}
	/* create event channels for all streams and publish */
	ret = xen_drv_create_stream_evtchannels(drv_info, stream_idx);
	if (ret < 0)
		goto fail;
	return snd_drv_vsnd_init(drv_info);
fail:
	return ret;
}

static int xen_drv_vsnd_on_backend_connected(struct xen_drv_vsnd_info *drv_info)
{
	return 0;
}

static void xen_drv_vsnd_on_backend_disconnected(struct xen_drv_vsnd_info *drv_info)
{
	LOG0("Cleaning up on backed disconnection");
	xen_drv_vsnd_remove_internal(drv_info);
}

/*
 * Xen driver stop
 */

static const struct xenbus_device_id xen_drv_vsnd_ids[] = {
	{ XENSND_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xen_vsnd_driver = {
	.ids = xen_drv_vsnd_ids,
	.probe = xen_drv_vsnd_probe,
	.remove = xen_drv_vsnd_remove,
	.resume = xen_drv_vsnd_resume,
	.otherend_changed = xen_drv_vsnd_backend_changed,
};

static int __init xen_drv_vsnd_init(void)
{
	if (!xen_domain())
		return -ENODEV;
	if (xen_initial_domain()) {
		LOG0(XENSND_DRIVER_NAME " cannot run in Dom0");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	LOG0("Registering XEN PV " XENSND_DRIVER_NAME);
	return xenbus_register_frontend(&xen_vsnd_driver);
}

static void __exit xen_drv_vsnd_cleanup(void)
{
	LOG0("Unregistering XEN PV " XENSND_DRIVER_NAME);
	xenbus_unregister_driver(&xen_vsnd_driver);
}

module_init(xen_drv_vsnd_init);
module_exit(xen_drv_vsnd_cleanup);

MODULE_DESCRIPTION("Xen virtual sound device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENSND_DRIVER_NAME);
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");

