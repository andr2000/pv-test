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

#include "vsndif.h"

#define VSND_DRIVER_NAME	"vsnd"

#define GRANT_INVALID_REF	0
/* timeout in ms to wait for backend to respond */
#define VSND_WAIT_BACK_MS	5000

#ifdef SILENT
#define LOG(log_level, fmt, ...)
#else
#define LOG(log_level, fmt, ...) \
		do { \
			printk(VSND_DRIVER_NAME #log_level " (%s:%d): " fmt "\n", \
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

struct xen_vsndif_ctrl_channel {
	struct xen_vsndif_ctrl_front_ring ring;
	int ring_ref;
	unsigned int evt_channel;
	unsigned int irq;
	spinlock_t io_lock;
	struct completion completion;
	/* state of the event channel */
	enum vsndif_state state;
	/* latest response status */
	int resp_status;
};

struct xen_drv_vsnd_info {
	struct xenbus_device *xen_bus_dev;
	/* array of virtual sound platform devices */
	struct platform_device **snd_drv_dev;

	struct xen_vsndif_ctrl_channel ctrl_channel;

	/* XXX: this comes from back to end configuration negotiation */
	/* number of virtual cards */
	int cfg_num_cards;
	/* card configuration */
	struct vsndif_card_config *cfg_cards;
};

struct snd_dev_card_platdata {
	struct xen_drv_vsnd_info *xen_drv_info;
	int index;
	struct vsndif_card_config *card_config;
};

struct snd_dev_pcm_instance_info;

struct snd_dev_card_info {
	struct xen_drv_vsnd_info *xen_drv_info;
	struct snd_card *card;
	struct snd_pcm_hardware pcm_hw;
	/* array of PCM instances of this card */
	int num_pcm_instances;
	struct snd_dev_pcm_instance_info *pcm_instance;
};

struct snd_dev_pcm_stream_info;

struct snd_dev_pcm_instance_info {
	struct snd_dev_card_info *card_info;
	struct snd_pcm *pcm;
	int num_pcm_streams_pb;
	struct snd_dev_pcm_stream_info *streams_pb;
	int num_pcm_streams_cap;
	struct snd_dev_pcm_stream_info *streams_cap;
};

struct snd_dev_pcm_stream_info {
	int dummy;
};

/* XXX: remove me */
#include "dbg_xen_sndfront.c"
/* XXX: remove me */


/*
 * Sound driver start
 */
int snd_drv_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_dev_pcm_instance_info *pcm_instance =
			snd_pcm_substream_chip(substream);
	struct snd_dev_pcm_stream_info *stream_info;
	struct snd_pcm_runtime *runtime = substream->runtime;

	LOG0("Substream is %s direction %d number %d", substream->name,
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

	stream_info->dummy = substream->number;
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
	struct vsndif_card_pcm_hw_config *card_pcm_hw;
	char card_id[sizeof(card->id)];
	int ret, i;

	platdata = dev_get_platdata(&pdev->dev);
	LOG0("Creating virtual sound card %d", platdata->index);
	LOG0("Will configure %d playback/capture streams",
			platdata->card_config->num_pcm_instances);

	snprintf(card_id, sizeof(card->id), VSND_DRIVER_NAME "%d",
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
			platdata->card_config->num_pcm_instances *
			sizeof(struct snd_dev_pcm_instance_info), GFP_KERNEL);
	if (!card_info->pcm_instance)
		goto fail;
	card_info->num_pcm_instances = platdata->card_config->num_pcm_instances;

	for (i = 0; i < platdata->card_config->num_pcm_instances; i++) {
		ret = snd_drv_vsnd_new_pcm(card_info,
				&platdata->card_config->pcm_instance[i],
				&card_info->pcm_instance[i]);
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
	strncpy(card->driver, VSND_DRIVER_NAME, sizeof(card->driver));
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
		.name	= VSND_DRIVER_NAME,
	},
};

static void snd_drv_vsnd_cleanup(struct xen_drv_vsnd_info *drv_info)
{
	int i;

	LOG0("Cleaning sound driver");
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		struct platform_device *snd_drv_dev;

		LOG0("Removing sound card %d", i);
		snd_drv_dev = drv_info->snd_drv_dev[i];
		if (snd_drv_dev)
			platform_device_unregister(snd_drv_dev);
	}
	LOG0("Removing sound driver");
	platform_driver_unregister(&snd_drv_vsnd_info);
	LOG0("Sound driver cleanup complete");
}

static int snd_drv_vsnd_init(struct xen_drv_vsnd_info *drv_info)
{
	char *cur_card_config;
	int i, num_cards, ret;

	LOG0();
	ret = platform_driver_register(&snd_drv_vsnd_info);
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
				(struct vsndif_card_config *)cur_card_config;
		snd_drv_dev = platform_device_register_data(NULL, VSND_DRIVER_NAME,
				i, &snd_dev_platdata, sizeof(snd_dev_platdata));
		if (IS_ERR(snd_drv_dev))
			goto fail;
		drv_info->snd_drv_dev[i] = snd_drv_dev;
		/* advance pointer to the next card configuration */
		cur_card_config += sizeof(struct vsndif_card_config) +
				snd_dev_platdata.card_config->num_pcm_instances *
				sizeof(struct vsndif_pcm_instance_config);
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

static int xen_drv_talk_to_soundback(struct xenbus_device *xen_bus_dev,
				struct xen_drv_vsnd_info *drv_info);
static int xen_drv_vsnd_on_backend_connected(struct xen_drv_vsnd_info *drv_info);
static void xen_drv_vsnd_on_backend_disconnected(struct xen_drv_vsnd_info *drv_info);

static int xen_drv_vsnd_remove(struct xenbus_device *dev);
static void xen_drv_vsnd_ctrl_ring_free(struct xen_drv_vsnd_info *drv_info);

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
	dev_set_drvdata(&xen_bus_dev->dev, drv_info);
	drv_info->xen_bus_dev = xen_bus_dev;
	return 0;
fail:
	xenbus_dev_fatal(xen_bus_dev, ret, "allocating device memory");
	return ret;
}

static int xen_drv_vsnd_remove(struct xenbus_device *dev)
{
	struct xen_drv_vsnd_info *drv_info = dev_get_drvdata(&dev->dev);

	snd_drv_vsnd_cleanup(drv_info);
	xen_drv_vsnd_ctrl_ring_free(drv_info);
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
	struct xen_drv_vsnd_info *info = dev_get_drvdata(&xen_bus_dev->dev);

	LOG0("Backend state is %s, front is %s", xenbus_strstate(backend_state),
			xenbus_strstate(xen_bus_dev->state));
	switch (backend_state) {
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateInitialising:
	case XenbusStateInitialised:
		break;

	case XenbusStateInitWait:
		if (xen_bus_dev->state != XenbusStateInitialising)
			break;
		if (xen_drv_talk_to_soundback(xen_bus_dev, info) < 0)
			break;
		xenbus_switch_state(xen_bus_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xen_drv_vsnd_on_backend_connected(info) < 0)
			break;
		LOG0("Sound initialized");
		xenbus_switch_state(xen_bus_dev, XenbusStateConnected);
		break;

	case XenbusStateUnknown:
	case XenbusStateClosed:
		if (xen_bus_dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		xen_drv_vsnd_on_backend_disconnected(info);
		xenbus_switch_state(xen_bus_dev, XenbusStateClosed);
		break;
	}
}

static void xen_drv_vsnd_ctrl_ring_free(struct xen_drv_vsnd_info *drv_info)
{
	struct xenbus_device *xen_bus_dev;
	struct xen_vsndif_ctrl_channel *channel;

	LOG0("Cleaning up ring");
	xen_bus_dev = drv_info->xen_bus_dev;
	channel = &drv_info->ctrl_channel;
	/* release all who still waits for response if any */
	channel->resp_status = -VSNDIF_OP_STATUS_IO;
	complete_all(&channel->completion);
	if (channel->irq)
		unbind_from_irqhandler(channel->irq, drv_info);
	if (channel->evt_channel)
		xenbus_free_evtchn(xen_bus_dev, channel->evt_channel);
	channel->evt_channel = 0;
	/* End access and free the pages */
	if (channel->ring_ref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->ring_ref, 0,
				(unsigned long)channel->ring.sring);
	channel->ring.sring = NULL;
}

static irqreturn_t xen_drv_vsnd_ctrl_interrupt(int irq, void *dev_id)
{
	struct xen_drv_vsnd_info *drv_info = dev_id;
	struct xen_vsndif_ctrl_channel *channel;
	struct xen_vsndif_ctrl_response *resp;
	RING_IDX i, rp;
	unsigned long flags;

	channel = &drv_info->ctrl_channel;
	spin_lock_irqsave(&channel->io_lock, flags);
	if (unlikely(channel->state != VSNDIF_STATE_CONNECTED))
		goto out;

 again:
	rp = channel->ring.sring->rsp_prod;
	rmb(); /* Ensure we see queued responses up to 'rp'. */

	for (i = channel->ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->ring, i);
		switch (resp->operation) {
		case VSNDIF_OP_READ_CONFIG:
			LOG0("Got response on VSNDIF_OP_READ_CONFIG");
			channel->resp_status = resp->status;
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
	spin_unlock_irqrestore(&channel->io_lock, flags);
	return IRQ_HANDLED;
}

static int xen_drv_vsnd_ctrl_ring_alloc(struct xen_drv_vsnd_info *drv_info)
{
	struct xenbus_device *xen_bus_dev;
	struct xen_vsndif_ctrl_channel *channel;
	struct xen_vsndif_ctrl_sring *sring;
	grant_ref_t gref;
	int ret;

	xen_bus_dev = drv_info->xen_bus_dev;
	channel = &drv_info->ctrl_channel;
	LOG0("Setting up ring");
	spin_lock_init(&channel->io_lock);
	init_completion(&channel->completion);
	channel->state = VSNDIF_STATE_DISCONNECTED;
	channel->ring_ref = GRANT_INVALID_REF;
	channel->ring.sring = NULL;
	sring = (struct xen_vsndif_ctrl_sring *)get_zeroed_page(GFP_NOIO | __GFP_HIGH);
	if (!sring) {
		ret = -ENOMEM;
		xenbus_dev_fatal(xen_bus_dev, ret, "allocating ring page");
		goto fail;
	}
	SHARED_RING_INIT(sring);
	FRONT_RING_INIT(&channel->ring, sring, PAGE_SIZE);

	ret = xenbus_grant_ring(xen_bus_dev, sring, 1, &gref);
	if (ret < 0)
		goto fail;
	channel->ring_ref = gref;

	ret = xenbus_alloc_evtchn(xen_bus_dev, &channel->evt_channel);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(channel->evt_channel,
			xen_drv_vsnd_ctrl_interrupt,
			0, xen_bus_dev->devicetype, drv_info);

	if (ret < 0)
		goto fail;
	channel->irq = ret;
	return 0;

fail:
	xenbus_dev_fatal(xen_bus_dev, ret, "allocating ring");
	xen_drv_vsnd_ctrl_ring_free(drv_info);
	return ret;
}

static inline void xen_drv_vsnd_ctrl_ring_flush(
		struct xen_vsndif_ctrl_channel *channel)
{
	int notify;

	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->ring, notify);

	if (notify)
		notify_remote_via_irq(channel->irq);
}

static int xen_drv_be_get_sound_config(struct xen_drv_vsnd_info *drv_info)
{
	struct xen_vsndif_ctrl_channel *channel;
	struct xen_vsndif_ctrl_request *req;
	unsigned long flags;

	channel = &drv_info->ctrl_channel;
	BUG_ON(channel->state != VSNDIF_STATE_CONNECTED);

	spin_lock_irqsave(&channel->io_lock, flags);
	if (RING_FULL(&channel->ring)) {
		spin_unlock_irqrestore(&channel->io_lock, flags);
		return -EBUSY;
	}
	reinit_completion(&channel->completion);

	req = RING_GET_REQUEST(&channel->ring, channel->ring.req_prod_pvt);
	req->operation = VSNDIF_OP_READ_CONFIG;

	channel->ring.req_prod_pvt++;
	xen_drv_vsnd_ctrl_ring_flush(channel);
	spin_unlock_irqrestore(&channel->io_lock, flags);

	if (wait_for_completion_interruptible_timeout(&channel->completion,
			msecs_to_jiffies(VSND_WAIT_BACK_MS)) <= 0) {
		/* TODO: remove me */
		DBG_vsnd_run(drv_info->xen_bus_dev, drv_info);
		return 0;
		/* TODO: remove me */
		return -ETIMEDOUT;
	}
	if (channel->resp_status < 0)
		/* TODO: check and convert VSNDIF error code */
		return -EIO;
	return 0;
}

/* Common code used when first setting up, and when resuming. */
static int xen_drv_talk_to_soundback(struct xenbus_device *xen_bus_dev,
				struct xen_drv_vsnd_info *drv_info)
{
	struct xenbus_transaction xbt;
	const char *message;
	int err;

	LOG0("Allocating and opening control ring channel");
	/* allocate and open control channel */
	err = xen_drv_vsnd_ctrl_ring_alloc(drv_info);
	if (err)
		goto out;
again:
	err = xenbus_transaction_start(&xbt);
	if (err) {
		xenbus_dev_fatal(xen_bus_dev, err, "starting transaction");
		goto destroy_ring;
	}

	/* Write control channel ring reference */
	err = xenbus_printf(xbt, xen_bus_dev->nodename,
			VSNDIF_PROTO_RING_NAME_CTRL, "%u",
			drv_info->ctrl_channel.ring_ref);
	if (err) {
		message = "writing " VSNDIF_PROTO_RING_NAME_CTRL;
		goto abort_transaction;
	}

	err = xenbus_printf(xbt, xen_bus_dev->nodename,
			VSNDIF_PROTO_CTRL_EVT_CHHNL, "%u",
			drv_info->ctrl_channel.evt_channel);
	if (err) {
		message = "writing " VSNDIF_PROTO_CTRL_EVT_CHHNL;
		goto abort_transaction;
	}

	err = xenbus_transaction_end(xbt, 0);
	if (err) {
		if (err == -EAGAIN)
			goto again;
		xenbus_dev_fatal(xen_bus_dev, err, "completing transaction");
		goto destroy_ring;
	}
	LOG0("Allocated and opened control ring channel: "
			VSNDIF_PROTO_RING_NAME_CTRL " ->%u",
			drv_info->ctrl_channel.ring_ref);
	return 0;

abort_transaction:
	xenbus_dev_fatal(xen_bus_dev, err, "%s", message);
	xenbus_transaction_end(xbt, 1);
destroy_ring:
	xen_drv_vsnd_ctrl_ring_free(drv_info);
out:
	return err;
}

static int xen_drv_vsnd_on_backend_connected(struct xen_drv_vsnd_info *drv_info)
{
	struct xen_vsndif_ctrl_channel *channel;
	unsigned long flags;
	int ret;

	channel = &drv_info->ctrl_channel;
	switch (channel->state) {
	case VSNDIF_STATE_CONNECTED:
		/* fall through */
	case VSNDIF_STATE_SUSPENDED:
		LOG0("Already connected");
		return -EEXIST;
	case VSNDIF_STATE_DISCONNECTED:
		break;
	default:
		return -ENOENT;
	}
	spin_lock_irqsave(&channel->io_lock, flags);
	channel->state = VSNDIF_STATE_CONNECTED;
	spin_unlock_irqrestore(&channel->io_lock, flags);
	LOG0("Requesting sound configuration");
	ret = xen_drv_be_get_sound_config(drv_info);
	if (ret < 0) {
		LOG0("Failed to get sound configuration with err %d", ret);
		return ret;
	}
	LOG0("Got sound configuration, initializing");
	return snd_drv_vsnd_init(drv_info);
}

static void xen_drv_vsnd_on_backend_disconnected(struct xen_drv_vsnd_info *drv_info)
{
	struct xen_vsndif_ctrl_channel *channel;
	unsigned long flags;

	LOG0("Cleaning up on backed disconnection");
	if (drv_info->xen_bus_dev->state == XenbusStateClosing)
		return;
	channel = &drv_info->ctrl_channel;
	spin_lock_irqsave(&channel->io_lock, flags);
	channel->state = VSNDIF_STATE_DISCONNECTED;
	xen_drv_vsnd_ctrl_ring_free(drv_info);
	spin_unlock_irqrestore(&channel->io_lock, flags);
}

/*
 * Xen driver stop
 */

static const struct xenbus_device_id xen_drv_vsnd_ids[] = {
	{ VSND_DRIVER_NAME },
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
		LOG0(VSND_DRIVER_NAME " cannot run in Dom0");
		return -ENODEV;
	}
	if (!xen_has_pv_devices())
		return -ENODEV;
	LOG0("Registering XEN PV " VSND_DRIVER_NAME);
	return xenbus_register_frontend(&xen_vsnd_driver);
}

static void __exit xen_drv_vsnd_cleanup(void)
{
	LOG0("Unregistering XEN PV " VSND_DRIVER_NAME);
	xenbus_unregister_driver(&xen_vsnd_driver);
}

module_init(xen_drv_vsnd_init);
module_exit(xen_drv_vsnd_cleanup);

MODULE_DESCRIPTION("Xen virtual sound device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"VSND_DRIVER_NAME);
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");

