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
#include <linux/vmalloc.h>

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

enum xdrv_evtchnl_state {
	EVTCHNL_STATE_DISCONNECTED,
	EVTCHNL_STATE_CONNECTED,
	EVTCHNL_STATE_SUSPENDED,
};

struct xdrv_evtchnl_info {
	struct xdrv_info *drv_info;
	struct xen_sndif_front_ring ring;
	int ring_ref;
	unsigned int port;
	unsigned int irq;
	struct completion completion;
	/* state of the event channel */
	enum xdrv_evtchnl_state state;
	/* latest response status and id */
	int resp_status;
	uint8_t resp_id;
};

struct sdev_alsa_timer_info {
	spinlock_t lock;
	struct timer_list timer;
	unsigned long base_time;
	unsigned int frac_pos;	/* fractional sample position (based HZ) */
	unsigned int frac_period_rest;
	unsigned int frac_buffer_size;	/* buffer_size * HZ */
	unsigned int frac_period_size;	/* period_size * HZ */
	unsigned int rate;
	int elapsed;
	struct snd_pcm_substream *substream;
};

struct sdev_pcm_stream_info {
	int index;
	struct snd_pcm_hardware pcm_hw;
	struct xdrv_evtchnl_info *evtchnl;
	grant_ref_t grefs[XENSND_MAX_PAGES_PER_REQUEST];
	unsigned char *vbuffer;
	bool is_open;
	uint8_t req_next_id;
	struct sdev_alsa_timer_info dpcm;
};

struct sdev_pcm_instance_info {
	struct sdev_card_info *card_info;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	int num_pcm_streams_pb;
	struct sdev_pcm_stream_info *streams_pb;
	int num_pcm_streams_cap;
	struct sdev_pcm_stream_info *streams_cap;
};

struct sdev_card_info {
	struct xdrv_info *xdrv_info;
	struct snd_card *card;
	struct snd_pcm_hardware pcm_hw;
	/* array of PCM instances of this card */
	int num_pcm_instances;
	struct sdev_pcm_instance_info *pcm_instances;
};

struct xdrv_info {
	struct xenbus_device *xb_dev;
	spinlock_t io_lock;
	struct mutex mutex;
	bool sdrv_registered;
	/* array of virtual sound platform devices */
	struct platform_device **sdrv_devs;

	int num_evt_channels;
	struct xdrv_evtchnl_info *evtchnls;

	/* number of virtual cards */
	int cfg_num_cards;
	struct sdev_card_plat_data *cfg_plat_data;
};

struct cfg_stream {
	int index;
	char *xenstore_path;
	struct snd_pcm_hardware pcm_hw;
};

struct cfg_pcm_instance {
	char name[80];
	/* device number */
	int device_id;
	/* Device's PCM hardware descriptor */
	struct snd_pcm_hardware pcm_hw;
	int  num_streams_pb;
	struct cfg_stream *streams_pb;
	int  num_streams_cap;
	struct cfg_stream *streams_cap;
};

struct cfg_card {
	/* card configuration */
	char shortname[32];
	char longname[80];
	/* number of PCM instances in this configuration */
	int num_devices;
	/* Card's PCM hardware descriptor */
	struct snd_pcm_hardware pcm_hw;

	/* pcm instance configurations */
	struct cfg_pcm_instance *pcm_instances;
};

struct sdev_card_plat_data {
	int index;
	struct xdrv_info *xdrv_info;
	struct cfg_card cfg_card;
};

static inline void xdrv_evtchnl_flush(
		struct xdrv_evtchnl_info *channel);
static void sdrv_copy_pcm_hw(struct snd_pcm_hardware *dst,
	struct snd_pcm_hardware *src,
	struct snd_pcm_hardware *ref_pcm_hw);

static int sndif_to_kern_error(int sndif_err)
{
	switch (sndif_err) {
	case XENSND_RSP_OKAY:
		return 0;
	case XENSND_RSP_ERROR:
		return -EIO;
	default:
		LOG0("Unsupported error code: %d", sndif_err);
		dump_stack();
		break;
	}
	return -EIO;
}

static uint64_t alsa_to_sndif_format(snd_pcm_format_t format)
{
	return format;
}

/*
 * Sound driver start
 */

struct sdev_pcm_stream_info * sdrv_stream_get(
		struct snd_pcm_substream *substream)
{
	struct sdev_pcm_instance_info *pcm_instance =
			snd_pcm_substream_chip(substream);
	struct sdev_pcm_stream_info *stream;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &pcm_instance->streams_pb[substream->number];
	else
		stream = &pcm_instance->streams_cap[substream->number];
	BUG_ON(stream == NULL);
	return stream;
}

static inline struct xensnd_req *sdrv_be_stream_prepare_req(
		struct sdev_pcm_stream_info *stream,
		uint8_t operation)
{
	struct xensnd_req *req;

	req = RING_GET_REQUEST(&stream->evtchnl->ring,
			stream->evtchnl->ring.req_prod_pvt);
	req->u.data.operation = operation;
	req->u.data.stream_idx = stream->index;
	req->u.data.id = stream->req_next_id++;
	stream->evtchnl->resp_id = req->u.data.id;
	return req;
}

void sdrv_be_stream_free(struct sdev_pcm_stream_info *stream)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(stream->grefs); i++)
		if (stream->grefs[i])
			gnttab_end_foreign_access(stream->grefs[i], 0, 0UL);
	memset(stream->grefs, 0, sizeof(stream->grefs));
	if (stream->vbuffer)
		vfree(stream->vbuffer);
	stream->vbuffer = NULL;
	stream->is_open = false;
	stream->req_next_id = 0;
}

/* CAUTION!!! Call this with the spin lock held.
 * This function will release it
 */
int sdrv_be_stream_do_io(struct snd_pcm_substream *substream,
		struct xdrv_info *xdrv_info,
		struct xensnd_req *req, unsigned long flags)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	int ret;

	reinit_completion(&stream->evtchnl->completion);
	if (unlikely(stream->evtchnl->state != EVTCHNL_STATE_CONNECTED)) {
		spin_unlock_irqrestore(&xdrv_info->io_lock, flags);
		return -EIO;
	}
	xdrv_evtchnl_flush(stream->evtchnl);
	spin_unlock_irqrestore(&xdrv_info->io_lock, flags);
	ret = 0;
	if (wait_for_completion_interruptible_timeout(&stream->evtchnl->completion,
			msecs_to_jiffies(VSND_WAIT_BACK_MS)) <= 0)
		ret = -ETIMEDOUT;
	LOG0("Got response ret %d", ret);
	if (ret < 0)
		return ret;
	return sndif_to_kern_error(stream->evtchnl->resp_status);
}

int sdrv_be_stream_open(struct snd_pcm_substream *substream,
		struct sdev_pcm_stream_info *stream)
{
	struct sdev_pcm_instance_info *pcm_instance =
				snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xdrv_info *xdrv_info;
	struct xensnd_req *req;
	int ret;
	unsigned long flags;

	LOG0("Opening stream idx %d evt port %u", stream->index, stream->evtchnl->port);
	xdrv_info = pcm_instance->card_info->xdrv_info;
	spin_lock_irqsave(&xdrv_info->io_lock, flags);

	req = sdrv_be_stream_prepare_req(stream, XENSND_OP_OPEN);
	req->u.data.op.open.format = alsa_to_sndif_format(runtime->format);
	req->u.data.op.open.channels = runtime->channels;
	req->u.data.op.open.rate = runtime->rate;
	memcpy(&req->u.data.op.open.grefs, &stream->grefs,
			sizeof(req->u.data.op.open.grefs) + BUILD_BUG_ON_ZERO(
				ARRAY_SIZE(req->u.data.op.open.grefs) !=
				ARRAY_SIZE(stream->grefs)));
	ret = sdrv_be_stream_do_io(substream, xdrv_info, req, flags);
	stream->is_open = ret < 0 ? false : true;
	return ret;
}

int sdrv_be_stream_close(struct snd_pcm_substream *substream,
		struct sdev_pcm_stream_info *stream)
{
	struct sdev_pcm_instance_info *pcm_instance =
				snd_pcm_substream_chip(substream);
	struct xdrv_info *xdrv_info;
	struct xensnd_req *req;
	int ret;
	unsigned long flags;

	LOG0("Closing stream idx %d", stream->index);
	xdrv_info = pcm_instance->card_info->xdrv_info;
	spin_lock_irqsave(&xdrv_info->io_lock, flags);

	req = sdrv_be_stream_prepare_req(stream, XENSND_OP_CLOSE);
	ret = sdrv_be_stream_do_io(substream, xdrv_info, req, flags);
	stream->is_open = false;
	return ret;
}

static void sdrv_alsa_timer_rearm(struct sdev_alsa_timer_info *dpcm)
{
	mod_timer(&dpcm->timer, jiffies +
		(dpcm->frac_period_rest + dpcm->rate - 1) / dpcm->rate);
}

static void sdrv_alsa_timer_update(struct sdev_alsa_timer_info *dpcm)
{
	unsigned long delta;

	delta = jiffies - dpcm->base_time;
	if (!delta)
		return;
	dpcm->base_time += delta;
	delta *= dpcm->rate;
	dpcm->frac_pos += delta;
	while (dpcm->frac_pos >= dpcm->frac_buffer_size)
		dpcm->frac_pos -= dpcm->frac_buffer_size;
	while (dpcm->frac_period_rest <= delta) {
		dpcm->elapsed++;
		dpcm->frac_period_rest += dpcm->frac_period_size;
	}
	dpcm->frac_period_rest -= delta;
}

static int sdrv_alsa_timer_start(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_alsa_timer_info *dpcm = &stream->dpcm;
	spin_lock(&dpcm->lock);
	dpcm->base_time = jiffies;
	sdrv_alsa_timer_rearm(dpcm);
	spin_unlock(&dpcm->lock);
	return 0;
}

static int sdrv_alsa_timer_stop(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_alsa_timer_info *dpcm = &stream->dpcm;
	spin_lock(&dpcm->lock);
	del_timer(&dpcm->timer);
	spin_unlock(&dpcm->lock);
	return 0;
}

static int sdrv_alsa_timer_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_alsa_timer_info *dpcm = &stream->dpcm;

	dpcm->frac_pos = 0;
	dpcm->rate = runtime->rate;
	dpcm->frac_buffer_size = runtime->buffer_size * HZ;
	dpcm->frac_period_size = runtime->period_size * HZ;
	dpcm->frac_period_rest = dpcm->frac_period_size;
	dpcm->elapsed = 0;

	return 0;
}

static void sdrv_alsa_timer_callback(unsigned long data)
{
	struct sdev_alsa_timer_info *dpcm = (struct sdev_alsa_timer_info *)data;
	unsigned long flags;
	int elapsed = 0;

	spin_lock_irqsave(&dpcm->lock, flags);
	sdrv_alsa_timer_update(dpcm);
	sdrv_alsa_timer_rearm(dpcm);
	elapsed = dpcm->elapsed;
	dpcm->elapsed = 0;
	spin_unlock_irqrestore(&dpcm->lock, flags);
	if (elapsed)
		snd_pcm_period_elapsed(dpcm->substream);
}

static snd_pcm_uframes_t sdrv_alsa_timer_pointer(
		struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_alsa_timer_info *dpcm = &stream->dpcm;
	snd_pcm_uframes_t pos;

	spin_lock(&dpcm->lock);
	sdrv_alsa_timer_update(dpcm);
	pos = dpcm->frac_pos / HZ;
	spin_unlock(&dpcm->lock);
	return pos;
}

static int sdrv_alsa_timer_create(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_alsa_timer_info *dpcm = &stream->dpcm;
	setup_timer(&dpcm->timer, sdrv_alsa_timer_callback,
			(unsigned long) dpcm);
	spin_lock_init(&dpcm->lock);
	dpcm->substream = substream;
	return 0;
}

int sdrv_alsa_open(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_instance_info *pcm_instance =
			snd_pcm_substream_chip(substream);
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xdrv_info *xdrv_info;
	int ret;
	unsigned long flags;

	LOG0("Substream is %s direction %d number %d stream idx %d", substream->name,
			substream->stream, substream->number, stream->index);

	sdrv_copy_pcm_hw(&runtime->hw,
		&stream->pcm_hw, &pcm_instance->pcm_hw);

	runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
			      SNDRV_PCM_INFO_MMAP_VALID |
			      SNDRV_PCM_INFO_DOUBLE |
			      SNDRV_PCM_INFO_BATCH |
			      SNDRV_PCM_INFO_NONINTERLEAVED |
			      SNDRV_PCM_INFO_RESUME |
			      SNDRV_PCM_INFO_PAUSE);
	runtime->hw.info |= SNDRV_PCM_INFO_INTERLEAVED;

	xdrv_info = pcm_instance->card_info->xdrv_info;
	ret = sdrv_alsa_timer_create(substream);
	spin_lock_irqsave(&xdrv_info->io_lock, flags);
	stream->req_next_id = 0;
	stream->is_open = false;
	stream->vbuffer = NULL;
	stream->evtchnl = &xdrv_info->evtchnls[stream->index];
	if (ret < 0)
		stream->evtchnl->state = EVTCHNL_STATE_DISCONNECTED;
	else
		stream->evtchnl->state = EVTCHNL_STATE_CONNECTED;
	spin_unlock_irqrestore(&xdrv_info->io_lock, flags);
	return ret;
}

int sdrv_alsa_close(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_instance_info *pcm_instance =
			snd_pcm_substream_chip(substream);
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct xdrv_info *xdrv_info;
	unsigned long flags;

	LOG0("TODO: mutex_lock/unlock: Substream is %s", substream->name);
	xdrv_info = pcm_instance->card_info->xdrv_info;
	sdrv_alsa_timer_stop(substream);
	spin_lock_irqsave(&xdrv_info->io_lock, flags);
	stream->evtchnl->state = EVTCHNL_STATE_DISCONNECTED;
	spin_unlock_irqrestore(&xdrv_info->io_lock, flags);
	return 0;
}

int sdrv_alsa_hw_params(struct snd_pcm_substream *substream,
		 struct snd_pcm_hw_params *params)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	grant_ref_t priv_gref_head;
	int ret, i, cur_ref;
	int otherend_id;

	LOG0("Substream is %s, allocating buffers for idx %d", substream->name,
			stream->index);
	/* TODO: use XC_PAGE_SIZE */
	stream->vbuffer = vmalloc(ARRAY_SIZE(stream->grefs) * PAGE_SIZE);
	if (!stream->vbuffer) {
		ret = -ENOMEM;
		goto fail;
	}
	ret = gnttab_alloc_grant_references(ARRAY_SIZE(stream->grefs),
			&priv_gref_head);
	if (ret)
		goto fail;
	otherend_id = stream->evtchnl->drv_info->xb_dev->otherend_id;
	for (i = 0; i < ARRAY_SIZE(stream->grefs); i++) {
		cur_ref = gnttab_claim_grant_reference(&priv_gref_head);
		if (cur_ref < 0) {
			ret = cur_ref;
			goto fail;
		}
		gnttab_grant_foreign_access_ref(cur_ref, otherend_id,
				xen_page_to_gfn(vmalloc_to_page(stream->vbuffer +
						PAGE_SIZE * i)), 0);
		stream->grefs[i] = cur_ref;
	}
	gnttab_free_grant_references(priv_gref_head);
	LOG0("Allocated buffers for stream idx %d", stream->index);
	return 0;

fail:
	LOG0("Failed to allocate buffers for stream idx %d", stream->index);
	sdrv_be_stream_free(stream);
	return ret;
}

int sdrv_alsa_hw_free(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	int ret;

	LOG0("Substream is %s", substream->name);
	ret = sdrv_be_stream_close(substream, stream);
	sdrv_be_stream_free(stream);
	return ret;
}

int sdrv_alsa_prepare(struct snd_pcm_substream *substream)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	int ret = 0;

	LOG0("Substream is %s", substream->name);
	if (!stream->is_open) {
		ret = sdrv_be_stream_open(substream, stream);
		if (ret < 0)
			return ret;
		ret = sdrv_alsa_timer_prepare(substream);
	}
	return ret;
}

int sdrv_alsa_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	LOG0("Substream is %s direction %d number %d stream idx %d cmd %d", substream->name,
			substream->stream, substream->number, stream->index, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* fall through */
	case SNDRV_PCM_TRIGGER_RESUME:
		return sdrv_alsa_timer_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
		/* fall through */
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return sdrv_alsa_timer_stop(substream);
	default:
		break;
	}
	return 0;
}

snd_pcm_uframes_t sdrv_alsa_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t pos = sdrv_alsa_timer_pointer(substream);
	LOG0("hw_ptr_base %lu pos %lu", substream->runtime->hw_ptr_base, pos);
	return pos;
}

int sdrv_alsa_playback_do_write(struct snd_pcm_substream *substream,
		snd_pcm_uframes_t len)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_pcm_instance_info *pcm_instance =
				snd_pcm_substream_chip(substream);
	struct xdrv_info *xdrv_info;
	struct xensnd_req *req;
	unsigned long flags;

	xdrv_info = pcm_instance->card_info->xdrv_info;
	spin_lock_irqsave(&xdrv_info->io_lock, flags);
	req = sdrv_be_stream_prepare_req(stream, XENSND_OP_WRITE);
	req->u.data.op.write.len = len;
	return sdrv_be_stream_do_io(substream, xdrv_info, req, flags);
}

int sdrv_alsa_playback_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	ssize_t len;

	LOG0("Substream is %s channel %d pos %lu count %lu stream idx %d, port %d",
			substream->name, channel, pos, count,
			stream->index, stream->evtchnl->port);
	len = frames_to_bytes(substream->runtime, count);
	/* TODO: use XC_PAGE_SIZE */
	if (len > PAGE_SIZE * ARRAY_SIZE(stream->grefs))
		return -EFAULT;
	if (copy_from_user(stream->vbuffer, buf, len))
		return -EFAULT;
	return sdrv_alsa_playback_do_write(substream, len);
}

int sdrv_alsa_capture_copy(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos,
		void __user *buf, snd_pcm_uframes_t count)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	struct sdev_pcm_instance_info *pcm_instance =
				snd_pcm_substream_chip(substream);
	struct xdrv_info *xdrv_info;
	struct xensnd_req *req;
	unsigned long flags;
	int ret;
	ssize_t len;

	LOG0("Substream is %s channel %d pos %lu count %lu stream idx %d, port %d",
			substream->name, channel, pos, count,
			stream->index, stream->evtchnl->port);
	len = frames_to_bytes(substream->runtime, count);
	/* TODO: use XC_PAGE_SIZE */
	if (len > PAGE_SIZE * ARRAY_SIZE(stream->grefs))
		return -EFAULT;
	xdrv_info = pcm_instance->card_info->xdrv_info;
	spin_lock_irqsave(&xdrv_info->io_lock, flags);
	req = sdrv_be_stream_prepare_req(stream, XENSND_OP_READ);
	req->u.data.op.read.len = len;
	ret = sdrv_be_stream_do_io(substream, xdrv_info, req, flags);
	if (ret < 0)
		return ret;
	return copy_to_user(buf, stream->vbuffer, len);
}

int sdrv_alsa_playback_silence(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	struct sdev_pcm_stream_info *stream = sdrv_stream_get(substream);
	ssize_t len;

	LOG0("Substream is %s channel %d pos %lu count %lu stream idx %d, port %d",
			substream->name, channel, pos, count,
			stream->index, stream->evtchnl->port);
	len = frames_to_bytes(substream->runtime, count);
	/* TODO: use XC_PAGE_SIZE */
	if (len > PAGE_SIZE * ARRAY_SIZE(stream->grefs))
		return -EFAULT;
	if (memset(stream->vbuffer, 0, len))
		return -EFAULT;
	return sdrv_alsa_playback_do_write(substream, len);
}

/* defaults */
/* TODO: use XC_PAGE_SIZE */
#define MAX_XEN_BUFFER_SIZE	(XENSND_MAX_PAGES_PER_REQUEST * PAGE_SIZE)
#define MAX_BUFFER_SIZE		(MAX_XEN_BUFFER_SIZE * 2)
#define MIN_PERIOD_SIZE		(MAX_BUFFER_SIZE / 2)
#define MAX_PERIOD_SIZE		MAX_BUFFER_SIZE
#define USE_FORMATS 		(SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)
#define USE_RATE		SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000
#define USE_RATE_MIN		5500
#define USE_RATE_MAX		48000
#define USE_CHANNELS_MIN 	1
#define USE_CHANNELS_MAX 	2
#define USE_PERIODS_MIN 	1
#define USE_PERIODS_MAX 	(MAX_BUFFER_SIZE / MIN_PERIOD_SIZE)

static struct snd_pcm_hardware sdrv_pcm_hardware_def = {
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

static struct snd_pcm_ops sdrv_alsa_playback_ops = {
		.open =		sdrv_alsa_open,
		.close =	sdrv_alsa_close,
		.ioctl =	snd_pcm_lib_ioctl,
		.hw_params =	sdrv_alsa_hw_params,
		.hw_free =	sdrv_alsa_hw_free,
		.prepare =	sdrv_alsa_prepare,
		.trigger =	sdrv_alsa_trigger,
		.pointer =	sdrv_alsa_pointer,
		.copy =		sdrv_alsa_playback_copy,
		.silence =	sdrv_alsa_playback_silence,
};

static struct snd_pcm_ops sdrv_alsa_capture_ops = {
		.open =		sdrv_alsa_open,
		.close =	sdrv_alsa_close,
		.ioctl =	snd_pcm_lib_ioctl,
		.hw_params =	sdrv_alsa_hw_params,
		.hw_free =	sdrv_alsa_hw_free,
		.prepare =	sdrv_alsa_prepare,
		.trigger =	sdrv_alsa_trigger,
		.pointer =	sdrv_alsa_pointer,
		.copy =		sdrv_alsa_capture_copy,
};

static int sdrv_new_pcm(struct sdev_card_info *card_info,
		struct cfg_pcm_instance *instance_config,
		struct sdev_pcm_instance_info *pcm_instance_info)
{
	struct snd_pcm *pcm;
	int ret, i;

	LOG0("Device \"%s\" with id %d playback %d capture %d",
			instance_config->name,
			instance_config->device_id,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap);
	pcm_instance_info->card_info = card_info;
	sdrv_copy_pcm_hw(&pcm_instance_info->pcm_hw,
		&instance_config->pcm_hw, &card_info->pcm_hw);
	/* allocate info for playback streams if any */
	if (instance_config->num_streams_pb) {
		pcm_instance_info->streams_pb = devm_kzalloc(&card_info->card->card_dev,
			instance_config->num_streams_pb *
			sizeof(struct sdev_pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_pb)
			return -ENOMEM;
	}
	/* allocate info for capture streams if any */
	if (instance_config->num_streams_cap) {
		pcm_instance_info->streams_cap = devm_kzalloc(&card_info->card->card_dev,
			instance_config->num_streams_cap *
			sizeof(struct sdev_pcm_stream_info),
			GFP_KERNEL);
		if (!pcm_instance_info->streams_cap)
			return -ENOMEM;
	}
	pcm_instance_info->num_pcm_streams_pb = instance_config->num_streams_pb;
	pcm_instance_info->num_pcm_streams_cap = instance_config->num_streams_cap;
	for (i = 0; i < pcm_instance_info->num_pcm_streams_pb; i++) {
		pcm_instance_info->streams_pb[i].pcm_hw =
			instance_config->streams_pb[i].pcm_hw;
		pcm_instance_info->streams_pb[i].index =
			instance_config->streams_pb[i].index;
	}
	for (i = 0; i < pcm_instance_info->num_pcm_streams_cap; i++) {
		pcm_instance_info->streams_cap[i].pcm_hw =
			instance_config->streams_cap[i].pcm_hw;
		pcm_instance_info->streams_cap[i].index =
			instance_config->streams_cap[i].index;
	}

	ret = snd_pcm_new(card_info->card, instance_config->name,
			instance_config->device_id,
			instance_config->num_streams_pb,
			instance_config->num_streams_cap,
			&pcm);
	if (ret < 0)
		return ret;
	if (instance_config->num_streams_pb)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				&sdrv_alsa_playback_ops);
	if (instance_config->num_streams_cap)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
				&sdrv_alsa_capture_ops);
	pcm->private_data = pcm_instance_info;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Virtual card PCM");
	pcm_instance_info->pcm = pcm;
	return 0;
}

static void sdrv_copy_pcm_hw(struct snd_pcm_hardware *dst,
	struct snd_pcm_hardware *src,
	struct snd_pcm_hardware *ref_pcm_hw)
{
	*dst = *ref_pcm_hw;
	if (src->formats)
		dst->formats = src->formats;
	if (src->buffer_bytes_max)
		dst->buffer_bytes_max =
				src->buffer_bytes_max;
	if (src->period_bytes_min)
		dst->period_bytes_min =
				src->period_bytes_min;
	if (src->period_bytes_max)
		dst->period_bytes_max =
				src->period_bytes_max;
	if (src->periods_min)
		dst->periods_min = src->periods_min;
	if (src->periods_max)
		dst->periods_max = src->periods_max;
	if (src->rates)
		dst->rates = src->rates;
	if (src->rate_min)
		dst->rate_min = src->rate_min;
	if (src->rate_max)
		dst->rate_max = src->rate_max;
	if (src->channels_min)
		dst->channels_min = src->channels_min;
	if (src->channels_max)
		dst->channels_max = src->channels_max;
}

static int sdrv_probe(struct platform_device *pdev)
{
	struct sdev_card_info *card_info;
	struct sdev_card_plat_data *platdata;
	struct snd_card *card;
	char card_id[sizeof(card->id)];
	int ret, i;

	platdata = dev_get_platdata(&pdev->dev);
	LOG0("Creating virtual sound card %d", platdata->index);
	LOG0("Will configure %d playback/capture streams",
			platdata->cfg_card.num_devices);

	snprintf(card_id, sizeof(card->id), XENSND_DRIVER_NAME "%d",
			platdata->index);
	ret = snd_card_new(&pdev->dev, platdata->index, card_id, THIS_MODULE,
			sizeof(struct sdev_card_info), &card);
	if (ret < 0)
		return ret;
	/* card_info is allocated and maintained by snd_card_new */
	card_info = card->private_data;
	card_info->xdrv_info = platdata->xdrv_info;
	card_info->card = card;
	card_info->pcm_instances = devm_kzalloc(&pdev->dev,
			platdata->cfg_card.num_devices *
			sizeof(struct sdev_pcm_instance_info), GFP_KERNEL);
	if (!card_info->pcm_instances)
		goto fail;
	card_info->num_pcm_instances = platdata->cfg_card.num_devices;

	sdrv_copy_pcm_hw(&card_info->pcm_hw,
		&platdata->cfg_card.pcm_hw, &sdrv_pcm_hardware_def);

	for (i = 0; i < platdata->cfg_card.num_devices; i++) {
		ret = sdrv_new_pcm(card_info,
				&platdata->cfg_card.pcm_instances[i],
				&card_info->pcm_instances[i]);
		if (ret < 0)
			goto fail;
	}
#if 0
		err = snd_card_dummy_new_mixer(dummy);
		if (err < 0)
			goto __nodev;

		dummy_proc_init(dummy);

#endif
	strncpy(card->driver, XENSND_DRIVER_NAME, sizeof(card->driver));
	strncpy(card->shortname, platdata->cfg_card.shortname,
			sizeof(card->shortname));
	strncpy(card->longname, platdata->cfg_card.longname,
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

static int sdrv_remove(struct platform_device *pdev)
{
	struct sdev_card_info *info;
	struct snd_card *card = platform_get_drvdata(pdev);
	info = card->private_data;
	LOG0("Removing Card %d", info->card->number);
	snd_card_free(card);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdrv_suspend(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct sdev_card_info *info = card->private_data;
	int i;

	LOG0("Suspending Card %d", info->card->number);
	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	for (i = 0; i < info->num_pcm_instances; i++)
		snd_pcm_suspend_all(info->pcm_instances[i].pcm);
	return 0;
}

static int sdrv_resume(struct device *dev)
{
	struct snd_card *card = dev_get_drvdata(dev);
	struct sdev_card_info *info = card->private_data;

	LOG0("Resuming Card %d", info->card->number);
	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(sdrv_pm, sdrv_suspend, sdrv_resume);
#define XENSND_PM_OPS	&sdrv_pm
#else
#define XENSND_PM_OPS	NULL
#endif

static struct platform_driver sdrv_info = {
	.probe		= sdrv_probe,
	.remove		= sdrv_remove,
	.driver		= {
		.name	= XENSND_DRIVER_NAME,
		.pm	= XENSND_PM_OPS,
	},
};

static void sdrv_cleanup(struct xdrv_info *drv_info)
{
	int i;

	if (!drv_info->sdrv_registered)
		return;
	LOG0("Cleaning sound driver");
	if (drv_info->sdrv_devs) {
		for (i = 0; i < drv_info->cfg_num_cards; i++) {
			struct platform_device *sdrv_dev;

			LOG0("Removing sound card %d", i);
			sdrv_dev = drv_info->sdrv_devs[i];
			if (sdrv_dev)
				platform_device_unregister(sdrv_dev);
		}
	}
	LOG0("Removing sound driver");
	platform_driver_unregister(&sdrv_info);
	drv_info->sdrv_registered = false;
	LOG0("Sound driver cleanup complete");
}

static int sdrv_init(struct xdrv_info *drv_info)
{
	int i, num_cards, ret;

	LOG0();
	ret = platform_driver_register(&sdrv_info);
	if (ret < 0)
		return ret;
	drv_info->sdrv_registered = true;

	num_cards = drv_info->cfg_num_cards;
	drv_info->sdrv_devs = devm_kzalloc(&drv_info->xb_dev->dev,
			sizeof(drv_info->sdrv_devs[0]) * num_cards, GFP_KERNEL);
	if (!drv_info->sdrv_devs)
		goto fail;
	for (i = 0; i < num_cards; i++) {
		struct platform_device *sdrv_dev;
		struct sdev_card_plat_data *snd_dev_platdata;

		snd_dev_platdata = &drv_info->cfg_plat_data[i];
		LOG0("Adding card %d", i);
		/* pass card configuration via platform data */
		sdrv_dev = platform_device_register_data(NULL, XENSND_DRIVER_NAME,
				snd_dev_platdata->index, snd_dev_platdata,
				sizeof(*snd_dev_platdata));
		drv_info->sdrv_devs[i] = sdrv_dev;
		if (IS_ERR(sdrv_dev)) {
			drv_info->sdrv_devs[i] = NULL;
			goto fail;
		}
	}
	LOG0("Added %d cards", num_cards);
	return 0;

fail:
	LOG0("Failed to register sound driver");
	sdrv_cleanup(drv_info);
	return -ENODEV;
}

/*
 * Sound driver stop
 */

static irqreturn_t xdrv_evtchnl_interrupt(int irq, void *dev_id)
{
	struct xdrv_evtchnl_info *channel = dev_id;
	struct xdrv_info *drv_info = channel->drv_info;
	struct xensnd_resp *resp;
	RING_IDX i, rp;
	unsigned long flags;

	spin_lock_irqsave(&drv_info->io_lock, flags);
	if (unlikely(channel->state != EVTCHNL_STATE_CONNECTED)) {
		LOG0("channel->state != VSNDIF_STATE_CONNECTED for port %d", channel->port);
		goto out;
	}

 again:
	rp = channel->ring.sring->rsp_prod;
	rmb(); /* Ensure we see queued responses up to 'rp'. */

	for (i = channel->ring.rsp_cons; i != rp; i++) {
		resp = RING_GET_RESPONSE(&channel->ring, i);
		LOG0("Got response %d", resp->u.data.operation);
		if (resp->u.data.id != channel->resp_id) {
			LOG0("Dropping operation %d with id %d on stream %d with status %d",
					resp->u.data.operation, resp->u.data.id,
					resp->u.data.stream_idx, resp->u.data.status);
			continue;
		}
		switch (resp->u.data.operation) {
		case XENSND_OP_OPEN:
		case XENSND_OP_CLOSE:
		case XENSND_OP_READ:
		case XENSND_OP_WRITE:
			channel->resp_status = resp->u.data.status;
			LOG0("complete(&channel->completion) status %d", channel->resp_status);
			complete(&channel->completion);
			break;
		case XENSND_OP_SET_VOLUME:
		case XENSND_OP_GET_VOLUME:
			LOG0("Not supported");
			channel->resp_status = XENSND_RSP_OKAY;
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

static void xdrv_evtchnl_free(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *channel)
{
	LOG0("Cleaning up ring-ref %u at port %u",
			channel->ring_ref, channel->port);
	if (!channel->ring.sring)
		return;
	channel->state = EVTCHNL_STATE_DISCONNECTED;
	/* release all who still waits for response if any */
	channel->resp_status = -XENSND_RSP_ERROR;
	complete_all(&channel->completion);
	if (channel->irq)
		unbind_from_irqhandler(channel->irq, channel);
	channel->irq = 0;
	if (channel->port)
		xenbus_free_evtchn(drv_info->xb_dev, channel->port);
	channel->port = 0;
	/* End access and free the pages */
	if (channel->ring_ref != GRANT_INVALID_REF)
		gnttab_end_foreign_access(channel->ring_ref, 0,
				(unsigned long)channel->ring.sring);
	channel->ring.sring = NULL;
}

static void xdrv_evtchnl_free_all(struct xdrv_info *drv_info)
{
	int i;

	LOG0("Cleaning up event channels for streams");
	if (!drv_info->evtchnls)
		return;
	for (i = 0; i < drv_info->num_evt_channels; i++)
		xdrv_evtchnl_free(drv_info,
				&drv_info->evtchnls[i]);
	devm_kfree(&drv_info->xb_dev->dev, drv_info->evtchnls);
	drv_info->evtchnls = NULL;
}

static int xdrv_evtchnl_alloc(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *evt_channel)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	struct xen_sndif_sring *sring;
	grant_ref_t gref;
	int ret;

	evt_channel->drv_info = drv_info;
	init_completion(&evt_channel->completion);
	evt_channel->state = EVTCHNL_STATE_DISCONNECTED;
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
	/* TODO: use XC_PAGE_SIZE */
	FRONT_RING_INIT(&evt_channel->ring, sring, PAGE_SIZE);

	ret = xenbus_grant_ring(xb_dev, sring, 1, &gref);
	if (ret < 0)
		goto fail;
	evt_channel->ring_ref = gref;

	ret = xenbus_alloc_evtchn(xb_dev, &evt_channel->port);
	if (ret < 0)
		goto fail;

	ret = bind_evtchn_to_irqhandler(evt_channel->port,
			xdrv_evtchnl_interrupt,
			0, xb_dev->devicetype, evt_channel);

	if (ret < 0)
		goto fail;
	evt_channel->irq = ret;
	return 0;

fail:
	LOG0("Failed to allocate ring with err %d", ret);
	return ret;
}

static int xdrv_evtchnl_create(struct xdrv_info *drv_info,
		struct xdrv_evtchnl_info *evt_channel,
		const char *path)
{
	const char *message;
	int ret;

	LOG0("Allocating and opening event channel");
	/* allocate and open control channel */
	ret = xdrv_evtchnl_alloc(drv_info, evt_channel);
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

static inline void xdrv_evtchnl_flush(
		struct xdrv_evtchnl_info *channel)
{
	int notify;

	channel->ring.req_prod_pvt++;

	RING_PUSH_REQUESTS_AND_CHECK_NOTIFY(&channel->ring, notify);

	if (notify)
		notify_remote_via_irq(channel->irq);
}

static int xdrv_evtchnl_create_all(struct xdrv_info *drv_info,
		int num_streams)
{
	int ret, c, d, s, stream_idx;

	drv_info->evtchnls = devm_kzalloc(&drv_info->xb_dev->dev,
			num_streams *
			sizeof(struct xdrv_evtchnl_info), GFP_KERNEL);
	if (!drv_info->evtchnls) {
		ret = -ENOMEM;
		goto fail;
	}
	for (c = 0; c < drv_info->cfg_num_cards; c++) {
		struct sdev_card_plat_data *plat_data;
		plat_data = &drv_info->cfg_plat_data[c];
		for (d = 0; d < plat_data->cfg_card.num_devices; d++) {
			struct cfg_pcm_instance *pcm_instance;
			pcm_instance = &plat_data->cfg_card.pcm_instances[d];
			for (s = 0; s < pcm_instance->num_streams_pb; s++) {
				stream_idx = pcm_instance->streams_pb[s].index;
				ret = xdrv_evtchnl_create(drv_info,
						&drv_info->evtchnls[stream_idx],
						pcm_instance->streams_pb[s].xenstore_path);
				if (ret < 0)
					goto fail;
			}
			for (s = 0; s < pcm_instance->num_streams_cap; s++) {
				stream_idx = pcm_instance->streams_cap[s].index;
				ret = xdrv_evtchnl_create(drv_info,
						&drv_info->evtchnls[stream_idx],
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
	xdrv_evtchnl_free_all(drv_info);
	return ret;
}

/* get number of nodes under the path to get number of
 * cards configured or number of devices within the card
 */
static char **xdrv_cfg_get_num_nodes(const char *path, const char *node,
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

struct CFG_HW_SAMPLE_RATE {
	const char *name;
	unsigned int mask;
};
static struct CFG_HW_SAMPLE_RATE xdrv_cfg_hw_supported_rates[] = {
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

struct CFG_HW_SAMPLE_FORMAT {
	const char * name;
	u64 mask;
};
static struct CFG_HW_SAMPLE_FORMAT xdrv_cfg_hw_supported_formats[] = {
	{ .name = XENSND_PCM_FORMAT_U8_STR,                 .mask = SNDRV_PCM_FMTBIT_U8 },
	{ .name = XENSND_PCM_FORMAT_S8_STR,                 .mask = SNDRV_PCM_FMTBIT_S8 },
	{ .name = XENSND_PCM_FORMAT_U16_LE_STR,             .mask = SNDRV_PCM_FMTBIT_U16_LE },
	{ .name = XENSND_PCM_FORMAT_U16_BE_STR,             .mask = SNDRV_PCM_FMTBIT_U16_BE },
	{ .name = XENSND_PCM_FORMAT_S16_LE_STR,             .mask = SNDRV_PCM_FMTBIT_S16_LE },
	{ .name = XENSND_PCM_FORMAT_S16_BE_STR,             .mask = SNDRV_PCM_FMTBIT_S16_BE },
	{ .name = XENSND_PCM_FORMAT_U24_LE_STR,             .mask = SNDRV_PCM_FMTBIT_U24_LE },
	{ .name = XENSND_PCM_FORMAT_U24_BE_STR,             .mask = SNDRV_PCM_FMTBIT_U24_BE },
	{ .name = XENSND_PCM_FORMAT_S24_LE_STR,             .mask = SNDRV_PCM_FMTBIT_S24_LE },
	{ .name = XENSND_PCM_FORMAT_S24_BE_STR,             .mask = SNDRV_PCM_FMTBIT_S24_BE },
	{ .name = XENSND_PCM_FORMAT_U32_LE_STR,             .mask = SNDRV_PCM_FMTBIT_U32_LE },
	{ .name = XENSND_PCM_FORMAT_U32_BE_STR,             .mask = SNDRV_PCM_FMTBIT_U32_BE },
	{ .name = XENSND_PCM_FORMAT_S32_LE_STR,             .mask = SNDRV_PCM_FMTBIT_S32_LE },
	{ .name = XENSND_PCM_FORMAT_S32_BE_STR,             .mask = SNDRV_PCM_FMTBIT_S32_BE },
	{ .name = XENSND_PCM_FORMAT_A_LAW_STR,              .mask = SNDRV_PCM_FMTBIT_A_LAW },
	{ .name = XENSND_PCM_FORMAT_MU_LAW_STR,             .mask = SNDRV_PCM_FMTBIT_MU_LAW },
	{ .name = XENSND_PCM_FORMAT_F32_LE_STR,             .mask = SNDRV_PCM_FMTBIT_FLOAT_LE },
	{ .name = XENSND_PCM_FORMAT_F32_BE_STR,             .mask = SNDRV_PCM_FMTBIT_FLOAT_BE },
	{ .name = XENSND_PCM_FORMAT_F64_LE_STR,             .mask = SNDRV_PCM_FMTBIT_FLOAT64_LE },
	{ .name = XENSND_PCM_FORMAT_F64_BE_STR,             .mask = SNDRV_PCM_FMTBIT_FLOAT64_BE },
	{ .name = XENSND_PCM_FORMAT_IEC958_SUBFRAME_LE_STR, .mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE },
	{ .name = XENSND_PCM_FORMAT_IEC958_SUBFRAME_BE_STR, .mask = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_BE },
	{ .name = XENSND_PCM_FORMAT_IMA_ADPCM_STR,          .mask = SNDRV_PCM_FMTBIT_IMA_ADPCM },
	{ .name = XENSND_PCM_FORMAT_MPEG_STR,               .mask = SNDRV_PCM_FMTBIT_MPEG },
	{ .name = XENSND_PCM_FORMAT_GSM_STR,                .mask = SNDRV_PCM_FMTBIT_GSM },
};

static void xdrv_cfg_hw_rates(char *list, unsigned int len,
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
		for (i = 0; i < ARRAY_SIZE(xdrv_cfg_hw_supported_rates); i++)
			if (!strncasecmp(cur_rate, xdrv_cfg_hw_supported_rates[i].name,
					XENSND_SAMPLE_RATE_MAX_LEN)) {
				cur_mask =xdrv_cfg_hw_supported_rates[i].mask;
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

static void xdrv_cfg_formats(char *list, unsigned int len,
		const char *path, struct snd_pcm_hardware *pcm_hw)
{
	u64 formats;
	char *cur_format;
	int i;

	cur_format = NULL;
	formats = 0;
	while ((cur_format = strsep(&list, XENSND_LIST_SEPARATOR))) {
		for (i = 0; i < ARRAY_SIZE(xdrv_cfg_hw_supported_formats); i++)
			if (!strncasecmp(cur_format,
				xdrv_cfg_hw_supported_formats[i].name,
				XENSND_SAMPLE_FORMAT_MAX_LEN)) {
					formats |= xdrv_cfg_hw_supported_formats[i].mask;
					LOG0("Found format: %s", cur_format);
			}
	}
	if (formats)
		pcm_hw->formats = formats;
}

static void xdrv_cfg_pcm_hw(const char *path,
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
		xdrv_cfg_hw_rates(list, len, path, pcm_hw);
		LOG0("Sample rates: \"%s\"", list);
		kfree(list);
	}
	list = xenbus_read(XBT_NIL, path, XENSND_FIELD_SAMPLE_FORMATS, &len);
	if (!IS_ERR(list)) {
		xdrv_cfg_formats(list, len, path, pcm_hw);
		LOG0("Sample formats: \"%s\"", list);
		kfree(list);
	}
}

static int xdrv_cfg_get_stream_type(const char *path, int index,
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

static int xdrv_cfg_stream(struct xdrv_info *drv_info,
		struct cfg_pcm_instance *pcm_instance,
		const char *path, int index, int *cur_pb, int *cur_cap,
		int *stream_idx)
{
	int ret;
	char *str = NULL;
	char *stream_path;
	struct cfg_stream *stream;

	stream_path = devm_kasprintf(&drv_info->xb_dev->dev,
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
	xdrv_cfg_pcm_hw(stream->xenstore_path,
			&pcm_instance->pcm_hw, &stream->pcm_hw);
	LOG0("Stream %s type %s index %d", stream->xenstore_path,
			str, stream->index);
	ret = 0;
fail:
	if (str)
		kfree(str);
	return -ret;
}

static int xdrv_cfg_device(struct xdrv_info *drv_info,
		struct cfg_pcm_instance *pcm_instance,
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
	if (!kstrtoint(device_node, 10, &pcm_instance->device_id)) {
		LOG0("Device id %d", pcm_instance->device_id);
	} else {
		LOG0("Wrong device id at %s", device_path);
		ret = -EINVAL;
		goto fail;
	}
	/* check if PCM HW configuration exists for this device
	 * and update if so */
	xdrv_cfg_pcm_hw(device_path, parent_pcm_hw, &pcm_instance->pcm_hw);
	/* read streams */
	stream_nodes = xdrv_cfg_get_num_nodes(device_path, XENSND_PATH_STREAM,
			&num_streams);
	if (stream_nodes)
		kfree(stream_nodes);
	LOG0("Reading %d stream(s)", num_streams);
	pcm_instance->num_streams_pb = 0;
	pcm_instance->num_streams_cap = 0;
	/* get number of playback and capture streams */
	for (i = 0; i < num_streams; i++) {
		ret = xdrv_cfg_get_stream_type(device_path, i,
				&num_pb, &num_cap);
		if (ret < 0)
			goto fail;
		pcm_instance->num_streams_pb += num_pb;
		pcm_instance->num_streams_cap += num_cap;
	}
	if (pcm_instance->num_streams_pb) {
		pcm_instance->streams_pb = devm_kzalloc(&drv_info->xb_dev->dev,
				pcm_instance->num_streams_pb *
				sizeof(struct cfg_stream),
				GFP_KERNEL);
		if (!pcm_instance->streams_pb) {
			ret = -ENOMEM;
			goto fail;
		}
	}
	if (pcm_instance->num_streams_cap) {
		pcm_instance->streams_cap = devm_kzalloc(&drv_info->xb_dev->dev,
				pcm_instance->num_streams_cap *
				sizeof(struct cfg_stream),
				GFP_KERNEL);
		if (!pcm_instance->streams_cap) {
			ret = -ENOMEM;
			goto fail;
		}
	}
	cur_pb = 0;
	cur_cap = 0;
	for (i = 0; i < num_streams; i++) {
		ret = xdrv_cfg_stream(drv_info,
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

static void xdrv_cfg_card_common(const char *path,
		struct cfg_card *card_config)
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
	xdrv_cfg_pcm_hw(path, &sdrv_pcm_hardware_def,
			&card_config->pcm_hw);
}

static int xdrv_cfg_card(struct xdrv_info *drv_info,
		struct sdev_card_plat_data *plat_data,
		int *stream_idx)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	char *path;
	char **device_nodes = NULL;
	int ret, num_devices, i;

	path = kasprintf(GFP_KERNEL, "%s/" XENSND_PATH_CARD "/%d",
			xb_dev->nodename, plat_data->index);
	if (!path) {
		ret = -ENOMEM;
		goto fail;
	}
	device_nodes = xdrv_cfg_get_num_nodes(path, XENSND_PATH_DEVICE,
			&num_devices);
	if (!num_devices) {
		LOG0("No devices configured for sound card %d at %s/%s",
				plat_data->index, path, XENSND_PATH_DEVICE);
		ret = -ENODEV;
		goto fail;
	}
	xdrv_cfg_card_common(path, &plat_data->cfg_card);
	/* read configuration for devices of this card */
	plat_data->cfg_card.pcm_instances = devm_kzalloc(
			&drv_info->xb_dev->dev,
			num_devices * sizeof(struct cfg_pcm_instance),
			GFP_KERNEL);
	if (!plat_data->cfg_card.pcm_instances) {
		ret = -ENOMEM;
		goto fail;
	}
	kfree(path);
	path = kasprintf(GFP_KERNEL,
			"%s/" XENSND_PATH_CARD "/%d/" XENSND_PATH_DEVICE,
			xb_dev->nodename, plat_data->index);
	if (!path) {
		ret = -ENOMEM;
		goto fail;
	}
	for (i = 0; i < num_devices; i++) {
		ret = xdrv_cfg_device(drv_info,
			&plat_data->cfg_card.pcm_instances[i],
			&plat_data->cfg_card.pcm_hw,
			path, device_nodes[i], stream_idx);
		if (ret < 0)
			goto fail;
	}
	plat_data->cfg_card.num_devices = num_devices;
	ret = 0;
fail:
	if (device_nodes)
		kfree(device_nodes);
	if (path)
		kfree(path);
	return ret;
}

static void xdrv_remove_internal(struct xdrv_info *drv_info)
{
	sdrv_cleanup(drv_info);
	xdrv_evtchnl_free_all(drv_info);
}

static int xdrv_probe(struct xenbus_device *xb_dev,
				const struct xenbus_device_id *id)
{
	struct xdrv_info *drv_info;
	int ret;

	LOG0();
	drv_info = devm_kzalloc(&xb_dev->dev, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		ret = -ENOMEM;
		goto fail;
	}

	/* FIXME: this is for insmod after rmmod
	 * after removing the driver it remains in XenbusStateClosed state
	 * but I would expect XenbusStateInitialising or XenbusStateUnknown */
	xenbus_switch_state(xb_dev, XenbusStateInitialising);

	drv_info->xb_dev = xb_dev;
	spin_lock_init(&drv_info->io_lock);
	mutex_init(&drv_info->mutex);
	drv_info->sdrv_registered = false;
	dev_set_drvdata(&xb_dev->dev, drv_info);
	return 0;
fail:
	xenbus_dev_fatal(xb_dev, ret, "allocating device memory");
	return ret;
}

static int xdrv_remove(struct xenbus_device *dev)
{
	struct xdrv_info *drv_info = dev_get_drvdata(&dev->dev);

	mutex_lock(&drv_info->mutex);
	xdrv_remove_internal(drv_info);
	mutex_unlock(&drv_info->mutex);
	return 0;
}

static int xdrv_resume(struct xenbus_device *dev)
{
	struct xdrv_info *drv_info = dev_get_drvdata(&dev->dev);
	LOG0("Resuming");
	mutex_lock(&drv_info->mutex);
	mutex_unlock(&drv_info->mutex);
	return 0;
}

static int xdrv_be_on_initwait(struct xdrv_info *drv_info)
{
	struct xenbus_device *xb_dev = drv_info->xb_dev;
	char **card_nodes;
	int stream_idx;
	int i, ret;

	LOG0("Reading number of cards to configure");
	card_nodes = xdrv_cfg_get_num_nodes(xb_dev->nodename,
			XENSND_PATH_CARD, &drv_info->cfg_num_cards);
	if (card_nodes)
		kfree(card_nodes);
	if (!drv_info->cfg_num_cards) {
		LOG0("No sound cards configured");
		return 0;
	}
	LOG0("Configuring %d sound cards", drv_info->cfg_num_cards);
	drv_info->cfg_plat_data = devm_kzalloc(&drv_info->xb_dev->dev,
			drv_info->cfg_num_cards *
			sizeof(struct sdev_card_plat_data), GFP_KERNEL);
	if (!drv_info->cfg_plat_data)
		return -ENOMEM;
	/* stream index must be unique through all cards: pass it in to be
	 * incremented when creating streams */
	stream_idx = 0;
	for (i = 0; i < drv_info->cfg_num_cards; i++) {
		/* read card configuration from the store and
		 * set platform data structure */
		drv_info->cfg_plat_data[i].index = i;
		drv_info->cfg_plat_data[i].xdrv_info = drv_info;
		ret = xdrv_cfg_card(drv_info,
				&drv_info->cfg_plat_data[i], &stream_idx);
		if (ret < 0)
			return ret;
	}
	/* create event channels for all streams and publish */
	return xdrv_evtchnl_create_all(drv_info, stream_idx);
}

static int xdrv_be_on_connected(struct xdrv_info *drv_info)
{
	return sdrv_init(drv_info);
}

static void xdrv_be_on_disconnected(struct xdrv_info *drv_info)
{
	LOG0("Cleaning up on backed disconnection");
	xdrv_remove_internal(drv_info);
}

static void xdrv_be_on_changed(struct xenbus_device *xb_dev,
				enum xenbus_state backend_state)
{
	struct xdrv_info *drv_info = dev_get_drvdata(&xb_dev->dev);
	int ret;

	LOG0("Backend state is %s, front is %s", xenbus_strstate(backend_state),
			xenbus_strstate(xb_dev->state));
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
		if (xb_dev->state != XenbusStateInitialising)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xdrv_be_on_initwait(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret, "initializing frontend");
			break;
		}
		xenbus_switch_state(xb_dev, XenbusStateInitialised);
		break;

	case XenbusStateConnected:
		if (xb_dev->state != XenbusStateInitialised)
			break;
		mutex_lock(&drv_info->mutex);
		ret = xdrv_be_on_connected(drv_info);
		mutex_unlock(&drv_info->mutex);
		if (ret < 0) {
			xenbus_dev_fatal(xb_dev, ret, "initializing sound driver");
			break;
		}
		LOG0("Sound initialized");
		xenbus_switch_state(xb_dev, XenbusStateConnected);
		break;

	case XenbusStateUnknown:
		/* fall through */
	case XenbusStateClosed:
		if (xb_dev->state == XenbusStateClosed)
			break;
		/* Missed the backend's CLOSING state -- fallthrough */
	case XenbusStateClosing:
		/* FIXME: is this check needed? */
		if (xb_dev->state == XenbusStateClosing)
			break;
		mutex_lock(&drv_info->mutex);
		xdrv_be_on_disconnected(drv_info);
		mutex_unlock(&drv_info->mutex);
		xenbus_switch_state(xb_dev, XenbusStateClosed);
		break;
	}
}

static const struct xenbus_device_id xdrv_ids[] = {
	{ XENSND_DRIVER_NAME },
	{ "" }
};

static struct xenbus_driver xen_driver = {
	.ids = xdrv_ids,
	.probe = xdrv_probe,
	.remove = xdrv_remove,
	.resume = xdrv_resume,
	.otherend_changed = xdrv_be_on_changed,
};

static int __init xdrv_init(void)
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
	return xenbus_register_frontend(&xen_driver);
}

static void __exit xdrv_cleanup(void)
{
	LOG0("Unregistering XEN PV " XENSND_DRIVER_NAME);
	xenbus_unregister_driver(&xen_driver);
}

module_init(xdrv_init);
module_exit(xdrv_cleanup);

MODULE_DESCRIPTION("Xen virtual sound device frontend");
MODULE_LICENSE("GPL");
MODULE_ALIAS("xen:"XENSND_DRIVER_NAME);
MODULE_SUPPORTED_DEVICE("{{ALSA,Virtual soundcard}}");

