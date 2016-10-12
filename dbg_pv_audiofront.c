struct vaudioif_stream_config DBG_CARD_CONFIG_PLAYBACK = {
	.type = VAUDIOIF_PROTO_STREAM_TYPE_PLAYBACK,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.buffer_bytes_max = ((65536-64)*8),
	.period_bytes_max = (65536-64),
	.periods_min = 2,
	.periods_max = 8,
	.channels_min = 2,
	.channels_max = 2,
	.rates = SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_192000,
	.rate_min = 48000,
	.rate_max = 192000,
};

struct vaudioif_stream_config DBG_CARD_CONFIG_CAPTURE = {
	.type = VAUDIOIF_PROTO_STREAM_TYPE_CAPTURE,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.buffer_bytes_max = ((65536-64)*8),
	.period_bytes_max = (65536-64),
	.periods_min = 2,
	.periods_max = 8,
	.channels_min = 2,
	.channels_max = 2,
	.rates = SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_192000,
	.rate_min = 48000,
	.rate_max = 192000,
};

static struct vaudioif_card_config DBG_CARD_CONFIG = {
	.shortname = "XXX vaudio short name",
	.longname = "XXX vaudio long name",
};

static int snd_drv_vaudio_init(struct xen_drv_vaudio_info *info);

void DBG_create_card_config(int num_playback,
		int num_capture, struct vaudioif_card_config *card_config)
{
	int i;

	memcpy(card_config, &DBG_CARD_CONFIG, sizeof(DBG_CARD_CONFIG));
	card_config->num_streams = num_playback + num_capture;
	LOG0("Add configuration for %d streams", card_config->num_streams);
	for (i = 0; i < num_playback; i++) {
		LOG0("Add configuration for playback at %d", i);
		card_config->stream[i] = DBG_CARD_CONFIG_PLAYBACK;
	}
	for (; i < num_playback + num_capture; i++) {
		LOG0("Add configuration for capture at %d", i);
		card_config->stream[i] = DBG_CARD_CONFIG_CAPTURE;
	}
}

#define DBG_NUM_CARDS	2

static int DBG_NUM_STREAMS_PB[DBG_NUM_CARDS] = {
		3,
		1,
};
static int DBG_NUM_STREAMS_CAP[DBG_NUM_CARDS] = {
		2,
		1,
};

int DBG_vaudio_run(struct xenbus_device *xen_bus_dev,
		struct xen_drv_vaudio_info *info)
{
	int i, num_streams;
	int ret;
	char *cur_card_ptr;
	LOG0("HACK! -------------------------------------------------- start");
	/* FIXME: for debug fill in card's number and configuration
	 * this must be done via negotiation with the backend in reality
	 */
	info->cfg_num_cards = DBG_NUM_CARDS;
	num_streams = 0;
	for (i = 0; i < info->cfg_num_cards; i++) {
		num_streams += DBG_NUM_STREAMS_PB[i] + DBG_NUM_STREAMS_CAP[i];
	}
	info->cfg_cards = devm_kzalloc(&xen_bus_dev->dev,
			info->cfg_num_cards * sizeof(struct vaudioif_card_config) +
			num_streams * sizeof(struct vaudioif_stream_config),
			GFP_KERNEL);
	if (!info->cfg_cards) {
		ret = -ENOMEM;
		goto fail;
	}
	cur_card_ptr = (char *)info->cfg_cards;
	for (i = 0; i < info->cfg_num_cards; i++) {
		LOG0("Add configuration for card %d", i);
		DBG_create_card_config(DBG_NUM_STREAMS_PB[i], DBG_NUM_STREAMS_CAP[i],
				(struct vaudioif_card_config *)cur_card_ptr);
		num_streams = DBG_NUM_STREAMS_PB[i] + DBG_NUM_STREAMS_CAP[i];
		cur_card_ptr += sizeof(struct vaudioif_card_config) +
				num_streams * sizeof(struct vaudioif_stream_config);
	}

	ret = snd_drv_vaudio_init(info);
	if (ret < 0)
		goto fail;
	LOG0("HACK! -------------------------------------------------- stop");
	return 0;
fail:
	return ret;
}
