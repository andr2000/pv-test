struct vsndif_card_pcm_hw_config DBG_CARD_PCM_HW_CONFIG = {
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

static struct vsndif_card_config DBG_CARD_CONFIG = {
	.shortname = "XXX vsnd short name",
	.longname = "XXX vsnd long name",
};

#define DBG_MAX_PCM_INSTANCE	8

static struct vsndif_pcm_instance_config DBG_PCM_INSTANCES[][DBG_MAX_PCM_INSTANCE] = {
		{
			{
				.name = "General Analog",
				.device = 0,
				.num_streams_pb = 1,
				.num_streams_cap = 1
			},
			{
				.name = "HDMI-OUT",
				.device = 2,
				.num_streams_pb = 1,
				.num_streams_cap = 0
			},
			{
				.name = "HDMI-IN",
				.device = 3,
				.num_streams_pb = 0,
				.num_streams_cap = 1
			},
		},
		{
			{
				.name = "SPDIF",
				.device = 2,
				.num_streams_pb = 1,
				.num_streams_cap = 1
			},
		},
};

static int snd_drv_vsnd_init(struct xen_drv_vsnd_info *info);

int DBG_vsnd_run(struct xenbus_device *xen_bus_dev,
		struct xen_drv_vsnd_info *info)
{
	int i, j, num_cards, num_pcm_instances;
	int ret;
	struct vsndif_card_config *cur_card_ptr;
	LOG0("HACK! -------------------------------------------------- start");
	/* FIXME: for debug fill in card's number and configuration
	 * this must be done via negotiation with the backend in reality
	 */
	num_cards = 0;
	num_pcm_instances = 0;
	while (DBG_PCM_INSTANCES[num_cards][0].name[0]) {
		i = 0;
		while (DBG_PCM_INSTANCES[num_cards][i].name[0])
			i++;
		num_pcm_instances += i;
		num_cards++;
	}
	LOG0("Found configuration for %d cards and %d PCM instances",
			num_cards, num_pcm_instances);
	info->cfg_cards = devm_kzalloc(&xen_bus_dev->dev,
			num_cards * sizeof(struct vsndif_card_config) +
			num_pcm_instances * sizeof(struct vsndif_pcm_instance_config),
			GFP_KERNEL);
	if (!info->cfg_cards) {
		ret = -ENOMEM;
		goto fail;
	}
	info->cfg_num_cards = num_cards;
	cur_card_ptr = info->cfg_cards;
	j = 0;
	while (DBG_PCM_INSTANCES[j][0].name[0]) {
		i = 0;
		while (DBG_PCM_INSTANCES[j][i].name[0])
			i++;
		LOG0("Add configuration for card %d and %d instances", j, i);
		/* copy card configuration */
		memcpy(cur_card_ptr, &DBG_CARD_CONFIG,
				sizeof(struct vsndif_card_config));
		memcpy(cur_card_ptr->pcm_instance, DBG_PCM_INSTANCES[j],
				i * sizeof(struct vsndif_pcm_instance_config));
		cur_card_ptr->num_pcm_instances = i;
		cur_card_ptr = (struct vsndif_card_config *)(
				(char *)cur_card_ptr +
				sizeof(struct vsndif_card_config) +
				i * sizeof(struct vsndif_pcm_instance_config));
		j++;
	}

	ret = snd_drv_vsnd_init(info);
	if (ret < 0)
		goto fail;
	LOG0("HACK! -------------------------------------------------- stop");
	return 0;
fail:
	return ret;
}
