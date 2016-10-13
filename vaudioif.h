/*
 *  Unified sound-device I/O interface for Xen guest OSes
 *  Copyright (c) 2016, Oleksandr Andrushchenko
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


#ifndef __XEN_PUBLIC_VAUDIOIF_H__
#define __XEN_PUBLIC_VAUDIOIF_H__

#define VAUDIOIF_PROTO_VERSION 0x0001

#define VAUDIOIF_PROTO_STREAM_TYPE_PLAYBACK	(1)
#define VAUDIOIF_PROTO_STREAM_TYPE_CAPTURE	(2)

struct vaudioif_stream_config {
	uint8_t  type;
} __attribute__((packed));

struct vaudioif_card_pcm_hw_config {
	uint64_t formats;
	uint64_t buffer_bytes_max;
	uint64_t period_bytes_min;
	uint64_t period_bytes_max;
	uint64_t periods_min;
	uint64_t periods_max;
	uint64_t rates;
	uint64_t rate_min;
	uint64_t rate_max;
	uint64_t channels_min;
	uint64_t channels_max;
} __attribute__((packed));

/* FIXME: have all the configuration data related to audio as a
 * single structure in order to minimize number of transactions
 * over the Xen event channel to get this and simplify protocol
 * error handling as consequence */
struct vaudioif_card_config {
	/* card configuration */
	char shortname[32];
	char longname[80];
	/* number of streams in this configuration */
	uint8_t num_streams;
	/* PCM hardware descriptor */
	struct vaudioif_card_pcm_hw_config card_pcm_hw;

	/* streams configurations - must be the last field in the structure
	 * configuration data will be appended here: stream[num_streams] */
	struct vaudioif_stream_config stream[0];
} __attribute__((packed));

/* get the size of the card configuration structure */
#define VAUDIOIF_CARD_CONFIG_SIZEOF(a)	((size_t)(\
	sizeof(struct vaudioif_card_config) +\
	(a)->num_streams * sizeof(struct vaudioif_stream_config)))

#endif /* __XEN_PUBLIC_VAUDIOIF_H__ */
