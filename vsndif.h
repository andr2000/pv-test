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

#ifndef __XEN_PUBLIC_VSNDIF_H__
#define __XEN_PUBLIC_VSNDIF_H__

#include <xen/interface/io/ring.h>
#include <xen/interface/grant_table.h>

#define VSNDIF_PROTO_VERSION 0x0001

#define VSNDIF_PROTO_RING_NAME_CTRL "ctrl-ring-ref"
#define VSNDIF_PROTO_CTRL_EVT_CHHNL "ctrl-event-channel"

struct vsndif_pcm_instance_config {
	char name[80];
	/* device number */
	uint8_t device;
	uint8_t  num_streams_pb;
	uint8_t  num_streams_cap;
} __attribute__((packed));

struct vsndif_card_pcm_hw_config {
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

/* FIXME: have all the configuration data related to sound as a
 * single structure in order to minimize number of transactions
 * over the Xen event channel to get this and simplify protocol
 * error handling as consequence */
struct vsndif_card_config {
	/* card configuration */
	char shortname[32];
	char longname[80];
	/* number of PCM instances in this configuration */
	uint8_t num_pcm_instances;
	/* PCM hardware descriptor */
	struct vsndif_card_pcm_hw_config card_pcm_hw;

	/* pcm instance configurations - must be the last field in the structure
	 * configuration data will be appended here: pcm_instance[num_streams] */
	struct vsndif_pcm_instance_config pcm_instance[0];
} __attribute__((packed));

/*
 * REQUEST CODES.
 */

/* Operation completed successfully. */
#define VSNDIF_OP_STATUS_OK	0
#define VSNDIF_OP_STATUS_IO	1

/* TODO: put description */
#define VSNDIF_OP_READ_CONFIG	0

struct xen_vsndif_ctrl_request {
	uint8_t operation;
} __attribute__((__packed__));

struct xen_vsndif_ctrl_response {
	uint8_t operation;	/* copied from request */
	int8_t	status;
} __attribute__((__packed__));

/*
 * Generate vsndif ring structures and types.
 */

DEFINE_RING_TYPES(xen_vsndif_ctrl, struct xen_vsndif_ctrl_request,
		struct xen_vsndif_ctrl_response);

#endif /* __XEN_PUBLIC_VSNDIF_H__ */
