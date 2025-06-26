/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Definitions shared between kernel and WPANUSB firmware
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 */

#define WPANUSB_VENDOR_ID	0x2fe3
#define WPANUSB_PRODUCT_ID	0x0101

enum wpanusb_requests {
	RESET,
	TX,
	XMIT_ASYNC,
	ED,
	SET_LBT,                    /* New: Listen Before Talk */
	SET_FRAME_RETRIES,          /* New: Frame retry count */
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SET_CCA_MODE,
	SET_CCA_ED_LEVEL,
	SET_CSMA_PARAMS,
	SET_PROMISCUOUS_MODE,
	QUERY_SUPPORTED_POWERS,     /* New: Query power levels */
	QUERY_SUPPORTED_CHANNELS,   /* New: Query channel support */
};

struct set_channel {
	__u8 page;
	__u8 channel;
} __packed;

struct set_short_addr {
	__le16 short_addr;
} __packed;

struct set_pan_id {
	__le16 pan_id;
} __packed;

struct set_ieee_addr {
	__le64 ieee_addr;
} __packed;

struct set_txpower {
	__le32 txpower;
} __packed;

struct set_cca_mode {
	__u8 mode;
	__u8 opt;
} __packed;

struct set_cca_ed_level {
	__le32 ed_level;
} __packed;

struct set_csma_params {
	__u8 min_be;
	__u8 max_be;
	__u8 retries;
} __packed;

struct set_promiscuous_mode {
	__u8 promiscuous;
} __packed;

/* New structures for enhanced features */
struct set_lbt {
	__le32 sensing_time_us;
	__le32 threshold_dbm;
	__u8 enabled;
} __packed;

struct set_frame_retries {
	__u8 retry_count;
} __packed;

struct power_level_response {
	__u8 count;
	__le32 powers[];
} __packed;

/* New structures for dynamic querying */
struct channel_support_query {
	__u8 page;
} __packed;

struct channel_support_response {
	__u8 page;
	__u8 channel_count;
	__u8 channels[];  /* List of supported channels */
} __packed;

struct power_query_response {
	__u8 count;
	__le32 min_power_mbm;
	__le32 max_power_mbm;
	__le32 power_levels[];  /* Array of supported power levels in mBm */
} __packed;
