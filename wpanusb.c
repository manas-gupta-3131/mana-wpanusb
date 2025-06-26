// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the WPANUSB IEEE 802.15.4 dongle
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * The driver implements SoftMAC 802.15.4 protocol based on atusb
 * driver for ATUSB IEEE 802.15.4 dongle.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/usb.h>

#include <net/cfg802154.h>
#include <net/mac802154.h>

#include "wpanusb.h"

#define WPANUSB_NUM_RX_URBS	4	/* allow for a bit of local latency */
#define WPANUSB_ALLOC_DELAY_MS	100	/* delay after failed allocation */

#define VENDOR_OUT		(USB_TYPE_VENDOR | USB_DIR_OUT)

#define WPANUSB_VALID_CHANNELS	(0x07FFF800)

struct lbt_params {
	u32 sensing_time_us;
	s32 threshold_dbm;
	bool enabled;
};

struct wpanusb {
	struct ieee802154_hw *hw;
	struct usb_device *udev;
	int shutdown;			/* non-zero if shutting down */

	/* RX variables */
	struct delayed_work work;	/* memory allocations */
	struct usb_anchor idle_urbs;	/* URBs waiting to be submitted */
	struct usb_anchor rx_urbs;	/* URBs waiting for reception */

	/* TX variables */
	struct usb_ctrlrequest tx_dr;
	struct urb *tx_urb;
	struct sk_buff *tx_skb;
	u8 tx_ack_seq;			/* current TX ACK sequence number */

	/* Enhanced features state */
	struct lbt_params lbt_config;
	int frame_retry_count;
	s32 *supported_powers;
	size_t supported_powers_size;
	u8 min_be, max_be, csma_retries;
	u8 cca_mode;
	s32 cca_ed_level;

	/* Dynamic channel and power support */
	u32 supported_channels[3]; /* Up to 3 channel pages */
	u32 channel_pages_supported;
	s32 min_power_mbm, max_power_mbm;
};

/* ----- USB commands without data ----------------------------------------- */

static int wpanusb_control_send(struct wpanusb *wpanusb, unsigned int pipe,
				u8 request, void *data, u16 size)
{
	struct usb_device *udev = wpanusb->udev;

	return usb_control_msg(udev, pipe, request, VENDOR_OUT,
			       0, 0, data, size, 1000);
}

/* ----- skb allocation ---------------------------------------------------- */

#define MAX_PSDU	127
#define MAX_RX_XFER	(1 + MAX_PSDU + 2 + 1)	/* PHR+PSDU+CRC+LQI */

#define SKB_WPANUSB(skb)	(*(struct wpanusb **)(skb)->cb)

static void wpanusb_bulk_complete(struct urb *urb);

static int wpanusb_submit_rx_urb(struct wpanusb *wpanusb, struct urb *urb)
{
	struct usb_device *udev = wpanusb->udev;
	struct sk_buff *skb = urb->context;
	int ret;

	if (!skb) {
		skb = alloc_skb(MAX_RX_XFER, GFP_KERNEL);
		if (!skb) {
			dev_warn_ratelimited(&udev->dev,
					     "can't allocate skb\n");
			return -ENOMEM;
		}
		skb_put(skb, MAX_RX_XFER);
		SKB_WPANUSB(skb) = wpanusb;
	}

	usb_fill_bulk_urb(urb, udev, usb_rcvbulkpipe(udev, 1),
			  skb->data, MAX_RX_XFER, wpanusb_bulk_complete, skb);
	usb_anchor_urb(urb, &wpanusb->rx_urbs);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		usb_unanchor_urb(urb);
		kfree_skb(skb);
		urb->context = NULL;
	}

	return ret;
}

static void wpanusb_work_urbs(struct work_struct *work)
{
	struct wpanusb *wpanusb =
		container_of(to_delayed_work(work), struct wpanusb, work);
	struct usb_device *udev = wpanusb->udev;
	struct urb *urb;
	int ret;

	if (wpanusb->shutdown)
		return;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			return;

		ret = wpanusb_submit_rx_urb(wpanusb, urb);
	} while (!ret);

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	dev_warn_ratelimited(&udev->dev, "can't allocate/submit URB (%d)\n",
			     ret);
	schedule_delayed_work(&wpanusb->work,
			      msecs_to_jiffies(WPANUSB_ALLOC_DELAY_MS) + 1);
}

/* ----- Asynchronous USB -------------------------------------------------- */

static void wpanusb_tx_done(struct wpanusb *wpanusb, uint8_t seq)
{
	struct usb_device *udev = wpanusb->udev;
	u8 expect = wpanusb->tx_ack_seq;

	dev_dbg(&udev->dev, "seq 0x%02x expect 0x%02x\n", seq, expect);

	if (seq == expect) {
		ieee802154_xmit_complete(wpanusb->hw, wpanusb->tx_skb, false);
	} else {
		dev_dbg(&udev->dev, "unknown ack %u\n", seq);

		ieee802154_wake_queue(wpanusb->hw);
		if (wpanusb->tx_skb)
			dev_kfree_skb_irq(wpanusb->tx_skb);
	}
}

static void wpanusb_process_urb(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);
	u8 len, lqi;

	if (!urb->actual_length) {
		dev_dbg(&udev->dev, "zero-sized URB ?\n");
		return;
	}

	len = *skb->data;

	dev_dbg(&udev->dev, "urb %p urb len %u pkt len %u", urb,
		urb->actual_length, len);

	/* Handle ACK */
	if (urb->actual_length == 1) {
		wpanusb_tx_done(wpanusb, len);
		return;
	}

	if (len + 1 > urb->actual_length - 1) {
		dev_dbg(&udev->dev, "frame len %d+1 > URB %u-1\n",
			len, urb->actual_length);
		return;
	}

	if (!ieee802154_is_valid_psdu_len(len)) {
		dev_dbg(&udev->dev, "frame corrupted\n");
		return;
	}

	print_hex_dump_bytes("> ", DUMP_PREFIX_OFFSET, skb->data,
			     urb->actual_length);

	/* Get LQI at the end of the packet */
	lqi = skb->data[len + 1];
	dev_dbg(&udev->dev, "rx len %d lqi 0x%02x\n", len, lqi);
	skb_pull(skb, 1);	/* remove length */
	skb_trim(skb, len);	/* remove LQI */
	ieee802154_rx_irqsafe(wpanusb->hw, skb, lqi);
	urb->context = NULL;	/* skb is gone */
}

static void wpanusb_bulk_complete(struct urb *urb)
{
	struct usb_device *udev = urb->dev;
	struct sk_buff *skb = urb->context;
	struct wpanusb *wpanusb = SKB_WPANUSB(skb);

	dev_dbg(&udev->dev, "status %d len %d\n",
		urb->status, urb->actual_length);

	if (urb->status) {
		if (urb->status == -ENOENT) { /* being killed */
			kfree_skb(skb);
			urb->context = NULL;
			return;
		}

		dev_dbg(&udev->dev, "URB error %d\n", urb->status);
	} else {
		wpanusb_process_urb(urb);
	}

	usb_anchor_urb(urb, &wpanusb->idle_urbs);
	if (!wpanusb->shutdown)
		schedule_delayed_work(&wpanusb->work, 0);
}

/* ----- URB allocation/deallocation --------------------------------------- */

static void wpanusb_free_urbs(struct wpanusb *wpanusb)
{
	struct urb *urb;

	do {
		urb = usb_get_from_anchor(&wpanusb->idle_urbs);
		if (!urb)
			break;
		kfree_skb(urb->context);
		usb_free_urb(urb);
	} while (true);
}

static int wpanusb_alloc_urbs(struct wpanusb *wpanusb, unsigned int n)
{
	struct urb *urb;

	while (n--) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			wpanusb_free_urbs(wpanusb);
			return -ENOMEM;
		}
		usb_anchor_urb(urb, &wpanusb->idle_urbs);
	}

	return 0;
}

/* ----- IEEE 802.15.4 interface operations -------------------------------- */

static void wpanusb_xmit_complete(struct urb *urb)
{
	dev_dbg(&urb->dev->dev, "urb transmit completed");
}

static int wpanusb_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	int ret;

	dev_dbg(&udev->dev, "len %u", skb->len);

	/* ack_seq range is 0x01 - 0xff */
	wpanusb->tx_ack_seq++;
	if (!wpanusb->tx_ack_seq)
		wpanusb->tx_ack_seq++;

	wpanusb->tx_skb = skb;
	wpanusb->tx_dr.wIndex = cpu_to_le16(wpanusb->tx_ack_seq);
	wpanusb->tx_dr.wLength = cpu_to_le16(skb->len);

	usb_fill_control_urb(wpanusb->tx_urb, udev,
			     usb_sndctrlpipe(udev, 0),
			     (unsigned char *)&wpanusb->tx_dr, skb->data,
			     skb->len, wpanusb_xmit_complete, NULL);
	ret = usb_submit_urb(wpanusb->tx_urb, GFP_ATOMIC);

	dev_dbg(&udev->dev, "%s: ret %d len %u seq %u\n", __func__, ret,
		skb->len, wpanusb->tx_ack_seq);

	return ret;
}

static int wpanusb_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_channel *req;
	int ret;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->page = page;
	req->channel = channel;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_CHANNEL, req, sizeof(*req));
	kfree(req);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed set channel, ret %d", ret);
		return ret;
	}

	dev_dbg(&udev->dev, "set page %u channel %u", page, channel);

	return 0;
}

static int wpanusb_ed(struct ieee802154_hw *hw, u8 *level)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	u8 ed_level;
	int ret;

	WARN_ON(!level);

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
                  ED, USB_TYPE_VENDOR | USB_DIR_IN,
                  0, 0, &ed_level, sizeof(ed_level),
                  USB_CTRL_GET_TIMEOUT);
    
    if (ret < 0) {
        dev_err(&udev->dev, "Failed to perform ED, ret %d\n", ret);
        *level = 0;
        return ret;
    }

    *level = ed_level;
    dev_dbg(&udev->dev, "ED level: 0x%02x\n", ed_level);
    return 0;
}

static int wpanusb_set_cca_mode(struct ieee802154_hw *hw,
				const struct wpan_phy_cca *cca)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_cca_mode *req;
	int ret;

	dev_dbg(&udev->dev, "Setting CCA mode %u opt %u\n", cca->mode, cca->opt);

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->mode = cca->mode;
	req->opt = cca->opt;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_CCA_MODE, req, sizeof(*req));
	kfree(req);
	
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set CCA mode, ret %d\n", ret);
		return ret;
	}

	wpanusb->cca_mode = cca->mode;
	return 0;
}

static int wpanusb_set_cca_ed_level(struct ieee802154_hw *hw, s32 mbm)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_cca_ed_level *req;
	int ret;

	dev_dbg(&udev->dev, "Setting CCA ED level: %d mBm\n", mbm);

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->ed_level = cpu_to_le32(mbm);

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_CCA_ED_LEVEL, req, sizeof(*req));
	kfree(req);
	
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set CCA ED level, ret %d\n", ret);
		return ret;
	}

	wpanusb->cca_ed_level = mbm;
	return 0;
}

static int wpanusb_set_csma_params(struct ieee802154_hw *hw, u8 min_be,
				   u8 max_be, u8 retries)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_csma_params *req;
	int ret;

	dev_dbg(&udev->dev, "Setting CSMA params: min_be=%u, max_be=%u, retries=%u\n",
		min_be, max_be, retries);

	if (min_be > max_be || max_be > 8 || retries > 7) {
		dev_err(&udev->dev, "Invalid CSMA parameters\n");
		return -EINVAL;
	}

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->min_be = min_be;
	req->max_be = max_be;
	req->retries = retries;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_CSMA_PARAMS, req, sizeof(*req));
	kfree(req);
	
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set CSMA params, ret %d\n", ret);
		return ret;
	}

	wpanusb->min_be = min_be;
	wpanusb->max_be = max_be;
	wpanusb->csma_retries = retries;
	wpanusb->frame_retry_count = retries;

	return 0;
}

static int wpanusb_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	struct wpanusb *wpanusb = hw->priv;
	struct usb_device *udev = wpanusb->udev;
	struct set_promiscuous_mode *req;
	int ret;

	dev_dbg(&udev->dev, "Setting promiscuous mode: %s\n", on ? "on" : "off");

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->promiscuous = on ? 1 : 0;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_PROMISCUOUS_MODE, req, sizeof(*req));
	kfree(req);
	
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set promiscuous mode, ret %d\n", ret);
		return ret;
	}

	return 0;
}

/* LBT implementation */
static int wpanusb_set_lbt(struct wpanusb *wpanusb, struct lbt_params *params)
{
    struct usb_device *udev = wpanusb->udev;
    struct set_lbt *req;
    int ret;

    if (!params)
        return -EINVAL;

    if (params->sensing_time_us < 50 || params->sensing_time_us > 10000)
        return -EINVAL;

    if (params->threshold_dbm < -100 || params->threshold_dbm > 0)
        return -EINVAL;

    req = kmalloc(sizeof(*req), GFP_KERNEL);
    if (!req)
        return -ENOMEM;

    req->sensing_time_us = cpu_to_le32(params->sensing_time_us);
    req->threshold_dbm = cpu_to_le32(params->threshold_dbm);
    req->enabled = params->enabled ? 1 : 0;

    ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
                   SET_LBT, req, sizeof(*req));
    kfree(req);
    
    if (ret < 0) {
        dev_err(&udev->dev, "Failed to set LBT parameters, ret %d\n", ret);
        return ret;
    }

    wpanusb->lbt_config = *params;
    return 0;
}

/* Frame retry implementation */
static int wpanusb_set_frame_retries(struct wpanusb *wpanusb, int retry_count)
{
	struct usb_device *udev = wpanusb->udev;
	struct set_frame_retries *req;
	int ret;

	if (retry_count < 0 || retry_count > 7)
		return -EINVAL;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	req->retry_count = (u8)retry_count;

	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0),
				   SET_FRAME_RETRIES, req, sizeof(*req));
	kfree(req);
	
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to set frame retries, ret %d\n", ret);
		return ret;
	}

	wpanusb->frame_retry_count = retry_count;
	return 0;
}

/* Dynamic power level querying */
static int wpanusb_query_tx_power_levels(struct wpanusb *wpanusb)
{
	struct usb_device *udev = wpanusb->udev;
	struct power_query_response *resp;
	s32 *powers;
	int ret, i;
	u8 buffer[256]; /* Buffer for USB response */

	dev_dbg(&udev->dev, "Querying supported TX power levels\n");

	/* Send USB control request to query power levels */
	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      QUERY_SUPPORTED_POWERS, USB_TYPE_VENDOR | USB_DIR_IN,
			      0, 0, buffer, sizeof(buffer),
			      USB_CTRL_GET_TIMEOUT);
	
	if (ret < 0) {
		dev_warn(&udev->dev, "Failed to query power levels, using defaults: %d\n", ret);
		goto use_defaults;
	}

	if (ret < sizeof(struct power_query_response)) {
		dev_warn(&udev->dev, "Invalid power query response size: %d\n", ret);
		goto use_defaults;
	}

	resp = (struct power_query_response *)buffer;
	
	if (resp->count == 0 || resp->count > 64) {
		dev_warn(&udev->dev, "Invalid power level count: %u\n", resp->count);
		goto use_defaults;
	}

	/* Allocate dynamic power array */
	powers = kmalloc_array(resp->count, sizeof(s32), GFP_KERNEL);
	if (!powers) {
		dev_err(&udev->dev, "Failed to allocate power array\n");
		return -ENOMEM;
	}

	/* Convert little-endian response to host byte order */
	for (i = 0; i < resp->count; i++) {
		powers[i] = le32_to_cpu(resp->power_levels[i]);
	}

	/* Free existing array if present */
	kfree(wpanusb->supported_powers);

	/* Update driver state */
	wpanusb->supported_powers = powers;
	wpanusb->supported_powers_size = resp->count;
	wpanusb->min_power_mbm = le32_to_cpu(resp->min_power_mbm);
	wpanusb->max_power_mbm = le32_to_cpu(resp->max_power_mbm);

	dev_info(&udev->dev, "Queried %u power levels (range: %d to %d mBm)\n",
		 resp->count, wpanusb->min_power_mbm, wpanusb->max_power_mbm);

	return 0;

use_defaults:
	/* Fallback to hard-coded defaults */
	static const s32 default_powers[] = {
		500, 400, 300, 200, 100, 0, -100, -200, -300, -400, -500,
		-600, -700, -800, -900, -1000, -1100, -1200, -1300, -1400,
		-1500, -1600, -1700, -1800, -1900, -2000, -2100, -2200,
		-2300, -2400, -2500, -2600, -2700, -2800, -2900, -3000
	};

	powers = kmalloc_array(ARRAY_SIZE(default_powers), sizeof(s32), GFP_KERNEL);
	if (!powers)
		return -ENOMEM;

	memcpy(powers, default_powers, sizeof(default_powers));
	
	kfree(wpanusb->supported_powers);
	wpanusb->supported_powers = powers;
	wpanusb->supported_powers_size = ARRAY_SIZE(default_powers);
	wpanusb->min_power_mbm = default_powers[ARRAY_SIZE(default_powers) - 1];
	wpanusb->max_power_mbm = default_powers[0];

	dev_info(&udev->dev, "Using default power levels (%zu levels)\n",
		 ARRAY_SIZE(default_powers));

	return 0;
}

/* Dynamic channel support querying */
static int wpanusb_query_supported_channels(struct wpanusb *wpanusb)
{
	struct usb_device *udev = wpanusb->udev;
	struct channel_support_query query;
	struct channel_support_response *resp;
	u8 buffer[256];
	int ret, page, i;
	u32 channel_mask;

	dev_dbg(&udev->dev, "Querying supported channels\n");

	/* Initialize channel support to empty */
	memset(wpanusb->supported_channels, 0, sizeof(wpanusb->supported_channels));

	/* Query channels for each page (0 = 2.4GHz, 2 = Sub-GHz) */
	for (page = 0; page <= 2; page++) {
		if (page == 1) continue; /* Skip page 1 (not commonly used) */

		query.page = page;

		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
				      QUERY_SUPPORTED_CHANNELS, USB_TYPE_VENDOR | USB_DIR_OUT,
				      0, 0, &query, sizeof(query),
				      USB_CTRL_SET_TIMEOUT);
		if (ret < 0) {
			dev_dbg(&udev->dev, "Failed to send channel query for page %d: %d\n", page, ret);
			continue;
		}

		ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				      QUERY_SUPPORTED_CHANNELS, USB_TYPE_VENDOR | USB_DIR_IN,
				      0, 0, buffer, sizeof(buffer),
				      USB_CTRL_GET_TIMEOUT);
		
		if (ret < sizeof(struct channel_support_response)) {
			dev_dbg(&udev->dev, "Invalid channel response for page %d: %d\n", page, ret);
			continue;
		}

		resp = (struct channel_support_response *)buffer;
		
		if (resp->page != page) {
			dev_warn(&udev->dev, "Page mismatch in response: expected %d, got %d\n", 
				 page, resp->page);
			continue;
		}

		if (resp->channel_count > 32) {
			dev_warn(&udev->dev, "Too many channels for page %d: %u\n", 
				 page, resp->channel_count);
			continue;
		}

		/* Convert channel list to bitmask */
		channel_mask = 0;
		for (i = 0; i < resp->channel_count; i++) {
			if (resp->channels[i] < 32) {
				channel_mask |= BIT(resp->channels[i]);
			}
		}

		wpanusb->supported_channels[page] = channel_mask;
		wpanusb->channel_pages_supported |= BIT(page);

		dev_info(&udev->dev, "Page %d: %u channels supported (mask: 0x%08x)\n",
			 page, resp->channel_count, channel_mask);
	}

	/* Fallback to default channels if query failed */
	if (wpanusb->channel_pages_supported == 0) {
		dev_info(&udev->dev, "Channel query failed, using defaults\n");
		wpanusb->supported_channels[0] = 0x07FFF800; /* Channels 11-26 for 2.4GHz */
		wpanusb->channel_pages_supported = BIT(0);
	}

	return 0;
}

static const struct ieee802154_ops wpanusb_ops = {
	.owner = THIS_MODULE,
	.xmit_sync = wpanusb_xmit,
	.ed = wpanusb_ed,
	.set_channel = wpanusb_channel,
	.set_cca_mode = wpanusb_set_cca_mode,
	.set_cca_ed_level = wpanusb_set_cca_ed_level,
	.set_csma_params = wpanusb_set_csma_params,
	.set_promiscuous_mode = wpanusb_set_promiscuous_mode,
};

/* ----- Setup ------------------------------------------------------------- */

static int wpanusb_probe(struct usb_interface *interface,
			 const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct ieee802154_hw *hw;
	struct wpanusb *wpanusb;
	int ret;

	hw = ieee802154_alloc_hw(sizeof(struct wpanusb), &wpanusb_ops);
	if (!hw)
		return -ENOMEM;

	wpanusb = hw->priv;
	wpanusb->hw = hw;
	wpanusb->udev = usb_get_dev(udev);
	usb_set_intfdata(interface, wpanusb);

	wpanusb->shutdown = 0;
	INIT_DELAYED_WORK(&wpanusb->work, wpanusb_work_urbs);
	init_usb_anchor(&wpanusb->idle_urbs);
	init_usb_anchor(&wpanusb->rx_urbs);

	ret = wpanusb_alloc_urbs(wpanusb, WPANUSB_NUM_RX_URBS);
	if (ret)
		goto fail;

	wpanusb->tx_dr.bRequestType = VENDOR_OUT;
	wpanusb->tx_dr.bRequest = TX;
	wpanusb->tx_dr.wValue = cpu_to_le16(0);

	wpanusb->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!wpanusb->tx_urb)
		goto fail;

	hw->parent = &udev->dev;
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT |
		    IEEE802154_HW_PROMISCUOUS;

	hw->phy->flags = WPAN_PHY_FLAG_TXPOWER;

	/* Set default and supported channels - will be updated by query */
	hw->phy->current_page = 0;
	hw->phy->current_channel = 11;

	/* Reset device first */
	ret = wpanusb_control_send(wpanusb, usb_sndctrlpipe(udev, 0), RESET,
				   NULL, 0);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to RESET ieee802154");
		goto fail;
	}

	/* Query dynamic capabilities from firmware */
	ret = wpanusb_query_tx_power_levels(wpanusb);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to query power levels");
		goto fail;
	}

	ret = wpanusb_query_supported_channels(wpanusb);
	if (ret < 0) {
		dev_err(&udev->dev, "Failed to query supported channels");
		goto fail;
	}

	/* Update hardware capabilities with queried data */
	hw->phy->supported.tx_powers = wpanusb->supported_powers;
	hw->phy->supported.tx_powers_size = wpanusb->supported_powers_size;
	hw->phy->transmit_power = wpanusb->supported_powers[0];

	/* Set supported channels for all queried pages */
	for (int page = 0; page < 3; page++) {
		if (wpanusb->channel_pages_supported & BIT(page)) {
			hw->phy->supported.channels[page] = wpanusb->supported_channels[page];
		}
	}

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);

	/* Initialize enhanced feature state */
	wpanusb->lbt_config.enabled = false;
	wpanusb->lbt_config.sensing_time_us = 1000;
	wpanusb->lbt_config.threshold_dbm = -70;
	wpanusb->frame_retry_count = 3;
	wpanusb->min_be = 3;
	wpanusb->max_be = 5;
	wpanusb->csma_retries = 3;
	wpanusb->cca_mode = NL802154_CCA_ENERGY;
	wpanusb->cca_ed_level = -7000;
	
	/* Initialize dynamic capability fields */
	wpanusb->supported_powers = NULL;
	wpanusb->supported_powers_size = 0;
	wpanusb->channel_pages_supported = 0;
	wpanusb->min_power_mbm = 0;
	wpanusb->max_power_mbm = 0;
	memset(wpanusb->supported_channels, 0, sizeof(wpanusb->supported_channels));

	ret = ieee802154_register_hw(hw);
	if (ret) {
		dev_err(&udev->dev, "Failed to register ieee802154");
		goto fail;
	}

	dev_info(&udev->dev, "WPANUSB IEEE 802.15.4 device registered with enhanced features\n");

	return 0;

fail:
	/* Cleanup dynamic arrays */
	kfree(wpanusb->supported_powers);
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);
	usb_put_dev(udev);
	ieee802154_free_hw(hw);

	return ret;
}

static void wpanusb_disconnect(struct usb_interface *interface)
{
	struct wpanusb *wpanusb = usb_get_intfdata(interface);

	wpanusb->shutdown = 1;
	cancel_delayed_work_sync(&wpanusb->work);

	usb_kill_anchored_urbs(&wpanusb->rx_urbs);
	wpanusb_free_urbs(wpanusb);
	usb_kill_urb(wpanusb->tx_urb);
	usb_free_urb(wpanusb->tx_urb);

	/* Free dynamic arrays */
	kfree(wpanusb->supported_powers);

	ieee802154_unregister_hw(wpanusb->hw);

	ieee802154_free_hw(wpanusb->hw);

	usb_set_intfdata(interface, NULL);
	usb_put_dev(wpanusb->udev);
}

/* The devices we work with */
static const struct usb_device_id wpanusb_device_table[] = {
	{
		USB_DEVICE_AND_INTERFACE_INFO(WPANUSB_VENDOR_ID,
					      WPANUSB_PRODUCT_ID,
					      USB_CLASS_VENDOR_SPEC,
					      0, 0)
	},
	/* end with null element */
	{}
};
MODULE_DEVICE_TABLE(usb, wpanusb_device_table);

static struct usb_driver wpanusb_driver = {
	.name		= "wpanusb",
	.probe		= wpanusb_probe,
	.disconnect	= wpanusb_disconnect,
	.id_table	= wpanusb_device_table,
};
module_usb_driver(wpanusb_driver);

MODULE_AUTHOR("Andrei Emeltchenko <andrei.emeltchenko@intel.com>");
MODULE_DESCRIPTION("WPANUSB IEEE 802.15.4 over USB Driver");
MODULE_LICENSE("GPL");
