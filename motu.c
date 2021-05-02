// SPDX-License-Identifier: GPL-2.0-only
/*
 * Motu AVB driver
 * Copyright (c) Clemens Ladisch <clemens@ladisch.de> (original UA101/UA1000 driver)
 * Copyright (c) Ralf Beck <ralfbeck1@gmx.de> (adaptation for Motu AVB series)
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/audio.h>
#include <linux/usb/audio-v2.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "usbaudio.h"
#include "midi.h"

MODULE_DESCRIPTION("Motu AVB driver");
MODULE_AUTHOR("Ralf Beck <ralfbeck1@gmx.de>");
MODULE_LICENSE("GPL v2");

/*
 * Should not be lower than the minimum scheduling delay of the host
 * controller.  Some Intel controllers need more than one frame; as long as
 * that driver doesn't tell us about this, use 1.5 frames just to be sure.
 */
#define MIN_QUEUE_LENGTH	2
/* Somewhat random. */
#define MAX_QUEUE_LENGTH	30
/*
 * This magic value optimizes memory usage efficiency for the Motu's packet
 * sizes at all sample rates, taking into account the stupid cache pool sizes
 * that usb_alloc_coherent() uses.
 */
#define DEFAULT_QUEUE_LENGTH	21

#define MAX_PACKET_SIZE		(24*3*25) /* hardware specific */
#define MAX_MEMORY_BUFFERS	DIV_ROUND_UP(MAX_QUEUE_LENGTH, \
					     PAGE_SIZE / MAX_PACKET_SIZE)

#define VENDOR_STRING_ID	1
#define PRODUCT_STRING_ID	2

#define INTCLOCK 1

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

/* module parameters */
static unsigned int queue_length = DEFAULT_QUEUE_LENGTH;
static bool midi = 0;
static unsigned int samplerate = 44100;
static bool vendor = 0;
static unsigned int ins = 0;
static unsigned int outs = 0;

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "card index");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "enable card");
module_param(queue_length, uint, 0644);
MODULE_PARM_DESC(queue_length, "USB queue length in microframes, "
		 __stringify(MIN_QUEUE_LENGTH)"-"__stringify(MAX_QUEUE_LENGTH));
module_param(midi, bool, 0444);
MODULE_PARM_DESC(midi, "device has midi ports");
module_param(samplerate, uint, 0444);
MODULE_PARM_DESC(samplerate, "samplerate");
module_param(vendor, bool, 0444);
MODULE_PARM_DESC(vendor, "vendor");

module_param(ins, uint, 0444);
MODULE_PARM_DESC(ins, "ins");
module_param(outs, uint, 0444);
MODULE_PARM_DESC(outs, "outs");

enum {
        INTF_AUDIOCONTROL,
	INTF_PLAYBACK,
	INTF_CAPTURE,
        INTF_VENDOR,
	INTF_MIDI,
	INTF_VENDOR_OUT,
	INTF_VENDOR_IN,
	INTF_UNUSED,

	INTF_COUNT
};

/* bits in struct motu_avb::states */
enum {
	USB_CAPTURE_RUNNING,
	USB_PLAYBACK_RUNNING,
	ALSA_CAPTURE_OPEN,
	ALSA_PLAYBACK_OPEN,
	ALSA_CAPTURE_RUNNING,
	ALSA_PLAYBACK_RUNNING,
	CAPTURE_URB_COMPLETED,
	PLAYBACK_URB_COMPLETED,
	DISCONNECTED,
};

struct motu_avb {
	struct usb_device *dev;
	struct snd_card *card;
	struct usb_interface *intf[INTF_COUNT];
	int card_index;
	struct snd_pcm *pcm;
	struct list_head midi_list;
	u64 format_bit;
	unsigned int rate;
        bool samplerate_is_set;
	unsigned int packets_per_second;
	spinlock_t lock;
	struct mutex mutex;
	unsigned long states;

	/* FIFO to synchronize playback rate to capture rate */
	unsigned int rate_feedback_start;
	unsigned int rate_feedback_count;
	u8 rate_feedback[MAX_QUEUE_LENGTH];

	struct list_head ready_playback_urbs;
	struct tasklet_struct playback_tasklet;
	wait_queue_head_t alsa_capture_wait;
	wait_queue_head_t rate_feedback_wait;
	wait_queue_head_t alsa_playback_wait;
	struct motu_avb_stream {
		struct snd_pcm_substream *substream;
		unsigned int usb_pipe;
		unsigned int channels;
		unsigned int frame_bytes;
		unsigned int max_packet_bytes;
		unsigned int period_pos;
		unsigned int buffer_pos;
		unsigned int queue_length;
		struct motu_avb_urb {
			struct urb urb;
			struct usb_iso_packet_descriptor iso_frame_desc[1];
			struct list_head ready_list;
		} *urbs[MAX_QUEUE_LENGTH];
		struct {
			unsigned int size;
			void *addr;
			dma_addr_t dma;
		} buffers[MAX_MEMORY_BUFFERS];
	} capture, playback;
};

static DEFINE_MUTEX(devices_mutex);
static unsigned int devices_used;
static struct usb_driver motu_avb_driver;

static void abort_alsa_playback(struct motu_avb *ua);
static void abort_alsa_capture(struct motu_avb *ua);

static void set_samplerate(struct motu_avb *ua)
{
        __le32 data;
/*        unsigned char data1; */
        int err;
/*        int count = 100; */

	if (ua->samplerate_is_set)
		return;

        ua->samplerate_is_set = true;

        data = cpu_to_le32(ua->rate);
	err = usb_control_msg(ua->dev, usb_sndctrlpipe(ua->dev, 0), UAC2_CS_CUR,
			      USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			      UAC2_CS_CONTROL_SAM_FREQ << 8,
			      0 | (INTCLOCK << 8),
			      &data, sizeof(data),
			      USB_CTRL_SET_TIMEOUT);
	if (err < 0)
		dev_warn(&ua->dev->dev,
				 "%s(): cannot set samplerate %d\n",
				   __func__,  ua->rate);

        dev_warn(&ua->dev->dev, "Motu driver 0.1\n");

        msleep(500);
/*
	do {
		msleep(100);

		err = usb_control_msg(ua->dev, usb_rcvctrlpipe(ua->dev, 0), UAC2_CS_CUR,
				      	USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
					UAC2_CS_CONTROL_CLOCK_VALID << 8,
					0 | (INTCLOCK << 8),
				        &data1, sizeof(data1),
					USB_CTRL_GET_TIMEOUT);
		if (err < 0) {
			dev_warn(&ua->dev->dev,
				 "%s(): cannot get clock validity for id %d, error = %d\n",
				   __func__,  data1, err);
		}
	}
	while (count--);
*/
}

static const char *usb_error_string(int err)
{
	switch (err) {
	case -ENODEV:
		return "no device";
	case -ENOENT:
		return "endpoint not enabled";
	case -EPIPE:
		return "endpoint stalled";
	case -ENOSPC:
		return "not enough bandwidth";
	case -ESHUTDOWN:
		return "device disabled";
	case -EHOSTUNREACH:
		return "device suspended";
	case -EINVAL:
	case -EAGAIN:
	case -EFBIG:
	case -EMSGSIZE:
		return "internal error";
	default:
		return "unknown error";
	}
}

static void abort_usb_capture(struct motu_avb *ua)
{
	if (test_and_clear_bit(USB_CAPTURE_RUNNING, &ua->states)) {
		wake_up(&ua->alsa_capture_wait);
		wake_up(&ua->rate_feedback_wait);
	}
}

static void abort_usb_playback(struct motu_avb *ua)
{
	if (test_and_clear_bit(USB_PLAYBACK_RUNNING, &ua->states))
		wake_up(&ua->alsa_playback_wait);
}

static void playback_urb_complete(struct urb *usb_urb)
{
	struct motu_avb_urb *urb = (struct motu_avb_urb *)usb_urb;
	struct motu_avb *ua = urb->urb.context;
	unsigned long flags;

	if (unlikely(urb->urb.status == -ENOENT ||	/* unlinked */
		     urb->urb.status == -ENODEV ||	/* device removed */
		     urb->urb.status == -ECONNRESET ||	/* unlinked */
		     urb->urb.status == -ESHUTDOWN)) {	/* device disabled */
		abort_usb_playback(ua);
		abort_alsa_playback(ua);
		return;
	}

	if (test_bit(USB_PLAYBACK_RUNNING, &ua->states)) {
		/* append URB to FIFO */
		spin_lock_irqsave(&ua->lock, flags);
		list_add_tail(&urb->ready_list, &ua->ready_playback_urbs);
		if (ua->rate_feedback_count > 0)
			tasklet_schedule(&ua->playback_tasklet);
		ua->playback.substream->runtime->delay -=
				urb->urb.iso_frame_desc[0].length /
						ua->playback.frame_bytes;
		spin_unlock_irqrestore(&ua->lock, flags);
	}
}

static void first_playback_urb_complete(struct urb *urb)
{
	struct motu_avb *ua = urb->context;

	urb->complete = playback_urb_complete;
	playback_urb_complete(urb);

	set_bit(PLAYBACK_URB_COMPLETED, &ua->states);
	wake_up(&ua->alsa_playback_wait);
}

/* copy data from the ALSA ring buffer into the URB buffer */
static bool copy_playback_data(struct motu_avb_stream *stream, struct urb *urb,
			       unsigned int frames)
{
	struct snd_pcm_runtime *runtime;
	unsigned int frame_bytes, frames1;
	const u8 *source;

	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;
	source = runtime->dma_area + stream->buffer_pos * frame_bytes;
	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		memcpy(urb->transfer_buffer, source, frames * frame_bytes);
	} else {
		/* wrap around at end of ring buffer */
		frames1 = runtime->buffer_size - stream->buffer_pos;
		memcpy(urb->transfer_buffer, source, frames1 * frame_bytes);
		memcpy(urb->transfer_buffer + frames1 * frame_bytes,
		       runtime->dma_area, (frames - frames1) * frame_bytes);
	}

	stream->buffer_pos += frames;
	if (stream->buffer_pos >= runtime->buffer_size)
		stream->buffer_pos -= runtime->buffer_size;
	stream->period_pos += frames;
	if (stream->period_pos >= runtime->period_size) {
		stream->period_pos -= runtime->period_size;
		return true;
	}
	return false;
}

static inline void add_with_wraparound(struct motu_avb *ua,
				       unsigned int *value, unsigned int add)
{
	*value += add;
	if (*value >= ua->playback.queue_length)
		*value -= ua->playback.queue_length;
}

static void playback_tasklet(unsigned long data)
{
	struct motu_avb *ua = (void *)data;
	unsigned long flags;
	unsigned int frames;
	struct motu_avb_urb *urb;
	bool do_period_elapsed = false;
	int err;

	if (unlikely(!test_bit(USB_PLAYBACK_RUNNING, &ua->states)))
		return;

	/*
	 * Synchronizing the playback rate to the capture rate is done by using
	 * the same sequence of packet sizes for both streams.
	 * Submitting a playback URB therefore requires both a ready URB and
	 * the size of the corresponding capture packet, i.e., both playback
	 * and capture URBs must have been completed.  Since the USB core does
	 * not guarantee that playback and capture complete callbacks are
	 * called alternately, we use two FIFOs for packet sizes and read URBs;
	 * submitting playback URBs is possible as long as both FIFOs are
	 * nonempty.
	 */
	spin_lock_irqsave(&ua->lock, flags);
	while (ua->rate_feedback_count > 0 &&
	       !list_empty(&ua->ready_playback_urbs)) {
		/* take packet size out of FIFO */
		frames = ua->rate_feedback[ua->rate_feedback_start];
		add_with_wraparound(ua, &ua->rate_feedback_start, 1);
		ua->rate_feedback_count--;

		/* take URB out of FIFO */
		urb = list_first_entry(&ua->ready_playback_urbs,
				       struct motu_avb_urb, ready_list);
		list_del(&urb->ready_list);

		/* fill packet with data or silence */
		urb->urb.iso_frame_desc[0].length =
			frames * ua->playback.frame_bytes;
		if (test_bit(ALSA_PLAYBACK_RUNNING, &ua->states))
			do_period_elapsed |= copy_playback_data(&ua->playback,
								&urb->urb,
								frames);
		else
			memset(urb->urb.transfer_buffer, 0,
			       urb->urb.iso_frame_desc[0].length);

		/* and off you go ... */
		err = usb_submit_urb(&urb->urb, GFP_ATOMIC);
		if (unlikely(err < 0)) {
			spin_unlock_irqrestore(&ua->lock, flags);
			abort_usb_playback(ua);
			abort_alsa_playback(ua);
			dev_err(&ua->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			return;
		}
		ua->playback.substream->runtime->delay += frames;
	}
	spin_unlock_irqrestore(&ua->lock, flags);
	if (do_period_elapsed)
		snd_pcm_period_elapsed(ua->playback.substream);
}

/* copy data from the URB buffer into the ALSA ring buffer */
static bool copy_capture_data(struct motu_avb_stream *stream, struct urb *urb,
			      unsigned int frames)
{
	struct snd_pcm_runtime *runtime;
	unsigned int frame_bytes, frames1;
	u8 *dest;

	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;
	dest = runtime->dma_area + stream->buffer_pos * frame_bytes;
	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		memcpy(dest, urb->transfer_buffer, frames * frame_bytes);
	} else {
		/* wrap around at end of ring buffer */
		frames1 = runtime->buffer_size - stream->buffer_pos;
		memcpy(dest, urb->transfer_buffer, frames1 * frame_bytes);
		memcpy(runtime->dma_area,
		       urb->transfer_buffer + frames1 * frame_bytes,
		       (frames - frames1) * frame_bytes);
	}

	stream->buffer_pos += frames;
	if (stream->buffer_pos >= runtime->buffer_size)
		stream->buffer_pos -= runtime->buffer_size;
	stream->period_pos += frames;
	if (stream->period_pos >= runtime->period_size) {
		stream->period_pos -= runtime->period_size;
		return true;
	}
	return false;
}

static void capture_urb_complete(struct urb *urb)
{
	struct motu_avb *ua = urb->context;
	struct motu_avb_stream *stream = &ua->capture;
	unsigned long flags;
	unsigned int frames, write_ptr;
	bool do_period_elapsed;
	int err;

	if (unlikely(urb->status == -ENOENT ||		/* unlinked */
		     urb->status == -ENODEV ||		/* device removed */
		     urb->status == -ECONNRESET ||	/* unlinked */
		     urb->status == -ESHUTDOWN))	/* device disabled */
		goto stream_stopped;

	if (urb->status >= 0 && urb->iso_frame_desc[0].status >= 0)
		frames = urb->iso_frame_desc[0].actual_length /
			stream->frame_bytes;
	else
		frames = 0;

	spin_lock_irqsave(&ua->lock, flags);

	if (frames > 0 && test_bit(ALSA_CAPTURE_RUNNING, &ua->states))
		do_period_elapsed = copy_capture_data(stream, urb, frames);
	else
		do_period_elapsed = false;

	if (test_bit(USB_CAPTURE_RUNNING, &ua->states)) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (unlikely(err < 0)) {
			spin_unlock_irqrestore(&ua->lock, flags);
			dev_err(&ua->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			goto stream_stopped;
		}

		/* append packet size to FIFO */
		write_ptr = ua->rate_feedback_start;
		add_with_wraparound(ua, &write_ptr, ua->rate_feedback_count);
		ua->rate_feedback[write_ptr] = frames;
		if (ua->rate_feedback_count < ua->playback.queue_length) {
			ua->rate_feedback_count++;
			if (ua->rate_feedback_count ==
						ua->playback.queue_length)
				wake_up(&ua->rate_feedback_wait);
		} else {
			/*
			 * Ring buffer overflow; this happens when the playback
			 * stream is not running.  Throw away the oldest entry,
			 * so that the playback stream, when it starts, sees
			 * the most recent packet sizes.
			 */
			add_with_wraparound(ua, &ua->rate_feedback_start, 1);
		}
		if (test_bit(USB_PLAYBACK_RUNNING, &ua->states) &&
		    !list_empty(&ua->ready_playback_urbs))
			tasklet_schedule(&ua->playback_tasklet);
	}

	spin_unlock_irqrestore(&ua->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(stream->substream);

	return;

stream_stopped:
	abort_usb_playback(ua);
	abort_usb_capture(ua);
	abort_alsa_playback(ua);
	abort_alsa_capture(ua);
}

static void first_capture_urb_complete(struct urb *urb)
{
	struct motu_avb *ua = urb->context;

	urb->complete = capture_urb_complete;
	capture_urb_complete(urb);

	set_bit(CAPTURE_URB_COMPLETED, &ua->states);
	wake_up(&ua->alsa_capture_wait);
}

static int submit_stream_urbs(struct motu_avb *ua, struct motu_avb_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->queue_length; ++i) {
		int err = usb_submit_urb(&stream->urbs[i]->urb, GFP_KERNEL);
		if (err < 0) {
			dev_err(&ua->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			return err;
		}
	}
	return 0;
}

static void kill_stream_urbs(struct motu_avb_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->queue_length; ++i)
		if (stream->urbs[i])
			usb_kill_urb(&stream->urbs[i]->urb);
}

static int enable_iso_interface(struct motu_avb *ua, unsigned int intf_index)
{
	struct usb_host_interface *alts;

	alts = ua->intf[intf_index]->cur_altsetting;
	if (alts->desc.bAlternateSetting != 1) {
		int err = usb_set_interface(ua->dev,
					    alts->desc.bInterfaceNumber, 1);
		if (err < 0) {
			dev_err(&ua->dev->dev,
				"cannot initialize interface; error %d: %s\n",
				err, usb_error_string(err));
			return err;
		}
	}
	return 0;
}

static void disable_iso_interface(struct motu_avb *ua, unsigned int intf_index)
{
	struct usb_host_interface *alts;

	if (!ua->intf[intf_index])
		return;

	alts = ua->intf[intf_index]->cur_altsetting;
	if (alts->desc.bAlternateSetting != 0) {
		int err = usb_set_interface(ua->dev,
					    alts->desc.bInterfaceNumber, 0);
		if (err < 0 && !test_bit(DISCONNECTED, &ua->states))
			dev_warn(&ua->dev->dev,
				 "interface reset failed; error %d: %s\n",
				 err, usb_error_string(err));
	}
}

static void stop_usb_capture(struct motu_avb *ua)
{
	clear_bit(USB_CAPTURE_RUNNING, &ua->states);

	kill_stream_urbs(&ua->capture);

	if (vendor)
	{
		disable_iso_interface(ua, INTF_VENDOR_IN);
	}
	else
	{
		disable_iso_interface(ua, INTF_CAPTURE);
	}
}

static int start_usb_capture(struct motu_avb *ua)
{
	int err;

	if (test_bit(DISCONNECTED, &ua->states))
		return -ENODEV;

	if (test_bit(USB_CAPTURE_RUNNING, &ua->states))
		return 0;

	kill_stream_urbs(&ua->capture);

	if (vendor)
	{
		enable_iso_interface(ua, INTF_VENDOR_IN);
	}
	else
	{
		enable_iso_interface(ua, INTF_CAPTURE);
	}
	if (err < 0)
		return err;

	clear_bit(CAPTURE_URB_COMPLETED, &ua->states);
	ua->capture.urbs[0]->urb.complete = first_capture_urb_complete;
	ua->rate_feedback_start = 0;
	ua->rate_feedback_count = 0;

	set_bit(USB_CAPTURE_RUNNING, &ua->states);
	err = submit_stream_urbs(ua, &ua->capture);
	if (err < 0)
		stop_usb_capture(ua);
	return err;
}

static void stop_usb_playback(struct motu_avb *ua)
{
	clear_bit(USB_PLAYBACK_RUNNING, &ua->states);

	kill_stream_urbs(&ua->playback);

	tasklet_kill(&ua->playback_tasklet);

	if (vendor)
	{
		disable_iso_interface(ua, INTF_VENDOR_OUT);
	}
	else
	{
		disable_iso_interface(ua, INTF_PLAYBACK);
	}
}

static int start_usb_playback(struct motu_avb *ua)
{
	unsigned int i, frames;
	struct urb *urb;
	int err = 0;

	if (test_bit(DISCONNECTED, &ua->states))
		return -ENODEV;

	clear_bit(USB_PLAYBACK_RUNNING, &ua->states);

	kill_stream_urbs(&ua->playback);
	tasklet_kill(&ua->playback_tasklet);

	if (vendor)
	{
		disable_iso_interface(ua, INTF_VENDOR_OUT);
	}
	else
	{
		disable_iso_interface(ua, INTF_PLAYBACK);
	}
	if (err < 0)
		return err;

	clear_bit(PLAYBACK_URB_COMPLETED, &ua->states);
	ua->playback.urbs[0]->urb.complete =
		first_playback_urb_complete;
	spin_lock_irq(&ua->lock);
	INIT_LIST_HEAD(&ua->ready_playback_urbs);

        /* reset rate feedback */
	ua->rate_feedback_start = 0;
	ua->rate_feedback_count = 0;
	spin_unlock_irq(&ua->lock);

	if (vendor)
	{
		enable_iso_interface(ua, INTF_VENDOR_OUT);
	}
	else
	{
		enable_iso_interface(ua, INTF_PLAYBACK);
	}
	if (err < 0)
		return err;

	/*
	 * We submit the initial URBs all at once, so we have to wait for the
	 * packet size FIFO to be full.
	 */
	wait_event(ua->rate_feedback_wait,
		   ua->rate_feedback_count >= ua->playback.queue_length ||
		   !test_bit(USB_CAPTURE_RUNNING, &ua->states) ||
		   test_bit(DISCONNECTED, &ua->states));
	if (test_bit(DISCONNECTED, &ua->states)) {
		stop_usb_playback(ua);
		return -ENODEV;
	}
	if (!test_bit(USB_CAPTURE_RUNNING, &ua->states)) {
		stop_usb_playback(ua);
		return -EIO;
	}

        // the very first urb must have the maximum number of frames 

        spin_lock_irq(&ua->lock);
       
        switch (ua->rate)
        {
        case 44100:
           frames = 6;
           break;
        case 48000:
           frames = 7;
           break;
        case 88200:
           frames = 12;
           break;
        case 96000:
           frames = 13;
           break;
        case 172400:
           frames = 24;
           break;
        case 192000:
           frames = 25;
           break;
        }

        ua->rate_feedback_count--;

	spin_unlock_irq(&ua->lock);

	urb = &ua->playback.urbs[0]->urb;
	urb->iso_frame_desc[0].length =
		frames * ua->playback.frame_bytes;
	memset(urb->transfer_buffer, 0,
	       urb->iso_frame_desc[0].length);

	for (i = 1; i < ua->playback.queue_length; ++i) {
		/* all initial URBs contain silence */
		spin_lock_irq(&ua->lock);
		frames = ua->rate_feedback[ua->rate_feedback_start];
		add_with_wraparound(ua, &ua->rate_feedback_start, 1);
		ua->rate_feedback_count--;
		spin_unlock_irq(&ua->lock);
		urb = &ua->playback.urbs[i]->urb;
		urb->iso_frame_desc[0].length =
			frames * ua->playback.frame_bytes;
		memset(urb->transfer_buffer, 0,
		       urb->iso_frame_desc[0].length);
	}

	set_bit(USB_PLAYBACK_RUNNING, &ua->states);
	err = submit_stream_urbs(ua, &ua->playback);
	if (err < 0)
		stop_usb_playback(ua);
	return err;
}

static void abort_alsa_capture(struct motu_avb *ua)
{
	if (test_bit(ALSA_CAPTURE_RUNNING, &ua->states))
		snd_pcm_stop_xrun(ua->capture.substream);
}

static void abort_alsa_playback(struct motu_avb *ua)
{
	if (test_bit(ALSA_PLAYBACK_RUNNING, &ua->states))
		snd_pcm_stop_xrun(ua->playback.substream);
}

static int set_stream_hw(struct motu_avb *ua, struct snd_pcm_substream *substream,
			 unsigned int channels)
{
	int err;

	substream->runtime->hw.info =
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_FIFO_IN_FRAMES;
	substream->runtime->hw.formats = SNDRV_PCM_FMTBIT_S24_3LE;
	substream->runtime->hw.rates = snd_pcm_rate_to_rate_bit(ua->rate);
	substream->runtime->hw.rate_min = ua->rate;
	substream->runtime->hw.rate_max = ua->rate;
	substream->runtime->hw.channels_min = channels;
	substream->runtime->hw.channels_max = channels;
	substream->runtime->hw.buffer_bytes_max = 45000 * 1024;
	substream->runtime->hw.period_bytes_min = 1;
	substream->runtime->hw.period_bytes_max = UINT_MAX;
	substream->runtime->hw.periods_min = 2;
	substream->runtime->hw.periods_max = UINT_MAX;
	err = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_PERIOD_TIME,
					   1500000 / ua->packets_per_second,
					   UINT_MAX);
	if (err < 0)
		return err;
	err = snd_pcm_hw_constraint_msbits(substream->runtime, 0, 32, 24);
	return err;
}

static int capture_pcm_open(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;
	int err;

        dev_err(&ua->dev->dev, "capture_open\n");

        set_samplerate(ua);

	ua->capture.substream = substream;
	err = set_stream_hw(ua, substream, ua->capture.channels);
	if (err < 0)
		return err;
	substream->runtime->hw.fifo_size =
		DIV_ROUND_CLOSEST(ua->rate, ua->packets_per_second);
	substream->runtime->delay = substream->runtime->hw.fifo_size;

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	if (err >= 0)
		set_bit(ALSA_CAPTURE_OPEN, &ua->states);
	mutex_unlock(&ua->mutex);
	return err;
}

static int playback_pcm_open(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;
	int err;

        dev_err(&ua->dev->dev, "playback_open\n");

        set_samplerate(ua);

	ua->playback.substream = substream;
	err = set_stream_hw(ua, substream, ua->playback.channels);
	if (err < 0)
		return err;
	substream->runtime->hw.fifo_size =
		DIV_ROUND_CLOSEST(ua->rate * ua->playback.queue_length,
				  ua->packets_per_second);

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	if (err < 0)
		goto error;
	err = start_usb_playback(ua);
	if (err < 0) {
		if (!test_bit(ALSA_CAPTURE_OPEN, &ua->states))
			stop_usb_capture(ua);
		goto error;
	}
	set_bit(ALSA_PLAYBACK_OPEN, &ua->states);
error:
	mutex_unlock(&ua->mutex);
	return err;
}

static int capture_pcm_close(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;

	mutex_lock(&ua->mutex);
	clear_bit(ALSA_CAPTURE_OPEN, &ua->states);
	if (!test_bit(ALSA_PLAYBACK_OPEN, &ua->states))
		stop_usb_capture(ua);
	mutex_unlock(&ua->mutex);
	return 0;
}

static int playback_pcm_close(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;

	mutex_lock(&ua->mutex);
	stop_usb_playback(ua);
	clear_bit(ALSA_PLAYBACK_OPEN, &ua->states);
	if (!test_bit(ALSA_CAPTURE_OPEN, &ua->states))
		stop_usb_capture(ua);
	mutex_unlock(&ua->mutex);
	return 0;
}


static int capture_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	struct motu_avb *ua = substream->private_data;
	int err = 0;

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	mutex_unlock(&ua->mutex);
	return err;
}

static int playback_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct motu_avb *ua = substream->private_data;
	int err = 0;

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	if (err >= 0)
		err = start_usb_playback(ua);
	mutex_unlock(&ua->mutex);
	return err;
}

static int capture_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;
	int err;

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	mutex_unlock(&ua->mutex);
	if (err < 0)
		return err;

	/*
	 * The EHCI driver schedules the first packet of an iso stream at 10 ms
	 * in the future, i.e., no data is actually captured for that long.
	 * Take the wait here so that the stream is known to be actually
	 * running when the start trigger has been called.
	 */
	wait_event(ua->alsa_capture_wait,
		   test_bit(CAPTURE_URB_COMPLETED, &ua->states) ||
		   !test_bit(USB_CAPTURE_RUNNING, &ua->states));
	if (test_bit(DISCONNECTED, &ua->states))
		return -ENODEV;
	if (!test_bit(USB_CAPTURE_RUNNING, &ua->states))
		return -EIO;

	ua->capture.period_pos = 0;
	ua->capture.buffer_pos = 0;
	return 0;
}

static int playback_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct motu_avb *ua = substream->private_data;
	int err;

	mutex_lock(&ua->mutex);
	err = start_usb_capture(ua);
	if (err >= 0)
		err = start_usb_playback(ua);
	mutex_unlock(&ua->mutex);
	if (err < 0)
		return err;

	/* see the comment in capture_pcm_prepare() */
	wait_event(ua->alsa_playback_wait,
		   test_bit(PLAYBACK_URB_COMPLETED, &ua->states) ||
		   !test_bit(USB_PLAYBACK_RUNNING, &ua->states));
	if (test_bit(DISCONNECTED, &ua->states))
		return -ENODEV;
	if (!test_bit(USB_PLAYBACK_RUNNING, &ua->states))
		return -EIO;

	substream->runtime->delay = 0;
	ua->playback.period_pos = 0;
	ua->playback.buffer_pos = 0;
	return 0;
}

static int capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct motu_avb *ua = substream->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (!test_bit(USB_CAPTURE_RUNNING, &ua->states))
			return -EIO;
		set_bit(ALSA_CAPTURE_RUNNING, &ua->states);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		clear_bit(ALSA_CAPTURE_RUNNING, &ua->states);
		return 0;
	default:
		return -EINVAL;
	}
}

static int playback_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct motu_avb *ua = substream->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (!test_bit(USB_PLAYBACK_RUNNING, &ua->states))
			return -EIO;
		set_bit(ALSA_PLAYBACK_RUNNING, &ua->states);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		clear_bit(ALSA_PLAYBACK_RUNNING, &ua->states);
		return 0;
	default:
		return -EINVAL;
	}
}

static inline snd_pcm_uframes_t motu_avb_pcm_pointer(struct motu_avb *ua,
						  struct motu_avb_stream *stream)
{
	unsigned long flags;
	unsigned int pos;

	spin_lock_irqsave(&ua->lock, flags);
	pos = stream->buffer_pos;
	spin_unlock_irqrestore(&ua->lock, flags);
	return pos;
}

static snd_pcm_uframes_t capture_pcm_pointer(struct snd_pcm_substream *subs)
{
	struct motu_avb *ua = subs->private_data;

	return motu_avb_pcm_pointer(ua, &ua->capture);
}

static snd_pcm_uframes_t playback_pcm_pointer(struct snd_pcm_substream *subs)
{
	struct motu_avb *ua = subs->private_data;

	return motu_avb_pcm_pointer(ua, &ua->playback);
}

static const struct snd_pcm_ops capture_pcm_ops = {
	.open = capture_pcm_open,
	.close = capture_pcm_close,
	.hw_params = capture_pcm_hw_params,
	.prepare = capture_pcm_prepare,
	.trigger = capture_pcm_trigger,
	.pointer = capture_pcm_pointer,
};

static const struct snd_pcm_ops playback_pcm_ops = {
	.open = playback_pcm_open,
	.close = playback_pcm_close,
	.hw_params = playback_pcm_hw_params,
	.prepare = playback_pcm_prepare,
	.trigger = playback_pcm_trigger,
	.pointer = playback_pcm_pointer,
};

static int detect_usb_format(struct motu_avb *ua)
{
        const struct usb_endpoint_descriptor *epd;

        ua->format_bit = SNDRV_PCM_FMTBIT_S24_3LE;
        ua->rate = samplerate;
	ua->packets_per_second = 8000;

	if (vendor && (samplerate <= 48000))
	{
		ua->capture.channels = 64;
		ua->playback.channels = 64;
        }
        else if (vendor && (samplerate <= 96000))
        {
		ua->capture.channels = 32;
		ua->playback.channels = 32;
        }
        else
        {
		ua->capture.channels = 24;
		ua->playback.channels = 24;
        }

        if (ins > 0)
        	ua->capture.channels = ins;

        if (outs > 0)
        	ua->playback.channels = outs;

	ua->capture.frame_bytes = ua->capture.channels*3;
	ua->playback.frame_bytes = ua->playback.channels*3;

	if (vendor)
	{
		epd = &ua->intf[INTF_VENDOR_IN]->altsetting[1].endpoint[0].desc;
	}
	else
	{
		epd = &ua->intf[INTF_CAPTURE]->altsetting[1].endpoint[0].desc;
	}

	if (!usb_endpoint_is_isoc_in(epd)) {
		dev_err(&ua->dev->dev, "invalid capture endpoint\n");
		return -ENXIO;
	}
	ua->capture.usb_pipe = usb_rcvisocpipe(ua->dev, usb_endpoint_num(epd));
	ua->capture.max_packet_bytes = usb_endpoint_maxp_mult(epd)*usb_endpoint_maxp(epd);

        dev_err(&ua->dev->dev, "max packets capture endpoint %d\n", ua->capture.max_packet_bytes);

	if (vendor)
	{
		epd = &ua->intf[INTF_VENDOR_OUT]->altsetting[1].endpoint[0].desc;
	}
	else
	{
		epd = &ua->intf[INTF_PLAYBACK]->altsetting[1].endpoint[0].desc;
	}

	if (!usb_endpoint_is_isoc_out(epd)) {
		dev_err(&ua->dev->dev, "invalid playback endpoint\n");
		return -ENXIO;
	}
	ua->playback.usb_pipe = usb_sndisocpipe(ua->dev, usb_endpoint_num(epd));
	ua->playback.max_packet_bytes = usb_endpoint_maxp_mult(epd)*usb_endpoint_maxp(epd);

        dev_err(&ua->dev->dev, "max packets playback endpoint %d\n", ua->playback.max_packet_bytes);

	return 0;
}

static int alloc_stream_buffers(struct motu_avb *ua, struct motu_avb_stream *stream)
{
	unsigned int remaining_packets, packets, packets_per_page, i;
	size_t size;

	stream->queue_length = queue_length;
	stream->queue_length = max(stream->queue_length,
				   (unsigned int)MIN_QUEUE_LENGTH);
	stream->queue_length = min(stream->queue_length,
				   (unsigned int)MAX_QUEUE_LENGTH);

	/*
	 * The cache pool sizes used by usb_alloc_coherent() (128, 512, 2048) are
	 * quite bad when used with the packet sizes of this device (e.g. 280,
	 * 520, 624).  Therefore, we allocate and subdivide entire pages, using
	 * a smaller buffer only for the last chunk.
	 */
	remaining_packets = stream->queue_length;
	packets_per_page = PAGE_SIZE / stream->max_packet_bytes;
	for (i = 0; i < ARRAY_SIZE(stream->buffers); ++i) {
		packets = min(remaining_packets, packets_per_page);
		size = packets * stream->max_packet_bytes;
		stream->buffers[i].addr =
			usb_alloc_coherent(ua->dev, size, GFP_KERNEL,
					   &stream->buffers[i].dma);
		if (!stream->buffers[i].addr)
			return -ENOMEM;
		stream->buffers[i].size = size;
		remaining_packets -= packets;
		if (!remaining_packets)
			break;
	}
	if (remaining_packets) {
		dev_err(&ua->dev->dev, "too many packets\n");
		return -ENXIO;
	}
	return 0;
}

static void free_stream_buffers(struct motu_avb *ua, struct motu_avb_stream *stream)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(stream->buffers); ++i)
		usb_free_coherent(ua->dev,
				  stream->buffers[i].size,
				  stream->buffers[i].addr,
				  stream->buffers[i].dma);
}

static int alloc_stream_urbs(struct motu_avb *ua, struct motu_avb_stream *stream,
			     void (*urb_complete)(struct urb *))
{
	unsigned max_packet_size = stream->max_packet_bytes;
	struct motu_avb_urb *urb;
	unsigned int b, u = 0;

	for (b = 0; b < ARRAY_SIZE(stream->buffers); ++b) {
		unsigned int size = stream->buffers[b].size;
		u8 *addr = stream->buffers[b].addr;
		dma_addr_t dma = stream->buffers[b].dma;

		while (size >= max_packet_size) {
			if (u >= stream->queue_length)
				goto bufsize_error;
			urb = kmalloc(sizeof(*urb), GFP_KERNEL);
			if (!urb)
				return -ENOMEM;
			usb_init_urb(&urb->urb);
			urb->urb.dev = ua->dev;
			urb->urb.pipe = stream->usb_pipe;
			urb->urb.transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			urb->urb.transfer_buffer = addr;
			urb->urb.transfer_dma = dma;
			urb->urb.transfer_buffer_length = max_packet_size;
			urb->urb.number_of_packets = 1;
			urb->urb.interval = 1;
			urb->urb.context = ua;
			urb->urb.complete = urb_complete;
			urb->urb.iso_frame_desc[0].offset = 0;
			urb->urb.iso_frame_desc[0].length = max_packet_size;
			stream->urbs[u++] = urb;
			size -= max_packet_size;
			addr += max_packet_size;
			dma += max_packet_size;
		}
	}
	if (u == stream->queue_length)
		return 0;
bufsize_error:
	dev_err(&ua->dev->dev, "internal buffer size error\n");
	return -ENXIO;
}

static void free_stream_urbs(struct motu_avb_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->queue_length; ++i) {
		kfree(stream->urbs[i]);
		stream->urbs[i] = NULL;
	}
}

static void free_usb_related_resources(struct motu_avb *ua,
				       struct usb_interface *interface)
{
	unsigned int i;
	struct usb_interface *intf;

	mutex_lock(&ua->mutex);
	free_stream_urbs(&ua->capture);
	free_stream_urbs(&ua->playback);
	mutex_unlock(&ua->mutex);
	free_stream_buffers(ua, &ua->capture);
	free_stream_buffers(ua, &ua->playback);

	for (i = 0; i < ARRAY_SIZE(ua->intf); ++i) {
		mutex_lock(&ua->mutex);
		intf = ua->intf[i];
		ua->intf[i] = NULL;
		mutex_unlock(&ua->mutex);
		if (intf) {
			usb_set_intfdata(intf, NULL);
			if (intf != interface)
				usb_driver_release_interface(&motu_avb_driver,
							     intf);
		}
	}
}

static void motu_avb_card_free(struct snd_card *card)
{
	struct motu_avb *ua = card->private_data;

	mutex_destroy(&ua->mutex);
}

static int motu_avb_probe(struct usb_interface *interface,
		       const struct usb_device_id *usb_id)
{
	static const struct snd_usb_midi_endpoint_info midi_ep = {
		.out_cables = 0x0001,
		.in_cables = 0x0001
	};
	static const struct snd_usb_audio_quirk midi_quirk = {
		.type = QUIRK_MIDI_FIXED_ENDPOINT,
		.data = &midi_ep
	};
	static const int intf_numbers[2][8] = {
		{	/* AVB devices without MIDI */
                        [INTF_AUDIOCONTROL] = 0,
			[INTF_PLAYBACK] = 1,
			[INTF_CAPTURE] = 2,
                        [INTF_VENDOR] = 3,
			[INTF_VENDOR_OUT] = 4,
			[INTF_VENDOR_IN] = 5,
			[INTF_UNUSED] = -1,
			[INTF_MIDI] = -1,
		},
		{	/* AVB devices with MIDI */
                        [INTF_AUDIOCONTROL] = 0,
			[INTF_PLAYBACK] = 1,
			[INTF_CAPTURE] = 2,
			[INTF_UNUSED] = 3,
			[INTF_MIDI] = 4,
                        [INTF_VENDOR] = 5,
			[INTF_VENDOR_OUT] = 6,
			[INTF_VENDOR_IN] = 7,
		},
	};
	struct snd_card *card;
	struct motu_avb *ua;
	unsigned int card_index, i;
	const char *name;
	char usb_path[32];
	int err;

	if (interface->altsetting->desc.bInterfaceNumber !=
	    intf_numbers[midi][0])
		return -ENODEV;

	mutex_lock(&devices_mutex);

	for (card_index = 0; card_index < SNDRV_CARDS; ++card_index)
		if (enable[card_index] && !(devices_used & (1 << card_index)))
			break;
	if (card_index >= SNDRV_CARDS) {
		mutex_unlock(&devices_mutex);
		return -ENOENT;
	}
	err = snd_card_new(&interface->dev,
			   index[card_index], id[card_index], THIS_MODULE,
			   sizeof(*ua), &card);
	if (err < 0) {
		mutex_unlock(&devices_mutex);
		return err;
	}
	card->private_free = motu_avb_card_free;
	ua = card->private_data;
	ua->dev = interface_to_usbdev(interface);
	ua->card = card;
	ua->card_index = card_index;
        ua->samplerate_is_set = false;
	INIT_LIST_HEAD(&ua->midi_list);
	spin_lock_init(&ua->lock);
	mutex_init(&ua->mutex);
	INIT_LIST_HEAD(&ua->ready_playback_urbs);
	tasklet_init(&ua->playback_tasklet,
		     playback_tasklet, (unsigned long)ua);
	init_waitqueue_head(&ua->alsa_capture_wait);
	init_waitqueue_head(&ua->rate_feedback_wait);
	init_waitqueue_head(&ua->alsa_playback_wait);

        dev_info(&ua->dev->dev, "samplerate = %d, queue_length = %d, midi = %d, vendor = %d\n", samplerate, queue_length, midi, vendor);

	ua->intf[0] = interface;
	for (i = 1; i < ARRAY_SIZE(ua->intf); ++i) {
		dev_info(&ua->dev->dev, "probing interface %d\n", intf_numbers[midi][i]);
		if (intf_numbers[midi][i] > 0)
		{

			ua->intf[i] = usb_ifnum_to_if(ua->dev,
						      intf_numbers[midi][i]);
			if (!ua->intf[i]) {
				dev_err(&ua->dev->dev, "interface %u not found\n",
					intf_numbers[midi][i]);
				err = -ENXIO;
				goto probe_error;
			}
			err = usb_driver_claim_interface(&motu_avb_driver,
							 ua->intf[i], ua);
			if (err < 0) {
				ua->intf[i] = NULL;
				err = -EBUSY;
				goto probe_error;
			}
		}

	}
	dev_info(&ua->dev->dev, "probing interfaces sucessful\n");

	err = detect_usb_format(ua);
	if (err < 0)
		goto probe_error;

	name = "MOTU-AVB";
	strcpy(card->driver, "MOTU-AVB");
	strcpy(card->shortname, name);
	usb_make_path(ua->dev, usb_path, sizeof(usb_path));
	snprintf(ua->card->longname, sizeof(ua->card->longname),
		 "MOTU %s", ua->dev->product);

	err = alloc_stream_buffers(ua, &ua->capture);
	if (err < 0)
		goto probe_error;
	err = alloc_stream_buffers(ua, &ua->playback);
	if (err < 0)
		goto probe_error;

	err = alloc_stream_urbs(ua, &ua->capture, capture_urb_complete);
	if (err < 0)
		goto probe_error;
	err = alloc_stream_urbs(ua, &ua->playback, playback_urb_complete);
	if (err < 0)
		goto probe_error;

	err = snd_pcm_new(card, name, 0, 1, 1, &ua->pcm);
	if (err < 0)
		goto probe_error;
	ua->pcm->private_data = ua;
	strcpy(ua->pcm->name, name);
	snd_pcm_set_ops(ua->pcm, SNDRV_PCM_STREAM_PLAYBACK, &playback_pcm_ops);
	snd_pcm_set_ops(ua->pcm, SNDRV_PCM_STREAM_CAPTURE, &capture_pcm_ops);
	snd_pcm_set_managed_buffer_all(ua->pcm, SNDRV_DMA_TYPE_VMALLOC,
				       NULL, 0, 0);

	if (ua->intf[INTF_MIDI] > 0)
	{
		err = snd_usbmidi_create(card, ua->intf[INTF_MIDI],
					 &ua->midi_list, &midi_quirk);
		if (err < 0)
			goto probe_error;
	}

	err = snd_card_register(card);
	if (err < 0)
		goto probe_error;

	usb_set_intfdata(interface, ua);
	devices_used |= 1 << card_index;

	mutex_unlock(&devices_mutex);
	return 0;

probe_error:
	free_usb_related_resources(ua, interface);
	snd_card_free(card);
	mutex_unlock(&devices_mutex);
	return err;
}

static void motu_avb_disconnect(struct usb_interface *interface)
{
	struct motu_avb *ua = usb_get_intfdata(interface);
	struct list_head *midi;

	if (!ua)
		return;

	mutex_lock(&devices_mutex);

	set_bit(DISCONNECTED, &ua->states);
	wake_up(&ua->rate_feedback_wait);

	/* make sure that userspace cannot create new requests */
	snd_card_disconnect(ua->card);

	/* make sure that there are no pending USB requests */
	list_for_each(midi, &ua->midi_list)
		snd_usbmidi_disconnect(midi);
	abort_alsa_playback(ua);
	abort_alsa_capture(ua);
	mutex_lock(&ua->mutex);
	stop_usb_playback(ua);
	stop_usb_capture(ua);
	mutex_unlock(&ua->mutex);

	free_usb_related_resources(ua, interface);

	devices_used &= ~(1 << ua->card_index);

	snd_card_free_when_closed(ua->card);

	mutex_unlock(&devices_mutex);
}

static const struct usb_device_id motu_avb_ids[] = {
	{ USB_DEVICE(0x07fd, 0x0005) }, /* Motu AVB */
	{ }
};
MODULE_DEVICE_TABLE(usb, motu_avb_ids);

static struct usb_driver motu_avb_driver = {
	.name = "snd-motu-avb",
	.id_table = motu_avb_ids,
	.probe = motu_avb_probe,
	.disconnect = motu_avb_disconnect,
#if 0
	.suspend = motu_avb_suspend,
	.resume = motu_avb_resume,
#endif
};

module_usb_driver(motu_avb_driver);
