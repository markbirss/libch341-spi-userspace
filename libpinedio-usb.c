// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * Copyright (C) 2024 Marek Kraus <gamelaster@outlook.com>
 * Updated for CH347 Compatibility (2026)
 *
 * This code is heavily based on ch341a_spi.c from the flashrom project.
 * Copyright (C) 2011 asbokid <ballymunboy@gmail.com>
 * Copyright (C) 2014 Pluto Yang <yangyj.ee@gmail.com>
 * Copyright (C) 2015-2016 Stefan Tauner
 * Copyright (C) 2015 Urja Rannikko <urjamin@gmail.com>
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "libpinedio-usb.h"

#if 0
#define pinedio_mutex_lock(...) { printf("Locking %s\n", __func__); pthread_mutex_lock(__VA_ARGS__); }
#define pinedio_mutex_unlock(...) { printf("Unlocking %s\n", __func__); pthread_mutex_unlock(__VA_ARGS__); }
#else
#define pinedio_mutex_lock(...) pthread_mutex_lock(__VA_ARGS__);
#define pinedio_mutex_unlock(...) pthread_mutex_unlock(__VA_ARGS__);
#endif

#define CH347_USB_TIMEOUT 1000

/* CH347 High-Speed Interface 1 Vendor Mode Endpoints */
#define CH347_WRITE_EP    0x02
#define CH347_READ_EP     0x82

/* CH347 Protocol Frame Defs */
#define CH347_CMD_HEADER_H       0x57
#define CH347_CMD_HEADER_L       0xAB
#define CH347_CMD_SPI_STREAM     0x06
#define CH347_HEADER_LEN         3

/* CH347 Max internal block buffers */
#define CH347_MAX_DATA_LEN       508
#define CH347_PACKET_LENGTH      512

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// store mode and state of d0-d7
uint16_t pinedio_d_mode = 0;
uint16_t pinedio_d_state = 0;

enum trans_state {TRANS_ACTIVE = -2, TRANS_ERR = -1, TRANS_IDLE = 0};

static void platform_sleep(uint32_t msecs) {
#ifdef __WIN32
  Sleep(msecs);
#else
  usleep(msecs * 1000);
#endif
}

static void cb_common(const char* func, struct libusb_transfer *transfer) {
  int* transfer_cnt = (int *) transfer->user_data;

  if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
    *transfer_cnt = TRANS_IDLE;
    return;
  }

  if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
    fprintf(stderr, "%s: error: %s\n", func, libusb_error_name(transfer->status));
    *transfer_cnt = TRANS_ERR;
  } else {
    *transfer_cnt = transfer->actual_length;
  }
}

static void LIBUSB_CALL cb_out(struct libusb_transfer *transfer) {
  cb_common(__func__, transfer);
}

static void LIBUSB_CALL cb_in(struct libusb_transfer *transfer) {
  cb_common(__func__, transfer);
}

/* Updated Async Core Wrapper adapted for CH347 Framed packets */
static int32_t usb_transfer(struct pinedio_inst *inst, const char *func, unsigned int writecnt, unsigned int readcnt,
                            const uint8_t *writearr, uint8_t *readarr, bool lock)
{
  int state_out = TRANS_IDLE;
  uint8_t tx_frame_buf[CH347_PACKET_LENGTH];

  if (lock) {
    pinedio_mutex_lock(&inst->usb_access_mutex);
  }

  /* Encapsulate legacy raw payloads into structural high-speed frames */
  if (writecnt > 0) {
    unsigned int data_len = MIN(writecnt, CH347_MAX_DATA_LEN);
    tx_frame_buf[0] = CH347_CMD_HEADER_H;
    tx_frame_buf[1] = CH347_CMD_HEADER_L;
    tx_frame_buf[2] = CH347_CMD_SPI_STREAM;
    tx_frame_buf[3] = (uint8_t)(data_len & 0xFF);
    tx_frame_buf[4] = (uint8_t)((data_len >> 8) & 0xFF);
    memcpy(&tx_frame_buf[5], writearr, data_len);

    inst->transfer_out->endpoint = CH347_WRITE_EP;
    inst->transfer_out->buffer = tx_frame_buf;
    inst->transfer_out->length = data_len + 5; // Header overhead bytes Included
    inst->transfer_out->user_data = &state_out;

    state_out = TRANS_ACTIVE;
    int ret = libusb_submit_transfer(inst->transfer_out);
    if (ret) {
      fprintf(stderr, "%s: failed to submit OUT transfer: %s\n", func, libusb_error_name(ret));
      state_out = TRANS_ERR;
      goto err;
    }
  }

  unsigned int free_idx = 0; 
  unsigned int in_idx = 0; 
  unsigned int in_done = 0;
  unsigned int in_active = 0;
  unsigned int out_done = 0;
  
  /* Allocate buffer area for incoming structured blocks stripping framing headers */
  uint8_t raw_rx_buf[USB_IN_TRANSFERS][CH347_PACKET_LENGTH];
  int state_in[USB_IN_TRANSFERS] = {0};

  do {
    /* CH347 replies arrive framing structure matching output requests */
    while ((in_done + in_active) < readcnt && state_in[free_idx] == TRANS_IDLE) {
      unsigned int cur_todo = MIN(CH347_MAX_DATA_LEN, readcnt - in_done - in_active);
      
      inst->transfer_ins[free_idx]->endpoint = CH347_READ_EP;
      inst->transfer_ins[free_idx]->length = cur_todo + 3; // +3 for CH347 Response Header length status bytes
      inst->transfer_ins[free_idx]->buffer = raw_rx_buf[free_idx];
      inst->transfer_ins[free_idx]->user_data = &state_in[free_idx];
      
      int ret = libusb_submit_transfer(inst->transfer_ins[free_idx]);
      if (ret) {
        state_in[free_idx] = TRANS_ERR;
        fprintf(stderr, "%s: failed to submit IN transfer: %s\n", func, libusb_error_name(ret));
        goto err;
      }
      in_active += cur_todo;
      state_in[free_idx] = TRANS_ACTIVE;
      free_idx = (free_idx + 1) % USB_IN_TRANSFERS;
    }

    libusb_handle_events_timeout(NULL, &(struct timeval){1, 0});

    if (out_done < writecnt) {
      if (state_out == TRANS_ERR) {
        goto err;
      } else if (state_out > 0) {
        // Handle payload offset logic normalization
        out_done += (state_out - 5); 
        state_out = TRANS_IDLE;
      }
    }

    while (state_in[in_idx] != TRANS_IDLE && state_in[in_idx] != TRANS_ACTIVE) {
      if (state_in[in_idx] == TRANS_ERR) {
        goto err;
      }
      
      int actual_read = state_in[in_idx];
      if (actual_read > 3) {
        int payload_len = actual_read - 3;
        // Copy payload out while stepping past CH347 response status frames
        memcpy(readarr + in_done, &raw_rx_buf[in_idx][3], payload_len);
        in_done += payload_len;
        in_active -= payload_len;
      }
      
      state_in[in_idx] = TRANS_IDLE;
      in_idx = (in_idx + 1) % USB_IN_TRANSFERS; 
    }
  } while ((writecnt > 0 && out_done < writecnt) || (readcnt > 0 && in_done < readcnt));

  if (lock) {
    pinedio_mutex_unlock(&inst->usb_access_mutex);
  }
  return 0;

err:
  if ((writecnt > 0) && (state_out == TRANS_ACTIVE)) {
    libusb_cancel_transfer(inst->transfer_out);
  }
  if (readcnt > 0) {
    for (unsigned int i = 0; i < USB_IN_TRANSFERS; i++) {
      if (state_in[i] == TRANS_ACTIVE)
        libusb_cancel_transfer(inst->transfer_ins[i]);
    }
  }

  while (1) {
    bool finished = true;
    if ((writecnt > 0) && (state_out == TRANS_ACTIVE)) finished = false;
    for (unsigned int i = 0; i < USB_IN_TRANSFERS; i++) {
      if (state_in[i] == TRANS_ACTIVE) finished = false;
    }
    if (finished) break;
    libusb_handle_events_timeout(NULL, &(struct timeval){1, 0});
  }

  if (lock) {
    pinedio_mutex_unlock(&inst->usb_access_mutex);
  }
  return -1;
}

static uint8_t reverse_byte(uint8_t x) {
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
  return x;
}

int32_t pinedio_init(struct pinedio_inst *inst, void *driver) {
  int32_t ret;
  inst->int_running_cnt = 0;
  inst->pin_poll_thread_exit = false;
  for (int i = 0; i < PINEDIO_INT_PIN_MAX; i++) {
    inst->interrupts[i].callback = NULL;
  }

  inst->options[PINEDIO_OPTION_AUTO_CS] = 1;

  ret = pthread_mutex_init(&inst->usb_access_mutex, NULL);
  if (ret != 0) {
    fprintf(stderr, "Failed to initialize mutex, res: %d.\n", ret);
    return -1;
  }

  ret = libusb_init(NULL);
  if (ret < 0) {
    fprintf(stderr, "Couldn't initialize libusb!\n");
    return -1;
  }

  libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
  
  /* Enforce CH347 Vendor Mode Descriptors */
  if (inst->options[PINEDIO_OPTION_VID] == 0) {
    inst->options[PINEDIO_OPTION_VID] = 0x1A86;
  }
  if (inst->options[PINEDIO_OPTION_PID] == 0) {
    inst->options[PINEDIO_OPTION_PID] = 0x55DA; // Mode 1 PID Default
  }

  libusb_device **list;
  libusb_device *found = NULL;
  ssize_t cnt = libusb_get_device_list(NULL, &list);
  ssize_t i = 0;

  for (i = 0; i < cnt; i++) {
    libusb_device *device = list[i];
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(device, &desc);

    if (desc.idVendor == inst->options[PINEDIO_OPTION_VID] && desc.idProduct == inst->options[PINEDIO_OPTION_PID]) {
      found = device;
      ret = libusb_open(found, &inst->handle);
      if (ret == 0) {
        // Successfully opened device. Ensure to claim Interface 1 for SPI Vendor Mode.
        libusb_claim_interface(inst->handle, 1);
        break;
      }
    }
  }
  
  libusb_free_device_list(list, 1);
  return (inst->handle) ? 0 : -1;
}
