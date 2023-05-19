/*
  Copyright 2020 K. Takeo. All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  3. Neither the name of the author nor other contributors may be
  used to endorse or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

 */

#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "libuvc/libuvc.h"
#include "thetauvc.h"

#define MAX_PIPELINE_LEN 1024
#define MAX_OUTPUT_PIPELINE_LEN 512 // It must be less than previous
#define DEFAULT_CAM_MODE "UHD"
#define DEFAULT_BITRATE 5000000
#define DEFAULT_KFI 300
// #define DEFAULT_RTP_ADDRESS "127.0.0.1"
#define DEFAULT_RTP_ADDRESS "192.168.1.150"
#define DEFAULT_RTP_PORT 6666
#define DEFAULT_RTP_MAX_DELAY 150000

struct gst_src {
  GstElement *pipeline;
  GstElement *appsrc;

  GMainLoop *loop;
  GTimer *timer;
  guint framecount;
  guint id;
  guint bus_watch_id;
  uint32_t dwFrameInterval;
  uint32_t dwClockFrequency;
};

struct gst_src src;

struct stream_parameters {
  enum thetauvc_mode_code mode;
  uint32_t bitrate;     // bps
  uint32_t max_bitrate; // bps
  uint32_t kfi;         // Key Frame Interval (# frames)
  char *address;
  uint16_t port;
  uint32_t max_delay;   // us
  char *filename;
};

void print_help()
{
  printf("Options:\n");
  printf("\t-m: Resolution Mode. Accepted values: UHD, FHD. Default: %s\n", DEFAULT_CAM_MODE);
  printf("\t-b: bitrate (bps). Default: %i. If set to 0, raw input from camera\n", DEFAULT_BITRATE);
  printf("\t-k: Keyframe Interval (frames). Default: %i\n", DEFAULT_KFI);
  printf("\t-a: RTP address. Default: %s\n", DEFAULT_RTP_ADDRESS);
  printf("\t-p: RTP port. Default: %i\n", DEFAULT_RTP_PORT);
  printf("\t-d: RTP max delay (us). Default: %i\n", DEFAULT_RTP_MAX_DELAY);
  printf("\t-f: Filename. If set, only recording to disk (TS stream). No RTP will be generated\n");
  exit(0);
}

static gboolean
gst_bus_cb(GstBus *bus, GstMessage *message, gpointer data)
{
  GError *err;
  gchar *dbg;

  switch (GST_MESSAGE_TYPE(message)) {
  case GST_MESSAGE_ERROR:
    gst_message_parse_error(message, &err, &dbg);
    g_print("Error: %s\n", err->message);
    g_error_free(err);
    g_free(dbg);
    g_main_loop_quit(src.loop);
    break;

  default:
    break;
  }

  return TRUE;
}


int
gst_src_init(int *argc, char ***argv, char *pipeline)
{
  GstCaps *caps;
  GstBus *bus;
  char pipeline_str[MAX_PIPELINE_LEN+50];

  snprintf(pipeline_str, MAX_PIPELINE_LEN+50,
      "appsrc name=ap ! queue ! h264parse config-interval=1 ! queue ! %s ", pipeline);

  fprintf(stderr, "%s\n", pipeline_str);

  gst_init(argc, argv);
  src.timer = g_timer_new();
  src.loop = g_main_loop_new(NULL, TRUE);
  src.pipeline = gst_parse_launch(pipeline_str, NULL);

  g_assert(src.pipeline);
  if (src.pipeline == NULL)
    return FALSE;
  gst_pipeline_set_clock(GST_PIPELINE(src.pipeline), gst_system_clock_obtain());

  src.appsrc = gst_bin_get_by_name(GST_BIN(src.pipeline), "ap");

  caps = gst_caps_new_simple("video/x-h264",
    "framerate", GST_TYPE_FRACTION, 30000, 1001,
    "stream-format", G_TYPE_STRING, "byte-stream",
    "profile", G_TYPE_STRING, "constrained-baseline", NULL);
  gst_app_src_set_caps(GST_APP_SRC(src.appsrc), caps);

  bus = gst_pipeline_get_bus(GST_PIPELINE(src.pipeline));
  src.bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, NULL);
  gst_object_unref(bus);
  return TRUE;
}

void *
keywait(void *arg)
{
  struct gst_src *s;
  char keyin[4];

  read(1, keyin, 1);

  s = (struct gst_src *)arg;
  g_main_loop_quit(s->loop);

}

void
cb(uvc_frame_t *frame, void *ptr)
{
  struct gst_src *s;
  GstBuffer *buffer;
  GstFlowReturn ret;
  GstMapInfo map;
  gdouble ms;
  uint32_t pts;

  s = (struct gst_src *)ptr;
  ms = g_timer_elapsed(s->timer, NULL);

  buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);;
  GST_BUFFER_PTS(buffer) = frame->sequence * s->dwFrameInterval*100;
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(buffer) = s->dwFrameInterval*100;
  GST_BUFFER_OFFSET(buffer) = frame->sequence;
  s->framecount++;

  gst_buffer_map(buffer, &map, GST_MAP_WRITE);
  memcpy(map.data, frame->data, frame->data_bytes);
  gst_buffer_unmap(buffer, &map);

  g_signal_emit_by_name(s->appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);

  if (ret != GST_FLOW_OK)
    fprintf(stderr, "pushbuffer errorn");
  return;
}

int
main(int argc, char **argv)
{
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_t **devlist;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  pthread_t thr;
  pthread_attr_t attr;

  struct stream_parameters cam_params;

  struct gst_src *s;
  int idx;
  char pipe_proc[MAX_PIPELINE_LEN];
  char *cmd_name;
  int opt;

  cmd_name = rindex(argv[0], '/');
  if (cmd_name == NULL)
    cmd_name = argv[0];
  else
    cmd_name++;

  cam_params.bitrate = DEFAULT_BITRATE;
  cam_params.max_bitrate = cam_params.bitrate * 1.5;
  if (strcmp(DEFAULT_CAM_MODE, "UHD") == 0)
    cam_params.mode = THETAUVC_MODE_UHD_30;
  else if (strcmp(DEFAULT_CAM_MODE, "FHD") == 0)
    cam_params.mode = THETAUVC_MODE_FHD_30;
  else
    cam_params.mode = THETAUVC_MODE_UHD_2997;
  cam_params.kfi = DEFAULT_KFI;
  cam_params.address = DEFAULT_RTP_ADDRESS;
  cam_params.port = DEFAULT_RTP_PORT;
  cam_params.max_delay = DEFAULT_RTP_MAX_DELAY;
  cam_params.filename = "\0";
  while ((opt = getopt(argc, argv, "b:d:m:k:a:p:f:h")) != -1) {
    switch (opt) {
      case 'b': cam_params.bitrate = (uint32_t) atoi(optarg); break;
      case 'd': cam_params.max_delay = atoi(optarg); break;
      case 'm':
      {
        if (strcmp(optarg, "UHD") == 0)
          cam_params.mode = THETAUVC_MODE_UHD_30;
        else if (strcmp(optarg, "FHD") == 0)
          cam_params.mode = THETAUVC_MODE_FHD_30;
        else
          cam_params.mode = THETAUVC_MODE_UHD_2997;
        break;
      }
      case 'k': cam_params.kfi = atoi(optarg); break;
      case 'a': cam_params.address = optarg; break;
      case 'p': cam_params.port = atoi(optarg); break;
      case 'f': cam_params.filename = optarg; break;
      case 'h': print_help(); break;
      default:
      {
          fprintf(stderr, "Usage: %s [-bdmkaph] (-h for options meaning)\n", argv[0]);
          exit(EXIT_FAILURE);
      }
    }
  }
  if (strcmp(cmd_name, "gst_loopback") == 0)
    snprintf(pipe_proc, MAX_PIPELINE_LEN,
      "decodebin ! autovideoconvert ! "
      "video/x-raw,format=I420 ! identity drop-allocation=true !"
      "v4l2sink device=/dev/video60 sync=false"
      );
  else {
    char output_str[MAX_OUTPUT_PIPELINE_LEN];
    if (strlen(cam_params.filename) == 0) {
      sprintf(output_str,
        "rtph264pay pt=96 config-interval=1 ! "
        "rtprateshape max-delay-us=%i max-bitrate=%i ! "
        "udpsink host=%s port=%i",
        cam_params.max_delay,
        cam_params.max_bitrate,
        cam_params.address,
        cam_params.port
        );
    } else {
      sprintf(output_str, "mpegtsmux ! filesink location=%s -e", cam_params.filename);
    }

    if (cam_params.bitrate == 0) {
      // TODO There is some CAPS issue that generate wrong PTS
      snprintf(pipe_proc, MAX_PIPELINE_LEN, "video/x-h264,framerate=30000/1001 ! %s", output_str);
    } else {
      snprintf(pipe_proc, MAX_PIPELINE_LEN,
        "omxh264dec ! "
        "omxh264enc insert-sps-pps=true profile=main control-rate=constant-skip-frames "
        "preset-level=FastPreset bitrate=%i iframeinterval=%i ! "
        "h264parse config-interval=1 ! %s",
        cam_params.bitrate,
        cam_params.kfi,
        output_str
        );
    }
  }

  if (!gst_src_init(&argc, &argv, pipe_proc))
    return -1;

  res = uvc_init(&ctx, NULL);
  if (res != UVC_SUCCESS) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  if (argc > 1 && strcmp("-l", argv[1]) == 0) {
    res = thetauvc_print_devices(ctx, stdout);
    if (res != UVC_SUCCESS) {
      uvc_perror(res,"Error printing devices: ");
      uvc_exit(ctx);
      return res;
    }
    uvc_exit(ctx);
    exit(0);
  }

  src.framecount = 0;
  res = thetauvc_find_device(ctx, &dev, 0);
  if (res != UVC_SUCCESS) {
    uvc_perror(res,"THETA not found");
    goto exit;
  }

  res = uvc_open(dev, &devh);
  if (res != UVC_SUCCESS) {
    uvc_perror(res,"Can't open THETA");
    goto exit;
  }

  uvc_print_diag(devh, stdout);

  gst_element_set_state(src.pipeline, GST_STATE_PLAYING);
  pthread_create(&thr, NULL, keywait, &src);

  res = thetauvc_get_stream_ctrl_format_size(devh,
      cam_params.mode, &ctrl);
  src.dwFrameInterval = ctrl.dwFrameInterval;
  src.dwClockFrequency = ctrl.dwClockFrequency;

  if (res != UVC_SUCCESS) {
    uvc_perror(res,"Can't set stream");
  }
  uvc_perror(res,"go on");

  res = uvc_start_streaming(devh, &ctrl, cb, &src, 0);
  if (res == UVC_SUCCESS) {
    g_main_loop_run(src.loop);

    fprintf(stderr, "stop streaming\n");
    uvc_stop_streaming(devh);

    gst_element_set_state(src.pipeline, GST_STATE_NULL);
    g_source_remove(src.bus_watch_id);
    g_main_loop_unref(src.loop);

    pthread_cancel(thr);
    pthread_join(thr, NULL);
  }

  uvc_close(devh);
  return UVC_ERROR_OTHER;

exit:
  uvc_exit(ctx);
  return res;
}
