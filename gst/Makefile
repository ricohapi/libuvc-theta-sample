# 
#   Copyright 2020 K. Takeo. All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1. Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above
#   copyright notice, this list of conditions and the following
#   disclaimer in the documentation and/or other materials provided
#   with the distribution.
#   3. Neither the name of the author nor other contributors may be
#   used to endorse or promote products derived from this software
#   without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 


PKG_CONFIG_LIB = gstreamer-app-1.0 libuvc

CFLAGS += $(shell pkg-config --cflags $(PKG_CONFIG_LIB))
LDFLAGS += $(shell pkg-config --libs $(PKG_CONFIG_LIB))

SRCS = gst_viewer.c thetauvc.c
OBJS := $(SRCS:.c=.o)
EXES = gst_viewer gst_loopback

all: $(EXES)

gst_viewer: $(OBJS)
	$(CC) $(OBJS) -o $@  $(LDFLAGS) -pthread

gst_loopback: gst_viewer
	ln -sf $< $@

clean:
	@$(RM) $(OBJS) $(EXES)
