LOCAL_PATH := $(call my-dir)

###################
###################
include $(CLEAR_VARS)

LOCAL_MODULE := libopenal_common

LOCAL_CFLAGS := -DAL_ALEXT_PROTOTYPES
LOCAL_CPPFLAGS := -DAL_ALEXT_PROTOTYPES

LOCAL_C_INCLUDES := \
$(LOCAL_PATH) \
$(LOCAL_PATH)/common \
$(LOCAL_PATH)/include \
$(LOCAL_PATH)/OpenAL32/Include \
$(LOCAL_PATH)/Alc

LOCAL_SRC_FILES := \
$(call include-path-for, audio-utils) \
common/almalloc.c \
common/atomic.c \
common/rwlock.c \
common/threads.c \
common/uintmap.c

include $(BUILD_STATIC_LIBRARY)
###################
###################

########################################################

###################
###################
include $(CLEAR_VARS)
LOCAL_MODULE := libopenal

LOCAL_CFLAGS := -DAL_ALEXT_PROTOTYPES
LOCAL_CPPFLAGS := -DAL_ALEXT_PROTOTYPES

LOCAL_C_INCLUDES := \
$(call include-path-for, audio-utils) \
$(LOCAL_PATH) \
$(LOCAL_PATH)/common \
$(LOCAL_PATH)/include \
$(LOCAL_PATH)/OpenAL32/Include \
$(LOCAL_PATH)/Alc

LOCAL_SRC_FILES := \
Alc/ALc.c \
Alc/ALu.c \
Alc/alcConfig.c \
Alc/alcRing.c \
Alc/ambdec.c \
Alc/bformatdec.c \
Alc/bs2b.c \
Alc/bsinc.c \
Alc/helpers.c \
Alc/hrtf.c \
Alc/mixer.c \
Alc/mixer_c.c \
Alc/panning.c \
Alc/uhjfilter.c \
Alc/nfcfilter.c \
Alc/mixer_neon.c \
Alc/alstring.c \
Alc/converter.c \
Alc/backends/alsa.c \
Alc/backends/base.c \
Alc/backends/loopback.c \
Alc/backends/null.c \
Alc/backends/wave.c \
Alc/effects/chorus.c \
Alc/effects/compressor.c \
Alc/effects/dedicated.c \
Alc/effects/distortion.c \
Alc/effects/echo.c \
Alc/effects/equalizer.c \
Alc/effects/flanger.c \
Alc/effects/modulator.c \
Alc/effects/null.c \
Alc/effects/reverb.c \
OpenAL32/alAuxEffectSlot.c \
OpenAL32/alBuffer.c \
OpenAL32/alEffect.c \
OpenAL32/alError.c \
OpenAL32/alExtension.c \
OpenAL32/alFilter.c \
OpenAL32/alListener.c \
OpenAL32/alSource.c \
OpenAL32/alState.c \
OpenAL32/alThunk.c \
OpenAL32/sample_cvt.c \
OpenAL32/alMain.c

LOCAL_STATIC_LIBRARIES   := libopenal_common
LOCAL_SHARED_LIBRARIES := liblog

include $(BUILD_SHARED_LIBRARY)
###################
###################
