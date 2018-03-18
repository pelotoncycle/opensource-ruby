/**
 * OpenAL cross platform audio library
 * Copyright (C) 1999-2007 by authors.
 * This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the
 *  Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * Or go to http://www.gnu.org/copyleft/lgpl.html
 */

#include "config.h"

#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include "alMain.h"
#include "alu.h"
#include "threads.h"
#include "compat.h"
#include "backends/base.h"

struct pcm_config
{
    unsigned int rate;
    unsigned int period_size;
    unsigned int period_count;
};

static	struct pcm_config	s_alsa_config;
static const ALCchar 		alsaDevice[] = "ALSA Default";

#ifdef HAVE_DYNLOAD
#define ALSA_FUNCS(MAGIC)                                                     \
    MAGIC(snd_strerror);                                                      \
    MAGIC(snd_pcm_open);                                                      \
    MAGIC(snd_pcm_close);                                                     \
    MAGIC(snd_pcm_nonblock);                                                  \
    MAGIC(snd_pcm_frames_to_bytes);                                           \
    MAGIC(snd_pcm_bytes_to_frames);                                           \
    MAGIC(snd_pcm_hw_params_malloc);                                          \
    MAGIC(snd_pcm_hw_params_free);                                            \
    MAGIC(snd_pcm_hw_params_any);                                             \
    MAGIC(snd_pcm_hw_params_current);                                         \
    MAGIC(snd_pcm_hw_params_set_access);                                      \
    MAGIC(snd_pcm_hw_params_set_format);                                      \
    MAGIC(snd_pcm_hw_params_set_channels);                                    \
    MAGIC(snd_pcm_hw_params_set_periods_near);                                \
    MAGIC(snd_pcm_hw_params_set_rate_near);                                   \
    MAGIC(snd_pcm_hw_params_set_rate);                                        \
    MAGIC(snd_pcm_hw_params_set_rate_resample);                               \
    MAGIC(snd_pcm_hw_params_set_buffer_time_near);                            \
    MAGIC(snd_pcm_hw_params_set_period_time_near);                            \
    MAGIC(snd_pcm_hw_params_set_buffer_size_near);                            \
    MAGIC(snd_pcm_hw_params_set_period_size_near);                            \
    MAGIC(snd_pcm_hw_params_set_buffer_size_min);                             \
    MAGIC(snd_pcm_hw_params_get_buffer_time_min);                             \
    MAGIC(snd_pcm_hw_params_get_buffer_time_max);                             \
    MAGIC(snd_pcm_hw_params_get_period_time_min);                             \
    MAGIC(snd_pcm_hw_params_get_period_time_max);                             \
    MAGIC(snd_pcm_hw_params_get_buffer_size);                                 \
    MAGIC(snd_pcm_hw_params_get_period_size);                                 \
    MAGIC(snd_pcm_hw_params_get_access);                                      \
    MAGIC(snd_pcm_hw_params_get_periods);                                     \
    MAGIC(snd_pcm_hw_params_test_format);                                     \
    MAGIC(snd_pcm_hw_params_test_channels);                                   \
    MAGIC(snd_pcm_hw_params);                                                 \
    MAGIC(snd_pcm_sw_params_malloc);                                          \
    MAGIC(snd_pcm_sw_params_current);                                         \
    MAGIC(snd_pcm_sw_params_set_avail_min);                                   \
    MAGIC(snd_pcm_sw_params_set_stop_threshold);                              \
    MAGIC(snd_pcm_sw_params);                                                 \
    MAGIC(snd_pcm_sw_params_free);                                            \
    MAGIC(snd_pcm_prepare);                                                   \
    MAGIC(snd_pcm_start);                                                     \
    MAGIC(snd_pcm_resume);                                                    \
    MAGIC(snd_pcm_reset);                                                     \
    MAGIC(snd_pcm_wait);                                                      \
    MAGIC(snd_pcm_delay);                                                     \
    MAGIC(snd_pcm_state);                                                     \
    MAGIC(snd_pcm_avail_update);                                              \
    MAGIC(snd_pcm_areas_silence);                                             \
    MAGIC(snd_pcm_mmap_begin);                                                \
    MAGIC(snd_pcm_mmap_commit);                                               \
    MAGIC(snd_pcm_readi);                                                     \
    MAGIC(snd_pcm_writei);                                                    \
    MAGIC(snd_pcm_drain);                                                     \
    MAGIC(snd_pcm_drop);                                                      \
    MAGIC(snd_pcm_recover);                                                   \
    MAGIC(snd_pcm_info_malloc);                                               \
    MAGIC(snd_pcm_info_free);                                                 \
    MAGIC(snd_pcm_info_set_device);                                           \
    MAGIC(snd_pcm_info_set_subdevice);                                        \
    MAGIC(snd_pcm_info_set_stream);                                           \
    MAGIC(snd_pcm_info_get_name);                                             \
    MAGIC(snd_ctl_pcm_next_device);                                           \
    MAGIC(snd_ctl_pcm_info);                                                  \
    MAGIC(snd_ctl_open);                                                      \
    MAGIC(snd_ctl_close);                                                     \
    MAGIC(snd_ctl_card_info_malloc);                                          \
    MAGIC(snd_ctl_card_info_free);                                            \
    MAGIC(snd_ctl_card_info);                                                 \
    MAGIC(snd_ctl_card_info_get_name);                                        \
    MAGIC(snd_ctl_card_info_get_id);                                          \
    MAGIC(snd_card_next);                                                     \
    MAGIC(snd_config_update_free_global)

static void *alsa_handle;
#define MAKE_FUNC(f) static __typeof(f) * p##f
ALSA_FUNCS(MAKE_FUNC);
#undef MAKE_FUNC

#define snd_strerror psnd_strerror
#define snd_pcm_open psnd_pcm_open
#define snd_pcm_close psnd_pcm_close
#define snd_pcm_nonblock psnd_pcm_nonblock
#define snd_pcm_frames_to_bytes psnd_pcm_frames_to_bytes
#define snd_pcm_bytes_to_frames psnd_pcm_bytes_to_frames
#define snd_pcm_hw_params_malloc psnd_pcm_hw_params_malloc
#define snd_pcm_hw_params_free psnd_pcm_hw_params_free
#define snd_pcm_hw_params_any psnd_pcm_hw_params_any
#define snd_pcm_hw_params_current psnd_pcm_hw_params_current
#define snd_pcm_hw_params_set_access psnd_pcm_hw_params_set_access
#define snd_pcm_hw_params_set_format psnd_pcm_hw_params_set_format
#define snd_pcm_hw_params_set_channels psnd_pcm_hw_params_set_channels
#define snd_pcm_hw_params_set_periods_near psnd_pcm_hw_params_set_periods_near
#define snd_pcm_hw_params_set_rate_near psnd_pcm_hw_params_set_rate_near
#define snd_pcm_hw_params_set_rate psnd_pcm_hw_params_set_rate
#define snd_pcm_hw_params_set_rate_resample psnd_pcm_hw_params_set_rate_resample
#define snd_pcm_hw_params_set_buffer_time_near psnd_pcm_hw_params_set_buffer_time_near
#define snd_pcm_hw_params_set_period_time_near psnd_pcm_hw_params_set_period_time_near
#define snd_pcm_hw_params_set_buffer_size_near psnd_pcm_hw_params_set_buffer_size_near
#define snd_pcm_hw_params_set_period_size_near psnd_pcm_hw_params_set_period_size_near
#define snd_pcm_hw_params_set_buffer_size_min psnd_pcm_hw_params_set_buffer_size_min
#define snd_pcm_hw_params_get_buffer_time_min psnd_pcm_hw_params_get_buffer_time_min
#define snd_pcm_hw_params_get_buffer_time_max psnd_pcm_hw_params_get_buffer_time_max
#define snd_pcm_hw_params_get_period_time_min psnd_pcm_hw_params_get_period_time_min
#define snd_pcm_hw_params_get_period_time_max psnd_pcm_hw_params_get_period_time_max
#define snd_pcm_hw_params_get_buffer_size psnd_pcm_hw_params_get_buffer_size
#define snd_pcm_hw_params_get_period_size psnd_pcm_hw_params_get_period_size
#define snd_pcm_hw_params_get_access psnd_pcm_hw_params_get_access
#define snd_pcm_hw_params_get_periods psnd_pcm_hw_params_get_periods
#define snd_pcm_hw_params_test_format psnd_pcm_hw_params_test_format
#define snd_pcm_hw_params_test_channels psnd_pcm_hw_params_test_channels
#define snd_pcm_hw_params psnd_pcm_hw_params
#define snd_pcm_sw_params_malloc psnd_pcm_sw_params_malloc
#define snd_pcm_sw_params_current psnd_pcm_sw_params_current
#define snd_pcm_sw_params_set_avail_min psnd_pcm_sw_params_set_avail_min
#define snd_pcm_sw_params_set_stop_threshold psnd_pcm_sw_params_set_stop_threshold
#define snd_pcm_sw_params psnd_pcm_sw_params
#define snd_pcm_sw_params_free psnd_pcm_sw_params_free
#define snd_pcm_prepare psnd_pcm_prepare
#define snd_pcm_start psnd_pcm_start
#define snd_pcm_resume psnd_pcm_resume
#define snd_pcm_reset psnd_pcm_reset
#define snd_pcm_wait psnd_pcm_wait
#define snd_pcm_delay psnd_pcm_delay
#define snd_pcm_state psnd_pcm_state
#define snd_pcm_avail_update psnd_pcm_avail_update
#define snd_pcm_areas_silence psnd_pcm_areas_silence
#define snd_pcm_mmap_begin psnd_pcm_mmap_begin
#define snd_pcm_mmap_commit psnd_pcm_mmap_commit
#define snd_pcm_readi psnd_pcm_readi
#define snd_pcm_writei psnd_pcm_writei
#define snd_pcm_drain psnd_pcm_drain
#define snd_pcm_drop psnd_pcm_drop
#define snd_pcm_recover psnd_pcm_recover
#define snd_pcm_info_malloc psnd_pcm_info_malloc
#define snd_pcm_info_free psnd_pcm_info_free
#define snd_pcm_info_set_device psnd_pcm_info_set_device
#define snd_pcm_info_set_subdevice psnd_pcm_info_set_subdevice
#define snd_pcm_info_set_stream psnd_pcm_info_set_stream
#define snd_pcm_info_get_name psnd_pcm_info_get_name
#define snd_ctl_pcm_next_device psnd_ctl_pcm_next_device
#define snd_ctl_pcm_info psnd_ctl_pcm_info
#define snd_ctl_open psnd_ctl_open
#define snd_ctl_close psnd_ctl_close
#define snd_ctl_card_info_malloc psnd_ctl_card_info_malloc
#define snd_ctl_card_info_free psnd_ctl_card_info_free
#define snd_ctl_card_info psnd_ctl_card_info
#define snd_ctl_card_info_get_name psnd_ctl_card_info_get_name
#define snd_ctl_card_info_get_id psnd_ctl_card_info_get_id
#define snd_card_next psnd_card_next
#define snd_config_update_free_global psnd_config_update_free_global
#endif

void SetPcmConfig(struct pcm_config config)
{
	s_alsa_config.rate 					= config.rate;
	s_alsa_config.period_size 		= config.period_size;
	s_alsa_config.period_count 	= config.period_count;
}

static ALCboolean alsa_load(void)
{
    ALCboolean error = ALC_FALSE;

	return !error;
}


typedef struct {
    al_string name;
    al_string device_name;
} DevMap;
TYPEDEF_VECTOR(DevMap, vector_DevMap)

/*static vector_DevMap PlaybackDevices;
static vector_DevMap CaptureDevices;*/

static void clear_devlist(vector_DevMap *devlist)
{
#define FREE_DEV(i) do {                                                      \
    AL_STRING_DEINIT((i)->name);                                              \
    AL_STRING_DEINIT((i)->device_name);                                       \
} while(0)
    VECTOR_FOR_EACH(DevMap, *devlist, FREE_DEV);
    VECTOR_RESIZE(*devlist, 0, 0);
#undef FREE_DEV
}


/*static const char *prefix_name(snd_pcm_stream_t stream)
{
    assert(stream == SND_PCM_STREAM_PLAYBACK || stream == SND_PCM_STREAM_CAPTURE);
    return (stream==SND_PCM_STREAM_PLAYBACK) ? "device-prefix" : "capture-prefix";
}

static void probe_devices(snd_pcm_stream_t stream, vector_DevMap *DeviceList)
{
    const char *main_prefix = "plughw:";
    snd_ctl_t *handle;
    snd_ctl_card_info_t *info;
    snd_pcm_info_t *pcminfo;
    int card, err, dev;
    DevMap entry;

    clear_devlist(DeviceList);

    snd_ctl_card_info_malloc(&info);
    snd_pcm_info_malloc(&pcminfo);

    AL_STRING_INIT(entry.name);
    AL_STRING_INIT(entry.device_name);
    alstr_copy_cstr(&entry.name, alsaDevice);
    alstr_copy_cstr(&entry.device_name, GetConfigValue(NULL, "alsa", (stream==SND_PCM_STREAM_PLAYBACK) ?
                                                           "device" : "capture", "default"));
    VECTOR_PUSH_BACK(*DeviceList, entry);

    card = -1;
    if((err=snd_card_next(&card)) < 0)
        ERR("Failed to find a card: %s\n", snd_strerror(err));
    ConfigValueStr(NULL, "alsa", prefix_name(stream), &main_prefix);
    while(card >= 0)
    {
        const char *card_prefix = main_prefix;
        const char *cardname, *cardid;
        char name[256];

        snprintf(name, sizeof(name), "hw:%d", card);
        if((err = snd_ctl_open(&handle, name, 0)) < 0)
        {
            ERR("control open (hw:%d): %s\n", card, snd_strerror(err));
            goto next_card;
        }
        if((err = snd_ctl_card_info(handle, info)) < 0)
        {
            ERR("control hardware info (hw:%d): %s\n", card, snd_strerror(err));
            snd_ctl_close(handle);
            goto next_card;
        }

        cardname = snd_ctl_card_info_get_name(info);
        cardid = snd_ctl_card_info_get_id(info);

        snprintf(name, sizeof(name), "%s-%s", prefix_name(stream), cardid);
        ConfigValueStr(NULL, "alsa", name, &card_prefix);

        dev = -1;
        while(1)
        {
            const char *device_prefix = card_prefix;
            const char *devname;
            char device[128];

            if(snd_ctl_pcm_next_device(handle, &dev) < 0)
                ERR("snd_ctl_pcm_next_device failed\n");
            if(dev < 0)
                break;

            snd_pcm_info_set_device(pcminfo, dev);
            snd_pcm_info_set_subdevice(pcminfo, 0);
            snd_pcm_info_set_stream(pcminfo, stream);
            if((err = snd_ctl_pcm_info(handle, pcminfo)) < 0) {
                if(err != -ENOENT)
                    ERR("control digital audio info (hw:%d): %s\n", card, snd_strerror(err));
                continue;
            }

            devname = snd_pcm_info_get_name(pcminfo);

            snprintf(name, sizeof(name), "%s-%s-%d", prefix_name(stream), cardid, dev);
            ConfigValueStr(NULL, "alsa", name, &device_prefix);

            snprintf(name, sizeof(name), "%s, %s (CARD=%s,DEV=%d)",
                        cardname, devname, cardid, dev);
            snprintf(device, sizeof(device), "%sCARD=%s,DEV=%d",
                        device_prefix, cardid, dev);

            TRACE("Got device \"%s\", \"%s\"\n", name, device);
            AL_STRING_INIT(entry.name);
            AL_STRING_INIT(entry.device_name);
            alstr_copy_cstr(&entry.name, name);
            alstr_copy_cstr(&entry.device_name, device);
            VECTOR_PUSH_BACK(*DeviceList, entry);
        }
        snd_ctl_close(handle);
    next_card:
        if(snd_card_next(&card) < 0) {
            ERR("snd_card_next failed\n");
            break;
        }
    }

    snd_pcm_info_free(pcminfo);
    snd_ctl_card_info_free(info);
}*/


static int verify_state(void *handle)
{
    return 0;
}

typedef struct ALCplaybackAlsa {
    DERIVE_FROM_TYPE(ALCbackend);

    //snd_pcm_t *pcmHandle;
    void *pcmHandle;

    ALvoid *buffer;
    ALsizei size;

    volatile int killNow;
    althrd_t thread;
} ALCplaybackAlsa;

static int ALCplaybackAlsa_mixerProc(void *ptr);
static int ALCplaybackAlsa_mixerNoMMapProc(void *ptr);

static void ALCplaybackAlsa_Construct(ALCplaybackAlsa *self, ALCdevice *device);
static DECLARE_FORWARD(ALCplaybackAlsa, ALCbackend, void, Destruct)
static ALCenum ALCplaybackAlsa_open(ALCplaybackAlsa *self, const ALCchar *name);
static void ALCplaybackAlsa_close(ALCplaybackAlsa *self);
static ALCboolean ALCplaybackAlsa_reset(ALCplaybackAlsa *self);
static ALCboolean ALCplaybackAlsa_start(ALCplaybackAlsa *self);
static void ALCplaybackAlsa_stop(ALCplaybackAlsa *self);
static DECLARE_FORWARD2(ALCplaybackAlsa, ALCbackend, ALCenum, captureSamples, void*, ALCuint)
static DECLARE_FORWARD(ALCplaybackAlsa, ALCbackend, ALCuint, availableSamples)
static ClockLatency ALCplaybackAlsa_getClockLatency(ALCplaybackAlsa *self);
static DECLARE_FORWARD(ALCplaybackAlsa, ALCbackend, void, lock)
static DECLARE_FORWARD(ALCplaybackAlsa, ALCbackend, void, unlock)
DECLARE_DEFAULT_ALLOCATORS(ALCplaybackAlsa)

DEFINE_ALCBACKEND_VTABLE(ALCplaybackAlsa);


static void ALCplaybackAlsa_Construct(ALCplaybackAlsa *self, ALCdevice *device)
{
    ALCbackend_Construct(STATIC_CAST(ALCbackend, self), device);
    SET_VTABLE2(ALCplaybackAlsa, ALCbackend, self);
}


static int ALCplaybackAlsa_mixerProc(void *ptr)
{
    return 0;
}

static int ALCplaybackAlsa_mixerNoMMapProc(void *ptr)
{
    return 0;
}


static ALCenum ALCplaybackAlsa_open(ALCplaybackAlsa *self, const ALCchar *name)
{
    ALCdevice *device = STATIC_CAST(ALCbackend, self)->mDevice;

    alstr_copy_cstr(&device->DeviceName, alsaDevice);

    return ALC_NO_ERROR;
}

static void ALCplaybackAlsa_close(ALCplaybackAlsa *self)
{
}

static ALCboolean ALCplaybackAlsa_reset(ALCplaybackAlsa *self)
{
	ALCdevice*	device = STATIC_CAST(ALCbackend, self)->mDevice;

	device->UpdateSize  		= s_alsa_config.period_size;
	device->NumUpdates	= s_alsa_config.period_count;
	device->Frequency			= s_alsa_config.rate;

	SetDefaultChannelOrder(device);

	return ALC_TRUE;
}

static ALCboolean ALCplaybackAlsa_start(ALCplaybackAlsa *self)
{
    return ALC_TRUE;
}

static void ALCplaybackAlsa_stop(ALCplaybackAlsa *self)
{
    int res;

    if(self->killNow)
        return;

    self->killNow = 1;
    althrd_join(self->thread, &res);

    al_free(self->buffer);
    self->buffer = NULL;
}

static ClockLatency ALCplaybackAlsa_getClockLatency(ALCplaybackAlsa *self)
{
    ALCdevice *device = STATIC_CAST(ALCbackend, self)->mDevice;
    //pcm_sframes_t delay = 0;
    ClockLatency ret;
    int err;

    return ret;
}


typedef struct ALCcaptureAlsa {
    DERIVE_FROM_TYPE(ALCbackend);

    //snd_pcm_t *pcmHandle;
    void *pcmHandle;

    ALvoid *buffer;
    ALsizei size;

    ALboolean doCapture;
    ll_ringbuffer_t *ring;

    //snd_pcm_sframes_t last_avail;
} ALCcaptureAlsa;

static void ALCcaptureAlsa_Construct(ALCcaptureAlsa *self, ALCdevice *device);
static DECLARE_FORWARD(ALCcaptureAlsa, ALCbackend, void, Destruct)
static ALCenum ALCcaptureAlsa_open(ALCcaptureAlsa *self, const ALCchar *name);
static void ALCcaptureAlsa_close(ALCcaptureAlsa *self);
static DECLARE_FORWARD(ALCcaptureAlsa, ALCbackend, ALCboolean, reset)
static ALCboolean ALCcaptureAlsa_start(ALCcaptureAlsa *self);
static void ALCcaptureAlsa_stop(ALCcaptureAlsa *self);
static ALCenum ALCcaptureAlsa_captureSamples(ALCcaptureAlsa *self, ALCvoid *buffer, ALCuint samples);
static ALCuint ALCcaptureAlsa_availableSamples(ALCcaptureAlsa *self);
static ClockLatency ALCcaptureAlsa_getClockLatency(ALCcaptureAlsa *self);
static DECLARE_FORWARD(ALCcaptureAlsa, ALCbackend, void, lock)
static DECLARE_FORWARD(ALCcaptureAlsa, ALCbackend, void, unlock)
DECLARE_DEFAULT_ALLOCATORS(ALCcaptureAlsa)

DEFINE_ALCBACKEND_VTABLE(ALCcaptureAlsa);


static void ALCcaptureAlsa_Construct(ALCcaptureAlsa *self, ALCdevice *device)
{
    ALCbackend_Construct(STATIC_CAST(ALCbackend, self), device);
    SET_VTABLE2(ALCcaptureAlsa, ALCbackend, self);
}


static ALCenum ALCcaptureAlsa_open(ALCcaptureAlsa *self, const ALCchar *name)
{
    return ALC_NO_ERROR;
}

static void ALCcaptureAlsa_close(ALCcaptureAlsa *self)
{

}

static ALCboolean ALCcaptureAlsa_start(ALCcaptureAlsa *self)
{
    return ALC_TRUE;
}

static void ALCcaptureAlsa_stop(ALCcaptureAlsa *self)
{

}

static ALCenum ALCcaptureAlsa_captureSamples(ALCcaptureAlsa *self, ALCvoid *buffer, ALCuint samples)
{
    return ALC_NO_ERROR;
}

static ALCuint ALCcaptureAlsa_availableSamples(ALCcaptureAlsa *self)
{
    return 0;
}

static ClockLatency ALCcaptureAlsa_getClockLatency(ALCcaptureAlsa *self)
{
    ALCdevice *device = STATIC_CAST(ALCbackend, self)->mDevice;
    //pcm_sframes_t delay = 0;
    ClockLatency ret;
    int err;

    return ret;
}


static inline void AppendAllDevicesList2(const DevMap *entry)
{ AppendAllDevicesList(alstr_get_cstr(entry->name)); }
static inline void AppendCaptureDeviceList2(const DevMap *entry)
{ AppendCaptureDeviceList(alstr_get_cstr(entry->name)); }

typedef struct ALCalsaBackendFactory {
    DERIVE_FROM_TYPE(ALCbackendFactory);
} ALCalsaBackendFactory;
#define ALCALSABACKENDFACTORY_INITIALIZER { { GET_VTABLE2(ALCalsaBackendFactory, ALCbackendFactory) } }

static ALCboolean ALCalsaBackendFactory_init(ALCalsaBackendFactory* UNUSED(self))
{
    return ALC_TRUE;
}

static void ALCalsaBackendFactory_deinit(ALCalsaBackendFactory* UNUSED(self))
{

}

static ALCboolean ALCalsaBackendFactory_querySupport(ALCalsaBackendFactory* UNUSED(self), ALCbackend_Type type)
{
    return ALC_TRUE;
}

static void ALCalsaBackendFactory_probe(ALCalsaBackendFactory* UNUSED(self), enum DevProbe type)
{
    
}

static ALCbackend* ALCalsaBackendFactory_createBackend(ALCalsaBackendFactory* UNUSED(self), ALCdevice *device, ALCbackend_Type type)
{
    if(type == ALCbackend_Playback)
    {
        ALCplaybackAlsa *backend;
        NEW_OBJ(backend, ALCplaybackAlsa)(device);
        if(!backend)
        	return NULL;
        return STATIC_CAST(ALCbackend, backend);
    }

    if(type == ALCbackend_Capture)
    {
        ALCcaptureAlsa *backend;
        NEW_OBJ(backend, ALCcaptureAlsa)(device);
        if(!backend)
        	return NULL;
        return STATIC_CAST(ALCbackend, backend);
    }

    return NULL;
}

DEFINE_ALCBACKENDFACTORY_VTABLE(ALCalsaBackendFactory);


ALCbackendFactory *ALCalsaBackendFactory_getFactory(void)
{
    static ALCalsaBackendFactory factory = ALCALSABACKENDFACTORY_INITIALIZER;
    return STATIC_CAST(ALCbackendFactory, &factory);
}
