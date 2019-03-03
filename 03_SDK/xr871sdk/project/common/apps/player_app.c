/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef __PRJ_CONFIG_XPLAYER

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "cedarx/xplayer/include/xplayer.h"
#include "driver/chip/hal_codec.h"
#include "audio/manager/audio_manager.h"
#include "kernel/os/os.h"
#include "util/atomic.h"
#include "sys/defs.h"
#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "player_app.h"

#define PLAYER_LOGD(msg, arg...)      //printf("[PLAYER_DBG] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define PLAYER_LOGI(msg, arg...)      printf("[PLAYER_INFO] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define PLAYER_LOGW(msg, arg...)      printf("[PLAYER_WRN] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)
#define PLAYER_LOGE(msg, arg...)      printf("[PLAYER_ERR] <%s : %d> " msg "\n", __func__, __LINE__, ##arg)

#if APP_PLAYER_SUPPORT_TONE
struct tone_base
{
    int (*play)(tone_base *base, char *url);
    int (*stop)(tone_base *base);
    int (*destroy)(tone_base *base);
    app_player_callback cb;
    void *arg;
};
#endif

typedef struct player_info
{
    char *url;
    uint32_t size;
#if APP_PLAYER_SUPPORT_TONE
    int pause_time;
#endif
} player_info;

typedef struct app_player
{
    player_base base;
    XPlayer *xplayer;
    SoundCtrl *sound;
    aplayer_states state;
    app_player_callback cb;
    player_info info;
    OS_Mutex_t lock;
    uint8_t mute;
#if APP_PLAYER_SUPPORT_TONE
    tone_base my_tone;
    tone_base *tone;
#endif
    int vol;
    uint16_t id;
    void *arg;
} app_player;

static app_player *player_singleton = NULL;

#if APP_PLAYER_SUPPORT_TONE
static void tone_handler(event_msg *msg);
#endif

static void player_handler(event_msg *msg);
static int player_stop(player_base *base);
static int player_seturl(player_base *base, const char *url);

#define APLAYER_ID_AND_STATE(id, state)     (((id) << 16) | (state))
#define APLAYER_GET_ID(id_state)            (((id_state) >> 16) & 0xFFFF)
#define APLAYER_GET_STATE(id_state)         ((id_state) & 0xFFFF)


static inline app_player *get_player()
{
    return player_singleton;
}

#if APP_PLAYER_SUPPORT_TONE
static inline void set_current_time(app_player *impl)
{
    if(XPlayerGetCurrentPosition(impl->xplayer, &impl->info.pause_time) != 0) {
        PLAYER_LOGW("tell() return fail.");
    }
}

static inline bool is_toning(app_player *impl)
{
    return impl->tone != NULL;
}
#endif

static inline void set_player_handler(app_player *impl, void (**handler)(event_msg *msg))
{
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
        *handler = tone_handler;
    else
#endif
        *handler = player_handler;
}

static void *wrap_realloc(void *p, uint32_t *osize, uint32_t nsize)
{
    if (p == NULL) {
        *osize = nsize;
        return malloc(nsize);
    }
    if (*osize >= nsize)
        return p;
    free(p);
    PLAYER_LOGD("free %d, malloc %d", *osize, nsize);
    *osize = nsize;
    return malloc(nsize);
}

static int set_url(app_player *impl, char* pUrl)
{
    //* set url to the AwPlayer.
    if(XPlayerSetDataSourceUrl(impl->xplayer,
                 (const char*)pUrl, NULL, NULL) != 0)
    {
        PLAYER_LOGE("setDataSource() return fail.");
        return -1;
    }

    if ((!strncmp(pUrl, "http://", 7)) || (!strncmp(pUrl, "https://", 8))) {
        if(XPlayerPrepareAsync(impl->xplayer) != 0)
        {
            PLAYER_LOGE("prepareAsync() return fail.");
            return -1;
        }
    } else {
        void (*handler)(event_msg *);
        set_player_handler(impl, &handler);
        sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_MEDIA_PREPARED), 10000);
    }
    return 0;
}

static int play(app_player *impl)
{
    if(XPlayerStart(impl->xplayer) != 0)
    {
        PLAYER_LOGE("start() return fail.");
        return -1;
    }
    PLAYER_LOGD("playing");
    return 0;
}

static int reset(app_player *impl)
{
    if(XPlayerReset(impl->xplayer) != 0)
    {
        PLAYER_LOGE("reset() return fail.");
        return -1;
    }
    impl->id++;
    return 0;
}

static void player_handler_preprocess(app_player *impl, player_events evt)
{
    PLAYER_LOGI("event: %d", (int)evt);

    switch (evt)
    {
        case PLAYER_EVENTS_MEDIA_PREPARED:
#if APP_PLAYER_SUPPORT_TONE
            if (impl->info.pause_time != 0)
            {
                if(XPlayerSeekTo(impl->xplayer, impl->info.pause_time) != 0) {
                    PLAYER_LOGE("seek() return fail.");
                }
                PLAYER_LOGI("seek to %d ms.", impl->info.pause_time);
                impl->info.pause_time = 0;
            }
#endif
            impl->cb(evt, NULL, impl->arg);
            if (impl->state != APLAYER_STATES_PAUSE)
            {
                play(impl);
            }
            impl->state = APLAYER_STATES_PLAYING;

            break;
        case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
        case PLAYER_EVENTS_MEDIA_ERROR:
            reset(impl);
            impl->state = APLAYER_STATES_STOPPED;
            impl->cb(evt, NULL, impl->arg);
            break;
        default:
            impl->cb(evt, NULL, impl->arg);
            break;
    }

    return;
}

static void player_handler(event_msg *msg)
{
    app_player *impl = get_player();
    player_events state = (player_events)APLAYER_GET_STATE(msg->data);

    OS_RecursiveMutexLock(&impl->lock, -1);
    if (APLAYER_GET_ID(msg->data) != impl->id)
        goto out;

    player_handler_preprocess(impl, state);

out:
    OS_RecursiveMutexUnlock(&impl->lock);
}

#if APP_PLAYER_SUPPORT_TONE
static void tone_handler(event_msg *msg)
{
    app_player* impl = get_player();
    player_events state = (player_events)APLAYER_GET_STATE(msg->data);
    app_player_callback cb = impl->tone->cb;

    OS_RecursiveMutexLock(&impl->lock, -1);
    if (APLAYER_GET_ID(msg->data) != impl->id)
        goto out;

    PLAYER_LOGI("tone event: %d, state: %d", (int)state, (int)impl->state);

    switch (state)
    {
        case PLAYER_EVENTS_MEDIA_PREPARED:
            play(impl);
            break;
        case PLAYER_EVENTS_TONE_STOPED:
            impl->tone = NULL;
            reset(impl);
            if (impl->state != APLAYER_STATES_STOPPED && impl->state != APLAYER_STATES_INIT)
                set_url(impl, impl->info.url);
            cb(PLAYER_EVENTS_TONE_STOPED, NULL, impl->tone->arg);
            break;
        case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
            impl->tone = NULL;
            reset(impl);
            if (impl->state != APLAYER_STATES_STOPPED && impl->state != APLAYER_STATES_INIT)
                set_url(impl, impl->info.url);
            cb(PLAYER_EVENTS_TONE_COMPLETE, NULL, impl->tone->arg);
            break;
        case PLAYER_EVENTS_MEDIA_ERROR:
            impl->tone = NULL;
            reset(impl);
            if (impl->state != APLAYER_STATES_STOPPED && impl->state != APLAYER_STATES_INIT)
                set_url(impl, impl->info.url);
            cb(PLAYER_EVENTS_TONE_ERROR, NULL, impl->tone->arg);
            break;
        default:
            break;
    }
out:
    OS_RecursiveMutexUnlock(&impl->lock);
}
#endif

static int player_callback(void* pUserData, int msg, int ext1, void* param)
{
    app_player* impl = (app_player*)pUserData;
    void (*handler)(event_msg *msg);

    set_player_handler(impl, &handler);

    switch(msg)
    {
        case AWPLAYER_MEDIA_INFO:
            switch(ext1)
            {
                case AW_MEDIA_INFO_NOT_SEEKABLE:
                    PLAYER_LOGI("media source is unseekable.");
                    break;
            }
            break;

        case AWPLAYER_MEDIA_ERROR:
            PLAYER_LOGW("open media source fail.\n"
                            "reason: maybe the network is bad, or the music file is not good.");
            sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_MEDIA_ERROR), 10000);
            break;

        case AWPLAYER_MEDIA_PREPARED:
            sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_MEDIA_PREPARED), 10000);
            break;

        case AWPLAYER_MEDIA_PLAYBACK_COMPLETE:
            PLAYER_LOGI("playback complete.");
            sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE), 10000);
            break;

        case AWPLAYER_MEDIA_SEEK_COMPLETE:
            PLAYER_LOGI("seek ok.");
            break;

        default:
            PLAYER_LOGD("unknown callback from AwPlayer");
            break;
    }

    PLAYER_LOGI("cedarx cb complete.");
    return 0;
}

static int player_stop(player_base *base)
{
    void (*handler)(event_msg *);
    app_player *impl = container_of(base, app_player, base);

    OS_RecursiveMutexLock(&impl->lock, -1);
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
#endif
    reset(impl);
    impl->state = APLAYER_STATES_STOPPED;

    set_player_handler(impl, &handler);
    sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_MEDIA_STOPPED), 10000);

    OS_RecursiveMutexUnlock(&impl->lock);

    return 0;
}

static int player_pause(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    int ret = 0;

    OS_RecursiveMutexLock(&impl->lock, -1);
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
#endif
    ret = XPlayerPause(impl->xplayer);
    impl->state = APLAYER_STATES_PAUSE;
    OS_RecursiveMutexUnlock(&impl->lock);

    return ret;
}

static int player_resume(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    int ret = 0;

    OS_RecursiveMutexLock(&impl->lock, -1);
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
#endif
    ret = play(impl);

#if APP_PLAYER_SUPPORT_TONE
    impl->info.pause_time = 0;
#endif
    impl->state = APLAYER_STATES_PLAYING;
    OS_RecursiveMutexUnlock(&impl->lock);

    return ret;
}

static int player_seek(player_base *base, int ms)
{
    app_player *impl = container_of(base, app_player, base);
    int ret = 0;

    OS_RecursiveMutexLock(&impl->lock, -1);
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }

    impl->info.pause_time = ms;
#endif

    if(XPlayerSeekTo(impl->xplayer, ms) != 0)
    {
        PLAYER_LOGE("seek() return fail.");
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
    PLAYER_LOGI("seek to %d ms.", ms);
    OS_RecursiveMutexUnlock(&impl->lock);

    return ret;
}

static int player_seturl(player_base *base, const char *url)
{
    int ret = 0;
    app_player *impl = container_of(base, app_player, base);
    char *play_url = (char *)url;

    if (url == NULL)
        return -1;

    OS_RecursiveMutexLock(&impl->lock, -1);
#if APP_PLAYER_SUPPORT_TONE
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
#endif

    impl->state = APLAYER_STATES_PREPARING;

    PLAYER_LOGI("request to play : %s", url);

#if APP_PLAYER_SUPPORT_TONE
    impl->info.pause_time = 0;
#endif
    impl->info.url = wrap_realloc(impl->info.url, &impl->info.size, strlen(play_url) + 1);
    memcpy(impl->info.url, play_url, strlen(play_url) + 1);
    ret = set_url(impl, impl->info.url);
    if (ret) {
        reset(impl);
        impl->state = APLAYER_STATES_STOPPED;
    }
    OS_RecursiveMutexUnlock(&impl->lock);

    return ret;
}

static int player_tell(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    int ms;

    OS_RecursiveMutexLock(&impl->lock, -1);
    if(XPlayerGetCurrentPosition(impl->xplayer, &ms) != 0)
    {
        PLAYER_LOGW("tell() return fail.");
        OS_RecursiveMutexUnlock(&impl->lock);
        return 0;
    }
    PLAYER_LOGI("tell to %d ms.", ms);
    OS_RecursiveMutexUnlock(&impl->lock);
    return ms;
}

static int player_size(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    int ms;

    OS_RecursiveMutexLock(&impl->lock, -1);
    if(XPlayerGetDuration(impl->xplayer, &ms) != 0)
    {
        PLAYER_LOGW("size() return fail.");
        OS_RecursiveMutexUnlock(&impl->lock);
        return 0;
    }
    PLAYER_LOGI("size to %d ms.", ms);
    OS_RecursiveMutexUnlock(&impl->lock);
    return ms;
}

static int player_setvol(player_base *base, int vol)
{
    app_player *impl = container_of(base, app_player, base);

    if (vol > 31)
    {
        PLAYER_LOGW("set vol %d larger than 31", vol);
        vol = 31;
    }
    else if (vol < 0)
    {
        PLAYER_LOGW("set vol %d lesser than 0", vol);
        vol = 0;
    }

    impl->vol = vol;
    aud_mgr_handler(AUDIO_DEVICE_MANAGER_VOLUME, AUDIO_OUT_DEV_SPEAKER, vol);
    return 0;
}

static int player_getvol(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    return impl->vol;
}

static int player_mute(player_base *base, bool is_mute)
{
    app_player *impl = container_of(base, app_player, base);
    impl->mute = (uint8_t)is_mute;
    aud_mgr_handler(AUDIO_DEVICE_MANAGER_MUTE, AUDIO_OUT_DEV_SPEAKER, is_mute);
    return 0;
}

static int player_is_mute(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    return impl->mute;
}

static aplayer_states player_get_states(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);

    return impl->state;
}

static int player_control(player_base *base, int command, void *data)
{
    app_player *impl = container_of(base, app_player, base);
    (void)impl;
    /* TODO: tbc... */
    return -1;
}

#if APP_PLAYER_SUPPORT_TONE
static int player_playtone(player_base *base, tone_base *drv, char *url)
{
    app_player *impl = container_of(base, app_player, base);
    int ret = 0;

    OS_RecursiveMutexLock(&impl->lock, -1);
    if (is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
    set_current_time(impl);
    reset(impl);
    impl->tone = drv;
    ret = drv->play(drv, url);
    OS_RecursiveMutexUnlock(&impl->lock);

    /* TODO: how to resume music if use other tone drv? */

    return ret;
}

static int player_stoptone(player_base *base, tone_base *drv)
{
    app_player *impl = container_of(base, app_player, base);

    OS_RecursiveMutexLock(&impl->lock, -1);
    if (!is_toning(impl))
    {
        OS_RecursiveMutexUnlock(&impl->lock);
        return -1;
    }
    drv->stop(drv);
    OS_RecursiveMutexUnlock(&impl->lock);

    return 0;
}

static int player_tone_play(tone_base *base, char *url)
{
    app_player *impl = container_of(base, app_player, my_tone);
    set_url(impl, url);
    return 0;
}

static int player_tone_stop(tone_base *base)
{
    app_player *impl = container_of(base, app_player, my_tone);

    void (*handler)(event_msg *);
    set_player_handler(impl, &handler);
    sys_handler_send(handler, APLAYER_ID_AND_STATE(impl->id, PLAYER_EVENTS_TONE_STOPED), 10000);

    return 0;
}

static int player_tone_destroy(tone_base *base)
{
    return 0;
}
#endif

static void player_null_callback(player_events event, void *data, void *arg)
{
    PLAYER_LOGI("cb event:%d", event);
}

static void player_setcb(player_base *base, app_player_callback cb, void *arg)
{
    app_player *impl = container_of(base, app_player, base);
    if (!cb)
        impl->cb = player_null_callback;
    else
        impl->cb = cb;
    impl->arg = arg;
}

player_base *player_create()
{
    if (player_singleton)
        return &player_singleton->base;

    app_player *impl = malloc(sizeof(*impl));
    if (impl == NULL)
        return NULL;
    memset(impl, 0, sizeof(*impl));

    impl->xplayer = XPlayerCreate();
    if(impl->xplayer == NULL)
        goto failed;

    //* set callback to player.
    XPlayerSetNotifyCallback(impl->xplayer, player_callback, (void*)impl);

    //* check if the player work.
    if(XPlayerInitCheck(impl->xplayer) != 0)
        goto failed;

SoundCtrl* SoundDeviceCreate();
    impl->sound = SoundDeviceCreate();
    XPlayerSetAudioSink(impl->xplayer, (void*)impl->sound);

    impl->base.play         = player_seturl;
    impl->base.stop         = player_stop;
    impl->base.pause        = player_pause;
    impl->base.resume       = player_resume;
    impl->base.seek         = player_seek;
    impl->base.tell         = player_tell;
    impl->base.size         = player_size;
    impl->base.setvol       = player_setvol;
    impl->base.getvol       = player_getvol;
    impl->base.mute         = player_mute;
    impl->base.is_mute      = player_is_mute;
    impl->base.control      = player_control;
#if APP_PLAYER_SUPPORT_TONE
    impl->base.play_tone    = player_playtone;
    impl->base.stop_tone    = player_stoptone;
#endif
    impl->base.set_callback = player_setcb;
    impl->base.get_status   = player_get_states;
    impl->cb                = player_null_callback;

    OS_RecursiveMutexCreate(&impl->lock);

    impl->state = APLAYER_STATES_INIT;

    player_singleton = impl;

    return &impl->base;

failed:
    PLAYER_LOGE("create player failed, quit.");
    if (impl->sound)
        free(impl->sound);
    if (impl->xplayer != NULL)
        XPlayerDestroy(impl->xplayer);
    if (impl != NULL)
        free(impl);
    return NULL;
}

void player_destroy(player_base *base)
{
    app_player *impl = container_of(base, app_player, base);
    if (player_singleton == NULL)
        return;

    PLAYER_LOGI("destroy AwPlayer.");
    player_stop(base);
    if (impl->xplayer != NULL)
        XPlayerDestroy(impl->xplayer);
    if (impl->info.url)
        free(impl->info.url);
    if (impl != NULL)
        free(impl);
    player_singleton = NULL;
}

#if APP_PLAYER_SUPPORT_TONE
tone_base * player_tone_create(player_base *base, app_player_callback cb, void *arg)
{
    app_player *impl = container_of(base, app_player, base);
    impl->my_tone.cb = cb;
    impl->my_tone.play = player_tone_play;
    impl->my_tone.stop = player_tone_stop;
    impl->my_tone.destroy = player_tone_destroy;
    impl->my_tone.arg = arg;

    return &impl->my_tone;
}

void tone_destroy(tone_base *base)
{
    base->destroy(base);
}
#endif

#endif

