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

#ifndef _APPS_PLAYER_APP_H_
#define _APPS_PLAYER_APP_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PLAYER_SUPPORT_TONE  0

#if APP_PLAYER_SUPPORT_TONE
typedef struct tone_base tone_base;
#endif

typedef enum aplayer_states
{
    APLAYER_STATES_PAUSE,
    APLAYER_STATES_PLAYING,
    APLAYER_STATES_PREPARING,
    APLAYER_STATES_PREPARED,
    APLAYER_STATES_STOPPED,
    APLAYER_STATES_RESUME,
    APLAYER_STATES_INIT,
    APLAYER_STATES_NONE,
    APLAYER_STATES_COMPLETED,
    APLAYER_STATES_ERROR,
#if APP_PLAYER_SUPPORT_TONE
    APLAYER_STATES_TONE_START,
    APLAYER_STATES_TONE_PREPARING,
    APLAYER_STATES_TONE_PLAYING,
    APLAYER_STATES_TONE_STOPED,
    APLAYER_STATES_TONE_COMPLETE,
    APLAYER_STATES_TONE_ERROR,
#endif
} aplayer_states;

typedef enum player_events
{
    PLAYER_EVENTS_NONE                      = APLAYER_STATES_NONE,
    PLAYER_EVENTS_MEDIA_PREPARED            = APLAYER_STATES_PLAYING,
    PLAYER_EVENTS_MEDIA_START_PLAY          = APLAYER_STATES_PLAYING,
    PLAYER_EVENTS_MEDIA_STOPPED             = APLAYER_STATES_STOPPED,
    PLAYER_EVENTS_MEDIA_ERROR               = APLAYER_STATES_ERROR,
    PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE   = APLAYER_STATES_COMPLETED,
#if APP_PLAYER_SUPPORT_TONE
    PLAYER_EVENTS_TONE_STOPED               = APLAYER_STATES_TONE_STOPED,
    PLAYER_EVENTS_TONE_COMPLETE             = APLAYER_STATES_TONE_COMPLETE,
    PLAYER_EVENTS_TONE_ERROR                = APLAYER_STATES_TONE_ERROR,
#endif
} player_events;

typedef void (*app_player_callback)(player_events event, void *data, void *arg);

typedef struct player_base
{
    int (*play)(struct player_base *base, const char *url);
    int (*stop)(struct player_base *base);
    int (*pause)(struct player_base *base);
    int (*resume)(struct player_base *base);
    int (*seek)(struct player_base *base, int ms);
    int (*tell)(struct player_base *base);
    int (*size)(struct player_base *base);
    int (*setvol)(struct player_base *base, int vol);
    int (*getvol)(struct player_base *base);
    int (*mute)(struct player_base *base, bool is_mute);
    int (*is_mute)(struct player_base *base);
    int (*control)(struct player_base *base, int command, void *data);
#if APP_PLAYER_SUPPORT_TONE
    int (*play_tone)(struct player_base *base, tone_base *drv, char *url);
    int (*stop_tone)(struct player_base *base, tone_base *drv);
#endif
    void (*set_callback)(struct player_base *base, app_player_callback cb, void *arg); /* callback can't use player interface */
    aplayer_states (*get_status)(struct player_base *base);
} player_base;

player_base * player_create();
void player_destroy(player_base *base);

#if APP_PLAYER_SUPPORT_TONE
tone_base * player_tone_create(player_base *player, app_player_callback cb, void *arg);
void tone_destroy(tone_base *base);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _APPS_PLAYER_APP_H_ */
