#ifndef __MQTTDEBUG_H__
#define __MQTTDEBUG_H__

#include "module_debug.h"
#include "stdint.h"

#define MQTT_DBG_ON
#define MOTT_ASSERT_ON
//#define MQTT_PLATFORM_DBG

#define MQTT_MODULE (DBG_ON | MLEVEL_WARNING)

#ifdef MOTT_ASSERT_ON
#define MQTT_ASSERT(assert, msg, arg...) MALERT(MOD_DBG_ALW_ON, (assert), "[MQTT assert] "msg, ##arg)
#else
#define MQTT_ASSERT(assert, msg, arg...)
#endif

#ifdef MQTT_DBG_ON

#define MQTT_INFO(msg, arg...) MINFO(MQTT_MODULE, NOEXPAND, "[MQTT info] " msg, ##arg)

#define MQTT_WARN(msg, arg...) MWARN(MQTT_MODULE, NOEXPAND, "[MQTT warning] " msg, ##arg)

#define MQTT_DEBUG(msg, arg...) MDEBUG(MQTT_MODULE, NOEXPAND, "[MQTT debug] " msg, ##arg)


#define MQTT_CAP_SEND(rc, c, len) \
	if (rc >= 0) { \
		char printbuf[150]; \
		MQTT_DEBUG("Rc %d from sending packet %s\n", rc,\
			   MQTTFormat_toServerString(printbuf, sizeof(printbuf), c->buf, len)); \
	}

#define MQTT_CAP_RECV(rc, c, len) \
	if (rc >= 0) { \
		char printbuf[100]; \
		MQTT_DEBUG("Rc %d from receiving packet %s\n", rc,\
			   MQTTFormat_toClientString(printbuf, sizeof(printbuf), c->readbuf, len)); \
	}



#define MQTT_ENTRY() MENTRY(MQTT_MODULE, "[MQTT entry] ")

#define MQTT_EXIT(ret) MRET(MQTT_MODULE, "[MQTT return] ", ret)

#else /* MQTT_DBG_ON */

#define MQTT_INFO(msg, arg...) 

#define MQTT_WARN(msg, arg...)

#define MQTT_DEBUG(msg, arg...) 


#define MQTT_CAP_SEND(rc, c, len)

#define MQTT_CAP_RECV(rc, c, len)


#define MQTT_ENTRY() 

#define MQTT_EXIT(ret) 

#endif /* MQTT_DBG_ON */


#if defined(MQTT_PLATFORM_DBG) && defined(MQTT_DBG_ON)

#define MQTT_PLATFORM (DBG_ON | MLEVEL_DEBUG)

#define MQTT_PLATFORM_WARN(msg, arg...) MWARN(MQTT_MODULE, NOEXPAND, "[MQTT PLATFORM warning] " msg, ##arg) 

#define MQTT_PLATFORM_ENTRY() MENTRY(MQTT_PLATFORM, "[MQTT PLATFORM entry] ")

#define MQTT_PLATFORM_EXIT(ret) MRET(MQTT_PLATFORM, "[MQTT PLATFORM return] ", ret)
#else /* MQTT_PLATFORM_DBG && MQTT_DBG_ON*/

#define MQTT_PLATFORM_WARN(msg, arg...) 

#define MQTT_PLATFORM_ENTRY() 

#define MQTT_PLATFORM_EXIT(ret) 

#endif /* MQTT_PLATFORM_DBG && MQTT_DBG_ON*/


#endif
