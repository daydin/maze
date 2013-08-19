/**********************************************************************************/
/* File:         radio.h                                                          */
/* Date:         Oct 22, 2008                                                     */
/* Description:  Webots C programming interface for the Radio node                */
/* Authors:      Olivier Michel                                                   */
/*                                                                                */
/* Copyright (c) 2008 Cyberbotics - www.cyberbotics.com                           */
/**********************************************************************************/

#ifndef WB_RADIO_H
#define WB_RADIO_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *WbRadioMessage;
typedef void *WbRadioEvent;

WbRadioMessage  wb_radio_message_new(int length,const char *body,const char *destination); 
void            wb_radio_message_delete(WbRadioMessage);
const char     *wb_radio_message_get_destination(WbRadioMessage);
int             wb_radio_message_get_length(WbRadioMessage);
const char     *wb_radio_message_get_body(WbRadioMessage);

void            wb_radio_enable(WbDeviceTag tag, int ms);
void            wb_radio_disable(WbDeviceTag tag);

void            wb_radio_set_address(WbDeviceTag tag, const char *address);
const char     *wb_radio_get_address(WbDeviceTag tag);

void            wb_radio_set_frequency(WbDeviceTag tag, double hz);
double          wb_radio_get_frequency(WbDeviceTag tag);

void            wb_radio_set_channel(WbDeviceTag tag, int channel);
int             wb_radio_get_channel(WbDeviceTag tag);

void            wb_radio_set_bitrate(WbDeviceTag tag, int bits_per_second);
int             wb_radio_get_bitrate(WbDeviceTag tag);

void            wb_radio_set_rx_sensitivity(WbDeviceTag tag, double dBm);
double          wb_radio_get_rx_sensitivity(WbDeviceTag tag);

void            wb_radio_set_tx_power(WbDeviceTag tag, double dBm);
double          wb_radio_get_tx_power(WbDeviceTag tag);

void            wb_radio_set_callback(WbDeviceTag tag, void (*)(const WbRadioEvent));

WbDeviceTag     wb_radio_event_get_radio(const WbRadioEvent);
char           *wb_radio_event_get_data(const WbRadioEvent);
int             wb_radio_event_get_data_size(const WbRadioEvent);
char           *wb_radio_event_get_emitter(const WbRadioEvent);
double          wb_radio_event_get_rssi(const WbRadioEvent);

void            wb_radio_send(WbDeviceTag tag, const WbRadioMessage, double delay);

#ifdef __cplusplus
}
#endif

#endif /* WB_RADIO_H */

