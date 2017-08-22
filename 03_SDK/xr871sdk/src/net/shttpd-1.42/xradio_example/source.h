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

#ifndef __SHTTPD_SOURCE__H_
#define __SHTTPD_SOURCE__H_

#define index_html                                   \
"\r\n<html>\r\n"                                     \
"<head>\r\n"                                         \
"<title>XR871</title>\r\n"                           \
"</head>\r\n"                                        \
"<body>\r\n"                                         \
"<h1>XR871 WEBSERVER DEMO</h1>\r\n"                  \
"<form method=\"POST\" action=\"/config.shtml\">\r\n"\
"<fieldset>\r\n"                                     \
"<div>\r\n"                                          \
"<button type=\"submit\">Config</button>\r\n"        \
"</div>\r\n"                                         \
"</fieldset>\r\n"                                    \
"</form>\r\n"                                        \
"    <p>\r\n"                                        \
"<h2>XR871 Features:</h2>"                                    \
"32bit RISC CPU with FPU, up to 192MHz<br /><br />"              \
"High integration with RF, MCU, PMU and Memory<br /><br />"      \
"Dynamic Power Consumption Management<br /><br />"               \
"Embedded NET80211, Supplicant, TCP/IP protocol<br /><br />"     \
"6x6mm 52pin QFN<br /><br />"                                    \
"    </p>\r\n"                                       \
"</body>\r\n"                                        \
"</html>\r\n"

#define config_shtml                                                     \
"<!DOCTYPE html>\r\n"                                                    \
"<html lang=\"en\">\r\n"                                                 \
"<head>\r\n"                                                             \
"  <title>config device demo</title>\r\n"                                \
"</head>\r\n"                                                            \
"<body>\r\n"                                                             \
"  <div class=\"content\">\r\n"                                          \
"    <h1>Config device demo </h1>\r\n"                                   \
"    <p>\r\n"                                                            \
"      web server example\r\n"                                           \
"    </p>\r\n"                                                           \
"    <form method=\"POST\" action=\"/devicename\">\r\n"                  \
"      <fieldset>\r\n"                                                   \
"        <legend>Device settings</legend>\r\n"                           \
"        <label>DeviceName </label> <input type=\"text\"\r\n"            \
"          name=\"DeviceName\" value=\"<!--#call DeviceName -->\" >\r\n" \
"        <label>DevicePasswd </label> <input type=\"text\"\r\n"          \
"          name=\"DevicePasswd\" value=\"<!--#call DevicePasswd -->\" >\r\n"\
"        <div>\r\n"                                                      \
"          <button type=\"submit\">Save </button>\r\n"                   \
"        </div>\r\n"                                                     \
"      </fieldset>\r\n"                                                  \
"    </form>\r\n"                                                        \
"  </div>\r\n"                                                           \
"</body>\r\n"                                                            \
"</html>\r\n"                                                            \

//#define NO_CHECKSUM

#if !defined(NO_CHECKSUM)

#define checksum1_txt                                                    \
"1234545ighntegrationwithRFMCU12345ighintegrationwithRin+"

#define checksum2_txt                                                    \
"12345ighintegrat12345ighintegrationwithRFMCU12345ighntegrationwithRFMC" \
"345ighintegrationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabe"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"UandMelabelDevicePasswdlaUa12345ighintegrat12345ighintegrationwithRFMCU"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"cePasswdlabelinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=te"\
"xtrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=telabelDevicswdlabelinputtype=telabelDevicePasswdlabelinpu"\
"ttype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrn"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"UandMelabelDevicePasswdlaUa12345ighintegrat12345ighintegrationwithRFMCU"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"cePasswdlabelinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=te"\
"xtrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=telabelDevicswdlabelinputtype=telabelDevicePasswdlabelinpu"\
"ttype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrn"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"cePasswdlabelinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=te"\
"xtrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=telabelDevicswdlabelinputtype=telabelDevicePasswdlabelinpu"\
"ttype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrn"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"UandMelabelDevicePasswdlaUa12345ighintegrat12345ighintegrationwithRFMCU"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"345ighintegrationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabe"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"UandMelabelDevicePasswdlaUa12345ighintegrat12345ighintegrationwithRFMCU"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"cePasswdlabelinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=te"\
"xtrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=telabelDevicswdlabelinputtype=telabelDevicePasswdlabelinpu"\
"ttype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrn"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"345ighintegrationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabe"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRF"\
"MCU12345ighintegrationwithRFMCUPMandMerationwithRintegrationwithRFMCUPM"\
"UandMelabelDevicePasswdlaUa12345ighintegrat12345ighintegrationwithRFMCU"\
"12345ighntegrationwithRFMCU12345ighintegrationwithRintegrationwithRFMCU"\
"PMUandMelabelDevicePasswdlabelinputtype=textrnPMUandMelabelDevicePasswd"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"cePasswdlabelinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=te"\
"xtrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=telabelDevicswdlabelinputtype=telabelDevicePasswdlabelinpu"\
"ttype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrn"\
"abelinputtype=textrnionwithRFMCU12345ighintegrationwithRFMCUPMUandMerat"\
"ionwithRintegrationwithRFMCUPMUandMelabelDevicePasswdlabelinputtype=tex"\
"trnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevicePasswdla"\
"belinputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwit"\
"hRFMCU12345ighintegrationwithRFMCUPMUandMelabelDevicePascePasswdlabelin"\
"uttype=textrnPMUandndMerationwithRintegrationwithRFMCUPMUandMelabelDevi"\
"inputtype=textrnPMUandMelabelDevicePasswdlabelinputtype=textrnionwithRFWWZ"
#endif

#endif /* __SHTTPD_SOURCE__H_ */
