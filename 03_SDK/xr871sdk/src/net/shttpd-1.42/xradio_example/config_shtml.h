#ifndef _CONFIG_SHTML_H
#define _CONFIG_SHTML_H

#define config_shtml "<!DOCTYPE html>\r\n"\
"<html lang=\"en\">\r\n"\
"<head>\r\n"\
"  <title>config device demo</title>\r\n"\
"</head>\r\n"\
"<body>\r\n"\
"  <div class=\"content\">\r\n"\
"    <h1>Config device demo </h1>\r\n"\
"    <p>\r\n"\
"      This page is one of the series of examples \r\n"\
"    </p>\r\n"\
"    <form method=\"POST\" action=\"/devicename\">\r\n"\
"      <fieldset>\r\n"\
"        <legend>Device settings</legend>\r\n"\
"        <label>DeviceName </label> <input type=\"text\"\r\n"\
"          name=\"DeviceName\" value=\"<!--#call DeviceName -->\" >\r\n"\
"        <div>\r\n"\
"          <button type=\"submit\">Save </button>\r\n"\
"        </div>\r\n"\
"      </fieldset>\r\n"\
"    </form>\r\n"\
"  </div>\r\n"\
"</body>\r\n"\
"</html>\r\n"\

#endif
