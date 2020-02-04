# S20-Life-Extender

Copyright (C) 2020 by Chris Byrneham

Keeps Orvibo S20 sockets viable by synchronising the time

This arduino sketch for an ESP8266 effectively keeps the switches working.

The cost of an ESP8266 is only a few pounds. 

Orvibo the suppliers of the switches stopped hosting the server that was called to get the time, effectively rendering them useless.

This software was developed independently and is not supported or endorsed in any way by Orvibo (c).

The switches are normally controlled by the Wiwo app which still works but,
when the switches are powered on they can't get access to the server to do a time update so they default to 1 Jan 1900.

In order to use this sketch you must set some parameters.

When the sketch starts it tries to load previous parameters from EEPROM.


There are four ways of setting the necessary SSID, Password, and Timezone offset parameters;

1) Hard code them.
2) Enter by using the arduino IDE serial monitor.
3) Enter by using a standalone access point TEMPSSID ( no password ) on a web server on http://192.168.1.100
4) I can supply ESP8266s programmed and ready to go for £10.00 plus postage.


1) Hard code them.

Enter the values into the three variables named hardCoded??????;

2) Use the serial monitor.

Use the free Arduino IDE to upload the sketch, then use the serial monitor to enter the SSID,Password, and offset in that order.

3) Use the temporary access point.

Use the TEMPSSID WiFi access point and go to http://192.168.1.100 to enter the values.

4) Pay me to do it.

Email me and, or send £10 plus appropriate postage from U.K. to your country. I can hard code the values for you.


Paypal Contributions appreciated  chris@byrneham.com

References and thanks to;

Fernando M Silva  fcr@netcabo.pt , A PHP web app for S20s 

Andrius A Tikonas , for initial work on reverse engineering the protocol.

