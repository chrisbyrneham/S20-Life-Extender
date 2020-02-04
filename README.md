# S20-Life-Extender

Keeps Orvibo S20 sockets viable by synchronising the time

This arduino sketch for an ESP8266 effectively keeps the switches working.

The cost of an ESP8266 is only a few pounds. 

Orvibo the suppliers of the switches stopped hosting the server that was called to get the time, effectively rendering them useless.

When the switches are powered on they can't get access to the server to do a time update so revert they to 1 Jan 1900.

There are four ways of setting the necessary SSID, Password, and Timezone offset parameters;

1) Hard code them.
2) Enter by using the arduino IDE serial monitor.
3) Enter by using a standalone access point TEMPSSID ( no password ) on a web server on http://192.168.1.100
4) I can supply ESP8266s programmed and ready to go for £10.00 plus postage.
