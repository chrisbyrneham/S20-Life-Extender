# S20-Life-Extender

Copyright (C) 2020 by Chris Byrneham.

Keeps Orvibo(c) S20 sockets viable by synchronising the time.

This arduino sketch for an ESP8266 effectively keeps the switches working.

The cost of an ESP8266 is only a few pounds. 

Orvibo the suppliers of the switches stopped hosting the server that was called to get time updates, effectively rendering them useless.

This software was developed independently and is not supported or endorsed in any way by Orvibo (c).

The switches are normally controlled by the Wiwo phone app which still works for me but,
when the switches are powered on they can't get access to the server to do a time update so they default to 1 Jan 1900.

In order to use this sketch with a blank ESP8266 you must know how to use the Arduino IDE and be able to use the board manager, add extra libraries etc. Otherwise see option 4 below.

When the sketch is running on the ESP8266 you must set some parameters.

When the sketch starts it tries to load previous parameters from EEPROM.
Then it listens to the serial monitor for a while.
Then it listens on a web page on a temporary access point for a while.
Then it uses hard coded values if present.

There are four ways of setting the necessary SSID, Password, and Timezone offset parameters;

1) Hard code them.
2) Enter by using the arduino IDE serial monitor.
3) Enter by using a standalone access point TEMPSSID ( no password ) on a web server on http://192.168.1.100
4) I can supply ESP8266s programmed and ready to go for £10.00 plus postage. Hard coding values if required.


## Hard code them.

Enter the values into the three variables named hardCoded??????;

## Use the serial monitor.

Use the free Arduino IDE to upload the sketch, then use the serial monitor to enter the SSID,Password, and offset in that order.

## Use the temporary access point.

Use the TEMPSSID WiFi access point and go to http://192.168.1.100 to enter the values.

## Pay me to do it.

Email me and, or send £10 plus appropriate postage from U.K. to your country. I can hard code the values for you if required.

Paypal Contributions are appreciated  chris@byrneham.com


## References and thanks to;

Fernando M Silva  fcr@netcabo.pt , A useful PHP web app for S20s. 

Andrius A Tikonas , for his work on reverse engineering the protocol.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
