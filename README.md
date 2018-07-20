# About VISCA-Dump

A longer time ago, I had the need to observe the communication between a
controlling device and a SONY camera. The camera used the VISCA protocol.
To get some idea what data is transfered, I wrote this small dumping tool.


# Usage

To use `visca-dump`, you must have two serial ports. One is connected to the
controlling device, called "sender". The second port must be connected to the
camera, called "receiver".

The VISCA protocol is a *master-slave* protocol. The "sender" is the master and
the "receiver" is the slave. This means, the "sender" sends the commands and
the "receiver" replies to them.

With `visca-dump`, this communication is analyzed and logged. To get
information about the timing, the replay time is analyzed too.

## Example

````
./visca-dump -r /dev/ttyUSB0 -s /dev/ttyUSB1
````

A possible log looks like this:

````
10:18:09[0468] CTL: 81 01 00 01 FF                                  {    /       }  - CMD: IfClear
10:18:09[0500] CAM: 90 50 FF                                        {0032/ 41.74D}  - RPL: Done
10:18:09[0516] CTL: 81 09 04 00 FF                                  {    /       }  - CMD: PowerInq
10:18:09[0564] CAM: 90 50 02 FF                                     {0047/ 41.79D}  - RPL: Byte
10:18:09[0579] CTL: 81 01 04 47 03 03 06 04 FF                      {    /       }  - CMD: ZoomDirect
10:18:09[0691] CTL: 81 01 04 39 00 FF                               {    /       }  - CMD: AE
10:18:09[0692] CAM: 90 41 FF                                        {    /       }  - RPL: Ack Sock1
10:18:09[0692] CAM: 90 51 FF                                        {    /       }  - RPL: Done Sock1
10:18:09[0755] CTL: 81 01 04 38 02 FF                               {    /       }  - CMD: FocusMode
10:18:09[0755] CAM: 90 41 FF                                        {    /       }  - RPL: Ack Sock1
10:18:09[0755] CAM: 90 51 FF                                        {    /       }  - RPL: Done Sock1
10:18:09[0803] CAM: 90 41 FF                                        {    /       }  - RPL: Ack Sock1
 ~~~~~~~~~~~~~~~~~ ack=11.000000 (49) | done=40.525253 (99) [ms] | unknown=0/1 | errors=0/0
10:18:09[0867] CTL: 81 01 04 74 03 FF                               {    /       }  - CMD: Title
10:18:09[0883] CAM: 90 41 FF                                        {0016/ 11.10A}  - RPL: Ack Sock1
````

**Bug:** as you can see, there is a problem left if the commands are nested!

Each packet is logged in an single line:

````
(1)             (2)  (3)                                                (4)  (5)     (6)
_v_____________ _v__ _v_____________________________________________    _v__ _v____  _v________________
"HH:MM:SS[mmmm] NNN: xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx - {ssss/sss.ss} tttttttttttttttttt"
````

The logged fields are:

* (1) the timestamp including milliseconds.
* (2) the "name" of the interface. The sender is called "CTL" and the receiver "CAM".
* (3) the raw dump of the packet
* (4) the time difference in *ms* between the command from the sender and the reply.
* (5) the average reply duration. The type (`A`=ACK `D`=DATA) is appended here.
* (6) the name of the command if found.



# Building `visca-dump`

All you need to build `visca-dump` is a ANSI-C compiler like `gcc` and the
installed library [ezV24](https://github.com/joede/libezV24).

The easiest way to compile `visca-dump` is the following call:

````
gcc -g -Wall -lezV24 -o visca-dump visca-dump.c
````

The second way is the usage of CMake. To make CMake recognize an installed
`libezV24`, a generated `ppkg-config` file is needed. This is part of a current
development branch of [ezV24](https://github.com/joede/libezV24). So, until the
next release of [ezV24](https://github.com/joede/libezV24), the direct call to
`gcc` is ways easier. ;-)

````
git clone https://github.com/joede/visca-dump.git
cd visca-dump
mkdir build && cd build
cmake ..
make
````


# License

Copyright (c) 2005-2018 Joerg Desch <github@jdesch.de>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
