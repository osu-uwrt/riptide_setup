External Electronics
====================

Within the secondary modules of Riptide there are several electrical systems in place to provide power to the vehicle and to control several external devices.


Battery Monitoring
------------------

Riptide is powered by two MaxAmps 8000 mAh 18.5 V Dual Core battery packs, allowing for two to three hours of testing depending on intensity.

There are battery monitor boards located in each battery housing. They monitor the temperature and pressure within the housing to ensure the battery conditions are safe and allow efficient operation. This status data is relayed to the Power Distribution board over a RS-232 serial connection.


Pneumatics Controller
---------------------

The Pneumatic control board activates the solenoid valves that direct air to the pneumatic actuators on the AUV. An atmega328 microcontroller receives commands from the computer through an RS-232 to UART interface to control eight separate outputs. Darlington transistors are used to amplify the logic signal to a higher current, 12 volt signal used to drive the solenoid valves.


Acoustics Processing
--------------------

Three hydrophones mounted below the vehicle provide data for determining the pinger’s position. The hydrophones are separated such that the underwater pinger’s transmitting wave cannot travel one whole wavelength in between the hydrophones.

The acoustics processing is currently being completed with a Codec shield connected to an Arduino Uno. The shield has a WM8731S Codec for capturing the hydrophone data, which is then transferred to the main computer for processing.
