Acoustics
=========

Overview
--------


Hardware
--------

?!


Firmware
--------

The TMS320C5515 eZDSP USB Stick was programmed using Code Composer Studio in the language of C++. After receiving a signal from the hydrophone array, the microcontroller would filter out unwanted frequency ranges using a digital bandpass filter. The specified frequency range is 25-40 kHz.

Once the signal is received by the vehicle’s three hydrophones, the angle of arrival of the emitted wave is calculated using phase difference between the three received signals. The hydrophones are separated such that, the underwater pinger’s transmitting wave cannot travel one whole wavelength in between the hydrophones. The angle of arrival of the wave is then calculated using the phase difference between each of the hydrophones and simple trigonometry. If the hydrophones were separated further than one wavelength then the phase difference could not be used because we would not be able to know which hydrophone received the signal first.

The heading information containing the direction that the vehicle needs to travel in order to reach the pinger is then sent to the central processing computer continuously until the breeching sequence activates.


Wiring
------



Battery
=======

Overview
--------



Hardware
--------



Firmware
--------




Wiring
------



Pneumatics
==========


Overview
--------



Hardware
--------



Firmware
--------




Wiring
------
