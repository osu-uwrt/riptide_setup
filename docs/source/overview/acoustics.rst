Acoustics Processing
====================

The acoustics processing software was programmed using Code Composer Studio in the language of C++. After receiving a signal from the hydrophone array, unwanted frequency ranges are filtered out using a digital bandpass filter. The specified frequency range is 25-40 kHz.

Once the signal is received by Riptide’s three hydrophones, the angle of arrival of the emitted wave is calculated using phase difference between the three received signals. With the hydrophones positioned so that one whole wavelength is larger than their separation, the angle of arrival of the wave is calculated using the phase difference between each of the hydrophones and simple trigonometry. If the hydrophones were separated further than one wavelength then the phase difference could not be used since the system would not be able to determine which hydrophone received the signal first.

The heading information containing the direction that the vehicle needs to travel in order to reach the pinger is then sent to the central processing computer continuously until the breeching sequence activates.
