Davis anemometer 6410 wireless sensor using an Arduino pro mini (328p) running at 8 MHz internal clock. 
The 328p has the Optiboot bootloader installed to enable the watchdog, and also allows
the 16 MHz pro mini board to be configured as 8 MHz 3.3V.

The anemometer is connected to a 5.1k pull-up resistor on the wind speed wire, a resistor network to keep 
the wind direction voltage approx. 0-1 VDC (ADC is running on internal 1.1V reference voltage), and
a couple of ceramic capacitors and TVS diodes to reduce static interference.  

A DS18B20 sensor is attached to report temperature.

The communication is using the MySensors library v2.3.2, and an RFM69H radio chip. 
Data is sent in passive mode, as this seems to be more reliable over time.

The unit is powered with a 18650 cell via a low power MCP1702 3.3V regulator, and charged 
with a solar panel. The battery voltage is monitored with a voltage divider. 
