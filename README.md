# pomelo
A complete low-power gamma ray spectrometer that can be used by itself or integrated into other projects

Details on Hackaday.io on [Pomelo Core](https://hackaday.io/project/194457-pomelo-gamma-spectroscopy-module) and [Pomelo Zest](https://hackaday.io/project/196334-pomelo-hand-held-gamma-ray-spectrometer).

***

This documentation provides a comprehensive guide to all serial communication commands for interacting with the device. All commands must be terminated by a **newline (`\n`)** or **carriage return (`\r`)** character.

***

### Regular Commands

These are single-character commands that perform an action or retrieve information. They do not require a colon or a value. Just send the character followed by a newline or carriage return.

* `h`: **Histogram**. Requests the device to send its current gamma-ray energy spectrum data. This includes counts per energy channel, serial number, threshold, total counts, energy calibration coefficients, temperature, and run time. If coincidence mode is enabled, it also includes coincidence counts and spectrum data. The output is a JSON string.
* `s`: **System Status**. Requests general information about the device's current state, including the serial number, uptime, power status (`running`), and the current operating temperature. The output is a JSON string.
* `c`: **Configuration Dump**. Requests a detailed dump of all current configuration parameters. This includes SiPM voltage settings, DAC settings, energy calibration, threshold, coincidence mode status, and output settings for both UART and USB. The output is a JSON string.
* `x`: **Power On**. Activates the main system power.
* `z`: **Power Off**. Deactivates the main system power.
* `q`: **Print Parameters**. Prints all current parameters, including versions, to the serial port in a formatted string. **Note:** The code indicates this command is for debugging and might be removed in future versions.
* `r`: **Load Parameters**. Instructs the device to reload all saved parameters from its non-volatile memory into active use.
* `g`: **CPM (Counts Per Minute)**. Requests the current counts per minute. This provides a direct measure of detected events over time. The output is a float value.
* `u`: **uSv/h (micro-Sieverts per hour)**. Requests the current radiation dose rate in micro-Sieverts per hour. This is a measure of the biological impact of radiation. The output is a float value.
* `m`: **Measure Dosimetry**. Requests both the CPM and uSv/h values simultaneously. The output is a JSON string.
* `i`: **SiPM Current**. Requests the current draw of the SiPM sensor. This can indicate the operational status of the sensor's high-voltage supply. The output is a JSON string.
* `p`: **Enter Parameter Mode**. This command is a prefix that tells the device to expect a subsequent command in the `[parameter_ID]:[value]` format. It transitions the device into a state where it specifically listens for parameter-setting commands. You must send a parameter command immediately after sending `p`.
* `/`: **Boost SiPM power**. Activates the boost mode for the SiPM high-voltage power supply.
* `*`: **Disable SiPM power boost**. Deactivates the boost mode for the SiPM high-voltage power supply.

***

### Parameter Setting Commands

These commands allow you to set specific device parameters. They follow the format: `[parameter_ID]:[value]`.

| Serial Code | Name | Description | Value Type/Range |
| :--- | :--- | :--- | :--- |
| 0 | sipm\_vMin | Sets the minimum operating voltage for the SiPM sensor. | Float, 0 to 4096 |
| 1 | sipm\_vMax | Sets the maximum operating voltage for the SiPM sensor. | Float, 0 to 4096 |
| 2 | sipm\_v0deg | Sets the SiPM voltage at 0 degrees Celsius. | Float, 0 to 4096 |
| 3 | sipm\_vTempComp | Temperature compensation coefficient for the SiPM voltage. | Float, -5 to 5 |
| 4 | ecal\[0\] | First coefficient for energy calibration (linear). | Float |
| 5 | ecal\[1\] | Second coefficient for energy calibration (quadratic). | Float |
| 6 | ecal\[2\] | Third coefficient for energy calibration (cubic). | Float |
| 7 | uSvph\_constant | Conversion constant for Sieverts per hour. | Float |
| 8 | vDac\[0\] | Offset for the HV Digital-to-Analog Converter (DAC). | Float |
| 9 | vDac\[1\] | Slope for the HV DAC. | Float |
| 10 | iMeas\[0\] | Offset for current measurement. | Float |
| 11 | iMeas\[1\] | Slope for current measurement. | Float |
| 12 | iMeas\[2\] | Resistor value for current measurement. | Float |
| 13 | threshold | ADC threshold for pulse detection. | Float, 1 to 4096 |
| 14 | sys\_outputs | Configures the system outputs (e.g., enable/disable data streaming). | Integer, 0 to 127 |
| 15 | sys\_coincidence | Enables (1) or disables (0) coincidence mode. | Integer, 0 or 1 |
| 16 | sys\_pulseChar | Sets the ASCII character for a fast UART pulse output. | Integer, 128 to 255 |

***

### Special Action Commands

These commands trigger specific functions within the device and require a fixed numerical value as the parameter.

| Serial Code | Action | Required Value | Description |
| :--- | :--- | :--- | :--- |
| 100 | Save parameters | -2024 | Saves all current parameters to non-volatile memory. |
| 200 | Start ADC calibration | -2024 | Initiates the ADC calibration process. |
| 300 | Initialize physics parameters | -2024 | Resets the physics parameters to their default values. |
| 1000 | System reset (reboot) | -2024 | Restarts the device. |
| 2000 | Reset and enter bootloader mode | -2024 | Resets the device and enters the bootloader. |
