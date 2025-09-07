# pomelo
A complete low-power gamma ray spectrometer that can be used by itself or integrated into other projects. Details on Hackaday.io on [Pomelo Core](https://hackaday.io/project/194457-pomelo-gamma-spectroscopy-module) and [Pomelo Zest](https://hackaday.io/project/196334-pomelo-hand-held-gamma-ray-spectrometer).

***

This documentation provides a comprehensive guide to all serial communication commands for interacting with the device.

---

### Regular Commands

These are single-character commands that perform an action or retrieve information. They do **not** need to be followed by a newline (`\n`) or carriage return (`\r`).

* `h`: **Histogram**. Requests the device to send its current gamma-ray energy spectrum data. The output is a JSON string with details like counts per energy channel, temperature, and run time.
* `s`: **System Status**. Requests general information about the device's current state, including uptime, power status, and temperature. The output is a JSON string.
* `c`: **Configuration Dump**. Requests a detailed dump of all current configuration parameters, such as voltage settings, energy calibration, and output settings. The output is a JSON string.
* `x`: **Power On/Clear Spectrum**. Activates the main system power. If the detector is already running, this command can also be used to clear the current spectrum and start a fresh measurement.
* `z`: **Power Off**. Deactivates the main system power.
* `r`: **Load Parameters**. Instructs the device to reload all saved parameters from its non-volatile memory.
* `q`: **Print Parameters**. Prints all current parameters to the serial port in a formatted string. **Note**: This is a debugging command and may be removed in future versions.
* `g`: **CPM (Counts Per Minute)**. Requests the current counts per minute. The output is a float value.
* `u`: **uSv/h (micro-Sieverts per hour)**. Requests the current radiation dose rate. The output is a float value.
* `m`: **Measure Dosimetry**. Requests both the CPM and uSv/h values simultaneously. The output is a JSON string.
* `i`: **SiPM Current**. Requests the current draw of the SiPM sensor. The output is a JSON string.
* `p`: **Enter Parameter Mode**. This is a prefix command that prepares the device to receive a parameter-setting command in the `[parameter_ID]:[value]` format.
* `/`: **Boost SiPM power**. Activates boost mode for the SiPM high-voltage power supply.
* `*`: **Disable SiPM power boost**. Deactivates boost mode for the SiPM high-voltage power supply.

---

### Parameter Setting Commands

These commands allow you to set specific device parameters. They follow the format: `[parameter_ID]:[value]`.

| Serial Code | Name              | Description                                                                                                                              | Value Type/Range      |
| :--- | :--- | :--- | :--- |
| 0           | sipm\_vMin        | Sets the minimum operating voltage for the SiPM sensor.                                                                                  | Float, 0 to 4096      |
| 1           | sipm\_vMax        | Sets the maximum operating voltage for the SiPM sensor.                                                                                  | Float, 0 to 4096      |
| 2           | sipm\_v0deg       | Sets the bias voltage for the SiPM at 0°C. The actual voltage is extrapolated from this value based on temperature and `sipm_vTempComp`. | Float, 0 to 4096      |
| 3           | sipm\_vTempComp   | Temperature compensation coefficient for the SiPM bias voltage.                                                                            | Float, -5 to 5        |
| 4           | ecal\[0\]         | Energy calibration coefficient (offset). Used in the energy calculation: $E = ecal[0] + ch \cdot ecal[1] + ch^2 \cdot ecal[2]$                | Float                 |
| 5           | ecal\[1\]         | Energy calibration coefficient (linear).                                                                                                 | Float                 |
| 6           | ecal\[2\]         | Energy calibration coefficient (quadratic).                                                                                              | Float                 |
| 7           | uSvph\_constant   | Conversion constant for Sieverts per hour.                                                                                               | Float                 |
| 8           | vDac\[0\]         | First coefficient to obtain the DAC value for a requested voltage, as per: $dacValue = vDac[0] + vDac[1] \cdot requestedVolts$           | Float                 |
| 9           | vDac\[1\]         | Second coefficient for the voltage to DAC value conversion.                                                                              | Float                 |
| 10          | iMeas\[0\]        | Offset for current measurement.                                                                                                          | Float                 |
| 11          | iMeas\[1\]        | Slope for current measurement.                                                                                                           | Float                 |
| 12          | iMeas\[2\]        | Resistor value for current measurement.                                                                                                  | Float                 |
| 13          | threshold         | ADC threshold for pulse detection. This is a 12-bit value which corresponds to a channel cut of approximately `threshold / 4` in the output histogram. | Float, 1 to 4096      |
| 14          | sys\_outputs      | Configures the system outputs (e.g., enable/disable data streaming).                                                                     | Integer, 0 to 127     |
| 15          | sys\_coincidence  | Enables (1) or disables (0) coincidence mode.                                                                                            | Integer, 0 or 1       |
| 16          | sys\_pulseChar    | Sets the ASCII character for a fast UART pulse output.                                                                                   | Integer, 128 to 255   |

---

### Special Action Commands

These commands trigger specific functions and require a fixed numerical value as the parameter.

| Serial Code | Action                          | Required Value | Description                                          |
| :--- | :--- | :--- | :--- |
| 100         | Save parameters                 | -2024          | Saves all current parameters to non-volatile memory. |
| 200         | Start ADC calibration           | -2024          | Initiates the ADC calibration process.               |
| 300         | Initialize physics parameters   | -2024          | Resets the physics parameters to their default values. |
| 1000        | System reset (reboot)           | -2024          | Restarts the device.                                 |
| 2000        | Reset and enter bootloader mode | -2024          | Resets the device and enters the bootloader.         |

---

### Parameter Storage and Usage Notes

* **Parameter Storage**: The parameters are stored on different boards. **`ecal[]`** and **`sipm_v0deg`** are stored on the **Physics board** (the detector itself). All other parameters (`vDac[]`, `iMeas[]`, `threshold`, etc.) are stored on the **Core board**. This means if you swap detectors, the energy calibration and bias voltage settings will move with the detector. Recalibration is still recommended after swapping.
* **Unique Identification**: To uniquely identify each detector, use the **serial number** reported by the `h` and `s` commands. The `detString` parameter is not configurable.
* **Optimization**: You can optimize measurements by adjusting the **SiPM bias voltage** and **threshold** to cover the specific energy range of interest. For example, you can increase the SiPM voltage to shift the spectrum to a higher energy, but this may introduce noise at lower energies.
* **Procedure**: When you send a new parameter, it is applied immediately. It's recommended to wait a few seconds (especially for voltage changes) before using the `x` command to clear the spectrum and begin a new measurement. Once you are satisfied with the settings, use the **100:-2024** command to save them to non-volatile memory.
* **Safety Warning**: Be aware that the `sys_power` setting is saved to memory. If `sys_power` is `1` when you save, the detector will turn on immediately upon receiving power. If the device cannot read the `sipm_vMax` and `sipm_v0deg` parameters from the Physics board (e.g., if the board is uninitialized), the bias voltage could become dangerously high. To avoid this, always use the **`z`** command to turn off the detector before saving parameters. This ensures `sys_power` is saved as `0`, giving you a chance to configure the voltage parameters safely after the next power-on.
