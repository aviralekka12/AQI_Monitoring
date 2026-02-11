# AQI System Sensor Specifications & Datasheet Summaries

## 1. MPM10-BS (Laser Particulate Matter Sensor)
* **Target:** PM1.0, PM2.5, PM10
* **Principle:** Laser Scattering
* **Interface:** UART (Serial)
    * [cite_start]**Baud Rate:** 9600 bps [cite: 1965]
    * [cite_start]**Data Bits:** 8, **Stop Bits:** 1, **Parity:** None [cite: 1965]
    * [cite_start]**Logic Level:** 3.3V (Requires Logic Level Converter for 5V Arduino) [cite: 1965]
* **Operating Voltage:** 5.0V (Typ), 4.5V - 5.5V
* **Output Mode:**
    * **Default:** Active (Automatic). [cite_start]Sends 32-byte frame every 1 second[cite: 1967].
* [cite_start]**Data Frame Structure (32 Bytes):** [cite: 1970]
    * [cite_start]**Header:** Byte 0 (`0x42`), Byte 1 (`0x4D`) [cite: 1971]
    * **Atmospheric Environment Values (Use these for AQI):**
        * [cite_start]**PM1.0:** Byte 10 (High) & Byte 11 (Low) [cite: 1971]
        * [cite_start]**PM2.5:** Byte 12 (High) & Byte 13 (Low) [cite: 1971]
        * [cite_start]**PM10:** Byte 14 (High) & Byte 15 (Low) [cite: 1971]
    * **Formula:** `Concentration (ug/m3) = (HighByte * 256) + LowByte`
    * **Checksum:** Bytes 30 (High) & 31 (Low). [cite_start]Sum of Bytes 0-29[cite: 1978].

---

## 2. DHT22 (Temperature & Humidity)
* **Target:** Ambient Temperature, Relative Humidity
* **Interface:** Single-Wire Digital (Proprietary Protocol)
* **Operating Voltage:** 3.3V - 6.0V
* **Sampling Rate:** 0.5 Hz (Read once every 2 seconds maximum)
* **Range & Accuracy:**
    * **Humidity:** 0-100% RH (±2% accuracy)
    * **Temperature:** -40 to 80°C (±0.5°C accuracy)

---

## 3. MQ-7 (Carbon Monoxide)
* **Target:** Carbon Monoxide (CO)
* **Type:** Semiconductor (SnO2)
* **Operating Logic:** **Cycle Heating Required**
    * **Phase 1 (Cleaning):** 5.0V Heater Voltage for **60 seconds**.
    * **Phase 2 (Sensing):** 1.5V (±0.1V) Heater Voltage for **90 seconds**.
    * **Reading Time:** Take analog reading at the end of Phase 2 (90s mark).
* **Circuit Constants:**
    * **Loop Voltage ($V_c$):** 5.0V
    * **Load Resistor ($R_L$):** Adjustable (rec. 10kΩ - 47kΩ)
    * **Actual $R_L$ Used:** 0.694 kΩ (measured)
* **Sensitivity:** Resistance ($R_s$) decreases as CO concentration increases.

---

## 4. MQ-135 (CO2 - Primary Use)
* **Target:** CO2 (Primary), also sensitive to Ammonia, Sulfide, Benzene, Smoke
* **Type:** Semiconductor
* **Operating Voltage:** 5.0V (Constant)
* **Circuit Constants:**
    * **Heater Voltage ($V_H$):** 5.0V ±0.1V
    * **Heater Resistance:** 33Ω ±5%
    * **Load Resistor ($R_L$):** Adjustable (20kΩ standard)
    * **Actual $R_L$ Used:** 0.793 kΩ (measured)
* **Calibration:**
    * **$R_0$:** Sensor resistance in clean air (approx. 400ppm CO2 equiv).
* **Sensitivity:** $R_s$ decreases as gas concentration increases.

---

## 5. MQ-137 (Ammonia)
* **Target:** Ammonia ($NH_3$)
* **Operating Voltage:** 5.0V (Constant)
* **Circuit Constants:**
    * **Heater Voltage ($V_H$):** 5.0V ±0.1V
    * **Heater Resistance:** 31Ω ±3Ω
    * **Load Resistor ($R_L$):** Adjustable
    * **Actual $R_L$ Used:** 0.783 kΩ (measured)
* **Detection Range:** 5 - 500 ppm $NH_3$
* **Sensitivity:** $R_s$ decreases significantly as $NH_3$ increases.

---

## 6. MQ-131 (Ozone)
* **Target:** Ozone ($O_3$)
* **Operating Voltage:** 5.0V (Constant)
* **Circuit Constants:**
    * **Heater Voltage ($V_H$):** 5.0V ±0.1V
    * **Heater Resistance:** 31Ω ± 3Ω (or 34Ω ± 3Ω depending on variant)
    * **Actual $R_L$ Used:** 0.698 kΩ (measured)
* **Sensitivity:** High sensitivity to $O_3$.
* **Note:** Unlike most MQ sensors, $R_s$ typically **increases** as $O_3$ concentration increases (Oxidizing gas on n-type semiconductor).

---

## 7. MQ-2 (SO2 - Primary Use)
* **Target:** SO2 (Primary Use), also sensitive to LPG, Propane, Hydrogen, Methane, Smoke
* **Operating Voltage:** 5.0V (Constant)
* **Circuit Constants:**
    * **Heater Voltage ($V_H$):** 5.0V ±0.1V
    * **Load Resistor ($R_L$):** Adjustable
    * **Actual $R_L$ Used:** 0.794 kΩ (measured)
* **Detection Range:** 300 - 10000 ppm (for combustible gases)
* **Note:** MQ-2 has limited SO2 sensitivity; readings are approximate. Empirical formula used in code.
* **Sensitivity:** $R_s$ decreases as gas concentration increases.

---

## 8. MEMS GM-102B (Nitrogen Dioxide)
* **Target:** $NO_2$
* **Type:** MEMS Semiconductor
* **Operating Conditions:**
    * **Heater Voltage:** **2.0V ± 0.1V** (Do NOT use 5V)
    * **Circuit Voltage:** 5.0V Max
* **Characteristics:** High sensitivity to $NO_2$, small size, low power.
* **Response:** $R_s$ increases as $NO_2$ concentration increases.

---

## 9. MEMS GM-502B (VOC)
* **Target:** VOCs (Volatile Organic Compounds), Formaldehyde, Toluene
* **Type:** MEMS Semiconductor
* **Operating Conditions:**
    * **Heater Voltage:** **2.5V ± 0.1V** (Do NOT use 5V)
    * **Circuit Voltage:** 5.0V Max
* **Characteristics:** High sensitivity to organic vapors.
* **Response:** $R_s$ decreases as gas concentration increases.