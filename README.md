This project is powered by a STM32F767ZI and a X-NUCLEO-IKS01A2. 

It consists in a FREERTOS based data acquisition subsystem with priority inheritance protocol (PIP) support for the 2 main tasks (taskI2cR and startPrintUART).

By using the HAL provided by ST, the task taski2cR acquire the following RawData from the sensors provided by the IKS01A2:
- LPS22HB Barometer
  
- LSM6DSL Accelerometer (FS = ±2 with 0.61 mg/LSB sensitivity) and Gyroscope (FS = ±2000 with 70 mdps/LSB sensitivity)
  
- LSM303AGR Accelerometer and Magnetometer
  

https://github.com/FNNN98/Drone-data-acquisition-subsystem/assets/49247414/058934ec-9744-4e3f-b067-2b5e9b1a7397

