## Description 
This project is powered by a STM32F767ZI and a X-NUCLEO-IKS01A2. 

It consists in a FREERTOS based data acquisition subsystem as part of a flight control software for a rotary-wing drone.

By using the HAL provided by ST, the task taski2cR acquire the following RawData from the sensors provided by the IKS01A2:

- LPS22HB Barometer
  Data acquired with ODR set to 50 Hz
  
- LSM6DSL Accelerometer
  Data acquired with ODR set to 833 Hz,
  FS = ±2 with 0.61 mg/LSB sensitivity
 
- LSM6DSL Gyroscope
  Data acquired with ODR set to 833 Hz,
  FS = ±2000 with 70 mdps/LSB sensitivity
    
- LSM303AGR Accelerometer
  Data acquired with ODR set to 200 Hz,
  FS = ±2 g with 3.9 mg/LSB sensitivity)
  
- LSM303AGR Magnetometer
  Data acquired with ODR set to 100 Hz,
  1.5 mgauss/LSB sensitivity)


Accelerometer RawData from the LSM6DSL and the LSM303AGR is averaged and saved on a data struct.

## Scheduling

Given the inability to provide thread-safe I2C communications between the Nucleo board and the shield when using the HAL provided by ST, I made the architectural decisions to use a single task running at 500hz to poll each sensor with each iteration. 

https://github.com/FNNN98/Drone-data-acquisition-subsystem/assets/49247414/058934ec-9744-4e3f-b067-2b5e9b1a7397

