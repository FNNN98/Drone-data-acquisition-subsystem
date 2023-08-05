https://github.com/FNNN98/Drone-data-acquisition-subsystem/assets/49247414/058934ec-9744-4e3f-b067-2b5e9b1a7397
## Description 
This project is powered by a STM32F767ZI and a X-NUCLEO-IKS01A2. 

It consists in a FREERTOS based data acquisition subsystem as part of a flight control software for a rotary-wing drone.

By using the HAL provided by ST, the task taski2cR acquire the following RawData from the sensors on the IKS01A2 and save it on the sensorData struct:

- LPS22HB Barometer
  Sensor polled with ODR set to 50 Hz
  
- LSM6DSL Accelerometer
  Sensor polled with ODR set to 833 Hz,
  FS = ±2 with 0.61 mg/LSB sensitivity
 
- LSM6DSL Gyroscope
  Sensor polled with ODR set to 833 Hz,
  FS = ±2000 with 70 mdps/LSB sensitivity

    
- LSM303AGR Accelerometer
  Sensor polled with ODR set to 200 Hz,
  FS = ±2 g with 3.9 mg/LSB sensitivity)
  
- LSM303AGR Magnetometer
  Sensor polled with ODR set to 100 Hz,
  1.5 mgauss/LSB sensitivity)


*Accelerometer RawData from the LSM6DSL and the LSM303AGR is then averaged and saved on a data struct. 


A second task startPrintUart, operating at 0.5hz, uses a custom _write() function to transmit real-time data over UART which can be read from the console.

## Scheduling
The 5 tasks tControlMotor, tAttitude, tAltitude, startPrintUart and taski2cR have been scheduled according to Rate Monotonic Scheduling.

Moreover, the system appears to be compliant with CPU utilization estimations (0.3844 < 0.7568) and the Hyperbolic Bound for RM results in 1.4046 which is below the 2.0000 threshold. It's fair to assume that the system is fully compliant with the scheduling implementation.

Given the inability to provide thread-safe I2C communications between the Nucleo board and the shield when using the HAL provided by ST, I made the architectural decisions to use a single task running at 500hz to acquire data from the 4 sensors with each iteration.
Considering that both taskI2cR and startPrintUart appear to write and read on the sensorData struct it was necessary to prevent priority inversion. For this reason I decided to use the semaphores provided by FREERTOS to introduce a simple but functional Priority Inheritance Protocol (PIP) implementation.

![IMG_20230527_142431 - Copia](https://github.com/FNNN98/Drone-data-acquisition-subsystem/assets/49247414/10fee20d-5f19-4544-8f0e-ff939ff2479a)



