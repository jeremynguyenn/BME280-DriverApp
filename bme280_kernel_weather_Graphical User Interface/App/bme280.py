```python
#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____
#   / _ \/ _ \(_) __/__  __ __
#  / , _/ ___/ /\ \/ _ \/ // /
# /_/|_/_/  /_/___/ .__/\_, /
#                /_/   /___/
#
#           bme280.py
#  Read data from a BME280 sensor via sysfs.
#
#  Official datasheet available from :
#  https://www.bosch-sensortec.com/bst/products/all_products/bme280
#
# Author : Nguyen Nhan
# Date   : 31/01/2025
#
# https://www.raspberrypi-spy.co.uk/
#
#--------------------------------------

import time

def readBME280All():
    # Đọc compensated data từ sysfs
    try:
        with open("/sys/class/bme280/temperature", "r") as f:
            temperature = float(f.read()) / 1000.0  # m°C to °C
        with open("/sys/class/bme280/pressure", "r") as f:
            pressure = float(f.read()) / 100.0  # Pa to hPa
        with open("/sys/class/bme280/humidity", "r") as f:
            humidity = float(f.read()) / 1024.0  # 1024*percent to percent
    except IOError as e:
        print(f"Error reading sysfs: {e}")
        return 0.0, 0.0, 0.0

    return temperature, pressure, humidity

def readPeriod():
    try:
        with open("/sys/class/bme280/period_ms", "r") as f:
            period = int(f.read())
    except IOError as e:
        print(f"Error reading period_ms: {e}")
        return 2000  # Default period
    return period

def main():
    period = readPeriod()

    while True:
        temperature, pressure, humidity = readBME280All()
        print(f"Temperature: {temperature:.2f} C")
        print(f"Pressure: {pressure:.2f} hPa")
        print(f"Humidity: {humidity:.2f} %")
        print(f"Update Period: {period} ms")

        period = readPeriod()
        time.sleep(period / 1000.0)

if __name__ == "__main__":
    main()
```