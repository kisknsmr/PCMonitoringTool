import clr
clr.AddReference("LibreHardwareMonitorLib")

from LibreHardwareMonitor import Hardware

computer = Hardware.Computer()
computer.IsCpuEnabled = True
computer.Open()

for hw in computer.Hardware:
    hw.Update()
    if hw.HardwareType == Hardware.HardwareType.Cpu:
        for sensor in hw.Sensors:
            if sensor.SensorType == Hardware.SensorType.Temperature:
                print(f"{sensor.Name}: {sensor.Value} °C")
