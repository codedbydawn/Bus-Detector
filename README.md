# Bus-Detector
Detects if LTC bus is near your LTC bus stop and sounds alarm 

# Before Compiling to ESP32
- Code deals with buzzer, LED, and OLED display
- Fill out stop_id in make_tripset.py
- Compile make_tripset.py which uses stop_times.txt to create tripset.h (Used in bus_alert.ino).
`python make_tripset.py`
- Fill out desired variables in bus_alert.ino and then compile with tripset.h to your ESP32. 
  
