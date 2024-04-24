# com304-project
much stolen from: https://github.com/hailanzs/comm-proj-radar

some major changes:
- moved `streaming/mmwave` to `mmwave`
- deleted `streaming/radar.py`
- deleted `capture_data.ipynb`
    - `RtttNetClientAPI` is only available in MatLAB and does not work in Python
- deleted `adc\ \(hshanbha@illinois.edu\).py`
- deleted `adc_modified.py`

<!-- # Andrew: HOW IS 'radar' A PYTHON SCRIPT???
#   RtttNetClientAPI is a MATLAB thing and there does not exist a Python package that provides it
#   if you look back at the graveyard you would notice that realtime_streaming.py used radar before (now we just like mmWave studio handle it)
# it would be fine with this but the way that the radar startup failed is really sneaky -->
