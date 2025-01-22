Neave Engineering Camera Calibration Utilities
==

This is primarily for calibrating cameras on Raspberry Pi based devices, it should work for others for pure calibration but the capture parts will assume you've a Pi camera attached.

If your device will be single purpose and you don't need to worry about dependency hell, have a look at Jeff Geerling's [post](https://www.jeffgeerling.com/blog/2023/how-solve-error-externally-managed-environment-when-installing-pip3) to disable the library safety mechanism that's enabled by default.

At which point just do this:

Create a venv:
```
python3 -m venv env
source env/bin/activate
```

And follow [this guide](https://forums.raspberrypi.com/viewtopic.php?t=361758) to using PiCamera2 in a venv. 


