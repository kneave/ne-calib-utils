import cv2
from picamera2 import Picamera2

# Initialize the Picamera2
picam = Picamera2()

# exposure_time = 250000
# # exposure_time = 7500
# analogue_gain = 1.0

# cam_config = picam.create_video_configuration(
#     main={
#             "size": (1920, 1080)
#         },
#     controls={
#             "ExposureTime": exposure_time, 
#             "AnalogueGain": analogue_gain,
#             "AwbEnable": 0,
#             "AeEnable": 0
#             }
#     )

# cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
# picam.configure(cam_config)

picam.start()
print(f"Camera started")

print("Capturing frame")
frame = picam.capture_array()
print("converting frame")
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
cv2.imshow('Camera Preview.png', frame)
cv2.waitKey(0)    
        
            
# Release the Picamera2 and close windows
picam.stop()

