#!/usr/bin/env python3

import cv2
from picamera2 import Picamera2
import numpy as np
import libcamera
import time
import os
import fnmatch

class Calibration:
    markers_required = 0
    markers_total = 0
    image_folders = {}
    
    left_calibration = None
    right_calibration = None
    stereo_calibration = None
        
    cameras = {"left":1, "right":0}
    
    rectification_alpha = 0.95
    
    
    def __init__(self):
        self.init_aruco_board()
        
        self.marker_colour = (0, 0, 255)  # GBR so red 

        self.image_folders = {"left":"images/left", "right":"images/right", "stereo":"images/stereo"}
        self.images = {"left":[], "right":[], "stereo":[]}
        # self.create_image_folders()
        
        
    def create_image_folders(self):
        for folder in self.image_folders.values():
            if not os.path.exists(folder):
                os.makedirs(folder)
                print(f"Created folder: {folder}")
            else:
                for file in os.listdir(folder):
                    file_path = os.path.join(folder, file)
                    if os.path.isfile(file_path):
                        os.remove(file_path)
                    print(f"Deleted existing files from {file_path}")


    def init_aruco_board(self):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.marker_dimensions = (8, 5)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.board = cv2.aruco.CharucoBoard(self.marker_dimensions, 0.05, 0.037, self.dictionary)

        self.markers_total = self.marker_dimensions[0] * self.marker_dimensions[1] // 2 
        self.markers_required = self.markers_total * 0.9
        print(f"markers total: {self.markers_total}, markers required: {self.markers_required}")    


    def show_markers(self, frame, corners, ids, scale):
        if corners is not None and len(corners) > self.markers_required:
            # Draw the detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=self.marker_colour)
            
        # Display the frame
        frame = cv2.resize(frame, (frame.shape[1] // scale, frame.shape[0] // scale))
        return frame
    
    
    def detect_markers(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Detect Aruco markers in the image
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        return corners, ids, rejectedImgPoints
    
    
    def P_controller(Kp: float = 0.05, setpoint: float = 0, measurement: float = 0, output_limits=(-10000, 10000)):
        e = setpoint - measurement
        P = Kp * e

        output_value = P

        # output and limit if output_limits set
        lower, upper = output_limits
        if (upper is not None) and (output_value > upper):
            return upper
        elif (lower is not None) and (output_value < lower):
            return lower
        return output_value


    def capture_metadata(self, left_cam, right_cam, debug=False):
        metadata_picam2a = left_cam.capture_metadata()
        metadata_picam2b = right_cam.capture_metadata()

        timestamp_picam2a = metadata_picam2a["SensorTimestamp"] / 1000  #  convert ns to µs because all other values are in µs
        timestamp_picam2b = metadata_picam2b["SensorTimestamp"] / 1000  #  convert ns to µs because all other values are in µs
        timestamp_delta = timestamp_picam2b - timestamp_picam2a

        controller_output_frameduration_delta = int(P_controller(0.05, 0, timestamp_delta, (-10000, 10000)))
        control_out_frameduration = int(metadata_picam2a["FrameDuration"] + controller_output_frameduration_delta)  # sync to a, so use that for ref

        if debug:
            print("Cam A: SensorTimestamp: ", timestamp_picam2a, " FrameDuration: ", metadata_picam2a["FrameDuration"])
            print("Cam B: SensorTimestamp: ", timestamp_picam2b, " FrameDuration: ", metadata_picam2b["FrameDuration"])
            print("SensorTimestampDelta: ", round(timestamp_delta / 1000, 1), "ms")
            print("FrameDurationDelta: ", controller_output_frameduration_delta, "new FrameDurationLimit: ", control_out_frameduration)

        with right_cam.controls as ctrl:
            # set new FrameDurationLimits based on P_controller output.
            ctrl.FrameDurationLimits = (control_out_frameduration, control_out_frameduration)
        return timestamp_picam2a, timestamp_picam2b, round(timestamp_delta / 1000, 1)

    
    def capture_images(self, camera=0, name="left"):
        # Initialize the Picamera2
        picam = Picamera2(camera)

        exposure_time = 250000
        # exposure_time = 7500
        analogue_gain = 1.0

        cam_config = picam.create_video_configuration(
            main={
                    "size": (1920, 1080)
                },
            controls={
                    "ExposureTime": exposure_time, 
                    "AnalogueGain": analogue_gain,
                    "AwbEnable": 0,
                    "AeEnable": 0
                    }
            )

        cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        picam.configure(cam_config)
        
        picam.start()
        print(f"Camera {name}/{camera} started")

        last_capture_time = time.time()
        capture_interval = 2 # time in seconds between captures
        capture_count = 0
            
        while (True):
            frame = picam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            og_frame = frame.copy()
                                        
            corners, ids, _ = self.detect_markers(frame)
            if(corners is not None and len(corners) > 0):
                print(f"Corners: {len(corners)}, IDs: {len(ids)}")
            
                if ids is not None and len(ids) >= self.markers_required:
                    if(time.time() - last_capture_time > capture_interval):
                        last_capture_time = time.time()
                        filename = f"images/{name}/captured_left_frame_{int(last_capture_time)}.png"
                        cv2.imwrite(filename, og_frame)
                        capture_count += 1                                            
                
                og_frame = self.show_markers(og_frame, corners, ids, scale=1)    

            cv2.putText(og_frame, f"Captured: {capture_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)                
            cv2.imshow(f'Camera {name}', og_frame)
                   

            keycode = cv2.waitKeyEx(1)
                
            # Break the loop on 'q' key press
            if keycode == 113:
                break
            
            margin_adjustment = 1
            if keycode == "65364":
                left_margin += margin_adjustment
                print(f"Left margin: {left_margin}")
                
            if keycode == "65362":
                left_margin -= margin_adjustment
                print(f"Left margin: {left_margin}")
            
            if keycode == "65363":
                right_margin += margin_adjustment
                print(f"Right margin: {right_margin}")
                
            if keycode == "65361":
                right_margin -= margin_adjustment
                print(f"Right margin: {right_margin}")
                
            if keycode == "44":
                rectification_alpha -= 0.001
                print(f"Rectification alpha: {rectification_alpha}")
            
            if keycode == "46":
                rectification_alpha += 0.001
                print(f"Rectification alpha: {rectification_alpha}")
            
            if keycode == "32":
                timestamp = int(time.time())
                cv2.imwrite(f"images/saves/{name}_capture_{timestamp}.png", frame)
                print("Images captured")
                
        # Release the Picamera2 and close windows
        picam.stop()
        cv2.destroyAllWindows()
              
    
    def capture_stereo_images(self, left=1, right=0):
        # Initialize the Picamera2
        left_picam = Picamera2(left)
        right_picam = Picamera2(right)

        exposure_time = 250000
        # exposure_time = 7500
        analogue_gain = 1.0

        left_cam_config = left_picam.create_video_configuration(
            main={
                    "size": (1920, 1080)
                },
            controls={
                    "ExposureTime": exposure_time, 
                    "AnalogueGain": analogue_gain,
                    "AwbEnable": 0,
                    "AeEnable": 0
                    }
            )

        left_cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        left_picam.configure(left_cam_config)
        
        right_cam_config = right_picam.create_video_configuration(
            main={
                    "size": (1920, 1080)
                },
            controls={
                    "ExposureTime": exposure_time, 
                    "AnalogueGain": analogue_gain,
                    "AwbEnable": 0,
                    "AeEnable": 0
                    }
            )

        right_cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        right_picam.configure(right_cam_config)
        
        left_picam.start()
        right_picam.start()
        print(f"Cameras started")

        last_capture_time = time.time()
        capture_interval = 2 # time in seconds between captures
        capture_count = 0
            
        # TODO: Add frame sync for stereo capture
        while (True):
            left_frame = left_picam.capture_array()
            left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)
            left_og_frame = left_frame.copy()
            
            right_frame = right_picam.capture_array()
            right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB)
            right_og_frame = right_frame.copy()
                                                    
            left_corners, left_ids, _ = self.detect_markers(left_frame)
            right_corners, right_ids, _ = self.detect_markers(right_frame)
            
            if(left_corners is not None and len(left_corners) > 0) and (right_corners is not None and len(right_corners) > 0):
                print(f"Left corners: {len(left_corners)}, IDs: {len(left_ids)}, Right corners: {len(right_corners)}, IDs: {len(right_ids)}")
            
                if (left_ids is not None and len(left_ids) >= self.markers_required) and (right_ids is not None and len(right_ids) >= self.markers_required):
                    if(time.time() - last_capture_time > capture_interval):
                        last_capture_time = time.time()
                        stereo_folder = self.image_folders["stereo"]
                        filename = f"{stereo_folder}/captured_left_frame_{int(last_capture_time)}.png"
                        cv2.imwrite(filename, left_og_frame)
                        
                        filename = f"{stereo_folder}/captured_right_frame_{int(last_capture_time)}.png"
                        cv2.imwrite(filename, right_og_frame)
                        capture_count += 1                                            
            
                left_og_frame = self.show_markers(left_og_frame, left_corners, left_ids, 1)    
                right_og_frame = self.show_markers(right_og_frame, right_corners, right_ids, 1)
            
            combined_frame = cv2.hconcat([left_og_frame, right_og_frame])                
            combined_frame = cv2.resize(combined_frame, (combined_frame.shape[1] // 2, combined_frame.shape[0] // 2))
            cv2.putText(combined_frame, f"Captured: {capture_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(f'Stereo Camera ', combined_frame)
                   
            keycode = cv2.waitKeyEx(1)
                
            # Break the loop on 'q' key press
            if keycode == 113:
                break
            
            margin_adjustment = 1
            if keycode == "65364":
                left_margin += margin_adjustment
                print(f"Left margin: {left_margin}")
                
            if keycode == "65362":
                left_margin -= margin_adjustment
                print(f"Left margin: {left_margin}")
            
            if keycode == "65363":
                right_margin += margin_adjustment
                print(f"Right margin: {right_margin}")
                
            if keycode == "65361":
                right_margin -= margin_adjustment
                print(f"Right margin: {right_margin}")
                
            if keycode == "44":
                rectification_alpha -= 0.001
                print(f"Rectification alpha: {rectification_alpha}")
            
            if keycode == "46":
                rectification_alpha += 0.001
                print(f"Rectification alpha: {rectification_alpha}")
            
            if keycode == "32":
                timestamp = int(time.time())
                cv2.imwrite(f"images/saves/{name}_capture_{timestamp}.png", frame)
                print("Images captured")
                
        # Release the Picamera2 and close windows
        left_picam.stop()
        right_picam.stop()
        cv2.destroyAllWindows()
       
       
    def calibrate(self, camera_name):
        folder = self.image_folders[camera_name]
        print(f"Opening folder: {folder}")
        images = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".png")]
        images.sort()  # Ensure files are in order
        print(f"{len(images)} images found in {folder}")
        
        all_charuco_corners = []
        all_charuco_ids = []
        
        ignored_images = 0
        
        for image_file in images:
            image = cv2.imread(image_file)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            marker_corners, marker_ids, _ = self.detector.detectMarkers(image)
            print(f"Found {len(marker_corners)} corners and {len(marker_ids)} IDs in {image_file}")
                
            if len(marker_corners) >= self.markers_required:
                if(len(marker_corners) is not len(marker_ids)):
                    print(f"Error: Marker corners and IDs are not the same length")
                    continue
                retval, charuco_corners, charuco_ids =  cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, self.board)
                if retval > 0:
                    all_charuco_corners.append(charuco_corners)
                    all_charuco_ids.append(charuco_ids)
                else:
                    # cv2.imshow("Failed Image", show_markers(image, marker_corners, marker_ids, 0, False))
                    # cv2.waitKey(0)
                    print(f"No corners interpolated in {image_file}, detected {len(marker_corners)}, ret: {retval}")
                    ignored_images += 1
                    
        print(f"Ignored {ignored_images} images of {len(images)}")
        print(f"Calibrating using {len(all_charuco_corners)} corners")
                        
        retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, self.board, image.shape[:2], None, None)
        print(f"Calibration complete for {camera_name}, error: {retval}")
        
        print(f"camera matrix: {camera_matrix}")
        print(f"dist coeffs: {dist_coeffs}")
        
        np.savez(camera_name, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs, rvecs=rvecs, tvecs=tvecs)
        
        return (camera_matrix, dist_coeffs, rvecs, tvecs)
 
 
    def stereo_calibrate(self):
        stereo_image_folder = self.image_folders["stereo"]
        left_images = []
        right_images = []
        
        images = [os.path.join(stereo_image_folder, f) for f in os.listdir(stereo_image_folder) if f.endswith(".png")]
        for file in images:
            if(fnmatch.fnmatch(file, '*left*')):
                left_images.append(file)
            else:
                right_images.append(file)
                
        left_images.sort()  # Ensure files are in order
        right_images.sort()  # Ensure files are in order
        
        objpoints = []  # 3d point in real world space
        imgpoints_left = []  # 2d points in image plane for left camera
        imgpoints_right = []  # 2d points in image plane for right camera

        imgids_left = []
        imgids_right = []
        
        allCorners = {'left': [], 'right': []}
        allIds = {'left': [], 'right': []}

        print(f"Left images: {len(left_images)}, Right images: {len(right_images)}")

        for left_image_file, right_image_file in zip(left_images, right_images):
            left_image = cv2.imread(left_image_file)
            right_image = cv2.imread(right_image_file)
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            # Find the Charuco corners
            leftCorners, leftIds, _ = self.detector.detectMarkers(left_gray)
            rightCorners, rightIds, _ = self.detector.detectMarkers(right_gray)
            print(f"Images: {left_image_file}, {right_image_file}")
            print(f"left corners: {len(allCorners['left'])}, left ids: {len(allIds['left'])}, right corners: {len(allCorners['right'])}, right ids: {len(allIds['right'])}")

            if leftIds is not None:
                retval_left, charuco_corners_left, charuco_ids_left = cv2.aruco.interpolateCornersCharuco(leftCorners, leftIds, left_gray, self.board)
                if(retval_left > 0):
                    allCorners['left'].append(charuco_corners_left)
                    allIds['left'].append(charuco_ids_left)
                
            if rightIds is not None:    
                retval_right, charuco_corners_right, charuco_ids_right = cv2.aruco.interpolateCornersCharuco(rightCorners, rightIds, right_gray, self.board)
                if(retval_right > 0):
                    allCorners['right'].append(charuco_corners_right)
                    allIds['right'].append(charuco_ids_right)

        # Match points and perform calibration
        matched_object_points = []
        matched_corners_left = []
        matched_corners_right = []

        for i in range(min(len(allCorners['left']), len(allCorners['right']))):
            # Ensure matching ids in both left and right images
            common_ids = np.intersect1d(allIds['left'][i], allIds['right'][i])
            if len(common_ids) > 0:
                indices_left = np.isin(allIds['left'][i], common_ids).flatten()
                indices_right = np.isin(allIds['right'][i], common_ids).flatten()

                matched_object_points.append(self.board.getChessboardCorners()[common_ids, :])
                matched_corners_left.append(allCorners['left'][i][indices_left])
                matched_corners_right.append(allCorners['right'][i][indices_right])


        # Now use matched corners to perform stereo calibration
        if matched_corners_left and matched_corners_right:
            ret, camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, R, T, E, F = cv2.stereoCalibrate(
                objectPoints=matched_object_points,
                imagePoints1=matched_corners_left,
                imagePoints2=matched_corners_right,
                cameraMatrix1=None,
                distCoeffs1=None,
                cameraMatrix2=None,
                distCoeffs2=None,
                imageSize=left_gray.shape[::-1],
                criteria=(cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
                flags=0
            )
            print("Stereo Calibration Reprojection Error:\n", ret)
            print("camera matrix left:\n", camera_matrix_left)
            print("camera matrix right:\n", camera_matrix_right)
            print("Distortion coeffecient left:\n", dist_coeffs_left)
            print("Distortion coeffecient right:\n", dist_coeffs_right)
            print("\nStereo Calibration results:")
            print("Rotation Matrix:\n", R)
            print("Translation Vector:\n", T)
            print("Essential Matrix:\n", E)
            print("Fundamental Matrix:\n", F)

        np.savez("stereo_calibration", camera_matrix_left=camera_matrix_left, dist_coeffs_left=dist_coeffs_left, camera_matrix_right=camera_matrix_right, dist_coeffs_right=dist_coeffs_right, R=R, T=T, E=E, F=F)
        print("Stereo Calibration complete")


    def load_calibrations(self):
        if(os.path.exists('left.npz')):
            self.left_calibration = np.load('left.npz')
        
        if(os.path.exists('right.npz')):
            self.right_calibration = np.load('right.npz')
        
        if(os.path.exists('stereo_calibration.npz')):
            self.stereo_calibration = np.load('stereo_calibration.npz')

    def rectify_images(self, left_image, right_image):        
        left_camera_matrix = self.stereo_calibration['camera_matrix_left']
        left_dist_coeffs = self.stereo_calibration['dist_coeffs_left']
        right_camera_matrix = self.stereo_calibration['camera_matrix_right']
        right_dist_coeffs = self.stereo_calibration['dist_coeffs_right']
        rotation = self.stereo_calibration['R']
        translation = self.stereo_calibration['T']

        image_size = left_image.shape[:2][::-1]

        # Stereo rectification
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            left_camera_matrix, left_dist_coeffs,
            right_camera_matrix, right_dist_coeffs,
            image_size, rotation, translation, alpha=self.rectification_alpha
        )

        # Compute the undistortion and rectification maps
        left_map1, left_map2 = cv2.initUndistortRectifyMap(
            left_camera_matrix, left_dist_coeffs, R1, P1, image_size, cv2.CV_16SC2
        )
        right_map1, right_map2 = cv2.initUndistortRectifyMap(
            right_camera_matrix, right_dist_coeffs, R2, P2, image_size, cv2.CV_16SC2
        )

        # Apply the rectification maps to the images
        left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

        return left_rectified, right_rectified


    def preview_stereo(self, rectify=True):
        # Initialize the Picamera2
        left_picam = Picamera2(self.cameras["left"])
        right_picam = Picamera2(self.cameras["right"])

        exposure_time = 250000
        # exposure_time = 7500
        analogue_gain = 1.0

        left_cam_config = left_picam.create_video_configuration(
            main={
                    "size": (1920, 1080)
                },
            controls={
                    "ExposureTime": exposure_time, 
                    "AnalogueGain": analogue_gain,
                    "AwbEnable": 0,
                    "AeEnable": 0
                    }
            )

        left_cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        left_picam.configure(left_cam_config)
        
        right_cam_config = right_picam.create_video_configuration(
            main={
                    "size": (1920, 1080)
                },
            controls={
                    "ExposureTime": exposure_time, 
                    "AnalogueGain": analogue_gain,
                    "AwbEnable": 0,
                    "AeEnable": 0
                    }
            )

        right_cam_config["transform"] = libcamera.Transform(hflip=1, vflip=1)
        right_picam.configure(right_cam_config)
        
        left_picam.start()
        right_picam.start()
        print(f"Cameras started")
        
        left_margin = 965
        right_margin = 931
                
        while (True):
            left_frame = left_picam.capture_array()
            right_frame = right_picam.capture_array()
            
            left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)
            right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB)
            
            left_og_frame = left_frame.copy()
            right_og_frame = right_frame.copy()
            
            
            if(rectify == True):
                left_rectified, right_rectified = self.rectify_images(left_frame, right_frame)
                height, width = left_rectified.shape[:2]
        
                cv2.line(left_rectified,(width - left_margin - 2,0),(width - left_margin - 2, height),(255,0,0),3)
                                
                # combined_frame = cv2.hconcat([left_rectified, right_rectified])
                combined_frame = cv2.hconcat([left_rectified[0:height, 0:width-left_margin], right_rectified[0:height, right_margin:width]])
                
                combined_frame = cv2.resize(combined_frame, (combined_frame.shape[1] // 1, combined_frame.shape[0] // 1))
                cv2.imshow('Rectified Images', combined_frame)
            
            else:                
                # left_undistorted = undistort_image(frame0, left_calibration["camera_matrix"], left_calibration["dist_coeffs"], 0)
                # 720, 1280
                # cv2.line(left_undistorted,(margin,0),(margin, 720),(255,0,0),2)
                # right_undistorted = undistort_image(frame1, right_calibration["camera_matrix"], right_calibration["dist_coeffs"], 1)
                # cv2.line(right_undistorted,(1280 - margin, 0),(1280 - margin, 720),(255,0,0),2)
                # combined_frame = cv2.hconcat([left_undistorted[0:720, 0:left_margin], right_undistorted[0:720, 1280 - right_margin:1280]])
                # combined_frame = cv2.hconcat([frame0[0:720, 0:left_margin], frame1[0:720, 1280 - right_margin:1280]])
                combined_frame = cv2.hconcat([frame0, frame1])
                
                # combined_frame = stitcher.stitch([left_undistorted, right_undistorted])
                cv2.imshow('Undistorted Images', combined_frame)
            
            keycode = cv2.waitKeyEx(1)
            
            # Break the loop on 'q' key press
            if keycode == 113:
                break
            
            if keycode == 44:
                self.rectification_alpha -= 0.001
                print(f"Rectification alpha: {self.rectification_alpha}")
            
            if keycode == 46:
                self.rectification_alpha += 0.001
                print(f"Rectification alpha: {self.rectification_alpha}")
                
            margin_adjustment = 1
            if keycode == 65364:
                left_margin += margin_adjustment
                print(f"Left margin: {left_margin}")
                
            if keycode == 65362:
                left_margin -= margin_adjustment
                print(f"Left margin: {left_margin}")
            
            if keycode == 65363:
                right_margin += margin_adjustment
                print(f"Right margin: {right_margin}")
                
            if keycode == 65361:
                right_margin -= margin_adjustment
                print(f"Right margin: {right_margin}")
            

if __name__ == "__main__":
    calibrate = Calibration()
    calibrate.load_calibrations()
    calibrate.preview_stereo()
    # calibrate.capture_images(1, "left")
    # calibrate.capture_images(0, "right")
    # calibrate.capture_stereo_images()
    # calibrate.calibrate("left")
    # calibrate.calibrate("right")
    # calibrate.stereo_calibrate()