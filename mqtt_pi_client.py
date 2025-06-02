# mqtt_pi_client.py

import paho.mqtt.client as mqtt
import time
import json
import grovepi
import os
import atexit
import subprocess
import cv2
import numpy as np
import re
import requests
import base64
import logging
import sys
import RPi.GPIO as GPIO
import threading
import signal
from datetime import datetime, timedelta
from picamera import PiCamera
from picamera.array import PiRGBArray
import firebase_admin
from firebase_admin import credentials, db
import random

# -------------------- LOGGING SETUP -------------------- #
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("smart_gate.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("SmartGate")

RED_LED = 23
BUZZER = 25


# -------------------- CONFIGURATION -------------------- #
# Load configuration from a separate file
CONFIG = {
    "mqtt": {
        "broker": "your broker",
        "port": 8883,
        "client_id": "you_id",
        "username": "your_user",
        "password": ""  # In production, use environment variables
    },
    "firebase": {
        "cred_path": "firebase_key.json",
        "db_url": "your_url",
        "project_id": "your_id"
    },
    "google_vision": {
        "api_key": "place_key_here"  # Use environment variables
    },
    "hardware": {
    "stepper": {
        "in1": 17,
        "in2": 18,
        "in3": 27,
        "in4": 22,
        "steps_full_rotation": 512,
        "delay": 0.002,
        "steps_open": 500,  # Steps to open gate
        "steps_close": 500  # Steps to close gate
    },
    "gate_open_time": 5  # Time in seconds to keep gate open before closing
},
    "camera": {
        "resolution": (640, 480),
        "mjpg_path": "/home/elio/mjpg-streamer/mjpg-streamer-experimental"
    },
    "topics": {
        "command": {
            "open": "gate/command/open",
            "generate_otp": "gate/command/generate_otp",
            "verify_otp": "gate/command/verify_otp",
            "env_status": "gate/command/request_env",
            "report": "gate/command/generate_report",
            "camera": "gate/command/camera",
            "detect_plate": "gate/command/detect_plate"
        },
        "status": {
            "plate": "gate/status/plate",
            "entry": "gate/status/entry",
            "otp": "gate/status/otp",
            "report": "gate/status/report",
            "report_data": "gate/status/report/data",
            "camera": "gate/status/camera"
        },
        "environment": {
            "ldr": "gate/environment/ldr"
        },
        "security": {
            "alerts": "gate/security/alerts"
        },
        "user": {
            "guest_arrival": "gate/user/guest_arrival"
        },
        "system": {
            "status": "gate/system/status"
        }
    },
    "files": {
        "log_file": "entry_log.json",
        "report_file": "weekly_report.json"
    },
    "plate_check_interval": 15,  # seconds between automatic plate checks
    "otp_expiry": 30,  # minutes until OTP expires
    "system_status_interval": 60  # seconds between system status updates
}

# Global shutdown flag
shutdown_requested = False

# -------------------- FIREBASE MANAGER -------------------- #
class FirebaseManager:
    def __init__(self, config):
        self.config = config
        self._initialize()
        
    def _initialize(self):
        """Initialize Firebase connection"""
        try:
            cred = credentials.Certificate(self.config["firebase"]["cred_path"])
            firebase_admin.initialize_app(cred, {
                'databaseURL': self.config["firebase"]["db_url"]
            })
            logger.info("Firebase initialized successfully")
        except Exception as e:
            logger.error(f"Firebase initialization error: {e}")
            
    def get_access_token(self):
        """Get Firebase access token for sending notifications"""
        try:
            from google.oauth2 import service_account
            from google.auth.transport.requests import Request
            
            credentials = service_account.Credentials.from_service_account_file(
                self.config["firebase"]["cred_path"],
                scopes=["https://www.googleapis.com/auth/firebase.messaging"]
            )
            credentials.refresh(Request())
            return credentials.token
        except Exception as e:
            logger.error(f"Error getting Firebase access token: {e}")
            return None
            
    def send_push_notification(self, title, body, topic="all"):
        """Send push notification to specified topic or device"""
        try:
            access_token = self.get_access_token()
            if not access_token:
                logger.error("Failed to get access token for notifications")
                return False
                
            url = f"https://fcm.googleapis.com/v1/projects/{self.config['firebase']['project_id']}/messages:send"
            
            headers = {
                "Authorization": f"Bearer {access_token}",
                "Content-Type": "application/json; UTF-8",
            }
            
            message = {
                "message": {
                    "topic": topic,
                    "notification": {
                        "title": title,
                        "body": body
                    }
                }
            }
            
            response = requests.post(url, headers=headers, data=json.dumps(message))
            
            if response.status_code == 200:
                logger.info("Push notification sent successfully")
                return True
            else:
                logger.error(f"Push notification failed: {response.status_code} - {response.text}")
                return False
        except Exception as e:
            logger.error(f"Error sending push notification: {e}")
            return False
            
    def fetch_allowed_plates(self):
        """Read all users/*/plates from Firebase RTDB and return a set of uppercase plate strings"""
        try:
            ref = db.reference('users')
            snapshot = ref.get()
            plates = []
            if snapshot:
                for user, data in snapshot.items():
                    user_plates = data.get('plates', [])
                    if isinstance(user_plates, list):
                        plates.extend(user_plates)
            # normalize
            allowed = {p.strip().upper() for p in plates if isinstance(p, str)}
            logger.info(f"Allowed plates: {allowed}")
            return allowed
        except Exception as e:
            logger.error(f"Error fetching plates from Firebase: {e}")
            return set()
            
    def upload_report(self, report):
        """Upload report to Firebase"""
        try:
            db.reference("reports/weekly").set(report)
            logger.info("Weekly report uploaded to Firebase")
            return True
        except Exception as e:
            logger.error(f"Error uploading report to Firebase: {e}")
            return False

class StepperMotorManager:
    def __init__(self, config):
        hw_cfg = config["hardware"]
        step_cfg = hw_cfg["stepper"]
        
        self.in1, self.in2, self.in3, self.in4 = (
            step_cfg["in1"],
            step_cfg["in2"],
            step_cfg["in3"],
            step_cfg["in4"],
        )
        self.steps_open = step_cfg["steps_open"]
        self.steps_close = step_cfg["steps_close"]
        self.delay = step_cfg["delay"]
        self.gate_open_time = hw_cfg["gate_open_time"]
        
        # half-step sequence
        self.step_seq = [
            [1,0,0,0],
            [1,1,0,0],
            [0,1,0,0],
            [0,1,1,0],
            [0,0,1,0],
            [0,0,1,1],
            [0,0,0,1],
            [1,0,0,1],
        ]
        self.seq_len = len(self.step_seq)
        self.is_gate_open = False
        
        self._setup()
        
    def _setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in (self.in1, self.in2, self.in3, self.in4):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        # Only Red LED and Buzzer
        for pin in (RED_LED, BUZZER):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        logger.info("Stepper, red LED, and buzzer initialized.")
        atexit.register(self._cleanup)


        
    def _cleanup(self):
        for pin in (self.in1, self.in2, self.in3, self.in4):
            GPIO.output(pin, 0)
        GPIO.cleanup((self.in1, self.in2, self.in3, self.in4))
        logger.info("Stepper motor pins cleaned up")


    def step_motor(self, steps, clockwise=True, flash=False):
        """Move exactly `steps` half-steps in the given direction.
        If flash=True, red LED and buzzer will flash/beep with each step."""
        dir_step = 1 if clockwise else -1
        idx = 0  # start at the first position in sequence

        try:
            for i in range(steps):
                # Flash LED and buzzer while stepping
                if flash:
                    GPIO.output(RED_LED, 1)
                    GPIO.output(BUZZER, 1)
                # apply one half-step
                seq = self.step_seq[idx]
                GPIO.output(self.in1, seq[0])
                GPIO.output(self.in2, seq[1])
                GPIO.output(self.in3, seq[2])
                GPIO.output(self.in4, seq[3])
                time.sleep(self.delay / 2 if flash else self.delay)

                if flash:
                    GPIO.output(RED_LED, 0)
                    GPIO.output(BUZZER, 0)
                # advance index, wrapping around
                idx = (idx + dir_step) % self.seq_len
                time.sleep(self.delay / 2 if flash else 0)
        except Exception as e:
            logger.error(f"Error stepping motor: {e}")
        finally:
            # always turn off coils to avoid heating
            for pin in (self.in1, self.in2, self.in3, self.in4):
                GPIO.output(pin, 0)
        logger.info(f"Rotated motor {steps} steps {'CW' if clockwise else 'CCW'}")

        

    def open_gate(self):
        if self.is_gate_open:
            logger.info("Gate already open")
            return
        logger.info("Opening gate…")
        self.is_gate_open = True
        self.step_motor(self.steps_open, clockwise=True, flash=True)
        # schedule auto-close
        t = threading.Timer(self.gate_open_time, self.close_gate)
        t.daemon = True
        t.start()
        logger.info(f"Gate will auto-close in {self.gate_open_time}s")

    def close_gate(self):
        if not self.is_gate_open:
            logger.info("Gate already closed")
            return
        logger.info("Closing gate…")
        self.step_motor(self.steps_close, clockwise=False, flash=True)
        self.is_gate_open = False
        logger.info("Gate closed")



# -------------------- PLATE RECOGNIZER -------------------- #
class PlateRecognizer:
    def __init__(self, config):
        self.config = config
        self.api_key = config["google_vision"]["api_key"]
        
    def recognize_plate(self):
        """Capture image and recognize license plate using the working logic"""
        try:
            # Initialize camera
            camera = PiCamera()
            camera.resolution = self.config["camera"]["resolution"]
            raw_capture = PiRGBArray(camera)
            time.sleep(2)  # Allow camera to warm up
    
            # Capture image
            camera.capture(raw_capture, format="bgr")
            image = raw_capture.array
            camera.close()
    
            # Save original image for debug
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            original_image_path = f"original_{timestamp}.jpg"
            cv2.imwrite(original_image_path, image)
    
            # Image preprocessing to improve OCR
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Enhanced contrast using CLAHE
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced = clahe.apply(gray)
            
            # Apply adaptive thresholding
            thresh = cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                         cv2.THRESH_BINARY_INV, 11, 2)
            
            # Apply morphological operations to clean the image
            kernel = np.ones((3, 3), np.uint8)
            morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # Save preprocessed image for debugging
            processed_image_path = f"processed_{timestamp}.jpg"
            cv2.imwrite(processed_image_path, morph)
            
            # Call Google Cloud Vision API for handwritten text recognition
            text = self.call_google_vision_api(original_image_path)
            
            # If Google API fails, try a fallback with our processed image
            if not text:
                text = self.call_google_vision_api(processed_image_path)
            
            # Clean up the recognized text - remove non-alphanumeric characters
            if text:
                clean_text = re.sub(r'[^A-Z0-9]', '', text.upper())
                logger.info(f"Recognized text: {clean_text}")
                return clean_text
            else:
                logger.warning("No text detected")
                return "ERROR"
    
        except Exception as e:
            logger.error(f"Error during plate recognition: {e}")
            import traceback
            traceback.print_exc()
            return "ERROR"
            
    def call_google_vision_api(self, image_path):
        """Call Google Cloud Vision API to recognize text in the image."""
        try:
            # Read the image file
            with open(image_path, "rb") as image_file:
                encoded_image = base64.b64encode(image_file.read()).decode('utf-8')
            
            # API endpoint for Google Cloud Vision
            url = f"https://vision.googleapis.com/v1/images:annotate?key={self.api_key}"
            
            # Prepare the request payload
            payload = {
                "requests": [
                    {
                        "image": {
                            "content": encoded_image
                        },
                        "features": [
                            {
                                "type": "DOCUMENT_TEXT_DETECTION"  # Better for handwritten text than TEXT_DETECTION
                            }
                        ],
                        "imageContext": {
                            "languageHints": ["en"]  # Language hint (adjust if needed)
                        }
                    }
                ]
            }
            
            # Make the API request
            response = requests.post(url, json=payload)
            
            # Check if the request was successful
            if response.status_code == 200:
                result = response.json()
                # Extract the text from the response
                if 'responses' in result and result['responses'] and 'fullTextAnnotation' in result['responses'][0]:
                    text = result['responses'][0]['fullTextAnnotation']['text']
                    return text
                else:
                    logger.warning("No text found in the image")
                    return ""
            else:
                logger.error(f"Google Vision API error: {response.status_code} - {response.text}")
                return ""
                
        except Exception as e:
            logger.error(f"Google Vision API call error: {e}")
            return ""
            
    def recognize_plate_from_image(self, image):
        """Recognize plate from an already captured image using the working logic"""
        try:
            if image is None:
                logger.error("No image provided for plate recognition")
                return "ERROR"
                
            # Save provided image for processing
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            temp_image_path = f"temp_{timestamp}.jpg"
            cv2.imwrite(temp_image_path, image)
    
            # Image preprocessing to improve OCR
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Enhanced contrast using CLAHE
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            enhanced = clahe.apply(gray)
            
            # Apply adaptive thresholding
            thresh = cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                         cv2.THRESH_BINARY_INV, 11, 2)
            
            # Apply morphological operations to clean the image
            kernel = np.ones((3, 3), np.uint8)
            morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            
            # Save preprocessed image for debugging
            processed_image_path = f"processed_{timestamp}.jpg"
            cv2.imwrite(processed_image_path, morph)
            
            # Call Google Cloud Vision API for text recognition
            text = self.call_google_vision_api(temp_image_path)
            
            # If Google API fails, try a fallback with our processed image
            if not text:
                text = self.call_google_vision_api(processed_image_path)
            
            # Clean up the recognized text - remove non-alphanumeric characters
            if text:
                clean_text = re.sub(r'[^A-Z0-9]', '', text.upper())
                logger.info(f"Recognized text from provided image: {clean_text}")
                return clean_text
            else:
                logger.warning("No text detected in provided image")
                return "ERROR"
                
        except Exception as e:
            logger.error(f"Error during plate recognition from provided image: {e}")
            import traceback
            traceback.print_exc()
            return "ERROR"
# -------------------- OTP MANAGER -------------------- #
class OTPManager:
    def __init__(self, config):
        self.config = config
        self.valid_otps = {}  # Store OTPs for verification
        self.otp_expiry = {}  # Store OTP expiry timestamps
        self.expiry_minutes = config["otp_expiry"]
        
        # Start cleanup thread
        self.cleanup_thread = threading.Thread(target=self._cleanup_expired_otps, daemon=True)
        self.cleanup_thread.start()
        
    def generate_otp(self, guest_name, length=4):
        """Generate a numeric OTP of fixed length with expiry"""
        otp = ''.join([str(random.randint(0, 9)) for _ in range(length)])
        self.valid_otps[otp] = guest_name
        expiry_time = datetime.now() + timedelta(minutes=self.expiry_minutes)
        self.otp_expiry[otp] = expiry_time
        logger.info(f"Generated OTP for guest '{guest_name}': {otp}, expires at {expiry_time}")
        return otp
        
    def verify_otp(self, otp):
        """Verify if OTP is valid and not expired"""
        if otp in self.valid_otps:
            guest_name = self.valid_otps[otp]
            expiry_time = self.otp_expiry.get(otp)
            if expiry_time and datetime.now() > expiry_time:
                logger.warning(f"OTP {otp} for guest '{guest_name}' has expired")
                self.valid_otps.pop(otp)
                self.otp_expiry.pop(otp)
                return False, None
            # Valid OTP, remove after use
            self.valid_otps.pop(otp)
            self.otp_expiry.pop(otp, None)
            return True, guest_name
        return False, None
        
    def _cleanup_expired_otps(self):
        """Periodically clean up expired OTPs"""
        while not shutdown_requested:
            try:
                now = datetime.now()
                expired_otps = [otp for otp, expiry in self.otp_expiry.items() if now > expiry]
                for otp in expired_otps:
                    if otp in self.valid_otps:
                        guest = self.valid_otps.pop(otp)
                        logger.info(f"Removed expired OTP {otp} for guest '{guest}'")
                    self.otp_expiry.pop(otp, None)
            except Exception as e:
                logger.error(f"Error in OTP cleanup: {e}")
            time.sleep(60)  # Check every minute

# -------------------- SENSOR MANAGER -------------------- #
class SensorManager:
    def __init__(self, config):
        self.config = config
        self.pir_pin = config["hardware"]["pir_pin"]
        self._setup()
        
    def _setup(self):
        """Initialize sensors"""
        try:
            grovepi.pinMode(self.pir_pin, "INPUT")
            logger.info(f"PIR motion sensor initialized on D{self.pir_pin}")
        except Exception as e:
            logger.error(f"Error initializing PIR sensor: {e}")
            
    def check_motion(self):
        """Check for motion detection"""
        try:
            motion = grovepi.digitalRead(self.pir_pin)
            return motion
        except Exception as e:
            logger.error(f"Motion sensor error: {e}")
            return False
            
    def get_ldr_value(self):
        """Get light sensor value (mock implementation)"""
        try:
            # TODO: Implement actual LDR reading
            return 320
        except Exception as e:
            logger.error(f"LDR sensor error: {e}")
            return 0

# -------------------- CAMERA MANAGER -------------------- #
class CameraManager:
    def __init__(self, config):
        self.config = config
        self.is_streaming = False
        self.stream_process = None
        self.mjpg_path = config["camera"]["mjpg_path"]
        self.resolution = config["camera"]["resolution"]
        
    def start_stream(self):
        """Start MJPG streaming"""
        if self.is_streaming:
            logger.info("Stream already running")
            return True
            
        try:
            logger.info("Starting MJPG stream...")
            self.stream_process = subprocess.Popen([
                f"{self.mjpg_path}/mjpg_streamer",
                "-i", f"./input_raspicam.so -fps 15 -x {self.resolution[0]} -y {self.resolution[1]}",
                "-o", "./output_http.so -w ./www -p 8080"
            ], cwd=self.mjpg_path)
            
            self.is_streaming = True
            return True
        except Exception as e:
            logger.error(f"Error starting stream: {e}")
            return False
            
    def stop_stream(self):
        """Stop MJPG streaming"""
        if not self.is_streaming or not self.stream_process:
            logger.info("No stream running")
            return True
            
        try:
            logger.info("Stopping MJPG stream...")
            self.stream_process.terminate()
            self.stream_process.wait()
            self.is_streaming = False
            self.stream_process = None
            return True
        except Exception as e:
            logger.error(f"Error stopping stream: {e}")
            return False
            
    def capture_snapshot(self, path="/tmp/snapshot.jpg"):
        """Capture a snapshot using raspistill"""
        was_streaming = self.is_streaming
        
        # Temporarily stop streaming if active
        if was_streaming:
            self.stop_stream()
            
        try:
            subprocess.run([
                "raspistill",
                "-w", str(self.resolution[0]), 
                "-h", str(self.resolution[1]),
                "-o", path,
                "-n"  # no preview
            ], check=True)
            logger.info(f"Snapshot captured to {path}")
            success = True
        except subprocess.CalledProcessError as e:
            logger.error(f"Error capturing snapshot: {e}")
            success = False
            
        # Restart streaming if it was active
        if was_streaming:
            self.start_stream()
            
        return success, path

# -------------------- LOG MANAGER -------------------- #
class LogManager:
    def __init__(self, config):
        self.config = config
        self.log_file = config["files"]["log_file"]
        self.report_file = config["files"]["report_file"]
        
    def log_entry(self, plate, method="auto", guest_name=None):
        """Log gate entry with plate or guest information"""
        log_entry = {
            "plate": plate,
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "method": method
        }
        
        if guest_name:
            log_entry["guest"] = guest_name
            
        try:
            # Load existing log
            if os.path.exists(self.log_file):
                with open(self.log_file, "r") as f:
                    data = json.load(f)
            else:
                data = []
                
            # Append new entry
            data.append(log_entry)
            
            # Save back to file
            with open(self.log_file, "w") as f:
                json.dump(data, f, indent=2)
                
            logger.info(f"Entry logged: {log_entry}")
            return True
        except Exception as e:
            logger.error(f"Error writing log: {e}")
            return False
            
    def generate_weekly_report(self):
        """Generate weekly report of gate entries"""
        try:
            if not os.path.exists(self.log_file):
                logger.warning("No log file found for report generation")
                return None
                
            with open(self.log_file, "r") as f:
                logs = json.load(f)
                
            now = datetime.now()
            one_week_ago = now - timedelta(days=7)
            
            recent_logs = [
                entry for entry in logs
                if datetime.strptime(entry["time"], "%Y-%m-%d %H:%M:%S") >= one_week_ago
            ]
            
            total = len(recent_logs)
            by_method = {}
            by_plate = {}
            by_hour = {str(h): 0 for h in range(24)}
            by_day = {"Monday": 0, "Tuesday": 0, "Wednesday": 0, "Thursday": 0, "Friday": 0, "Saturday": 0, "Sunday": 0}
            
            for entry in recent_logs:
                method = entry["method"]
                plate = entry["plate"]
                entry_time = datetime.strptime(entry["time"], "%Y-%m-%d %H:%M:%S")
                hour = str(entry_time.hour)
                day = entry_time.strftime("%A")
                
                by_method[method] = by_method.get(method, 0) + 1
                by_plate[plate] = by_plate.get(plate, 0) + 1
                by_hour[hour] = by_hour.get(hour, 0) + 1
                by_day[day] = by_day.get(day, 0) + 1
                
            report = {
                "generated_at": now.strftime("%Y-%m-%d %H:%M:%S"),
                "entries_last_7_days": total,
                "by_method": by_method,
                "by_plate": by_plate,
                "by_hour": by_hour,
                "by_day": by_day,
                "most_active_hour": max(by_hour.items(), key=lambda x: x[1])[0] if by_hour else None,
                "most_active_day": max(by_day.items(), key=lambda x: x[1])[0] if by_day else None,
                "most_frequent_plates": sorted(by_plate.items(), key=lambda x: x[1], reverse=True)[:3] if by_plate else [],
                "least_active_plates": sorted(by_plate.items(), key=lambda x: x[1])[:3] if by_plate else []
            }
            
            # Save report
            with open(self.report_file, "w") as f:
                json.dump(report, f, indent=2)

           
            logger.info(f"Weekly report generated: {self.report_file}")
            return report
            
        except Exception as e:
            logger.error(f"Error generating report: {e}")
            return None

# -------------------- MQTT CLIENT -------------------- #
class MQTTClient:
    def __init__(self, config, firebase_manager, stepper_manager, plate_recognizer, otp_manager, camera_manager, log_manager):
        self.config = config
        self.firebase = firebase_manager
        self.stepper = stepper_manager  # Changed from servo_manager
        self.plate_recognizer = plate_recognizer
        self.otp_manager = otp_manager
        self.camera = camera_manager
        self.log_manager = log_manager
        
        self.mqtt_client = None
        self.connected = False
        self.topics = config["topics"]
        self.reconnect_timer = None
        
    def connect(self):
        """Connect to MQTT broker"""
        try:
            client_id = self.config["mqtt"]["client_id"]
            self.mqtt_client = mqtt.Client(client_id=client_id)
            self.mqtt_client.username_pw_set(
                self.config["mqtt"]["username"], 
                self.config["mqtt"]["password"]
            )
            self.mqtt_client.tls_set()
            
            # Set callbacks
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_message = self._on_message
            self.mqtt_client.on_disconnect = self._on_disconnect
            
            # Connect to broker
            logger.info(f"Connecting to MQTT broker at {self.config['mqtt']['broker']}:{self.config['mqtt']['port']}...")
            self.mqtt_client.connect(
                self.config["mqtt"]["broker"], 
                self.config["mqtt"]["port"], 
                60
            )
            
            # Start network loop
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            logger.error(f"MQTT connection error: {e}")
            self._schedule_reconnect()
            return False
            
    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.mqtt_client:
            try:
                # Cancel any reconnection timer
                if self.reconnect_timer:
                    self.reconnect_timer.cancel()
                    self.reconnect_timer = None
                
                # Publish offline status
                self.publish_status("offline")
                
                # Stop loop and disconnect
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                logger.info("Disconnected from MQTT broker")
            except Exception as e:
                logger.error(f"MQTT disconnect error: {e}")
                
    def _schedule_reconnect(self, delay=5):
        """Schedule reconnection attempt after delay"""
        if not self.reconnect_timer:
            logger.info(f"Scheduling reconnection in {delay} seconds")
            self.reconnect_timer = threading.Timer(delay, self._reconnect)
            self.reconnect_timer.daemon = True
            self.reconnect_timer.start()
            
    def _reconnect(self):
        """Attempt to reconnect to MQTT broker"""
        try:
            if not self.connected and not shutdown_requested:
                logger.info("Attempting to reconnect to MQTT broker...")
                self.connect()
        except Exception as e:
            logger.error(f"Reconnection error: {e}")
            self._schedule_reconnect(10)  # Try again with longer delay
        finally:
            self.reconnect_timer = None
            
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            logger.info("Connected to MQTT broker successfully")
            self.connected = True
            
            # Subscribe to command topics
            subscribe_topics = [
                (self.topics["command"]["open"], 0),
                (self.topics["command"]["generate_otp"], 0),
                (self.topics["command"]["verify_otp"], 0),
                (self.topics["command"]["report"], 0),
                (self.topics["command"]["camera"], 0),
                (self.topics["command"]["detect_plate"], 0)
            ]
            client.subscribe(subscribe_topics)
            logger.info(f"Subscribed to {len(subscribe_topics)} topics")
            
            # Publish online status
            self.publish_status("online")
        else:
            logger.error(f"Failed to connect to MQTT broker with code {rc}")
            self.connected = False
            self._schedule_reconnect()
            
    def _on_disconnect(self, client, userdata, rc):
        """Callback for when client disconnects from broker"""
        logger.warning(f"Disconnected from MQTT broker with code {rc}")
        self.connected = False
        
        # Schedule reconnection if not requested by user
        if rc != 0 and not shutdown_requested:
            self._schedule_reconnect()
            
    def _on_message(self, client, userdata, msg):
   
        topic = msg.topic
        try:
            payload = msg.payload.decode()
            logger.info(f"Received message on {topic}: {payload}")
            
            # Process message based on topic
            if topic == self.topics["command"]["open"]:
                self._handle_open_command(payload)
                
            elif topic == self.topics["command"]["camera"]:
                self._handle_camera_command(payload)
                
            elif topic == self.topics["command"]["generate_otp"]:
                self._handle_generate_otp_command(payload)
                
            elif topic == self.topics["command"]["verify_otp"]:
                self._handle_verify_otp_command(payload)
                
            elif topic == self.topics["command"]["report"]:
                self._handle_report_command(payload)
                
            elif topic == self.topics["command"]["detect_plate"]:
                self._handle_detect_plate_command(payload)

        except Exception as e:
            logger.error(f"Error processing message: {e}")

    def _handle_open_command(self, payload):
   
        try:
            self.stepper.open_gate()  # Use stepper instead of servo
            self.mqtt_client.publish(
                self.topics["status"]["entry"], 
                json.dumps({"status": "Gate Opened Manually"})
            )
            logger.info("Gate opened manually via MQTT command")
        except Exception as e:
            logger.error(f"Error handling open command: {e}")
            
    def _handle_camera_command(self, payload):
        """Handle camera control command"""
        try:
            action = payload.strip().lower()
            if action == "start":
                if self.camera.start_stream():
                    self.mqtt_client.publish(
                        self.topics["status"]["camera"], 
                        json.dumps({"status": "streaming", "port": 8080})
                    )
            elif action == "stop":
                if self.camera.stop_stream():
                    self.mqtt_client.publish(
                        self.topics["status"]["camera"], 
                        json.dumps({"status": "stopped"})
                    )
            else:
                logger.warning(f"Unknown camera command: {action}")
        except Exception as e:
            logger.error(f"Error handling camera command: {e}")
            
    def _handle_generate_otp_command(self, payload):
        """Handle OTP generation command"""
        try:
            guest_name = payload.strip()
            otp = self.otp_manager.generate_otp(guest_name)
            
            self.mqtt_client.publish(
                self.topics["status"]["otp"], 
                json.dumps({
                    "guest": guest_name,
                    "otp": otp
                })
            )
            logger.info(f"Generated OTP for guest '{guest_name}'")
        except Exception as e:
            logger.error(f"Error handling generate OTP command: {e}")
            
    def _handle_verify_otp_command(self, payload):
        """Handle OTP verification command"""
        try:
            data = json.loads(payload)
            otp = data.get("otp")
            
            valid, guest_name = self.otp_manager.verify_otp(otp)
            
            if valid and guest_name:
                self.stepper.open_gate()  # Changed from servo to stepper
                self.log_manager.log_entry(guest_name, method="otp", guest_name=guest_name)
                
                self.mqtt_client.publish(
                    self.topics["status"]["entry"], 
                    json.dumps({
                        "status": "Gate Opened via OTP", 
                        "guest": guest_name
                    })
                )
                
                self.firebase.send_push_notification(
                    "Gate Accessed", 
                    f"{guest_name} entered using OTP."
                )
                
                logger.info(f"Valid OTP used by guest '{guest_name}'")
            else:
                self.mqtt_client.publish(
                    self.topics["security"]["alerts"], 
                    json.dumps({
                        "error": "Invalid OTP", 
                        "otp": otp
                    })
                )
                logger.warning(f"Invalid OTP attempt: {otp}")
        except Exception as e:
            logger.error(f"Error handling verify OTP command: {e}")
            
            
    def _handle_report_command(self, payload):
        """Handle report generation command"""
        try:
            report = self.log_manager.generate_weekly_report()
            
            if report:
                # Upload to Firebase
                self.firebase.upload_report(report)
                
                # Publish confirmation
                 # publish simple confirmation
                self.mqtt_client.publish(
                    self.topics["status"]["report"],
                    json.dumps({
                        "status": "Weekly report generated and uploaded",
                        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    })
                )
                # publish the full report data for your Dart app
                self.mqtt_client.publish(
                    self.topics["status"]["report_data"],
                    json.dumps(report)
                )
                logger.info("Weekly report generated and uploaded")
            else:
                self.mqtt_client.publish(
                    self.topics["status"]["report"], 
                    json.dumps({
                        "status": "Error generating report",
                        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    })
                )
                logger.error("Failed to generate weekly report")
        except Exception as e:
            logger.error(f"Error handling report command: {e}")
            
    def _handle_detect_plate_command(self, payload):
        """Handle license plate detection command"""
        try:
            # Capture snapshot
            success, snapshot_path = self.camera.capture_snapshot()
            
            if success:
                # Load image & run OCR
                image = cv2.imread(snapshot_path)
                plate = self.plate_recognizer.recognize_plate_from_image(image)
                plate = plate.upper() if plate else "ERROR"
                
                # Publish result
                self.mqtt_client.publish(
                    self.topics["status"]["plate"], 
                    json.dumps({"plate": plate})
                )
                logger.info(f"Plate detection from app request: {plate}")
            else:
                self.mqtt_client.publish(
                    self.topics["status"]["plate"], 
                    json.dumps({"plate": "ERROR", "reason": "Failed to capture image"})
                )
                logger.error("Failed to capture image for plate detection")
        except Exception as e:
            logger.error(f"Error handling detect plate command: {e}")
            self.mqtt_client.publish(
                self.topics["status"]["plate"], 
                json.dumps({"plate": "ERROR", "reason": str(e)})
            )
            
    def publish_status(self, status):
        """Publish system status message"""
        try:
            self.mqtt_client.publish(
                self.topics["system"]["status"], 
                json.dumps({
                    "status": status,
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "uptime": self._get_uptime()
                }),
                retain=True
            )
            logger.info(f"Published system status: {status}")
        except Exception as e:
            logger.error(f"Error publishing status: {e}")
            
    def _get_uptime(self):
        """Get system uptime in human-readable format"""
        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.readline().split()[0])
                
            days = int(uptime_seconds // 86400)
            hours = int((uptime_seconds % 86400) // 3600)
            minutes = int((uptime_seconds % 3600) // 60)
            
            return f"{days}d {hours}h {minutes}m"
        except Exception as e:
            logger.error(f"Error getting uptime: {e}")
            return "unknown"
            
    def start_status_updates(self, interval=60):
        """Start periodic status updates"""
        def update_status():
            while not shutdown_requested and self.connected:
                try:
                    self.publish_status("online")
                except Exception as e:
                    logger.error(f"Error in status update: {e}")
                time.sleep(interval)
                
        status_thread = threading.Thread(target=update_status, daemon=True)
        status_thread.start()
        logger.info(f"Started periodic status updates every {interval} seconds")

if __name__ == "__main__":
    # Register signal handlers for graceful shutdown
    def handle_shutdown_signal(signum, frame):
        global shutdown_requested
        logger.info(f"Received shutdown signal {signum}")
        shutdown_requested = True
        
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    
    # Initialize all components
    firebase_mgr = FirebaseManager(CONFIG)
    stepper_mgr = StepperMotorManager(CONFIG)  # Use stepper instead of servo
    plate_recog = PlateRecognizer(CONFIG)
    otp_mgr = OTPManager(CONFIG)
    camera_mgr = CameraManager(CONFIG)
    log_mgr = LogManager(CONFIG)

    # Create and start MQTT client
    mqtt_client = MQTTClient(
        CONFIG,
        firebase_mgr,
        stepper_mgr,  # Use stepper_mgr instead of servo_mgr
        plate_recog,
        otp_mgr,
        camera_mgr,
        log_mgr
    )
    
    if not mqtt_client.connect():
        logger.error("Exiting: can't connect to MQTT broker")
        sys.exit(1)

    # Start periodic system status updates
    mqtt_client.start_status_updates(CONFIG["system_status_interval"])

    # Main loop
    try:
        # Run until shutdown is requested
        while not shutdown_requested:
            # Just keep the main thread alive
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutdown requested by user")
    finally:
        # Clean shutdown
        shutdown_requested = True
        mqtt_client.disconnect()
        GPIO.cleanup()  # Clean up GPIO
        logger.info("Clean exit complete")
        sys.exit(0)