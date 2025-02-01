import cv2
import mediapipe as mp
import numpy as np
from tkinter import Tk, Label
from PIL import Image, ImageTk
import math
import serial

root = Tk()
root.title("Hand Gesture Servo Control")

video_label = Label(root)
video_label.pack()

status_label = Label(root, text="Status: Initializing...", fg="blue")
status_label.pack()

# Initialize Mediapipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Initialize Webcam
cap = cv2.VideoCapture(0)

# Initialize Arduino Serial Communication
try:
    arduino = serial.Serial(port='/dev/tty.usbmodemXXXX', baudrate=9600, timeout=1)  # Update the port as needed
    status_label.config(text="Status: Arduino Connected", fg="green")
except Exception as e:
    arduino = None
    status_label.config(text=f"Status: Arduino Not Connected - {str(e)}", fg="red")

def process_frame(frame):
    """
    Process the frame to track hand landmarks and calculate the angle.
    """
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    angle = None
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Extract coordinates for the index fingertip and wrist
            index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

            # Convert normalized coordinates to pixel values
            index_tip_coords = (int(index_tip.x * frame.shape[1]), int(index_tip.y * frame.shape[0]))
            wrist_coords = (int(wrist.x * frame.shape[1]), int(wrist.y * frame.shape[0]))

            # Calculate angle
            angle = calculate_angle(index_tip_coords, wrist_coords)

            # Display the calculated angle on the frame
            cv2.putText(frame, f"Angle: {angle} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return frame, angle

# Update video feed in the GUI
def update_video_feed(frame):
    image = Image.fromarray(frame)
    photo = ImageTk.PhotoImage(image)
    video_label.configure(image=photo)
    video_label.image = photo

def send_angle_to_arduino(angle):
    """
    Send the calculated angle to the Arduino via serial communication.
    """
    if arduino and angle is not None:
        try:
            arduino.write(f"{angle}\n".encode())
            status_label.config(text=f"Angle Sent: {angle}°", fg="green")
        except Exception as e:
            status_label.config(text=f"Error Sending Angle: {str(e)}", fg="red")

def calculate_angle(index_tip, wrist):
    """
    Calculate the angle formed by the index fingertip with respect to the horizontal axis.
    """
    dx = index_tip[0] - wrist[0]
    dy = index_tip[1] - wrist[1]
    angle_radians = math.atan2(dy, dx)
    angle_degrees = math.degrees(angle_radians)

    # Ensure angles are in the range [0, 180] (upper semicircle)
    if angle_degrees < 0:
        angle_degrees += 360
    if angle_degrees > 180:
        angle_degrees = 360 - angle_degrees

    return int(angle_degrees)



# Main Loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame, angle = process_frame(frame)

    # Update GUI with video feed
    update_video_feed(frame)

    # Print angle to terminal and send it to Arduino
    if angle is not None:
        print(f"Angle: {angle} degrees")
        send_angle_to_arduino(angle)

    root.update_idletasks()
    root.update()

# Release resources
cap.release()
hands.close()
if arduino:
    arduino.close()

