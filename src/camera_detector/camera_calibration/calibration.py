import cv2
import numpy as np
import os

# Dimensioni della scacchiera
chessboard_size = (9, 6)
square_size = 0.025

print("start calibration")

# Criteri di terminazione per la ricerca degli angoli della scacchiera
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparazione degli oggetti punti (0,0,0), (1,0,0), (2,0,0) ..., (10,7,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Liste per memorizzare i punti oggetto e i punti immagine da tutti i frame
objpoints = []  # Punti 3D nel mondo reale
imgpoints = []  # Punti 2D nel piano dell'immagine

# Carica il video
video_path = "./src/camera_detector/camera_calibration/2025-05-05-115559.webm"
cap = cv2.VideoCapture(video_path)

image_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
image_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Video width: {image_width}, height: {image_height}")

# Ottieni il numero totale di frame nel video
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
frame_count = 0

while cap.isOpened() and frame_count < total_frames:

    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_count)
    ret, frame = cap.read()
    
    if not ret:
        break

    print(ret)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # Se sono stati trovati gli angoli, aggiungi i punti oggetto e i punti immagine
    if found:
        annotated_frame = cv2.drawChessboardCorners(frame, chessboard_size, corners, found)
        cv2.imshow("Chessboard", annotated_frame)
        cv2.waitKey(1)
        print(f"OK: Frame elaborati: {frame_count}/{total_frames} ({frame_count / total_frames:.2%})")
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
    else:
        print(f"NO: Frame elaborati: {frame_count}/{total_frames} ({frame_count / total_frames:.2%})")
    
    frame_count += 20

# Calibrazione della fotocamera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]
print(f"fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}")

# # Salva i risultati in un file
file_name = os.path.basename(video_path).split(".")[0]
np.savez(f'./src/camera_detector/camera_calibration/camera_calibration_{file_name}.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
print("file salvato correttamente") 

#Snippet per calcolare il FOV orizzontale
fov_x_rad = 2 * np.arctan(image_width / (2 * fx))
fov_x_deg = np.degrees(fov_x_rad)
print(f"FOV orizzontale: {fov_x_deg:.2f}°")


