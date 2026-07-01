""""
# Dimensões reais do retângulo formado pelos 4 ArUcos na lousa do laboratório de RMA
REAL_WIDTH =  0.78 # 78 cm  
REAL_HEIGHT = 0.52 # 52 cm  
"""
import cv2
import numpy as np
import csv
import time
import math

# ==========================================
# PARÂMETROS DO SETUP FÍSICO
# ==========================================
ID_ROBOT = 4
IDS_CORNERS = {0: "TOP_LEFT", 1: "TOP_RIGHT", 2: "BOTTOM_RIGHT", 3: "BOTTOM_LEFT"}

REAL_WIDTH = 0.6  # 78 cm  
REAL_HEIGHT = 0.47 # 52 cm  

dst_pts = np.array([
    [0, REAL_HEIGHT],               #TOP LEFT 
    [REAL_WIDTH, REAL_HEIGHT],      #TOP RIGHT
    [REAL_WIDTH, 0],                #BOT RIGHT
    [0, 0]                          #BOT LEFT
], dtype=np.float32)

def load_calibration_data():
    camera_matrix = np.load('/home/vitor/artigo_ieee_ws/src/robot_digital_twin/config/camera_matrix.npy')
    dist_coeffs   = np.load('/home/vitor/artigo_ieee_ws/src/robot_digital_twin/config/dist_coeffs.npy')
    return camera_matrix, dist_coeffs

def main():
    camera_matrix, dist_coeffs = load_calibration_data()
    cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    # Variáveis de controle de gravação e offsets
    recording = False
    csv_file = None
    csv_writer = None
    off_x, off_y, off_theta = 0.0, 0.0, 0.0

    print("--- MONITOR DE GROUND TRUTH ---")
    print("Comandos: 's' para Iniciar Gravação | 'f' para Finalizar e Salvar | 'q' para Sair")

    homography_matrix = None

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame_undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)

            # 1. Cálculo da Homografia (Matriz de Transformação Metrica)
            corner_pts = {}
            for i, marker_id in enumerate(ids):
                if marker_id in IDS_CORNERS:
                    c = corners[i][0]
                    cx, cy = np.mean(c, axis=0)
                    corner_pts[marker_id] = [cx, cy]

            if len(corner_pts) == 4:
                src_pts = np.array([corner_pts[0], corner_pts[1], corner_pts[2], corner_pts[3]], dtype=np.float32)
                homography_matrix, _ = cv2.findHomography(src_pts, dst_pts)

            # 2. Rastreamento do Robô
            if homography_matrix is not None and ID_ROBOT in ids:
                robot_idx = np.where(ids == ID_ROBOT)[0][0]
                rc = corners[robot_idx][0]
                
                # Centro e Frente em Pixels
                robot_cx, robot_cy = np.mean(rc, axis=0)
                robot_fx, robot_fy = np.mean(rc[0:2], axis=0)

                # Transformação para Metros (Global ArUco Frame)
                pts_pixel = np.array([[[robot_cx, robot_cy], [robot_fx, robot_fy]]], dtype=np.float32)
                pts_real = cv2.perspectiveTransform(pts_pixel, homography_matrix)
                
                raw_x, raw_y = pts_real[0][0]
                raw_fx, raw_fy = pts_real[0][1]
                raw_theta = math.atan2(raw_fy - raw_y, raw_fx - raw_x)

                # Lógica de Gravação e Transformação Local
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('s') and not recording:
                    # Captura o estado atual como Origem (0,0,0)
                    off_x, off_y, off_theta = raw_x, raw_y, raw_theta
                    csv_file = open('ground_truth.csv', mode='w', newline='')
                    csv_writer = csv.writer(csv_file)
                    csv_writer.writerow(['timestamp', 'x', 'y', 'theta'])
                    recording = True
                    print(">> Gravação INICIADA. Sistema de coordenadas zerado.")

                if key == ord('f') and recording:
                    recording = False
                    csv_file.close()
                    print(">> Gravação FINALIZADA. Arquivo 'ground_truth.csv' salvo.")

                if recording:
                    # 3. Cálculo das Coordenadas Relativas e Rotação de Frame
                    # Subtrai posição inicial
                    dx = raw_x - off_x
                    dy = raw_y - off_y
                    
                    # Rotaciona para alinhar o Heading inicial com o Eixo X local
                    # X_local = dx*cos(-off_theta) - dy*sin(-off_theta)
                    local_x = dx * math.cos(off_theta) + dy * math.sin(off_theta)
                    local_y = -dx * math.sin(off_theta) + dy * math.cos(off_theta)
                    local_theta = raw_theta - off_theta
                    
                    csv_writer.writerow([time.time(), local_x, local_y, local_theta])

                    # Feedback visual de gravação
                    cv2.circle(frame_undistorted, (30, 30), 10, (0, 0, 255), -1)
                    cv2.putText(frame_undistorted, "GRAVANDO", (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(frame_undistorted, f"LX: {local_x:.3f} LY: {local_y:.3f} LT: {local_theta:.2f}", 
                                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    cv2.putText(frame_undistorted, "Aguardando 's' para iniciar", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Ground Truth Metric Tracking", frame_undistorted)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()
    if recording: csv_file.close()

if __name__ == "__main__":
    main()