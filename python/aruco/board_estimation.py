'''
Sample Usage:-
python board_estimation.py
'''

import redis
import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time

import chess
import chess.svg
from skimage.transform import ProjectiveTransform
import math

scan_num = 0
scan_redundancy = 10

board = chess.Board()

pieceNames = ["BISHOP", "KING", "KNIGHT", "PAWN", "QUEEN", "ROOK"]
pieceSymbols = ["B", "K", "N", "P", "Q", "R"]

def scan_board(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''
    global scan_num

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    if(scan_num % 20 == 0):
        board.clear()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

    boardCorners = np.asarray([[0, 0], [0, 0], [0, 0], [0, 0]])
    boardCornersCount = 0
    # If markers are detected
    # Draw a square around the markers
    cv2.aruco.drawDetectedMarkers(frame, corners, ids) 
    if len(corners) > 0:
        for i in range(0, len(ids)):
            idNum = ids[i][0]
            if (idNum ==10):
                boardCorners[0] = corners[i][0][3]
                boardCornersCount += 1
            if (idNum ==11):
                boardCorners[1] = corners[i][0][2]
                boardCornersCount += 1
            if (idNum ==12):
                boardCorners[2] = corners[i][0][1]
                boardCornersCount += 1
            if (idNum ==13):
                boardCorners[3] = corners[i][0][0]
                boardCornersCount += 1
        if (boardCornersCount >= 4):
            t = ProjectiveTransform()
            dst = np.asarray([[0, 8], [8, 8], [0, 0], [8, 0]])
            if not t.estimate(boardCorners, dst): raise Exception("estimate failed")

            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                           distortion_coefficients)

                # Draw Axis
                cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

                #Board
                idNum = ids[i][0]
                if (idNum < 6):
                    center = np.array([0, 0])
                    for j in range(0, 4):
                        center = center + corners[i][0][j]
                    center = center / 4.0
                    [[xfloat,yfloat]] = t(center)
                    x = math.floor(xfloat)
                    y = math.floor(yfloat)
                    square = chess.square(x,y)
                    pieceSymbol = pieceSymbols[idNum]
                    piece = chess.Piece.from_symbol(pieceSymbol)
                    board.set_piece_at(square,piece)
            
            scan_num += 1
            if(scan_num % 20 == 0):
                print(board)
                print("")
                #chess.svg.board(board)

        #else:
            #print("Full Board Not Found")



    return frame


if __name__ == '__main__':

    '''
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)
    

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    '''
    aruco_dict_type = ARUCO_DICT["DICT_4X4_100"]
    calibration_matrix_path = "calibration_matrix.npy"
    distortion_coefficients_path = "distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            print("Cannot find video stream")
            print("Try plugging in a webcam")
            break
        
        output = scan_board(frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()