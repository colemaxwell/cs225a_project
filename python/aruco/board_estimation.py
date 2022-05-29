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
current_board_string = ""

pieceNames = ["BISHOP", "KING", "KNIGHT", "PAWN", "QUEEN", "ROOK"]
pieceSymbols = ["B", "K", "N", "P", "Q", "R"]
#nums = {1:"a", 2:"b", 3:"c", 4:"d", 5:"e", 6:"f", 7:"g", 8:"h"}

letters = ["a", "b", "c", "d", "e", "f", "g", "h"]
numbers = ["8", "7", "6", "5", "4", "3", "2", "1"]

def fen_to_board(fen):
    board = []
    for row in fen.split('/'):
        brow = []
        for c in row:
            if c == ' ':
                break
            elif c in '12345678':
                brow.extend( ['--'] * int(c) )
            elif c == 'p':
                brow.append( 'bp' )
            elif c == 'P':
                brow.append( 'wp' )
            elif c > 'Z':
                brow.append( 'b'+c.upper() )
            else:
                brow.append( 'w'+c )

        board.append( brow )
    return board


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

        #else:
            #print("Full Board Not Found")



    return frame, board


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
    redisClient = redis.Redis()
    aruco_dict_type = ARUCO_DICT["DICT_4X4_100"]
    calibration_matrix_path = "calibration_matrix.npy"
    distortion_coefficients_path = "distortion_coefficients.npy"

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(0)
    time.sleep(2)
    
    initial_board = ""
    final_board = ""
    
    board_width = 8
    board_height = 8

    while True:
        ret, frame = video.read()
                       
        output, board = scan_board(frame, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)
        
        ready_initial = redisClient.get('READY_SCAN_BOARD_INITIAL').decode('utf-8')
        ready_final = redisClient.get('READY_SCAN_BOARD_FINAL').decode('utf-8')
          
        if (ready_initial == "1"):
        	initial_board = fen_to_board(board.fen())
        	print(initial_board)
        	redisClient.set('READY_SCAN_BOARD_INITIAL', 0)
        	ready_initial = "0"
        elif (ready_final == "1"):
        	final_board = fen_to_board(board.fen())
        	
        	initial_move = ""
        	final_move = ""
        	
        	for i in range(board_width):
        		for j in range(board_height):
        			if initial_board[i][j] != final_board[i][j]:
        				if (initial_board[i][j] != '--'):
        					initial_move = str(letters[j]) + str(numbers[i])
        				else:
        					final_move = str(letters[j]) + str(numbers[i])
        	move = initial_move + final_move
        	
        	print(move)
        	redisClient.set('PLAYER_MOVE_KEY',move)
        	redisClient.set('READY_SCAN_BOARD_FINAL', 0)
        	ready_final = "0"
        	
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
        	break
    video.release()
    cv2.destroyAllWindows()
