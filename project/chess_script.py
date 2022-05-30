import chess
import redis
import numpy as np
import time
from stockfish import Stockfish 


if __name__ == "__main__":

	
	stockfish = Stockfish(path="../python/stockfish_15_linux_x64/stockfish_15_src/src/stockfish", parameters={"Minimum Thinking Time":10000})
	redisClient = redis.Redis()
	board = chess.Board()
	
	n_rows = 8
	n_cols = 8
	chessboard_array = np.zeros((8, 8, 2))
	for i in range(n_rows):
		chessboard_array[i, :, 0] = 0.61-0.06*i
	for i in range(n_cols):
		chessboard_array[:, n_cols-1-i, 1] = 0.21-0.06*i
	
	letters = ["a", "b", "c", "d", "e", "f", "g", "h"]
	numbers = ["1", "2", "3", "4", "5", "6", "7", "8"]
	
	chessboard_dictionary = {}
	chessboard_grid_positions = {}
	
	for i in range(n_rows):
		for j in range(n_cols):
			key = letters[j]+numbers[i]
			chessboard_grid_positions.update({key:chessboard_array[i,j,:]})
	
	chessboard_dictionary = {
	"h1": "WRook1", "g1": "WKnight1", "f1":"WBishop1", "e1": "WQueen", "d1": "WKing", "c1": "WBishop2", "b1": "WKnight2", "a1": "WRook2", 
"h2": "WPawn1", "g2":"WPawn2", "f2":"WPawn3", "e2":"WPawn4", "d2":"WPawn5", "c2":"WPawn6", "b2":"WPawn7", "a2":"WPawn8",
"h3":"", "g3":"", "f3":"", "e3":"", "d3":"", "c3":"", "b3":"", "a3":"", 
"h4":"", "g4":"", "f4":"", "e4":"", "d4":"", "c4":"", "b4":"", "a4":"", 
"h5":"", "g5":"", "f5":"", "e5":"", "d5":"", "c5":"", "b5":"", "a5":"", 
"h6":"", "g6":"", "f6":"", "e6":"", "d6":"", "c6":"", "b6":"", "a6":"", 
"h7": "BPawn1", "g7": "BPawn2", "f7": "BPawn3", "e7": "BPawn4", "d7": "BPawn5", "c7": "BPawn6", "b7": "BPawn7", "a7": "BPawn8",
"h8":"BRook1", "g8":"BKnight1", "f8":"BBishop1", "e8":"BQueen", "d8":"BKing", "c8":"BBishop2", "b8":"BKnight2", "a8":"BRook2"}
	
	GAME_STATE = True #game in session
	ROBOT_TURN = True
	PLAYER_TURN = False
	
	PIECE_NAME_KEY = "sai2::cs225a::pieces::name";
	FINAL_PIECE_LOCATION_KEY_X = "sai2::cs225a::pieces::final_pos_x";
	FINAL_PIECE_LOCATION_KEY_Y = "sai2::cs225a::pieces::final_pos_y";
	INITIAL_PIECE_LOCATION_KEY_X = "sai2::cs225a::pieces::initial_pos_x";
	INITIAL_PIECE_LOCATION_KEY_Y = "sai2::cs225a::pieces::initial_pos_y";
	ROBOT_RUNNING_KEY = "sai2::cs225a::state::dynamics";
	READY_SCAN_BOARD_INITIAL_KEY = 'READY_SCAN_BOARD_INITIAL'
	READY_SCAN_BOARD_FINAL_KEY = 'READY_SCAN_BOARD_FINAL'
	PLAYER_MOVE_KEY = 'PLAYER_MOVE_KEY'
	
	redisClient.set(ROBOT_RUNNING_KEY, '0')
	redisClient.set(READY_SCAN_BOARD_INITIAL_KEY, '0')
	redisClient.set(READY_SCAN_BOARD_FINAL_KEY, '0')
	redisClient.set('PLAYER_MOVE_KEY', '0')
	USER_READY = 0
	READY_SCAN_BOARD_INITIAL = '0'
	READY_SCAN_BOARD_FINAL = '0'
	
	while(GAME_STATE):
	
		ROBOT_RUNNING = redisClient.get(ROBOT_RUNNING_KEY).decode('utf-8')
		
		#print(ROBOT_RUNNING)
	
		if (ROBOT_TURN and ROBOT_RUNNING == '0'):
			move = stockfish.get_best_move();
			stockfish.make_moves_from_current_position([move])
			initial_position = move[0:2]
			final_position = move[2:]
			
			CHESS_PIECE = chessboard_dictionary[initial_position]
			FINAL_POSITION = chessboard_grid_positions[final_position]
			INITIAL_POSITION = chessboard_grid_positions[initial_position]
			print("move1: " + str(move))
			
			if (chessboard_dictionary[final_position] != ""):
				redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(0.37))
				redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(0.3))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
				redisClient.set(ROBOT_RUNNING_KEY,'2')
				ROBOT_RUNNING = '2'
				while (ROBOT_RUNNING == '2'):
					ROBOT_RUNNING = redisClient.get(ROBOT_RUNNING_KEY).decode('utf-8')
			
							
			chessboard_dictionary[initial_position] = ""
			chessboard_dictionary[final_position] = CHESS_PIECE
									
			redisClient.set(PIECE_NAME_KEY, CHESS_PIECE)
			redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
			redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
			redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(INITIAL_POSITION[0]))
			redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(INITIAL_POSITION[1]))
			redisClient.set(ROBOT_RUNNING_KEY,'1')
									
			PLAYER_TURN = True
			ROBOT_TURN = False
		
		elif (PLAYER_TURN and ROBOT_RUNNING == '0'):
				
			while (READY_SCAN_BOARD_INITIAL != '1'):
				READY_SCAN_BOARD_INITIAL = input("Enter 1 to scan the board (pre_move): ")
				print(type(READY_SCAN_BOARD_INITIAL))
				
			redisClient.set(READY_SCAN_BOARD_INITIAL_KEY, READY_SCAN_BOARD_INITIAL)
					
			time.sleep(1)
			
			while (READY_SCAN_BOARD_FINAL != '1'):
				READY_SCAN_BOARD_FINAL = input("Enter 1 if you have completed your move: ") 
				
			redisClient.set(READY_SCAN_BOARD_FINAL_KEY, READY_SCAN_BOARD_FINAL)
			
			time.sleep(1)
			
			move = redisClient.get(PLAYER_MOVE_KEY).decode('utf-8')
			
			READY_SCAN_BOARD_INITIAL = '0'
			READY_SCAN_BOARD_FINAL = '0'
			
			redisClient.set('READY_SCAN_BOARD_INITIAL', READY_SCAN_BOARD_INITIAL)
			redisClient.set('READY_SCAN_BOARD_FINAL', READY_SCAN_BOARD_INITIAL)
			
			print("Your move is " + move + ".")
			
			correct_move = input("Input 1 if the displayed move is correct.")
			
			if (correct_move != "1"):
				move = stockfish.get_best_move();
				stockfish.make_moves_from_current_position([move])
				initial_position = move[0:2]
				final_position = move[2:]
				
				CHESS_PIECE = chessboard_dictionary[initial_position]
				FINAL_POSITION = chessboard_grid_positions[final_position]
				INITIAL_POSITION = chessboard_grid_positions[initial_position]
				print("move2: " + str(move))
				
				if (chessboard_dictionary[final_position] != ""):			
					redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(0.37))
					redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(0.3))
					redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
					redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
					redisClient.set(ROBOT_RUNNING_KEY,'2')
					ROBOT_RUNNING = '2'
					while (ROBOT_RUNNING == '2'):
						ROBOT_RUNNING = redisClient.get(ROBOT_RUNNING_KEY).decode('utf-8')
				
				chessboard_dictionary[initial_position] = ""
				chessboard_dictionary[final_position] = CHESS_PIECE
				
				redisClient.set(PIECE_NAME_KEY, CHESS_PIECE)
				redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
				redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(INITIAL_POSITION[0]))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(INITIAL_POSITION[1]))
				redisClient.set(ROBOT_RUNNING_KEY,'1')
										
				ROBOT_TURN = True
				PLAYER_TURN = False
			else:
				stockfish.make_moves_from_current_position([move])
				initial_position = move[0:2]
				final_position = move[2:]
				
				CHESS_PIECE = chessboard_dictionary[initial_position]
				FINAL_POSITION = chessboard_grid_positions[final_position]
				INITIAL_POSITION = chessboard_grid_positions[initial_position]
				print("move2: " + str(move))
				
				if (chessboard_dictionary[final_position] != ""):			
					redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(0.37))
					redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(0.3))
					redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
					redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
					redisClient.set(ROBOT_RUNNING_KEY,'2')
					ROBOT_RUNNING = '2'
					while (ROBOT_RUNNING == '2'):
						ROBOT_RUNNING = redisClient.get(ROBOT_RUNNING_KEY).decode('utf-8')
				
				chessboard_dictionary[initial_position] = ""
				chessboard_dictionary[final_position] = CHESS_PIECE
				
				redisClient.set(PIECE_NAME_KEY, CHESS_PIECE)
				redisClient.set(FINAL_PIECE_LOCATION_KEY_X,str(FINAL_POSITION[0]))
				redisClient.set(FINAL_PIECE_LOCATION_KEY_Y,str(FINAL_POSITION[1]))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_X,str(INITIAL_POSITION[0]))
				redisClient.set(INITIAL_PIECE_LOCATION_KEY_Y,str(INITIAL_POSITION[1]))
				redisClient.set(ROBOT_RUNNING_KEY,'1')
										
				ROBOT_TURN = True
				PLAYER_TURN = False
			
			
			
	
	
	
	
	
		

