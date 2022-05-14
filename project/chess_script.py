import chess
import redis



if __name__ == "__main__":
	redisClient = redis.Redis();
	print(redisClient);
	board = chess.Board()
	print(board);
