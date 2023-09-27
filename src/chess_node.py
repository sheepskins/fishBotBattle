#!/usr/bin/env python3

import rospy
from stockfish import Stockfish
import chess
import chess.svg
from chess import engine
from fishbot_ros.srv import chess_service, chess_serviceRequest, chess_serviceResponse

fish_path = rospy.get_param("/stockfish_file_path")
fish_diff = rospy.get_param("/stockfish_difficulty", default=20)

board = chess.Board()
fish = engine.SimpleEngine.popen_uci(fish_path)
fish.configure({"Skill Level":fish_diff})
time_limit = chess.engine.Limit(time = 0.1)


def chessCallback(req):
    
    prev_move = chess.Move.from_uci(req.prev_move)
    board.push(prev_move)
    if not board.is_game_over():
        action = fish.play(board,time_limit)
        chess_move = str(action.move)
        if board.is_capture(action.move): 
            result = chess_move +',yes'+',no'
        elif board.is_castling(action.move): 
            result = chess_move +',no'+',yes'
        elif len(chess_move) == 5: 
            result = chess_move[:-1] +',no'+',no' + ',' + chess_move[-1]
        else: 
            result = chess_move +',no'+',no'
        return chess_serviceResponse(result)
    else: return chess_serviceResponse("")



if __name__ == "__main__":
    try:
        rospy.init_node("Chess_Engine")
        rospy.loginfo("Starting Chess Engine")
        serv = rospy.Service('chess_service', chess_service, chessCallback)
        rospy.loginfo("Chess Engine Ready")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass