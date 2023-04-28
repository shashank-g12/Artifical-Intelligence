import random
import sys
import copy

sys.path.append('../../')
import numpy as np
from typing import List, Tuple, Dict
from connect4.utils import get_pts, get_valid_actions, Integer
import multiprocessing as mp

class AIPlayer:
    def __init__(self, player_number: int, time: int):
        """
        :param player_number: Current player number
        :param time: Time per move (seconds)
        """
        self.player_number = player_number
        self.type = 'ai'
        self.player_string = 'Player {}:ai'.format(player_number)
        self.time = time
        # Do the rest of your implementation here

    def get_expectimax_move(self, state: Tuple[np.array, Dict[int, Integer]]) -> Tuple[int, bool]:
        """
        Given the current state of the board, return the next move based on
        the Expecti max algorithm.
        This will play against the random player, who chooses any valid move
        with equal probability
        :param state: Contains:environment
                        1. board
                            - a numpy array containing the state of the board using the following encoding:
                            - the board maintains its same two dimensions
                                - row 0 is the top of the board and so is the last row filled
                            - spaces that are unoccupied are marked as 0
                            - spaces that are occupied by player 1 have a 1 in them
                            - spaces that are occupied by player 2 have a 2 in them
                        2. Dictionary of int to Integer. It will tell the remaining popout moves given a player
        :return: action (0 based index of the column and if it is a popout move)
        """
        # Do the rest of your implementation here
        action = Tuple[int, bool]
        recv_end, send_end = mp.Pipe(False)
        p = mp.Process(target = self.get_action_expectimax, args = (state, send_end))
        p.start()
        
        if p.join(self.time-0.5) is None:
        	p.terminate()
        	while recv_end.poll():
        		action = recv_end.recv()
        #print(action)
        return action

    def get_action_expectimax(self, state, send_end):
    	
    	def result(player_number, state, move):
    		# returns state of a move by player
    		board = copy.deepcopy(state[0])
    		popout = copy.deepcopy(state[1])
    		if move[1]==True:
    			col = move[0]
    			for row in range(board.shape[0]-1,0,-1):
    				board[row][col] = board[row-1][col]
    			board[0][col]=0
    			popout[player_number].decrement()
    		else:
    			col = move[0]
    			update_row = -1
    			for row in range(board.shape[0]):
    				if row == board.shape[0]-1:
    					update_row = row
    				elif board[row][col]==0 and board[row+1][col]>0:
    					update_row = row
    				if update_row!=-1:
    					board[update_row][col] = player_number
    					break
    		return (board, popout)

    	def random(player_number, state, curr_depth, max_depth):
    		if (curr_depth >= max_depth): return self.eval(state[0])
    		
    		opp = 2 if player_number==1 else 1
    		valid_moves = get_valid_actions(player_number,state)
    		if (len(valid_moves)==0): return self.eval(state[0])
    		n = len(valid_moves)
    		value = 0
    		for move in valid_moves:
    			value += (1/n*(Max(opp, result(player_number, state, move), curr_depth+1, max_depth)))
    		return value

    	def Max(player_number, state, curr_depth, max_depth):
    		if(curr_depth >= max_depth): return self.eval(state[0]) #depth cuttoff test
    		opp = 2 if player_number==1 else 1
    		valid_moves = get_valid_actions(player_number, state)
    		if (len(valid_moves)==0): return self.eval(state[0]) #terminal test
    		value = -float("inf")
    		for move in valid_moves:
    			value = max(value, random(opp, result(player_number, state, move), curr_depth+1, max_depth))
    		return value

    	for max_depth in range(1,20):
    		opp = 2 if self.player_number == 1 else 1
    		valid_moves = get_valid_actions(self.player_number,state)
    		value = -float("inf")
    		best_move = Tuple[int, bool]
    		for move in valid_moves:
    			temp = random(opp, result(self.player_number, state, move), 1, max_depth)
    			if(temp>value):
    				value = temp
    				best_move = move
    		send_end.send(best_move)

    def get_intelligent_move(self, state: Tuple[np.array, Dict[int, Integer]]) -> Tuple[int, bool]:
        """
        Given the current state of the board, return the next move
        This will play against either itself or a human player
        :param state: Contains:
                        1. board
                            - a numpy array containing the state of the board using the following encoding:
                            - the board maintains its same two dimensions
                                - row 0 is the top of the board and so is the last row filled
                            - spaces that are unoccupied are marked as 0
                            - spaces that are occupied by player 1 have a 1 in them
                            - spaces that are occupied by player 2 have a 2 in them
                        2. Dictionary of int to Integer. It will tell the remaining popout moves given a player
        :return: action (0 based index of the column and if it is a popout move)
        """
        # Do the rest of your implementation here
        action = Tuple[int, bool]
        recv_end, send_end = mp.Pipe(False)
        p = mp.Process(target = self.get_action_intelligent, args = (state, send_end))
        p.start()
        
        if p.join(self.time-0.5) is None:
        	p.terminate()
        	while recv_end.poll():
        		action = recv_end.recv()
        #print(action)
        return action

    def get_action_intelligent(self, state, send_end):
    	
    	def result(player_number, state, move):
    		# returns state of a move by player
    		board = copy.deepcopy(state[0])
    		popout = copy.deepcopy(state[1])
    		if move[1]==True:
    			col = move[0]
    			for row in range(board.shape[0]-1,0,-1):
    				board[row][col] = board[row-1][col]
    			board[0][col]=0
    			popout[player_number].decrement()
    		else:
    			col = move[0]
    			update_row = -1
    			for row in range(board.shape[0]):
    				if row == board.shape[0]-1:
    					update_row = row
    				elif board[row][col]==0 and board[row+1][col]>0:
    					update_row = row
    				if update_row!=-1:
    					board[update_row][col] = player_number
    					break
    		return (board, popout)

    	def Min(player_number, state, curr_depth, max_depth, alpha, beta):
    		if (curr_depth >= max_depth): return self.eval(state[0])
    		valid_moves = get_valid_actions(player_number,state)
    		if (len(valid_moves)==0): return self.eval(state[0])
    		
    		opp = 2 if player_number==1 else 1
    		value = float("inf")
    		for move in valid_moves:
    			value = min(value, Max(opp, result(player_number, state, move), curr_depth+1, max_depth, alpha, beta))
    			if value <= alpha: return value
    			beta = min(beta, value)
    		return value

    	def Max(player_number, state, curr_depth, max_depth, alpha, beta):
    		if(curr_depth >= max_depth): return self.eval(state[0]) #depth cuttoff test
    		valid_moves = get_valid_actions(player_number, state)
    		if (len(valid_moves)==0): return self.eval(state[0]) #terminal test

    		opp = 2 if player_number==1 else 1
    		value = -float("inf")
    		for move in valid_moves:
    			value = max(value, Min(opp, result(player_number, state, move), curr_depth+1, max_depth, alpha, beta))
    			if value>=beta: return value
    			alpha = max(alpha, value)
    		return value

    	for max_depth in range(1,20):
    		opp = 2 if self.player_number == 1 else 1
    		valid_moves = get_valid_actions(self.player_number,state)
    		alpha = -float("inf")
    		beta = float("inf")
    		value = -float("inf")
    		best_move = Tuple[int, bool]
    		for move in valid_moves:
    			temp = Min(opp, result(self.player_number, state, move), 1, max_depth, alpha, beta)
    			if(temp>value):
    				value = temp
    				best_move = move
    			if value>=beta: break
    			alpha = max(alpha, value)
    		send_end.send(best_move)

    def eval(self, board:np.array):
    	ai = self.player_number
    	op = 2 if self.player_number==1 else 1
    	score_ai = get_pts(ai,board)
    	score_op = get_pts(op,board)
    	window = [3,4]
    	weights = [0, 0, 3, 7, 20]
    	m,n = board.shape[0],board.shape[1]
    	depth = []

    	for j in range(n):
    		column = board[:,j]
    		for i in range(m):
    			if column[i]>0:
    				depth.append(m-i-1)
    				break
    		if column[m-1]==0:
    			depth.append(-1)

    	for win in window:
    		for i in range(m):
    			for j in range(n):
    				if j+win<=n:
    					temp = board[i,j:j+win]
    					if ai not in temp and 0 in temp and op in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j+k]))
    						num = weights[win]/(win-np.count_nonzero(temp==op))
    						score_op += (num/denom)
    					if op not in temp and 0 in temp and ai in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j+k]))
    						num = weights[win]/(win - np.count_nonzero(temp==ai))
    						score_ai += (num/denom) 
    				if i+win<=m:
    					temp = board[i:i+win,j]
    					if ai not in temp and 0 in temp and op in temp:
    						score_op += weights[win]*(np.count_nonzero(temp==op)/win)*1.25
    					if op not in temp and 0 in temp and ai in temp:
    						score_ai += weights[win]*(np.count_nonzero(temp==ai)/win)*1.25
    				if i+win<=m and j+win<=n:
    					temp = []
    					for k in range(win):
    						temp.append(board[i+k, j+k])
    					temp = np.array(temp)
    					if ai not in temp and 0 in temp and op in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j+k]))
    						num = weights[win]*(win - np.count_nonzero(temp==op))
    						score_op += (num/denom)
    					if op not in temp and 0 in temp and ai in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j+k]))
    						num = weights[win]*(win - np.count_nonzero(temp==ai))
    						score_ai += (num/denom)
    				if i+win<=m and j+1>=win:
    					temp = []
    					for k in range(win):
    						temp.append(board[i+k, j-k])
    					temp = np.array(temp)
    					if ai not in temp and 0 in temp and op in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j-k]))
    						num = weights[win]*(win - np.count_nonzero(temp==op))
    						score_op += (num/denom)
    					if op not in temp and 0 in temp and ai in temp:
    						denom = 1
    						for k in range(len(temp)):
    							if temp[k]==0:
    								denom *= (2**(m-i-1-depth[j-k]))
    						num = weights[win]*(win - np.count_nonzero(temp==ai))
    						score_ai += (num/denom)
    	return score_ai-score_op