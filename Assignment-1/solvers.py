class SentenceCorrector(object):
    def __init__(self, cost_fn, conf_matrix):
        self.conf_matrix = conf_matrix
        self.cost_fn = cost_fn

        # You should keep updating following variable with best string so far.
        self.best_state = None  

    def search(self, start_state):
        """
        :param start_state: str Input string with spelling errors
        """
        # You should keep updating self.best_state with best string so far.
        self.best_state = start_state
        self.best_so_far = self.cost_fn(start_state)

        self.rev_map = {}
        for key,value in self.conf_matrix.items():
        	for v in value:
        		if v in self.rev_map:
        			self.rev_map[v].append(key)
        		else:
        			self.rev_map[v] = [key]

        step = 0
        while True:
        	best_loss, best_neighbour = self.best_successor(self.best_state)
        	if(best_loss<self.best_so_far):
        			self.best_so_far = best_loss
        			self.best_state = best_neighbour
        	else:
        		break
        	step+=1
        print(self.best_so_far)

    def best_successor(self, string):
    	best_loss = 1e10
    	best_state = ""
    	for idx in range(len(string)):
    		if string[idx] not in self.rev_map:
    			continue
    		for poss in self.rev_map[string[idx]]:
    			s = list(string)
    			s[idx] = poss
    			rs = "".join(s)
    			if(self.cost_fn(rs)<best_loss):
    				best_loss = self.cost_fn(rs)
    				best_state = rs
    	return best_loss,best_state
