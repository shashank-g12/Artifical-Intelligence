import util 
from util import Belief, pdf 
from engine.const import Const

# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols) 
        self.transProb = util.loadTransProb() 
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    # 
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################
    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        # BEGIN_YOUR_CODE
        import math, random, collections
        def grid_update():
            n_r, n_c = self.belief.getNumRows(), self.belief.getNumCols()
            new_belief = util.Belief(n_r, n_c, 0)
            for tile_coord in self.particles:
                loc_x, loc_y = tile_coord[0], tile_coord[1]
                new_belief.setProb(loc_x, loc_y, self.particles[tile_coord])
            new_belief.normalize()
            self.belief = new_belief

        def resampling(weight_dict):
            sorted_weight = dict(reversed(list(sorted(weight_dict.items(), key=lambda item: item[1]))))
            weight_dict=dict(sorted_weight)
            weights, indx = [], []
            for loc in weight_dict:
                w = weight_dict[loc]
                weights.append(w)
                indx.append(loc)            
            sm = sum(weights)
            samples_range = random.uniform(0, sm)
            running_sum = 0.0
            for i in range(len(weights)):
                running_sum += weights[i]
                if running_sum > samples_range:
                    return indx[i]
        
        rows = len(self.belief.grid)
        cols = len(self.belief.grid[0])
        
        
        
        if isParked:
            self.prev_grid = self.belief.grid.copy()

            n_particles = int((rows*cols))
            self.trans_Prob_Dict = {}
            for tup in self.transProb:
                if not tup[0] in self.trans_Prob_Dict:
                    self.trans_Prob_Dict[tup[0]] = collections.Counter()
                self.trans_Prob_Dict[tup[0]][tup[1]] = self.transProb[(tup[0], tup[1])]

            self.particles = collections.Counter()
            potential_particles_loc = list(self.trans_Prob_Dict.keys())
            for i in range(n_particles):
                self.particles[potential_particles_loc[int(random.random() * len(potential_particles_loc))]] += 1
            grid_update()

            for tile_coord in self.particles:
                x, y = (posX - util.colToX(tile_coord[1])) ** 2, (posY - util.rowToY(tile_coord[0])) ** 2
                prior = self.particles[tile_coord]
                likelihood = util.pdf(observedDist, Const.SONAR_STD, math.sqrt(x + y))
                posterior = prior * likelihood
                self.particles[tile_coord] = posterior
            resampled_particles = collections.Counter()
            for i in range(n_particles):
                sample = resampling(dict(self.particles))
                resampled_particles[sample] += 1
            self.particles = resampled_particles
            grid_update()

            new_grid = []
            for i in range(rows):
                r_list = []
                for j in range(cols):
                    r_list.append(self.belief.grid[i][j] * self.prev_grid[i][j])
                new_grid.append(r_list)
            zero_check = 0
            for r in new_grid:
                zero_check+=sum(r)
            if zero_check !=0:
                self.belief.grid = new_grid.copy()
                self.belief.normalize()

        else:        
            self.prev_grid_dict = {}
            for i in range(len(self.belief.grid)):
                for j in range(len(self.belief.grid[i])):
                    self.prev_grid_dict[(i,j)] = self.belief.grid[i][j]

            
            trans_mat = {}
            for i in range(rows):
                for j in range(cols):
                    for k in range(rows):
                        for l in range(cols):
                            if (((i,j),(k,l))) in self.transProb:
                                trans_mat[((i,j),(k,l))] = self.transProb[((i,j),(k,l))]
                            else:
                                trans_mat[((i,j),(k,l))] = 0

            trans_grid = {}
            for i in range(rows):
                for j in range(cols):
                    next_prob = 0
                    for k in range(rows):
                        for l in range(cols):
                            next_prob += self.prev_grid_dict[(k,l)]*trans_mat[((k,l),(i,j))]
                    trans_grid[(i,j)] = next_prob


            n_particles = int((rows*cols))
            self.trans_Prob_Dict = {}
            for tup in self.transProb:
                if not tup[0] in self.trans_Prob_Dict:
                    self.trans_Prob_Dict[tup[0]] = collections.Counter()
                self.trans_Prob_Dict[tup[0]][tup[1]] = self.transProb[(tup[0], tup[1])]

            self.particles = collections.Counter()
            potential_particles_loc = list(self.trans_Prob_Dict.keys())
            for i in range(n_particles):
                self.particles[potential_particles_loc[int(random.random() * len(potential_particles_loc))]] += 1
            grid_update()

            for tile_coord in self.particles:
                x, y = (posX - util.colToX(tile_coord[1])) ** 2, (posY - util.rowToY(tile_coord[0])) ** 2
                prior = self.particles[tile_coord]
                likelihood = util.pdf(observedDist, Const.SONAR_STD, math.sqrt(x + y))
                posterior = prior * likelihood
                self.particles[tile_coord] = posterior
            resampled_particles = collections.Counter()
            for i in range(n_particles):
                sample = resampling(dict(self.particles))
                resampled_particles[sample] += 1
            self.particles = resampled_particles
            grid_update()
            new_grid = []
            for i in range(rows):
                r_list = []
                for j in range(cols):
                    r_list.append(self.belief.grid[i][j] * trans_grid[(i,j)])
                new_grid.append(r_list)
            zero_check = 0
            for r in new_grid:
                zero_check+=sum(r)
            if zero_check !=0:
                self.belief.grid = new_grid.copy()
                self.belief.normalize()
        
        return
  
    def getBelief(self) -> Belief:
        return self.belief

   