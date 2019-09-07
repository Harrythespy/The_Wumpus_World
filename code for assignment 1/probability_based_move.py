
#----- IFN680 Assignment 1 -----------------------------------------------#
#  The Wumpus World: a probability based agent
#
#  Implementation of two functions
#   1. PitWumpus_probability_distribution()
#   2. next_room_prob()
#
#    Student no: n10298851,
#    Student name: Po-Ying Shen,
#
#-------------------------------------------------------------------------#
from random import *
from AIMA.logic import *
from AIMA.utils import *
from AIMA.probability import *
from tkinter import messagebox
from logic_based_move import *

#--------------------------------------------------------------------------------------------------------------
#
#  The following two functions are to be developed by you. They are functions in class Robot. If you need,
#  you can add more functions in this file. In this case, you need to link these functions at the beginning
#  of class Robot in the main program file the_wumpus_world.py.
#
#--------------------------------------------------------------------------------------------------------------
#   Function 1. PitWumpus_probability_distribution(self, width, height)
#
# For this assignment, we treat a pit and the wumpus equally. Each room has two states: 'empty' or 'containing a pit or the wumpus'.
# A Boolean variable to represent each room: 'True' means the room contains a pit/wumpus, 'False' means the room is empty.
#
# For a cave with n columns and m rows, there are totally n*m rooms, i.e., we have n*m Boolean variables to represent the rooms.
# A configuration of pits/wumpus in the cave is an event of these variables.
#
# The function PitWumpus_probability_distribution() below is to construct the joint probability distribution of all possible
# pits/wumpus configurations in a given cave, two parameters
#
# width : the number of columns in the cave
# height: the number of rows in the cave
#
# In this function, you need to create an object of JointProbDist to store the joint probability distribution and  
# return the object. The object will be used by your function next_room_prob() to calculate the required probabilities.
#
# This function will be called in the constructor of class Robot in the main program the_wumpus_world.py to construct the
# joint probability distribution object. Your function next_room_prob() will need to use the joint probability distribution
# to calculate the required conditional probabilities.
#
def PitWumpus_probability_distribution(self, width, height): 
    # Create a list of variable names to represent the rooms. 
    # A string '(i,j)' is used as a variable name to represent a room at (i, j)
    self.PW_variables = [] 
    for column in range(1, width + 1):
        for row in range(1, height + 1):
            self.PW_variables  = self.PW_variables  + ['(%d,%d)'%(column,row)]

    #--------- Add your code here -------------------------------------------------------------------
    # define the number of probability for each status
    pitWumpus = 0.2
    nonPitWumpus = 1 - pitWumpus

    var_values = {each: [T, F] for each in self.PW_variables}
    Pr_room = JointProbDist(self.PW_variables, var_values)
    # known_PW = self.observation_pits(self.visited_rooms)
    # get all variables
    events = all_events_jpd(self.PW_variables, Pr_room, {})

    for event in events:
        # initial values
        prob = 1
        for (var, val) in event.items():
            prob = prob * pitWumpus if val == T else prob * nonPitWumpus
        Pr_room[event] = prob
    # print(Pr_room.show_approx())
    return Pr_room
#---------------------------------------------------------------------------------------------------
#   Function 2. next_room_prob(self, x, y)
#
#  The parameters, (x, y), are the robot's current position in the cave environment.
#  x: column
#  y: row
#
#  This function returns a room location (column,row) for the robot to go.
#  There are three cases:
#
#    1. Firstly, you can call the function next_room() of the logic-based agent to find a
#       safe room. If there is a safe room, return the location (column,row) of the safe room.
#    2. If there is no safe room, this function needs to choose a room whose probability of containing
#       a pit/wumpus is lower than the pre-specified probability threshold, then return the location of
#       that room.
#    3. If the probabilities of all the surrounding rooms are not lower than the pre-specified probability
#       threshold, return (0,0).
#---------------------------------------------------------------------------------------------------

def next_room_prob(self, x, y):

    # first check if the variable is valid or not
    if self.jdP_PWs.is_valid():
        surrounding = self.cave.getsurrounding(x,y)
        # To choose the room to access, first have to get R_Query
        # R_Query = Surrounding Room - R_known
        r_Query = R_Query(self,surrounding)  # R_query stores all the adjacent rooms of current position for the agent
        # print(r_Query)
        # The agent have to decide which room to go
        # By calculating the possibility of danger of each room
        # Then go to the one with lower dangers
        PqTrues = dict()
        R_others = set() # fix here
        for each_room in self.PW_variables:
            test_room = int(each_room[1]), int(each_room[3])
            if test_room not in self.visited_rooms and test_room not in r_Query:
                R_others.add(test_room)

        for room in r_Query:
            # Firstly, need to find R_others which represents the rooms that haven't accessed
            # And the rooms which are not adjacent for the agent
            # Calculate R_unknown
            R_unknown = list(R_others.union(r_Query - {room}))
            R_unknown_str = tupleConverted(R_unknown)
            known_BS = self.observation_breeze_stench(self.visited_rooms)
            known_PW = self.observation_pits(self.visited_rooms)
            # Calculate the possibility of having pits or Wumpus in the next room
            # Accumulate all of the possibility that Pq is true
            known_PW['({},{})'.format(room[0], room[1])] = True

            Pq_true_events = all_events_jpd(R_unknown_str, self.jdP_PWs, known_PW)
            PqTrue = 0 # Accumulate the truth of having Wumpus in JPD
            for event in Pq_true_events:
                consistency = self.consistent(known_BS, event)
                PqTrue += self.jdP_PWs[event] * consistency

            # Accumulate all of the possibility that Pq is false
            known_PW['({},{})'.format(room[0], room[1])] = False
            Pq_false_events = all_events_jpd(R_unknown_str, self.jdP_PWs, known_PW)
            PqFalse = 0 # Accumulate the truth of having no Wumpus in JPD
            for event in Pq_false_events:
                consistency = self.consistent(known_BS, event)
                if consistency == 0:
                    PqFalse += self.jdP_PWs[event]

            print('Pq true:',PqTrue)
            print('Pq False:', PqFalse)
            # Pq_true normalised = Pq_true / Pq_false + Pq_true
            Pq_true_normalised = PqTrue / (PqFalse + PqTrue)
            # Using dictionary to store coordinate and the truth of possibility
            PqTrues[room] = Pq_true_normalised
        print(PqTrues)
        for cor_A in PqTrues:
            lowest_pos = (0,0)
            for cor_B in PqTrues:
                if PqTrues[cor_A] <= PqTrues[cor_B]:
                    if PqTrues[cor_A] <= self.max_pit_probability:
                        lowest_pos = cor_A
            print(lowest_pos)
            return lowest_pos
    return (0,0)
#---------------------------------------------------------------------------------------------------
def R_Query(self, surrounding):
    r_qurey = set()
    for room in surrounding:
        if room not in self.visited_rooms:
            r_qurey.add(room)
    return r_qurey

def tupleConverted(tuples):
    tuple_to_str = []
    for t in tuples:
        tuple_to_str.append('({},{})'.format(t[0], t[1]))
    return tuple_to_str

####################################################################################################