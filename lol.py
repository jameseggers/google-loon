# Allow random spreading of balloons
	# try to keep balloons over a goal region if it is already over one, or
	# allow "natural spreading" to continue
# Make sure a balloon wont leave the map at every timestep

# Simulation control:
	# At every timetep of the simulation, we need to:
		# Update the "current_location" of every balloon based on the previous position and the relevant movement vector
		# Update the altitude of every balloon based on whether it is over a goal region or not
		# Output the changes in altitudes for every balloon


# START SIMULATION
import random

world_map = {R: , C: , A: }
balloons = {position: (x,y), altitude: A}
targetCells = [(x,y), (x,y),]
activeTargetCells = [(x,y), (x,y),]
wind = [[[(0,1), (-1,2)], [(0,1), (-1,2)] ], [[(0,1), (-1,2)], [(0,1), (-1,2)] ]]
output = file.open(output.txt, 'w')

for timestep in range(0, T):
	# update position of each balloon based on wind and position in previous timestep
	for balloon in balloons:
		update_position(balloon, wind, world_map)
	# decide on what altitude each balloon should next have
	for balloon in balloons:
		update_altitude(world_map, balloon, targetCells, activeTargetCells, wind, output)


def update_position(balloon, wind, world_map):
	balloon.position = balloon.position[0] + wind[position][0] , (balloon.position[1] + wind[position][1] ) % world_map.C

def update_altitude(world_map, balloon, targetCells, activeTargetCells, wind, output):
	if(exclusively_over_valid_region(balloon, targetCells, activeTargetCells)):
		keep_in_same_place(balloon, wind, targetCells, output)
	else:
		spread(balloon, wind, output)

def exclusively_over_valid_region(balloon, targetCells, activeTargetCells):
	if(balloon.position in activeTargetCells):
		return false
	elif(balloon.position in targetCells):
		activeTargetCells.add[balloon.position]
		return true
	else
		return false

def keep_in_same_place(balloon, wind, targetCells, output):
	# Use A* to find best path
	balloon.altitude += search_A_star(balloon, wind, targetCells)

def spread(balloon, wind, output):
	random_int = random.randint(-1, 1)
	while(not safe(balloon, random_int)):
		random_int = random.randint(-1, 1)
	balloon.altitude += random_int

def safe(balloon, random_int):


def search_A_star(balloon, map, targetCells, activeTargetCells, wind):
	node = new Node(balloon.position, 'root', 0, 0, heuristic(balloon, activeTargetCells))
	frontier = [node]
	explored = {}
	optimal_path = {node: node, value: 0}

	node_counter = 0
	while(frontier.size > 0 and node_counter < NODE_COUNTER_LIMIT):
		node = frontier.pop()
		node_counter += 1

		if(goal_test(node)):
			path_value = path_value(node)
			if(value > optimal_path.value):
				optimal_path.node = node
				optimal_path.value = path_value

		explored.add(node)

		children = generate_children(node)

		for child in children:
			if(child not in explored and child not in frontier):
				add_to_frontier(frontier, child)
			elif(child in frontier)
				if(better_option(child, frontier))
					replace(frontier, child)

	if(optimal_path.value == 0) return safe_move(balloon)
	return extract_move(optimal_path)

def heuristic(balloon, activeTargetCells):

def goal_test(node):

def path_value(node):	

def generate_children(node):

def add_to_frontier(frontier, child):

def better_option(child, frontier):

def replace(frontier, child):

def safe_move(balloon, wind):
	random_int = random.randint(-1, 1)
	while(not safe(balloon, random_int)):
		random_int = random.randint(-1, 1)
	return random_int	

def extract_move(optimal_path):
	node = optimal_path.node
	while(node.parent != 'root'):
		node = node.parent
		move = node.move
	return move


class Node:
	def __init__(self, start, parent, move, cost, heuristic)
		self.start = start
		self.parent = parent
		self.move = move
		self.cost = cost
		self.heursitc = heursitc

