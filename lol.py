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
# from main import LoonBalloons

# data = LoonBalloons().process()

T = 1000
# cell_radius = data.V;
cell_radius = 5
# world_map = {R: data['R'], C: data['C'], A: data['A']}
world_map = {R: 10, C: 10, A: 10}
balloons = [{position: (0,0), altitude: 0, balloon_id: 0}, {position: (0,0), altitude: 0, balloon_id: 1}, {position: (0,0), altitude: 0, balloon_id: 2}]
# targetCells = [(x,y), (x,y),]
targetCells = [(3,2), (9,4)]
activeTargetCells = { }
active_balloons = { }
wind = [[[(0,1), (-1,2)], [(0,1), (-1,2)] ], [[(0,1), (-1,2)], [(0,1), (-1,2)] ]]
output = file.open(output.txt, 'w')

w, h = world_map.R, world_map.C
target_cell_map = [[0 for x in range(w)] for y in range(h)] 

make_target_cell_map(targetCells, cell_radius, target_cell_map)

for timestep in range(0, T):
	# update position of each balloon based on wind and position in previous timestep
	for balloon, balloon_id in balloons:
		update_position(balloon, wind, world_map)
	# decide on what altitude each balloon should next have
	for balloon, balloon_id in balloons:
		update_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output)

def make_target_cell_map(targetCells, cell_radius, target_cell_map, world_map):
	for cell in targetCells:
		for i in range(cell[0] - cell_radius, cell[0] + cell_radius):
			for j in range(cell[1] - cell_radius, cell[1] + cell_radius):
				if(in_range((i,j), cell)):
					target_cell_map[i][j] = cell

def update_position(balloon, wind, world_map):
	balloon.position = balloon.position[0] + wind[balloon.altitude][balloon.position[0]][balloon.position[1]][0] , (balloon.position[1] + wind[balloon.altitude][balloon.position[0]][balloon.position[1]][1] ) % world_map.C

def update_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output):
	if(exclusively_over_valid_region(balloon, balloon_id, activeTargetCells, target_cell_map)):
		keep_in_same_place(balloon, wind, activeTargetCells, target_cell_map)
	else:
		spread(balloon, wind, output)

def exclusively_over_valid_region(balloon, balloon_id, activeTargetCells, target_cell_map):
	target = target_cell_map[balloon.position[0]][balloon.position[1]]

	if(target == 0):
		if(balloon_id in active_balloons):
			# remove from activeTargetCells
			old_target = active_balloons.pop(balloon_id)
			# remove from active_balloons
			del activeTargetCells[old_target]
		return false
	elif(target in activeTargetCells):
		if(balloon_id in active_balloons):
			if(active_balloons.balloon_id == target):
				return true
		else: false
	else:
		activeTargetCells[target] = balloon_id
		active_balloons[balloon_id] = target
		return true

def keep_in_same_place(balloon, wind, activeTargetCells, target_cell_map):
	# Use A* to find best path
	balloon.altitude += search_A_star(balloon, activeTargetCells, wind, target_cell_map)

def spread(balloon, wind, output):
	random_int = random.randint(-1, 1)
	while(not safe(balloon, random_int)):
		random_int = random.randint(-1, 1)
	balloon.altitude += random_int

def safe(balloon, random_int):
	if(balloon.altitude + random_int > world_map.A or balloon.altitude + random_int < 1) return false
	return true

def search_A_star(balloon, activeTargetCells, wind, target_cell_map):
	target = target_cell_map[balloon.position[0]][balloon.position[1]]
	node = new Node(balloon.position, balloon.altitude, 'root', 0, 0, heuristic(balloon.position, target_cell_map))
	frontier = [node]
	explored = {}
	optimal_path = {node: node, value: 0}

	node_counter = 0
	while(frontier.size > 0 and node_counter < NODE_COUNTER_LIMIT):
		node = frontier.pop()
		node_counter += 1

		if(goal_test(node, target, target_cell_map)):
			path_value = path_value(node, target, target_cell_map)
			if(value > optimal_path.value):
				optimal_path.node = node
				optimal_path.value = path_value

		explored.add(node)

		children = generate_children(node, target, wind)

		for child in children:
			if(child not in explored and child not in frontier):
				add_to_frontier(frontier, child)
			elif(child in frontier)
					replace_if_better(child, frontier)

	if(optimal_path.value == 0) return safe_move(balloon)
	return extract_move(optimal_path)

def in_range(position, target_cell):
	if(((abs(position[0]) - abs(target_cell[0]))^2 + (abs(position[1]) - abs(target_cell[1]))^2) <= cell_radius) return true
	return false

def heuristic(position, target_cell_map):
	# Use SLD heuristic
	target = target_cell_map[position[0]][position[1]]
	return sqrt((position[0] - target[0])^2 + (position[1] - target[1])^2)


def goal_test(node, target, target_cell_map):
	if( target_cell_map[node.position[0]][node.position[1]] == target) return true
	return false

def path_value(node, target, target_cell_map):	
	value = 1
	while(node.parent != 'root'):
		node = node.parent
		if(goal_test(node, target, target_cell_map)):
			value += 1
	return value

def generate_children(node, target, wind):
	moves = [-1, 0, 1] # a move is an altitude change
	for(move in moves):
		# Check if safe move
		new_position = node.position[0] + wind[node.altitude][node.position[0]][node.position[1]][0] , (node.position[1] + wind[node.altitude][node.position[0]][node.position[1]][1] ) % world_map.C
		cost = sqrt((new_position[0] - target[0])^2 + (new_position[1] - target[1])^2)
		child = new Node(new_position, node.altitude + move, node, move, node.cost + cost, heuristic(node.position, target_cell_map))
		children.add(child)
	return children

def add_to_frontier(frontier, child):
	for index, node in enumerate(frontier):
		if(child.cost + child.heuristic < node.cost + node.heuristic):
			frontier.add(index, child)

def replace_if_better(child, frontier):
	for index, node in enumerate(frontier):
		if(node.position == child.position):
			if(node.cost + node.heuristic > child.cost + child.heuristic):
				frontier[index] = child
				return
	return

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
	def __init__(self, position, altitude, parent, move, cost, heuristic)
		self.position = position
		self.altitude = altitude
		self.parent = parent
		self.move = move
		self.cost = cost
		self.heursitc = heursitc
