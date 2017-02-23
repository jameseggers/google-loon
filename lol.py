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
import math
from main import LoonBalloons

data = LoonBalloons().process()

def first_search_A_star(balloon, opt_region, wind):
	print("1st A*!")
	target = opt_region
	node = Node(balloon['position'], balloon['altitude'], 'root', 0, 0, first_heuristic(balloon['position'], target))
	frontier = [node]
	explored = []
	optimal_path = {'node': node, 'value': 0}
	tolerance = 10

	node_counter = 0
	while(len(frontier) > 0 and node_counter < NODE_COUNTER_LIMIT):
		print(node.position)
		print(node.altitude)
		print("\n")
		node = frontier.pop()
		node_counter += 1

		if(first_goal_test(node, target, tolerance)):
			print("Pass!")
			first_moves = first_extract_move(node)
			return first_moves
		print("failed goal test")
		explored.append(node.position)

		children = first_generate_children(node, target, wind)

		for child in children:
			if(child.position not in explored and not_in_frontier(child, frontier)):
				add_to_frontier(frontier, child)

def not_in_frontier(child, frontier):
	for node in frontier:
		if(node.position == child.position): return False
	else: return True

def make_target_cell_map(targetCells, cell_radius, target_cell_map):
	print("Making target cell map")
	for cell in targetCells:
		for i in range(cell[0] - cell_radius, cell[0] + cell_radius):
			for j in range(cell[1] - cell_radius, cell[1] + cell_radius):
				if(in_range((i,j), cell)):
					if(i >= 0 and j >= 0 and i < h and j < w):
						target_cell_map[i][j] = (int(cell[0]), int(cell[1]))

def update_position(balloon, wind, world_map):
	print("\n\nUpdating position...\n\nBefore:")
	print(balloon['position'])
	new_position = balloon['position'][0] + int(wind[balloon['altitude']][0][0][balloon['position'][0]][0]) , (balloon['position'][1] + int(wind[balloon['altitude']][0][0][balloon['position'][0]][1])) % world_map['C']
	# if(balloon['position'][0] >= world_map['R'] or balloon['position'][1] >= world_map['C'] or balloon['position'][0] < world_map['R']):
	# 	# print("Out of range!")
	# 	return
	balloon['position'] = new_position
	print("After:")
	print(balloon['position'])

def update_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output):
    print("\n\nUpdating ALTITUDE for ballon " + str(balloon_id))
    # print(balloon)
    if(exclusively_over_valid_region(balloon, balloon_id, activeTargetCells, target_cell_map)):
    	print("Exclusive!")
        keep_in_same_place(balloon, wind, activeTargetCells, target_cell_map)
    else:
    	print("spreading!")
        spread(balloon, wind, output)

def set_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output, move):
	print("SETTING altitude for balloon")
	balloon['altitude'] = balloon['altitude'] + move

def exclusively_over_valid_region(balloon, balloon_id, activeTargetCells, target_cell_map):
	if(balloon['position'][0] >= world_map['C'] or balloon['position'][1] >= world_map['R']):
		# print("Out of range!")
		return False
	target = target_cell_map[balloon['position'][0]][balloon['position'][1]]
	# print("Target: " + str(target))

	if(target == 0):
		# print("Not over a valid region")
		if(balloon_id in active_balloons):
			# remove from activeTargetCells
			old_target = active_balloons.pop(balloon_id)
			# remove from active_balloons
			del activeTargetCells[old_target]
		# print("Returning false...")
		return False
	elif(target in activeTargetCells):
		print("Over a valid region: is exclusive?")
		if(balloon_id in active_balloons):
			# print("balloon is active!")
			if(active_balloons[balloon_id] == target):
				# print("Yes!")
				return True
			# print("No")
			return False
		else: 
			# print("No")
			return False
	else:
		activeTargetCells[target] = balloon_id
		active_balloons[balloon_id] = target
		# print("Yes!")
		return True

def keep_in_same_place(balloon, wind, activeTargetCells, target_cell_map):
	# Use A* to find best path
	balloon['altitude'] += search_A_star(balloon, activeTargetCells, wind, target_cell_map)

def spread(balloon, wind, output):
	random_int = random.randint(-1, 1)
	while(not safe(balloon, random_int)):
		# print("Still not safe")
		random_int = random.randint(-1, 1)
	balloon['altitude'] += random_int

def safe(balloon, random_int):
	if(balloon['altitude'] + random_int >= (world_map['A'] -1) or balloon['altitude'] + random_int < 1): return False
	new_position = balloon['position'][0] + int(wind[balloon['altitude'] + random_int][0][0][balloon['position'][0]][0]) , (balloon['position'][1] + int(wind[balloon['altitude'] + random_int][0][0][balloon['position'][0]][1])) % world_map['C']
	# print("New position will be: " + str(new_position))
	if(new_position[0] >= world_map['R'] or new_position[1] >= world_map['C'] or new_position[0] < 1 or new_position[1] < 1): return False
	return True

def search_A_star(balloon, activeTargetCells, wind, target_cell_map):
	print("A* search")
	target = target_cell_map[balloon['position'][0]][balloon['position'][1]]
	node = Node(balloon['position'], balloon['altitude'], 'root', 0, 0, heuristic(balloon['position'], target_cell_map))
	frontier = [node]
	explored = []
	optimal_path = {'node': node, 'value': 0}

	node_counter = 0
	while(len(frontier) > 0 and node_counter < NODE_COUNTER_LIMIT):
		node = frontier.pop()
		node_counter += 1

		if(goal_test(node, target, target_cell_map)):
			pathvalue = path_value(node, target, target_cell_map)
			if(pathvalue > optimal_path['value']):
				optimal_path['node'] = node
				optimal_path['value'] = pathvalue

		explored.append(node)

		children = generate_children(node, target, wind)

		for child in children:
			if(child not in explored and child not in frontier):
				add_to_frontier(frontier, child)
			elif(child in frontier):
					replace_if_better(child, frontier)

	if(optimal_path['value'] == 0): return safe_move(balloon, wind)
	move = 1
	extract_move(optimal_path, move)
	return move

def in_range(position, target_cell):
	if(((abs(position[0]) - abs(target_cell[0]))^2 + (abs(position[1]) - abs(target_cell[1]))^2) <= cell_radius): 
		# print("In range!")
		return True
	return False

def heuristic(position, target_cell_map):
	# Use SLD heuristic
	if(position[0] + 1 >= world_map['R'] or position[1] + 1 >= world_map['C']): return 1000000000
	target = target_cell_map[position[1]][position[0]]
	if(target == 0): return 1000000000
	return math.sqrt(abs((position[0] - target[0])^2 + (position[1] - target[1])^2))

def first_heuristic(position, target):
	# Use SLD heuristic
	return math.sqrt(abs((position[0] - target[0])^2 + (position[1] - target[1])^2))	


def goal_test(node, target, target_cell_map):
	if(node.position[0] + 1 >= world_map['C'] or node.position[1] + 1 >= world_map['R']): return False
	if( target_cell_map[node.position[0]][node.position[1]] == 0): return False
	if( target_cell_map[node.position[0]][node.position[1]] == target): return True
	return False

def first_goal_test(node, target, tolerance):
	if(math.sqrt(abs((node.position[0] - target[0])^2 + (node.position[1] - target[1])^2)) < tolerance): return True
	return False

def path_value(node, target, target_cell_map):	
	value = 1
	while(node.parent != 'root'):
		node = node.parent
		if(goal_test(node, target, target_cell_map)):
			value += 1
	return value

def generate_children(node, target, wind):
	print("Generating kids")
	moves = [-1, 0, 1] # a move is an altitude change
	children = []
	for move in moves:
		# Check if safe move
		if(node.altitude + move + 1 >= world_map['A'] or node.altitude + move < 1): continue
		new_position = node.position[0] + int(wind[node.altitude + move][node.position[0]][0][node.position[1]][0]), (node.position[1] + int(wind[node.altitude + move][node.position[0]][0][node.position[1]][1])) % world_map['C']
		
		cost = math.sqrt(abs((new_position[0] - target[0])^2 + (new_position[1] - target[1])^2))
		child = Node(new_position, node.altitude + move, node, move, node.cost + cost, heuristic(node.position, target_cell_map))
		children.append(child)
	return children

def first_generate_children(node, target, wind):
	moves = [-1, 0, 1] # a move is an altitude change
	children = []
	for move in moves:
		# Check if safe move
		if(node.altitude + move  + 1 >= world_map['A'] or node.altitude + move < 1): continue
		new_position = node.position[0] + int(wind[node.altitude + move][node.position[0]][0][node.position[1]][0]), (node.position[1] + int(wind[node.altitude + move][node.position[0]][0][node.position[1]][1])) % world_map['C']
		
		cost = math.sqrt(abs((new_position[0] - target[0])^2 + (new_position[1] - target[1])^2))
		child = Node(new_position, node.altitude + move, node, move, node.cost + cost, first_heuristic(node.position, target))
		children.append(child)
	return children	

def add_to_frontier(frontier, child):
	for index, node in enumerate(frontier):
		if(child.cost + child.heuristic < node.cost + node.heuristic):
			frontier.insert(index, child)
			return
	frontier.append(child)

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

def extract_move(optimal_path, move):
	node = optimal_path['node']
	print("Move: " + str(move))
	while(node.parent != 'root'):
		node = node.parent
		move = node.move
		print("Move: " + str(move))
	return

def first_extract_move(node):
	moves = []
	while(node.parent != 'root'):
		node = node.parent
		move = node.move
		moves.append(move)
	return moves



class Node:
	def __init__(self, position, altitude, parent, move, cost, heuristic):
		self.position = position
		self.altitude = altitude
		self.parent = parent
		self.move = move
		self.cost = cost
		self.heuristic = heuristic

T = data['T']
# cell_radius = 5
cell_radius = data['V']
world_map = {'R': data['R'], 'C': data['C'], 'A': data['A']}
# world_map = data['world_map']
# balloons = [{'position': (4,4), 'altitude': 0, 'balloon_id': 0}, {'position': (4,4), 'altitude': 0, 'balloon_id': 1}, {'position': (4,4), 'altitude': 0, 'balloon_id': 2}]
B = data['B']
balloons = []

for i in range(B):
	balloons.append({'position': data['startingCell'], 'altitude': 0, 'balloon_id': i})

# targetCells = [(3,2), (9,4)]
targetCells = data['targetCells']

activeTargetCells = { }
active_balloons = { }
wind = data['windVectors']
# wind = [[[(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)], 
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)]],

# [[(3,-1), (1,2), (1,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (-1,3), (1,-3)], 
# [(3,1), (1,2), (2,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(4,1), (1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,1), (1,0), (1,3), (1,-3)],
# [(2,-1), (-1,2), (3,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (1,-3)],
# [(1,-1), (-1,0), (1,0), (3,-2), (-0,2), (1,-1), (-1,1), (2,0), (1,3), (1,-3)],
# [(1,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(-3,1), (1,2), (5,1), (3,-2), (0,-2), (-1,1), (-1,1), (0,0), (1,3), (1,-3)],
# [(0,-1), (-1,0), (-1,1), (3,-2), (0,2), (1,-1), (-1,0), (1,0), (-1,-3), (-1,-3)],
# [(0,1), (-2,2), (-1,1), (3,-2), (-0,2), (-1,-1), (-1,0), (1,0), (1,3), (1,-3)],
# [(-2,1), (-1,2), (-1,1), (3,-2), (0,2), (0,-1), (-1,1), (1,0), (1,-3), (1,-3)]],

# [[(3,-1), (1,2), (1,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (-1,3), (1,-3)], 
# [(3,1), (1,2), (2,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(4,1), (1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,1), (1,0), (1,3), (1,-3)],
# [(2,-1), (-1,2), (3,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (1,-3)],
# [(1,-1), (-1,0), (1,0), (3,-2), (-0,2), (1,-1), (-1,1), (2,0), (1,3), (1,-3)],
# [(1,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(-3,1), (1,2), (5,1), (3,-2), (0,-2), (-1,1), (-1,1), (0,0), (1,3), (1,-3)],
# [(0,-1), (-1,0), (-1,1), (3,-2), (0,2), (1,-1), (-1,0), (1,0), (-1,-3), (-1,-3)],
# [(0,1), (-2,2), (-1,1), (3,-2), (-0,2), (-1,-1), (-1,0), (1,0), (1,3), (1,-3)],
# [(-2,1), (-1,2), (-1,1), (3,-2), (0,2), (0,-1), (-1,1), (1,0), (1,-3), (1,-3)]],

# [[(3,-1), (1,2), (1,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (-1,3), (1,-3)], 
# [(3,1), (1,2), (2,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(4,1), (1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,1), (1,0), (1,3), (1,-3)],
# [(2,-1), (-1,2), (3,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (1,-3)],
# [(1,-1), (-1,0), (1,0), (3,-2), (-0,2), (1,-1), (-1,1), (2,0), (1,3), (1,-3)],
# [(1,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(-3,1), (1,2), (5,1), (3,-2), (0,-2), (-1,1), (-1,1), (0,0), (1,3), (1,-3)],
# [(0,-1), (-1,0), (-1,1), (3,-2), (0,2), (1,-1), (-1,0), (1,0), (-1,-3), (-1,-3)],
# [(0,1), (-2,2), (-1,1), (3,-2), (-0,2), (-1,-1), (-1,0), (1,0), (1,3), (1,-3)],
# [(-2,1), (-1,2), (-1,1), (3,-2), (0,2), (0,-1), (-1,1), (1,0), (1,-3), (1,-3)]],

# [[(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)], 
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)],
# [(0,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,3), (-1,-3)]],

# [[(3,-1), (1,2), (1,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (-1,3), (1,-3)], 
# [(3,1), (1,2), (2,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(4,1), (1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,1), (1,0), (1,3), (1,-3)],
# [(2,-1), (-1,2), (3,0), (3,-2), (0,2), (1,-1), (-1,0), (0,0), (1,-3), (1,-3)],
# [(1,-1), (-1,0), (1,0), (3,-2), (-0,2), (1,-1), (-1,1), (2,0), (1,3), (1,-3)],
# [(1,1), (-1,2), (1,1), (3,-2), (0,2), (-1,-1), (-1,0), (0,0), (1,-3), (-1,-3)],
# [(-3,1), (1,2), (5,1), (3,-2), (0,-2), (-1,1), (-1,1), (0,0), (1,3), (1,-3)],
# [(0,-1), (-1,0), (-1,1), (3,-2), (0,2), (1,-1), (-1,0), (1,0), (-1,-3), (-1,-3)],
# [(0,1), (-2,2), (-1,1), (3,-2), (-0,2), (-1,-1), (-1,0), (1,0), (1,3), (1,-3)],
# [(-2,1), (-1,2), (-1,1), (3,-2), (0,2), (0,-1), (-1,1), (1,0), (1,-3), (1,-3)]]]


output = open('output', 'w')

NODE_COUNTER_LIMIT = 100
points = 0

w, h = world_map['R'], world_map['C']
print("R: " + str(w))
print("C: " + str(h))
target_cell_map = [[0 for x in range(w)] for y in range(h)] 

make_target_cell_map(targetCells, cell_radius, target_cell_map)
# print("TARGET CELL MAP:")
# print(target_cell_map)
# print(len(target_cell_map))

first_moves = first_search_A_star(balloons[0], (50, 150), wind)
first_moves.reverse()
print(first_moves)

for timestep in range(0, T):
	print("\n\nTimestep:\t" + str(timestep) + "\n\n")
	# update position of each balloon based on wind and position in previous timestep
	print(balloons[0]['position'])
	print(balloons[0]['altitude'])
	if(timestep < len(first_moves)):
		for balloon in balloons:
			balloon_id = balloon['balloon_id']
			update_position(balloon, wind, world_map)
			set_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output, first_moves[timestep])			
			# print(balloons[0]['position'])
	else:
		for balloon in balloons:
			update_position(balloon, wind, world_map)
		# decide on what altitude each balloon should next have
		for balloon in balloons:
			balloon_id = balloon['balloon_id']
			update_altitude(world_map, balloon, balloon_id, activeTargetCells, target_cell_map, wind, output)

		for balloon in balloons:
			if(exclusively_over_valid_region(balloon, balloon['balloon_id'], activeTargetCells, target_cell_map)):
				print("hurrah! Pass go and take a point!")
				points += 1
		print("Timestep:\t" + str(timestep))
		print("Points:  \t" + str(points) + "\n")

