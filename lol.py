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

balloons = {position: (x,y), altitude: A}
targetCells = [(x,y), (x,y),]
wind = {}
output = file.open(output.txt, 'w')

for timestep in range(0, T):
	for balloon in balloons:
		update_position(balloon, wind)
	for balloon in balloons:
		update_altitude(balloon, wind, targetCells, output)

def update_position(balloon, wind):
	balloon.position = balloon.position[0] + wind[position][0] , (balloon.position[1] + wind[position][1] ) % C

def update_altitude(balloon, wind, targetCells, output):
	if(exclusively_over_valid_region(balloon, targetCells)):
		keep_in_same_place(balloon, wind, output)
	else:
		spread(balloon, wind, output)

def keep_in_same_place(balloon, wind, output):
	

def spread(balloon, wind, output):
	random_int = random.randint(-1, 1)
	while(not safe(balloon, random_int)):
		random_int = random.randint(-1, 1)
	balloon.altitude += random_int

def exclusively_over_valid_region(balloon, targetCells):

def safe(balloon, random_int):
