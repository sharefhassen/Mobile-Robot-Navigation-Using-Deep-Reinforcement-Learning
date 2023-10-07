# Timeout setting:
timeout = 200 		# Number of steps within which the robot must grab each coke cans


# Object detector settings: 
min_confidence      = 0.7  		# Minimum confidance required to ASSUME the object detected to be a coke can
threshold_confidence=  0.85  	# Minimum confidence required to CONFIRM the object detected to be the coke can


# Summary frequency:
report_freq = 10 		# The frequency in which the summary of episodes must be displayed


# Gazebo related settings:
gazebo          = True 		# True/False - depending on weather or not of gazebo will be run
gazebo_marker   = True 		# True/False - weather or not the green/yellow circle goal marker must be displayed


# Verbose:
verbose = True      # To print necessary information on the screen


# Debug mode:
debug   = False 	# Turn on/off the debug mode.


# For testing environment :
test_weight_file = 'latest_model_120000_steps.zip' # file name of weights matrix to be tested. 


# For training environment:
n_steps = 150000        # Number of steps for which training has to be done
initial_weights = 'latest_model_120000_steps.zip'  # File name of the weight matrix from which training has to be initialized.
                        # You can keep it 'None' if random initialization is required