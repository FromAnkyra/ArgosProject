#!/bin/sh

runs=1 # First command line argument is the number of runs
time=3000

config="experiments/foraging.argos" # Path to your ARGoS configuration file

for random_seed in $(seq $runs) # Use a different random seed for each run
do
	temp="experiments/temp_${random_seed}.argos" # Create a temporary ARGoS configuration file name based on the random seed

	cp $config $temp # Make a copy of the original ARGoS configuration file, so it doesn't get messed up by the following commands
	
	sed -i "s/random_seed='[0-9]'*/random_seed='$random_seed'/" $temp # Find and replace random seed value
	sed -i "s/<experiment length='2000'*/<experiment length='$time'/" $temp # Find and replace random seed value
	sed -i "s/file_name='data1'*/file_name='data$random_seed'/" $temp # Find and replace random seed value
	sed -i '/<visualization>/,/<\/visualization>/d' $temp # Delete the visualization block


	argos3 -c $temp # Run ARGoS using the temporary configuration file

	rm $temp
done
