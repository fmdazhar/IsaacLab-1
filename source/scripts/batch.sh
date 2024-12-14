#!/bin/bash

# Define ranges for hyperparameters
actor_lr_values=(0.0001 0.0005 0.001)
critic_lr_values=(0.0001  0.00055 0.001)
target_entropy_values=(-20 -15 -10)
num_steps_per_episode_values=(1 4 8)

# Loop through all combinations of hyperparameters
for actor_lr in "${actor_lr_values[@]}"
do
    for critic_lr in "${critic_lr_values[@]}"
    do
        for target_entropy in "${target_entropy_values[@]}"
        do
            for num_steps_per_episode in "${num_steps_per_episode_values[@]}"
            do
                # Print the current configuration
                echo "Running training with actor_lr=$actor_lr, critic_lr=$critic_lr, target_entropy=$target_entropy, num_steps_per_episode=$num_steps_per_episode"
                
                # Run the training script
                ./isaaclab.sh -p source/standalone/workflows/rl_games/train.py \
                --task Isaac-Velocity-Flat-Anymal-C-SAC-v0 --headless \
                agent.params.config.actor_lr=$actor_lr agent.params.config.critic_lr=$critic_lr agent.params.config.target_entropy=$target_entropy agent.params.config.num_steps_per_episode=$num_steps_per_episode
                
                # Wait for the current training to finish before starting the next one
                wait
            done
        done
    done
done
