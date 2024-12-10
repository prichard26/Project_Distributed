import numpy as np

# Read the file content
with open('./Centralized/experiment_logs.txt', 'r') as file:
    content = file.read()

# Split the content into experiments
experiments = content.strip().split('===== End of Simulation: Key metrics ======')
experiments = [exp.strip() for exp in experiments if exp.strip()]

num_experiments = len(experiments)
num_robots = 5

# Initialize arrays
activation_times = np.zeros((num_robots, num_experiments))
total_collisions = np.zeros(num_experiments)
total_distance = np.zeros(num_experiments)
total_time_handling_task = np.zeros(num_experiments)
events_per_second = np.zeros(num_experiments)

# Parse each experiment
for i, exp in enumerate(experiments):
    lines = exp.split('\n')
    for line in lines:
        line = line.strip()
        if line.startswith('Robot'):
            # Extract robot ID and activation time
            robot_id = int(line.split(':')[0].split(' ')[1])
            activation_time = float(line.split('=')[1].strip('%'))
            activation_times[robot_id, i] = activation_time
        elif line.startswith('Total collisions'):
            total_collisions[i] = int(line.split('=')[1].strip())
        elif line.startswith('Total distance travelled'):
            total_distance[i] = float(line.split('=')[1].strip())
        elif line.startswith('Total time handeling task'):
            total_time_handling_task[i] = float(line.split('=')[1].strip())
        elif line.startswith('Handled'):
            # Extract events handled per second
            events_per_second[i] = float(line.split('=')[1].strip())

# Compute mean activation time per experiment (over robots)
mean_activation_times_per_experiment = activation_times.mean(axis=0)

# Compute intra-experiment std (over robots)
std_activation_times_per_experiment = activation_times.std(axis=0)

# Compute mean and std over experiments for activation times
mean_activation_time_over_experiments = mean_activation_times_per_experiment.mean()
std_activation_time_over_experiments = mean_activation_times_per_experiment.std()

# Compute mean and std over experiments for other metrics
mean_total_collisions = total_collisions.mean()
std_total_collisions = total_collisions.std()

mean_total_distance = total_distance.mean()
std_total_distance = total_distance.std()

mean_total_time_handling_task = total_time_handling_task.mean()
std_total_time_handling_task = total_time_handling_task.std()

mean_events_per_second = events_per_second.mean()
std_events_per_second = events_per_second.std()

# Print the results nicely
print(f"Mean Activation Time over Experiments: {mean_activation_time_over_experiments:.2f}% ± {std_activation_time_over_experiments:.2f}%")

print("\nActivation Time per Experiment:")
for i in range(num_experiments):
    mean_at = mean_activation_times_per_experiment[i]
    std_at = std_activation_times_per_experiment[i]
    print(f"Experiment {i+1}: Mean = {mean_at:.2f}%, Std = {std_at:.2f}%")

print(f"\nTotal Collisions: Mean = {mean_total_collisions:.2f}, Std = {std_total_collisions:.2f}")
print(f"Total Distance Travelled: Mean = {mean_total_distance:.6f}, Std = {std_total_distance:.6f}")
print(f"Total Time Handling Task: Mean = {mean_total_time_handling_task:.6f}, Std = {std_total_time_handling_task:.6f}")
print(f"Events Handled Per Second: Mean = {mean_events_per_second:.2f}, Std = {std_events_per_second:.2f}")