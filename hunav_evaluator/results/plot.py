import matplotlib.pyplot as plt

def read_data(file_path):
    """
    Reads data from a file and returns time stamps and average robot linear speeds as lists.
    """
    time_stamps = []
    avg_robot_linear_speed = []

    with open(file_path, 'r') as file:
        # Skip the header line
        next(file)
        
        for line in file:
            # Split line into values and convert to appropriate types
            values = line.strip().split('\t')
            if len(values) == 2:
                try:
                    time_stamp = float(values[0])
                    speed = float(values[1])
                    time_stamps.append(time_stamp)
                    avg_robot_linear_speed.append(speed)
                except ValueError:
                    # Skip lines that do not convert to float properly
                    continue

    return time_stamps, avg_robot_linear_speed

def plot_data(time_stamps, avg_robot_linear_speed):
    """
    Plots time stamps versus average robot linear speed.
    """
    plt.figure(figsize=(10, 6))
    plt.plot(time_stamps, avg_robot_linear_speed, marker='o', linestyle='-', color='b')
    plt.xlabel('Time (s)')
    plt.ylabel('Average Robot Linear Speed (m/s)')
    plt.title('Robot Linear Speed vs Time')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Define the path to the data file
    file_path = 'metrics_steps_1.txt'
    
    # Read the data from the file
    time_stamps, avg_robot_linear_speed = read_data(file_path)
    
    # Plot the data
    plot_data(time_stamps, avg_robot_linear_speed)
