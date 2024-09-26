import matplotlib.pyplot as plt

def read_data(file_path):
    """
    Reads data from a file and returns time stamps, average robot linear speeds,
    and average robot angular speeds as lists.
    """
    time_stamps = []
    avg_robot_linear_speed = []
    avg_robot_angular_speed = []

    with open(file_path, 'r') as file:
        # Skip the header line
        next(file)
        
        for line in file:
            # Split line into values and convert to appropriate types
            values = line.strip().split('\t')
            if len(values) == 3:
                try:
                    time_stamp = float(values[0])
                    linear_speed = float(values[1])
                    angular_speed = float(values[2])
                    time_stamps.append(time_stamp)
                    avg_robot_linear_speed.append(linear_speed)
                    avg_robot_angular_speed.append(angular_speed)
                except ValueError:
                    # Skip lines that do not convert to float properly
                    continue

    return time_stamps, avg_robot_linear_speed, avg_robot_angular_speed

def plot_data(time_stamps, avg_robot_linear_speed, avg_robot_angular_speed):
    """
    Plots time stamps versus average robot linear speed and angular speed on the same figure.
    """
    plt.figure(figsize=(12, 6))

    # Plot linear speed
    plt.plot(time_stamps, avg_robot_linear_speed, marker='o', linestyle='-', color='b', label='Avg Robot Linear Speed')
    
    # Plot angular speed
    plt.plot(time_stamps, avg_robot_angular_speed, marker='x', linestyle='--', color='r', label='Avg Robot Angular Speed')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Speed')
    plt.title('Robot Speed vs Time')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Define the path to the data file
    file_path = 'metrics_steps_1.txt'
    
    # Read the data from the file
    time_stamps, avg_robot_linear_speed, avg_robot_angular_speed = read_data(file_path)
    
    # Plot the data
    plot_data(time_stamps, avg_robot_linear_speed, avg_robot_angular_speed)
