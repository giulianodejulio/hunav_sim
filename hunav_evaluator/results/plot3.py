import matplotlib.pyplot as plt

def read_data(file_path):
    """
    Reads data from a file and returns time stamps and a dictionary of metrics with column names as keys.
    """
    time_stamps = []
    metrics = {}

    with open(file_path, 'r') as file:
        # Read the header to get the metric names
        header = next(file).strip().split('\t')
        
        # Initialize an empty list for each metric
        for metric_name in header[1:]:  # Skip time stamps column
            metrics[metric_name] = []

        # Read the data lines
        for line in file:
            values = line.strip().split('\t')
            if len(values) == len(header):
                try:
                    time_stamp = float(values[0])
                    time_stamps.append(time_stamp)
                    
                    # Add the metric values to their corresponding lists
                    for i, metric_name in enumerate(header[1:], start=1):
                        metrics[metric_name].append(float(values[i]))
                except ValueError:
                    # Skip lines that do not convert to float properly
                    continue

    return time_stamps, metrics

def plot_data(time_stamps, metrics):
    """
    Plots time stamps versus each metric on the same figure.
    """
    plt.figure(figsize=(12, 6))

    # Plot each metric
    for metric_name, metric_values in metrics.items():
        plt.plot(time_stamps, metric_values, marker='', linestyle='-', label=metric_name)

    plt.xlabel('Time (s)')
    plt.ylabel('Values')
    plt.title('Metrics vs Time')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Define the path to the data file
    file_path = 'metrics_steps_1.txt'
    
    # Read the data from the file
    time_stamps, metrics = read_data(file_path)
    
    # Plot the data
    plot_data(time_stamps, metrics)
