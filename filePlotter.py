# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors(filename: str):
    
    headers, values=FileReader(filename).read_file()
    filename_list = filename.split('_')
    print(filename_list)
    sensor = filename_list[0]
    filename_list[-1] = filename_list[-1].split('.')[0]
    movement = filename_list[-1]
    time_list=[]
    first_stamp=values[0][-1]

    # X and Y lists used for graphing Odometry X vs. Y graph 
    # x_list = []
    # y_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)
        # x_list.append(val[0])
        # y_list.append(val[1])

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    # Plotting x to y for odom
    # plt.plot(x_list, y_list)
    
    plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.title(f"{sensor} measurements for {movement} movement")
    # plt.title(f"Odometry X-Y Positional Data")
    plt.xlabel("Timestamp (ns)")
    # plt.xlabel("X Location")
    plt.ylabel("X, Y Acceleration (m/s^2) & Th Anglular Velocity (rad/s)")
    # plt.ylabel("Y location")
    plt.legend()
    plt.grid()
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)
