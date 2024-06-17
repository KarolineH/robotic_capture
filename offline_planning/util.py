
import numpy as np
import matplotlib.pyplot as plt
import json

def plot_points(points):
    import matplotlib
    matplotlib.use('Qt5Agg')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_coords, y_coords, z_coords = zip(*points)
    ax.scatter(x_coords, y_coords, z_coords, s=20)

    # Set the aspect ratio to be equal
    ax.set_box_aspect([1,1,1])
    plt.show()
    return

def plot_radius_vs_reachables(results):
    import matplotlib
    matplotlib.use('Qt5Agg')
    matplotlib.pyplot.scatter(np.asarray([q[1] for q in results]),np.asarray([len(q[2]) for q in results]))
    matplotlib.pyplot.show()
    return

def plot_yz_offset_vs_reachables(results):
    import matplotlib
    matplotlib.use('Qt5Agg')
    # make a 2D heatmap where x and y are the offsets and the color is the number of reachable points
    x = np.asarray([q[0][1] for q in results])
    y = np.asarray([q[0][2] for q in results])
    z = np.asarray([len(q[2]) for q in results])
    matplotlib.pyplot.hist2d(x, y, bins=(21, 13), weights=z, cmap=matplotlib.pyplot.cm.jet)
    matplotlib.pyplot.colorbar()
    matplotlib.pyplot.xlabel('Y offset')
    matplotlib.pyplot.ylabel('Z offset')
    matplotlib.pyplot.show()
    return
    
def plot_yzr_fit(results):
    # 3D plot
    import matplotlib
    matplotlib.use('Qt5Agg')
    x = np.asarray([q[0][1] for q in results])
    y = np.asarray([q[0][2] for q in results])
    z = np.asarray([q[1] for q in results])
    c = np.asarray([len(q[2]) for q in results])
    fig = matplotlib.pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c=c, cmap='viridis')
    ax.set_xlabel('y')
    ax.set_ylabel('z')
    ax.set_zlabel('Radius')
    matplotlib.pyplot.show()
    return

def save_pickle(file_name, results):
    import pickle
    with open(file_name, 'wb') as f:
        pickle.dump(results, f)

def load_pickle(file_name):
    import pickle
    with open(file_name, 'rb') as f:
        return pickle.load(f)

def append_to_log(file_path, iteration_results):
    with open(file_path, 'a') as file:
        json.dump(iteration_results, file)
        file.write('\n')  # Ensure each iteration result is on a new line

def load_log(file_path):
    results = []
    with open(file_path, 'r') as file:
        for line in file:
            # Parse JSON line
            iteration_results = json.loads(line)
            
            # Reconstruct the data structures
            origin = np.asarray(iteration_results[0])
            radius = iteration_results[1]
            joint_states = [entry for entry in iteration_results[2]]
            reachable_poses = [np.asarray(entry) for entry in iteration_results[3]]

            results.append((origin, radius, joint_states, reachable_poses))
    return results