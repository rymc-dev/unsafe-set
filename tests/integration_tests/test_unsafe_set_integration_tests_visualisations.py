import sys
import os
from pathlib import Path
sys.path.append(os.path.abspath(__package__) + '/src')
from colav_unsafe_set.unsafe_set.unsafe_set import create_unsafe_set
import pytest
import yaml
import os
from pprint import pprint  # For better readability of printed YAML data
import matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
plt.style.use('default')
from colav_unsafe_set.objects import Agent, DynamicObstacle
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation

# Set matplotlib style
plt.style.use('default')

SCENARIO_DIR = Path(__file__).resolve().parent / 'scenarios'
PLOT_DIR = Path(__file__).resolve().parent / 'plots'
PLOT_DIR.mkdir(exist_ok=True)  # Ensure plots directory exists

def get_scenario_files():
    """Retrieve a list of YAML scenario files."""
    return [f for f in SCENARIO_DIR.iterdir() if f.suffix == '.yml']

def load_yaml(file_path):
    """Load YAML content from a file."""
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def quaternion_to_yaw(quaternion):
    """Convert a quaternion (w, x, y, z) to a yaw angle in radians."""
    r = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])  # (x, y, z, w)
    return r.as_euler('xyz', degrees=False)[2]  # Yaw (rotation about Z-axis)

def compute_triangle_vertices(position, orientation, length=5, width=2):
    """Compute vertices of an elongated triangle for a given position and orientation."""
    yaw = quaternion_to_yaw(orientation)
    # Define the triangle in the local frame: tip at (length, 0) and base at (0, ±width/2)
    triangle = np.array([
        [length, 0],
        [0, -width / 2],
        [0, width / 2]
    ])
    # Build the rotation matrix from the yaw angle
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])
    # Rotate the triangle vertices and then translate to the world position
    return (rotation_matrix @ triangle.T).T + position

def draw_elongated_triangle(ax, position, orientation, length=5, width=2):
    """Draw and return an elongated triangle representing an object at a given position and orientation."""
    vertices = compute_triangle_vertices(position, orientation, length, width)
    triangle_patch = patches.Polygon(vertices, closed=True, facecolor='black')
    ax.add_patch(triangle_patch)
    return triangle_patch

def update_orientation(orientation, yaw_rate):
    """
    Update the orientation (heading) based on the yaw_rate and timestep.
    Assuming the timestep is 1 frame (you can adjust this as needed).
    """
    # Convert quaternion to yaw, add yaw rate, and convert back to quaternion
    yaw = quaternion_to_yaw(orientation)
    new_yaw = yaw + yaw_rate  # Update yaw by yaw_rate
    return yaw_to_quaternion(new_yaw)

def quaternion_to_yaw(orientation):
    """
    Convert quaternion orientation to yaw angle (in radians).
    Assumes the orientation is a unit quaternion: [w, x, y, z].
    """
    # Extract the yaw from quaternion (assuming it is in the form [w, x, y, z])
    w, x, y, z = orientation
    sin_yaw = 2 * (w * z + x * y)
    cos_yaw = 1 - 2 * (y ** 2 + z ** 2)
    yaw = np.arctan2(sin_yaw, cos_yaw)
    return yaw

def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion [w, x, y, z].
    """
    qw = np.cos(yaw / 2)
    qx = 0
    qy = 0
    qz = np.sin(yaw / 2)
    return np.array([qw, qx, qy, qz])

def plot_scenario(agent, obstacles, width, height, scenario_name, dsf, animate=False):
    """Generate and optionally animate a plot of the scenario."""
    fig, ax = plt.subplots()

    # Calculate half of the width and height to set axis limits centered at (0, 0)
    half_width = width / 2
    half_height = height / 2

    # Set axis limits from -half_width to half_width and -half_height to half_height
    ax.set_xlim(-half_width, half_width)
    ax.set_ylim(-half_height, half_height)

    # Add grid with intervals based on width/height
    ax.grid(color='black', linestyle='--', linewidth=0.5)
    ax.set_xticks(np.arange(-half_width, half_width + 1, 50))  # Set X ticks at intervals of 50
    ax.set_yticks(np.arange(-half_height, half_height + 1, 50))  # Set Y ticks at intervals of 50

    # Plot agent and obstacles at the initial positions
    agent_circle = ax.add_patch(plt.Circle(agent.position, agent.safety_radius,
                                           facecolor='blue', alpha=0.8, edgecolor='black'))
    agent_triangle = draw_elongated_triangle(ax, agent.position, agent.orientation)

    obstacle_circles = []
    obstacle_triangles = []
    for obstacle in obstacles:
        obstacle_circle = ax.add_patch(plt.Circle(obstacle.position, obstacle.safety_radius,
                                                  facecolor='red', alpha=0.5, edgecolor='black'))
        triangle_patch = draw_elongated_triangle(ax, obstacle.position, obstacle.orientation)
        obstacle_circles.append(obstacle_circle)
        obstacle_triangles.append(triangle_patch)

    # Configure plot aesthetics
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')

    def update(frame):
        # Update agent orientation based on its yaw_rate
        agent.orientation = update_orientation(agent.orientation, agent.yaw_rate)
        
        # Update agent position based on its orientation and velocity
        delta_agent = np.array([
            np.cos(quaternion_to_yaw(agent.orientation)) * agent.velocity,
            np.sin(quaternion_to_yaw(agent.orientation)) * agent.velocity
        ])
        agent.position += delta_agent
        agent_circle.center = agent.position

        # Recompute agent triangle vertices and update patch
        new_agent_vertices = compute_triangle_vertices(agent.position, agent.orientation)
        agent_triangle.set_xy(new_agent_vertices)

        # Update obstacles' positions and their orientations based on yaw_rate
        for i, obstacle in enumerate(obstacles):
            # Update obstacle orientation based on its yaw_rate
            obstacle.orientation = update_orientation(obstacle.orientation, obstacle.yaw_rate)

            # Update obstacle position based on its velocity and new orientation
            delta_obs = np.array([
                np.cos(quaternion_to_yaw(obstacle.orientation)) * obstacle.velocity,
                np.sin(quaternion_to_yaw(obstacle.orientation)) * obstacle.velocity
            ])
            obstacle.position += delta_obs
            obstacle_circles[i].center = obstacle.position

            # Recompute obstacle triangle vertices and update patch
            new_obs_vertices = compute_triangle_vertices(obstacle.position, obstacle.orientation)
            obstacle_triangles[i].set_xy(new_obs_vertices)

        # Create or update the unsafe set patch
        unsafe_set_vertices = create_unsafe_set(agent, obstacles, dsf)
        if unsafe_set_vertices:
            if not hasattr(update, 'unsafe_patch'):  # If unsafe_patch does not exist, create it
                update.unsafe_patch = patches.Polygon(unsafe_set_vertices, closed=True, edgecolor='black',
                                                    facecolor='#ff7f0e', alpha=0.5)
                ax.add_patch(update.unsafe_patch)  # Add it to the axis
            else:
                update.unsafe_patch.set_xy(unsafe_set_vertices)  # Update the existing patch
        else:
            if hasattr(update, 'unsafe_patch'):
                update.unsafe_patch.set_visible(False)  # Hide the patch if no unsafe set exists

        # Return a list of Artist objects to be animated
        return [agent_circle, agent_triangle] + obstacle_circles + obstacle_triangles + \
            ([update.unsafe_patch] if hasattr(update, 'unsafe_patch') and update.unsafe_patch.get_visible() else [])


    if animate:
        ani = FuncAnimation(fig, update, frames=range(100), interval=100, blit=True)
        plt.title(f"Animation for {scenario_name}")
        gif_path = PLOT_DIR / f"{scenario_name.replace(':', '_')}_animation.gif"
        # Save the animation as a GIF using the Pillow writer
        ani.save(gif_path, writer='pillow', fps=10)
        plt.close()
    else:
        plt.title(f"{scenario_name}")
        plt.xlabel("X-Axis (meters)")
        plt.ylabel("Y-Axis (meters)")
        # Save the static plot image in the specified folder
        plt.savefig(PLOT_DIR / f"{scenario_name.replace(':', '_')}_plot.png", facecolor=fig.get_facecolor())
        plt.close()

@pytest.mark.parametrize("scenario_file", get_scenario_files())
def test_unsafe_set_gen(scenario_file):
    """Test unsafe set generation and visualization for given scenarios."""
    scenario = load_yaml(scenario_file)
    width, height = int(scenario['matrix']['width']), int(scenario['matrix']['height'])

    agent = Agent(**scenario['agent_vessel'])
    if scenario.get('obstacles') is not None:
        obstacles = [DynamicObstacle(**obs) for obs in scenario.get('obstacles', [])]
    else:
        obstacles = []

    plot_scenario(agent, obstacles, width, height, scenario_file.stem, scenario.get('dsf'), animate=True)

def main():
    for scenario_file in get_scenario_files():
        test_unsafe_set_gen(scenario_file)

if __name__ == '__main__':
    main()
