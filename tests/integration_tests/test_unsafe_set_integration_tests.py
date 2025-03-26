import sys
import os
from pathlib import Path
sys.path.append(os.path.abspath(__package__) + '/src')
from colav_unsafe_set.unsafe_set.unsafe_set import create_unsafe_set
import pytest
import yaml
import os
from pprint import pprint  # Import pprint for better readability of printed YAML data
import matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
plt.style.use('default') 
from colav_unsafe_set.objects import Agent, DynamicObstacle
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as R


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


def plot_scenario(agent, obstacles, unsafe_set_vertices, width, height, scenario_name):
    """Generate and save a plot of the scenario."""
    fig, ax = plt.subplots()
    ax.imshow(np.zeros((width, height)), cmap='gray_r', interpolation='nearest')

    # Plot unsafe set
    if unsafe_set_vertices:
        ax.add_patch(patches.Polygon(unsafe_set_vertices, closed=True, edgecolor='black', facecolor='#ff7f0e', alpha=0.5))

    # Plot agent vessel
    ax.add_patch(plt.Circle(agent.position, agent.safety_radius, facecolor='blue', alpha=0.8, edgecolor='black'))
    ax.add_patch(patches.RegularPolygon(
        agent.position, numVertices=3, radius=2,
        orientation=quaternion_to_yaw(agent.orientation), facecolor='black'
    ))

    # Plot obstacles
    for obstacle in obstacles:
        ax.add_patch(plt.Circle(obstacle.position, obstacle.safety_radius, facecolor='red', alpha=0.5, edgecolor='black'))
        ax.add_patch(patches.RegularPolygon(
            obstacle.position, numVertices=3, radius=2,
            orientation=quaternion_to_yaw(obstacle.orientation), facecolor='black'
        ))

    # Configure plot aesthetics
    ax.grid(color='black', linestyle='--', linewidth=0.5)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_facecolor('white')
    fig.patch.set_facecolor('white')

    # Save the plot
    plt.title(f"{scenario_name}")
    plt.xlabel("X-Axis (meters)")
    plt.ylabel("Y-Axis (meters)")
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

    unsafe_set_vertices = create_unsafe_set(agent, obstacles, scenario.get('dsf'))
    plot_scenario(agent, obstacles, unsafe_set_vertices, width, height, scenario_file.stem)


def main():
    for scenario_file in get_scenario_files():
        test_unsafe_set_gen(scenario_file)


if __name__ == '__main__':
    main()