Here's a `README.md` for your MuJoCo Drone Swarm simulation project. You can copy this directly into a markdown file in your project directory.

---

# MuJoCo Drone Swarm Simulation

## Project Description

This Python project uses the MuJoCo physics engine to simulate a drone swarm with a leader-follower dynamic. The simulation allows the leader drone to control the movement and formation of the follower drones in real-time. Each follower adjusts its position relative to the leader based on predefined transformations.

## Prerequisites

To run this simulation, ensure you have the following installed:

- Python 3.6 or higher
- [MuJoCo](https://mujoco.org/) 2.1.0 or higher
- mujoco-py
- NumPy
- SciPy

Make sure you have a valid MuJoCo license and that MuJoCo is correctly installed on your system.

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://yourrepositorylink.git
   cd your-repository-folder
   ```

2. **Set up a virtual environment (optional but recommended):**
   ```bash
   python -m venv env
   source env/bin/activate  # On Windows use `env\Scripts\activate`
   ```

3. **Install dependencies:**
   ```bash
   pip install mujoco numpy scipy
   ```

## Usage

To run the simulation:

1. **Prepare your drone model file:**
   Ensure `drone_swarm.xml` is in the project directory or adjust the path in the script to match the location of your XML model file.

2. **Execute the script:**
   ```bash
   python drone_simulation.py
   ```

## Features

- **Leader Control**: Dynamically control the leader drone's position and orientation through script parameters.
- **Formation Dynamics**: Follower drones automatically adjust their positions to maintain a specified formation relative to the leader.
- **Multiple Camera Views**: Includes several camera views (top, bottom, front, back, left, right) to fully visualize the drone dynamics.

## Configuration

- **Modify Drone Formation**: Adjust the `formation_dict` in the `Drone` class to change the formation geometry of the swarm.
- **Leader Settings**: Modify the leader's actions within the script to change how the leader moves and how the followers respond.

## Development

Feel free to fork this project and extend it with more features like obstacle avoidance, more complex formations, or enhanced control algorithms.

## Acknowledgements

This project uses the [MuJoCo](https://mujoco.org/) physics engine developed by Roboti LLC. Thank you to the MuJoCo community for providing extensive documentation and support.

## License

This project is released under the MIT License. See the `LICENSE` file for more details.

---

**Note**: Remember to replace `https://yourrepositorylink.git` with the actual URL of your Git repository and to provide any additional instructions specific to your project setup if necessary.