# Ball in Spinning Hexagon

## Features

- **Physics Simulation**: The programs simulate gravitational acceleration, friction, and ball-wall collisions using vector mathematics.
- **Hexagon Rotation**: The hexagon continuously rotates, affecting the ball dynamics.
- **Multiple Variants**:
  - `ball-in-spinning-hex-1.py`: Single ball bouncing inside the hexagon.
  - `ball-in-spinning-hex-2.py`: Two balls bouncing inside the hexagon.
  - `ball-in-spinning-hex-3.py`: Three balls bouncing inside the hexagon.
  - `ball-in-spinning-hex-rand10.py`: Randomly generates up to 10 balls with different velocities and positions.
  - `ball-in-spinning-hex-rand100.py`: Randomly generates up to 100 balls for a more chaotic simulation.

## Program Descriptions

### ball-in-spinning-hex-1.py
A **physics simulation** built with Pygame that demonstrates a bouncing ball inside a continuously rotating hexagon.

**Visual Elements:**
- A white hexagon (200px radius) that rotates continuously at the center of an 800x600 window
- A red ball (10px radius) that bounces around inside the hexagon
- Black background

**Physics Simulation:**
- **Gravity**: Ball experiences downward acceleration (0.5 pixels/frame²)
- **Air Friction**: Ball velocity is reduced by 1% each frame
- **Collision Response**: Ball bounces off hexagon walls with 90% energy retention (restitution coefficient)
- **Moving Wall Interaction**: The collision system accounts for the hexagon's rotation, properly calculating the wall velocity at collision points using rotational physics (v = ω × r)

**Technical Implementation:**
- Uses **edge-based collision detection** - checks distance from ball center to each of the 6 hexagon edges
- Implements **relative velocity collision response** - when the ball hits a moving wall, it calculates velocity relative to that wall's motion, reflects it, then adds the wall velocity back
- Includes **penetration correction** to prevent the ball from getting stuck in walls
- Has a backup **containment system** using ray-casting to ensure the ball stays inside the hexagon even if primary collision detection fails

The hexagon rotates at approximately 1.8°/frame (0.01π radians/frame), creating an interesting dynamic environment where the ball's bouncing behavior is influenced by the rotating container walls.

### ball-in-spinning-hex-rand100.py
An **enhanced multi-ball physics simulation** that extends the original single-ball concept to handle hundreds of colored balls simultaneously.

**Key Differences from the Single-Ball Version:**

**Multiple Balls:**
- Creates **three groups of balls** in different colors: red, blue, and green
- Each color group has a **random quantity** between 1-100 balls (determined at startup)
- Total ball count ranges from 3 to 300 balls
- Prints the count of each color to console on startup

**Initialization:**
- Balls are spawned at **random positions** inside the hexagon using rejection sampling
- Each ball gets a **random initial velocity** between -3 and 3 pixels/frame in both x and y directions

**Data Structure:**
- Each ball is stored as a dictionary containing:
  - `pos`: position (numpy array)
  - `vel`: velocity (numpy array)
  - `radius`: size (10 pixels for all)
  - `color`: RGB tuple (RED, BLUE, or GREEN)

**Physics:**
- Same collision physics as the single-ball version (gravity, friction, restitution, moving wall interactions)
- All balls experience independent physics calculations
- Same edge-based collision detection with penetration correction
- Same backup containment system

**Performance:**
- Runs at 60 FPS with potentially hundreds of simultaneous collision calculations
- Each ball checks all 6 hexagon edges every frame

This creates a visually dynamic simulation with a chaotic mass of colored balls bouncing around inside the rotating hexagon container.

### Other Variants
The remaining programs (`ball-in-spinning-hex-2.py`, `ball-in-spinning-hex-3.py`, `ball-in-spinning-hex-rand10.py`) are variations of these two programs with different numbers of balls.

## Requirements
Ensure you have Python and Pygame installed before running the scripts.

## Setup Instructions
Clone the repository and set up a virtual environment:
```sh
git clone https://github.com/drhitchen/ball-in-spinning-hex.git
cd ball-in-spinning-hex
python -m venv venv
source venv/bin/activate  # On Windows use: venv\Scripts\activate
pip install -r requirements.txt
```

## Running the Simulations
Execute any of the scripts using Python:
```sh
python ball-in-spinning-hex-1.py
```
Replace the filename with any of the other script names to see different variations of the simulation.

## Controls
Click the **close button** to exit the simulation.

## File Structure
```
ball-in-spinning-hex/
├── ball-in-spinning-hex-1.py
├── ball-in-spinning-hex-2.py
├── ball-in-spinning-hex-3.py
├── ball-in-spinning-hex-rand10.py
├── ball-in-spinning-hex-rand100.py
├── requirements.txt
```

## Credit
Credit to Flavio Adamo on X for creating this.

[Flavio Adamo](https://x.com/flavioAd/status/1885449107436679394)

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
