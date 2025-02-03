import pygame
import math
import numpy as np
import random

# Initialize Pygame
pygame.init()

# Screen dimensions and frame rate
WIDTH, HEIGHT = 800, 600
FPS = 60

# Physics constants
GRAVITY = 0.5           # gravitational acceleration (pixels/frame^2)
FRICTION = 0.99         # air friction (applied to ball velocity each frame)
RESTITUTION = 0.9       # coefficient of restitution for collisions

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED   = (255, 0, 0)
BLUE  = (0, 0, 255)
GREEN = (0, 255, 0)

# Create the screen and clock
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Balls in a Spinning Hexagon")
clock = pygame.time.Clock()

# Hexagon parameters
hex_center = np.array([WIDTH / 2, HEIGHT / 2])
hex_radius = 200
hex_rotation = 0.0               # initial rotation angle (radians)
angular_velocity = 0.01 * math.pi  # angular velocity in radians per frame

def get_hexagon_vertices(center, radius, rotation):
    """Return a list of 6 vertices for a hexagon rotated by 'rotation'."""
    vertices = []
    for i in range(6):
        angle = math.radians(60 * i) + rotation
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        vertices.append(np.array([x, y]))
    return vertices

def point_in_polygon(point, polygon):
    """
    Ray-casting algorithm to check if a point is inside a polygon.
    'polygon' is a list of np.array vertices.
    """
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
    return inside

def get_random_point_in_polygon(polygon):
    """Return a random point inside the given polygon using rejection sampling."""
    xs = [v[0] for v in polygon]
    ys = [v[1] for v in polygon]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    while True:
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        point = np.array([x, y])
        if point_in_polygon(point, polygon):
            return point

# Generate initial hexagon vertices for position generation (with rotation = 0)
initial_hex_vertices = get_hexagon_vertices(hex_center, hex_radius, 0)

# Generate a random number (1-100) of each colored ball.
balls = []

# Red balls
red_count = random.randint(1, 100)
for _ in range(red_count):
    ball = {
        "pos": get_random_point_in_polygon(initial_hex_vertices),
        "vel": np.array([random.uniform(-3, 3), random.uniform(-3, 3)], dtype=float),
        "radius": 10,
        "color": RED
    }
    balls.append(ball)

# Blue balls
blue_count = random.randint(1, 100)
for _ in range(blue_count):
    ball = {
        "pos": get_random_point_in_polygon(initial_hex_vertices),
        "vel": np.array([random.uniform(-3, 3), random.uniform(-3, 3)], dtype=float),
        "radius": 10,
        "color": BLUE
    }
    balls.append(ball)

# Green balls
green_count = random.randint(1, 100)
for _ in range(green_count):
    ball = {
        "pos": get_random_point_in_polygon(initial_hex_vertices),
        "vel": np.array([random.uniform(-3, 3), random.uniform(-3, 3)], dtype=float),
        "radius": 10,
        "color": GREEN
    }
    balls.append(ball)

print(f"Red balls: {red_count}, Blue balls: {blue_count}, Green balls: {green_count}")

running = True
while running:
    dt = clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Update Hexagon ---
    hex_rotation += angular_velocity
    vertices = get_hexagon_vertices(hex_center, hex_radius, hex_rotation)

    # --- Update Each Ball's Physics ---
    for ball in balls:
        # Apply gravity
        ball["vel"][1] += GRAVITY
        # Apply friction (air resistance)
        ball["vel"] *= FRICTION
        # Update position
        ball["pos"] += ball["vel"]

        # --- Collision Detection & Response (Edge-based) ---
        for i in range(6):
            A = vertices[i]
            B = vertices[(i + 1) % 6]
            AB = B - A
            AP = ball["pos"] - A
            # Projection factor (clamped to the edge)
            t = np.dot(AP, AB) / np.dot(AB, AB)
            t = max(0, min(1, t))
            closest = A + t * AB
            dist = np.linalg.norm(ball["pos"] - closest)

            if dist < ball["radius"]:
                # Collision detected.
                n = (ball["pos"] - closest) / (dist if dist != 0 else 1)
                # Compute wall's velocity at the collision point (v = ω × r)
                rel_point = closest - hex_center
                v_wall = angular_velocity * np.array([-rel_point[1], rel_point[0]])
                # Ball's relative velocity to the moving wall
                v_rel = ball["vel"] - v_wall

                # Reflect only if moving into the wall
                if np.dot(v_rel, n) < 0:
                    v_rel = v_rel - 2 * np.dot(v_rel, n) * n
                    ball["vel"] = v_rel + v_wall
                    ball["vel"] *= RESTITUTION
                    # Resolve penetration
                    penetration = ball["radius"] - dist
                    ball["pos"] += n * penetration

        # --- Additional Correction: Keep Ball Fully Inside the Hexagon ---
        if not point_in_polygon(ball["pos"], vertices):
            # Find the closest point on the hexagon boundary
            min_dist = float('inf')
            closest_point = None
            for i in range(6):
                A = vertices[i]
                B = vertices[(i + 1) % 6]
                AB = B - A
                AP = ball["pos"] - A
                t = np.dot(AP, AB) / np.dot(AB, AB)
                t = max(0, min(1, t))
                pt = A + t * AB
                d = np.linalg.norm(ball["pos"] - pt)
                if d < min_dist:
                    min_dist = d
                    closest_point = pt

            if closest_point is not None:
                # Compute the inward normal (from the ball toward the boundary)
                n = closest_point - ball["pos"]
                norm = np.linalg.norm(n)
                n = n / norm if norm != 0 else np.array([0, 1])
                # Reflect the ball's velocity using the inward normal
                ball["vel"] = ball["vel"] - 2 * np.dot(ball["vel"], n) * n
                ball["vel"] *= RESTITUTION
                # Reposition the ball so that its edge touches the boundary from the inside
                ball["pos"] = closest_point + n * ball["radius"]

    # --- Rendering ---
    screen.fill(BLACK)
    # Draw the rotating hexagon
    hex_points = [vertex.tolist() for vertex in vertices]
    pygame.draw.polygon(screen, WHITE, hex_points, 2)
    # Draw all balls
    for ball in balls:
        pygame.draw.circle(screen, ball["color"], ball["pos"].astype(int).tolist(), ball["radius"])
    pygame.display.flip()

pygame.quit()
