import pygame
import math
import numpy as np

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

# Create the screen and clock
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Bouncing Ball in a Spinning Hexagon")
clock = pygame.time.Clock()

# Hexagon parameters
hex_center = np.array([WIDTH / 2, HEIGHT / 2])
hex_radius = 200
hex_rotation = 0.0              # current rotation angle in radians
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
    Ray-casting algorithm to check if point is inside a polygon.
    'polygon' is a list of np.array vertices.
    """
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        j = (i + 1) % n
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        # Check if point is between the y-coordinates of the edge
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
    return inside

# Ball parameters
ball_pos = np.array([WIDTH / 2, HEIGHT / 2 - 100], dtype=float)
ball_radius = 10
ball_vel = np.array([3, 0], dtype=float)

running = True
while running:
    dt = clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Update Hexagon ---
    hex_rotation += angular_velocity
    vertices = get_hexagon_vertices(hex_center, hex_radius, hex_rotation)

    # --- Update Ball Physics ---
    # Apply gravity and friction
    ball_vel[1] += GRAVITY
    ball_vel *= FRICTION
    ball_pos += ball_vel

    # --- Collision Detection & Response (Edge-based) ---
    for i in range(6):
        A = vertices[i]
        B = vertices[(i + 1) % 6]
        AB = B - A
        AP = ball_pos - A
        # Projection factor (clamped to segment)
        t = np.dot(AP, AB) / np.dot(AB, AB)
        t = max(0, min(1, t))
        closest = A + t * AB
        dist = np.linalg.norm(ball_pos - closest)

        if dist < ball_radius:
            # Collision detected with this edge.
            # Determine the collision normal (from wall to ball)
            n = (ball_pos - closest) / (dist if dist != 0 else 1)
            
            # Compute the wall's velocity at the collision point (v = ω × r)
            rel_point = closest - hex_center
            v_wall = angular_velocity * np.array([-rel_point[1], rel_point[0]])

            # Compute the ball's velocity relative to the moving wall
            v_rel = ball_vel - v_wall

            # Only respond if the ball is moving into the wall
            if np.dot(v_rel, n) < 0:
                # Reflect relative velocity over the normal
                v_rel = v_rel - 2 * np.dot(v_rel, n) * n
                ball_vel = v_rel + v_wall
                ball_vel *= RESTITUTION

                # Correct the position to resolve penetration
                penetration = ball_radius - dist
                ball_pos += n * penetration

    # --- Additional Correction: Keep Ball Inside the Hexagon ---
    if not point_in_polygon(ball_pos, vertices):
        # Find the closest point on the hexagon's boundary
        min_dist = float('inf')
        closest_point = None
        for i in range(6):
            A = vertices[i]
            B = vertices[(i + 1) % 6]
            AB = B - A
            AP = ball_pos - A
            t = np.dot(AP, AB) / np.dot(AB, AB)
            t = max(0, min(1, t))
            pt = A + t * AB
            dist = np.linalg.norm(ball_pos - pt)
            if dist < min_dist:
                min_dist = dist
                closest_point = pt

        if closest_point is not None:
            # Compute the normal from the boundary toward the ball
            n = ball_pos - closest_point
            norm = np.linalg.norm(n)
            n = n / norm if norm != 0 else np.array([0, 1])
            # Reflect the ball's velocity off this boundary
            ball_vel = ball_vel - 2 * np.dot(ball_vel, n) * n
            ball_vel *= RESTITUTION
            # Reposition the ball so it's just inside the hexagon
            ball_pos = closest_point + n * ball_radius

    # --- Rendering ---
    screen.fill(BLACK)
    # Draw the hexagon (convert vertices to a list of tuples)
    hex_points = [vertex.tolist() for vertex in vertices]
    pygame.draw.polygon(screen, WHITE, hex_points, 2)
    # Draw the ball
    pygame.draw.circle(screen, RED, ball_pos.astype(int).tolist(), ball_radius)
    pygame.display.flip()

pygame.quit()
