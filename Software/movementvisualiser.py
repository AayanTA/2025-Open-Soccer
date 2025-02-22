import pygame
import math
import sys

# Constants
PI = math.pi
DEG2RAD = PI / 180.0

# Window and robot parameters
window_width, window_height = 800, 600
window_center = (window_width // 2, window_height // 2)
robot_radius = 100  # pixels

# Motor mounting positions (in degrees) and effective force directions (in degrees)
motor_mount_angles = [45, 135, 225, 315]     # Where motors are placed on robot circumference
motor_force_angles = [135, 225, 315, 45]       # Effective drive directions

# Colors
BLACK = (0, 0, 0)
GRAY = (50, 50, 50)
YELLOW = (255, 255, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)

# Visualization scales
arrow_scale = 80.0   # Scale for motor force arrows
net_scale = 100.0    # Scale for net force arrow

# Desired translation parameters
desired_angle_deg = 90.0  # initial desired direction (0=forward, 90=right, etc.)
translation_speed = 1.0   # normalized (0 to 1)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Omni-Drive Visualization")
clock = pygame.time.Clock()

# Load a font
font = pygame.font.SysFont("Arial", 16)

def draw_arrow(surface, start, end, color):
    """ Draw an arrow from start to end on the given surface. """
    pygame.draw.line(surface, color, start, end, 2)
    # Compute the direction vector
    diff = (end[0] - start[0], end[1] - start[1])
    length = math.hypot(diff[0], diff[1])
    if length == 0:
        return
    unit = (diff[0] / length, diff[1] / length)
    # Arrowhead parameters
    head_length = 10
    head_angle = 30 * DEG2RAD
    # Left arrowhead line
    left_x = end[0] - head_length * (unit[0] * math.cos(head_angle) - unit[1] * math.sin(head_angle))
    left_y = end[1] - head_length * (unit[0] * math.sin(head_angle) + unit[1] * math.cos(head_angle))
    # Right arrowhead line
    right_x = end[0] - head_length * (unit[0] * math.cos(-head_angle) - unit[1] * math.sin(-head_angle))
    right_y = end[1] - head_length * (unit[0] * math.sin(-head_angle) + unit[1] * math.cos(-head_angle))
    pygame.draw.line(surface, color, end, (left_x, left_y), 2)
    pygame.draw.line(surface, color, end, (right_x, right_y), 2)

def draw_text(surface, text, pos, color=WHITE):
    """ Draw text on surface at pos. """
    text_surface = font.render(text, True, color)
    surface.blit(text_surface, pos)

# Main loop
running = True
while running:
    # Event handling: allow adjusting desired angle with left/right keys, quit on window close.
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                desired_angle_deg = (desired_angle_deg - 5) % 360
            elif event.key == pygame.K_RIGHT:
                desired_angle_deg = (desired_angle_deg + 5) % 360

    # Clear screen
    screen.fill(BLACK)

    # Draw robot body (circle)
    pygame.draw.circle(screen, GRAY, window_center, int(robot_radius), 0)

    # Compute desired translation direction in radians
    desired_angle_rad = math.radians(desired_angle_deg)

    # For each motor, compute its output and effective force vector
    motor_outputs = []
    motor_force_vectors = []
    for i in range(4):
        # Get motor's effective force angle in radians
        force_angle_rad = math.radians(motor_force_angles[i])
        # Compute output = translation_speed * sin(desired_angle - motor_force_angle)
        output = translation_speed * math.sin(desired_angle_rad - force_angle_rad)
        motor_outputs.append(output)
        # Compute effective force vector (output times unit vector in force direction)
        force_vector = (output * math.cos(force_angle_rad), output * math.sin(force_angle_rad))
        motor_force_vectors.append(force_vector)

    # Compute net force as the sum of the motor force vectors
    net_force = [0.0, 0.0]
    for vec in motor_force_vectors:
        net_force[0] += vec[0]
        net_force[1] += vec[1]

    # Draw motors, their arrows, and labels
    for i in range(4):
        # Compute motor position on robot circumference (using mounting angle)
        mount_rad = math.radians(motor_mount_angles[i])
        motor_x = window_center[0] + robot_radius * math.cos(mount_rad)
        motor_y = window_center[1] + robot_radius * math.sin(mount_rad)
        motor_pos = (motor_x, motor_y)
        # Draw motor as a small circle
        pygame.draw.circle(screen, YELLOW, (int(motor_x), int(motor_y)), 8)
        # Compute arrow end point from motor_pos based on force vector and arrow_scale
        arrow_end = (motor_x + motor_force_vectors[i][0] * arrow_scale,
                     motor_y + motor_force_vectors[i][1] * arrow_scale)
        draw_arrow(screen, motor_pos, arrow_end, GREEN)
        # Draw text label for motor output
        label = f"M{i+1}: {motor_outputs[i]:.2f}"
        draw_text(screen, label, (motor_x + 10, motor_y + 10), WHITE)

    # Draw net force arrow from the center
    net_end = (window_center[0] + net_force[0] * net_scale,
               window_center[1] + net_force[1] * net_scale)
    draw_arrow(screen, window_center, net_end, RED)
    net_mag = math.hypot(net_force[0], net_force[1])
    net_angle = math.degrees(math.atan2(net_force[1], net_force[0]))
    net_text = f"Net: Mag={net_mag:.2f}, Angle={net_angle:.2f} deg"
    draw_text(screen, net_text, (10, 10), RED)

    # Draw desired translation angle text
    desired_text = f"Desired Angle: {desired_angle_deg:.2f} deg"
    draw_text(screen, desired_text, (10, 30), CYAN)

    # Update display
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()
