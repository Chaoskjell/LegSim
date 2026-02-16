import pygame
import math

pygame.init()
screen = pygame.display.set_mode((900, 600))
pygame.display.set_caption("Inverse Kinematik – Roboterbein")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

# Beinlängen
L1 = 170
L2 = 150

# Servo-Kalibrierung
HIP_OFFSET = 0
KNEE_OFFSET = 90
KNEE_GAIN = 1.0

origin = (450, 200)

def clamp(val, minv, maxv):
    return max(minv, min(maxv, val))

def rad2deg(r):
    return r * 180 / math.pi

running = True
target = list(origin)

while running:
    clock.tick(60)
    screen.fill((20, 20, 20))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Mausziel
    if pygame.mouse.get_pressed()[0]:
        target = list(pygame.mouse.get_pos())

    # Ziel relativ zur Hüfte
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    dist = math.hypot(dx, dy)

    # Reichweite begrenzen
    max_reach = L1 + L2 - 1
    min_reach = abs(L1 - L2) + 1
    dist = clamp(dist, min_reach, max_reach)

    # Inverse Kinematik
    cos_theta2 = (dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = clamp(cos_theta2, -1, 1)
    theta2 = math.acos(cos_theta2)

    theta1 = math.atan2(dy, dx) - math.atan2(
        L2 * math.sin(theta2),
        L1 + L2 * math.cos(theta2)
    )

    # Vorwärtskinematik (zum Zeichnen)
    knee_x = origin[0] + L1 * math.cos(theta1)
    knee_y = origin[1] + L1 * math.sin(theta1)

    foot_x = knee_x + L2 * math.cos(theta1 + theta2)
    foot_y = knee_y + L2 * math.sin(theta1 + theta2)

    # Servo-Werte
    servo_hip = int(rad2deg(theta1) + HIP_OFFSET)
    servo_knee = int(rad2deg(theta2) * KNEE_GAIN + KNEE_OFFSET)

    # Zeichnen
    pygame.draw.line(screen, (200,200,200), origin, (knee_x, knee_y), 6)
    pygame.draw.line(screen, (100,200,255), (knee_x, knee_y), (foot_x, foot_y), 6)

    pygame.draw.circle(screen, (255,0,0), origin, 8)
    pygame.draw.circle(screen, (0,255,0), (int(knee_x), int(knee_y)), 8)
    pygame.draw.circle(screen, (255,255,0), (int(foot_x), int(foot_y)), 10)

    pygame.draw.circle(screen, (255,100,100), target, 6)

    # Text
    info = [
        f"Servo Hüfte: {servo_hip}°",
        f"Servo Knie:  {servo_knee}°",
        f"Fuß X: {int(foot_x - origin[0])}",
        f"Fuß Y: {int(foot_y - origin[1])}",
        "Linke Maustaste = Fuß bewegen"
    ]

    for i, t in enumerate(info):
        screen.blit(font.render(t, True, (255,255,255)), (10, 10+i*22))

    pygame.display.flip()

pygame.quit()
