import pygame
import math

pygame.init()
screen = pygame.display.set_mode((1000, 700))
pygame.display.set_caption("Roboterbein – Servo2 sichtbar eingezeichnet")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 22)

# ================= PARAMETER =================
L1 = 160
L2 = 140
ATTACH = 50
SERVO2_OFFSET = 60

STANGE_BASIS = 120
HEBEL = 40

SERVO1_MIN = -1
SERVO1_MAX = 181
SERVO2_MIN = -91
SERVO2_MAX = 47

origin = (500, 200)

servo1 = 0
servo2 = 90

def servo2_to_length(angle):
    return STANGE_BASIS + HEBEL * math.sin(math.radians(angle))

def foot_position(s1, s2):
    piston = servo2_to_length(s2)
    t1 = math.radians(s1)

    knee_x = origin[0] + L1 * math.cos(t1)
    knee_y = origin[1] + L1 * math.sin(t1)

    # === SERVO 2 BASIS (FEST AM OBERSCHENKEL) ===
    s2_x = origin[0] + SERVO2_OFFSET * math.cos(t1)
    s2_y = origin[1] + SERVO2_OFFSET * math.sin(t1)

    dx = s2_x - knee_x
    dy = s2_y - knee_y
    d = max(math.hypot(dx, dy), 1)

    a = (ATTACH**2 - piston**2 + d**2) / (2 * d)
    h2 = ATTACH**2 - a**2
    if h2 < 0:
        return None

    h = math.sqrt(h2)

    xm = knee_x + a * dx / d
    ym = knee_y + a * dy / d

    rx = -dy * (h / d)
    ry = dx * (h / d)

    attach_x = xm + rx
    attach_y = ym + ry

    ang = math.atan2(attach_y - knee_y, attach_x - knee_x)

    foot_x = knee_x + L2 * math.cos(ang)
    foot_y = knee_y + L2 * math.sin(ang)

    # === SERVO 2 HEBEL ===
    lever_angle = t1 + math.radians(s2 - 90)
    lever_x = s2_x + HEBEL * math.cos(lever_angle)
    lever_y = s2_y + HEBEL * math.sin(lever_angle)

    return knee_x, knee_y, s2_x, s2_y, lever_x, lever_y, attach_x, attach_y, foot_x, foot_y

running = True
while running:
    clock.tick(25)
    screen.fill((20,20,20))

    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False

    mx, my = pygame.mouse.get_pos()
    mouse_down = pygame.mouse.get_pressed()[0]

    if mouse_down:
        best_err = 1e9
        best = None

        for s1 in range(SERVO1_MIN, SERVO1_MAX+1, 2):
            for s2 in range(SERVO2_MIN, SERVO2_MAX+1, 2):
                res = foot_position(s1, s2)
                if not res:
                    continue
                fx, fy = res[8], res[9]
                err = math.hypot(fx-mx, fy-my)
                if err < best_err:
                    best_err = err
                    best = (s1, s2, res)

        if best:
            servo1, servo2, geom = best
    else:
        geom = foot_position(servo1, servo2)

    if geom:
        kx, ky, s2x, s2y, lx, ly, ax, ay, fx, fy = geom

        # === BEINE ===
        pygame.draw.line(screen, (180,180,180), origin, (kx, ky), 6)
        pygame.draw.line(screen, (100,200,255), (kx, ky), (fx, fy), 6)

        # === STANGE ===
        pygame.draw.line(screen, (200,0,200), (lx, ly), (ax, ay), 4)

        # === SERVO 2 ===
        pygame.draw.circle(screen, (255,0,0), (int(s2x), int(s2y)), 9)
        pygame.draw.line(screen, (255,200,0), (s2x, s2y), (lx, ly), 3)

        # === GELENKE ===
        pygame.draw.circle(screen, (255,0,0), origin, 8)
        pygame.draw.circle(screen, (0,255,0), (int(kx), int(ky)), 8)
        pygame.draw.circle(screen, (255,255,0), (int(fx), int(fy)), 10)

    pygame.draw.circle(screen, (255,255,255), (mx, my), 5)

    info = [
        f"Servo1 Hüfte: {servo1}°",
        f"Servo2 Hebel: {servo2}°",
        f"Stangenlänge: {int(servo2_to_length(servo2))}",
        "Servo2 = roter Kreis + gelber Hebel",
        "Maus halten = IK"
    ]
    for i,t in enumerate(info):
        screen.blit(font.render(t, True, (255,255,255)), (10, 10+i*22))

    pygame.display.flip()

pygame.quit()
