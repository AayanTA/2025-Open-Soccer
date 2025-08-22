import pygame
import math
import controller
import physics
import striker_behavior
import goalkeeper_behavior
from common import WIDTH, HEIGHT, BLUE, RED, BLACK, WHITE, GOAL_OPENING

# ---------- Config ----------
WIDTH, HEIGHT = 900, 600
FPS = 60

FIELD_COLOR = (28, 120, 28)
LINE_COLOR = (220, 255, 220)
GOAL_COLOR = (230, 210, 70)
BLUE = (60, 140, 240)
RED = (240, 80, 80)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

MAX_TRANSLATION_SPEED = 260.0
MAX_ANGULAR_SPEED = 180.0

BALL_RADIUS = 10
ROBOT_RADIUS = 28
GOAL_WALL = 10
GOAL_OPENING = 160

BALL_FRICTION = 0.7
BALL_RESTITUTION = 0.5

DRIBBLER_DEPTH = 0.8 * ROBOT_RADIUS
DRIBBLER_WIDTH = 1.2 * ROBOT_RADIUS
DRIBBLER_OFFSET = ROBOT_RADIUS + BALL_RADIUS - 3
DRIBBLER_BACKSPIN = 80.0
DRIBBLER_STICKINESS = 0.25

def clamp(v, lo, hi): return max(lo, min(hi, v))
def deg_to_rad(d): return d * math.pi / 180.0
def normalize_angle_deg(a):
    while a >= 180.0: a -= 360.0
    while a < -180.0: a += 360.0
    return a

def angle_to_target_deg(src_x, src_y, target_x, target_y):
    dx = target_x - src_x
    dy = target_y - src_y
    return (math.degrees(math.atan2(dx, -dy)) % 360.0)

# ---------- Entities ----------
class Ball:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.vx = self.vy = 0.0
        self.r = BALL_RADIUS
    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.vx, self.vy = physics.apply_friction(self.vx, self.vy, 1.0 - BALL_FRICTION, dt)
        self.x, self.y, self.vx, self.vy = physics.keep_in_bounds(
            self.x, self.y, self.vx, self.vy, self.r, WIDTH, HEIGHT, restitution=BALL_RESTITUTION)
    def draw(self, surf): pygame.draw.circle(surf, WHITE, (int(self.x), int(self.y)), self.r)

class Robot:
    def __init__(self, x, y, heading_deg, color, role="striker"):
        self.x, self.y, self.heading = x, y, heading_deg
        self.initial_heading = heading_deg
        self.color = color
        self.role = role
        self.r = ROBOT_RADIUS
        self.last_wheels = [0,0,0,0]
        self.last_translation = (0.0,0.0)
        self.last_rotation = 0.0
        self.last_translation_angle_deg = 0.0
        self.has_ball = False

    def update_kinematic(self, translation_angle_deg, translation_speed_norm, rotation_norm, dt):
        world_angle = (self.heading + translation_angle_deg) % 360.0
        rad = deg_to_rad(world_angle)
        speed = clamp(translation_speed_norm, 0.0, 1.0) * MAX_TRANSLATION_SPEED
        vx, vy = math.sin(rad)*speed, -math.cos(rad)*speed
        self.x += vx*dt; self.y += vy*dt
        ang_vel = clamp(rotation_norm, -1.0, 1.0) * MAX_ANGULAR_SPEED
        self.heading = (self.heading + ang_vel*dt) % 360.0
        self.last_translation = (vx,vy); self.last_rotation = ang_vel
        self.last_translation_angle_deg = translation_angle_deg
        self.x = clamp(self.x, self.r, WIDTH-self.r)
        self.y = clamp(self.y, self.r, HEIGHT-self.r)

    def draw(self, surf):
        pygame.draw.circle(surf, self.color, (int(self.x), int(self.y)), self.r)
        head = deg_to_rad(self.heading)
        hx = self.x + math.sin(head)*(self.r+6)
        hy = self.y - math.cos(head)*(self.r+6)
        pygame.draw.line(surf, BLACK, (int(self.x), int(self.y)), (int(hx), int(hy)), 3)
        role_text = "S" if self.role=="striker" else "G"
        font = pygame.font.SysFont(None, 18)
        surf.blit(font.render(role_text,True,WHITE),(int(self.x-5),int(self.y-8)))

# ---------- Menu ----------
def menu(screen, font):
    setup = {"blue": [], "red": []}
    side = "blue"
    cursor = 0
    while True:
        screen.fill((50,50,50))
        t1 = font.render("←/→ switch team | ↑ add | ↓ remove | [ / ] toggle role | ENTER start", True, WHITE)
        screen.blit(t1,(20,20))

        screen.blit(font.render("BLUE team:",True,BLUE),(40,60))
        for i,r in enumerate(setup["blue"]):
            mark = ">" if side=="blue" and cursor==i else " "
            screen.blit(font.render(f"{mark} Robot {i+1}: {r}",True,WHITE),(60,90+i*20))

        screen.blit(font.render("RED team:",True,RED),(40,180))
        for i,r in enumerate(setup["red"]):
            mark = ">" if side=="red" and cursor==i else " "
            screen.blit(font.render(f"{mark} Robot {i+1}: {r}",True,WHITE),(60,210+i*20))

        pygame.display.flip()

        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: pygame.quit(); exit()
            elif ev.type==pygame.KEYDOWN:
                if ev.key==pygame.K_RETURN: return setup
                elif ev.key in (pygame.K_LEFT,pygame.K_RIGHT):
                    side = "red" if side=="blue" else "blue"; cursor=0
                elif ev.key==pygame.K_UP and len(setup[side])<2:   # <= 2 per team
                    setup[side].append("striker"); cursor=len(setup[side])-1
                elif ev.key==pygame.K_DOWN and setup[side]:
                    setup[side].pop(); cursor=max(0,cursor-1)
                elif ev.key in (pygame.K_LEFTBRACKET,pygame.K_RIGHTBRACKET):
                    if setup[side]:
                        r=setup[side][cursor]
                        setup[side][cursor] = "goalkeeper" if r=="striker" else "striker"

# ---------- sim ----------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Open Soccer Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 22)

    setup = menu(screen,font)

    ball = Ball(WIDTH//2, HEIGHT//2)
    robots=[]
    for i,role in enumerate(setup["blue"]):
        y = HEIGHT*(0.4+0.2*i)
        robots.append(Robot(WIDTH*0.25,y,90.0,BLUE,role))
    for i,role in enumerate(setup["red"]):
        y = HEIGHT*(0.4+0.2*i)
        robots.append(Robot(WIDTH*0.75,y,270.0,RED,role))

    paused=True; dragging=None

    while True:
        dt = clock.tick(FPS)/1000.0
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: pygame.quit(); return
            elif ev.type==pygame.KEYDOWN:
                if ev.key==pygame.K_SPACE: paused=not paused
                elif ev.key==pygame.K_r:
                    ball.x,ball.y=WIDTH//2,HEIGHT//2; ball.vx=ball.vy=0
                    for i,rb in enumerate(robots):
                        if rb.color==BLUE: rb.x,rb.y,rb.heading=WIDTH*0.25,HEIGHT*(0.4+0.2*i),90.0
                        else: rb.x,rb.y,rb.heading=WIDTH*0.75,HEIGHT*(0.4+0.2*i),270.0
                        rb.has_ball=False
                # quick toggle example: '[' or ']' in menu only, left here intentionally empty

            elif ev.type==pygame.MOUSEBUTTONDOWN:
                mx,my=ev.pos
                if math.hypot(mx-ball.x,my-ball.y)<=ball.r+5: dragging=('ball',)
                else:
                    for i,rb in enumerate(robots):
                        if math.hypot(mx-rb.x,my-rb.y)<=rb.r+5: dragging=('robot',i); break
            elif ev.type==pygame.MOUSEBUTTONUP: dragging=None
            elif ev.type==pygame.MOUSEMOTION and dragging:
                if dragging[0]=='ball': ball.x,ball.y=ev.pos; ball.vx=ball.vy=0
                else: _,idx=dragging; robots[idx].x,robots[idx].y=ev.pos

        if not paused:
            for rb in robots:
                # --- detect ball capture at dribbler ---
                head = deg_to_rad(rb.heading)
                ux, uy = math.sin(head), -math.cos(head)
                front_x = rb.x + ux * (rb.r + 8)
                front_y = rb.y + uy * (rb.r + 8)
                rb.has_ball = (math.hypot(ball.x-front_x,ball.y-front_y)<20.0)

                # --- role behavior ---
                if rb.role == "goalkeeper":
                    aim,t_speed,rot_cmd,wheels = goalkeeper_behavior.step(rb, ball, dt)
                else:  # striker
                    aim,t_speed,rot_cmd,wheels = striker_behavior.step(rb, ball, dt)

                rb.last_wheels=wheels
                rb.update_kinematic(aim,t_speed,rot_cmd,dt)

                # --- robot ↔ ball collision (kept) ---
                ball.vx, ball.vy = physics.resolve_robot_ball_collision(
                    rb.x, rb.y, rb.last_translation[0], rb.last_translation[1], rb.r,
                    ball.x, ball.y, ball.vx, ball.vy, ball.r,
                    restitution=0.5, damping=0.97, separate=True)

                # --- dribbler stickiness (kept) ---
                if rb.has_ball:
                    px = rb.x + ux*DRIBBLER_OFFSET
                    py = rb.y + uy*DRIBBLER_OFFSET
                    ball.x = ball.x*(1.0-DRIBBLER_STICKINESS)+px*DRIBBLER_STICKINESS
                    ball.y = ball.y*(1.0-DRIBBLER_STICKINESS)+py*DRIBBLER_STICKINESS
                    ball.vx += -ux*DRIBBLER_BACKSPIN*dt
                    ball.vy += -uy*DRIBBLER_BACKSPIN*dt

            # --- robot ↔ robot collisions (kept) ---
            for i in range(len(robots)):
                for j in range(i+1,len(robots)):
                    rA,rB=robots[i],robots[j]
                    coll,dist,nx,ny=physics.circle_circle_collision_detection(rA.x,rA.y,rA.r,rB.x,rB.y,rB.r)
                    if coll and dist>1e-6:
                        overlap=(rA.r+rB.r)-dist
                        sep=overlap*0.5
                        rA.x-=nx*sep; rA.y-=ny*sep
                        rB.x+=nx*sep; rB.y+=ny*sep

            ball.update(dt)

        # ---- draw ----
        screen.fill(FIELD_COLOR)
        pygame.draw.line(screen,LINE_COLOR,(WIDTH//2,0),(WIDTH//2,HEIGHT),2)
        gh=GOAL_OPENING; gy0=HEIGHT//2-gh//2; gy1=HEIGHT//2+gh//2
        pygame.draw.rect(screen,GOAL_COLOR,(0,0,GOAL_WALL,gy0))
        pygame.draw.rect(screen,GOAL_COLOR,(0,gy1,GOAL_WALL,HEIGHT-gy1))
        pygame.draw.rect(screen,GOAL_COLOR,(WIDTH-GOAL_WALL,0,GOAL_WALL,gy0))
        pygame.draw.rect(screen,GOAL_COLOR,(WIDTH-GOAL_WALL,gy1,GOAL_WALL,HEIGHT-gy1))
        ball.draw(screen)
        for rb in robots: rb.draw(screen)
        screen.blit(font.render("SPACE run/pause | R reset | drag robots/ball",True,WHITE),(8,8))
        pygame.display.flip()

if __name__=="__main__": main()
