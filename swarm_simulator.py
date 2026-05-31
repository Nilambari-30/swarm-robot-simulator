"""
=============================================================================
swarm_simulator.py  —  Swarm Robot Movement Simulator
=============================================================================
Project Key Points Implemented:
  1. Boids algorithm (flocking) in Python — 30 robots
  2. Pygame real-time visualisation — cohesion, alignment, separation
  3. Obstacle avoidance layer
  4. Core vector math in C, called via Python ctypes

Controls:
  P       — Play / Pause
  R       — Reset simulation
  T       — Toggle trails
  + / -   — Increase / Decrease speed
  Mouse   — Click canvas to place an obstacle

Requirements:
  pip install pygame
  Build swarm_math.so first:  bash build.sh   (or build.bat on Windows)
=============================================================================
"""

# ── Standard library ──────────────────────────────────────────────────────────
import ctypes
import math
import os
import platform
import random
import sys

# ── Third-party ───────────────────────────────────────────────────────────────
import pygame


# =============================================================================
# SECTION 1 — Load the C shared library (ctypes bridge)
# =============================================================================

def load_c_library():
    """
    Load swarm_math.so / .dll from the same directory as this script.
    Raises FileNotFoundError with build instructions if missing.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    if platform.system() == "Windows":
        candidates = ["swarm_math.dll"]
    elif platform.system() == "Darwin":
        candidates = ["swarm_math.dylib", "swarm_math.so"]
    else:
        candidates = ["swarm_math.so"]

    for name in candidates:
        path = os.path.join(here, name)
        if os.path.exists(path):
            return ctypes.CDLL(path)

    raise FileNotFoundError(
        "\n[ERROR] C library not found. Build it first:\n"
        "  Linux / Mac : bash build.sh\n"
        "  Windows     : build.bat  (needs MinGW gcc)\n"
    )


# =============================================================================
# SECTION 2 — ctypes struct definitions
# These MUST match the structs in swarm_math.c exactly (field order & types).
# =============================================================================

class CRobot(ctypes.Structure):
    """Maps to typedef struct CRobot in swarm_math.c"""
    _fields_ = [
        ("x",    ctypes.c_double),   # position x
        ("y",    ctypes.c_double),   # position y
        ("vx",   ctypes.c_double),   # velocity x
        ("vy",   ctypes.c_double),   # velocity y
        ("id",   ctypes.c_int),      # unique robot id
        ("ci",   ctypes.c_int),      # colour index (0=amber, 1=rose, 2=emerald)
        ("_pad", ctypes.c_int),      # struct padding
    ]


class CObs(ctypes.Structure):
    """Maps to typedef struct CObs in swarm_math.c"""
    _fields_ = [
        ("x",      ctypes.c_double),  # centre x
        ("y",      ctypes.c_double),  # centre y
        ("radius", ctypes.c_double),  # bounding radius
        ("w",      ctypes.c_double),  # width  (rect only)
        ("h",      ctypes.c_double),  # height (rect only)
        ("shape",  ctypes.c_int),     # 0 = circle, 1 = rect
        ("_pad",   ctypes.c_int),
    ]


class CParams(ctypes.Structure):
    """Maps to typedef struct CParams in swarm_math.c"""
    _fields_ = [
        ("sep",     ctypes.c_double),  # separation weight
        ("ali",     ctypes.c_double),  # alignment  weight
        ("coh",     ctypes.c_double),  # cohesion   weight
        ("avoid_r", ctypes.c_double),  # obstacle-avoidance radius
        ("nr",      ctypes.c_double),  # neighbour-detection radius
        ("speed",   ctypes.c_double),  # target speed
        ("sim_w",   ctypes.c_int),     # canvas width
        ("sim_h",   ctypes.c_int),     # canvas height
        ("r_rad",   ctypes.c_int),     # robot radius (pixels)
        ("_pad",    ctypes.c_int),
    ]


def bind_c_functions(lib):
    """Set argtypes/restype for every exported C function we use."""
    # boids_step — core physics loop (runs entirely in C)
    lib.boids_step.restype  = None
    lib.boids_step.argtypes = [
        ctypes.POINTER(CRobot), ctypes.c_int,   # robots, count
        ctypes.POINTER(CObs),   ctypes.c_int,   # obstacles, count
        ctypes.POINTER(CParams),                # parameters
        ctypes.POINTER(ctypes.c_double),        # random noise buffer
    ]
    # compute_stats — average speed + cluster count
    lib.compute_stats.restype  = None
    lib.compute_stats.argtypes = [
        ctypes.POINTER(CRobot), ctypes.c_int,
        ctypes.c_double,                        # neighbour radius
        ctypes.POINTER(ctypes.c_double),        # out: avg speed
        ctypes.POINTER(ctypes.c_int),           # out: clustered count
    ]
    # Exported vector utilities (available for direct Python use)
    lib.vec2_len.restype   = ctypes.c_double
    lib.vec2_len.argtypes  = [ctypes.c_double, ctypes.c_double]
    lib.vec2_dist.restype  = ctypes.c_double
    lib.vec2_dist.argtypes = [ctypes.c_double, ctypes.c_double,
                               ctypes.c_double, ctypes.c_double]


# =============================================================================
# SECTION 3 — Constants & colour palette
# =============================================================================

WINDOW_W, WINDOW_H = 1280, 720
PANEL_W             = 260
SIM_W               = WINDOW_W - PANEL_W   # usable simulation area
FPS                 = 60
ROBOT_RADIUS        = 8
OBS_RADIUS_BASE     = 22

# Three robot colour groups
COLORS = [(232, 131, 42),   # Amber
          (217, 107, 158),   # Rose
          (61,  190, 122)]   # Emerald
BG_COLOR = (7, 9, 26)


# =============================================================================
# SECTION 4 — Simulation helpers
# =============================================================================

def random_velocity(speed: float) -> tuple[float, float]:
    """Return a velocity vector with random direction and given magnitude."""
    angle = random.random() * math.pi * 2
    return math.cos(angle) * speed, math.sin(angle) * speed


def make_params(sep, ali, coh, avoid_r, nr, speed) -> CParams:
    p = CParams()
    p.sep = sep;  p.ali = ali;  p.coh = coh
    p.avoid_r = avoid_r;  p.nr = nr;  p.speed = speed
    p.sim_w = SIM_W;  p.sim_h = WINDOW_H;  p.r_rad = ROBOT_RADIUS
    p._pad  = 0
    return p


def spawn_obstacles(count: int, lib) -> list[CObs]:
    """Generate `count` random obstacles (mix of circles and rects)."""
    out = []
    for i in range(count):
        is_rect = random.random() >= 0.5
        r = OBS_RADIUS_BASE + random.random() * 14
        w = (38 + random.random() * 32) if is_rect else r * 2
        h = (26 + random.random() * 20) if is_rect else r * 2
        pad = max(w, h) / 2 + 12
        x, y = SIM_W / 2, WINDOW_H / 2
        for _ in range(40):
            tx = pad + random.random() * (SIM_W  - pad * 2)
            ty = pad + random.random() * (WINDOW_H - pad * 2)
            if all(lib.vec2_dist(o.x, o.y, tx, ty) >= o.radius + r + 20 for o in out):
                x, y = tx, ty
                break
        o = CObs()
        o.x = x; o.y = y; o.radius = r
        o.w = w; o.h = h
        o.shape = 1 if is_rect else 0
        o._pad  = 0
        out.append(o)
    return out


def spawn_robots(count: int, speed: float, obstacles: list[CObs], lib) -> list[CRobot]:
    """Spawn `count` robots at valid positions (not inside obstacles)."""
    out = []
    for i in range(count):
        x, y = SIM_W / 2.0, WINDOW_H / 2.0
        for _ in range(50):
            tx = ROBOT_RADIUS * 2 + random.random() * (SIM_W   - ROBOT_RADIUS * 4)
            ty = ROBOT_RADIUS * 2 + random.random() * (WINDOW_H - ROBOT_RADIUS * 4)
            clear = all(lib.vec2_dist(o.x, o.y, tx, ty) >= o.radius + ROBOT_RADIUS * 3
                        for o in obstacles)
            if clear:
                x, y = tx, ty
                break
        vx, vy = random_velocity(speed)
        r = CRobot()
        r.id = i; r.ci = i % 3
        r.x = x; r.y = y
        r.vx = vx; r.vy = vy
        r._pad = 0
        out.append(r)
    return out


# =============================================================================
# SECTION 5 — Simulator (Pygame window + game-loop)
# =============================================================================

class SwarmSimulator:
    """
    Main simulator class.
    - Holds simulation state (robots, obstacles, parameters)
    - Calls C boids physics via ctypes each frame
    - Renders everything with Pygame
    """

    def __init__(self, lib):
        self.lib = lib
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption(
            "Swarm Robot Movement Simulator — Python + C ctypes")
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont("Arial", 11)
        self.font_m = pygame.font.SysFont("Arial", 13, bold=True)
        self.font_l = pygame.font.SysFont("Arial", 16, bold=True)

        # ── Simulation parameters ─────────────────────────────────────────
        self.running  = False     # paused at start
        self.trails   = True
        self.speed    = 2.2
        self.sep      = 1.6       # separation strength
        self.ali      = 1.0       # alignment strength
        self.coh      = 0.9       # cohesion strength
        self.avoid_r  = 55.0      # obstacle avoidance radius (px)
        self.nr       = 80.0      # neighbour detection radius (px)

        # ── Runtime stats ─────────────────────────────────────────────────
        self.tick      = 0
        self.avg_speed = 0.0
        self.clustered = 0

        # ── Agents ───────────────────────────────────────────────────────
        self.obstacles:   list[CObs]   = []
        self.robots:      list[CRobot] = []
        self.trail_data:  list[list]   = []   # per-robot trail points

        self.reset()

    # ── Init / Reset ──────────────────────────────────────────────────────────
    def reset(self):
        self.obstacles  = spawn_obstacles(8, self.lib)
        self.robots     = spawn_robots(30, self.speed, self.obstacles, self.lib)
        self.trail_data = [[] for _ in self.robots]
        self.tick = 0;  self.avg_speed = 0.0;  self.clustered = 0

    # ── Physics step (C does the heavy lifting) ───────────────────────────────
    def physics_step(self):
        """
        Calls boids_step() in C via ctypes.
        C function implements:
          Rule 1 — Separation  (avoid crowding neighbours)
          Rule 2 — Alignment   (steer toward average heading)
          Rule 3 — Cohesion    (steer toward average position)
          + Obstacle avoidance & boundary bounce
        """
        n   = len(self.robots)
        n_o = len(self.obstacles)

        # Build flat ctypes arrays to pass into C
        robot_arr = (CRobot * n)(*self.robots)
        obs_arr   = ((CObs * n_o)(*self.obstacles)
                     if n_o > 0 else (CObs * 1)())

        params = make_params(self.sep, self.ali, self.coh,
                             self.avoid_r, self.nr, self.speed)

        # Small random noise values — generated in Python, consumed in C
        rand_vals = [random.random() - 0.5 for _ in range(n * 2)]
        rand_arr  = (ctypes.c_double * (n * 2))(*rand_vals)

        # ⚡ C executes the full O(n²) boids loop here
        self.lib.boids_step(robot_arr, n, obs_arr, n_o,
                            ctypes.byref(params), rand_arr)

        # Write updated positions/velocities back to Python list
        for i in range(n):
            self.robots[i] = robot_arr[i]

        # Update trails (lightweight, stays in Python)
        if self.trails:
            for i, r in enumerate(self.robots):
                trail = self.trail_data[i]
                trail.append((r.x, r.y))
                if len(trail) > 22:
                    del trail[0]
        else:
            for t in self.trail_data:
                t.clear()

        self.tick += 1

        # Recompute stats every 30 frames (also done in C)
        if self.tick % 30 == 0 and n > 0:
            avg_out = ctypes.c_double(0.0)
            cl_out  = ctypes.c_int(0)
            self.lib.compute_stats(robot_arr, n, self.nr,
                                   ctypes.byref(avg_out),
                                   ctypes.byref(cl_out))
            self.avg_speed = avg_out.value
            self.clustered = cl_out.value

    # ── Drawing ───────────────────────────────────────────────────────────────
    def _draw_robot(self, x: float, y: float, angle: float, ci: int):
        """Draw a single car-shaped robot at (x,y) facing `angle` radians."""
        col = COLORS[ci]
        cw, ch = 11, 18
        surf = pygame.Surface((cw + 12, ch + 12), pygame.SRCALPHA)
        cx2, cy2 = (cw + 12) // 2, (ch + 12) // 2

        # Body
        pygame.draw.rect(surf, (*col, 220),
                         (cx2-cw//2, cy2-ch//2, cw, ch), border_radius=4)
        pygame.draw.rect(surf, (255, 255, 255, 70),
                         (cx2-cw//2, cy2-ch//2, cw, ch), 1, border_radius=4)
        # Windshield
        pygame.draw.rect(surf, (180, 220, 255, 150),
                         (cx2-cw//2+2, cy2-ch//2+3, cw-4, int(ch*0.38)),
                         border_radius=2)
        # Headlights
        pygame.draw.circle(surf, (255,245,150), (cx2-cw//2+2, cy2-ch//2+2), 2)
        pygame.draw.circle(surf, (255,245,150), (cx2+cw//2-2, cy2-ch//2+2), 2)
        # Tail-lights
        pygame.draw.circle(surf, (255, 80, 80), (cx2-cw//2+2, cy2+ch//2-2), 2)
        pygame.draw.circle(surf, (255, 80, 80), (cx2+cw//2-2, cy2+ch//2-2), 2)

        rot  = pygame.transform.rotate(surf, -math.degrees(angle))
        glow = pygame.Surface((36, 36), pygame.SRCALPHA)
        pygame.draw.circle(glow, (*col, 35), (18, 18), 16)
        self.screen.blit(glow, (int(x)-18, int(y)-18))
        self.screen.blit(rot, rot.get_rect(center=(int(x), int(y))))

    def _draw_obstacle(self, obs: CObs):
        x, y = int(obs.x), int(obs.y)
        if obs.shape == 0:  # circle
            R = int(obs.radius)
            glow = pygame.Surface((R*4, R*4), pygame.SRCALPHA)
            pygame.draw.circle(glow, (80,120,255,25), (R*2,R*2), R*2)
            self.screen.blit(glow, (x-R*2, y-R*2))
            pygame.draw.circle(self.screen, (25, 42, 110), (x, y), R)
            pygame.draw.circle(self.screen, (150,190,255), (x, y), R, 2)
        else:               # rect
            hw, hh = int(obs.w/2), int(obs.h/2)
            rect = pygame.Rect(x-hw, y-hh, int(obs.w), int(obs.h))
            pygame.draw.rect(self.screen, (25, 42, 110), rect, border_radius=8)
            pygame.draw.rect(self.screen, (140,180,255), rect, 2, border_radius=8)

    def _draw_minimap(self):
        MW, MH, PAD = 160, 90, 14
        mx, my = SIM_W - MW - PAD, WINDOW_H - MH - PAD
        mm = pygame.Surface((MW, MH), pygame.SRCALPHA)
        mm.fill((8, 14, 45, 210))
        sx, sy = MW / SIM_W, MH / WINDOW_H
        for obs in self.obstacles:
            ox2, oy2 = int(obs.x*sx), int(obs.y*sy)
            if obs.shape == 0:
                pygame.draw.circle(mm,(80,110,220,180),(ox2,oy2),max(2,int(obs.radius*sx)))
            else:
                rw, rh = max(4,int(obs.w*sx)), max(3,int(obs.h*sy))
                pygame.draw.rect(mm,(80,110,220,180),(ox2-rw//2,oy2-rh//2,rw,rh))
        for r in self.robots:
            col = COLORS[r.ci]
            pygame.draw.circle(mm,(*col,200),(int(r.x*sx),int(r.y*sy)),2)
        self.screen.blit(mm, (mx, my))
        pygame.draw.rect(self.screen,(120,170,255),(mx,my,MW,MH),1)
        self.screen.blit(self.font_s.render('MINI MAP',True,(140,170,255)),(mx+4,my+3))
        cnt = self.font_s.render(f'{len(self.robots)} robots',True,(180,210,255))
        self.screen.blit(cnt,(mx+MW-cnt.get_width()-4, my+MH-14))

    def _draw_panel(self):
        panel = pygame.Surface((PANEL_W, WINDOW_H), pygame.SRCALPHA)
        panel.fill((15, 20, 55, 220))
        self.screen.blit(panel, (SIM_W, 0))
        pygame.draw.line(self.screen,(60,80,180),(SIM_W,0),(SIM_W,WINDOW_H),1)

        px, y = SIM_W + 14, 16
        self.screen.blit(self.font_l.render('SWARM SIMULATOR',True,(160,190,255)),(px,y)); y+=18
        self.screen.blit(self.font_s.render('Python + C ctypes Edition',True,(100,130,200)),(px,y)); y+=16
        pygame.draw.line(self.screen,(50,70,150),(px,y),(SIM_W+PANEL_W-14,y)); y+=10

        # Live stats
        stats = [('Robots',    str(len(self.robots)),   COLORS[0]),
                 ('Obstacles', str(len(self.obstacles)), (100,130,255)),
                 ('Avg Speed', f'{self.avg_speed:.2f}',  COLORS[2]),
                 ('Clustered', str(self.clustered),      COLORS[1])]
        for lbl, val, col in stats:
            self.screen.blit(self.font_s.render(lbl, True,(140,160,210)),(px,y))
            self.screen.blit(self.font_m.render(val, True, col),(px+110,y)); y+=17
        y+=8
        pygame.draw.line(self.screen,(50,70,150),(px,y),(SIM_W+PANEL_W-14,y)); y+=10

        # C library badge
        badge = self.font_s.render('⚡ Physics: C library via ctypes',True,(130,220,140))
        self.screen.blit(badge,(px,y)); y+=18
        pygame.draw.line(self.screen,(50,70,150),(px,y),(SIM_W+PANEL_W-14,y)); y+=10

        # Boids rules info
        self.screen.blit(self.font_s.render('BOIDS RULES (C)',True,(120,150,220)),(px,y)); y+=14
        rules = [('■','Separation','avoid crowding'),
                 ('■','Alignment', 'match heading'),
                 ('■','Cohesion',  'stay together')]
        rule_cols = [COLORS[0], COLORS[1], COLORS[2]]
        for (dot, name, desc), col in zip(rules, rule_cols):
            self.screen.blit(self.font_s.render(f'{name}', True, col),(px+8,y))
            self.screen.blit(self.font_s.render(f'— {desc}',True,(140,160,210)),(px+80,y)); y+=13
        y+=6
        pygame.draw.line(self.screen,(50,70,150),(px,y),(SIM_W+PANEL_W-14,y)); y+=10

        # Controls
        self.screen.blit(self.font_s.render('CONTROLS',True,(120,150,220)),(px,y)); y+=15
        for k, d in [('P','Play / Pause'),('R','Reset'),('T','Toggle Trails'),
                     ('+','Speed Up'),('-','Speed Down'),('Click','Add Obstacle')]:
            self.screen.blit(self.font_s.render(f'[{k}]',True,(99,179,237)),(px,y))
            self.screen.blit(self.font_s.render(d,True,(160,180,220)),(px+50,y)); y+=15
        y+=6
        pygame.draw.line(self.screen,(50,70,150),(px,y),(SIM_W+PANEL_W-14,y)); y+=10

        # Parameters + bar graphs
        self.screen.blit(self.font_s.render('PARAMETERS',True,(120,150,220)),(px,y)); y+=15
        for name, val, lo, hi, col in [
                ('Speed',     self.speed, 0.5, 5.0, COLORS[2]),
                ('Separation',self.sep,   0,   4,   COLORS[0]),
                ('Alignment', self.ali,   0,   4,   COLORS[1]),
                ('Cohesion',  self.coh,   0,   4,   COLORS[2])]:
            self.screen.blit(self.font_s.render(f'{name}: {val:.1f}',True,col),(px,y))
            bw = int((val-lo)/(hi-lo)*(PANEL_W-28))
            pygame.draw.rect(self.screen,(40,50,100),(px,y+13,PANEL_W-28,4),border_radius=2)
            pygame.draw.rect(self.screen,col,(px,y+13,bw,4),border_radius=2); y+=26

        # Status bar
        sc = (61,190,122) if self.running else (252,129,74)
        self.screen.blit(self.font_m.render('RUNNING' if self.running else 'PAUSED',True,sc),(px,WINDOW_H-28))
        self.screen.blit(self.font_s.render('Trails:'+('ON' if self.trails else 'OFF'),True,(140,200,255)),(px+95,WINDOW_H-28))

    def _draw_frame(self):
        """Render one complete frame."""
        self.screen.fill(BG_COLOR)

        # Background grid
        for gx in range(0, SIM_W, 60):
            a = 55 if gx % 300 == 0 else 18
            pygame.draw.line(self.screen,(80,120,220,a),(gx,0),(gx,WINDOW_H))
        for gy in range(0, WINDOW_H, 60):
            a = 55 if gy % 300 == 0 else 18
            pygame.draw.line(self.screen,(80,120,220,a),(0,gy),(SIM_W,gy))

        # Obstacles
        for obs in self.obstacles:
            self._draw_obstacle(obs)

        # Trails (fading history of each robot's path)
        if self.trails:
            for i, r in enumerate(self.robots):
                trail = self.trail_data[i]
                if len(trail) >= 2:
                    col = COLORS[r.ci]
                    for t in range(1, len(trail)):
                        a = int((t / len(trail)) * 110)
                        pygame.draw.line(self.screen, (*col, a),
                                         (int(trail[t-1][0]), int(trail[t-1][1])),
                                         (int(trail[t][0]),   int(trail[t][1])), 1)

        # Robots
        for r in self.robots:
            self._draw_robot(r.x, r.y, math.atan2(r.vy, r.vx), r.ci)

        # Mini-map (top-down overview)
        self._draw_minimap()

        # Side panel
        self._draw_panel()

        # Pause overlay
        if not self.running:
            msg  = self.font_l.render(
                "Press  P  to start   |   Click canvas to add obstacles",
                True, (180,210,255))
            box  = pygame.Surface((msg.get_width()+30, msg.get_height()+20), pygame.SRCALPHA)
            box.fill((20,30,80,180))
            bx   = SIM_W//2 - box.get_width()//2
            by   = WINDOW_H//2 - box.get_height()//2
            self.screen.blit(box, (bx, by))
            self.screen.blit(msg, (bx+15, by+10))

        pygame.display.flip()

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        while True:
            # ── Event handling ──────────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type == pygame.KEYDOWN:
                    if   event.key == pygame.K_p:
                        self.running = not self.running
                    elif event.key == pygame.K_r:
                        self.running = False
                        self.reset()
                    elif event.key == pygame.K_t:
                        self.trails = not self.trails
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                        self.speed = min(5.0, round(self.speed + 0.2, 1))
                    elif event.key == pygame.K_MINUS:
                        self.speed = max(0.5, round(self.speed - 0.2, 1))

                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    mx, my = event.pos
                    if mx < SIM_W:   # only add obstacle inside simulation area
                        is_rect = random.random() >= 0.5
                        o = CObs()
                        o.radius = OBS_RADIUS_BASE + random.random() * 10
                        o.shape  = 1 if is_rect else 0
                        o.w = (38 + random.random()*28) if is_rect else o.radius*2
                        o.h = (26 + random.random()*18) if is_rect else o.radius*2
                        o.x = mx; o.y = my; o._pad = 0
                        self.obstacles.append(o)

            # ── Physics (runs in C) ─────────────────────────────────────────
            if self.running:
                self.physics_step()

            # ── Render ──────────────────────────────────────────────────────
            self._draw_frame()
            self.clock.tick(FPS)


# =============================================================================
# SECTION 6 — Entry point
# =============================================================================

if __name__ == "__main__":
    try:
        lib = load_c_library()
    except FileNotFoundError as e:
        print(e)
        sys.exit(1)

    bind_c_functions(lib)
    SwarmSimulator(lib).run()
