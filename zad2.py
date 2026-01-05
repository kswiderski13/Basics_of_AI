# imports
import pygame
from pygame.math import Vector2
from pygame.locals import *
import sys
import classes2
from classes2 import dummybot

pygame.init()
vector = pygame.math.Vector2

# setup
height = 800
width = 580
fps = 60
wall_color = (0, 100, 255)

# walls
def draw_walls(screen):
    thickness = 10
    top = pygame.Rect(0, 0, width, thickness)
    bottom = pygame.Rect(0, height - thickness, width, thickness)
    left = pygame.Rect(0, 0, thickness, height)
    right = pygame.Rect(width - thickness, 0, thickness, height)

    pygame.draw.rect(screen, wall_color, top)
    pygame.draw.rect(screen, wall_color, bottom)
    pygame.draw.rect(screen, wall_color, left)
    pygame.draw.rect(screen, wall_color, right)

    return [top, bottom, left, right]

framepersecond = pygame.time.Clock()

display = pygame.display.set_mode((width, height))
pygame.display.set_caption("DM_exercise_2")

# obstacles (na razie okręgi)
obstacle1 = classes2.Obstacle(40, 150, 200, display)
obstacle2 = classes2.Obstacle(50, 350, 400, display)
obstacle3 = classes2.Obstacle(35, 420, 150, display)
obstacles = [obstacle1, obstacle2, obstacle3]

MAP_RECT = pygame.Rect(0, 0, width, height)
BOT_RADIUS = 15

nav_graph = classes2.build_nav_graph_flood_fill(
    start_pos=Vector2(50, 50),
    obstacles=obstacles,
    map_rect=MAP_RECT,
    bot_radius=BOT_RADIUS
)

# spawn pointy
spawn_points = [
    Vector2(80, 80),
    Vector2(width - 80, 80),
    Vector2(80, height - 80),
    Vector2(width - 80, height - 80)
]

bots = []
for sp in spawn_points:
    b = dummybot(sp, display)
    b.planner = classes2.PathPlanner(b, nav_graph)
    bots.append(b)

bullets = []
rockets = []
pickups = classes2.spawn_pickups(display, MAP_RECT, obstacles)

# main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    dt = framepersecond.tick(fps) / 1000.0

    display.fill((0, 0, 0))
    walls = draw_walls(display)

    for o in obstacles:
        o.draw()

    classes2.draw_nav_graph(display, nav_graph)

    for p in pickups[:]:
        p.draw(display)

    for b in bullets[:]:
        b.update(dt)
        b.draw(display)
        if not MAP_RECT.collidepoint(b.pos.x, b.pos.y):
            bullets.remove(b)

    for r in rockets[:]:
        exploded = r.update(dt, bots)
        r.draw(display)
        if exploded or not MAP_RECT.collidepoint(r.pos.x, r.pos.y):
            rockets.remove(r)

    for bot in bots:
        bot.update(dt, bots, obstacles, pickups, bullets, rockets, MAP_RECT)
        bot.draw(display)

    pygame.display.update()



    #TO DO:
    #zachowanie botów, bo są upo teraz
    #zakończenie gry, kiedy zostanie ostatni bot i restart
    #zamiana okręgów na wielokąty i ich rozstawienie jako przeszkody w coś typu labirynt


