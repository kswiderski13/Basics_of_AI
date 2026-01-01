# imports
import pygame
from pygame.math import Vector2
from pygame.locals import *
import sys
import classes2

pygame.init()
vector = pygame.math.Vector2

#setup
height = 800
width = 580
fps = 60
acceleration = 0.5
friction = -0.1
wall_color = (0, 100, 255)

#walls
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

def handle_wall_collision_entity(entity, walls, thickness=10):
    if hasattr(entity, 'collider'):
        for wall in walls:
            if entity.collider.colliderect(wall):
                if wall.top == 0 and wall.height == thickness:
                    entity.pos.y = wall.bottom
                    entity.vel.y = 0
                elif wall.bottom == height and wall.height == thickness:
                    entity.pos.y = wall.top - entity.collider.height
                    entity.vel.y = 0
                elif wall.left == 0 and wall.width == thickness:
                    entity.pos.x = wall.right
                    entity.vel.x = 0
                elif wall.right == width and wall.width == thickness:
                    entity.pos.x = wall.left - entity.collider.width
                    entity.vel.x = 0
                entity.collider.topleft = (entity.pos.x, entity.pos.y)
    else:
        for wall in walls:
            if wall.left == 0 and wall.width == thickness:
                min_x = wall.right + entity.radius
                if entity.pos.x < min_x:
                    entity.pos.x = min_x
                    entity.vel.x = 0
            elif wall.right == width and wall.width == thickness:
                max_x = wall.left - entity.radius
                if entity.pos.x > max_x:
                    entity.pos.x = max_x
                    entity.vel.x = 0
            elif wall.top == 0 and wall.height == thickness:
                min_y = wall.bottom + entity.radius
                if entity.pos.y < min_y:
                    entity.pos.y = min_y
                    entity.vel.y = 0
            elif wall.bottom == height and wall.height == thickness:
                max_y = wall.top - entity.radius
                if entity.pos.y > max_y:
                    entity.pos.y = max_y
                    entity.vel.y = 0

#triangle coords (player)
triangle = [
    (10, 10),
    (30, 10),
    (20, 30)
]

framepersecond = pygame.time.Clock()

display = pygame.display.set_mode((width, height))
pygame.display.set_caption("DM_exercise_2")

#player and obstacles
Player = classes2.PlayerChar(display, triangle)

obstacle1 = classes2.Obstacle(20, 100, 60, display)
obstacle2 = classes2.Obstacle(20, 200, 300, display)
obstacle3 = classes2.Obstacle(20, 150, 150, display)
obstacles = [obstacle1, obstacle2, obstacle3]

bullets = []

#main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            b = Player.shoot()
            if b is not None:
                bullets.append(b)

    dt = framepersecond.tick(fps) / 1000.0

    display.fill((0, 0, 0))
    walls = draw_walls(display)

    Player.draw()
    for o in obstacles:
        o.draw()

    Player.move(acceleration, friction, obstacles)
    classes2.resolve_wall_penetration(Player, walls)

    for b in bullets[:]:
        b.update(dt)
        b.draw(display)

    pygame.display.update()
