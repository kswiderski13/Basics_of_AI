#imports
import pygame
from pygame.math import Vector2
from pygame.locals import *
import sys
import classes
#import math
from classes import check_collision

pygame.init()
vector = pygame.math.Vector2

#setup
height = 960
width = 580
fps = 60
acceleration = 0.5
friction = -0.1

#triangle coords
triangle = [(10,10),
            (30,10),
            (20,30)]

#for shooting // not sure
bulletVel = 0.5

framepersecond = pygame.time.Clock()

display = pygame.display.set_mode((width, height))
pygame.display.set_caption("Zombies_exercise_1")

#player, obstacles, enemies etc.
Player = classes.PlayerChar(display, triangle)
obstacle = classes.Obstacle(20, 50, 50, display)
Enemy = classes.Enemy(display, Vector2(400, 200), 15, 1, 2, 5)

#sprites = pygame.sprite.Group()
#sprites.add(Player)
#sprites.add(obstacle)

#main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    display.fill((0,0,0))

    # for i in sprites:
    #     display.blit(i.surf, i.rect)
    Player.draw()
    obstacle.draw()
    Player.move(acceleration, friction, [obstacle])
    Enemy.update(Player.pos)
    Enemy.draw()
    pygame.display.update()
    framepersecond.tick(fps)