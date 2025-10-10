#imports
import pygame
from pygame.locals import *
import sys
import classes
#import math

pygame.init()
vector = pygame.math.Vector2

#setup
height = 520
width = 340
fps = 60
acceleration = 0.5
friction = -0.1

framepersecond = pygame.time.Clock()

display = pygame.display.set_mode((width, height))
pygame.display.set_caption("Zombies_exercise_1")

#player, obstacles, enemies etc.
Player = classes.PlayerChar()
obstacle = classes.Platform(width, height)


sprites = pygame.sprite.Group()
sprites.add(Player)
sprites.add(obstacle)

#main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    display.fill((0,0,0))

    for i in sprites:
        display.blit(i.surf, i.rect)

    pygame.display.update()
    Player.move(acceleration, friction)
    framepersecond.tick(fps)