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
height = 800
width = 580
fps = 60
acceleration = 0.5
friction = -0.1
wall_color = (0, 100, 255)

#walls
def draw_walls(screen):                   
    thickness = 10                        #grubość ścian
    pygame.draw.rect(screen, wall_color, (0, 0, width, thickness)) #GÓRA
    pygame.draw.rect(screen, wall_color, (0, height - thickness, width, thickness)) #DÓŁ
    pygame.draw.rect(screen, wall_color, (0, 0, thickness, height)) #LEWO
    pygame.draw.rect(screen, wall_color, (width - thickness, 0, thickness, height)) #PRAWO



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
obstacle1 = classes.Obstacle(20, 100, 60, display)
obstacle2 = classes.Obstacle(20, 200, 300, display)
obstacle3 = classes.Obstacle(20, 150, 150, display)
Enemy1 = classes.Enemy(display, Vector2(200, 200), 15, 1, 2, 5)
Enemy2 = classes.Enemy(display, Vector2(300, 200), 15, 1, 2, 5)

#sprites = pygame.sprite.Group()
#sprites.add(Player)
#sprites.add(obstacle)
enemies = pygame.sprite.Group()
enemies.add()
#main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    display.fill((0,0,0))
    draw_walls(display)

    # for i in sprites:
    #     display.blit(i.surf, i.rect)
    Player.draw()
    obstacle1.draw()
    obstacle2.draw()
    obstacle3.draw()
    obstacles = [obstacle1, obstacle2, obstacle3]
    Player.move(acceleration, friction, [obstacles])
    #do przerobienia 
    Enemy1.update(Player.pos)
    Enemy1.draw()
    Enemy2.update(Player.pos)
    Enemy2.draw()
    #
    pygame.display.update()
    framepersecond.tick(fps)