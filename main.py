#imports
import pygame
import sys

pygame.init()

#setup
height = 320
width = 240
fps = 60

display = pygame.display.set_mode((height, width))
pygame.display.set_caption("Zombies_exercise_1")

#main loop
while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()

    display.fill((0,0,0))

    pygame.display.update()