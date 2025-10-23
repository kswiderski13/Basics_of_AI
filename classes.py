import pygame
from pygame.math import Vector2
import random


class PlayerChar(object):
    def __init__(self, surface, coords):
        super().__init__()
        self.surf = surface
        #self.surf.fill((138,225,200))
        #self.rect = self.surf.get_rect()
        self.pos = Vector2(10, 385)
        self.acc = Vector2(0,0)
        self.vel = Vector2(0,0)
        self.coords = coords
        self.collider = pygame.Rect(self.pos.x, self.pos.y, 40, 40)
    

    def draw(self):
        newCoords = []
        for p in self. coords:
            newX = p[0] + self.pos.x
            newY = p[1] + self.pos.y
            newCoords.append((newX, newY))
        pygame.draw.polygon(self.surf, (138,225,200), newCoords)
        self.collider.topleft = (self.pos.x, self.pos.y)

    def move(self, acceleration, friction, obstacles):
        self.acc = pygame.math.Vector2(0,0)
        keys = pygame.key.get_pressed()
        if keys[pygame.K_a]:
            self.acc.x = -acceleration
        if keys[pygame.K_d]:
            self.acc.x = acceleration
        if keys[pygame.K_w]:
            self.acc.y = -acceleration
        if keys[pygame.K_s]:
            self.acc.y = acceleration
        #if keys[pygame.K_SPACE]:
         #   Bullet.shoot()
        self.acc.x += self.vel.x * friction
        self.acc.y += self.vel.y * friction
        self.vel += self.acc
        old_pos = self.pos.copy()
        self.pos += self.vel + 0.5 * self.acc
        self.collider.topleft = (self.pos.x, self.pos.y)

        for ob in obstacles:
            if check_collision(self, ob):
                self.pos = old_pos
                self.vel = pygame.Vector2(0,0)
                self.collider.topleft = (self.pos.x, self.pos.y)

#obstacles //add collider
class Obstacle(object):
    def __init__(self, radius, posX, posY, surface):
        self.radius = radius
        self.x = posX
        self.y = posY
        self.surface = surface
        #self.color = pygame.Vector3(255,100,155)
        self.collider = pygame.Rect(self.x - radius, self.y - radius, radius * 2, radius * 2)

    def draw(self):
        pygame.draw.circle(self.surface, (255, 0, 0), (self.x, self.y), self.radius)


def check_collision(player, obstacle):
    return player.collider.colliderect(obstacle.collider)

#change to class xyz(object) and add draw() function
class Bullet(pygame.sprite.Sprite):
    def __init__(self, x, y, radius, velocity ):
        super().__init__()
        self.x = x
        self.y = y
        self.v = velocity
        bulletPos = pygame.math.Vector2(self.x, self.y)
        
        def instantiate():
            pass

class SteeringBehaviour():
    @staticmethod
    def seek(enemy_pos: Vector2, targetPos: Vector2, maxSpeed, enemy_velocity: Vector2):
        DesiredVelocity = Vector2((targetPos - enemy_pos) * maxSpeed).normalize()

        return (DesiredVelocity - enemy_velocity)
    def flee():
        pass

    def wander(wanderRadius, wanderDistance, wanderJiter):
        pass

#na podstawie ksiazki, pominalem klase BaseGameEntity, enemy ma korzystac z klasy steering behaviours
class Enemy(object):
    def __init__(self, surface, pos: Vector2, radius, mass, maxSpeed, turnRate):
        self.pos = pos
        self.surf = surface
        self.radius = radius
        self.mass = mass
        self.vel = Vector2(0,0)
        self.maxSpeed = maxSpeed
        self.turnRate = turnRate

    def update(self, target_pos: Vector2):
        steering = SteeringBehaviour.seek(self.pos, target_pos, self.maxSpeed, self.vel)
        self.vel += steering /self.mass

        if self.vel.length() > self.maxSpeed:
            self.vel.scale_to_length(self.maxSpeed)
            
        self.pos += self.vel
    def draw(self):
        pygame.draw.circle(self.surf, (0, 255, 0), (self.pos.x, self.pos.y), self.radius)