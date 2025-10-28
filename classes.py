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


def check_collision(player, obstacles):
    return any(player.collider.colliderect(obstacle.collider) for obstacle in obstacles)

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
    def __init__(self, agent=None):
        self.agent = agent
        self.wanderTarget = Vector2(1, 0)  # Default direction
        self.agentVelocity = Vector2(0, 0) # Default velocity

    @staticmethod
    def seek(enemy_pos: Vector2, targetPos: Vector2, maxSpeed, enemy_velocity: Vector2):
        DesiredVelocity = Vector2((targetPos - enemy_pos) * maxSpeed).normalize()

        return (DesiredVelocity - enemy_velocity)
    def flee(enemy_pos: Vector2, targetPos: Vector2, maxSpeed, enemy_velocity: Vector2):
        panicDistance = 100 ** 2

        if enemy_pos.distance_squared_to(targetPos) > panicDistance:
            return Vector2(0,0)
        
        DesiredVelocity = Vector2((enemy_pos - targetPos) * maxSpeed).normalize()

        return (DesiredVelocity - enemy_velocity)
    
    def arrive(self, target_pos, declaration = 1.0):
        toTarget = target_pos - self.agent.pos
        distance = toTarget.length()
        if distance > 0:
            declaration_tweaker = 0.3
            speed = min(distance / (declaration * declaration_tweaker), self.agent.maxSpeed)
            desiredVelocity = toTarget * (speed / distance)
            return desiredVelocity - self.agent.velocity
        return pygame.Vector2(0,0)
            
        
    #uruchamia sie, do sprawdzenia bo dziala slabo
    #radius < distance /// wtedy jest bardziej smooth
    @staticmethod
    def wander(enemy_pos: Vector2, enemy_velocity: Vector2, wanderRadius = 1, wanderDistance = 5, wanderJiter = 40, dt = 1/60):
        jitter = wanderJiter 
        wanderTarget = Vector2(0,0)
        wanderTarget += Vector2(random.uniform(-1,1) * jitter, random.uniform(-1,1) * jitter)
        if wanderTarget.length() != 0:
            wanderTarget = wanderTarget.normalize()
        wanderTarget *= wanderRadius
        #circleCenter = enemy_velocity.normalize() * wanderDistance
        #targetWorld = enemy_pos + circleCenter + wanderTarget
        targetLocal = wanderTarget + Vector2(wanderDistance,0)
        target = targetLocal
        return target
    
    #idk czy potrzebne, jak boty sie zbiora to moga scigac gracza za pomoca seek
    def pursue(self, target_agent):
        to_target = target_agent.pos - self.agent.pos
        relative_heading = self.agent.velocity.normalize().dot(target_agent.velocity.normalize())
        if to_target.dot(self.agent.velocity.normalize()) > 0 and relative_heading < -0.95:
            return self.seek(target_agent.pos)
        look_ahead_time = to_target.length() / (self.agent.maxSpeed + target_agent.velocity.length())
        predicted_position = target_agent.pos + target_agent.velocity * look_ahead_time
        return self.seek(predicted_position)
    
    def evade(self, target_agent):
        to_target = target_agent.pos - self.agent.pos
        look_ahead_time = to_target.length() / (self.agent.maxSpeed + target_agent.velocity.length())
        predicted_position = target_agent.pos + target_agent.velocity * look_ahead_time
        return self.flee(predicted_position)
    
    def obstacle_avoidance(self, obstacles, detectionRadius = 100):
        closest_obstacle = None
        closest_distance = float('inf')
        for obstacle in obstacles:
            distance = (obstacle.pos - self.agent.pos).length() - getattr(obstacle, 'radius', 0)
            if 0 < distance < detectionRadius and distance < closest_distance:
                closest_obstacle = obstacle
                closest_distance = distance
        if closest_obstacle:
            avoidance_force = (self.agent.pos - closest_obstacle.pos).normalize()
            avoidence_strength = max(0, detectionRadius - closest_distance) / detectionRadius
            steering = avoidance_force * avoidence_strength * self.agent.max_force
            return steering
        return pygame.Vector2(0,0)
                
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
        #steering = SteeringBehaviour.seek(self.pos, target_pos, self.maxSpeed, self.vel)
        #steering = SteeringBehaviour.flee(self.pos, target_pos, self.maxSpeed, self.vel)
        steering = SteeringBehaviour.wander(self.pos, self.vel)
        self.vel += steering /self.mass

        if self.vel.length() > self.maxSpeed:
            self.vel.scale_to_length(self.maxSpeed)
            
        self.pos += self.vel
    def draw(self):
        pygame.draw.circle(self.surf, (0, 255, 0), (self.pos.x, self.pos.y), self.radius)