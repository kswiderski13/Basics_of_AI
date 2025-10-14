import pygame

#change player to triangle, better to use class xyz(object) and def draw()
class PlayerChar(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.surf = pygame.Surface((30, 30))
        self.surf.fill((138,225,200))
        self.rect = self.surf.get_rect()
        self.pos = pygame.math.Vector2((10, 385))
        self.acc = pygame.math.Vector2(0,0)
        self.vel = pygame.math.Vector2(0,0)

    def move(self, acceleration, friction):
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
        self.pos += self.vel + 0.5 * self.acc
        self.rect.midbottom = self.pos
"""
#change to class xyz(object) and add draw() function
class Obstacle(pygame.sprite.Sprite):
    def __init__(self, width, height):
        super().__init__()
        self.surf = pygame.Surface((20, 20))
        self.surf.fill((255,0,0))
        #self.rect = self.surf.get_rect(center = (width/2, height - 10))
        self.rect = self.surf.get_rect()
"""
class Obstacle(object):
    def __init__(self, width, height, posX, posY):
        self.width = width
        self.height = height
        self.x = posX
        self.y = posY
        #self.color = pygame.Vector3(255,100,155)

    def draw():
        pygame.draw.rect(display, pygame.Color(color_red), (self.x, self.y) )

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
#to do, enemy has to be a circle
class Enemy(pygame.sprite.Sprite):
    pass