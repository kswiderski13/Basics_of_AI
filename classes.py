import pygame
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
        if keys[pygame.K_LEFT]:
            self.acc.x = -acceleration
        if keys[pygame.K_RIGHT]:
            self.acc.x = acceleration
        self.acc.x += self.vel.x * friction
        self.vel += self.acc
        self.pos += self.vel + 0.5 * self.acc
        self.rect.midbottom = self.pos

class Platform(pygame.sprite.Sprite):
    def __init__(self, width, height):
        super().__init__()
        self.surf = pygame.Surface((width, 20))
        self.surf.fill((255,0,0))
        self.rect = self.surf.get_rect(center = (width/2, height - 10))