import pygame
from pygame.math import Vector2
import random
import math

def resolve_wall_penetration(entity, walls):
    is_centered = hasattr(entity, 'radius')
    for w in walls:
        if not hasattr(entity, 'collider'):
            continue
        if entity.collider.colliderect(w):
            overlap = entity.collider.clip(w)
            if overlap.width == 0 and overlap.height == 0:
                continue
            if overlap.width < overlap.height:
                if entity.collider.centerx < w.centerx:
                    dx = -overlap.width
                else:
                    dx = overlap.width
                dy = 0
            else:
                if entity.collider.centery < w.centery:
                    dy = -overlap.height
                else:
                    dy = overlap.height
                dx = 0
            if is_centered:
                entity.pos.x += dx
                entity.pos.y += dy
                entity.collider.topleft = (entity.pos.x - entity.radius, entity.pos.y - entity.radius)
            else:
                entity.pos.x += dx
                entity.pos.y += dy
                entity.collider.topleft = (entity.pos.x, entity.pos.y)
            v = Vector2(entity.vel)
            if abs(dx) > 0:
                v.x = 0
            if abs(dy) > 0:
                v.y = 0
            entity.vel = v * 0.1
            if hasattr(entity, '_stuck_timer'):
                entity._stuck_timer = 0.0

class PlayerChar(object):
    def __init__(self, surface, coords):
        super().__init__()
        self.surf = surface
        #self.surf.fill((138,225,200))
        #self.rect = self.surf.get_rect()
        self.pos = Vector2(10, 385)
        self.acc = Vector2(0,0)
        self.vel = Vector2(0,0)
        self.base_coords = coords
        self.coords = coords
        self.collider = pygame.Rect(self.pos.x, self.pos.y, 40, 40)

    def draw(self):
        newCoords = []
        for p in self.coords:
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
                resolve_wall_penetration(self, [ob.collider])

        # obracanie w kierunku myszy
        mouse_x, mouse_y = pygame.mouse.get_pos()
        center_offset = Vector2(20, 20)
        direction = Vector2(mouse_x, mouse_y) - (self.pos + center_offset)
        angle = math.atan2(direction.y, direction.x) - math.pi/2
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        rotated_coords = []
        for p in self.base_coords:
            rel = Vector2(p) - center_offset
            rx = rel.x * cos_a - rel.y * sin_a
            ry = rel.x * sin_a + rel.y * cos_a
            rotated_coords.append((rx + center_offset.x, ry + center_offset.y))
        self.coords = rotated_coords

    def shoot(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        direction = Vector2(mouse_x, mouse_y) - (self.pos + Vector2(20, 20))
        if direction.length_squared() == 0:
            return None
        
        heading = direction.normalize()
        best_proj = -float('inf')
        nose_local = None
        for p in self.coords:
            local = Vector2(p)  # coords są lokalne względem centrum (20,20)
            proj = local.dot(heading)
            if proj > best_proj:
                best_proj = proj
                nose_local = local
        if nose_local is None:
            nose_local = Vector2(20, 20)
        start = self.pos + nose_local
        return Bullet(start, direction)

    # def shoot_ray(self, enemies):
    #     mouse_x, mouse_y = pygame.mouse.get_pos()
    #     start = self.pos + Vector2(20, 20)
    #     end = Vector2(mouse_x, mouse_y)
        
    #     ray_dir = (end - start)
    #     if ray_dir.length() == 0:
    #         return None
    #     ray_dir = ray_dir.normalize()

    #     max_dist = 2000
    #     closest_enemy = None
    #     closest_dist = float('inf') #nieskonczonosc

    #     for e in enemies:
    #         to_enemy = e.pos - start
    #         proj_length = to_enemy.dot(ray_dir)

    #         # jeśli wróg jest za graczem lub dalej niż zasięg promienia
    #         if proj_length < 0 or proj_length > max_dist:
    #             continue

    #         closest_point = start + ray_dir * proj_length
    #         dist_to_enemy = (closest_point - e.pos).length()

    #         if dist_to_enemy <= e.radius:
    #             if proj_length < closest_dist:
    #                 closest_dist = proj_length
    #                 closest_enemy = e

    #     if closest_enemy:
    #         closest_enemy.hp -= 1

    #     return end


class Obstacle(object):
    def __init__(self, radius, posX, posY, surface):
        self.radius = radius
        self.x = posX
        self.y = posY
        self.surface = surface
        #self.color = pygame.Vector3(255,100,155)
        self.collider = pygame.Rect(self.x - radius, self.y - radius, radius *2, radius *2)

    def draw(self):
        pygame.draw.circle(self.surface, (55, 100, 180), (int(self.x), int(self.y)), self.radius)
        self.collider.topleft = (self.x - self.radius, self.y - self.radius)

def check_collision(player, obstacle_or_iterable):
    #jeśli przekazano pojedynczy obstacle (ma atrybut collider), sprawdzany bezpośrednio
    if hasattr(obstacle_or_iterable, "collider"):
        return player.collider.colliderect(obstacle_or_iterable.collider)
    #jeżeli przekazano iterowalny zbiór przeszkód, sprawdzamy każdy element
    try:
        return any(player.collider.colliderect(ob.collider) for ob in obstacle_or_iterable)
    except Exception:
        return False

class Bullet:
    def __init__(self, pos: Vector2, direction: Vector2):
        self.pos    = Vector2(pos)
        self.vel    = direction.normalize() * 600
        self.radius = 4
        self.color  = (255, 255, 0)

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(surf, self.color, (int(self.pos.x), int(self.pos.y)), self.radius)


class enemy2:
    pass