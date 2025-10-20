import math
import random
import pygame
from pygame.math import Vector2

# funkcja sprawdzająca kolizję z okręgiem
def line_intersects_circle(p1, p2, center, radius):
    ap = center - p1
    ab = (p2 - p1).normalize()
    proj = ap.dot(ab)
    closest = p1 + ab * proj
    return center.distance_to(closest) <= radius and proj > 0

# funkcja generująca przeszkody
def generate_obstacles(count, width, height, player_size, margin=2):
    obs = []                      # lista przeszkód
    max_attempts = count * 100    # maksymalna liczba prób umieszczenia przeszkody
    attempts = 0                  # licznik prób

# generowanie przeszkód
    while len(obs) < count and attempts < max_attempts:
        attempts += 1
        r = random.randint(20, 40)

       # odstęp od krawędzi ekranu
        edge_gap = 2 * player_size + margin
        min_x    = r + edge_gap
        max_x    = width  - r - edge_gap
        min_y    = r + edge_gap
        max_y    = height - r - edge_gap

        # sprawdzamy czy jest miejsce na przeszkodę
        if min_x > max_x or min_y > max_y:    
            continue

        x = random.randint(min_x, max_x)
        y = random.randint(min_y, max_y)
        pos = Vector2(x, y)

        #odstęp od okręgów
        ok = True
        for other in obs:
            # minimalna odległość między krawędziami
            min_dist = r + other.radius + 2 * player_size + margin
            if pos.distance_to(other.pos) < min_dist:
                ok = False
                break

# dodaj przeszkodę jeśli jest miejsce
        if ok:
            obs.append(Obstacle(pos, r))

    return obs

#klasa pocisku
class Bullet:
    def __init__(self, pos: Vector2, direction: Vector2):
        self.pos    = Vector2(pos)
        self.vel    = direction.normalize() * 600
        self.radius = 4
        self.color  = (255, 255, 0)

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(surf, self.color, self.pos, self.radius)

# klasa gracza
class Player:
    def __init__(self, pos: Vector2):
        self.pos      = pos
        self.vel      = Vector2(0, 0)
        self.acc      = Vector2(0, 0)
        self.speed    = 300
        self.friction = -4 #tarcie
        self.size     = 15

# sterowanie graczem
    def handle_input(self):
        keys = pygame.key.get_pressed()
        self.acc = Vector2(0, 0)
        if keys[pygame.K_w]:
            self.acc.y = -1
        if keys[pygame.K_s]:
            self.acc.y = 1
        if keys[pygame.K_a]:
            self.acc.x = -1
        if keys[pygame.K_d]:
            self.acc.x = 1
        if self.acc.length_squared() > 0:                #normalizacja wektora
            self.acc = self.acc.normalize() * self.speed #przyspieszenie
        self.acc += self.vel * self.friction * 0.1       #tarcie

    def update(self, dt: float, obstacles: list):
        old_pos = self.pos.copy()
        self.handle_input()
        self.vel += self.acc * dt
        self.pos += self.vel * dt

        # kolizje z przeszkodami
        for ob in obstacles:
            if self.pos.distance_to(ob.pos) < self.size + ob.radius:
                self.pos = old_pos
                self.vel = Vector2(0, 0)
                break

        #granice ekranu
        w, h = pygame.display.get_surface().get_size()
        self.pos.x = max(self.size, min(self.pos.x, w - self.size))
        self.pos.y = max(self.size, min(self.pos.y, h - self.size))
    # rysowanie gracza
    def draw(self, surf: pygame.Surface):
        mx, my    = pygame.mouse.get_pos()
        direction = Vector2(mx, my) - self.pos
        angle     = math.degrees(math.atan2(direction.y, direction.x))
        pts = [
            Vector2(self.size, 0),
            Vector2(-self.size * 0.6,  self.size * 0.7),
            Vector2(-self.size * 0.6, -self.size * 0.7) 
        ]
        rot_pts = [(p.rotate(angle) + self.pos).xy for p in pts]
        pygame.draw.polygon(surf, (200, 200, 50), rot_pts)
# strzelanie
    def shoot(self):
        mx, my    = pygame.mouse.get_pos()
        direction = Vector2(mx, my) - self.pos
        if direction.length_squared() == 0:
            return None
        start = self.pos + direction.normalize() * (self.size + 5)
        return Bullet(start, direction)

#klasa przeszkody
class Obstacle:
    def __init__(self, pos: Vector2, radius: int):
        self.pos    = pos
        self.radius = radius

    def update(self, dt: float):
        pass

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(surf, (100, 100, 100), self.pos, self.radius)

# klasa bota
class Enemy:
    def __init__(self, pos: Vector2, min_speed: float = 25, max_speed: float = 50):
        self.pos          = pos
        self.vel          = Vector2(random.uniform(-1, 1),
                                    random.uniform(-1, 1))
        if self.vel.length() == 0:
            self.vel = Vector2(1, 0)
        self.min_speed    = min_speed    # minimalna prędkość
        self.acc          = Vector2(0, 0)
        self.max_speed    = max_speed    # maksymalna prędkość
        self.max_force    = 200          # maksymalna siła
        self.size         = 12           # rozmiar bota
        self.state        = "explore"    # stan bota
        self.wander_theta = 0            # kąt wędrówki
        self.last_attack  = -math.inf    # czas ostatniego ataku


    def draw(self, surf: pygame.Surface):
        color = (50, 200, 50) if self.state == "explore" else (200, 50, 50)
        pygame.draw.circle(surf, color, self.pos, self.size)

    def apply_force(self, force: Vector2):
        self.acc += force
# szukanie celu
    def seek(self, target: Vector2) -> Vector2:
        desired = (target - self.pos).normalize() * self.max_speed
        steer   = desired - self.vel
        if steer.length() > self.max_force:
            steer.scale_to_length(self.max_force)
        return steer
# wędrówka
    def wander(self) -> Vector2:
        wander_radius   = 50
        wander_distance = 60
        change          = 0.3
        self.wander_theta += random.uniform(-change, change)
  #punkty na okręgu wędrówki
        if self.vel.length() > 0: 
            circle_center = self.vel.normalize() * wander_distance #centrum okręgu
        else:
            angle = random.uniform(0, math.tau) 
            circle_center = Vector2(math.cos(angle), math.sin(angle)) * wander_distance

        offset = Vector2(math.cos(self.wander_theta),     #punkt na okręgu
                         math.sin(self.wander_theta)) * wander_radius   #promień okręgu
        return circle_center + offset
# unikanie przeszkód
    def avoid_obstacles(self, obstacles: list) -> Vector2:
        steer = Vector2(0, 0)
        for ob in obstacles:
            to_ob = ob.pos - self.pos
            dist  = to_ob.length() - ob.radius - self.size
            if 0 < dist < 50:
                steer += (self.pos - ob.pos).normalize() * (50 - dist)
        return steer
# unikanie ścian
    def avoid_walls(self, width: int, height: int) -> Vector2:
        margin = 30
        steer  = Vector2(0, 0)
        if self.pos.x < margin:
            steer += Vector2(self.max_force,  0)
        if self.pos.x > width - margin:
            steer += Vector2(-self.max_force, 0)
        if self.pos.y < margin:
            steer += Vector2(0,  self.max_force)
        if self.pos.y > height - margin:
            steer += Vector2(0, -self.max_force)
        return steer
# aktualizacja bota
    def update(self, dt: float, player: Player, obstacles: list):
        old_pos = self.pos.copy()
        self.acc = Vector2(0, 0)
        w, h = pygame.display.get_surface().get_size()
# zachowanie bota w zależności od stanu
        if self.state == "explore":
            blocked = any(
                line_intersects_circle(self.pos, player.pos, ob.pos, ob.radius)
                for ob in obstacles
            )
            if not blocked:
                nearest   = min(obstacles, key=lambda o: self.pos.distance_to(o.pos))
                hide_dir  = (nearest.pos - player.pos).normalize()
                target    = nearest.pos + hide_dir * (nearest.radius + self.size + 5)
                force     = self.seek(target)
            else:
                force = self.wander()

            force += self.avoid_obstacles(obstacles)
            force += self.avoid_walls(w, h)
        else:
            force = self.seek(player.pos)
            force += self.avoid_obstacles(obstacles)
            force += self.avoid_walls(w, h)

        self.apply_force(force)
        self.vel += self.acc * dt
        if self.vel.length() > 0:
            self.vel.scale_to_length(min(self.vel.length(), self.max_speed))
        self.pos += self.vel * dt

        # kolizja z przeszkodami
        for ob in obstacles:
            if self.pos.distance_to(ob.pos) < self.size + ob.radius:
                self.pos = old_pos
                self.vel = Vector2(0, 0)
                break