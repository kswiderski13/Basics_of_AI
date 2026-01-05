import pygame
from pygame.math import Vector2
import math
from collections import deque
import random

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
                entity.collider.topleft = (
                    entity.pos.x - entity.radius,
                    entity.pos.y - entity.radius
                )
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

class Obstacle(object):
    def __init__(self, radius, posX, posY, surface):
        self.radius = radius
        self.x = posX
        self.y = posY
        self.surface = surface
        self.collider = pygame.Rect(
            self.x - radius,
            self.y - radius,
            radius * 2,
            radius * 2
        )

    def draw(self):
        pygame.draw.circle(
            self.surface,
            (55, 100, 180),
            (int(self.x), int(self.y)),
            self.radius
        )
        self.collider.topleft = (
            self.x - self.radius,
            self.y - self.radius
        )

def check_collision(player, obstacle_or_iterable):
    if hasattr(obstacle_or_iterable, "collider"):
        return player.collider.colliderect(obstacle_or_iterable.collider)
    try:
        return any(
            player.collider.colliderect(ob.collider)
            for ob in obstacle_or_iterable
        )
    except Exception:
        return False

class Bullet:
    def __init__(self, pos: Vector2, direction: Vector2, speed=600, damage=20, owner=None):
        self.pos = Vector2(pos)
        self.vel = direction.normalize() * speed
        self.radius = 3
        self.color = (255, 255, 0)
        self.damage = damage
        self.owner = owner

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(
            surf,
            self.color,
            (int(self.pos.x), int(self.pos.y)),
            self.radius
        )

class Rocket:
    def __init__(self, pos: Vector2, direction: Vector2, owner=None):
        self.pos = Vector2(pos)
        self.vel = direction.normalize() * 200
        self.radius = 5
        self.color = (255, 120, 0)
        self.owner = owner
        self.explosion_radius = 60
        self.damage = 40
        self.life_time = 3.0
        self.age = 0.0

    def update(self, dt: float, bots):
        self.pos += self.vel * dt
        self.age += dt
        if self.age >= self.life_time:
            self.explode(bots)
            return True
        for b in bots:
            if b is self.owner:
                continue
            if (b.pos - self.pos).length_squared() <= (b.radius + self.radius) ** 2:
                self.explode(bots)
                return True
        return False

    def explode(self, bots):
        for b in bots:
            d_sq = (b.pos - self.pos).length_squared()
            if d_sq <= self.explosion_radius ** 2:
                b.hp -= self.damage

    def draw(self, surf: pygame.Surface):
        pygame.draw.circle(
            surf,
            self.color,
            (int(self.pos.x), int(self.pos.y)),
            self.radius
        )

class NavigationNode:
    def __init__(self, position: Vector2):
        self.position = Vector2(position)
        self.neighbors = []

    bot_radius = 15
    step = bot_radius

def can_place_bot(pos: Vector2, obstacles, map_rect, radius: float) -> bool:
    if pos.x - radius < map_rect.left:
        return False
    if pos.x + radius > map_rect.right:
        return False
    if pos.y - radius < map_rect.top:
        return False
    if pos.y + radius > map_rect.bottom:
        return False
    for ob in obstacles:
        d_sq = (Vector2(ob.x, ob.y) - pos).length_squared()
        if d_sq < (radius + ob.radius) ** 2:
            return False
    return True

def build_nav_graph_flood_fill(start_pos: Vector2, obstacles, map_rect: pygame.Rect, bot_radius: float):
    step = bot_radius * 0.75
    nodes: dict[tuple[int, int], NavigationNode] = {}
    queue = deque()

    start = Vector2(start_pos)
    if not can_place_bot(start, obstacles, map_rect, bot_radius):
        return nodes

    key = (int(start.x), int(start.y))
    nodes[key] = NavigationNode(start)
    queue.append(start)

    directions = [
        Vector2(1, 0), Vector2(-1, 0),
        Vector2(0, 1), Vector2(0, -1),
        Vector2(1, 1), Vector2(-1, 1),
        Vector2(1, -1), Vector2(-1, -1)
    ]

    while queue:
        current = queue.popleft()
        current_key = (int(current.x), int(current.y))
        current_node = nodes[current_key]

        for d in directions:
            new_pos = current + d * step
            new_key = (int(new_pos.x), int(new_pos.y))

            if new_key in nodes:
                continue

            if not can_place_bot(new_pos, obstacles, map_rect, bot_radius):
                continue

            new_node = NavigationNode(new_pos)
            nodes[new_key] = new_node

            current_node.neighbors.append(new_node)
            new_node.neighbors.append(current_node)

            queue.append(new_pos)

    return nodes

def draw_nav_graph(surface, nav_nodes):
    for node in nav_nodes.values():
        pygame.draw.circle(surface, (0, 180, 0), node.position, 2)
        for n in node.neighbors:
            pygame.draw.line(surface, (0, 90, 0), node.position, n.position, 1)

class PathPlanner:
    def __init__(self, owner, nav_graph: dict):
        self.nav_graph = nav_graph
        self.owner = owner
        self.destination_node = None

    def get_closest_node(self, pos: Vector2):
        closest_node = None
        closest_dist_sq = float('inf')
        for node in self.nav_graph.values():
            dist_sq = (node.position - pos).length_squared()
            if dist_sq < closest_dist_sq:
                closest_dist_sq = dist_sq
                closest_node = node
        return closest_node

    def plan_path(self, target_pos: Vector2, path: list):
        path.clear()

        start_node = self.get_closest_node(self.owner.pos)
        end_node = self.get_closest_node(target_pos)

        #Jeśli nie ma węzła startowego lub końcowego, false
        if start_node is None or end_node is None:
            return False

        # Jeśli start i koniec są takie same, false
        if start_node == end_node:
            return False

        # A*
        open_set = []
        closed_set = set()
        came_from = {}

        g_score = {}
        f_score = {}

        for node in self.nav_graph.values():
            g_score[node] = float('inf')
            f_score[node] = float('inf')

        g_score[start_node] = 0
        f_score[start_node] = (start_node.position - end_node.position).length()

        open_set.append(start_node)

        while open_set:
            # wybór węzła o najmniejszym f_score
            current = min(open_set, key=lambda node: f_score[node])

            # jeśli dotarliśmy do celu, rekonstrukcja ścieżki
            if current == end_node:
                self.reconstruct_path(came_from, current, path)
                return True

            open_set.remove(current)
            closed_set.add(current)

            for neighbor in current.neighbors:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + (current.position - neighbor.position).length()

                if neighbor not in open_set:
                    open_set.append(neighbor)
                elif tentative_g_score >= g_score[neighbor]:
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + (neighbor.position - end_node.position).length()

        return False

    def reconstruct_path(self, came_from: dict, current: 'NavigationNode', path: list):
        total_path = [current.position]

        while current in came_from:
            current = came_from[current]
            total_path.append(current.position)

        total_path.reverse()
        path.extend(total_path)

class Pickup:
    def __init__(self, pos: Vector2, kind: str, amount: int, surface):
        self.pos = Vector2(pos)
        self.kind = kind
        self.amount = amount
        self.radius = 10
        self.surface = surface

    def draw(self, surf):
        if self.kind == "health":
            color = (0, 255, 0)
        elif self.kind == "rail":
            color = (0, 200, 255)
        else:
            color = (255, 100, 0)
        pygame.draw.circle(surf, color, (int(self.pos.x), int(self.pos.y)), self.radius)

def spawn_pickups(surface, map_rect, obstacles):
    pickups = []
    kinds = ["health", "rail", "rocket"]
    for _ in range(8):
        for _ in range(20):
            x = random.randint(40, map_rect.width - 40)
            y = random.randint(40, map_rect.height - 40)
            pos = Vector2(x, y)
            ok = True
            for ob in obstacles:
                if (Vector2(ob.x, ob.y) - pos).length_squared() < (ob.radius + 20) ** 2:
                    ok = False
                    break
            if ok:
                kind = random.choice(kinds)
                amount = 25 if kind == "health" else 5
                pickups.append(Pickup(pos, kind, amount, surface))
                break
    return pickups

class dummybot:
    def __init__(self, pos, surface):
        self.pos = Vector2(pos)
        self.vel = Vector2(0, 0)
        self.speed = 120
        self.radius = 15
        self.collider = pygame.Rect(self.pos.x - self.radius, self.pos.y - self.radius, self.radius * 2, self.radius * 2)
        self.path = []
        self.current_wp = 0
        self.surf = surface

        self.hp = 100
        self.max_hp = 100
        self.ammo_rail = 10
        self.ammo_rocket = 5
        self.reload_time_rail = 1.2
        self.reload_time_rocket = 2.0
        self.rail_cooldown = 0.0
        self.rocket_cooldown = 0.0

        self.state = "search"
        self.target_enemy = None
        self.planner = None

    def set_path(self, path):
        filtered = [p for p in path if (p - self.pos).length_squared() > 1.0]
        self.path = filtered
        self.current_wp = 0

    def follow_path(self, dt):
        if not self.path or self.current_wp >= len(self.path):
            self.vel = Vector2(0, 0)
            return
        target = self.path[self.current_wp]
        to_target = target - self.pos
        dist = to_target.length()
        if dist < 1.0 or (self.vel.length_squared() > 0 and dist < self.vel.length() * dt * 1.5):
            self.pos = target
            self.current_wp += 1
            self.vel = Vector2(0, 0)
            return
        self.vel = to_target.normalize() * self.speed
        self.pos += self.vel * dt
        self.collider.center = self.pos

    def choose_state(self, bots, pickups):
        if self.hp <= 0:
            self.state = "dead"
            return
        low_hp = self.hp < 40
        low_ammo = (self.ammo_rail + self.ammo_rocket) < 3
        visible_enemy = self.find_closest_enemy(bots)
        if low_hp or low_ammo:
            best_pickup = self.find_best_pickup(pickups)
            if best_pickup is not None:
                self.state = "gather"
                self.target_enemy = best_pickup
                return
        if visible_enemy is not None:
            self.state = "fight"
            self.target_enemy = visible_enemy
            return
        self.state = "search"
        self.target_enemy = None

    def find_closest_enemy(self, bots):
        best = None
        best_d = float('inf')
        for b in bots:
            if b is self or b.hp <= 0:
                continue
            d = (b.pos - self.pos).length_squared()
            if d < best_d:
                best_d = d
                best = b
        return best

    def find_best_pickup(self, pickups):
        best = None
        best_d = float('inf')
        for p in pickups:
            d = (p.pos - self.pos).length_squared()
            if d < best_d:
                best_d = d
                best = p
        return best

    def plan_to_target(self, target_pos):
        if self.planner is None:
            return
        if (target_pos - self.pos).length_squared() < 100:
            return
        path = []
        if self.planner.plan_path(target_pos, path):
            self.set_path(path)

    def update_combat(self, dt, bots, bullets, rockets):
        self.rail_cooldown = max(0.0, self.rail_cooldown - dt)
        self.rocket_cooldown = max(0.0, self.rocket_cooldown - dt)

        if self.target_enemy is None or self.target_enemy.hp <= 0:
            return

        to_enemy = self.target_enemy.pos - self.pos
        dist = to_enemy.length()
        if dist < 1:
            return

        if self.ammo_rail > 0 and self.rail_cooldown <= 0.0:
            spread = random.uniform(-0.08, 0.08)
            dir = to_enemy.normalize()
            angle = math.atan2(dir.y, dir.x) + spread
            dir = Vector2(math.cos(angle), math.sin(angle))
            bullets.append(Bullet(self.pos, dir, speed=900, damage=25, owner=self))
            self.ammo_rail -= 1
            self.rail_cooldown = self.reload_time_rail
        elif self.ammo_rocket > 0 and self.rocket_cooldown <= 0.0:
            spread = random.uniform(-0.15, 0.15)
            dir = to_enemy.normalize()
            angle = math.atan2(dir.y, dir.x) + spread
            dir = Vector2(math.cos(angle), math.sin(angle))
            rockets.append(Rocket(self.pos, dir, owner=self))
            self.ammo_rocket -= 1
            self.rocket_cooldown = self.reload_time_rocket

    def handle_pickups(self, pickups):
        for p in pickups[:]:
            if (p.pos - self.pos).length_squared() <= (self.radius + p.radius) ** 2:
                if p.kind == "health":
                    self.hp = min(self.max_hp, self.hp + p.amount)
                elif p.kind == "rail":
                    self.ammo_rail += p.amount
                else:
                    self.ammo_rocket += p.amount
                pickups.remove(p)

    def update(self, dt, bots, obstacles, pickups, bullets, rockets, map_rect):
        if self.hp <= 0:
            return

        self.choose_state(bots, pickups)

        if self.state == "search":
            if not self.path or self.current_wp >= len(self.path):
                for _ in range(10):
                    rand_pos = Vector2(
                        random.randint(40, map_rect.width - 40),
                        random.randint(40, map_rect.height - 40)
                    )
                    if (rand_pos - self.pos).length_squared() > 1000:
                        self.plan_to_target(rand_pos)
                        break
            self.follow_path(dt)

        elif self.state == "gather":
            if isinstance(self.target_enemy, Pickup):
                self.plan_to_target(self.target_enemy.pos)
            self.follow_path(dt)

        elif self.state == "fight":
            if self.target_enemy is not None:
                self.plan_to_target(self.target_enemy.pos)
            self.follow_path(dt)
            self.update_combat(dt, bots, bullets, rockets)

        self.handle_pickups(pickups)
        self.collider.center = self.pos

    def draw(self, surface):
        if self.hp > 0:
            pygame.draw.circle(surface, (220, 60, 60), (int(self.pos.x), int(self.pos.y)), self.radius)
            if len(self.path) > 1:
                pygame.draw.lines(surface, (255, 200, 0), False, self.path, 2)
            hp_ratio = self.hp / self.max_hp
            bar_w = 30
            bar_h = 4
            x = self.pos.x - bar_w / 2
            y = self.pos.y - self.radius - 10
            pygame.draw.rect(surface, (60, 0, 0), (x, y, bar_w, bar_h))
            pygame.draw.rect(surface, (0, 200, 0), (x, y, bar_w * hp_ratio, bar_h))