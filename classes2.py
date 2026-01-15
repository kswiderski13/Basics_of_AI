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


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    px1, py1 = poly[0]
    for i in range(n + 1):
        px2, py2 = poly[i % n]
        if y > min(py1, py2):
            if y <= max(py1, py2):
                if x <= max(px1, px2):
                    if py1 != py2:
                        xinters = (y - py1) * (px2 - px1) / (py2 - py1) + px1
                    if px1 == px2 or x <= xinters:
                        inside = not inside
        px1, py1 = px2, py2
    return inside


class Bullet:
    def __init__(self, pos: Vector2, direction: Vector2, speed=600, damage=20, owner=None):
        self.pos = Vector2(pos)
        self.vel = direction.normalize() * speed
        self.radius = 3
        self.color = (255, 255, 0)
        self.damage = damage
        self.owner = owner
        self.alive = True

    def update(self, dt: float, obstacles):
        new_pos = self.pos + self.vel * dt
        for r in obstacles:
            if r.collidepoint(new_pos.x, new_pos.y):
                self.alive = False
                return
        self.pos = new_pos

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
        self.alive = True

    def update(self, dt: float, bots, obstacles):
        new_pos = self.pos + self.vel * dt
        for r in obstacles:
            if r.collidepoint(new_pos.x, new_pos.y):
                self.pos = new_pos
                self.explode(bots)
                self.alive = False
                return
        self.pos = new_pos
        self.age += dt
        if self.age >= self.life_time:
            self.explode(bots)
            self.alive = False
            return
        for b in bots:
            if b is self.owner:
                continue
            if (b.pos - self.pos).length_squared() <= (b.radius + self.radius) ** 2:
                self.explode(bots)
                self.alive = False
                return

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


def can_place_bot(pos: Vector2, obstacles, poly_obstacles, map_rect: pygame.Rect, radius: float) -> bool:
    if pos.x - radius < map_rect.left:
        return False
    if pos.x + radius > map_rect.right:
        return False
    if pos.y - radius < map_rect.top:
        return False
    if pos.y + radius > map_rect.bottom:
        return False
    bot_rect = pygame.Rect(pos.x - radius, pos.y - radius, radius * 2, radius * 2)
    for r in obstacles:
        if bot_rect.colliderect(r.inflate(10, 10)):
            return False
    for poly in poly_obstacles:
        if point_in_poly(pos.x, pos.y, poly):
            return False
    return True


def build_nav_graph_flood_fill(map_rect: pygame.Rect, bot_radius: float, obstacles, poly_obstacles):
    nodes: dict[tuple[int, int], NavigationNode] = {}
    step = int(bot_radius * 2)
    
    for y in range(map_rect.top + int(bot_radius), map_rect.bottom - int(bot_radius), step):
        for x in range(map_rect.left + int(bot_radius), map_rect.right - int(bot_radius), step):
            pos = Vector2(x, y)
            if can_place_bot(pos, obstacles, poly_obstacles, map_rect, bot_radius):
                nodes[(x, y)] = NavigationNode(pos)
    for (x, y), node in nodes.items():
        for dx, dy in [(step, 0), (-step, 0), (0, step), (0, -step)]:
            nx, ny = x + dx, y + dy
            if (nx, ny) in nodes:
                node.neighbors.append(nodes[(nx, ny)])
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
        if start_node is None or end_node is None:
            return False
        if start_node == end_node:
            return False
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
            current = min(open_set, key=lambda node: f_score[node])
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
        if total_path and (total_path[0] - self.owner.pos).length_squared() < 1.0:
            total_path.pop(0)
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
                if hasattr(ob, "x"):
                    center = Vector2(ob.x, ob.y)
                    r = ob.radius
                    if (center - pos).length_squared() < (r + 20) ** 2:
                        ok = False
                        break
            if ok:
                kind = random.choice(kinds)
                amount = 25 if kind == "health" else 5
                pickups.append(Pickup(pos, kind, amount, surface))
                break
    return pickups



class Message:
    def __init__(self, sender, receiver, msg, extra=None):
        self.sender = sender
        self.receiver = receiver
        self.msg = msg
        self.extra = extra


class MessageDispatcher:
    def __init__(self):
        self.queue = deque()

    def dispatch(self, sender, receiver, msg, extra=None):
        self.queue.append(Message(sender, receiver, msg, extra))

    def process(self):
        while self.queue:
            m = self.queue.popleft()
            if hasattr(m.receiver, "handle_message"):
                m.receiver.handle_message(m)


dispatcher = MessageDispatcher()


class Regulator:
    def __init__(self, updates_per_second):
        self.interval = 1.0 / updates_per_second
        self.acc = 0.0

    def ready(self, dt):
        self.acc += dt
        if self.acc >= self.interval:
            self.acc = 0.0
            return True
        return False


class Goal:
    def __init__(self, bot):
        self.bot = bot
        self.active = False

    def activate(self):
        self.active = True

    def process(self):
        return "inactive"

    def terminate(self):
        self.active = False


class GoalFollowPath(Goal):
    def activate(self):
        self.active = True

    def process(self):
        if not self.bot.path or self.bot.current_wp >= len(self.bot.path):
            return "completed"
        self.bot.follow_path(self.bot.dt, self.bot.bots_ref, self.bot.obstacles_ref)
        if not self.bot.path or self.bot.current_wp >= len(self.bot.path):
            return "completed"
        return "active"


class GoalAttackTarget(Goal):
    def process(self):
        if self.bot.target_enemy is None or self.bot.target_enemy.hp <= 0:
            return "completed"
        self.bot.update_combat(
            self.bot.dt,
            self.bot.bots_ref,
            self.bot.bullets_ref,
            self.bot.rockets_ref,
            self.bot.obstacles_ref
        )
        return "active"


class GoalThink(Goal):
    def __init__(self, bot):
        super().__init__(bot)
        self.subgoals = []

    def add_subgoal(self, goal):
        self.subgoals.insert(0, goal)

    def process(self):
        while self.subgoals and not self.subgoals[0].active:
            self.subgoals[0].activate()

        if not self.subgoals:
            return "inactive"

        status = self.subgoals[0].process()

        if status == "completed":
            self.subgoals[0].terminate()
            self.subgoals.pop(0)

        return "active"
class dummybot:
    def __init__(self, pos, surface):
        self.pos = Vector2(pos)
        self.vel = Vector2(0, 0)
        self.speed = 120
        self.radius = 15
        self.collider = pygame.Rect(self.pos.x - self.radius, self.pos.y - self.radius,
                                    self.radius * 2, self.radius * 2)
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
        self.combat_range = 260
        self.too_close_range = 120
        self.stop_shoot_speed = 10
        self.last_seen_enemy_pos = None
        self.time_since_enemy_seen = 0.0
        self.state_time = 0.0
        self.search_target_pos = None

        self.memory = {
            "last_seen_enemy": None,
            "last_seen_time": 0.0,
            "last_hit_time": 0.0,
            "last_shot_time": 0.0,
        }

        self.sense_reg = Regulator(4)
        self.think_reg = Regulator(2)

        self.feeler_length = 40

        self.brain = GoalThink(self)

        self.dt = 0.0
        self.bots_ref = None
        self.bullets_ref = None
        self.rockets_ref = None
        self.obstacles_ref = None

    def has_line_of_sight(self, target, obstacles):
        x1, y1 = self.pos
        x2, y2 = target.pos
        for r in obstacles:
            if r.clipline((x1, y1), (x2, y2)):
                return False
        return True

    def set_path(self, path):
        filtered = [p for p in path if (p - self.pos).length_squared() > 1.0]
        self.path = filtered
        self.current_wp = 0

    def is_position_blocked(self, pos, bots):
        for b in bots:
            if b is self or b.hp <= 0:
                continue
            if (b.pos - pos).length_squared() < (self.radius * 2) ** 2:
                return True
        return False

    def follow_path(self, dt, bots, obstacles):
        if not self.path or self.current_wp >= len(self.path):
            self.vel = Vector2(0, 0)
            return

        target = self.path[self.current_wp]
        to_target = target - self.pos
        dist = to_target.length()

        # dotarcie do waypointa
        if dist < 1.0 or (self.vel.length_squared() > 0 and dist < self.vel.length() * dt * 1.5):
            self.pos = target
            self.current_wp += 1
            self.vel = Vector2(0, 0)
            self.collider.center = self.pos
            return

        # ruch w stronę waypointa
        move_dir = to_target.normalize()
        next_pos = self.pos + move_dir * self.speed * dt

        # pełny collider bota po ruchu
        next_collider = pygame.Rect(
            next_pos.x - self.radius,
            next_pos.y - self.radius,
            self.radius * 2,
            self.radius * 2
        )

        # test kolizji z przeszkodami 
        for r in obstacles:
            if next_collider.colliderect(r.inflate(10, 10)):
                self.current_wp += 1
                self.vel = Vector2(0, 0)
                return

        # ruch jest bezpieczny to wykonujemy go
        self.vel = move_dir * self.speed
        self.pos = next_pos
        self.collider.center = self.pos

        if self.current_wp >= len(self.path):
            self.vel = Vector2(0, 0)
    def plan_to_target(self, target_pos):
        if self.planner is None:
            return
        path = []
        if self.planner.plan_path(target_pos, path):
            self.set_path(path)

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

    def sense(self, bots, obstacles):
        closest_enemy = self.find_closest_enemy(bots)
        if closest_enemy is not None and self.has_line_of_sight(closest_enemy, obstacles):
            self.last_seen_enemy_pos = Vector2(closest_enemy.pos)
            self.time_since_enemy_seen = 0.0
            self.memory["last_seen_enemy"] = Vector2(closest_enemy.pos)
            self.memory["last_seen_time"] = 0.0
            dispatcher.dispatch(self, self, "ENEMY_SPOTTED", Vector2(closest_enemy.pos))

    def handle_message(self, msg):
        if msg.msg == "ENEMY_SPOTTED":
            self.last_seen_enemy_pos = Vector2(msg.extra)
            self.time_since_enemy_seen = 0.0
            self.memory["last_seen_enemy"] = Vector2(msg.extra)
            self.memory["last_seen_time"] = 0.0

    def choose_state(self, bots, pickups, obstacles):
        if self.hp <= 0:
            self.state = "dead"
            self.target_enemy = None
            return

        closest_enemy = self.find_closest_enemy(bots)

        low_hp = self.hp < 40
        low_ammo = (self.ammo_rail + self.ammo_rocket) < 3

        if low_hp or low_ammo:
            best_pickup = self.find_best_pickup(pickups)
            if best_pickup is not None:
                self.state = "gather"
                self.target_enemy = best_pickup
                self.search_target_pos = None
                return

        if closest_enemy is not None and self.has_line_of_sight(closest_enemy, obstacles):
            self.state = "fight"
            self.target_enemy = closest_enemy
            self.search_target_pos = None
            self.path = []
            self.current_wp = 0
            return

        if self.last_seen_enemy_pos is not None and self.time_since_enemy_seen < 6.0:
            self.state = "search"
            self.target_enemy = None
            self.search_target_pos = Vector2(self.last_seen_enemy_pos)
            return

        self.state = "search"
        self.target_enemy = None
        if self.search_target_pos is None:
            self.search_target_pos = None

    def update_combat(self, dt, bots, bullets, rockets, obstacles):
        self.rail_cooldown = max(0.0, self.rail_cooldown - dt)
        self.rocket_cooldown = max(0.0, self.rocket_cooldown - dt)

        if self.target_enemy is None or self.target_enemy.hp <= 0:
            return

        if self.has_line_of_sight(self.target_enemy, obstacles):
            self.last_seen_enemy_pos = Vector2(self.target_enemy.pos)
            self.time_since_enemy_seen = 0.0
            self.memory["last_seen_enemy"] = Vector2(self.target_enemy.pos)
            self.memory["last_seen_time"] = 0.0
        else:
            if self.last_seen_enemy_pos is not None and self.time_since_enemy_seen < 6.0:
                self.plan_to_target(self.last_seen_enemy_pos)
                return
            return

        to_enemy = self.target_enemy.pos - self.pos
        dist = to_enemy.length()
        if dist < 1:
            return

        if dist < self.too_close_range:
            self.vel = -to_enemy.normalize() * self.speed * 0.7
            self.pos += self.vel * dt
            self.collider.center = self.pos
            return

        if dist > self.combat_range:
            self.plan_to_target(self.target_enemy.pos)
            self.follow_path(dt, bots, obstacles)
            return

        self.vel = Vector2(0, 0)
        self.collider.center = self.pos

        if self.vel.length() > self.stop_shoot_speed:
            return

        if self.ammo_rail > 0 and self.rail_cooldown <= 0.0:
            spread = random.uniform(-0.08, 0.08)
            dir = to_enemy.normalize()
            angle = math.atan2(dir.y, dir.x) + spread
            dir = Vector2(math.cos(angle), math.sin(angle))
            bullets.append(Bullet(self.pos, dir, speed=900, damage=25, owner=self))
            self.ammo_rail -= 1
            self.rail_cooldown = self.reload_time_rail
            self.memory["last_shot_time"] = 0.0
        elif self.ammo_rocket > 0 and self.rocket_cooldown <= 0.0:
            spread = random.uniform(-0.15, 0.15)
            dir = to_enemy.normalize()
            angle = math.atan2(dir.y, dir.x) + spread
            dir = Vector2(math.cos(angle), math.sin(angle))
            rockets.append(Rocket(self.pos, dir, owner=self))
            self.ammo_rocket -= 1
            self.rocket_cooldown = self.reload_time_rocket
            self.memory["last_shot_time"] = 0.0

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

    def apply_separation(self, bots):
        for b in bots:
            if b is self:
                continue
            d = b.pos - self.pos
            dist_sq = d.length_squared()
            min_dist = (self.radius * 2) ** 2
            if dist_sq < min_dist and dist_sq > 0:
                push = d.normalize() * -4.0
                self.pos += push
                self.collider.center = self.pos

    def feelers(self):
        if self.vel.length_squared() == 0:
            return []
        forward = self.vel.normalize() * self.feeler_length
        left = forward.rotate(45)
        right = forward.rotate(-45)
        return [
            self.pos + forward,
            self.pos + left,
            self.pos + right
        ]

    def update(self, dt, bots, obstacles, pickups, bullets, rockets, map_rect):
        self.dt = dt
        self.bots_ref = bots
        self.bullets_ref = bullets
        self.rockets_ref = rockets
        self.obstacles_ref = obstacles

        dispatcher.process()

        print(f"[{self.state}] pos={self.pos} vel={self.vel} path_len={len(self.path)} wp={self.current_wp}")

        if self.hp <= 0:
            self.state = "dead"
            return

        self.time_since_enemy_seen += dt
        self.state_time += dt
        self.memory["last_seen_time"] += dt
        self.memory["last_shot_time"] += dt

        if self.sense_reg.ready(dt):
            self.sense(bots, obstacles)

        prev_state = self.state
        self.choose_state(bots, pickups, obstacles)
        if self.state != prev_state:
            self.state_time = 0.0

        if self.think_reg.ready(dt):
            self.brain.process()

        if self.state == "search":
            if self.search_target_pos is not None and self.time_since_enemy_seen < 6.0:
                if not self.path or self.current_wp >= len(self.path):
                    self.plan_to_target(self.search_target_pos)
                self.follow_path(dt, bots, obstacles)
            else:
                if not self.path or self.current_wp >= len(self.path):
                    for _ in range(10):
                        rand_pos = Vector2(
                            random.randint(40, map_rect.width - 40),
                            random.randint(40, map_rect.height - 40)
                        )
                        if (rand_pos - self.pos).length_squared() > 1000:
                            self.plan_to_target(rand_pos)
                            break
                self.follow_path(dt, bots, obstacles)
            if self.state_time > 8.0:
                self.path = []
                self.current_wp = 0
                self.search_target_pos = None
                self.state_time = 0.0

        elif self.state == "gather":
            if isinstance(self.target_enemy, Pickup):
                if not self.path or self.current_wp >= len(self.path):
                    self.plan_to_target(self.target_enemy.pos)
            self.follow_path(dt, bots, obstacles)

        elif self.state == "fight":
            self.update_combat(dt, bots, bullets, rockets, obstacles)

        for f in self.feelers():
            for r in obstacles:
                if r.collidepoint(f.x, f.y):
                    self.pos -= self.vel * 0.1
                    self.collider.center = self.pos

        self.apply_separation(bots)

        for r in obstacles:
            resolve_wall_penetration(self, [r])

        self.handle_pickups(pickups)
        self.collider.center = self.pos

    def draw(self, surface):
        if self.hp <= 0:
            return
        pygame.draw.circle(surface, (220, 60, 60), (int(self.pos.x), int(self.pos.y)), self.radius)

        if len(self.path) > 1:
            pygame.draw.lines(surface, (255, 200, 0), False, self.path, 2)
            if self.current_wp < len(self.path):
                wp = self.path[self.current_wp]
                pygame.draw.circle(surface, (0, 0, 255), (int(wp.x), int(wp.y)), 4)

        if self.vel.length_squared() > 0:
            end = self.pos + self.vel.normalize() * 20
            pygame.draw.line(surface, (0, 255, 255), self.pos, end, 2)

        hp_ratio = self.hp / self.max_hp
        bar_w = 30
        bar_h = 4
        x = self.pos.x - bar_w / 2
        y = self.pos.y - self.radius - 10
        pygame.draw.rect(surface, (60, 0, 0), (x, y, bar_w, bar_h))
        pygame.draw.rect(surface, (0, 200, 0), (x, y, bar_w * hp_ratio, bar_h))