import pygame
from pygame.math import Vector2
import random
import math

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

        #do poprawki
        mouse_x, mouse_y = pygame.mouse.get_pos()
        direction = Vector2(mouse_x - (self.pos.x + 20), mouse_y - (self.pos.y + 20))
        angle = math.degrees(math.atan2(-direction.y, direction.x))
        rotated_coords = []
        for p in self.base_coords:
            rotated_x = p[0] * math.cos(math.radians(angle)) - p[1] * math.sin(math.radians(angle))
            rotated_y = p[0] * math.sin(math.radians(angle)) + p[1] * math.cos(math.radians(angle))
            rotated_coords.append((rotated_x, rotated_y))
        self.coords = rotated_coords

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
        pygame.draw.circle(self.surface, (255, 0, 0), (int(self.x), int(self.y)), self.radius)

def check_collision(player, obstacle_or_iterable):
    # jeśli przekazano pojedynczy obstacle (ma atrybut collider), sprawdzamy bezpośrednio
    if hasattr(obstacle_or_iterable, "collider"):
        return player.collider.colliderect(obstacle_or_iterable.collider)
    # jeżeli przekazano iterowalny zbiór przeszkód, sprawdzamy każdy element
    try:
        return any(player.collider.colliderect(ob.collider) for ob in obstacle_or_iterable)
    except Exception:
        return False

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
        #self.agentVelocity = Vector2(0, 0) # Default velocity
        self.max_force = getattr(agent, 'max_force', 200.0)

    def _vector_truncate(self, v: Vector2, max_value: float):
        if v.length() > max_value:
            v = v.normalize() * max_value
        return v

   # @staticmethod
    def seek(self, target_pos: Vector2):
        desired = (target_pos - self.agent.pos)
        if desired.length() > 0:
            desired = desired.normalize() * self.agent.maxSpeed
        return desired - self.agent.vel



    def flee(self, target_pos: Vector2, panic_distance_sq = 100**2):
        if self.agent.pos.distance_squared_to(target_pos) > panic_distance_sq:
            return Vector2(0,0)
        desired = (self.agent.pos - target_pos)
        if desired.length() > 0:
            desired = desired.normalize() * self.agent.maxSpeed + Vector2(10,10)
        return desired - self.agent.vel

    
    def arrive(self, target_pos, declaration = 1.0):
        toTarget = target_pos - self.agent.pos
        distance = toTarget.length()
        if distance > 0:
            declaration_tweaker = 0.3
            speed = min(distance / (declaration * declaration_tweaker), self.agent.maxSpeed)
            desiredVelocity = toTarget * (speed / distance)
            return desiredVelocity - self.agent.velocity
        return pygame.Vector2(0,0)
            
    #radius < distance /// wtedy jest bardziej smooth
    def wander(self, wanderRadius = 50.0, wanderDistance = 100.0, wanderJitter = 40.0, dt = 1/60.0):

        jitter = Vector2(random.uniform(-1,1) * wanderJitter * dt, random.uniform(-1,1) * wanderJitter * dt)
        self.wanderTarget += jitter
        if self.wanderTarget.length() == 0:
            self.wanderTarget = Vector2(1, 0)
        self.wanderTarget = self.wanderTarget.normalize() * wanderRadius
        targetLocal = self.wanderTarget + Vector2(wanderDistance, 0)
        if self.agent.vel.length() > 0:
            heading = self.agent.vel.normalize()
        else:
            heading = Vector2(random.uniform(-1,1), random.uniform(-1,1)).normalize()
        side = Vector2(-heading.y, heading.x)
        targetWorld = self.agent.pos + heading * targetLocal.x + side * targetLocal.y
        desired_velocity = (targetWorld - self.agent.pos)
        if desired_velocity.length() > 0:
            desired_velocity = desired_velocity.normalize() * self.agent.maxSpeed
        return desired_velocity - self.agent.vel
    
    #idk czy potrzebne, jak boty sie zbiora to moga scigac gracza za pomoca seek
    def pursue(self, target_agent):
        target_vel = getattr(target_agent, 'vel', getattr(target_agent, 'velocity', Vector2(0,0)))
        to_target = target_agent.pos - self.agent.pos
        if self.agent.vel.length() > 0 and target_vel.length() > 0:
            relative_heading = self.agent.vel.normalize().dot(target_vel.normalize())
        else:
            relative_heading = 0.0
        if self.agent.vel.length() > 0 and to_target.dot(self.agent.vel.normalize()) > 0 and relative_heading < -0.95:
            return self.seek(target_agent.pos)
        denom = (self.agent.maxSpeed + target_vel.length())
        look_ahead_time = to_target.length() / denom if denom != 0 else 0.0
        future_pos = target_agent.pos + target_vel * look_ahead_time
        return self.seek(future_pos)


    def evade(self, target_agent):
        target_vel = getattr(target_agent, 'vel', getattr(target_agent, 'velocity', Vector2(0,0)))
        to_target = target_agent.pos - self.agent.pos
        denom = (self.agent.maxSpeed + target_vel.length())
        look_ahead_time = to_target.length() / denom if denom != 0 else 0.0
        future_pos = target_agent.pos + target_vel * look_ahead_time
        return self.flee(future_pos)

    
    def obstacle_avoidance(self, obstacles, detectionRadius = 100):
        feeler_lengths = [detectionRadius, detectionRadius * 0.6, detectionRadius * 0.3]
        heading = self.agent.vel.normalize() if self.agent.vel.length() > 0 else Vector2(1,0)
        side = Vector2(-heading.y, heading.x)
        agent_pos = self.agent.pos
        closest_ob = None
        closest_dist = float('inf')
        closest_point = None
        agent_radius = getattr(self.agent, 'radius', 0)
        buffer = getattr(self, 'OBSTACLE_BUFFER', 10) + agent_radius
        for feeler in feeler_lengths:
            feeler_end = agent_pos + heading * feeler
            for ob in obstacles:
                ob_pos = Vector2(getattr(ob, 'x', 0), getattr(ob, 'y', 0))
                r = getattr(ob, 'radius', 0)
                to_circle = ob_pos - agent_pos
                proj = to_circle.dot(heading)
                if proj <= 0 or proj > feeler:
                    continue
                closest = agent_pos + heading * proj
                dist_sq = (ob_pos - closest).length_squared()
                thresh = (r + buffer) ** 2
                if dist_sq < thresh and proj < closest_dist:
                    closest_dist = proj
                    closest_ob = ob
                    closest_point = closest

        if closest_ob is not None and closest_point is not None:
            ob_pos = Vector2(getattr(closest_ob, 'x', 0), getattr(closest_ob, 'y', 0))
            avoidance_dir = (self.agent.pos - ob_pos)
            if avoidance_dir.length() > 0:
                avoidance_dir = avoidance_dir.normalize()
            strength = max(0.0, (detectionRadius - closest_dist) / detectionRadius)
            max_force = getattr(self.agent, 'max_force', getattr(self, 'max_force', 200.0))
            steering = avoidance_dir * strength * max_force
            return steering
        return Vector2(0,0)

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
        self.steering = SteeringBehaviour(self)
        self.state = "wander"
        self.locked = False
        self.group_id = None
        self.last_state_change = 0.0
        self.max_force = 500.0
        if self.vel.length() < 0.01:
            self.vel = Vector2(random.uniform(-0.1,0.1), random.uniform(-0.1,0.1))
        self.hp = getattr(self, 'hp', 0)

    def _neighbors(self, enemies, radius=None):
        if radius is None:
            radius = self.CLUSTER_RADIUS if hasattr(self, 'CLUSTER_RADIUS') else 80.0
        n = []
        for e in enemies:
            if e is self:
                continue
            if (e.pos - self.pos).length() <= radius:
                n.append(e)
        return n

    def _find_group(self, enemies):
        visited = set()
        to_visit = [self]
        group = []
        while to_visit:
            cur = to_visit.pop()
            if cur in visited:
                continue
            visited.add(cur)
            if getattr(cur, 'locked', False):
                continue
            group.append(cur)
            for nb in cur._neighbors(enemies):
                if nb not in visited and not getattr(nb, 'locked', False):
                    to_visit.append(nb)
        return group

    def _lock_group(self, group, now_time):
        gid = int(now_time * 1000)
        for e in group:
            e.locked = True
            e.group_id = gid
            e.state = "attack"
            e.last_state_change = now_time

    def _unlock_group(self, group):
        for e in group:
            e.locked = False
            e.group_id = None
            e.state = "wander"

    def update(self, player, enemies, obstacles, dt, now):
        #function that makes lone enemies evade player if they spot him
        if not getattr(self, 'locked', False) and self.state == "wander":
            dist_to_player = (player.pos - self.pos).length()
            SPOT_PLAYER_DIST = getattr(self, 'SPOT_PLAYER_DIST', 150.0)
            if dist_to_player < SPOT_PLAYER_DIST:
                self.state = "evade"
                self.last_state_change = now

        if not getattr(self, 'locked', False):
            group = self._find_group(enemies)
            if len(group) >= getattr(self, 'MIN_GROUP_SIZE', 4):
                self._lock_group(group, now)

        steering = Vector2(0, 0)

        if getattr(self, 'locked', False):
            if hasattr(player, "vel"):
                steering = self.steering.pursue(player)
            else:
                steering = self.steering.seek(player.pos)
            steering += self.steering.obstacle_avoidance(obstacles, detectionRadius=100)
        else:
            w = self.steering.wander(wanderRadius=50.0, wanderDistance=100.0, wanderJitter=40.0, dt=dt)
            steer_obs = self.steering.obstacle_avoidance(obstacles, detectionRadius=100)
            steering += w + steer_obs

            dist_to_player = (player.pos - self.pos).length()
            if dist_to_player < getattr(self, 'AVOID_PLAYER_DIST', 200.0):
                flee_force = self.steering.flee(player.pos)
                t = max(0.0, (getattr(self, 'AVOID_PLAYER_DIST', 200.0) - dist_to_player) / getattr(self, 'AVOID_PLAYER_DIST', 200.0))
                steering += flee_force * (0.5 + 0.5 * t)

            nearby = [e for e in enemies if e is not self and (e.pos - self.pos).length() <= getattr(self, 'JOIN_DISTANCE', 150.0) and not getattr(e, 'locked', False)]
            if nearby:
                centroid = Vector2(0, 0)
                for nb in nearby:
                    centroid += nb.pos
                centroid /= len(nearby)
                to_centroid = (centroid - self.pos)
                if to_centroid.length() > 0:
                    desired = to_centroid.normalize() * self.maxSpeed
                    steering += (desired - self.vel) * 0.5

        # separacja — zapobiega nachodzeniu się wrogów
        separation_force = Vector2(0, 0)
        sep_count = 0
        SEPARATION_RADIUS = getattr(self, 'SEPARATION_RADIUS', 40.0)
        SEPARATION_WEIGHT = getattr(self, 'SEPARATION_WEIGHT', 1.5)

        for other in enemies:
            if other is self:
                continue
            offset = self.pos - other.pos
            dist = offset.length()
            if 0 < dist < SEPARATION_RADIUS:
                if dist > 0:
                    separation_force += offset.normalize() * (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS
                else:
                    separation_force += Vector2(random.uniform(-1,1), random.uniform(-1,1))
                sep_count += 1

        if sep_count > 0:
            separation_force /= sep_count
            if separation_force.length() > 0:
                separation_force = separation_force.normalize() * self.maxSpeed - self.vel
            steering += separation_force * SEPARATION_WEIGHT

        force = steering
        if force.length() > self.max_force:
            force = force.normalize() * self.max_force

        acceleration = force / max(1.0, self.mass)
        self.vel += acceleration * dt

        if self.vel.length() > self.maxSpeed:
            self.vel = self.vel.normalize() * self.maxSpeed

        self.pos += self.vel * dt

    def draw(self):
        color = (0, 255, 0) if not getattr(self, 'locked', False) else (200, 40, 40)
        pygame.draw.circle(self.surf, color, (int(self.pos.x), int(self.pos.y)), self.radius)