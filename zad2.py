import pygame
import random
from pygame.math import Vector2

from classes2 import (
    dummybot,
    PathPlanner,
    NavigationNode,
    Bullet,
    Rocket,
    Pickup,
    build_nav_graph_flood_fill,
    point_in_poly,
    resolve_wall_penetration,
    draw_nav_graph

)

pygame.init()

WIDTH, HEIGHT = 800, 800
display = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
fps = 60

MAP_RECT = pygame.Rect(0, 0, WIDTH, HEIGHT)

OBSTACLES = [
    pygame.Rect(0, 0, WIDTH, 40),
    pygame.Rect(0, HEIGHT - 40, WIDTH, 40),
    pygame.Rect(0, 0, 40, HEIGHT),
    pygame.Rect(WIDTH - 40, 0, 40, HEIGHT),

    pygame.Rect(300, 250, 200, 40),
    pygame.Rect(300, 500, 200, 40),

    pygame.Rect(150, 150, 40, 200),
    pygame.Rect(650, 150, 40, 200),
    pygame.Rect(150, 450, 40, 200),
    pygame.Rect(650, 450, 40, 200),

    pygame.Rect(250, 350, 40, 40),
    pygame.Rect(550, 350, 40, 40),

    pygame.Rect(650, 280, 110, 140)
]

POLY_OBSTACLES = [
    [(100, 300), (150, 280), (150, 320)],
    
]

SPAWNS = [
    (100, 100), (200, 100), (300, 100),
    (500, 100), (600, 100), (700, 100),

    (100, 700), (200, 700), (300, 700),
    (500, 700), (600, 700), (700, 700),
]


def main():
    nav_graph = build_nav_graph_flood_fill(
        MAP_RECT,
        15,
        OBSTACLES,
        POLY_OBSTACLES
    )

    bots = []
    bullets = []
    rockets = []

    pickups = [
        Pickup((400, 200), "health", 25, display),
        Pickup((400, 600), "rail", 5, display),
        Pickup((400, 400), "rocket", 3, display),
    ]

    used_spawns = random.sample(SPAWNS, 4)

    for pos in used_spawns:
        bot = dummybot(Vector2(pos), display)
        bot.planner = PathPlanner(bot, nav_graph)
        bots.append(bot)

    running = True
    while running:
        dt = clock.tick(fps) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for bot in bots:
            bot.update(dt, bots, OBSTACLES, pickups, bullets, rockets, MAP_RECT)

        for b in bullets[:]:
            b.update(dt, OBSTACLES)
            if not b.alive:
                bullets.remove(b)
                continue

            bullet_rect = pygame.Rect(b.pos.x - b.radius, b.pos.y - b.radius, b.radius * 2, b.radius * 2)
            hit = False

            for r in OBSTACLES:
                if bullet_rect.colliderect(r):
                    hit = True
                    break

            if not hit:
                for poly in POLY_OBSTACLES:
                    if point_in_poly(b.pos.x, b.pos.y, poly):
                        hit = True
                        break

            if hit:
                bullets.remove(b)
                continue

        for r in rockets[:]:
            r.update(dt, bots, OBSTACLES)
            if not r.alive:
                rockets.remove(r)
                continue

            rocket_rect = pygame.Rect(r.pos.x - r.radius, r.pos.y - r.radius, r.radius * 2, r.radius * 2)
            hit = False

            for ob in OBSTACLES:
                if rocket_rect.colliderect(ob):
                    r.explode(bots)
                    hit = True
                    break

            if not hit:
                for poly in POLY_OBSTACLES:
                    if point_in_poly(r.pos.x, r.pos.y, poly):
                        r.explode(bots)
                        hit = True
                        break

            if hit or not r.alive:
                rockets.remove(r)

        display.fill((20, 20, 20))


        for r in OBSTACLES:
            pygame.draw.rect(display, (60, 60, 60), r)

        for poly in POLY_OBSTACLES:
            pygame.draw.polygon(display, (80, 80, 80), poly)

        draw_nav_graph(display, nav_graph)     

        for p in pickups:
            p.draw(display)

        for b in bots:
            b.draw(display)

        for b in bullets:
            b.draw(display)

        for r in rockets:
            r.draw(display)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()