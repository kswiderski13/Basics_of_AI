import sys
import random
import pygame
from pygame.locals import *
from pygame.math import Vector2

import classes

WIDTH, HEIGHT      = 800, 600    # rozmiary okna gry
FPS                = 60
NUM_ENEMIES        = 15          #liczba botów
MIN_CLUSTER_SIZE   = 4           # minimalna liczba botów w grupie
CLUSTER_RADIUS     = 80          # promień wykrywania grupy
PLAYER_SIZE        = 15          # rozmiar gracza

BG_COLOR   = (30, 30, 30)
WALL_COLOR = (0, 100, 255)

def draw_walls(screen):                   #ściany
    thickness = 10                        #grubość ścian
    pygame.draw.rect(screen, WALL_COLOR, (0, 0, WIDTH, thickness)) #GÓRA
    pygame.draw.rect(screen, WALL_COLOR, (0, HEIGHT - thickness, WIDTH, thickness)) #DÓŁ
    pygame.draw.rect(screen, WALL_COLOR, (0, 0, thickness, HEIGHT)) #LEWO
    pygame.draw.rect(screen, WALL_COLOR, (WIDTH - thickness, 0, thickness, HEIGHT)) #PRAWO

def cluster_detection(enemies):                    # wykrywanie grup botów
    explorers = [e for e in enemies if e.state == "explore"]      #boty w stanie "explore"
    visited = set()
    for e in explorers:
        if e in visited:
            continue
        cluster = [
            other for other in explorers
            if other.pos.distance_to(e.pos) <= CLUSTER_RADIUS
        ]
        if len(cluster) >= MIN_CLUSTER_SIZE:
            for mem in cluster:
                mem.state = "attack"
            visited.update(cluster)

def get_free_position(size, obstacles, margin=5):       #znajdź wolną pozycję na mapie
    while True:
        x = random.uniform(size + margin, WIDTH  - size - margin)   #pozycja x
        y = random.uniform(size + margin, HEIGHT - size - margin)   #pozycja y
        p = Vector2(x, y)                                           #wektor pozycji
        if all(p.distance_to(ob.pos) >= size + ob.radius + margin for ob in obstacles): #sprawdź kolizje
            return p

# główna pętla gry
def main():
    pygame.init() 
    screen = pygame.display.set_mode((WIDTH, HEIGHT)) 
    pygame.display.set_caption("Zombie Survival") #tytuł okna
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)
    high_score = 0

    while True:
        score            = 0
        hp               = 100
        level            = 0
        enemy_min_speed  = 25
        enemy_max_speed  = 50

        obstacles = classes.generate_obstacles(         # liczba przeszkód
            count=10,
            width=WIDTH,
            height=HEIGHT,
            player_size=PLAYER_SIZE,
            margin=2
        )

        player_pos = get_free_position(PLAYER_SIZE, obstacles) #pozycja gracza
        player     = classes.Player(player_pos) 

        enemies = [
            classes.Enemy(get_free_position(PLAYER_SIZE, obstacles),     #pozycja bota
                          enemy_min_speed, enemy_max_speed)              #prędkość bota
            for _ in range(NUM_ENEMIES)
        ]

        bullets   = []
        game_over = False

# pętla gry
        while True:      
            dt      = clock.tick(FPS) / 1000.0
            now     = pygame.time.get_ticks() / 1000.0 
            restart = False

            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    pygame.quit()
                    sys.exit()
                if not game_over and event.type == MOUSEBUTTONDOWN and event.button == 1:
                    b = player.shoot()
                    if b:
                        bullets.append(b) 
                if game_over and event.type == KEYDOWN and event.key == K_r:
                    restart = True

            if restart:
                break

            if not game_over:
                # aktualizacja gracza i botów
                player.update(dt, obstacles)
                for ob in obstacles:
                    ob.update(dt)
                for e in enemies:
                    e.update(dt, player, obstacles)

                # aktualizacja pocisków i sprawdzanie kolizji
                for b in bullets[:]:
                    b.update(dt)

                    blocked = False
                    for ob in obstacles:
                        if b.pos.distance_to(ob.pos) < b.radius + ob.radius:
                            bullets.remove(b)
                            blocked = True
                            break
                    if blocked:
                        continue
                        # kolizja pocisk-bot
                    hit = False
                    for e in enemies[:]:
                        if b.pos.distance_to(e.pos) < b.radius + e.size:
                            # przyznaj punkty
                            points = 1 if e.state == "explore" else 5
                            score += points
                            # aktualizuj najlepszy wynik
                            if score > high_score:
                                high_score = score

                            bullets.remove(b)
                            enemies.remove(e)
                            # spawn nowego bota
                            new_pos = get_free_position(PLAYER_SIZE, obstacles)
                            enemies.append(classes.Enemy(new_pos,
                                                         enemy_min_speed,
                                                         enemy_max_speed))
                            # co 20 pkt level up
                            if score // 20 > level:
                                level = score // 20
                                enemy_min_speed += 1
                                enemy_max_speed += 1
                            hit = True
                            break
                    if hit:
                        continue

                    if not (0 <= b.pos.x <= WIDTH and 0 <= b.pos.y <= HEIGHT):
                        bullets.remove(b)

                # kolizja bot-gracz
                for e in enemies:
                    if e.pos.distance_to(player.pos) < e.size + player.size:
                        if now - e.last_attack >= 1.0:
                            hp -= 5
                            e.last_attack = now
                            if hp <= 0:
                                game_over = True
                        break

                cluster_detection(enemies)

            # rysowanie
            screen.fill(BG_COLOR)
            draw_walls(screen)
            for ob in obstacles:
                ob.draw(screen)
            for e in enemies:
                e.draw(screen)
            for b in bullets:
                b.draw(screen)
            player.draw(screen)

            # HUD: wynik, najlepszy wynik i HP
            score_surf = font.render(f"Score: {score}  High: {high_score}", True, (255, 255, 255))
            hp_surf    = font.render(f"HP: {hp}", True, (255, 255, 255))
            screen.blit(score_surf, (10, 10))
            screen.blit(hp_surf,    (10, 30))

            if game_over:
                go_text = font.render(f"Game Over! Score: {score}", True, (255, 255, 255))
                rr_text = font.render("Press R to Restart", True, (255, 255, 255))
                go_rect = go_text.get_rect(center=(WIDTH//2, HEIGHT//2 - 20))
                rr_rect = rr_text.get_rect(center=(WIDTH//2, HEIGHT//2 + 20))
                screen.blit(go_text, go_rect)
                screen.blit(rr_text, rr_rect)

            pygame.display.flip()

         #Restart [r]
        continue

if __name__ == "__main__":
    main()