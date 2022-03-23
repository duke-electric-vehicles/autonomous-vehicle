import pygame

pygame.init()

screen = pygame.display.set_mode((0, 0), pygame.RESIZABLE)

clock = pygame.time.Clock()

while True:
    screen.fill((255,0,0))

    pygame.draw.rect(screen, (0,255,0), [0,0,200,200])
    
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            pygame.quit()
            raise SystemExit            

    clock.tick(60)
