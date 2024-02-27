import pygame

def main():
     # This is the entry point of the package, as specified in the setup.py file
    pygame.init()

    pygame.display.set_caption('Quick Start')
    window_surface = pygame.display.set_mode((800, 600))

    background = pygame.Surface((800, 600))
    background.fill(pygame.Color('#000000'))

    is_running = True

    while is_running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running = False

        window_surface.blit(background, (0, 0))
        pygame.display.update()

# This is the entry point of the package, as specified in the setup.py file
if __name__ == '__main__':
    main()