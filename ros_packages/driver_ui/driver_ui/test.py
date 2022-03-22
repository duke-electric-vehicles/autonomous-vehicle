import pygame, sys
from pygame.locals import*

pygame.init()
SCREENWIDTH = 800
SCREENHEIGHT = 800
RED = (255,0,0)
screen = pygame.display.set_mode((SCREENWIDTH, SCREENHEIGHT))

pygame.draw.rect(screen, RED, (400, 400, 20, 20),0)
screen.fill(RED)




pygame.display.update()