import pygame
import time

pygame.init()
pygame.event.set_grab(True)

running = True
while running:
    print(pygame.key.get_focused())
    # buttons = pygame.key.get_pressed()
    # print(buttons)
    time.sleep(.5)

pygame.quit()