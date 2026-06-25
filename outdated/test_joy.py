import pygame
print('start')
pygame.init(); pygame.joystick.init()
print('joysticks', pygame.joystick.get_count())
for i in range(pygame.joystick.get_count()):
    j = pygame.joystick.Joystick(i); j.init(); print('name', j.get_name())
print('done')