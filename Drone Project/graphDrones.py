import pygame
import os
from pygame.locals import *

pygame.init()
pygame.mixer.init()

screen=pygame.display.set_mode((640, 480))
black=pygame.Color("black")
white=pygame.Color("white")
yellow=pygame.Color("yellow")
pygame.mouse.set_visible(False)
FPS = 10
clock = pygame.time.Clock()

def parse(line):
	coordinates = line.split(':')
	coordinates[2] = coordinates[2][:-1]
	coordinates[3] = coordinates[3][:-1]
	return float(coordinates[2]), float(coordinates[3]), float(coordinates[4])

def main():
	f = open('Position.txt', 'r')
	x = 0.0
	y = 0.0
	z = 0.0
	while 1:
		for line in f:
			pygame.draw.polygon(screen, black, ((x-5, y), (x+5, y), (x, y+5)), 0)
			x,y,z = parse(line)
			pygame.draw.polygon(screen, yellow, ((x-5, y), (x+5, y), (x, y+5)), 0)
			pygame.display.update()
			clock.tick(FPS)

		
if __name__=='__main__':
	main()