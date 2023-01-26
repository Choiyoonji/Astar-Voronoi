import pygame


pygame.init()

SCREEN_WIDTH = 600
SCREEN_HEIGHT = 600
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

clock = pygame.time.Clock()

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
CELL_SIZE = 200


ball = pygame.Rect(SCREEN_WIDTH // 2 - 16 // 2, SCREEN_HEIGHT // 2 - 16 // 2, 16, 16)

coordinate = []

while True:
    screen.fill(BLACK)
    
    pos = None
    event = pygame.event.poll()
    
    if event.type == pygame.QUIT:
        break
    elif event.type == pygame.MOUSEBUTTONDOWN:
        pos = pygame.mouse.get_pos()
        coordinate.append(pos)
        for i in coordinate:
            pygame.draw.circle(screen, WHITE, i, ball.width // 2)
        print(pos,',')
        
        
    pygame.display.update()
    clock.tick(30)