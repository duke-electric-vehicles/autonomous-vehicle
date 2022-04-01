import pygame as pg


pg.init()
screen = pg.display.set_mode((640, 480))
BG_COLOR = pg.Color(30, 30, 50)

HEAT_BAR_IMAGE = pg.Surface((250, 20))
#color = pg.Color(0, 255, 0)
color = pg.Color(255, 0, 0)
color2 = pg.Color(0, 0, 255)
# Fill the image with a simple gradient.
for x in range(round(HEAT_BAR_IMAGE.get_width() / 2)):
    for y in range(HEAT_BAR_IMAGE.get_height()):
        HEAT_BAR_IMAGE.set_at((x, y), color)
    
    if color.g < 254:
        color.g += 2
    if color.r > 1:
        color.r -= 2
for x in range(round(HEAT_BAR_IMAGE.get_width() / 2), HEAT_BAR_IMAGE.get_width()):
    for y in range(HEAT_BAR_IMAGE.get_height()):
        HEAT_BAR_IMAGE.set_at((x, y), color)
    
    if color.r < 254:
        color.r += 2
    if color.g > 1:
        color.g -= 2
    

    '''
    if color.r > 100 and color.r < 254:
        color.r += 2
    if color.g > 1:
        color.g -= 2
    '''

def main():
    clock = pg.time.Clock()
    heat_rect = HEAT_BAR_IMAGE.get_rect(topleft=(200, 100))
    ticker_rect = HEAT_BAR_IMAGE.get_rect(topleft=(200, 10))
    # `heat` is the percentage of the surface's width and
    # is used to calculate the visible area of the image.
    
    heat = 50  # 5% of the image are already visible.
    done = False

    while not done:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                done = True

        keys = pg.key.get_pressed()
        if keys[pg.K_x]:
            heat += 1  # Now 4% more are visible.
        if keys[pg.K_c]:
            heat -= 1

        #heat -= 0.5  # Reduce the heat every frame.
        heat = max(1, min(heat, 100))  # Clamp the value between 1 and 100.
        # Pass a rect or tuple as the `area` argument.
            # Use the `heat` percentage to calculate the current width. 
        screen.fill(BG_COLOR)
        screen.blit(HEAT_BAR_IMAGE, heat_rect, (100, 100, heat_rect.w/100 * heat, heat_rect.h - 10))
        screen.blit(HEAT_BAR_IMAGE, heat_rect, (0, 0, heat_rect.w, heat_rect.h))

        pg.display.flip()
        clock.tick(30)


if __name__ == '__main__':
    main()
    pg.quit()
