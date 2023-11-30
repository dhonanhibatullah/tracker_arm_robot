import pygame as pg
import numpy as np



class TarGraphic:



    def __init__(self) -> None:
        
        # Constants
        self.WINDOW_TITLE       = 'Tracker Arm Robot'
        self.SCREEN_WIDTH       = 800
        self.SCREEN_HEIGHT      = 800
        self.SCREEN_CENTER_X    = self.SCREEN_WIDTH//2
        self.SCREEN_CENTER_Y    = self.SCREEN_HEIGHT//2
        self.FPS                = 60
        self.JOINT_RADIUS       = 22
        self.LINK_RADIUS        = 10
        self.LINK_LENGTH        = 180
        self.JOINT_COLOR        = (108, 163, 109)
        self.LINK_COLOR         = (255, 255, 255)
        self.BG_COLOR           = (20, 20, 20)
        self.WS_COLOR           = (0, 0, 0)
        self.CURSOR_COLOR1      = (66, 165, 240)
        self.CURSOR_COLOR2      = (240, 66, 75)
        self.CURSOR_RADIUS      = 15


        # Initiate pygame
        pg.init()
        pg.display.set_caption(self.WINDOW_TITLE)
        self.__screen__ = pg.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        self.__clock__  = pg.time.Clock()



    def getMouseHover(self) -> list:
        x, y = pg.mouse.get_pos()
        x = 2.0*(x - self.SCREEN_CENTER_X)/(2*self.LINK_LENGTH)
        y = 2.0*(self.SCREEN_CENTER_Y - y)/(2*self.LINK_LENGTH)
        
        # Return only if mouse inside the workspace
        if x**2.0 + y**2.0 < 4.0:
            return [x, y, True]
        else:
            return [x, y, False]


    
    def stepRender(self, joint1_theta:float, joint2_theta:float) -> None:
        
        # Fill screen
        self.__screen__.fill(self.BG_COLOR)
        pg.draw.circle(self.__screen__, self.WS_COLOR, [self.SCREEN_CENTER_X, self.SCREEN_CENTER_Y], int(2*self.LINK_LENGTH), 0)


        # Terminate process if window is closed
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                exit()


        # Get value from arguments
        theta1  = -joint1_theta
        theta2  = -joint2_theta


        # Calculate the polygon points
        joint1_circ = [self.SCREEN_CENTER_X, self.SCREEN_CENTER_Y]
        joint2_circ = [int(self.LINK_LENGTH*np.cos(theta1)) + joint1_circ[0], int(self.LINK_LENGTH*np.sin(theta1)) + joint1_circ[1]]
        joint3_circ = [int(self.LINK_LENGTH*np.cos(theta1 + theta2)) + joint2_circ[0], int(self.LINK_LENGTH*np.sin(theta1 + theta2)) + joint2_circ[1]]
        link1_rect  = [
            [int(self.LINK_RADIUS*np.sin(theta1)) + joint1_circ[0], int(-self.LINK_RADIUS*np.cos(theta1)) + joint1_circ[1]],
            [joint2_circ[0] + int(self.LINK_RADIUS*np.sin(theta1)), joint2_circ[1] - int(self.LINK_RADIUS*np.cos(theta1))],
            [joint2_circ[0] - int(self.LINK_RADIUS*np.sin(theta1)), joint2_circ[1] + int(self.LINK_RADIUS*np.cos(theta1))],
            [int(-self.LINK_RADIUS*np.sin(theta1)) + joint1_circ[0], int(self.LINK_RADIUS*np.cos(theta1)) + joint1_circ[1]]
        ]
        link2_rect  = [
            [int(self.LINK_RADIUS*np.sin(theta1 + theta2)) + joint2_circ[0], int(-self.LINK_RADIUS*np.cos(theta1 + theta2)) + joint2_circ[1]],
            [joint3_circ[0] + int(self.LINK_RADIUS*np.sin(theta1 + theta2)), joint3_circ[1] - int(self.LINK_RADIUS*np.cos(theta1 + theta2))],
            [joint3_circ[0] - int(self.LINK_RADIUS*np.sin(theta1 + theta2)), joint3_circ[1] + int(self.LINK_RADIUS*np.cos(theta1 + theta2))],
            [int(-self.LINK_RADIUS*np.sin(theta1 + theta2)) + joint2_circ[0], int(self.LINK_RADIUS*np.cos(theta1 + theta2)) + joint2_circ[1]]
        ]


        # Draw the arm
        pg.draw.polygon(self.__screen__, self.LINK_COLOR, link1_rect, 0)
        pg.draw.polygon(self.__screen__, self.LINK_COLOR, link2_rect, 0)
        pg.draw.circle(self.__screen__, self.JOINT_COLOR, joint1_circ, self.JOINT_RADIUS, 0)
        pg.draw.circle(self.__screen__, self.JOINT_COLOR, joint2_circ, self.JOINT_RADIUS, 0)
        pg.draw.circle(self.__screen__, self.JOINT_COLOR, joint3_circ, self.JOINT_RADIUS, 0)


        # Draw cursor
        x, y = pg.mouse.get_pos()
        cursor_circ = [x, y]
        if (x - self.SCREEN_CENTER_X)**2.0 + (y - self.SCREEN_CENTER_Y)**2.0 < 4.0*(self.LINK_LENGTH**2.0):
            pg.draw.circle(self.__screen__, self.CURSOR_COLOR1, cursor_circ, self.CURSOR_RADIUS, 0)
        else:
            pg.draw.circle(self.__screen__, self.CURSOR_COLOR2, cursor_circ, self.CURSOR_RADIUS, 0)


        # Update display
        pg.display.update()
        self.__clock__.tick(self.FPS)