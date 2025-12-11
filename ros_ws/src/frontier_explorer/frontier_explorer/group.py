from nav_msgs.msg import OccupancyGrid
import math

class Group():
    def __init__(self, grid : OccupancyGrid, id):
        self.group_id = id
        self.cells = []
        self.center_cell = None

        self.grid = grid
        self.grid_width = grid.info.width
        self.grid_height = grid.info.height
        self.grid_size = grid.info.width * grid.info.height

    def add_cell(self, index):
        self.cells.append(index)
        self.update_center()

    def update_center(self):
        sum_x = 0
        sum_y = 0

        for cell_i in self.cells:
            sum_x += cell_i % self.grid_width
            sum_y += math.floor(cell_i/self.grid_width)

        self.center_cell = (sum_x/len(self.cells)) + (self.grid_width * (sum_y/len(self.cells)))

    def get_closest_edge(self, bot_i):
        shortest_dist = 100000
        closest_i = None

        bot_x = bot_i % self.grid_width
        bot_y = math.floor(bot_i / self.grid_width)

        for cell_i in self.cells:
            cell_x = cell_i % self.grid_width
            cell_y = math.floor(cell_i/self.grid_width)

            delta_x = abs(cell_x - bot_x)
            delta_y = abs(cell_y - bot_y)

            dist = math.sqrt(delta_x**2 + delta_y**2)

            if(closest_i is None or dist < shortest_dist):
                shortest_dist = dist
                closest_i = cell_i

        return closest_i

    def get_center_dist(self, bot_i):
        center_x = self.center_cell % self.grid_width
        center_y = math.floor(self.center_cell / self.grid_width)

        bot_x = bot_i % self.grid_width
        bot_y = math.floor(bot_i / self.grid_width)

        delta_x = abs(center_x - bot_x)
        delta_y = abs(center_y - bot_y)

        return math.sqrt(delta_x ** 2 + delta_y ** 2)