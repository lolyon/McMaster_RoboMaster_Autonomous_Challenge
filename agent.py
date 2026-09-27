"""
agent template for the nav challenge
"""
import math

class Agent:
  def __init__(self, cfg:dict):
    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg

    self.width = cfg["width_m"]
    self.height = cfg["height_m"]
    self.resolution = cfg["resolution"]
    self.robot_radius = cfg["robot_radius"]
    self.v_max = cfg["v_max"]
    self.a_max = cfg["a_max"]
    self.dt = cfg["dt"]
    self.sense_cells = cfg["sense_cells"]
    self.goal_tol = cfg["goal_tol"]
    self.goal = cfg["goal"]
    self.width_cells = math.floor(self.width / self.resolution)
    self.height_cells = math.floor(self.height / self.resolution)

    self.world = [["?"] * self.width_cells for _ in range(self.height_cells)]

  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float, float]:
    """
    called once per tick.

    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans.
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """

    '''
    Initially I created a map, because it is unknown i used '?' to create a world for the robot where everything is unknown.
    I used pose to go the location where we have to start updating the world. 
    And becasue the world is created in __init__ the world keeps updating with each new scan.
    '''

    pose_x, pose_y = pose
    cx0, cy0, rows = scan

    scan_size = 2 * self.sense_cells + 1
 
    for i in range(scan_size):   #Goes to the location and replaces '?' with scanned information.
      for j in range(scan_size):
        self.world[cy0 + i][cx0 + j] = rows[j][i]

    return (0.0, 0.0)   # stop by default



  def caution(self): 
    '''
    Tells us the route we should avoid.
    1) No going too close to the objects because the robot size it 0.3m.
    2) Avoid taking diagonal route - avoid planning a route that is risky.
    '''

    '''
    Because the robot size is 0.3m that means 3 cells. We need to avoid the obstacle, starting from 
    3 cells away from the obstacle. If its exactly 3 cells away, it can theoretically 
    but practically anything can happen so we just mark it just in case.

    We leave the unknown spot as it is.
    We surround the obstacle "#" with 1,2,3 that tells us how much further the obstacle actually is.
    We go above, below and sideways as well.

    By marking these spots we can also avoid the robot taking a diagonal route as well.  
    '''
    location = []

    for i in range(self.height_cells):
      for j in range(self.width_cells):
        if self.world[j][i] == "#":
          location.append([j,i])

    for i in range(len(location)):
      x, y = location[i]

      for j in range(1,4):
        if y - j >= 0 and self.world[y - j][x] == ".":
          self.world[y - j][x] = str(j)

        if y + j < self.height_cells and self.world[y + j][x] == ".":
          self.world[y + j][x] = str(j)

        if x - j >= 0 and self.world[y][x - j] == ".":
          self.world[y][x - j] = str(j)

        if x + j < self.width_cells and self.world[y][x + j] == ".":
          self.world[y][x + j] = str(j)

    for i in range(self.height_cells):
      for j in range(self.width_cells):
        if self.world[i][j] == "1":
          if i - 1 >= 0 and self.world[i - 1][j] == ".":
            self.world[i - 1][j] = "1"
          if i - 2 >= 0 and self.world[i - 2][j] == ".":
            self.world[i - 2][j] = "1"
          if i - 3 >= 0 and self.world[i - 3][j] == ".":
            self.world[i - 3][j] = "1"
          if i + 1 < self.height_cells and self.world[i + 1][j] == ".":
            self.world[i + 1][j] = "1"
          if i + 2 < self.height_cells and self.world[i + 2][j] == ".":
            self.world[i + 2][j] = "1"
          if i + 3 < self.height_cells and self.world[i + 3][j] == ".":
            self.world[i + 3][j] = "1"

        elif self.world[i][j] == "2":
          if i - 1 >= 0 and self.world[i - 1][j] == ".":
            self.world[i - 1][j] = "2"
          if i - 2 >= 0 and self.world[i - 2][j] == ".":
            self.world[i - 2][j] = "2"
          if i - 3 >= 0 and self.world[i - 3][j] == ".":
            self.world[i - 3][j] = "2"
          if i + 1 < self.height_cells and self.world[i + 1][j] == ".":
            self.world[i + 1][j] = "2"
          if i + 2 < self.height_cells and self.world[i + 2][j] == ".":
            self.world[i + 2][j] = "2"
          if i + 3 < self.height_cells and self.world[i + 3][j] == ".":
            self.world[i + 3][j] = "2"


    return (0,0)

  def path(self, start, goal):
      '''
      First the function goes to the starting point. 
      Then let's say the goal is to go to a location above the starting point; it does the following things.

      First it scans the row and looks for places to go.
      If there is any blockade like 1, 2, 3, or #, it just leaves it and doesn't go beyond. 
      So imagine all these spots have a green text.
      Now using these green spots, the code checks which all possible directions
      it can go the line above. and then repeats the same exact thing. 
      Now, during this process, there might be multiple possible ways to reach the 
      final goal point. But the function finds the shortest path.
      '''
      paths = [[start]]
      visited = {start}

      while paths:
          # Take the next path to explore.
          current_path = paths.pop(0)
          x, y = current_path[-1]

          if (x, y) == goal:
              return current_path

          # Only move horizontally or vertically. # This prevents diagonal corner cutting.
          directions = [
              (0, -1),  # up
              (0, 1),   # down
              (-1, 0),  # left
              (1, 0),   # right
          ]

          for dx, dy in directions:
              nx = x + dx
              ny = y + dy

              if nx < 0 or nx >= self.width_cells:
                  continue

              if ny < 0 or ny >= self.height_cells:
                  continue


              # Do not plan through obstacles or caution cells.
              if self.world[ny][nx] in ["#", "1", "2", "3"]:
                  continue

              if (nx, ny) in visited:
                  continue


              # Avoid visiting the same cell more than once.
              visited.add((nx, ny))

              new_path = current_path + [(nx, ny)]
              paths.append(new_path)

      return []


  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """
    return {}
  