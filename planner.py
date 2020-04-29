import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from sklearn.neighbors import KDTree
import cProfile



def fn_straight_line(start, end, distance=None):

    vector = np.array(end) - np.array(start)
    if distance:
        vector = vector / np.linalg.norm(vector) * distance
    dx, dy = list(vector)
    coordinates = [start[0]+dx, start[1]+dy]
    if distance is None:
        distance = np.sqrt(dx**2 + dy**2)
    return coordinates, distance


class PathNode:
    """
    RRT Node
    """

    def __init__(self, coordinates, parent=None, cost=None):
        """
        Args:
            coordinates (list/tuple): Coordinates (x, y)
        """
        self.coordinates = coordinates
        self.path = []
        self.parent = parent
        self.children = set()
        self.cost = cost
        self.truncated = False  # Flag as trimmed during due to length
        self.collision_avoided = False  # Flag if trimmed due to collision

    def close_to_node(self, other_node, epsilon=0.01):
        delta = np.array(self.coordinates) - np.array(other_node.coordinates)
        if np.linalg.norm(delta) < epsilon:
            return True
        return False

    def cost_to_node(self, other_node, cost_fn=fn_straight_line):
        _, distance = cost_fn(self.coordinates, other_node.coordinates)
        return distance

    def update_cost(self, cost_fn=fn_straight_line):
        self.cost = self.parent.cost + self.cost_to_node(self.parent, cost_fn=cost_fn)

    def path_same_as_parent(self, distance, epsilon=0.01):
        parent_distance = self.cost_to_node(self.parent)
        if abs(parent_distance - distance) < epsilon:
            return True
        return False


class RrtStar:

    def __init__(self, start, goal, obstacle_list, map_limits,
                 path_max=5, path_elements=10, max_iter=1000, goal_sample_rate=0.1):
        """
        Args:
            start (list/tuple): Start coordinates (x, y)
            goal (list/tuple): Goal Coordinates (x, y)
            obstacle_list (list): List of obstacles [[x, y, dx, dy], ...]
            map_limits (list/tuple): Search space boundaries [x_min, m_max, y_min, y_max]
            path_max (number): Maximum path length
            path_elements (int): Resolution of path to check for collisions
            max_iter (int): Maximum number of iterations to search
            goal_sample_rate (number): Rate at which to randomly sample goal position as next node (0 -> 1).
                                        This creates a bias towards exploring in the goal direction.
        """
        self.obstacle_list = obstacle_list
        # Create kd-tree of obstacles using (x, y) coordinates
        self.obstacle_tree = KDTree([x[:2] for x in obstacle_list])

        if self.point_collision_free(start):
            self.start = PathNode(start, cost=0)
        else:
            raise ValueError("Start position in collision")

        if self.point_collision_free(goal):
            self.goal = PathNode(goal)
        else:
            raise ValueError("Goal position in collision")
        
        self.map_limits = map_limits
        self.path_max = path_max
        self.path_elements = path_elements
        self.max_iter = max_iter
        self.node_list = []
        self.goal_sample_rate = goal_sample_rate
        self.goal_node = False

    def point_collision_free(self, point):
        """
        Args:
            point (list/tuple): Coordinates (x, y)
        """
        obstacle_list = []
        i_obstacles = self.obstacle_tree.query([point], k=1, return_distance=False)[0]
        for i_obstacle in i_obstacles:
            obstacle_list.append(self.obstacle_list[i_obstacle])
        for (ox, oy, odx, ody) in obstacle_list:
            dx = abs(ox - point[0])
            dy = abs(oy - point[1])
            if dx <= odx and dy <= ody:
                return False  # Collision
        return True  # safe
        

    def path_collision_free(self, path):
        """
        Args:
            path (list/tuple): Coordinates ((x, y), ...)
        """
        obstacle_list = []

        for coordinates in path:
            i_obstacles = self.obstacle_tree.query([coordinates], k=1, return_distance=False)[0]
            for i_obstacle in i_obstacles:
                obstacle_list.append(self.obstacle_list[i_obstacle])

        for (ox, oy, odx, ody) in obstacle_list:
            dx_list = [abs(ox - x[0]) for x in path]
            dy_list = [abs(oy - x[1]) for x in path]

            for dx, dy in zip(dx_list, dy_list):
                if dx <= odx and dy <= ody:
                    return False  # Collision

        return True  # safe

    def get_random_node(self):
        if np.random.random() > self.goal_sample_rate or self.goal_node:
            node = PathNode([
                np.random.uniform(self.map_limits[0], self.map_limits[1]),
                np.random.uniform(self.map_limits[2], self.map_limits[3])
            ])
        else:  # goal point sampling
            node = PathNode(self.goal.coordinates)
        return node

    def create_valid_path(self, node, cost_fn=fn_straight_line, limit_path=True):
        if node.parent is None:
            raise ValueError("Node requires a parent")
        
        # Move along subpath until distance limit is reached or obstacle is hit
        step_count = 1
        collision_free = True
        start = node.parent.coordinates
        distance_to_parent = node.cost_to_node(node.parent, cost_fn=cost_fn)
        if distance_to_parent == 0:
            return None

        updated_node = PathNode(start, parent=node.parent)
        updated_node.path = [start]

        if limit_path:
            path_length = min(self.path_max, distance_to_parent)
        else:
            path_length = distance_to_parent
        path_increment = path_length / self.path_elements

        while (step_count * path_increment <= path_length and collision_free):
            xy, _ = cost_fn(updated_node.path[-1], node.coordinates, distance=path_increment)
            # Check for collisison
            if self.point_collision_free(xy):
                updated_node.path.append(xy)
                updated_node.coordinates = xy
            else:
                collision_free = False
                updated_node.collision_avoided = True
            step_count += 1
        # If no path found return None
        if len(updated_node.path) <= 1:
            return None
        else:
            updated_node.update_cost(cost_fn=cost_fn)
            # Check if original node has been adjusted due to obstacles or maximum length
            if not updated_node.path_same_as_parent((step_count - 1) * path_increment):
                updated_node.truncated = True
            return updated_node
            
    def get_parent_node(self, node, node_tree=None, method=2):
        if node_tree is None:
            node_tree = KDTree([n.coordinates for n in self.node_list])

        # Method 1: FIn closest node then look for cheapest neighbour
        if method ==1:
            # Get closest nodes
            i_closest = node_tree.query([node.coordinates], k=1, return_distance=False)[0][0]
            # Look in radius around this for cheaper node
            i_nearby = node_tree.query_radius([self.node_list[i_closest].coordinates], r=self.path_max*0.5, return_distance=False)[0]
            costs = []
            for i_node in i_nearby:
                costs.append(self.node_list[i_node].cost)
            i_cheapest = i_nearby[np.argmin(costs)]

        # Method 2: FInd cheapest node from within radius
        if method == 2:
            n_neighbours = min(5, len(self.node_list))
            i_nearby = node_tree.query([node.coordinates], k=n_neighbours, return_distance=False)[0]
            costs = []
            for i_node in i_nearby:
                node_nearby = self.node_list[i_node]
                cost_to_neighbour = node_nearby.cost + node_nearby.cost_to_node(node)
                costs.append(cost_to_neighbour)
            
            i_cheapest = i_nearby[np.argmin(costs)]
        
        return self.node_list[i_cheapest]

    def plan(self, animation=False):
        print("Starting planning ...")
        # Reinitialise node list
        self.node_list = [self.start]
        self.goal_found = False
        print_iters = self.max_iter / 10
        for i in range(self.max_iter):
            if i % print_iters == 0:
                print("Iteration {}".format(i))
            # Select random node
            random_node = self.get_random_node()
            # Create KD Tree of nodes for multiple queries
            node_tree = KDTree([n.coordinates for n in self.node_list])
            # Pick parent node based upon proximity and cost
            random_node.parent = self.get_parent_node(random_node, node_tree=node_tree)
            # Update node to reflect path constraints
            valid_node = self.create_valid_path(random_node)
            # Assuming valid update graph
            if valid_node:
                if valid_node.close_to_node(self.goal):
                    self.goal_node = valid_node
                self.node_list.append(valid_node)
                valid_node.parent.children.add(valid_node)
                self.rewire(valid_node, node_tree)
            if animation:
                self.draw_map(animation=True)

        print("Reached max iterations")
        if self.goal_node:
            print("Goal found")
        else:
            print("Goal not found")
        

    def rewire(self, new_node, node_tree, path_fn=fn_straight_line):
        # Find nodes within radius equal to max path length
        i_nearby = node_tree.query_radius([new_node.coordinates], r=self.path_max, return_distance=False)[0]
        for i_node in i_nearby:
            # Look at each of these nodes in turn
            node = self.node_list[i_node]
            # Create temporary node with position at each nearby node
            # As a test, make the new node it's parent
            temp_node = PathNode(node.coordinates, parent=new_node)
            temp_node.update_cost()
        
            checked_node = self.create_valid_path(temp_node)
            if checked_node and not checked_node.truncated:  # This means path is complete and unobstructed
                # If this is better then update graph
                if checked_node.cost < self.node_list[i_node].cost:
                    # Update children
                    node.parent.children.remove(node)
                    new_node.children.add(checked_node)
                    # Replace node with new one
                    self.node_list[i_node] = checked_node
                    self.propogate_cost_to_leaves(checked_node)
    
    def propogate_cost_to_leaves(self, parent_node, check_all=True):
        """
        Update cost of downstream (leaves of tree) element after rewiring
        """
        if check_all:  # Check every node (old method)
            for node in self.node_list:
                if node.parent == parent_node:
                    node.update_cost()
                    self.propogate_cost_to_leaves(node)
        else:  # Check only recorded child nodes (intended to speed up)
            for child_node in parent_node.children:
                child_node.update_cost()
                self.propogate_cost_to_leaves(child_node)

    def get_path(self, optimise=True, return_type='points'):
        if self.goal_node:
            path = []
            node = self.goal_node
            while node.parent is not None:
                path.insert(0, node)
                node = node.parent
            path.insert(0, node) # Goal

            if optimise:
                # Remove unnecessary nodes
                for i_a, node_a in enumerate(path):
                    # for i_b in range(len(path)-1, i_a, -1):
                    for j, node_b in reversed(list(enumerate(path[i_a+1:]))):
                        i_b = j + i_a + 1
                        temp_node = PathNode(node_b.coordinates, parent=node_a)
                        valid_node = self.create_valid_path(temp_node, limit_path=False)
                        if valid_node and not valid_node.collision_avoided:
                            del path[i_a+1 : i_b]
                            break

            if return_type == 'points':
                path = [node.coordinates for node in path]

            return path
        else:
            print("No path was found last time")
            return None

    @staticmethod
    def plot_rectangle(x, y, dx, dy, color="b"):
        rect = patches.Rectangle((x-dx, y-dy), 2*dx, 2*dy, facecolor=color)
        # Add the patch to the Axes
        ax = plt.gca()
        ax.add_patch(rect)

    def draw_map(self, animation=False):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        for (ox, oy, odx, ody) in self.obstacle_list:
            self.plot_rectangle(ox, oy, odx, ody)

        for node in self.node_list:
            plt.scatter([node.coordinates[0]], [node.coordinates[1]], marker="o", c="m", s=4)
            if node.parent:
                plt.plot([x[0] for x in node.path], [x[1] for x in node.path], "-m", linewidth=1, alpha=0.3)

        if self.goal_node:
            # node = self.goal_node
            full_path = self.get_path(optimise=False, return_type='points')
            plt.plot([x[0] for x in full_path], [x[1] for x in full_path], "--g", linewidth=2, alpha=1)
            # while node is not None:
            #     plt.plot([x[0] for x in node.path], [x[1] for x in node.path], "-g", linewidth=2, alpha=1)
            #     node = node.parent
            short_path = self.get_path(optimise=True, return_type='points')
            plt.plot([x[0] for x in short_path], [x[1] for x in short_path], "-g", linewidth=3, alpha=1)

        plt.plot(self.start.coordinates[0], self.start.coordinates[1], "xr", linewidth=3)
        plt.plot(self.goal.coordinates[0], self.goal.coordinates[1], "xg", linewidth=3)
        plt.axis(self.map_limits)
        plt.grid(True)

        if animation:
            plt.pause(0.05)
        else:
            plt.show()


def main():
    # ====Search Path with RRT*====
    obstacle_list = [
        (5, 5, 0.5, 0.5),
        (3, 6, 1, 1),
        (3, 8, 1, 1),
        (9, 5, 1, 1),
        (6, 12, 0.5, 0.5)
    ]  # [x,y,dx,dy]

    # Set Initial parameters
    rrt_star = RrtStar(start=[0, 0],
                       goal=[6, 10],
                       map_limits=[-2, 15, -5, 20],
                       obstacle_list=obstacle_list,
                       path_max = 5,
                       max_iter=500)

    # cProfile.run('rrt_star.plan(animation=False)')
    rrt_star.plan(animation=False)

    rrt_star.draw_map()

    max_children = 0
    for node in rrt_star.node_list:
        n = len(node.children)
        if n > max_children:
            max_children = n
    print("Maximum chidlren = {}".format(max_children))


if __name__ == '__main__':
    main()
