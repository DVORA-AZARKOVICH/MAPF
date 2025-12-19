import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, is_constrained


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    for t in range(max(len(path1), len(path2))):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': t}
        if t > 0:
            prev_loc1 = get_location(path1, t - 1)
            prev_loc2 = get_location(path2, t - 1)
            if loc1 == prev_loc2 and loc2 == prev_loc1:
                return {'loc': [prev_loc1, loc1], 'timestep': t}
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    if collision is not None:
        if len(collision['loc']) == 2: 
            constraints.append({
                'agent': collision['a1'],
                'loc': [collision['loc'][0], collision['loc'][1]],
                'timestep': collision['timestep']
            })
            constraints.append({
                'agent': collision['a2'],
                'loc': [collision['loc'][1], collision['loc'][0]],
                'timestep': collision['timestep']
            })
        else:
            constraints.append({
                'agent': collision['a1'],
                'loc': collision['loc'],
                'timestep': collision['timestep']
                })
            constraints.append({
                'agent': collision['a2'],
                'loc': collision['loc'],
                'timestep': collision['timestep']
                })
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    constraints = []
    if collision is not None:
        agent = random.choice([collision['a1'], collision['a2']])
        if len(collision['loc']) == 2: 
            if agent == collision['a1']:
                first_loc = collision['loc'][0]
                second_loc = collision['loc'][1]
            else:
                first_loc = collision['loc'][1]
                second_loc = collision['loc'][0]
            constraints.append({
                'agent': agent,
                'loc': [first_loc, second_loc],
                'timestep': collision['timestep'],
                'positive': True
            })
            constraints.append({
                'agent': agent,
                'loc': [first_loc, second_loc],
                'timestep': collision['timestep'],
                'positive': False
            })
        else:
            constraints.append({
                'agent': agent,
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': True
                })
            constraints.append({
                'agent': agent,
                'loc': collision['loc'],
                'timestep': collision['timestep'],
                'positive': False
                })
    return constraints


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            node = self.pop_node()
            if len(node['collisions']) == 0:
                self.print_results(node)
                return node['paths']
            collision = node['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)
            if constraints is None:
                print(f"ERROR: constraints is None! disjoint={disjoint}")
                print(f"Collision data: {collision}")
                constraints = [] # Prevent crash - should not happen
            for constraint in constraints:
                child = {'cost': 0,
                         'constraints': node['constraints'] + [constraint],
                         'paths': node['paths'].copy(),
                         'collisions': []}
                
                violating_agents = self.paths_violate_constraint(constraint, node['paths'])
                
                all_paths_found = True
                for agent in violating_agents:
                    path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                                self.heuristics[agent], agent, child['constraints'])
                    if path is None:
                        all_paths_found = False
                        break
                    child['paths'][agent] = path
                if all_paths_found:
                    child['collisions'] = detect_collisions(child['paths'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)
        raise BaseException('No solutions')
    
    def paths_violate_constraint(self, constraint, paths):
        if constraint.get('positive', False):
            agents_to_update = []
            for agent in range(len(paths)):
                curr_loc = get_location(paths[agent], constraint['timestep'])
                prev_loc = get_location(paths[agent], constraint['timestep'] - 1)
                
                if agent == constraint['agent']:
                    if len(constraint['loc']) == 1 and curr_loc != constraint['loc'][0]:
                        agents_to_update.append(agent)
                    elif len(constraint['loc']) == 2 and [prev_loc, curr_loc] != constraint['loc']:
                        agents_to_update.append(agent)
                        
                else:
                    if len(constraint['loc']) == 1 and curr_loc == constraint['loc'][0]:
                        agents_to_update.append(agent)
                    elif len(constraint['loc']) == 2:
                        if [prev_loc, curr_loc] == constraint['loc'] or [curr_loc, prev_loc] == constraint['loc']:
                            agents_to_update.append(agent)

            return agents_to_update

        else:
            return [constraint['agent']]
    
    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
