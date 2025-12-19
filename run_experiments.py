#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    first_line = f.readline()
    my_map = []
    starts = []
    goals = []

    if 'type' in first_line or 'height' in first_line or 'width' in first_line or 'version' in first_line:
        f.close()
        f = open(filename, 'r')
        
        height = 0
        width = 0
        
        for line in f:
            if line.startswith('height'):
                height = int(line.split()[1])
            elif line.startswith('width'):
                width = int(line.split()[1])
            elif line.startswith('map'):
                break
        
        for line in f:
            row = [char != '.' for char in line.strip()]
            if len(row) > 0:
                my_map.append(row)
    else:
        rows, columns = [int(x) for x in first_line.split(' ')]
        for _ in range(rows):
            line = f.readline()
            row = [char == '@' for char in line.strip()]
            my_map.append(row)
            
    f.close()

    if not filename.endswith('.map'): 
        try:
            num_agents = int(f.readline())
            for _ in range(num_agents):
                line = f.readline()
                sx, sy, gx, gy = [int(x) for x in line.split(' ')]
                starts.append((sx, sy))
                goals.append((gx, gy))
        except:
            pass 
    return my_map, starts, goals

def load_scenario(scen_filename, num_agents_to_load=None):
    starts = []
    goals = []
    optimal_lengths = [] 
    with open(scen_filename, 'r') as f:
        f.readline() 
        for line in f:
            parts = line.split()
            if len(parts) < 9: continue
            
            sy, sx = int(parts[5]), int(parts[4])
            gy, gx = int(parts[7]), int(parts[6])
            optimal_len = float(parts[8]) 

            starts.append((sy, sx))
            goals.append((gy, gx))
            optimal_lengths.append(optimal_len) 
            
            if num_agents_to_load and len(starts) >= num_agents_to_load:
                break
    return starts, goals, optimal_lengths


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=True,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    if args.instance is None:
        raise RuntimeError("Error: --instance argument is required. Please specify an instance file or pattern.")

    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):
        print(f"*** Import an instance: {file} ***")
        
        if file.endswith('.scen'):
            map_file = file.replace('.scen', '')
            my_map, _, _ = import_mapf_instance(map_file)
            starts, goals, optimal_lengths = load_scenario(file, num_agents_to_load=2) 
        else:
            my_map, starts, goals = import_mapf_instance(file)

        print_mapf_instance(my_map, starts, goals)

        try:
            if args.solver == "CBS":
                print("***Run CBS***")
                cbs = CBSSolver(my_map, starts, goals)
                paths = cbs.find_solution(args.disjoint)
            elif args.solver == "Independent":
                print("***Run Independent***")
                solver = IndependentSolver(my_map, starts, goals)
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print("***Run Prioritized***")
                solver = PrioritizedPlanningSolver(my_map, starts, goals)
                paths = solver.find_solution()
            else:
                raise RuntimeError("Unknown solver!")

            cost = get_sum_of_cost(paths)
            result_file.write("{},{},{}\n".format(file, cost, args.solver))

            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                animation.show()

        except BaseException as e:
            print(f"FAILED on {file}: {e}")
            result_file.write("{},NA,{}\n".format(file, args.solver))

    result_file.close()