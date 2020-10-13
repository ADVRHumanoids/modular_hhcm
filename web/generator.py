import itertools


def generate_robot_set(n):
    elements = [1, 1, 1, 0, 0, 0]
    modules_combinations = set(itertools.permutations(elements, n))
    robots = []

    # Split the modules combination in two so to create two robot chains
    # The tuple is sorted so to avoid identical robots where 1st and 2nd chain are just swapped
    # The set() of the list of tuples removes the copies
    for index in range(len(elements)):
        for mc in modules_combinations:
            first_chain = mc[:index]
            second_chain = mc[index:]
            robot = (first_chain, second_chain)
            robots.append(tuple(sorted(robot, reverse=True)))

    print "Number of combinations:" , len(robots)
    unique_robots_set = set(robots)
    print "Number of unique robots:" , len(unique_robots_set)

    return unique_robots_set