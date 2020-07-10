# coding=utf-8
import heapq
import math
import numpy as np


class PriorityQueue:
    """
    A queue structure where each element is served in order of priority.

    Elements in the queue are popped based on the priority with higher priority
    elements being served before lower priority elements.  If two elements have
    the same priority, they will be served in the order they were added to the
    queue.

    Traditionally priority queues are implemented with heaps, but there are any
    number of implementation options.

    (Hint: take a look at the module heapq)

    Attributes:
        queue (list): Nodes added to the priority queue.
    """

    def __init__(self):
        """Initialize a new Priority Queue."""

        self.queue = []
        self.count = 0

    def pop(self):
        """
        Pop top priority node from queue.

        Returns:
            The node with the highest priority.
        """
        node_w_count = heapq.heappop(self.queue)
        return (node_w_count[0], node_w_count[2])

    def remove(self, node_id):
        """
        Remove a node from the queue.

        This is a hint, you might require this in ucs,
        however, if you choose not to use it, you are free to
        define your own method and not use it.

        Args:
            node_id: Index of node in queue.
        """
        del self.queue[node_id]

    def __iter__(self):
        """Queue iterator."""

        return iter(sorted(self.queue))

    def __str__(self):
        """Priority Queue to string."""

        return 'PQ:%s' % self.queue

    def append(self, node):
        """
        Append a node to the queue.

        Args:
            node: Comparable Object to be added to the priority queue.
        """

        priority = node[0]
        loc = node[1]
        heapq.heappush(self.queue, (priority, self.count, loc))
        self.count += 1

    def __contains__(self, key):
        """
        Containment Check operator for 'in'

        Args:
            key: The key to check for in the queue.

        Returns:
            True if key is found in queue, False otherwise.
        """

        return key in [n[-1] for n in self.queue]

    def __eq__(self, other):
        """
        Compare this Priority Queue with another Priority Queue.

        Args:
            other (PriorityQueue): Priority Queue to compare against.

        Returns:
            True if the two priority queues are equivalent.
        """

        return self.queue == other.queue

    def size(self):
        """
        Get the current size of the queue.

        Returns:
            Integer of number of items in queue.
        """

        return len(self.queue)

    def clear(self):
        """Reset queue to empty (no nodes)."""

        self.queue = []

    def top(self):
        """
        Get the top item in the queue.

        Returns:
            The first item stored in teh queue.
        """

        return self.queue[0]

    def at(self, i):
        cost, count, node = self.queue[i]
        return (cost, node)


def breadth_first_search(graph, start, goal):
    """
    Warm-up exercise: Implement breadth-first-search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    if start == goal:
        return []

    pq = PriorityQueue()

    previous = {}
    explored = []

    pq.append((0, start))

    while True:
        if pq.size() == 0:
            return False

        node = pq.pop()
        location = node[1]
        print(f"popping off {location}")
        explored.append(location)
        neighbors = graph[location]
        print(f"neighbors of {node}: {sorted(neighbors)}")
        for n in sorted(neighbors):
            if n not in explored and n not in pq:
                if n == goal:
                    previous[n] = location
                    return generate_path(previous, start, goal)
                print(f"The prev to {location} is {n}")
                previous[n] = location
                print(f"appending {n} to the pq")
                pq.append((0, n))


def generate_path(previous, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def generate_joined_path(start_previous, goal_previous, start, goal, pin):
    f_path = generate_path(start_previous, start, pin)
    r_path = generate_path(goal_previous, goal, pin)

    r_path = r_path[:-1]
    r_path.reverse()
    return f_path + r_path


def uniform_cost_search(graph, start, goal):
    if start == goal:
        return []

    pq = PriorityQueue()

    previous = {}
    explored = []
    total_cost = float("-inf")

    pq.append((0, start))

    while True:
        if pq.size() == 0:
            return False

        cost, node = pq.pop()
        if node == goal:
            return generate_path(previous, start, goal)

        print(f"popping off {node}")
        explored.append(node)

        neighbors = graph[node]
        print(f"neighbors of {node}: {sorted(neighbors)}")

        for n in sorted(neighbors):
            if n not in explored:
                total_cost = graph.get_edge_weight(n, node) + cost
                if n not in pq:
                    previous[n] = node
                    print(f"appending {n} to the pq")
                    pq.append((total_cost, n))
                elif n in pq:
                    for i in range(pq.size()):
                        test_cost, test_node = pq.at(i)
                        if n == test_node and test_cost > total_cost:
                            pq.remove(i)
                            previous[n] = node
                            pq.append((total_cost, n))



def euclidean_dist_heuristic(graph, v, goal):
    """
    Warm-up exercise: Implement the euclidean distance heuristic.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        v (str): Key for the node to calculate from.
        goal (str): Key for the end node to calculate to.

    Returns:
        Euclidean distance between `v` node and `goal` node
    """

    # Get x and y location of node
    node_x, node_y = graph.nodes[v]['pos']

    # Get x and y location of goal
    goal_x, goal_y = graph.nodes[goal]['pos']

    return np.sqrt((goal_x - node_x)**2 + (goal_y - node_y)**2)


def a_star(graph, start, goal, heuristic=euclidean_dist_heuristic):
    """
    Warm-up exercise: Implement A* algorithm.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # Check if the start and goal are the same
    if start == goal:
        return []

    # Create the queue
    pq = PriorityQueue()

    previous = {}
    explored = []

    # Initialize to a very low number to ensure costs are greater
    total_cost = float("-inf")

    # Push the first node into the queue
    pq.append((heuristic(graph, start, goal), start))

    while True:

        # If the queue is empty we never reached the goal
        if pq.size() == 0:
            return False

        cost, node = pq.pop()

        if node == goal:
            return generate_path(previous, start, goal)

        cost = cost - heuristic(graph, node, goal)
        explored.append(node)
        neighbors = graph[node]

        for n in sorted(neighbors):
            if n not in explored:
                total_cost = graph.get_edge_weight(n, node) + cost + heuristic(graph, n, goal)

                if n not in pq:
                    previous[n] = node
                    pq.append((total_cost, n))
                elif n in pq:
                    for i in range(pq.size()):
                        ith_cost, ith_node = pq.at(i)

                        if n == ith_node and ith_cost > total_cost:
                            pq.remove(i)
                            previous[n] = node
                            pq.append((total_cost, n))

    return generate_path(previous, start, goal)


def bidirectional_ucs(graph, start, goal):
    """
    Exercise 1: Bidirectional Search.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """
    # Don't actually need to route anywhere, just return empty list
    if start == goal:
        return []

    # Create dictionaries to house all DSs
    fw = {'previous': {}, 'explored': [], 'queue': PriorityQueue(), 'cost': {}}
    rev = {'previous': {}, 'explored': [],
           'queue': PriorityQueue(), 'cost': {}}

    controller = {True: fw, False: rev}

    # Value for assessing the cheapest path
    mu = float("inf")

    # Adding the start and goal nodes to their respective queues
    controller[True]['queue'].append((0, start))
    controller[False]['queue'].append((0, goal))

    # Store the cost of the node from start -> node
    controller[True]['cost'][start] = 0
    controller[False]['cost'][goal] = 0

    # Work in the forward direction first
    forward = True

    # Continue to loop until we terminate the search
    while True:

        # Verify that each queue has nodes in it
        if controller[forward]['queue'].size() == 0 or controller[not forward]['queue'].size() == 0:
            return False

        # Get the combined costs of the top nodes from each direction
        top_candidate = controller[forward]['queue'].top(
        )[0] + controller[not forward]['queue'].top()[0]

        if top_candidate >= mu:
            # return generate_path(previous, start, goal)
            break

        # Get the cheapest in the forward search
        cost, node = controller[forward]['queue'].pop()

        # Add the node to the explored list
        controller[forward]['explored'].append(node)

        # Get all adjacent nodes
        neighbors = graph[node]
        for n in sorted(neighbors):
            if n not in controller[forward]['explored']:
                # Get the cost of start -> node -> neighbor
                total_cost = graph.get_edge_weight(node, n) + cost

                if n in controller[not forward]['queue'] or n in controller[not forward]['explored']:
                    candidate = controller[not forward]['cost'][n] + total_cost
                    if candidate < mu:
                        mu = candidate
                        pin = n

                if n not in controller[forward]['queue']:
                    # Assign node as n's parent
                    controller[forward]['previous'][n] = node

                    # Add n to the queue
                    controller[forward]['queue'].append((total_cost, n))

                    # Save the cost for n
                    controller[forward]['cost'][n] = total_cost
                else:
                    for i in range(controller[forward]['queue'].size()):
                        test_cost, test_node = controller[forward]['queue'].at(
                            i)
                        if n == test_node and test_cost > total_cost:
                            controller[forward]['queue'].remove(i)
                            controller[forward]['previous'][n] = node
                            controller[forward]['queue'].append(
                                (total_cost, n))
                            controller[forward]['cost'][n] = total_cost

        # Send us in the opposite direction
        forward = not forward
    f_path = generate_path(controller[True]['previous'], start, pin)
    r_path = generate_path(controller[False]['previous'], goal, pin)

    r_path = r_path[:-1]
    r_path.reverse()
    return f_path + r_path


def bidirectional_a_star(graph, start, goal,
                         heuristic=euclidean_dist_heuristic):
    """
    Exercise 2: Bidirectional A*.

    See README.md for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        start (str): Key for the start node.
        goal (str): Key for the end node.
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from the start and goal nodes (including both).
    """

    # Don't actually need to route anywhere, just return empty list
    if start == goal:
        return []

    # Create dictionaries to house all DSs
    fw = {'previous': {}, 'explored': [], 'queue': PriorityQueue(), 'cost': {}}
    rev = {'previous': {}, 'explored': [],
           'queue': PriorityQueue(), 'cost': {}}

    controller = {True: fw, False: rev}

    # Value for assessing the cheapest path
    mu = float("inf")

    # Adding the start and goal nodes to their respective queues
    controller[True]['queue'].append((heuristic(graph, start, goal), start))
    controller[False]['queue'].append((heuristic(graph, goal, start), goal))

    # Store the cost of the node from start -> node
    controller[True]['cost'][start] = 0
    controller[False]['cost'][goal] = 0

    # Work in the forward direction first
    forward = True

    # Continue to loop until we terminate the search
    while True:

        # Verify that each queue has nodes in it
        if controller[forward]['queue'].size() == 0 or controller[not forward]['queue'].size() == 0:
            return False

        # top_candidate = controller[forward]['queue'].top()[0] + controller[not forward]['queue'].top()[0]

        t1_node = controller[True]['queue'].top()[-1]
        t1_cost = controller[True]['queue'].top()[0]

        t2_node = controller[False]['queue'].top()[-1]
        t2_cost = controller[False]['queue'].top()[0]

        t1_cost -= heuristic(graph, t1_node, goal)
        t2_cost -= heuristic(graph, t2_node, start)

        top_candidate = t1_cost + t2_cost

        if top_candidate >= mu:
            # return generate_path(previous, start, goal)
            break

        # Get the cheapest in the forward search
        cost, node = controller[forward]['queue'].pop()

        if forward:
            cost -= heuristic(graph, node, goal)
        else:
            cost -= heuristic(graph, node, start)

        # Add the node to the explored list
        controller[forward]['explored'].append(node)

        # Get all adjacent nodes
        neighbors = graph[node]
        for n in sorted(neighbors):
            if n not in controller[forward]['explored']:
                # Get the cost of start -> node -> neighbor
                base_cost = graph.get_edge_weight(node, n) + cost

                if forward:
                    # Add distance from n --> goal to the total cost
                    total_cost = base_cost + heuristic(graph, n, goal)
                else:
                    # Add distance from n --> start to the total cost
                    total_cost = base_cost + heuristic(graph, n, start)

                # Check to see if the neighbor is in the frontier or explored of the opp direction
                if n in controller[not forward]['queue'] or n in controller[not forward]['explored']:
                    # Get the cost of matched neighbor + heuristic
                    candidate = controller[not forward]['cost'][n] + base_cost
                    # If the candidate is less than our current path mark, pin the node
                    if candidate < mu:
                        mu = candidate
                        pin = n

                if n not in controller[forward]['queue']:
                    # Assign node as n's parent
                    controller[forward]['previous'][n] = node

                    # Add n to the queue
                    controller[forward]['queue'].append((total_cost, n))

                    # Save the cost for n
                    controller[forward]['cost'][n] = base_cost
                else:
                    for i in range(controller[forward]['queue'].size()):
                        test_cost, test_node = controller[forward]['queue'].at(
                            i)
                        if n == test_node and test_cost > total_cost:
                            controller[forward]['queue'].remove(i)
                            controller[forward]['previous'][n] = node
                            controller[forward]['queue'].append(
                                (total_cost, n))
                            controller[forward]['cost'][n] = base_cost

        # Send us in the opposite direction
        forward = not forward
    f_path = generate_path(controller[True]['previous'], start, pin)
    r_path = generate_path(controller[False]['previous'], goal, pin)

    r_path = r_path[:-1]
    r_path.reverse()
    return f_path + r_path


def tridirectional_search(graph, goals):
    """
    Exercise 3: Tridirectional UCS Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals

    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """
    # Goal is filled with the same node, just return empty
    if len(set(goals)) == 1:
        return []

    if len(set(goals)) == 2:
        # goals = list(set(goals))
        print("Something")

    # Create dictionaries to house all DSs
    controller = {i: {'previous': {}, 'explored': [],
                      'queue': PriorityQueue(), 'cost': {}} for i in range(len(goals))}

    # Create dictionary to house and mu values
    mu = {i: float("inf") for i in range(len(goals))}

    # Create dictionary to house all pins
    pin = {i: None for i in range(len(goals))}

    for i in range(len(goals)):
        # Assign the start points of each queue
        controller[i]['queue'].append((0, goals[i]))
        # Store the cost of the node from start -> node
        controller[i]['cost'][goals[i]] = 0

    # Work in the zero direction first
    direction = 0

    # Continue to loop until we terminate the search
    num_goals = len(goals)
    while True:

        # Verify that each queue has nodes in it
        if any(controller[i]['queue'].size() == 0 for i in range(num_goals)):
            return False

        # Get the combined costs of the top nodes from all three searches
        tops = [controller[i % num_goals]['queue'].top(
        )[0] + controller[(i + 1) % num_goals]['queue'].top()[0] for i in range(num_goals)]
        print(tops)

        # mu_zero_one
        if tops[0] >= mu[0] and tops[1] >= mu[1]:
            print("Connections to one are done!")
            direction = 2
        if tops[1] >= mu[1] and tops[2] >= mu[2]:
            direction = 0
            print("Connections to two are done!")
        if tops[0] >= mu[0] and tops[2] >= mu[2]:
            direction = 1
            print("Connections to zero are done!")

        if all(tops[i] >= mu[i] for i in range(len(goals))):
            break

        cost, node = controller[direction]['queue'].pop()
        controller[direction]['explored'].append(node)

        neighbors = graph[node]

        for n in sorted(neighbors):
            if n not in controller[direction]['explored']:
                total_cost = graph.get_edge_weight(node, n) + cost

                # Check if direction d + 1 has seen the neighbor yet
                if n in controller[(direction + 1) % 3]['queue'] or n in controller[(direction + 1) % 3]['explored']:
                    candidate = controller[(direction + 1) %
                                           3]['cost'][n] + total_cost
                    if candidate < mu[direction]:
                        mu[direction] = candidate
                        pin[direction] = n

                # Check if direction d + 2 has seen the neighbor yet
                if n in controller[(direction + 2) % 3]['queue'] or n in controller[(direction + 2) % 3]['explored']:
                    candidate = controller[(direction + 2) %
                                           3]['cost'][n] + total_cost
                    if candidate < mu[(direction + 2) % 3]:
                        mu[(direction + 2) % 3] = candidate
                        pin[(direction + 2) % 3] = n

                if n not in controller[direction]['queue']:
                    controller[direction]['previous'][n] = node
                    controller[direction]['queue'].append((total_cost, n))
                    controller[direction]['cost'][n] = total_cost
                else:
                    for i in range(controller[direction]['queue'].size()):
                        test_cost, test_node = controller[direction]['queue'].at(i)
                        if n == test_node and test_cost > total_cost:
                            controller[direction]['queue'].remove(i)
                            controller[direction]['previous'][n] = node
                            controller[direction]['queue'].append(
                                (total_cost, n))
                            controller[direction]['cost'][n] = total_cost

        # Send us in the opposite direction
        direction = (direction + 1) % 3

    zero_to_one_path = generate_path(controller[0]['previous'], goals[0], pin[0])
    one_to_zero_path = generate_path(controller[1]['previous'], goals[1], pin[0])

    one_to_zero_path = one_to_zero_path[:-1]
    one_to_zero_path.reverse()
    zero_to_one = zero_to_one_path + one_to_zero_path

    zero_to_two_path = generate_path(controller[0]['previous'], goals[0], pin[2])
    two_to_zero_path = generate_path(controller[2]['previous'], goals[2], pin[2])

    two_to_zero_path = two_to_zero_path[:-1]
    two_to_zero_path.reverse()
    zero_to_two = zero_to_two_path + two_to_zero_path

    one_to_two_path = generate_path(controller[1]['previous'], goals[1], pin[1])
    two_to_one_path = generate_path(controller[2]['previous'], goals[2], pin[1])

    two_to_one_path = two_to_one_path[:-1]
    two_to_one_path.reverse()
    one_to_two = one_to_two_path + two_to_one_path

    all_paths = [zero_to_one, one_to_two, zero_to_two]

    s = min(mu, key=mu.get)
    del mu[s]
    ns = min(mu, key=mu.get)

    shortest = all_paths[s]
    next_shortest = all_paths[ns]


    if shortest[0] == next_shortest[0]:
        shortest.reverse()
        shortest = shortest[:-1]
    elif shortest[-1] == next_shortest[-1]:
        next_shortest = next_shortest[:-1]
        next_shortest.reverse()
    elif shortest[0] == next_shortest[-1]:
        next_shortest.reverse()
        shortest.reverse()
        shortest = shortest[:-1]
    elif shortest[-1] == next_shortest[0]:
        shortest = shortest[:-1]
    return shortest + next_shortest


def tridirectional_upgraded(graph, goals, heuristic=euclidean_dist_heuristic):
    """
    Exercise 4: Upgraded Tridirectional Search

    See README.MD for exercise description.

    Args:
        graph (ExplorableGraph): Undirected graph to search.
        goals (list): Key values for the 3 goals
        heuristic: Function to determine distance heuristic.
            Default: euclidean_dist_heuristic.

    Returns:
        The best path as a list from one of the goal nodes (including both of
        the other goal nodes).
    """

    print(f"Goals: {goals}")
    # Goal is filled with the same node, just return empty
    if len(set(goals)) == 1:
        return []


    num_goals = len(goals)

    # Create dictionaries to house all DSs
    controller = {i: {'previous': {}, 'explored': [], 'queue': 
                      PriorityQueue(), 'cost': {}, 'heur': {}} for i in range(num_goals)}

    mu = {i: float("inf") for i in range(num_goals)}

    pin = {i: None for i in range(num_goals)}

    for i in range(num_goals):
        # Assign the start points of each queue
        a = heuristic(graph, goals[i], goals[(i + 1) % num_goals])
        b = heuristic(graph, goals[i % num_goals], goals[(i + 2) % num_goals])

        controller[i]['queue'].append((min(a, b), goals[i]))
        controller[i]['cost'][goals[i]] = 0

    # Work in the zero direction first
    direction = 0

    while True:

        # Verify that each queue has nodes in it
        if any(controller[i]['queue'].size() == 0 for i in range(num_goals)):
            return False

        costs = [controller[i]['queue'].top()[0] for i in range(num_goals)]
        nodes = [controller[i]['queue'].top()[-1] for i in range(num_goals)]

        c = 0
        for node in nodes:
            goal1 = heuristic(graph, node, goals[(c + 1) % num_goals])
            goal2 = heuristic(graph, node, goals[(c + 2) % num_goals])

            minimum = min(goal1, goal2)
            costs[c] -= minimum
            c += 1

        tops = [costs[i] + costs[(i+1) % num_goals] for i in range(num_goals)]

        if tops[0] >= mu[0] and tops[1] >= mu[1]:
            print("Connections to one are done!")
            direction = 2
        if tops[1] >= mu[1] and tops[2] >= mu[2]:
            direction = 0
            print("Connections to two are done!")
        if tops[0] >= mu[0] and tops[2] >= mu[2]:
            direction = 1
            print("Connections to zero are done!")

        if all(tops[i] >= mu[i] for i in range(len(goals))):
            break

        # Get the cheapest in the current direction search
        cost, node = controller[direction]['queue'].pop()

        a = heuristic(graph, goals[direction],
                      goals[(direction + 1) % num_goals])
        b = heuristic(graph, goals[direction],
                      goals[(direction + 2) % num_goals])

        cost -= min(a, b)

        controller[direction]['explored'].append(node)
        neighbors = graph[node]

        for n in sorted(neighbors):
            if n not in controller[direction]['explored']:
                base_cost = graph.get_edge_weight(node, n) + cost

                a = heuristic(graph, n, goals[(direction + 1) % num_goals])
                b = heuristic(graph, n, goals[(direction + 2) % num_goals])

                total_cost = base_cost + min(a, b)

                # Check if direction d + 1 has seen the neighbor yet
                if n in controller[(direction + 1) % num_goals]['queue'] or n in controller[(direction + 1) % num_goals]['explored']:
                    candidate = controller[(direction + 1) %
                                           num_goals]['cost'][n] + base_cost
                    if candidate < mu[direction]:
                        mu[direction] = candidate
                        pin[direction] = n

                # Check if direction d + 2 has seen the neighbor yet
                if n in controller[(direction + 2) % num_goals]['queue'] or n in controller[(direction + 2) % num_goals]['explored']:
                    candidate = controller[(direction + 2) %
                                           num_goals]['cost'][n] + base_cost
                    if candidate < mu[(direction + 2) % num_goals]:
                        mu[(direction + 2) % 3] = candidate
                        pin[(direction + 2) % 3] = n

                # Make sure the node isn't already in the current direction's queue
                if n not in controller[direction]['queue']:
                    controller[direction]['previous'][n] = node
                    controller[direction]['queue'].append((total_cost, n))
                    controller[direction]['cost'][n] = base_cost
                else:
                    for i in range(controller[direction]['queue'].size()):
                        test_cost, test_node = controller[direction]['queue'].at(
                            i)
                        if n == test_node and test_cost > total_cost:
                            controller[direction]['queue'].remove(i)
                            controller[direction]['previous'][n] = node
                            controller[direction]['queue'].append(
                                (total_cost, n))
                            controller[direction]['cost'][n] = base_cost

        # Send us in the opposite direction
        direction = (direction + 1) % 3

    zero_to_one_path = generate_path(controller[0]['previous'], goals[0], pin[0])
    one_to_zero_path = generate_path(controller[1]['previous'], goals[1], pin[0])

    one_to_zero_path = one_to_zero_path[:-1]
    one_to_zero_path.reverse()
    zero_to_one = zero_to_one_path + one_to_zero_path

    zero_to_two_path = generate_path(controller[0]['previous'], goals[0], pin[2])
    two_to_zero_path = generate_path(controller[2]['previous'], goals[2], pin[2])

    two_to_zero_path = two_to_zero_path[:-1]
    two_to_zero_path.reverse()
    zero_to_two = zero_to_two_path + two_to_zero_path

    one_to_two_path = generate_path(controller[1]['previous'], goals[1], pin[1])
    two_to_one_path = generate_path(controller[2]['previous'], goals[2], pin[1])

    two_to_one_path = two_to_one_path[:-1]
    two_to_one_path.reverse()
    one_to_two = one_to_two_path + two_to_one_path

    all_paths = [zero_to_one, one_to_two, zero_to_two]

    s = min(mu, key=mu.get)
    del mu[s]
    ns = min(mu, key=mu.get)

    shortest = all_paths[s]
    next_shortest = all_paths[ns]


    if shortest[0] == next_shortest[0]:
        shortest.reverse()
        shortest = shortest[:-1]
    elif shortest[-1] == next_shortest[-1]:
        next_shortest = next_shortest[:-1]
        next_shortest.reverse()
    elif shortest[0] == next_shortest[-1]:
        next_shortest.reverse()
        shortest.reverse()
        shortest = shortest[:-1]
    elif shortest[-1] == next_shortest[0]:
        shortest = shortest[:-1]

    return shortest + next_shortest

