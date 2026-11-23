"""The one-time searches compared with D* Lite: Dijkstra, A*, Greedy Best-First and
Bidirectional Dijkstra.

"One-time" means each search starts from nothing every time it is asked, even if only one
road changed. D* Lite (in dstar_lite.py) is different: it keeps its earlier work and repairs it.

Every search here returns the same kind of answer, so the page can compare them:
    found      True if a route exists
    nodes      the junctions of the route, from the start to the destination
    segments   the road pieces of the route, which the page draws
    length_m   total distance in meters
    time_s     total travel time in seconds
    expanded   how many junctions the search had to check (see below)
    ms         how long the search took, in milliseconds
    destination  the name of the destination reached

The two performance numbers:
    expanded   = junctions checked. A junction is "checked" when the search takes it out of
                 its queue and looks at its neighbors. Fewer means less work. This number does
                 not depend on how fast the computer is, so it is the fairest comparison.
    ms         = milliseconds. This does depend on the computer and on what else it is doing,
                 so it changes a little on every run. The study repeats each test many times.
"""

# heapq is Python's priority queue. It always gives back the smallest item first.
import heapq
import time

from analysis import describe_path, destination_names, search_graph
from network import distance_m

# Infinity means "no known route". Any real time is smaller than this.
INFINITY = float("inf")


def straight_line_guess(network, mode, start):
    """Makes the "guess" used by A* and Greedy Best-First.

    The guess for a junction is the time it would take to reach the start in a straight
    line at top speed. Real roads are never shorter than a straight line, so this guess
    never says a junction is farther than it really is.
    """
    speed = network.speed_mps[mode]
    goal = network.nodes[start]

    def guess(node):
        place = network.nodes[node]
        # Multiplying by 0.99 keeps the guess safely low despite the rounding of saved lengths.
        return 0.99 * distance_m(place["lat"], place["lon"], goal["lat"], goal["lon"]) / speed

    return guess


def finish(network, mode, nodes, started, expanded):
    """Builds the shared answer format from a finished search."""
    milliseconds = (time.perf_counter() - started) * 1000
    if nodes is None:
        return {"found": False, "ms": milliseconds, "expanded": expanded}
    result = describe_path(network, mode, nodes)
    result.update(found=True, nodes=nodes, ms=milliseconds, expanded=expanded,
                  destination=destination_names(network, mode, nodes[-1]))
    return result


def best_first_search(network, mode, start, target, order):
    """The shared engine behind Dijkstra, A* and Greedy Best-First.

    All three work the same way: keep a queue of junctions, always check the one with the
    smallest priority next, and stop when the start is reached. They differ ONLY in how the
    priority is worked out, which is the "order" passed in:
        Dijkstra:          time so far                (no guessing at all)
        A*:                time so far + guess        (guess points toward the start)
        Greedy Best-First: guess only                 (ignores the time already traveled)

    The search runs backward, from the destinations to the start, like every search in this
    system. That way one search handles "go to the nearest of several destinations".
    """
    goals = network.goals(mode, target)
    graph = search_graph(network, mode)
    started = time.perf_counter()
    # best[node] = the best time found so far from that junction to a destination.
    best = {goal: 0.0 for goal in goals}
    # next_step[node] = the junction after it on the way to the destination.
    next_step = {}
    queue = [(order(0.0, goal), goal) for goal in goals]
    heapq.heapify(queue)
    done = set()
    expanded = 0
    while queue:
        _, node = heapq.heappop(queue)
        # The same junction can be queued more than once. Only its first visit counts.
        if node in done:
            continue
        done.add(node)
        expanded += 1
        # Reaching the start ends the search.
        if node == start:
            break
        # The graph is reversed, so these neighbors are the junctions that lead INTO node.
        for previous, roads in graph[node].items():
            # A junction already checked is final. Dijkstra and A* never find a better time
            # for it anyway. Greedy Best-First might, but by design it does not go back.
            if previous in done:
                continue
            # Two roads can join the same junctions; use the faster one.
            time_s = best[node] + min(data["time_s"] for data in roads.values())
            if time_s < best.get(previous, INFINITY):
                best[previous] = time_s
                next_step[previous] = node
                heapq.heappush(queue, (order(time_s, previous), previous))

    if start not in done:
        return finish(network, mode, None, started, expanded)
    # Follow next_step from the start until reaching a destination.
    nodes = [start]
    while nodes[-1] not in goals:
        nodes.append(next_step[nodes[-1]])
    return finish(network, mode, nodes, started, expanded)


def dijkstra_search(network, mode, start, target="any"):
    """Dijkstra: always checks the junction with the smallest travel time so far.

    It spreads out evenly in every direction, like a ripple, so it always finds the
    fastest route but checks many junctions that lead away from the start.
    """
    return best_first_search(network, mode, start, target, lambda time_s, node: time_s)


def astar_search(network, mode, start, target="any"):
    """A*: Dijkstra plus a guess of the time still left.

    Junctions in the direction of the start get checked first, so it usually checks far
    fewer junctions than Dijkstra. Because the guess is never too high, it still always
    finds the fastest route.
    """
    guess = straight_line_guess(network, mode, start)
    return best_first_search(network, mode, start, target, lambda time_s, node: time_s + guess(node))


def greedy_search(network, mode, start, target="any"):
    """Greedy Best-First: uses ONLY the guess, and ignores the time already traveled.

    It rushes toward the start, so it usually checks the fewest junctions of all. The catch:
    it can be fooled by a road that heads the right way but turns out long, so its route is
    NOT always the fastest. That is why it is useful in the comparison: it shows the price of
    speed without a guarantee.
    """
    guess = straight_line_guess(network, mode, start)
    return best_first_search(network, mode, start, target, lambda time_s, node: guess(node))


def bidirectional_search(network, mode, start, target="any"):
    """Bidirectional Dijkstra: two Dijkstra searches at once, meeting in the middle.

    One search grows backward from the destinations, the other forward from the start.
    Two small circles that meet cover much less ground than one big circle, so it checks
    fewer junctions than plain Dijkstra, and it still always finds the fastest route.
    """
    goals = network.goals(mode, target)
    graph = search_graph(network, mode)
    started = time.perf_counter()
    expanded = 0
    # Side 0 searches backward from the destinations. Side 1 searches forward from the start.
    # Each side keeps its own best times, its own queue and its own checked junctions.
    best = [{goal: 0.0 for goal in goals}, {start: 0.0}]
    link = [{}, {}]  # link[side][node] = the neighbor this side reached node from
    queues = [[(0.0, goal) for goal in goals], [(0.0, start)]]
    heapq.heapify(queues[0])
    done = [set(), set()]
    # The best complete route found so far, and the junction where its two halves meet.
    shortest, meeting = INFINITY, None

    while queues[0] and queues[1]:
        # Stop rule: once the two smallest queued times add up to at least the best route
        # found, no undiscovered route can be faster, so the search is finished.
        if queues[0][0][0] + queues[1][0][0] >= shortest:
            break
        # Grow the side with the smaller queue, which keeps both circles about the same size.
        side = 0 if len(queues[0]) <= len(queues[1]) else 1
        time_s, node = heapq.heappop(queues[side])
        if node in done[side]:
            continue
        done[side].add(node)
        expanded += 1
        # In the reversed graph, graph[node] lists junctions that lead INTO node (the backward
        # side's direction), and graph.pred[node] lists junctions node leads TO (forward).
        neighbors = graph[node] if side == 0 else graph.pred[node]
        for other, roads in neighbors.items():
            new_time = time_s + min(data["time_s"] for data in roads.values())
            if new_time < best[side].get(other, INFINITY):
                best[side][other] = new_time
                link[side][other] = node
                heapq.heappush(queues[side], (new_time, other))
            # If the other side has reached this junction too, a full route exists through it.
            if other in best[1 - side] and best[side][other] + best[1 - side][other] < shortest:
                shortest = best[side][other] + best[1 - side][other]
                meeting = other
        # The start may itself be a destination, which is a route of zero length.
        if node in best[1 - side] and best[0][node] + best[1][node] < shortest:
            shortest, meeting = best[0][node] + best[1][node], node

    if meeting is None:
        return finish(network, mode, None, started, expanded)
    # Join the two halves: start -> meeting junction (forward side, read backward),
    # then meeting junction -> destination (backward side).
    first_half = [meeting]
    while first_half[-1] != start:
        first_half.append(link[1][first_half[-1]])
    nodes = list(reversed(first_half))
    while nodes[-1] not in goals:
        nodes.append(link[0][nodes[-1]])
    return finish(network, mode, nodes, started, expanded)


# The searches the page compares, by the name the page uses for each.
SEARCHES = {
    "dijkstra": dijkstra_search,
    "astar": astar_search,
    "greedy": greedy_search,
    "bidirectional": bidirectional_search,
}
