"""A* search, used to compare its chosen route with Dijkstra and D* Lite."""

# heapq is Python's priority queue. It always gives back the smallest item first.
import heapq
import time

from analysis import describe_path, destination_names, search_graph
from network import distance_m

# Infinity means "no known route". Any real time is smaller than this.
INFINITY = float("inf")


def astar_route(network, mode, start, target="any"):
    """Finds the fastest route from a junction to the chosen destination(s) with A*.

    A* works like Dijkstra, but it adds a guess of the time still left, so it looks
    at junctions in the direction of the goal first and can skip the rest.
    Like the other searches here, it runs backward: from the destinations to the start.
    That lets one search handle "nearest of several destinations".
    """
    goals = network.goals(mode, target)
    graph = search_graph(network, mode)
    # The fastest possible speed, so the guess never overestimates the time.
    speed = network.speed_mps[mode]
    goal_place = network.nodes[start]

    def guess(node):
        """A lower-bound guess of the time from this junction to the start."""
        place = network.nodes[node]
        # Straight-line distance / top speed can never exceed the true travel time.
        # Multiplying by 0.99 keeps that true despite the rounding of saved lengths.
        return 0.99 * distance_m(place["lat"], place["lon"], goal_place["lat"], goal_place["lon"]) / speed

    started = time.perf_counter()
    # best[node] is the best time found so far from that junction to a destination.
    best = {goal: 0.0 for goal in goals}
    # next_step[node] is the junction after it on the way to the destination.
    next_step = {}
    # Each queue item is (time so far + guess, junction). Smallest is looked at first.
    queue = [(guess(goal), goal) for goal in goals]
    heapq.heapify(queue)
    done = set()
    while queue:
        _, node = heapq.heappop(queue)
        # The same junction can be queued more than once. Only its first visit counts.
        if node in done:
            continue
        done.add(node)
        # Reaching the start means its best time is final, so the search can stop.
        if node == start:
            break
        # The graph is reversed, so these neighbors are the junctions that lead INTO node.
        for previous, roads in graph[node].items():
            # Two roads can join the same junctions; use the faster one.
            time_s = best[node] + min(data["time_s"] for data in roads.values())
            if time_s < best.get(previous, INFINITY):
                best[previous] = time_s
                next_step[previous] = node
                heapq.heappush(queue, (time_s + guess(previous), previous))
    milliseconds = (time.perf_counter() - started) * 1000

    if start not in done:
        return {"found": False, "ms": milliseconds}
    # Follow next_step from the start until reaching a destination.
    nodes = [start]
    while nodes[-1] not in goals:
        nodes.append(next_step[nodes[-1]])
    result = describe_path(network, mode, nodes)
    result.update(found=True, ms=milliseconds, nodes=nodes,
                  destination=destination_names(network, mode, nodes[-1]))
    return result
