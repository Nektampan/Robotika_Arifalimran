#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import os

class PRM:
    def __init__(self, start, goal, num_nodes, connection_radius, map_size):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.num_nodes = num_nodes
        self.connection_radius = connection_radius
        self.map_size = map_size
        self.nodes = [self.start] + self.generate_random_nodes() + [self.goal]
        self.graph = nx.Graph()

    def generate_random_nodes(self):
        return [(np.random.randint(0, self.map_size[0]), np.random.randint(0, self.map_size[1])) for _ in range(self.num_nodes)]

    def is_within_radius(self, node1, node2):
        return np.linalg.norm(np.array(node1) - np.array(node2)) < self.connection_radius

    def build_graph(self):
        for i, node in enumerate(self.nodes):
            for j, other_node in enumerate(self.nodes):
                if i != j and self.is_within_radius(node, other_node):
                    distance = np.linalg.norm(np.array(node) - np.array(other_node))
                    self.graph.add_edge(node, other_node, weight=distance)

    def find_path(self):
        try:
            return nx.shortest_path(self.graph, source=self.start, target=self.goal, weight="weight")
        except nx.NetworkXNoPath:
            rospy.logwarn("No path found between start and goal")
            return None

    def visualize(self, path=None):
        plt.figure(figsize=(8, 8))
        plt.xlim(0, self.map_size[0])
        plt.ylim(0, self.map_size[1])

        # Draw nodes
        for node in self.nodes:
            plt.scatter(*node, color="blue", s=10)

        # Draw edges
        for edge in self.graph.edges:
            node1, node2 = edge
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], color="gray", linestyle="--")

        # Draw path if found
        if path:
            for i in range(len(path) - 1):
                plt.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], color="red", linewidth=2)

        # Draw start and goal
        plt.scatter(*self.start, color="green", s=50, label="Start")
        plt.scatter(*self.goal, color="red", s=50, label="Goal")
        plt.legend()
        plt.title("Probabilistic Roadmap (PRM)")
        plt.show()

def load_params():
    config_path = os.path.join(rospy.get_param('/prm_package/config_path'), 'params.yaml')
    with open(config_path, 'r') as file:
        params = yaml.safe_load(file)["prm_params"]
    return params

if __name__ == "__main__":
    rospy.init_node("prm_node", anonymous=True)
    
    # Load parameters from params.yaml
    params = load_params()
    num_nodes = params["num_nodes"]
    connection_radius = params["connection_radius"]
    start = params["start"]
    goal = params["goal"]
    map_size = params["map_size"]

    prm = PRM(start, goal, num_nodes, connection_radius, map_size)
    prm.build_graph()
    path = prm.find_path()
    
    if path:
        rospy.loginfo(f"Path found: {path}")
    else:
        rospy.logwarn("No path could be found.")
    
    prm.visualize(path)
