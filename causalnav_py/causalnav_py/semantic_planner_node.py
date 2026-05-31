import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
import networkx as nx
import requests
import math

class SemanticPlanner(Node):
    def __init__(self):
        super().__init__('semantic_planner')
        
        self.G = nx.Graph()
        self.build_topological_map()
        
        self.command_sub = self.create_subscription(String, '/user_command', self.command_cb, 10)
        self.route_pub = self.create_publisher(PoseArray, '/planned_route', 10)
        
        # Local LLM Endpoint 
        self.llm_url = "http://127.0.0.1:8080/v1/chat/completions"
        self.get_logger().info("Semantic Planner Online. Topological Graph Loaded.")

    def build_topological_map(self):
        nodes = {
            "spawn_point": (0.0, 0.0),
            "main_intersection": (10.0, 0.0),
            "urban_crosswalk": (10.0, 15.0),
            "alley_entrance": (25.0, 0.0),
            "cafe_patio": (25.0, -10.0)
        }
        
        for name, coords in nodes.items():
            self.G.add_node(name, pos=coords)

        edges = [
            ("spawn_point", "main_intersection"),
            ("main_intersection", "urban_crosswalk"),
            ("main_intersection", "alley_entrance"),
            ("alley_entrance", "cafe_patio")
        ]
        
        for u, v in edges:
            x1, y1 = self.G.nodes[u]['pos']
            x2, y2 = self.G.nodes[v]['pos']
            dist = math.hypot(x2 - x1, y2 - y1)
            self.G.add_edge(u, v, weight=dist)

    def command_cb(self, msg):
        user_intent = msg.data
        valid_targets = list(self.G.nodes())
        
        system_prompt = (
            "You are the routing brain for a robot. Match the user's intent to the closest destination. "
            f"Valid destinations: {valid_targets}. "
            "Respond ONLY with the exact destination name."
        )
        
        payload = {
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_intent}
            ],
            "temperature": 0.1,
            "max_tokens": 10
        }

        try:
            # Blocking request is safe here, it does not freeze motion control
            response = requests.post(self.llm_url, json=payload, timeout=10.0)
            target_node = response.json()['choices'][0]['message']['content'].strip()
            
            if target_node not in self.G:
                self.get_logger().error(f"LLM hallucinated invalid node: {target_node}")
                return
                
            start_node = "spawn_point"
            path = nx.shortest_path(self.G, source=start_node, target=target_node, weight='weight')
            self.get_logger().info(f"Calculated Route: {' -> '.join([str(node) for node in path])}")
            
            route_msg = PoseArray()
            route_msg.header.frame_id = "map"
            route_msg.header.stamp = self.get_clock().now().to_msg()
            
            for node_name in path:
                x, y = self.G.nodes[node_name]['pos']
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                route_msg.poses.append(pose)
                
            self.route_pub.publish(route_msg)

        except Exception as e:
            self.get_logger().error(f"Routing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SemanticPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()