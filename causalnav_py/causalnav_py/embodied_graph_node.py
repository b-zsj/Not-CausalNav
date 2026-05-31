import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import threading

class GraphNode:
    """Represents a unique physical object in the CausalNav Embodied Graph."""
    def __init__(self, node_id, label, position, initial_track_id):
        self.id = node_id
        self.label = label
        self.position = np.array(position)  # [x, y, z] in map/world frame
        self.observations = 1
        self.associated_track_ids = {initial_track_id}

        # Spatial-Temporal Corridor metrics to identify moving objects
        self.initial_position = np.array(position)
        self.is_dynamic = False

    def update_position(self, new_pos, track_id, alpha=0.2):
        """Smooths the node position using a moving average filter."""
        self.position = (1 - alpha) * self.position + alpha * np.array(new_pos)
        self.observations += 1
        self.associated_track_ids.add(track_id)

        # Corridor Filter: If an entity drifts more than 1.5 meters from its origin,
        # it is a dynamic entity (e.g., a pedestrian), not a static landmark.
        total_displacement = np.linalg.norm(self.position - self.initial_position)
        if total_displacement > 1.5:
            self.is_dynamic = True

class EmbodiedGraphNode(Node):
    def __init__(self):
        super().__init__('embodied_graph_node')
        
        # Graph Storage & Thread Safety
        self.graph = {}  # dict mapping node_id -> GraphNode instance
        self.node_counter = 0
        self.lock = threading.Lock()
        
        self.active_tracks = {}
        # Hyperparameters
        self.clustering_threshold = 1.5  # Distance in meters to merge observations
        
        # Temporal Buffer for incoming detections within the 1s window
        self.detection_buffer = []

        # Subscriber: Receives 3D positions of detected objects
        # Note: You'll need to modify your segmentation node to publish to this topic
        self.subscription = self.create_subscription(
            PointStamped,
            '/causalnav/tracked_centroids',
            self.centroid_callback,
            10
        )

        # 1 Hz Timer to process the buffer and update the graph structural topology
        self.graph_timer = self.create_timer(1.0, self.update_graph_callback)
        self.get_logger().info("Embodied Graph Node initialized at 1 Hz.")

    def centroid_callback(self, msg):
        # Unpack the compound frame_id
        try:
            label, track_id = msg.header.frame_id.split(':')
        except ValueError:
            label = msg.header.frame_id
            track_id = "unknown"

        pos = [msg.point.x, msg.point.y, msg.point.z]
        with self.lock:
            self.detection_buffer.append({'label': label, 'track_id': track_id, 'pos': pos})

    def update_graph_callback(self):
        """Processes the 1-second batch of detections at 1 Hz."""
        with self.lock:
            local_buffer = list(self.detection_buffer)
            self.detection_buffer.clear()

        if not local_buffer:
            return

        for obs in local_buffer:
            label = obs['label']
            track_id = obs['track_id']
            pos = obs['pos']
            matched = False

            # 1. Deterministic Association via ByteTrack ID
            if track_id != "unknown" and track_id in self.active_tracks:
                node_id = self.active_tracks[track_id]
                self.graph[node_id].update_position(pos, track_id)
                matched = True

            # 2. Re-entry Spatial Association fallback
            if not matched:
                for node_id, node in self.graph.items():
                    if node.label == label:
                        distance = np.linalg.norm(node.position - np.array(pos))
                        if distance < self.clustering_threshold:
                            node.update_position(pos, track_id)
                            if track_id != "unknown":
                                self.active_tracks[track_id] = node_id
                            matched = True
                            break

            # If no close match exists, instantiate a new semantic node
            if not matched:
                new_node = GraphNode(self.node_counter, label, pos, track_id)
                self.graph[self.node_counter] = new_node

                #self.get_logger().info(f"New Node Added: [{new_node.id}] {new_node.label} at {new_node.position}")
                if track_id != "unknown":
                    self.active_tracks[track_id] = self.node_counter
                self.node_counter += 1

        # Debug printout of current scene state
        self.log_graph_state()

    def log_graph_state(self):
        self.get_logger().info(f"\n--- Embodied Graph State (Total Nodes: {len(self.graph)}) ---")
        for node_id, node in self.graph.items():
            if node.observations > 2:
                status = "DYNAMIC (Ignored)" if node.is_dynamic else "STATIC (Landmark)"
                pos_str = f"[{node.position[0]:.2f}, {node.position[1]:.2f}, {node.position[2]:.2f}]"
                self.get_logger().info(
                    f" Node {node_id} | {node.label:<12} | Pos: {pos_str} | Hits: {node.observations:<3} | {status}"
                )
        self.get_logger().info("---------------------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = EmbodiedGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()