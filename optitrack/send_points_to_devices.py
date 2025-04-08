import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
import sys
import pandas as pd
import numpy as np
import argparse
import scipy.spatial


class GeofencePusher(Node):
    def __init__(self):
        super().__init__("geofence_waypoints_node")

        # Create a service client for pushing waypoints
        self.geofence_client = self.create_client(WaypointPush, "/mavros/geofence/push")

        

        # Wait for the service to be available
        if not self.geofence_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Geofence push service not available")
            sys.exit(1)

        # Define waypoints
        points = [
            (-3.0, -3.0),  # Northwest point
            (-3.0, 3.0),  # Northeast point
            (3.0, 3.0),  # Southeast point
            (3.0, -3.0),  # Southwest point
        ]

        points = [self.xy_to_latlon(point) for point in points]
        self.waypoints = [
            Waypoint(
                frame=Waypoint.FRAME_GLOBAL,
                command=5001,
                is_current=True,
                autocontinue=True,
                param1=4.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                x_lat=x,
                y_long=y,
                z_alt=0.0, # ignored
            )
            for x, y in points
        ]
    
    def make_waypoints(self, geofence): 
        waypoints = [
            Waypoint(
                frame=Waypoint.FRAME_GLOBAL,
                command=5001,
                is_current=True,
                autocontinue=True,
                param1=4.0,
                param2=0.0,
                param3=0.0,
                param4=0.0,
                x_lat=x,
                y_long=y,
                z_alt=0.0, # ignored
            )
            for x, y in geofence
        ]

        return waypoints

    def xy_to_latlon(self, xy_tuple: tuple) -> tuple: 
        x, y = xy_tuple
        lat = y/111320.0
        lon = x/(111320.0*np.cos(lat))

        return (lat, lon)

    def push_geofence(self, geofence):
        """
        Push geofence waypoints to mavros
        """
        # Prepare the request
        request = WaypointPush.Request()

        # Set start index and waypoints
        request.start_index = 0
        waypoints = self.make_waypoints(geofence)
        request.waypoints = self.waypoints
        future = self.geofence_client.call_async(request)
        return future


def get_geofence(csv_file_path):
    data = pd.read_csv(csv_file_path)
    print(data)
    data = data.drop(["z"], axis=1)
    data = data[~data.duplicated(subset=["x", "y"], keep='first')]
    hull = scipy.spatial.ConvexHull(data[["x", "y"]])
    simplices = hull.simplices

    edges = []
    print(simplices)
    for simplex in simplices:
        edges.append((min(simplex[0], simplex[1]), max(simplex[0], simplex[1])))
    
    edges_visited = [False]*len(edges)
    print(edges_visited)
    cur_edge = 0

    visit_order = [edges[cur_edge][0]]
    
    while not all(edges_visited): 
        edge = edges[cur_edge]
        edges_visited[cur_edge] = True

        
        vertex_a, vertex_b = edge
        if vertex_a in visit_order: 
            next_vertex = vertex_b
        else: 
            next_vertex = vertex_a

        visit_order.append(next_vertex)
        # find edge that contains next vertex
        for prospective_edge_id in range(len(edges)):
            if edges_visited[prospective_edge_id]: 
                continue
            
            prospective_edge = edges[prospective_edge_id]
            if prospective_edge[0] == next_vertex or prospective_edge[1] == next_vertex: 
                cur_edge = prospective_edge_id

        if cur_edge in edges_visited:
            # we're done
            break
    
    return data[["x", "y"]].to_numpy()[visit_order]
            
        


   
    

def main(mainargs=None):

    parser = argparse.ArgumentParser("Geofence Pusher")
    parser.add_argument("filename")

    args = parser.parse_args()

    geofence = get_geofence(args.filename)

    # Initialize ROS2
    rclpy.init(args=mainargs)

    # Create the node
    node = GeofencePusher()

    # Push geofence and determine exit code
    future = node.push_geofence(geofence)

    rclpy.spin_until_future_complete(node, future)
    # Clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
