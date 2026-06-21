#!/usr/bin/env python3
"""
Publica dos MarkerArrays:
  /waypoint_markers  — esferas + etiquetas en los 5 waypoints del pick-place.
  /scene_markers     — meshes estáticos de surgery_table y ur5_base en la misma
                       posición que tienen en Gazebo, convertida al frame base_link:
                         z_base_link = z_Gazebo − 0.63
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


WAYPOINT_KEYS = ['point_A_pre', 'point_A', 'point_via', 'point_B', 'point_B_post']
LABELS        = ['A_pre', 'A', 'via', 'B', 'B_post']

COLORS = [
    (0.2, 0.9, 0.2, 0.9),   # A_pre  — verde
    (0.9, 0.2, 0.2, 0.9),   # A      — rojo
    (0.2, 0.4, 1.0, 0.9),   # via    — azul
    (1.0, 0.5, 0.0, 0.9),   # B      — naranja
    (0.2, 0.9, 0.7, 0.9),   # B_post — cian
]

MARKER_FRAME = 'base_link'

# surgery_table: mesh bottom at z_local=0; model at z_Gazebo=0.289901.
#   Para que las patas toquen el piso de RViz (grid z=-0.63), marker en z_pin=-0.63.
# ur5_base: model at z_Gazebo=-0.000817; marker en z_pin=-0.631.
SCENE_MESHES = [
    {
        'id':       0,
        'ns':       'scene',
        'pos':      (0.300,  0.000, -0.630),
        'scale':    0.01,
        'color':    (0.80, 0.75, 0.65, 0.85),
        'mesh_uri': 'package://ur5_pick_place/meshes/surgery_table/meshes/surgery_table.dae',
    },
    {
        'id':       1,
        'ns':       'scene',
        'pos':      (0.000,  0.000, -0.631),
        'scale':    0.01,
        'color':    (0.55, 0.55, 0.55, 0.90),
        'mesh_uri': 'package://ur5_pick_place/meshes/ur5_base/meshes/ur5_base.dae',
    },
]


class WaypointMarkersNode(Node):
    def __init__(self):
        super().__init__('waypoint_markers')

        pp_share = get_package_share_directory('ur5_pick_place')
        yaml_path = os.path.join(pp_share, 'config', 'pick_place_params.yaml')
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        p = data['pick_place_node']['ros__parameters']
        self._waypoints = [p[k] for k in WAYPOINT_KEYS]

        self._pub_wp    = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self._pub_scene = self.create_publisher(MarkerArray, '/scene_markers',    10)
        self.create_timer(0.3, self._publish)

        self.get_logger().info(f'Waypoint markers en frame "{MARKER_FRAME}":')
        for label, pt in zip(LABELS, self._waypoints):
            self.get_logger().info(f'  {label}: {pt}')

    def _publish(self):
        now = self.get_clock().now().to_msg()

        # ── Waypoints ────────────────────────────────────────────────────────
        wp_arr = MarkerArray()
        for i, (pt, label, color) in enumerate(
                zip(self._waypoints, LABELS, COLORS)):
            m = Marker()
            m.header.frame_id = MARKER_FRAME
            m.header.stamp    = now
            m.ns   = 'waypoints'
            m.id   = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pt[0])
            m.pose.position.y = float(pt[1])
            m.pose.position.z = float(pt[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.07
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            wp_arr.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns     = 'labels'
            t.id     = i
            t.type   = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = float(pt[0])
            t.pose.position.y = float(pt[1])
            t.pose.position.z = float(pt[2]) + 0.06
            t.pose.orientation.w = 1.0
            t.scale.z = 0.07
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = label
            t.lifetime.sec = 0
            t.lifetime.nanosec = 0
            wp_arr.markers.append(t)

        self._pub_wp.publish(wp_arr)

        # ── Scene meshes ─────────────────────────────────────────────────────
        sc_arr = MarkerArray()
        for obj in SCENE_MESHES:
            m = Marker()
            m.header.frame_id = MARKER_FRAME
            m.header.stamp    = now
            m.ns     = obj['ns']
            m.id     = obj['id']
            m.type   = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = obj['pos']
            m.pose.orientation.w = 1.0
            s = obj['scale']
            m.scale.x = m.scale.y = m.scale.z = s
            r, g, b, a = obj['color']
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            m.mesh_resource = obj['mesh_uri']
            m.mesh_use_embedded_materials = True
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            sc_arr.markers.append(m)

        self._pub_scene.publish(sc_arr)


def main():
    rclpy.init()
    node = WaypointMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
