#!/usr/bin/env python3
"""
Publica tres MarkerArrays:
  /waypoint_markers  — esferas + etiquetas en los 5 waypoints del pick-place.
  /scene_markers     — meshes estáticos de surgery_table y ur5_base en la misma
                       posición que tienen en Gazebo, convertida al frame base_link:
                         z_base_link = z_Gazebo − 0.63
  /obstacle_markers  — caja AABB del obstáculo leída de pick_place_params.yaml
                       (se recarga automáticamente si el YAML cambia tras rebuild).
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


WAYPOINT_KEYS = ['point_A', 'point_B', 'point_O', 'point_C', 'point_D']
LABELS        = ['A', 'B', 'O', 'C', 'D']

COLORS = [
    (0.2, 0.9, 0.2, 0.9),   # A — verde
    (0.9, 0.2, 0.2, 0.9),   # B — rojo
    (0.2, 0.4, 1.0, 0.9),   # O — azul
    (1.0, 0.5, 0.0, 0.9),   # C — naranja
    (0.2, 0.9, 0.7, 0.9),   # D — cian
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

# Obstacle AABB visualization
OBSTACLE_NS    = 'obstacle'
OBSTACLE_COLOR = (0.85, 0.15, 0.10, 0.40)   # rojo semi-transparente
OBSTACLE_LABEL_COLOR = (1.0, 0.4, 0.2, 0.95)

# Recarga del YAML cada N segundos para reflejar cambios tras colcon build
YAML_RELOAD_PERIOD = 2.0


class WaypointMarkersNode(Node):
    def __init__(self):
        super().__init__('waypoint_markers')

        pp_share = get_package_share_directory('ur5_pick_place')
        self._yaml_path = os.path.join(pp_share, 'config', 'pick_place_params.yaml')

        # Placeholders; populated by _load_yaml()
        self._waypoints        = [[0.0, 0.0, 0.0]] * 5
        self._obs_center       = [0.0, 0.0, 0.0]
        self._obs_half         = [0.1, 0.1, 0.1]

        self._load_yaml()

        self._pub_wp    = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self._pub_scene = self.create_publisher(MarkerArray, '/scene_markers',    10)
        self._pub_obs   = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        self.create_timer(0.3,              self._publish)
        self.create_timer(YAML_RELOAD_PERIOD, self._reload_yaml)

    # ── YAML loading ──────────────────────────────────────────────────────────

    def _load_yaml(self):
        try:
            with open(self._yaml_path) as f:
                data = yaml.safe_load(f)
            p = data['pick_place_node']['ros__parameters']
            self._waypoints  = [p[k] for k in WAYPOINT_KEYS]
            self._obs_center = p['obstacle_center']
            self._obs_half   = p['obstacle_half_extents']
        except Exception as exc:
            self.get_logger().error(f'Error al leer YAML: {exc}')
            return

        self.get_logger().info(
            f'YAML recargado — '
            f'obs_center={self._obs_center}  '
            f'half_extents={self._obs_half}'
        )

    def _reload_yaml(self):
        self._load_yaml()

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self._publish_waypoints(now)
        self._publish_scene(now)
        self._publish_obstacle(now)

    def _publish_waypoints(self, now):
        wp_arr = MarkerArray()
        for i, (pt, label, color) in enumerate(
                zip(self._waypoints, LABELS, COLORS)):
            # Sphere
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
            m.lifetime.sec = m.lifetime.nanosec = 0
            wp_arr.markers.append(m)

            # Text label
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
            t.lifetime.sec = t.lifetime.nanosec = 0
            wp_arr.markers.append(t)

        self._pub_wp.publish(wp_arr)

    def _publish_scene(self, now):
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
            m.lifetime.sec = m.lifetime.nanosec = 0
            sc_arr.markers.append(m)

        self._pub_scene.publish(sc_arr)

    def _publish_obstacle(self, now):
        cx, cy, cz = [float(v) for v in self._obs_center]
        hx, hy, hz = [float(v) for v in self._obs_half]

        obs_arr = MarkerArray()

        # ── Caja AABB ────────────────────────────────────────────────────────
        box = Marker()
        box.header.frame_id = MARKER_FRAME
        box.header.stamp    = now
        box.ns     = OBSTACLE_NS
        box.id     = 0
        box.type   = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = cx
        box.pose.position.y = cy
        box.pose.position.z = cz
        box.pose.orientation.w = 1.0
        box.scale.x = 2.0 * hx
        box.scale.y = 2.0 * hy
        box.scale.z = 2.0 * hz
        r, g, b, a = OBSTACLE_COLOR
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, a
        box.lifetime.sec = box.lifetime.nanosec = 0
        obs_arr.markers.append(box)

        # ── Aristas de la caja (wireframe como LINE_LIST) ─────────────────
        wire = Marker()
        wire.header.frame_id = MARKER_FRAME
        wire.header.stamp    = now
        wire.ns     = OBSTACLE_NS
        wire.id     = 1
        wire.type   = Marker.LINE_LIST
        wire.action = Marker.ADD
        wire.pose.orientation.w = 1.0
        wire.scale.x = 0.005   # grosor de línea [m]
        wire.color.r = 0.7
        wire.color.g = 0.0
        wire.color.b = 0.0
        wire.color.a = 0.9
        wire.lifetime.sec = wire.lifetime.nanosec = 0

        from geometry_msgs.msg import Point
        corners = [
            (cx + sx * hx, cy + sy * hy, cz + sz * hz)
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
        ]
        # 12 aristas de un cubo (pares de índices de esquinas):
        edges = [
            (0,1),(2,3),(4,5),(6,7),   # aristas en Z
            (0,2),(1,3),(4,6),(5,7),   # aristas en Y
            (0,4),(1,5),(2,6),(3,7),   # aristas en X
        ]
        for i0, i1 in edges:
            p0 = Point(); p0.x, p0.y, p0.z = corners[i0]
            p1 = Point(); p1.x, p1.y, p1.z = corners[i1]
            wire.points.append(p0)
            wire.points.append(p1)
        obs_arr.markers.append(wire)

        # ── Etiqueta de texto ─────────────────────────────────────────────
        lbl = Marker()
        lbl.header.frame_id = MARKER_FRAME
        lbl.header.stamp    = now
        lbl.ns     = OBSTACLE_NS
        lbl.id     = 2
        lbl.type   = Marker.TEXT_VIEW_FACING
        lbl.action = Marker.ADD
        lbl.pose.position.x = cx
        lbl.pose.position.y = cy
        lbl.pose.position.z = cz + hz + 0.06
        lbl.pose.orientation.w = 1.0
        lbl.scale.z = 0.06
        r2, g2, b2, a2 = OBSTACLE_LABEL_COLOR
        lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = r2, g2, b2, a2
        lbl.text = (
            f'Obstáculo\n'
            f'{2*hx:.2f}×{2*hy:.2f}×{2*hz:.2f} m'
        )
        lbl.lifetime.sec = lbl.lifetime.nanosec = 0
        obs_arr.markers.append(lbl)

        self._pub_obs.publish(obs_arr)


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
