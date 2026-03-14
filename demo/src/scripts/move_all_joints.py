#!/usr/bin/env python3
# teleop_revolution_arrows.py
#
# Selecciona un joint (1..12 => Revolution_1..Revolution_12) y muévelo con ← / → (±0.1 rad).
# 'R' resetea TODOS los joints a su posición inicial (snapshot al detectar por 1ª vez todos los joints).
# Publica una JointTrajectory con los 12 joints: solo cambia el seleccionado; el resto se mantiene.
#
# Ejecuta:  ros2 run <tu_paquete> teleop_revolution_arrows.py
# (o: python3 teleop_revolution_arrows.py)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import curses
import time
from typing import Dict, List

# === CONFIGURACIÓN ===
CONTROLLER_NAME = "joint_trajectory_controller"  # ajusta al nombre de tu controlador
STEP_RAD = 0.1
POINT_TIME_SEC = 0.5

# Lista fija de joints en el orden que quieras mandar al controlador
JOINTS: List[str] = [
    "FR_elbow",
    "FR_shoulder",
    "FR_hip",
    "RR_elbow",
    "RR_shoulder",
    "RR_hip",
    "RL_elbow",
    "RL_shoulder",
    "RL_hip",
    "FL_elbow",
    "FL_shoulder",
    "FL_hip",
]  # Revolution_1 .. Revolution_12


class JointTeleop(Node):
    def __init__(self):
        super().__init__("teleop_revolution_arrows")

        qos = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, qos
        )

        self.pub = self.create_publisher(
            JointTrajectory, f"/{CONTROLLER_NAME}/joint_trajectory", 10
        )

        self.current_positions: Dict[str, float] = {}
        self.initial_positions: Dict[str, float] = {}
        self.initial_captured: bool = False
        self.selected_index: int = 0  # 0 => Revolution_1

        self.get_logger().info(
            f"Publicando a '/{CONTROLLER_NAME}/joint_trajectory'. "
            "Pulsa 1..12 para elegir joint; ←/→ para mover; 'R' reset; 'q' salir."
        )

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

        # Captura inicial cuando estén todos los joints disponibles
        if not self.initial_captured:
            have_all = all(j in self.current_positions for j in JOINTS)
            if have_all:
                self.initial_positions = {j: float(self.current_positions[j]) for j in JOINTS}
                self.initial_captured = True
                self.get_logger().info("Posiciones iniciales capturadas desde /joint_states.")

    def get_positions_vector(self) -> List[float]:
        """Devuelve las posiciones actuales para todos los JOINTS en orden; si falta, 0.0."""
        return [float(self.current_positions.get(j, 0.0)) for j in JOINTS]

    def get_initial_vector(self) -> List[float]:
        """Vector de posiciones iniciales (snapshot). Si no hay snapshot completo, usa 0.0 donde falte."""
        if self.initial_captured:
            return [float(self.initial_positions.get(j, 0.0)) for j in JOINTS]
        else:
            return [0.0 for _ in JOINTS]

    def set_selected_by_number(self, n: int):
        if 1 <= n <= len(JOINTS):
            self.selected_index = n - 1
            self.get_logger().info(f"Seleccionado: {JOINTS[self.selected_index]} (#{n})")
        else:
            self.get_logger().warn(f"Número fuera de rango: {n}")

    def _publish_positions(self, positions: List[float]):
        traj = JointTrajectory()
        traj.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(
            sec=int(POINT_TIME_SEC),
            nanosec=int((POINT_TIME_SEC % 1) * 1e9)
        )
        traj.points = [pt]
        self.pub.publish(traj)

    def send_increment(self, delta: float):
        positions = self.get_positions_vector()
        start = positions[self.selected_index]
        target = start + delta
        positions[self.selected_index] = target
        self._publish_positions(positions)
        self.get_logger().info(
            f"{JOINTS[self.selected_index]}: {start:+.3f} -> {target:+.3f} rad (Δ {delta:+.3f})"
        )

    def reset_to_initial(self):
        positions = self.get_initial_vector()
        self._publish_positions(positions)
        if self.initial_captured:
            msg = "Reset a posiciones iniciales capturadas."
        else:
            msg = "Reset a 0.0 (aún no se habían capturado posiciones iniciales)."
        self.get_logger().info(msg)


def curses_main(stdscr, node: JointTeleop):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()

    # Estado para introducir 1..12 con dos dígitos (10, 11, 12)
    digit_buffer = ""
    digit_buffer_time = 0.0
    DIGIT_TIMEOUT = 0.6  # s para completar 2º dígito

    def render():
        stdscr.erase()
        stdscr.addstr(0, 0, "Teleop Revolution joints | '1..12' elegir | ←/→ mover ±0.1 | 'R' reset | 'q' salir")
        stdscr.addstr(2, 0, f"Controller: {CONTROLLER_NAME}")
        stdscr.addstr(3, 0, f"Paso: {STEP_RAD} rad | Tiempo punto: {POINT_TIME_SEC}s")
        sel = node.selected_index + 1
        stdscr.addstr(5, 0, f"Seleccionado: #{sel}  {JOINTS[node.selected_index]}")
        pos_vec = node.get_positions_vector()
        stdscr.addstr(7, 0, "Posiciones actuales:")
        # Mostrar compactado en filas de 4
        row = 8
        for i, (name, val) in enumerate(zip(JOINTS, pos_vec), 1):
            stdscr.addstr(row, 0 + ((i - 1) % 4) * 26, f"{i:>2}) {name}: {val:+.4f}")
            if i % 4 == 0:
                row += 1
        if digit_buffer:
            stdscr.addstr(row + 2, 0, f"Entrada numérica: {digit_buffer}_")
        stdscr.refresh()

    last_ui = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.time()

            # Timeout del buffer numérico
            if digit_buffer and (now - digit_buffer_time) > DIGIT_TIMEOUT:
                n = int(digit_buffer)
                node.set_selected_by_number(n)
                digit_buffer = ""

            ch = stdscr.getch()
            if ch != curses.ERR:
                if ch in (ord('q'), ord('Q')):
                    break

                # Flechas
                if ch == curses.KEY_RIGHT:
                    node.send_increment(+STEP_RAD)
                elif ch == curses.KEY_LEFT:
                    node.send_increment(-STEP_RAD)

                # Reset
                elif ch in (ord('r'), ord('R')):
                    node.reset_to_initial()

                # Dígitos '0'..'9' para formar 1..12
                elif ord('0') <= ch <= ord('9'):
                    d = chr(ch)
                    if not digit_buffer and d == '0':
                        pass  # ignora '0' inicial
                    else:
                        digit_buffer += d
                        digit_buffer_time = now
                        if len(digit_buffer) == 2:
                            n = int(digit_buffer)
                            if 1 <= n <= len(JOINTS):
                                node.set_selected_by_number(n)
                            elif digit_buffer[1] != '0':
                                n = int(digit_buffer[1])
                                node.set_selected_by_number(n)
                            digit_buffer = ""

            if (now - last_ui) > 0.1:
                render()
                last_ui = now

            time.sleep(0.01)

    finally:
        stdscr.keypad(False)
        curses.nocbreak()
        curses.echo()


def main():
    rclpy.init()
    node = JointTeleop()
    try:
        curses.wrapper(curses_main, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
