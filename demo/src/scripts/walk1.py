#!/usr/bin/env python3
# gait_duty_autoadapt.py

import math, time, collections, argparse
from typing import List, Tuple, Deque, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# ----------------- Config base -----------------
CONTROLLER_NAME = "joint_trajectory_controller"
JOINTS: List[str] = [f"Revolution_{i}" for i in range(1, 13)]

# Grupos (índices en JOINTS): [elbow, shoulder, hip]
FL = [0, 1, 2]
RL = [3, 4, 5]
RR = [6, 7, 8]
FR = [9, 10, 11]
LEGS = [FL, RL, RR, FR]

# Patrón de trote: FL/RR en fase, FR/RL contrafase
PHASES = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}
# Alternativa si el avance sale al revés con tu geometría:
# PHASES = {"FL": math.pi, "RR": math.pi, "FR": 0.0, "RL": 0.0}

# --- Parámetros de marcha (últimos que nos pasaste) ---
FREQUENCY_HZ      = 0.38
DUTY_STANCE       = 0.66
PUBLISH_RATE_HZ   = 80.0
POINT_HORIZON_SEC = 0.18

DRIVER_DIR   = +1               # se puede invertir por odom si retrocede (global)
DRIVER_AMPL  = 0.26
AUX_AMPL     = 0.10
ELBOW_LIFT   = 0.16
BIAS_SHOULDER = +0.06
BIAS_HIP_ABD  = 0.05

# Signos (por cableado/frames) – ajusta si hiciera falta
SHOULDER_SIGN = {"FL": +1.0, "FR": +1.0, "RL": +1.0, "RR": +1.0}
ELBOW_SIGN    = {"FL": +1.0, "FR": +1.0, "RL": +1.0, "RR": +1.0}
# (si quieres también hip, crea HIP_SIGN y multiplícalo donde se fija q_hp)

# Dirección por pata (multiplica SOLO el hombro/driver)
LEG_DIR_DEFAULT = {"FL": +1, "FR": +1, "RL": +1, "RR": +1}

# Auto-adaptación por odometría
ODOM_TOPIC_DEFAULT = "/odom"
VX_WINDOW_SEC = 2.0
VX_INVERT_THRESH = -0.01

# Impresión
PRINT_EVERY_SEC = 1.0
# -------------------------------------------------------

def leg_name(idx):
    if idx is FL: return "FL"
    if idx is RL: return "RL"
    if idx is RR: return "RR"
    if idx is FR: return "FR"
    return "?"

def half_cosine(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return 0.5 - 0.5 * math.cos(math.pi * x)

def triangle(x: float) -> float:
    x = x - math.floor(x)
    if x < 0.25: return 4.0 * x
    elif x < 0.75: return 2.0 - 4.0 * x
    else: return -4.0 + 4.0 * x

def duty_profile(phase: float, duty: float) -> Tuple[float, bool, float]:
    phase = (phase % (2*math.pi)) / (2*math.pi)
    if phase < duty:
        tau = phase / duty
        s = -1.0 + tau * 1.0   # stance: -1 -> 0
        return s, True, tau
    else:
        tau = (phase - duty) / max(1e-6, (1.0 - duty))
        s = 0.0 + tau * 1.0    # swing: 0 -> +1
        return s, False, tau

# ----------------- Nodo de marcha -----------------
class DutyGaitAuto(Node):
    def __init__(self, odom_topic: str, do_print: bool, print_actual: bool, print_rate: float,
                 global_dir: int, leg_dir: Dict[str, int]):
        super().__init__("gait_duty_autoadapt")

        self.pub = self.create_publisher(JointTrajectory, f"/{CONTROLLER_NAME}/joint_trajectory", 10)

        qos_best = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_best)
        self.js_sub   = self.create_subscription(JointState, "/joint_states", self._js_cb, qos_best)

        # Estado
        self.t0 = time.time()
        self.global_dir = int(global_dir)          # reemplaza al antiguo driver_dir
        self.leg_dir = dict(LEG_DIR_DEFAULT)
        # aplica overrides de CLI (no None)
        for k, v in leg_dir.items():
            if v is not None:
                self.leg_dir[k] = int(v)

        self.vx_hist: Deque[Tuple[float, float]] = collections.deque()
        self.last_positions: Dict[str, float] = {}
        self._last_print_t = 0.0
        self._printed_header = False

        self.do_print = do_print
        self.print_actual = print_actual
        self.print_period = 1.0 / max(1e-6, print_rate)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        self.get_logger().info(
            f"Gait duty autoadapt: freq={FREQUENCY_HZ}Hz, duty={DUTY_STANCE}, DRIVER=shoulder, "
            f"global_dir={self.global_dir}, leg_dir={self.leg_dir} | odom={odom_topic}"
        )

    # ---- Callbacks ----
    def _odom_cb(self, msg: Odometry):
        t = time.time()
        vx = msg.twist.twist.linear.x
        self.vx_hist.append((t, vx))
        tmin = t - VX_WINDOW_SEC
        while self.vx_hist and self.vx_hist[0][0] < tmin:
            self.vx_hist.popleft()

    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.last_positions[n] = p

    def _avg_vx(self) -> Optional[float]:
        if not self.vx_hist:
            return None
        return sum(v for _, v in self.vx_hist) / len(self.vx_hist)

    # ---- Ciclo principal ----
    def _tick(self):
        t = time.time() - self.t0
        omega = 2.0 * math.pi * FREQUENCY_HZ
        q = [0.0] * len(JOINTS)

        # auto-inversión global si retrocede
        vx = self._avg_vx()
        if vx is not None and vx < VX_INVERT_THRESH:
            self.global_dir *= -1
            self.vx_hist.clear()
            self.get_logger().warn(
                f"vx medio={vx:.3f} m/s < {VX_INVERT_THRESH:.3f} → invierto global_dir a {self.global_dir}"
            )

        # generar comandos por pata
        for leg in LEGS:
            elbow, shoulder, hip = leg
            name = leg_name(leg)  # "FL/FR/RL/RR"
            phi = omega * t + PHASES[name]
            s, in_stance, tau = duty_profile(phi, DUTY_STANCE)

            # hombro (driver): multiplica global_dir * dir por pata * signo de cableado
            effective_dir = self.global_dir * self.leg_dir[name] * int(SHOULDER_SIGN[name])
            q_sh = BIAS_SHOULDER + (effective_dir * DRIVER_AMPL * s)

            # cadera acompaña poco (si tu hip es abducción/aducción, esto es pitch “virtual”)
            q_hp = AUX_AMPL * 0.35 * q_sh / max(1e-6, DRIVER_AMPL)

            # abrir base (abd/adducción alterna: delanteras +, traseras -)
            abd = +BIAS_HIP_ABD if name in ("FL", "FR") else -BIAS_HIP_ABD

            # codo: levanta en swing
            lift = 0.0 if in_stance else ELBOW_LIFT * half_cosine(tau)
            tri = triangle((phi / (2*math.pi)) + 0.25)
            q_el = ELBOW_SIGN[name] * (0.25 * (q_sh + q_hp) - lift * tri)

            # asignar
            q[shoulder] = q_sh
            q[hip]      = q_hp + abd
            q[elbow]    = q_el

        # publicar
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start = Duration(
            sec=int(POINT_HORIZON_SEC),
            nanosec=int((POINT_HORIZON_SEC % 1.0) * 1e9),
        )
        traj.points = [pt]
        self.pub.publish(traj)

        # impresión periódica
        if self.do_print:
            now = time.time()
            if (now - self._last_print_t) >= self.print_period:
                self._last_print_t = now
                if not self._printed_header:
                    hdr = "# t[s]  joint_name       cmd(rad)"
                    if self.print_actual:
                        hdr += "    act(rad)    err(rad)"
                    print(hdr)
                    self._printed_header = True

                for name, cmd in zip(JOINTS, q):
                    if self.print_actual:
                        act = self.last_positions.get(name, float("nan"))
                        err = act - cmd if (act == act) else float("nan")
                        print(f"t={t:6.2f}  {name:14s}  cmd={cmd:+.3f}   act={act:+.3f}   err={err:+.3f}")
                    else:
                        print(f"t={t:6.2f}  {name:14s}  cmd={cmd:+.3f}")

                if vx is not None:
                    print(f"vx̄≈ {vx:+.3f} m/s | global_dir={self.global_dir} leg_dir={self.leg_dir}")

# ----------------- Nodo de PRUEBA de signos -----------------
class JointTestNode(Node):
    def __init__(self, step: float, dwell: float, repetitions: int):
        super().__init__("gait_joint_test")
        self.pub = self.create_publisher(JointTrajectory, f"/{CONTROLLER_NAME}/joint_trajectory", 10)

        qos_best = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, qos_best)
        self.last_positions: Dict[str, float] = {}

        self.step = float(step)
        self.dwell = float(dwell)
        self.repetitions = int(repetitions)

    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.last_positions[n] = p

    def _read(self, name: str) -> float:
        return self.last_positions.get(name, float("nan"))

    def run(self):
        print("# ===== PRUEBA DE SIGNOS POR ARTICULACIÓN =====")
        print("# joint_name     step     cmd(rad)   base(rad)    act(rad)    delta(rad)    sentido")

        r = self.repetitions
        for name in JOINTS:
            base = self._read(name)
            # +step
            self._send_single(name, +self.step)
            time.sleep(self.dwell)
            act = self._read(name)
            delta = act - base if (act == act and base == base) else float("nan")
            sentido = "POS(↑)" if (delta == delta and delta >= 0.0) else "NEG(↓)"
            print(f"{name:14s}  {'+':<6s}  {(+self.step):+7.3f}   {base:+.3f}    {act:+.3f}    {delta:+.3f}    {sentido}")

            # -step
            base = self._read(name)
            self._send_single(name, -self.step)
            time.sleep(self.dwell)
            act = self._read(name)
            delta = act - base if (act == act and base == base) else float("nan")
            sentido = "POS(↑)" if (delta == delta and delta >= 0.0) else "NEG(↓)"
            print(f"{name:14s}  {'-':<6s}  {(-self.step):+7.3f}   {base:+.3f}    {act:+.3f}    {delta:+.3f}    {sentido}")

            # volver a 0
            self._send_single(name, 0.0)
            time.sleep(max(0.3, self.dwell*0.5))

    def _send_single(self, name: str, cmd: float):
        traj = JointTrajectory()
        traj.joint_names = [name]
        pt = JointTrajectoryPoint()
        pt.positions = [cmd]
        pt.time_from_start = Duration(sec=0, nanosec=int(0.15 * 1e9))
        traj.points = [pt]
        self.pub.publish(traj)

# ----------------- CLI -----------------
def parse_args():
    ap = argparse.ArgumentParser()
    # Modo normal: impresiones
    ap.add_argument("--print", dest="do_print", action="store_true",
                    help="Imprime comandos (y lecturas si se usa --print-actual).")
    ap.add_argument("--print-actual", action="store_true",
                    help="Imprime también act (joint_states) y err (act - cmd).")
    ap.add_argument("--print-rate", type=float, default=1.0,
                    help="Frecuencia de impresión en Hz (por defecto 1.0).")

    # Odom
    ap.add_argument("--odom-topic", type=str, default=ODOM_TOPIC_DEFAULT,
                    help=f"Tópico de odometría para autoajuste (por defecto {ODOM_TOPIC_DEFAULT}).")

    # Dirección global y por pata
    ap.add_argument("--dir", type=int, choices=[-1, 1], default=DRIVER_DIR,
                    help="Sentido global del driver del hombro (+1/-1).")
    ap.add_argument("--invert", action="store_true",
                    help="Invierte el sentido global al arrancar.")
    ap.add_argument("--dir-fl", type=int, choices=[-1, 1], default=None,
                    help="Multiplicador del hombro para FL (+1/-1).")
    ap.add_argument("--dir-fr", type=int, choices=[-1, 1], default=None,
                    help="Multiplicador del hombro para FR (+1/-1).")
    ap.add_argument("--dir-rl", type=int, choices=[-1, 1], default=None,
                    help="Multiplicador del hombro para RL (+1/-1).")
    ap.add_argument("--dir-rr", type=int, choices=[-1, 1], default=None,
                    help="Multiplicador del hombro para RR (+1/-1).")

    # Modo PRUEBA
    ap.add_argument("--prueba", action="store_true",
                    help="Modo test: mueve cada joint de uno en uno y muestra el sentido.")
    ap.add_argument("--prueba-step", type=float, default=0.20,
                    help="Paso de prueba en rad (default 0.20).")
    ap.add_argument("--prueba-dwell", type=float, default=0.35,
                    help="Espera entre pasos (s) (default 0.35).")
    ap.add_argument("--prueba-reps", type=int, default=1,
                    help="Repeticiones por joint (no acumulativo) (default 1).")

    return ap.parse_args()

# ----------------- Main -----------------
def main():
    args = parse_args()
    rclpy.init()

    if args.prueba:
        node = JointTestNode(step=args.prueba_step, dwell=args.prueba_dwell, repetitions=args.prueba_reps)
        try:
            # No hace falta spin continuo; corremos la prueba y salimos
            # Pero necesitamos procesar /joint_states mientras tanto:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)
            # Arranca un pequeño "spin" paralelo durante la prueba
            start = time.time()
            # Ejecuta la prueba publicando y leyendo
            node.run()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    # Modo marcha
    leg_dir_cli = {"FL": args.dir_fl, "FR": args.dir_fr, "RL": args.dir_rl, "RR": args.dir_rr}
    global_dir = args.dir * (-1 if args.invert else +1)

    node = DutyGaitAuto(
        odom_topic=args.odom_topic,
        do_print=args.do_print,
        print_actual=args.print_actual,
        print_rate=args.print_rate,
        global_dir=global_dir,
        leg_dir=leg_dir_cli,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
