#!/usr/bin/env python3
# gait_duty_autoadapt.py
# ROS 2 (rclpy) - patrón de marcha con compensaciones “software” para geometría rara (URDF sin tocar).

import math, time, collections, argparse
from typing import List, Tuple, Deque, Optional, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# ======================= CONFIG BASE =======================
CONTROLLER_NAME = "joint_trajectory_controller"
JOINTS: List[str] = [f"Revolution_{i}" for i in range(1, 13)]

# Índices (en JOINTS): [elbow, shoulder, hip]
FL = [0, 1, 2]
RL = [3, 4, 5]
RR = [6, 7, 8]
FR = [9, 10, 11]
LEGS = [FL, RL, RR, FR]

LEG_NAMES = {
    id(FL): "FL",
    id(RL): "RL",
    id(RR): "RR",
    id(FR): "FR",
}

def leg_name(leg):
    return LEG_NAMES.get(id(leg), "?")

# Trote: FL/RR en fase; FR/RL a π.
PHASES = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}

# ====== Parámetros “suaves” por defecto (ajustables por CLI) ======
FREQUENCY_HZ      = 0.60     # un poco más bajo para estabilidad
DUTY_STANCE       = 0.66
PUBLISH_RATE_HZ   = 60.0
POINT_HORIZON_SEC = 0.18

# Driver en hombro
DRIVER_AMPL       = 0.24     # menor para no saturar fácil
AUX_AMPL          = 0.10
ELBOW_LIFT        = 0.18     # un poco más de lift
BIAS_SHOULDER     = +0.06
BIAS_HIP_ABD      = 0.05

# Límites (coinciden con URDF)
JOINT_LOWER = -1.57
JOINT_UPPER = +1.57
SOFT_MARGIN = 0.12           # margen para “desaturar” hombro

# Auto-inversión por odometría (opcional)
ODOM_TOPIC_DEFAULT = "/odom"
VX_WINDOW_SEC = 2.0
VX_INVERT_THRESH = -0.01

# ====== Ganancias/dir por pata (se pueden cambiar por CLI) ======
DEFAULT_DIR = {"FL": +1.0, "FR": +1.0, "RL": +1.0, "RR": +1.0}
# Ganancias para compensar geometría: empuja más con codo que con hombro
DEFAULT_SHOULDER_GAIN = {"FL": 1.0, "FR": 1.0, "RL": 1.0, "RR": 1.0}
DEFAULT_ELBOW_GAIN    = {"FL": 2.5, "FR": 2.5, "RL": 2.5, "RR": 2.5}
# ================================================================

def half_cosine(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return 0.5 - 0.5 * math.cos(math.pi * x)

def triangle(x: float) -> float:
    x = x - math.floor(x)
    if x < 0.25: return 4.0 * x
    elif x < 0.75: return 2.0 - 4.0 * x
    else: return -4.0 + 4.0 * x

def duty_profile(phase: float, duty: float) -> Tuple[float, bool, float]:
    """Devuelve s∈[-1,1], in_stance, tau∈[0,1] dentro de stance/swing."""
    phase = (phase % (2*math.pi)) / (2*math.pi)
    if phase < duty:
        tau = phase / duty
        s = -1.0 + tau * 1.0
        return s, True, tau
    else:
        tau = (phase - duty) / max(1e-6, (1.0 - duty))
        s = 0.0 + tau * 1.0
        return s, False, tau

class DutyGaitAuto(Node):
    def __init__(self,
                 odom_topic: str,
                 do_print: bool,
                 print_actual: bool,
                 print_rate: float,
                 dir_per_leg: Dict[str, float],
                 shoulder_gain: Dict[str, float],
                 elbow_gain: Dict[str, float],
                 geom_comp: bool,
                 test_mode: bool,
                 test_step: float,
                 test_hold: float):
        super().__init__("gait_duty_autoadapt")

        # Publicador
        self.pub = self.create_publisher(
            JointTrajectory, f"/{CONTROLLER_NAME}/joint_trajectory", 10
        )

        # Odom
        qos_odom = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=20,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_odom)

        # Joint states
        qos_js = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=50,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, qos_js)

        # Estado
        self.t0 = time.time()
        self.vx_hist: Deque[Tuple[float, float]] = collections.deque()
        self.last_positions: Dict[str, float] = {}
        self._last_print_t = 0.0
        self._printed_header = False

        # Config impresión
        self.do_print = do_print
        self.print_actual = print_actual
        self.print_period = 1.0 / max(1e-6, print_rate)

        # Correcciones
        self.dir_per_leg = dir_per_leg.copy()
        self.shoulder_gain = shoulder_gain.copy()
        self.elbow_gain = elbow_gain.copy()
        self.geom_comp = geom_comp

        # Test
        self.test_mode = test_mode
        self.test_step = test_step
        self.test_hold = test_hold
        self._test_state = 0
        self._test_index = 0
        self._test_last_t = time.time()

        # Timer
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        # Log
        self.get_logger().info(
            f"Gait duty autoadapt: freq={FREQUENCY_HZ}Hz, duty={DUTY_STANCE}, "
            f"geom_comp={self.geom_comp} | odom={odom_topic}"
        )
        self.get_logger().info(
            f"DIR per leg: {self.dir_per_leg} | shoulder_gain={self.shoulder_gain} | elbow_gain={self.elbow_gain}"
        )

    # ------ Callbacks ------
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

    # ------ Utilidades ------
    def _soft_limit(self, name: str, cmd: float) -> float:
        """Limita suave y devuelve cmd recortado si se acerca al límite."""
        up = JOINT_UPPER - SOFT_MARGIN
        lo = JOINT_LOWER + SOFT_MARGIN
        if cmd > up:
            cmd = up - 0.02
        elif cmd < lo:
            cmd = lo + 0.02
        return cmd

    # ------ TEST MODE ------
    def _tick_test(self) -> List[float]:
        """Secuencia: cada joint +step, espera; luego -step, espera; pasa al siguiente."""
        q = [0.0] * len(JOINTS)
        now = time.time()
        if self._test_state == 0:
            # base
            if now - self._test_last_t > self.test_hold:
                self._test_state = 1
                self._test_last_t = now
        elif self._test_state == 1:
            # +step
            q[self._test_index] = self.test_step
            if now - self._test_last_t > self.test_hold:
                self._test_state = 2
                self._test_last_t = now
        elif self._test_state == 2:
            # -step
            q[self._test_index] = -self.test_step
            if now - self._test_last_t > self.test_hold:
                self._test_state = 3
                self._test_last_t = now
        elif self._test_state == 3:
            # siguiente joint
            self._test_index += 1
            if self._test_index >= len(JOINTS):
                self._test_index = 0
            self._test_state = 0
            self._test_last_t = now

        # impresión simple
        if self.do_print and (now - self._last_print_t) >= self.print_period:
            self._last_print_t = now
            jn = JOINTS[self._test_index]
            act = self.last_positions.get(jn, float("nan"))
            print(f"[TEST] joint={jn:14s} state={self._test_state} cmd={q[self._test_index]:+.3f} act={act:+.3f}")
        return q

    # ------ MAIN TICK ------
    def _tick(self):
        # Modo prueba
        if self.test_mode:
            q = self._tick_test()
            self._publish(q)
            return

        t = time.time() - self.t0
        omega = 2.0 * math.pi * FREQUENCY_HZ
        q = [0.0] * len(JOINTS)

        # auto-inversión si retrocede (no invierte si test_mode)
        vx = self._avg_vx()
        # Nota: aquí no invertimos nada automáticamente; dejamos al usuario fijar dir_per_leg.
        # Si quisieras auto invertir TODO, podrías multiplicar todas las dir_per_leg por -1
        # cuando vx < VX_INVERT_THRESH.

        # comandos por pata
        for leg in LEGS:
            elbow, shoulder, hip = leg
            name = leg_name(leg)
            phi = omega * t + PHASES[name]
            s, in_stance, tau = duty_profile(phi, DUTY_STANCE)

            # hombro (driver) – dir por pata y ganancia
            q_sh = BIAS_SHOULDER + self.dir_per_leg[name] * (DRIVER_AMPL * s)
            q_sh *= self.shoulder_gain[name]

            # cadera acompaña discretamente (abducción fija + pequeño seguidor)
            q_hp = (AUX_AMPL * 0.35 * q_sh / max(1e-6, DRIVER_AMPL))
            abd = +BIAS_HIP_ABD if name in ("FL", "FR") else -BIAS_HIP_ABD

            # codo – más lift en swing, con ganancia grande para “compensar”
            lift = 0.0 if in_stance else ELBOW_LIFT * half_cosine(tau)
            tri = triangle((phi / (2*math.pi)) + 0.25)
            q_el = 0.25 * (q_sh + q_hp) - lift * tri
            q_el *= self.elbow_gain[name]

            # Compensación: si hombro se acerca a límite, reduce hombro y pasa parte al codo
            if self.geom_comp:
                q_sh_soft = self._soft_limit(f"{name}_shoulder", q_sh)
                if abs(q_sh_soft - q_sh) > 1e-6:
                    # hemos recortado → mover efecto al codo
                    delta = q_sh - q_sh_soft
                    q_sh = q_sh_soft
                    q_el += 0.6 * delta * self.elbow_gain[name]

            # clamps finales
            q_sh = self._soft_limit(f"{name}_shoulder", q_sh)
            q_el = self._soft_limit(f"{name}_elbow",    q_el)
            q_hp = self._soft_limit(f"{name}_hip",      q_hp + abd)

            q[shoulder] = q_sh
            q[hip]      = q_hp
            q[elbow]    = q_el

        self._publish(q)

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
                    print(f"vx̄≈ {vx:+.3f} m/s | dir={self.dir_per_leg}")

    def _publish(self, positions: List[float]):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(
            sec=int(POINT_HORIZON_SEC),
            nanosec=int((POINT_HORIZON_SEC % 1.0) * 1e9),
        )
        traj.points = [pt]
        self.pub.publish(traj)

# ======================= CLI =======================
def parse_args():
    ap = argparse.ArgumentParser(description="Duty gait con compensaciones software (sin tocar URDF).")

    ap.add_argument("--print", dest="do_print", action="store_true",
                    help="Imprime comandos (y lecturas si se usa --print-actual).")
    ap.add_argument("--print-actual", action="store_true",
                    help="Imprime también act (joint_states) y err (act - cmd).")
    ap.add_argument("--print-rate", type=float, default=1.0,
                    help="Frecuencia de impresión en Hz (por defecto 1.0).")
    ap.add_argument("--odom-topic", type=str, default=ODOM_TOPIC_DEFAULT,
                    help=f"Tópico de odometría para autoajuste (por defecto {ODOM_TOPIC_DEFAULT}).")

    # Overrides rápidos
    ap.add_argument("--freq", type=float, default=FREQUENCY_HZ)
    ap.add_argument("--duty", type=float, default=DUTY_STANCE)
    ap.add_argument("--rate", type=float, default=PUBLISH_RATE_HZ)
    ap.add_argument("--horizon", type=float, default=POINT_HORIZON_SEC)

    ap.add_argument("--driver-ampl", type=float, default=DRIVER_AMPL)
    ap.add_argument("--aux-ampl", type=float, default=AUX_AMPL)
    ap.add_argument("--elbow-lift", type=float, default=ELBOW_LIFT)
    ap.add_argument("--bias-shoulder", type=float, default=BIAS_SHOULDER)
    ap.add_argument("--bias-abd", type=float, default=BIAS_HIP_ABD)

    # Direcciones por pata
    ap.add_argument("--dir-FL", type=float, default=DEFAULT_DIR["FL"])
    ap.add_argument("--dir-FR", type=float, default=DEFAULT_DIR["FR"])
    ap.add_argument("--dir-RL", type=float, default=DEFAULT_DIR["RL"])
    ap.add_argument("--dir-RR", type=float, default=DEFAULT_DIR["RR"])

    # Ganancias por pata
    ap.add_argument("--shoulder-gain-FL", type=float, default=DEFAULT_SHOULDER_GAIN["FL"])
    ap.add_argument("--shoulder-gain-FR", type=float, default=DEFAULT_SHOULDER_GAIN["FR"])
    ap.add_argument("--shoulder-gain-RL", type=float, default=DEFAULT_SHOULDER_GAIN["RL"])
    ap.add_argument("--shoulder-gain-RR", type=float, default=DEFAULT_SHOULDER_GAIN["RR"])

    ap.add_argument("--elbow-gain-FL", type=float, default=DEFAULT_ELBOW_GAIN["FL"])
    ap.add_argument("--elbow-gain-FR", type=float, default=DEFAULT_ELBOW_GAIN["FR"])
    ap.add_argument("--elbow-gain-RL", type=float, default=DEFAULT_ELBOW_GAIN["RL"])
    ap.add_argument("--elbow-gain-RR", type=float, default=DEFAULT_ELBOW_GAIN["RR"])

    ap.add_argument("--geom-comp", action="store_true",
                    help="Activa compensación: reduce hombro cerca de límite y deriva parte al codo.")

    # Modo prueba
    ap.add_argument("--test", action="store_true",
                    help="Modo prueba: mueve cada joint +step/-step secuencialmente.")
    ap.add_argument("--test-step", type=float, default=0.20)
    ap.add_argument("--test-hold", type=float, default=0.60)

    return ap.parse_args()

def main():
    global FREQUENCY_HZ, DUTY_STANCE, PUBLISH_RATE_HZ, POINT_HORIZON_SEC
    global DRIVER_AMPL, AUX_AMPL, ELBOW_LIFT, BIAS_SHOULDER, BIAS_HIP_ABD

    args = parse_args()
    # aplica overrides
    FREQUENCY_HZ      = args.freq
    DUTY_STANCE       = args.duty
    PUBLISH_RATE_HZ   = args.rate
    POINT_HORIZON_SEC = args.horizon

    DRIVER_AMPL    = args.driver_ampl
    AUX_AMPL       = args.aux_ampl
    ELBOW_LIFT     = args.elbow_lift
    BIAS_SHOULDER  = args.bias_shoulder
    BIAS_HIP_ABD   = args.bias_abd

    dir_per_leg = {"FL": args.dir_FL, "FR": args.dir_FR, "RL": args.dir_RL, "RR": args.dir_RR}
    shoulder_gain = {
        "FL": args.shoulder_gain_FL, "FR": args.shoulder_gain_FR,
        "RL": args.shoulder_gain_RL, "RR": args.shoulder_gain_RR,
    }
    elbow_gain = {
        "FL": args.elbow_gain_FL, "FR": args.elbow_gain_FR,
        "RL": args.elbow_gain_RL, "RR": args.elbow_gain_RR,
    }

    rclpy.init()
    node = DutyGaitAuto(
        odom_topic=args.odom_topic,
        do_print=args.do_print,
        print_actual=args.print_actual,
        print_rate=args.print_rate,
        dir_per_leg=dir_per_leg,
        shoulder_gain=shoulder_gain,
        elbow_gain=elbow_gain,
        geom_comp=args.geom_comp,
        test_mode=args.test,
        test_step=args.test_step,
        test_hold=args.test_hold,
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
