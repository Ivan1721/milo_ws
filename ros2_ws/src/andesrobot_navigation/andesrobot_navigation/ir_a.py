"""Mandar a Milo a ubicaciones con nombre usando Nav2, y registrar cómo le fue (para el paper).

Ejemplos (con la navegación corriendo: ./sim.sh nav <mapa>):
  ros2 run andesrobot_navigation ir_a --lista
  ros2 run andesrobot_navigation ir_a kitchen
  ros2 run andesrobot_navigation ir_a sofa 16 entrada          # secuencia (16 = dinner_table)
  ros2 run andesrobot_navigation ir_a --tour --repeticiones 3 --csv /ros2_ws/resultados/tour.csv

--tour visita TODAS las ubicaciones del rulebook en orden aleatorio (semilla fija => repetible).
Por cada objetivo guarda en el CSV:
  ok, tiempo [s], largo del recorrido por odometría [m], largo del camino planeado al inicio [m],
  error final de posición [m] y de ángulo [grados] (vs. la pose objetivo, usando AMCL/TF),
  número de recuperaciones de Nav2 (spin/backup/clear costmap).
Con eso se arman las tablas de éxito / eficiencia (SPL) / precisión.
"""

import argparse
import csv
import math
import os
import random
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener

from andesrobot_navigation import ubicaciones as U


def make_pose(nav, frame, d):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(d['x'])
    p.pose.position.y = float(d['y'])
    p.pose.orientation.z, p.pose.orientation.w = U.yaw_to_quat(float(d['yaw']))
    return p


def path_length(path):
    if path is None or len(path.poses) < 2:
        return float('nan')
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Runner:
    def __init__(self, nav, frame):
        self.nav, self.frame = nav, frame
        # Nodo auxiliar para TF y odometría, con su propio hilo (BasicNavigator hace spin de su
        # propio nodo por dentro; no se puede meter el mismo nodo en dos executors).
        self.aux = rclpy.create_node('ir_a_aux')
        self.last_odom = None
        self.odom_dist = 0.0
        self.aux.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self.aux, spin_thread=True)

    def on_odom(self, msg):
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_odom is not None:
            self.odom_dist += math.dist(p, self.last_odom)
        self.last_odom = p

    def robot_pose(self):
        try:
            t = self.tf.lookup_transform(
                self.frame, 'base_footprint', Time(), timeout=Duration(seconds=1.0)
            )
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def go(self, d, timeout):
        nav = self.nav
        goal = make_pose(nav, self.frame, d)
        start = self.robot_pose()
        planned = float('nan')
        if start is not None:
            s = make_pose(nav, self.frame, dict(x=start[0], y=start[1], yaw=start[2]))
            planned = path_length(nav.getPath(s, goal, use_start=True))
        self.odom_dist = 0.0
        t0 = time.monotonic()
        nav.goToPose(goal)
        recov, last_print, cancelled = 0, 0.0, False
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                recov = max(recov, fb.number_of_recoveries)
                if time.monotonic() - last_print > 2.0:
                    last_print = time.monotonic()
                    nav.info(f'  -> {d["name"]}: faltan {fb.distance_remaining:.2f} m')
            if not cancelled and time.monotonic() - t0 > timeout:
                cancelled = True
                nav.warn(f'  {d["name"]}: se acabó el tiempo ({timeout:.0f} s), cancelando')
                nav.cancelTask()
        elapsed = time.monotonic() - t0
        res = nav.getResult()
        end = self.robot_pose()
        err_xy = err_yaw = float('nan')
        if end is not None:
            err_xy = math.dist(end[:2], (d['x'], d['y']))
            err_yaw = abs(math.degrees(ang_diff(end[2], d['yaw'])))
        return dict(
            ubicacion=d['name'],
            ok=int(res == TaskResult.SUCCEEDED),
            resultado=str(res).split('.')[-1],
            tiempo_s=round(elapsed, 2),
            recorrido_m=round(self.odom_dist, 3),
            plan_inicial_m=round(planned, 3),
            error_xy_m=round(err_xy, 3),
            error_yaw_deg=round(err_yaw, 1),
            recuperaciones=recov,
        )


def main():
    ap = argparse.ArgumentParser(description='Ir a ubicaciones con nombre (Nav2)')
    ap.add_argument('lugares', nargs='*', help='nombres, ids del rulebook o prefijos')
    ap.add_argument('--lista', action='store_true', help='mostrar ubicaciones y salir')
    ap.add_argument('--ubicaciones', default=None, help='otro .yaml de ubicaciones')
    ap.add_argument('--tour', action='store_true', help='visitar todas las del rulebook')
    ap.add_argument('--repeticiones', type=int, default=1)
    ap.add_argument('--semilla', type=int, default=0)
    ap.add_argument('--timeout', type=float, default=120.0, help='s máximo por objetivo')
    ap.add_argument('--csv', default=None, help='guardar resultados en este CSV')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    frame, locs = U.load(args.ubicaciones)
    if args.lista or (not args.lugares and not args.tour):
        for n, d in sorted(locs.items(), key=lambda kv: (kv[1]['room'], kv[0])):
            rid = d.get('id')
            print(
                f'  {n:20s} id={rid if rid is not None else "-":>4}  sala={d["room"]:12s} '
                f'({d["x"]:.2f}, {d["y"]:.2f}, {math.degrees(d["yaw"]):.0f}°)'
            )
        return 0

    if args.tour:
        base = [d for d in locs.values() if d.get('id') is not None]
        if not base:  # archivo simple, sin ids del rulebook: visitar todos los lugares
            base = list(locs.values())
        rng = random.Random(args.semilla)
        plan = []
        for _ in range(args.repeticiones):
            seq = base[:]
            rng.shuffle(seq)
            plan += seq
    else:
        try:
            plan = [U.resolve(n, locs) for n in args.lugares] * args.repeticiones
        except KeyError as e:
            print(e)
            return 1

    rclpy.init()
    nav = BasicNavigator(node_name='ir_a')
    # No usamos waitUntilNav2Active() porque en Humble re-publica /initialpose en (0,0)
    # y pisaría la pose inicial que ya tiene AMCL (x:= y:= yaw:= del launch).
    nav._waitForNodeToActivate('amcl')
    nav._waitForNodeToActivate('bt_navigator')
    runner = Runner(nav, frame)
    rows = []
    for i, d in enumerate(plan):
        nav.info(f'[{i + 1}/{len(plan)}] Yendo a {d["name"]} ({d["x"]:.2f}, {d["y"]:.2f})')
        r = runner.go(d, args.timeout)
        r['n'] = i + 1
        rows.append(r)
        nav.info(
            f'  {r["resultado"]}: {r["tiempo_s"]} s, {r["recorrido_m"]} m, '
            f'error {r["error_xy_m"]} m / {r["error_yaw_deg"]}°'
        )

    ok = sum(r['ok'] for r in rows)
    nav.info(f'Listo: {ok}/{len(rows)} objetivos alcanzados')
    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        with open(args.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=['n'] + [k for k in rows[0] if k != 'n'])
            w.writeheader()
            w.writerows(rows)
        nav.info(f'Resultados en {args.csv}')
    nav.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(rows) else 2


if __name__ == '__main__':
    sys.exit(main())
