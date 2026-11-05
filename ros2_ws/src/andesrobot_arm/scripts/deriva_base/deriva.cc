// Plugin de Gazebo (sin ROS) para medir la deriva de la base; lo usa correr.sh.
// Reproduce lo que hacen gazebo_ros2_control (mando por posición: SetPosition(q, true) +
// SetVelocity(0) en cada paso, con la orden nueva a 100 Hz) y gazebo_ros_diff_drive (ruedas con
// "vel" = 0 y "fmax" = torque máximo) sin ROS, para medir cuánto resbala la base con el brazo en
// el aire. Variables de entorno: SALIDA, N, ESCALA, TASA (Hz de la orden), TORQUE, SEMILLA.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace gazebo {
static double env(const char *k, double d) { const char *v = getenv(k); return v ? atof(v) : d; }

class Deriva : public ModelPlugin {
  physics::ModelPtr m;
  std::vector<physics::JointPtr> brazo, dedos, ruedas;
  std::vector<double> q0, q1, qcmd;
  double espera = 1, t_ini = 0, dur = 0, ultimo_cmd = -1, escala, tasa, torque;
  int fase = 0, i = 0, n;
  std::mt19937 rng;
  FILE *f;
  event::ConnectionPtr con;
  ignition::math::Pose3d pose_a;
  double rueda_a[2];
  const double R[7][2] = {{-0.2, 0.4}, {-1.2, 1.2}, {-0.8, 0.8}, {-1.2, 1.2},
                          {-1.5, 1.5}, {-1.2, 1.2}, {-1.5, 1.5}};
  const double V[7] = {0.1, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

 public:
  void Load(physics::ModelPtr _m, sdf::ElementPtr) override {
    m = _m;
    const char *nb[] = {"vertical_lift_joint", "joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6"};
    for (auto s : nb) brazo.push_back(m->GetJoint(s));
    for (auto s : {"right_finger_joint", "left_finger_joint"})
      if (m->GetJoint(s)) dedos.push_back(m->GetJoint(s));
    ruedas = {m->GetJoint("left_wheel_joint"), m->GetJoint("right_wheel_joint")};
    for (auto &j : brazo) if (!j) { gzerr << "falta una articulación\n"; return; }
    escala = env("ESCALA", 1); tasa = env("TASA", 100); torque = env("TORQUE", 5);
    n = (int)env("N", 20); espera = env("ESPERA", 1);
    rng.seed((unsigned)env("SEMILLA", 7));
    const char *sal = getenv("SALIDA");
    f = fopen(sal ? sal : "/tmp/deriva.txt", "w");  // una fila por movimiento
    fprintf(f, "# i dur dx_mm dy_mm dyaw_deg rueda_izq_rad rueda_der_rad\n");
    q0 = q1 = qcmd = std::vector<double>(7, 0.0);
    con = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &u) {
      Paso(u.simTime.Double());
    });
  }

  void Iniciar(double t, const std::vector<double> &q, double d) {
    q0 = qcmd; q1 = q; t_ini = t; dur = d;
  }

  void Paso(double t) {
    // Ruedas: lo mismo que gazebo_ros_diff_drive con /cmd_vel = 0.
    for (auto &r : ruedas) { r->SetParam("fmax", 0, torque); r->SetParam("vel", 0, 0.0); }
    // Orden nueva a la tasa del controlador (trayectoria cúbica, velocidad 0 en los extremos).
    if (t - ultimo_cmd >= 1.0 / tasa - 1e-9) {
      ultimo_cmd = t;
      double s = dur > 0 ? std::min(1.0, std::max(0.0, (t - t_ini) / dur)) : 1.0;
      double h = 3 * s * s - 2 * s * s * s;
      for (int k = 0; k < 7; ++k) qcmd[k] = q0[k] + h * (q1[k] - q0[k]);
    }
    for (int k = 0; k < 7; ++k) { brazo[k]->SetPosition(0, qcmd[k], true);
                                  brazo[k]->SetVelocity(0, 0.0); }
    for (auto &d : dedos) { d->SetPosition(0, 0.0, true); d->SetVelocity(0, 0.0); }

    if (fase == 0 && t > 3.0) { Iniciar(t, std::vector<double>(7, 0.0), 3.0 * escala); fase = 1; }
    else if (fase == 1 && t > t_ini + dur + 2.0) { Guardar(); Nuevo(t); fase = 2; }
    else if (fase == 2 && t > t_ini + dur + espera) {
      auto p = m->WorldPose();
      double yaw = p.Rot().Yaw() - pose_a.Rot().Yaw();
      fprintf(f, "%d %.2f %.3f %.3f %.4f %.5f %.5f\n", i, dur,
              1000 * (p.Pos().X() - pose_a.Pos().X()), 1000 * (p.Pos().Y() - pose_a.Pos().Y()),
              yaw * 180 / M_PI, ruedas[0]->Position(0) - rueda_a[0],
              ruedas[1]->Position(0) - rueda_a[1]);
      fflush(f);
      if (++i >= n) { fprintf(f, "FIN\n"); fclose(f); fase = 3; return; }
      Guardar(); Nuevo(t);
    }
  }

  void Guardar() {
    pose_a = m->WorldPose();
    rueda_a[0] = ruedas[0]->Position(0); rueda_a[1] = ruedas[1]->Position(0);
  }

  void Nuevo(double t) {
    std::vector<double> q(7);
    double d = 1.0;
    for (int k = 0; k < 7; ++k) {
      q[k] = std::uniform_real_distribution<double>(R[k][0], R[k][1])(rng);
      d = std::max(d, std::fabs(q[k] - qcmd[k]) / V[k]);
    }
    Iniciar(t, q, d * escala);
  }
};
GZ_REGISTER_MODEL_PLUGIN(Deriva)
}  // namespace gazebo
