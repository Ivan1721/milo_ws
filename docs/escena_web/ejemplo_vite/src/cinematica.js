// Cinemática directa e inversa del brazo de Milo en JavaScript (sin dependencias).
//
// Es una traducción de andesrobot_arm/andesrobot_arm/kinematics.py: mismas fórmulas, mismos
// parámetros, para que la escena web mueva el brazo igual que arm_ik_node en Gazebo.
// La cadena (origen, eje y límites de cada articulación) viene de robot.json
// (cinematica_inversa.cadena), que genera exportar_escena_web.py desde el URDF.
//
// Convenciones (las de ROS): metros, radianes, +Z arriba, todo expresado en base_footprint.
// Las matrices 4x4 son arreglos de 16 números por filas: m[fila * 4 + columna].

// ---------- Matrices ----------
export function identidad() {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

export function multiplicar(a, b) {
  const r = new Array(16).fill(0);
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 4; k++) r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
  return r;
}

// R = Rz(yaw) Ry(pitch) Rx(roll): la convención del <origin rpy> del URDF.
export function rpyAMatriz(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

// Pose 4x4 a partir de posición [x, y, z] y rpy [roll, pitch, yaw].
export function pose(xyz, rpy = [0, 0, 0]) {
  const R = rpyAMatriz(...rpy);
  return [R[0], R[1], R[2], xyz[0], R[3], R[4], R[5], xyz[1], R[6], R[7], R[8], xyz[2], 0, 0, 0, 1];
}

// Fórmula de Rodrigues: rotación de 'angulo' alrededor del eje unitario 'k' (3x3 por filas).
function ejeAngulo(k, angulo) {
  const [x, y, z] = k;
  const s = Math.sin(angulo), c = 1 - Math.cos(angulo);
  const K = [0, -z, y, z, 0, -x, -y, x, 0];
  const KK = [
    -(y * y + z * z), x * y, x * z,
    x * y, -(x * x + z * z), y * z,
    x * z, y * z, -(x * x + y * y),
  ];
  return [1, 0, 0, 0, 1, 0, 0, 0, 1].map((v, i) => v + s * K[i] + c * KK[i]);
}

// Cuaternión (x, y, z, w) -> matriz 3x3 por filas.
export function cuaternionAMatriz(x, y, z, w) {
  const n = Math.hypot(x, y, z, w);
  x /= n; y /= n; z /= n; w /= n;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

// Pose 4x4 a partir de posición y cuaternión (lo que entrega three.js o un PoseStamped).
export function poseDesdeCuaternion(xyz, q) {
  const R = cuaternionAMatriz(q[0], q[1], q[2], q[3]);
  return [R[0], R[1], R[2], xyz[0], R[3], R[4], R[5], xyz[1], R[6], R[7], R[8], xyz[2], 0, 0, 0, 1];
}

// Vector de rotación (eje * ángulo) de una matriz 3x3 por filas.
function logRotacion(R) {
  const ang = Math.acos(Math.min(1, Math.max(-1, (R[0] + R[4] + R[8] - 1) / 2)));
  if (ang < 1e-9) return [0, 0, 0];
  if (Math.PI - ang < 1e-6) {
    // Cerca de pi la parte antisimétrica se anula: el eje sale de la parte simétrica.
    const diag = [R[0], R[4], R[8]].map(d => Math.sqrt(Math.max(0, (d + 1) / 2)));
    const i = diag.indexOf(Math.max(...diag));
    for (let j = 0; j < 3; j++)
      if (j !== i) diag[j] = Math.sign(R[i * 3 + j] + R[j * 3 + i] || 1) * Math.abs(diag[j]);
    const n = Math.hypot(...diag);
    return diag.map(v => (ang * v) / n);
  }
  const f = ang / (2 * Math.sin(ang));
  return [(R[7] - R[5]) * f, (R[2] - R[6]) * f, (R[3] - R[1]) * f];
}

function rot3(m) { return [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]; }
function mul3(a, b) {
  const r = new Array(9).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) r[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
  return r;
}
function trasp3(a) { return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]]; }

// Resuelve A x = b (A n x n) por eliminación de Gauss con pivoteo.
function resolver(A, b) {
  const n = b.length;
  const M = A.map((fila, i) => [...fila, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

// Generador pseudoaleatorio con semilla (mulberry32): mismos reinicios en cada corrida.
function aleatorio(semilla) {
  let a = semilla >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ---------- La cadena ----------
export class Cadena {
  // cadena = robot.cinematica_inversa.cadena; parametros = robot.cinematica_inversa.parametros
  constructor(cadena, parametros = {}) {
    this.cadena = cadena.map(j => {
      const n = Math.hypot(...j.eje);
      return { ...j, eje: j.eje.map(v => v / n), M0: pose(j.origen.xyz, j.origen.rpy) };
    });
    this.moviles = this.cadena.filter(j => j.tipo !== 'fixed');
    this.nombres = this.moviles.map(j => j.nombre);
    this.p = { lambda: 0.01, pasoMax: 0.5, iteraciones: 200, reinicios: 20, tolPos: 1e-4,
      tolRot: 1e-3, pesoLift: 10, ...renombrar(parametros) };
    // Peso de cada articulación: el lift "cuesta" más, así la IK prefiere mover el brazo.
    this.pesos = this.moviles.map(j => (j.tipo === 'prismatic' ? this.p.pesoLift : 1));
  }

  // Origin de la articulación por su movimiento (giro en el eje o traslación a lo largo de él).
  _transformada(j, q) {
    const mov = identidad();
    if (j.tipo === 'revolute' || j.tipo === 'continuous') {
      const R = ejeAngulo(j.eje, q);
      for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) mov[i * 4 + k] = R[i * 3 + k];
    } else if (j.tipo === 'prismatic') {
      mov[3] = j.eje[0] * q; mov[7] = j.eje[1] * q; mov[11] = j.eje[2] * q;
    }
    return multiplicar(j.M0, mov);
  }

  // Frame de cada articulación móvil (antes de su movimiento) y pose del TCP.
  frames(q) {
    let T = identidad();
    const fr = [];
    let i = 0;
    for (const j of this.cadena) {
      if (j.tipo === 'fixed') { T = multiplicar(T, j.M0); continue; }
      fr.push(multiplicar(T, j.M0));
      T = multiplicar(T, this._transformada(j, q[i]));
      i++;
    }
    return { frames: fr, tcp: T };
  }

  // Cinemática directa: pose 4x4 del TCP (gripper_tcp) en base_footprint.
  fk(q) { return this.frames(q).tcp; }

  // Jacobiano geométrico 6 x N: filas 0-2 lineal, 3-5 angular.
  jacobiano(q) {
    const { frames, tcp } = this.frames(q);
    const J = Array.from({ length: 6 }, () => new Array(this.moviles.length).fill(0));
    const pe = [tcp[3], tcp[7], tcp[11]];
    this.moviles.forEach((j, c) => {
      const F = frames[c];
      const z = [0, 1, 2].map(r => F[r * 4] * j.eje[0] + F[r * 4 + 1] * j.eje[1] + F[r * 4 + 2] * j.eje[2]);
      if (j.tipo === 'prismatic') {
        for (let r = 0; r < 3; r++) J[r][c] = z[r];
      } else {
        const d = [pe[0] - F[3], pe[1] - F[7], pe[2] - F[11]];
        J[0][c] = z[1] * d[2] - z[2] * d[1];
        J[1][c] = z[2] * d[0] - z[0] * d[2];
        J[2][c] = z[0] * d[1] - z[1] * d[0];
        for (let r = 0; r < 3; r++) J[3 + r][c] = z[r];
      }
    });
    return J;
  }

  // Límites: los rangos de vuelta completa (±pi provisorios) dan la vuelta en vez de recortarse.
  _limitar(q) {
    return q.map((v, i) => {
      const lim = this.moviles[i].limite;
      if (!lim) return v;
      const [lo, hi] = lim;
      if (this.moviles[i].tipo === 'revolute' && hi - lo >= 2 * Math.PI - 1e-6) {
        const r = (v - lo) % (2 * Math.PI);
        return (r < 0 ? r + 2 * Math.PI : r) + lo;
      }
      return Math.min(hi, Math.max(lo, v));
    });
  }

  _intento(objetivo, q, soloPosicion) {
    const filas = soloPosicion ? 3 : 6;
    const inv = this.pesos.map(w => 1 / w);
    const lam2 = this.p.lambda ** 2;
    q = this._limitar(q);
    let err = [Infinity, Infinity];
    for (let it = 0; it < this.p.iteraciones; it++) {
      const T = this.fk(q);
      const ep = [objetivo[3] - T[3], objetivo[7] - T[7], objetivo[11] - T[11]];
      const er = logRotacion(mul3(rot3(objetivo), trasp3(rot3(T))));
      err = [Math.hypot(...ep), soloPosicion ? 0 : Math.hypot(...er)];
      if (err[0] < this.p.tolPos && err[1] < this.p.tolRot) return { q, ok: true, err };
      const e = [...ep, ...er].slice(0, filas);
      const J = this.jacobiano(q).slice(0, filas);
      // DLS ponderado: dq = W^-1 J^T (J W^-1 J^T + lambda^2 I)^-1 e
      const A = J.map((fi, a) => J.map((fj, b) =>
        fi.reduce((s, v, k) => s + v * inv[k] * fj[k], 0) + (a === b ? lam2 : 0)));
      const y = resolver(A, e);
      let dq = q.map((_, k) => inv[k] * J.reduce((s, fila, r) => s + fila[k] * y[r], 0));
      const norma = Math.hypot(...dq);
      if (norma > this.p.pasoMax) dq = dq.map(v => (v * this.p.pasoMax) / norma);
      q = this._limitar(q.map((v, k) => v + dq[k]));
    }
    return { q, ok: false, err };
  }

  // Cinemática inversa: objetivo = pose 4x4 del TCP en base_footprint; q0 = postura actual.
  // Devuelve { q, ok, err: [error_posición_m, error_rotación_rad] }. Si no converge devuelve
  // el mejor intento con ok = false: revisar siempre ok.
  ik(objetivo, q0, { soloPosicion = false, semilla = 1 } = {}) {
    const rnd = aleatorio(semilla);
    let mejor = null;
    for (let intento = 0; intento <= this.p.reinicios; intento++) {
      const inicio = intento === 0 ? q0 : this.moviles.map(j => {
        const [lo, hi] = j.limite || [-Math.PI, Math.PI];
        return lo + rnd() * (hi - lo);
      });
      const r = this._intento(objetivo, inicio, soloPosicion);
      if (r.ok) return r;
      if (!mejor || r.err[0] + 0.1 * r.err[1] < mejor.err[0] + 0.1 * mejor.err[1]) mejor = r;
    }
    return mejor;
  }

  // Duración del movimiento como arm_ik_node: la articulación más lenta manda.
  duracion(q0, q, { velBrazo = 0.5, velLift = 0.1, minimo = 1.0 } = {}) {
    let t = minimo;
    this.moviles.forEach((j, i) => {
      const v = j.tipo === 'prismatic' ? velLift : velBrazo;
      t = Math.max(t, Math.abs(q[i] - q0[i]) / v);
    });
    return t;
  }
}

// Nombres de robot.json (en español, con unidades) -> nombres cortos de esta clase.
function renombrar(p) {
  const m = {};
  if (p.amortiguamiento_lambda !== undefined) m.lambda = p.amortiguamiento_lambda;
  if (p.paso_maximo !== undefined) m.pasoMax = p.paso_maximo;
  if (p.iteraciones_por_intento !== undefined) m.iteraciones = p.iteraciones_por_intento;
  if (p.reinicios !== undefined) m.reinicios = p.reinicios;
  if (p.tolerancia_posicion_m !== undefined) m.tolPos = p.tolerancia_posicion_m;
  if (p.tolerancia_rotacion_rad !== undefined) m.tolRot = p.tolerancia_rotacion_rad;
  if (p.peso_lift !== undefined) m.pesoLift = p.peso_lift;
  return m;
}
