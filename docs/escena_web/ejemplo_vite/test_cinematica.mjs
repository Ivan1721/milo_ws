// Prueba de src/cinematica.js contra los valores de la versión en Python (kinematics.py).
// Uso: node test_cinematica.mjs   (lee ros2_ws/escena_web/robot.json: generarlo antes, ver README)
import { readFileSync } from 'node:fs';
import { Cadena, poseDesdeCuaternion } from './src/cinematica.js';

const ruta = new URL('../../../ros2_ws/escena_web/robot.json', import.meta.url);
const robot = JSON.parse(readFileSync(ruta, 'utf8'));
const ik = robot.cinematica_inversa;
const cad = new Cadena(ik.cadena, ik.parametros);
let fallas = 0;
const revisar = (nombre, ok, detalle) => {
  console.log(`${ok ? 'pasa ' : 'FALLA'}  ${nombre}  ${detalle}`);
  if (!ok) fallas++;
};

// 1. Pose cero: el TCP en base_footprint con el lift en 0 es (0.3551, 0, 1.7764) en Python.
const cero = new Array(cad.moviles.length).fill(0);
const T0 = cad.fk(cero);
const e0 = Math.hypot(T0[3] - 0.355077, T0[7], T0[11] - 1.776375);
revisar('pose cero', e0 < 1e-4, `TCP (${T0[3].toFixed(4)}, ${T0[7].toFixed(4)}, ${T0[11].toFixed(4)})`);

// 2. Jacobiano lineal contra diferencias finitas en una postura cualquiera.
const q = [0.2, 0.3, -0.5, 0.8, 0.4, -0.6, 1.1];
const J = cad.jacobiano(q);
let maxDif = 0;
for (let c = 0; c < q.length; c++) {
  const h = 1e-6;
  const qa = [...q]; qa[c] += h;
  const qb = [...q]; qb[c] -= h;
  const A = cad.fk(qa), B = cad.fk(qb);
  [3, 7, 11].forEach((k, r) => { maxDif = Math.max(maxDif, Math.abs((A[k] - B[k]) / (2 * h) - J[r][c])); });
}
revisar('jacobiano', maxDif < 1e-6, `diferencia máx. ${maxDif.toExponential(2)}`);

// 3. IK: 20 poses alcanzables (FK de posturas al azar), resueltas desde la pose cero.
let resueltas = 0, peor = 0;
for (let n = 0; n < 20; n++) {
  const qr = cad.moviles.map((j, i) => {
    const [lo, hi] = j.limite || [-Math.PI, Math.PI];
    return lo + ((Math.sin(17 * n + 3 * i) + 1) / 2) * (hi - lo);
  });
  const r = cad.ik(cad.fk(qr), cero, { semilla: n + 1 });
  if (r.ok) { resueltas++; peor = Math.max(peor, r.err[0]); }
}
revisar('IK de 20 poses al azar', resueltas >= 19, `${resueltas}/20, peor error ${(peor * 1000).toFixed(3)} mm`);

// 4. La pose de prueba del informe, que en Gazebo dejó el lift en +0.19 m.
const obj = poseDesdeCuaternion([0.5048, -0.2163, 1.862], [-0.1252, 0.5501, -0.7869, 0.2502]);
const r = cad.ik(obj, cero);
revisar('pose del informe', r.ok, `lift ${r.q[0].toFixed(3)} m, error ${(r.err[0] * 1000).toFixed(3)} mm`);

process.exit(fallas ? 1 : 0);
