// Ejemplo mínimo: Milo y su arena en el navegador (three.js + urdf-loader), con la misma
// cinemática inversa que arm_ik_node. Explicación completa en ../README.md.
//
// CONVENCIÓN DE EJES: ROS usa +Z arriba y three.js +Y arriba. Todo lo de ROS (robot, arena, mesa,
// objetivo de la IK) va dentro del grupo "mundo", girado -90° en X: adentro se trabaja con las
// coordenadas de ROS tal cual (metros, base_footprint en el origen).
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import URDFLoader from 'urdf-loader';
import { Cadena, poseDesdeCuaternion } from './cinematica.js';

const estado = document.getElementById('estado');

// ---------- Render, cámara del usuario y luces ----------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xdfe3e8);
const mundo = new THREE.Group();
mundo.rotation.x = -Math.PI / 2; // ROS (Z arriba) -> three.js (Y arriba)
scene.add(mundo);

const camara = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.05, 200);
camara.position.set(2.6, 2.4, 2.4);
const orbita = new OrbitControls(camara, renderer.domElement);
orbita.target.set(0.5, 0.9, 0);
orbita.update();

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f96, 1.6));
const sol = new THREE.DirectionalLight(0xffffff, 1.6);
sol.castShadow = true;
sol.shadow.mapSize.set(2048, 2048);
Object.assign(sol.shadow.camera, { left: -4, right: 4, top: 4, bottom: -4, near: 1, far: 40 });

// rgba de ROS (0-1) -> color de three.js
const color = rgba => new THREE.Color().setRGB(rgba[0], rgba[1], rgba[2], THREE.SRGBColorSpace);

// Pose de ROS (xyz + rpy) aplicada a un objeto: rpy = Rz(yaw) Ry(pitch) Rx(roll) = orden 'ZYX'.
function aplicarPose(obj, p) {
  obj.position.set(...p.xyz);
  obj.rotation.set(p.rpy[0], p.rpy[1], p.rpy[2], 'ZYX');
}

// ---------- Escenario (escena.json) ----------
function crearObjeto(o) {
  const g = o.geometria;
  let geo;
  if (g.tipo === 'box') geo = new THREE.BoxGeometry(...g.tamano);
  else if (g.tipo === 'cylinder') {
    // El cilindro de three.js va a lo largo de Y; el de ROS/SDF a lo largo de Z.
    geo = new THREE.CylinderGeometry(g.radio, g.radio, g.largo, 32).rotateX(Math.PI / 2);
  } else return null;
  const m = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: color(o.color), roughness: 0.8 }));
  aplicarPose(m, o.pose);
  m.castShadow = m.receiveShadow = true;
  m.name = o.nombre;
  return m;
}

async function cargarEscena() {
  const esc = await (await fetch('/escena.json')).json();
  const suelo = new THREE.Mesh(new THREE.PlaneGeometry(...esc.suelo.tamano_m),
    new THREE.MeshStandardMaterial({ color: color(esc.suelo.color), roughness: 1 }));
  suelo.receiveShadow = true;
  mundo.add(suelo);
  for (const o of [...esc.arena.objetos, ...esc.mesa_prueba.objetos]) {
    const m = crearObjeto(o);
    if (m) mundo.add(m);
  }
  // El sol de Gazebo: la luz viene desde -dirección.
  const d = esc.luz.direccion;
  sol.position.set(-d[0] * 10, -d[1] * 10, -d[2] * 10);
  mundo.add(sol, sol.target);
}

// ---------- Robot (milo.urdf + robot.json) ----------
const info = await (await fetch('/robot.json')).json();
const cadena = new Cadena(info.cinematica_inversa.cadena, info.cinematica_inversa.parametros);
const limites = Object.fromEntries(info.joints.filter(j => j.limite)
  .map(j => [j.nombre, [j.limite.inferior, j.limite.superior]]));
const q = Object.fromEntries(info.articulaciones_moviles.map(n => [n, 0])); // postura actual

const manager = new THREE.LoadingManager();
const loader = new URDFLoader(manager);
let robot = null;
const robotListo = new Promise(ok => {
  loader.load('/milo.urdf', r => { robot = r; mundo.add(r); });
  manager.onLoad = ok;
});

// Mallas de un link sin entrar a los links hijos (cuelgan de sus joints).
function mallasDe(link) {
  const out = [];
  const recorrer = o => {
    for (const c of o.children) {
      if (c.isURDFJoint || c.isURDFLink) continue;
      if (c.isMesh) out.push(c);
      recorrer(c);
    }
  };
  recorrer(link);
  return out;
}

function pintarRobot() {
  // El URDF solo trae gris y negro: se usan los colores sugeridos de robot.json. Los STL de
  // Fusion traen un color en el encabezado; con un material nuevo se ignora.
  for (const l of info.links) {
    const link = robot.links[l.nombre];
    if (!link || !l.color_sugerido) continue;
    for (const m of mallasDe(link)) {
      m.material = new THREE.MeshStandardMaterial({ color: l.color_sugerido, roughness: 0.55, metalness: 0.1 });
      m.castShadow = m.receiveShadow = true;
    }
  }
}

function fijar(nombre, valor) {
  q[nombre] = valor;
  robot.setJointValue(nombre, valor); // urdf-loader mueve también el dedo "mimic"
  const fila = document.getElementById('art-' + nombre);
  if (fila) {
    fila.querySelector('input').value = valor;
    fila.querySelector('span').textContent = valor.toFixed(nombre.includes('lift') || nombre.includes('finger') ? 3 : 2);
  }
}

function crearDeslizadores() {
  const cont = document.getElementById('articulaciones');
  for (const n of info.articulaciones_moviles) {
    const [lo, hi] = limites[n];
    const fila = document.createElement('div');
    fila.className = 'fila';
    fila.id = 'art-' + n;
    fila.innerHTML = `<label>${n.replace('_joint', '').replace('vertical_lift', 'lift')}</label>
      <input type="range" min="${lo}" max="${hi}" step="0.001" value="0"><span>0</span>`;
    fila.querySelector('input').addEventListener('input', e => {
      animacion = null;
      fijar(n, parseFloat(e.target.value));
      moverObjetivoAlTcp();
    });
    cont.appendChild(fila);
  }
}

// ---------- Objetivo e IK (lo mismo que arm_marker_node + arm_ik_node) ----------
const objetivo = new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16),
  new THREE.MeshStandardMaterial({ color: 0xeb6834, emissive: 0x552200 }));
objetivo.add(new THREE.AxesHelper(0.08));
mundo.add(objetivo);
const mover = new TransformControls(camara, renderer.domElement);
mover.setSize(0.7);
mover.attach(objetivo);
const ayudaMover = mover.getHelper ? mover.getHelper() : mover; // three >= r169 usa getHelper()
scene.add(ayudaMover);
mover.addEventListener('dragging-changed', e => { orbita.enabled = !e.value; });
mover.addEventListener('mouseUp', () => resolverIK());
window.addEventListener('keydown', e => {
  if (e.key === 'w') mover.setMode('translate');
  if (e.key === 'e') mover.setMode('rotate');
});

const qCadena = () => cadena.nombres.map(n => q[n]);

function moverObjetivoAlTcp() {
  const T = cadena.fk(qCadena());
  const m = new THREE.Matrix4().set(...T); // set() recibe la matriz por filas, como cinematica.js
  m.decompose(objetivo.position, objetivo.quaternion, new THREE.Vector3());
}

let animacion = null; // { desde, hasta, nombres, t0, dur }

function animarHacia(nombres, desde, hasta, dur) {
  animacion = { nombres, desde, hasta, t0: performance.now(), dur: dur * 1000 };
}

function resolverIK() {
  const p = objetivo.position, o = objetivo.quaternion;
  const obj = poseDesdeCuaternion([p.x, p.y, p.z], [o.x, o.y, o.z, o.w]);
  const q0 = qCadena();
  const r = cadena.ik(obj, q0);
  if (!r.ok) {
    estado.textContent = `IK sin solución (mejor error ${(r.err[0] * 1000).toFixed(1)} mm, ` +
      `${(r.err[1] * 180 / Math.PI).toFixed(1)}°): fuera de alcance. No me muevo.`;
    return;
  }
  const dur = cadena.duracion(q0, r.q);
  estado.textContent = `IK ok: lift ${r.q[0].toFixed(3)} m, moviendo en ${dur.toFixed(1)} s`;
  animarHacia(cadena.nombres, q0, r.q, dur);
}

document.getElementById('abrir').onclick = () =>
  animarHacia([info.pinza.articulacion], [q[info.pinza.articulacion]], [info.pinza.abierta_m], 1);
document.getElementById('cerrar').onclick = () =>
  animarHacia([info.pinza.articulacion], [q[info.pinza.articulacion]], [info.pinza.cerrada_m], 1);
document.getElementById('mirar-mesa').onclick = () => {
  // La pose de BRAZO.md: pinza sobre el borde de la mesa apuntando 45° hacia abajo.
  objetivo.position.set(0.5, 0, 1.4);
  objetivo.quaternion.set(0.3827, 0, 0.9239, 0);
  resolverIK();
};

// ---------- Cámara de la pinza (Orbbec Gemini Plus) ----------
// Una cámara de three.js en el frame óptico del sensor de color: mira hacia -Z y tiene +Y arriba;
// el frame óptico de ROS mira hacia +Z y tiene +Y abajo -> girar 180° en X.
function camaraDeSensor(s, lejos) {
  const vfov = 2 * Math.atan(Math.tan(s.fov_horizontal_rad / 2) * s.alto_px / s.ancho_px);
  const c = new THREE.PerspectiveCamera(THREE.MathUtils.radToDeg(vfov), s.ancho_px / s.alto_px,
    s.profundidad_min_m || s.cerca_m, lejos);
  c.rotation.x = Math.PI;
  robot.links[s.frame_optico].add(c);
  return c;
}
let camColor = null;
const vista = { ancho: 320, alto: 240 };

// ---------- Arranque ----------
await cargarEscena();
await robotListo;
pintarRobot();
crearDeslizadores();
moverObjetivoAlTcp();
const sensores = Object.fromEntries(info.camara.sensores.map(s => [s.nombre, s]));
camColor = camaraDeSensor(sensores.gripper_camera_color, 20);
// Volumen donde la cámara mide profundidad (0.25 a 2.5 m), dibujado en la vista principal.
const camProf = camaraDeSensor(sensores.gripper_camera_depth, sensores.gripper_camera_depth.profundidad_max_m);
const volumen = new THREE.CameraHelper(camProf);
scene.add(volumen);
estado.textContent = 'Listo. Arrastra la esfera naranja o usa "Mirar la mesa".';

function cuadro(ahora) {
  if (animacion) {
    const a = animacion;
    const t = Math.min(1, (ahora - a.t0) / a.dur);
    const s = t * t * (3 - 2 * t); // arranque y frenado suaves
    a.nombres.forEach((n, i) => fijar(n, a.desde[i] + (a.hasta[i] - a.desde[i]) * s));
    if (t >= 1) { animacion = null; estado.textContent += ' · objetivo alcanzado'; }
  }
  volumen.update();
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, w, h);
  renderer.render(scene, camara);
  // Recuadro: lo que ve la cámara de la pinza (sin el objetivo ni las ayudas).
  objetivo.visible = volumen.visible = ayudaMover.visible = false;
  renderer.setScissorTest(true);
  renderer.setScissor(w - vista.ancho - 12, 12, vista.ancho, vista.alto);
  renderer.setViewport(w - vista.ancho - 12, 12, vista.ancho, vista.alto);
  renderer.render(scene, camColor);
  objetivo.visible = volumen.visible = ayudaMover.visible = true;
  requestAnimationFrame(cuadro);
}
requestAnimationFrame(cuadro);

window.addEventListener('resize', () => {
  camara.aspect = window.innerWidth / window.innerHeight;
  camara.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
