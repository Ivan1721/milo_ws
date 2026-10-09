// Configuración de Vite. publicDir = carpeta cuyos archivos se sirven tal cual en la raíz del
// sitio: apunta al paquete que genera exportar_escena_web.py (milo.urdf, meshes/, robot.json,
// escena.json), así no hay que copiarlo. En un proyecto propio, copia esa carpeta a public/milo
// y cambia las rutas de main.js.
import { defineConfig } from 'vite';

export default defineConfig({
  publicDir: '../../../ros2_ws/escena_web',
  server: { host: true },
  // main.js usa await fuera de funciones (top-level await): necesita ES2022.
  build: { target: 'es2022' },
});
