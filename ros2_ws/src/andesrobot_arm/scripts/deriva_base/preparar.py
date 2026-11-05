"""
Arma el mundo de Gazebo para medir la deriva de la base (lo llama correr.sh, dentro del
contenedor de deriva_base; ver README.md de esta carpeta).

Qué hace:
  1. Procesa el xacro de Milo con los mismos argumentos que arm_sim.launch.py (brazo libre,
     mallas como colisión), pero sin cámara (sus sensores necesitan render) y sin ros2_control
     (arm_control:=false): al brazo y a las ruedas los mueve el plugin deriva.cc, que imita lo que
     hacen gazebo_ros2_control y gazebo_ros_diff_drive.
  2. Borra los plugins de ROS (en este contenedor no existen) y agrega el plugin de medición.
  3. Lo pasa a SDF con `gz sdf -p` y lo mete en un mundo con el mismo paso de física que la arena
     (ODE, 1 ms), pero corriendo tan rápido como se pueda (real_time_update_rate 0).

Variantes, por variables de entorno (sin ellas, todo queda como en el repo):
  CASTER_MU     fricción de la rueda loca (en el repo 0.0)
  MU_RUEDA      fricción de las ruedas motrices (en el repo 1.2)
  FRICCION_EJE  fricción en el eje de las ruedas, <dynamics friction> (en el repo 0.0)
  ITERS         iteraciones del solver de ODE (si no se da, Gazebo usa 50)
  SOLVER        quick (el de Gazebo por defecto) o world (exacto y lento)
"""
import os
import re
import subprocess
import sys

sys.path.insert(0, '/cache/xacro_src')   # xacro es Python puro: se clona de GitHub
import xacro  # noqa: E402

# xacro resuelve $(find andesrobot_description) con el índice de ament: se arma uno mínimo.
PREFIJO = '/tmp/prefijo'
os.makedirs(f'{PREFIJO}/share/ament_index/resource_index/packages', exist_ok=True)
open(f'{PREFIJO}/share/ament_index/resource_index/packages/andesrobot_description', 'w').close()
if not os.path.exists(f'{PREFIJO}/share/andesrobot_description'):
    os.symlink('/src/andesrobot_description', f'{PREFIJO}/share/andesrobot_description')
os.environ['AMENT_PREFIX_PATH'] = PREFIJO

urdf = xacro.process_file('/src/andesrobot_description/urdf/andesrobot.urdf.xacro', mappings={
    'lock_arm': 'false', 'arm_control': 'false', 'simple_collision': 'false',
    'gripper_camera': 'false'}).toxml()
urdf = re.sub(r'<!--.*?-->', '', urdf, flags=re.S)
urdf = re.sub(r'<gazebo>\s*<plugin[^>]*libgazebo_ros[^>]*>.*?</plugin>\s*</gazebo>', '', urdf,
              flags=re.S)


def cambiar(patron, nuevo, veces):
    """Reemplaza y comprueba que el patrón apareció las veces esperadas (si el xacro cambia,
    mejor fallar que medir otra cosa sin darse cuenta)."""
    global urdf
    urdf, k = re.subn(patron, nuevo, urdf, flags=re.S)
    assert k == veces, f'{patron}: {k} en vez de {veces}'


if os.environ.get('CASTER_MU'):
    mu = os.environ['CASTER_MU']
    cambiar(r'(<gazebo reference="front_caster_link_1">\s*<mu1>)0\.0(</mu1>\s*<mu2>)0\.0',
            rf'\g<1>{mu}\g<2>{mu}', 1)
if os.environ.get('MU_RUEDA'):
    mu = os.environ['MU_RUEDA']
    cambiar(r'(<gazebo reference="(?:left|right)_wheel_link_1">\s*<mu1>)1\.2(</mu1>\s*<mu2>)1\.2',
            rf'\g<1>{mu}\g<2>{mu}', 2)
if os.environ.get('FRICCION_EJE'):
    cambiar(r'(<joint name="(?:left|right)_wheel_joint" type="continuous">.*?'
            r'<dynamics damping="[\d.]+" friction=")0\.0"',
            rf'\g<1>{os.environ["FRICCION_EJE"]}"', 2)

# Gazebo no conoce package://: las mallas se leen directo del código montado en /src.
urdf = urdf.replace('package://andesrobot_description/', 'file:///src/andesrobot_description/')
urdf = urdf.replace('</robot>', '<gazebo><plugin name="deriva" '
                    'filename="/cache/build/libderiva.so"/></gazebo></robot>')
open('/tmp/robot.urdf', 'w').write(urdf)

sdf = subprocess.run(['gz', 'sdf', '-p', '/tmp/robot.urdf'], capture_output=True, text=True,
                     check=True).stdout
modelo = re.search(r'<model .*</model>', sdf, flags=re.S).group(0)
# 2 cm sobre el suelo, como el spawn de arm_sim (-z 0.02).
modelo = modelo.replace("<model name='andesrobot'>",
                        "<model name='andesrobot'><pose>0 0 0.02 0 0 0</pose>", 1)
iters = os.environ.get('ITERS')
solver = os.environ.get('SOLVER', 'quick')
ode = f'<ode><solver><type>{solver}</type><iters>{iters}</iters></solver></ode>' if iters else ''
mundo = f'''<?xml version="1.0"?><sdf version="1.6"><world name="deriva">
<include><uri>model://ground_plane</uri></include>
<physics type="ode"><max_step_size>0.001</max_step_size>{ode}
<real_time_factor>0</real_time_factor><real_time_update_rate>0</real_time_update_rate></physics>
{modelo}</world></sdf>'''
open('/tmp/mundo.sdf', 'w').write(mundo)
masa = sum(float(x) for x in re.findall(r'<mass>([\d.e+-]+)</mass>', modelo))
print(f'masa total del modelo: {masa:.3f} kg')
