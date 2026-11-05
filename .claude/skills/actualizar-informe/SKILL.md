---
name: actualizar-informe
description: Después de un avance en el brazo de Milo (código, prueba en Gazebo, medida del robot real, decisión), revisar qué documentos de milo_ws quedaron desactualizados y corregirlos con las convenciones del repo, gastando pocos tokens. Usar al cerrar un pendiente de docs/PENDIENTES.md, cuando se pida actualizar el informe o la documentación, o antes de abrir un PR.
---

# Actualizar la documentación de milo_ws

Objetivo: dejar los documentos coherentes con el código **sin leerlos enteros**. El informe
(`docs/informe/README.md`) tiene ~24 000 palabras: se busca con `grep -n` la sección o la tabla que
toca y se lee solo eso.

## 1. Qué cambió
- `git diff --stat origin/brazo...HEAD` y los mensajes de los commits nuevos.
- El pendiente que se trabajó en `docs/PENDIENTES.md` (campo «Dónde mirar»).

## 2. Qué documento toca cada cambio
| Si cambió… | Revisar |
| --- | --- |
| cualquier cosa del brazo | `docs/PENDIENTES.md`: marcar lo hecho, dejar lo que falta y por qué; tachar en la tabla si se cerró |
| una prueba (`def test_`) agregada o borrada | informe §21 («**N de N** en `X`») y la lista de pruebas del CI (`.github/workflows/ci.yml` y `CLAUDE.md`) |
| URDF / xacro / masas / límites | informe §4–§6 y §23; `andesrobot_description/docs/ROBOT.md`; regenerar figuras con `scripts/figuras.py` |
| MoveIt, controladores, launch del brazo | informe §13 y la Figura C (`scripts/fig_lazo_moveit.py`); `andesrobot_arm/docs/BRAZO.md` |
| cámara, puerta, detector | informe §14; `BRAZO.md` |
| comandos de `sim.sh` / `robot.sh` | informe §24, `README.md`, la ayuda del propio script (rango del `sed -n`) |
| un hecho nuevo que una sesión futura necesita | `CLAUDE.md`, en inglés y corto |
| un commit importante | informe §1, tabla «Estado del repositorio» |

## 3. Convenciones
- Español, para estudiantes; misma densidad de comentarios que el código vecino.
- Cada sección del informe empieza con «En palabras simples». Las figuras numeradas van 1, 2, 3… en
  orden de aparición; fórmulas con `\mathrm`, nunca la macro operatorname (GitHub no la muestra).
- Lo no probado se dice: «sin probar en la simulación». No inventar cifras: si no se midió, se anota
  como pendiente.
- `milo_ws` es público: nada de los repos privados (Next-Best-View, ActivePerceptionArms).

## 4. Comprobar (lo mismo que corre el CI)
```bash
python3 ros2_ws/src/andesrobot_arm/scripts/revisar_docs.py
python3 -m flake8 --max-line-length=99 ros2_ws/src/andesrobot_arm ros2_ws/src/andesrobot_description ros2_ws/src/andesrobot_moveit_config
(cd ros2_ws/src/andesrobot_arm && python3 -m pytest -q test/test_torques.py test/test_medidas.py test/test_revisar_docs.py test/test_moveit_opciones.py test/test_manija.py)
(cd ros2_ws/src/andesrobot_safety && python3 -m pytest -q test)
```

## 5. Al terminar
- Decir qué documentos se tocaron y qué quedó sin verificar.
- El artifact de claude.ai «Informe técnico de Milo» está congelado en el 19-oct-2026 con un
  aviso que apunta a `docs/informe/README.md`: no se sincroniza; la versión vigente es la de GitHub.
