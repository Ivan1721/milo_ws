# Hardware real del brazo de Milo

Documentación de las piezas reales del brazo, la pinza y la cámara, guardada aquí para poder
revisarla sin internet. Lo que viene del fabricante es de sus autores (Toolbox Robotics, Orbbec,
Hiwonder, CUI, LIN Engineering, StepperOnline) y tiene derechos reservados: por eso **git solo
guarda este README** y las carpetas de abajo están en `.gitignore` (existen solo en el PC donde se
descargaron). En otro PC hay que volver a bajarlas desde las fuentes de la tabla.

Descargado el 4 de octubre de 2026. Los datos clave están resumidos abajo y en el informe del
proyecto (sección 5, masas; sección 18, cámara).

## Qué hay en cada carpeta

| Carpeta | Contenido | Fuente |
|---|---|---|
| `brazo_eb300/` | Documento de ensamblaje (58 págs.), guía eléctrica (27 págs.), imagen de la página y actualización de STL (dic. 2023) | <https://toolboxrobotics.com/robotic-arm-eb300> |
| `brazo_eb300/extractos/` | Páginas recortadas de los PDF: qué motor va en cada articulación y la ficha de los motores | los dos PDF de arriba |
| `gripper_ebg20/` | ZIP original del gripper EBG-20: planos de ensamblaje con lista de materiales, STL, STEP, SolidWorks, DWB y `punta brazo.f3d` (modelo propio en Fusion) | Toolbox Robotics |
| `motores/` | Datasheets y modelos 3D de los motores (ver tabla abajo) | archivos propios del proyecto |
| `camara_gemini_plus/web/` | Imágenes de la página de Hubot (ficha técnica, medidas, contenido de la caja) y una captura completa de la página | <https://hubot.cl/producto/camara-3d-orbbec-gemini-plus-para-robotica-y-vehiculos-inteligentes-compatible-ros-y-slam-sku-5012/> |
| `camara_gemini_plus/documentacion_hiwonder/` | Tutoriales de Hiwonder (introducción, configuración ROS 1 y ROS 2) y el driver `OrbbecSDK_ROS2` que usan | ZIP "Depth Camera" de Hiwonder |

**No incluido:** `OrbbecSDK_ROS1.zip` (161 MB). Supera el límite de 100 MB por archivo de GitHub
y Milo usa ROS 2. Queda en `~/Downloads/Depth Camera-20261004T121024Z-1-001.zip`.

**Videos** de la página de la cámara (no se descargan, solo los enlaces):
- <https://www.youtube.com/watch?v=CieseoSrZz4>
- <https://www.youtube.com/watch?v=jpbjC_9eQEI>
- <https://www.youtube.com/watch?v=DfxplVOBLmM>

## Brazo EB300 (Toolbox Robotics)

- 6 ejes, impreso en 3D (PLA+, ABS o PA-CF) con tubos de ABS de 2" y 3" (220 mm).
- Reductores planetarios impresos EBA-17 (para NEMA17) y EBA-23 (para NEMA23).
- Control: Arduino MEGA 2560 + 6 drivers TB6600 (señales STEP/DIR) + fuente 24 V 15 A 360 W.
- Los documentos del fabricante **no traen un modelo Denavit-Hartenberg** ni medidas de los
  eslabones: la cinemática de Milo sale del URDF (ver `docs/BRAZO.md`).

Motor de cada articulación (guía eléctrica, págs. 22-27) y ficha del fabricante (ensamblaje, pág. 4):

| Articulación | Motor | Torque de retención | Corriente | Datasheet en `motores/` | Masa |
|---|---|---|---|---|---|
| J1 base | NEMA23 82 mm | 2.4 N·m | 4.0 A | `nema23_82mm/23HS32-4004S` (coincide: 2.4 N·m, 4.0 A, 82 mm) | no la trae; ≈ 1.1 kg extrapolado |
| J2 hombro | NEMA23 82 mm | 2.4 N·m | 4.0 A | ídem | ≈ 1.1 kg |
| J3 codo | NEMA23 76 mm | 1.85 N·m | 2.8 A | `nema23_76mm/` (solo CAD) | ≈ 1.07 kg (modelo CUI/LIN de 3.10", 2.35 lb) |
| J4 muñeca 1 | NEMA17 60 mm | 0.65 N·m | 2.1 A | `nema17_60mm/NEMA17-AMT112S.PDF` | ≈ 0.41 kg (modelo de 2.34", 0.90 lb) |
| J5 muñeca 2 | NEMA17 60 mm | 0.65 N·m | 2.1 A | ídem | ≈ 0.41 kg |
| J6 muñeca 3 | NEMA17 60 mm | 0.65 N·m | 2.1 A | ídem | ≈ 0.41 kg |
| **Total** | | | | | **≈ 4.5 kg** sin reductores |

Los datasheets `NEMA17-AMT112S` y `NEMA23-AMT112S` (CUI + LIN Engineering) son motores con encoder
AMT112S; sus masas se usan como referencia por largo de cuerpo, no son exactamente los motores del
EB300. Las articulaciones del EB300 se numeran J1…J6 igual que `joint_1…6` en el URDF.

## Gripper EBG-20 (Toolbox Robotics)

- Pinza paralela de piñón y cremalleras, movida por un servo MG99x ("MG99 or +").
- 15 piezas impresas en **ABS**: 156 cm³ de volumen en sus STL → ≈ 0.17 kg macizas. Con el servo y
  la tornillería M3/M4, ≈ 0.23 kg como máximo. En el URDF pesa 1.07 kg porque Fusion le asignó acero.
- Sus mallas coinciden con `link_6_1` y los dedos del URDF (cuerpo 85 × 65 × 52 mm; cada dedo 21.1 cm³).

## Cámara Orbbec Gemini Plus

Ficha técnica (imagen `web/5012-9-481x1024.png`) y medidas (`web/5012-4_medidas_captura.jpg`):

| | |
|---|---|
| Tecnología | luz estructurada binocular activa, chip MX6000 |
| Rango de profundidad | 0.25 – 2.5 m |
| FOV profundidad | 67.9° × 45.3° |
| Profundidad | USB 3.0: 1280×800 @ 30 fps, 640×400 @ 60 fps · USB 2.0: 1280×800 @ 7 fps, 640×400 @ 30 fps |
| Precisión | 5 mm a 1 m |
| RGB | FOV 71° × 56.7° · hasta 1920×1080 @ 30 fps (USB 3.0) |
| Conexión · consumo | USB 3.0 · < 2.2 W · láser clase 1 · 10–40 °C |
| Medidas | 76 mm de ancho, 20 mm de fondo; 68 mm de alto con el soporte. Soporte con bisagra (inclinación ajustable), agujeros Φ4 mm a 30 mm |
| Masa | 0.122 kg (dato del vendedor) |

No hay datasheet oficial de Orbbec ni modelo 3D (STL/STEP) de este modelo.
