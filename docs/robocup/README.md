# RoboCup@Home 2027: escenario y objetos para la pinza de Milo

Qué pide la llamada a participación de RoboCup@Home 2027, cómo es el escenario según el reglamento y
**qué objetos puede tomar la pinza de Milo** (EBG-20) tal como está hoy. Sirve para decidir qué
mostrar en el video de calificación y qué cambiar en la pinza. Se limita al brazo, el lift y la
pinza.

> **En palabras simples:** la competencia es en una casa armada dentro de un pabellón, con muebles
> y objetos de uso diario. Muchos de esos objetos son más anchos de lo que abre nuestra pinza
> (49 mm). Este documento mide cada objeto y dice cuáles caben, cuáles solo se pueden tomar por una
> parte (un asa, un borde, un mango) y cuáles no caben. Los cubiertos, plumones y herramientas
> finas caben; tres de los cuatro objetos "comunes" de la competencia (lata de Red Bull, tarro de
> Pringles y cubo Rubik) no.

Revisado el 11 de noviembre de 2026. Fuentes al final.

## Contenido

1. [La llamada a participación 2027](#1-la-llamada-a-participación-2027)
2. [El escenario](#2-el-escenario)
3. [Los objetos del reglamento](#3-los-objetos-del-reglamento)
4. [Las pruebas y su escenario](#4-las-pruebas-y-su-escenario)
5. [Qué puede tomar la pinza](#5-qué-puede-tomar-la-pinza)
6. [Qué falta para el resto](#6-qué-falta-para-el-resto)
7. [Medidas de los objetos YCB](#7-medidas-de-los-objetos-ycb)
8. [Fuentes](#8-fuentes)

## 1. La llamada a participación 2027

| Fecha (23:59 GMT) | Qué |
| --- | --- |
| 30-oct-2026 | intención de participar (recomendada) |
| **6-dic-2026** | **material de calificación (obligatorio)** |
| 23-dic-2026 | anuncio de los equipos calificados |
| 22-ene-2027 | confirmación de participación (obligatoria) |
| 15 al 21-jun-2027 | competencia en Núremberg, Alemania |

- **Una sola liga**, la *Open Platform League* (cada equipo trae su robot). Ya no hay ligas de
  plataforma estándar.
- **Video del equipo:** máximo 10 minutos, debe mostrar **al menos 5 habilidades integradas**
  (ejemplos que da la llamada: razonamiento sobre el entorno, planificación de tareas, interacción
  con personas, localización, **manipulación**, **reconocimiento de objetos**, reconocimiento de
  personas), en una tarea continua y robusta. Se sugiere mostrar pruebas del reglamento 2026.
- **TDP** (*Team Description Paper*): máximo 8 páginas en formato Springer LNAI, más un anexo de 2
  (fotos del robot, software de terceros, hardware).
- **Página web** del equipo: fotos, videos, publicaciones, repositorios e integrantes.
- Habrá una segunda llamada a comienzos de 2027 para quienes no alcancen.

## 2. El escenario

Del capítulo *Scenario* del reglamento (versión 2026.2, la última publicada):

- **Departamento** con al menos living, cocina y dos piezas más (dormitorio, oficina, comedor,
  lavandería…), parecido a una casa normal del país sede, con decoración (plantas, cuadros, libros,
  velas).
- **Puertas con manilla, no con pomo**; pueden estar cerradas y se espera que el robot las abra (es
  lo que trabaja `manija_node`). Ancho recomendado 915 mm.
- **Piso** parejo, sin escalones (puede haber alfombras y juntas).
- **Muebles mínimos:** sillón, mesa de centro, mesa de comedor con dos sillas, basurero,
  **estantería con repisas a 30 cm o más**, lavadora de carga frontal, lavavajillas y despensa
  (mueble alto con repisas a 30 cm o más).
- Los muebles se pueden correr un poco entre pruebas (la mesa del comedor, hasta 50 cm).
- **Robot:** tiene que pasar por una puerta: 200 × 70 cm como máximo.

## 3. Los objetos del reglamento

- El comité arma una lista de **al menos 30 objetos**, cada uno con foto, nombre y categoría (por
  ejemplo *Apple* en *Fruits*). "La mayoría son livianos y fáciles de tomar con una mano."
- De dónde salen: **objetos comunes** (los mismos en todo el mundo), **objetos del set YCB** (un
  set de objetos estándar para investigación en manipulación, con modelos 3D) y **objetos locales**
  comprados en el país sede. También pueden pedir objetos *parecidos* a los de la lista y objetos
  desconocidos (decoración).
- Se pueden pedir por categoría o por atributo: color, tamaño relativo ("el más grande"), peso
  relativo ("el más liviano"), posición ("el de la izquierda") o descripción ("frágil", "se puede
  vaciar", "necesita dos manos").

**Lista mínima** que exige el reglamento:

| Tipo | Ejemplos |
| --- | --- |
| Vajilla | plato, bowl, taza o tazón, servilleta |
| Cubiertos | tenedor, cuchillo, cuchara |
| Bolsa | liviana, con asas rígidas verticales |
| Bandeja | bandeja o canasto, **para dos manos** |
| Para vaciar | por ejemplo una caja de cereal |
| Pesado | **entre 1.0 y 1.5 kg** |
| Diminuto | hasta 5 cm (una frutilla, un dado) |
| Frágil | por ejemplo un huevo de chocolate |
| Deformable | por ejemplo un paño |
| Ropa | una polera para doblar |
| Canasto de ropa | ancho y liviano, **para dos manos** |
| **Objetos comunes** | **lata de Red Bull, tarro rojo de Pringles, cubo Rubik resuelto, caja de pasta dental Colgate** |

El modelo de objetos del equipo (`ros2_ws/modelos/objetos/best.pt`, 27 clases) ya tiene clases para
estos objetos (cup, bowl, plate, fork, knife, spoon, bag, tray, cereal_box, redbull_can, pringles,
rubik_cube, colgate_box…).

## 4. Las pruebas y su escenario

Además del escenario común (sección 2), cada prueba define su propio escenario: en qué pieza
ocurre, qué muebles, objetos y personas hay, y qué se le pide al robot. La competencia tiene
varias pruebas y después una **final** para los mejores equipos; aparte hay un **desafío abierto**
de demostración. Puntajes de las hojas de puntaje del reglamento (`scoresheets/`).

| Prueba | Dónde | Qué le pide al brazo y la pinza | ¿Milo hoy? |
| --- | --- | --- | --- |
| [Pick and Place](#41-pick-and-place) | cocina: mesa del comedor, lavavajillas, estante, basurero | tomar y dejar cubiertos, vajilla, basura, 2 objetos comunes, desayuno (bowl, cuchara, cereal, leche); opcional abrir el lavavajillas y vaciar | en parte: cubiertos sí; vajilla por el borde y 3 de 4 objetos comunes no |
| [Interacción con personas](#42-interacción-con-personas-recepcionista) | living y entrada | recibir una **bolsa** de la mano de un invitado y dejarla en el suelo; opcional abrir la puerta de entrada | posible si el asa de la bolsa cabe (≤ 45 mm) y el brazo aguanta su peso |
| [Lavar ropa](#43-lavar-ropa) | lavandería: lavadora, mesa, canasto | sacar poleras del canasto o la lavadora y **doblarlas**; opcional abrir la lavadora | tomar una polera sí; doblar con un brazo, muy difícil; el canasto es para dos manos |
| [Robot de servicio general (GPSR)](#44-robot-de-servicio-general-gpsr) | toda la casa | 3 órdenes habladas al azar: muchas son "trae tal objeto de tal lugar" | lo que se pueda tomar (sección 5) |
| [Final](#45-final) | toda la casa | encontrar y resolver problemas: basura en el suelo, objetos fuera de lugar, cerrar el lavavajillas, mover el canasto, **abrir la puerta** a un invitado | basura chica y puerta (manilla) son lo más cercano |
| [Desafío abierto](#46-desafío-abierto) | la casa, se puede reordenar | lo que el equipo quiera mostrar (8 min + 2 de preguntas) | buen lugar para la apertura de puertas o el agarre con cámara |

### 4.1 Pick and Place

La prueba de manipulación, en la cocina:

1. **Ordenar la mesa del comedor**, que tiene 6 objetos, a veces apilados: un cubierto, una taza,
   un plato, un objeto que cuenta como basura y dos objetos más. Cubiertos y vajilla van al
   **lavavajillas**, la basura al **basurero** y lo demás a un **estante sin puertas**, junto a los
   objetos de su misma categoría.
2. **Servir desayuno:** bowl, cuchara, cereal y leche en la mesa (la cuchara al lado del bowl). El
   bowl y la cuchara están en una superficie de la cocina; la leche (cerrada) y el cereal
   (abierto), en el estante.
3. **Superficie extra:** **2 objetos comunes** (Red Bull, Pringles, Rubik o Colgate) que hay que
   guardar en el estante.
4. **Suelo:** una basura junto al basurero.
5. Opcionales: abrir y cerrar el lavavajillas (200 c/u), sacar y meter su bandeja (100), poner la
   pastilla del lavavajillas, vaciar leche y cereal en el bowl (200 c/u), abrir la leche (400).

**Puntaje del brazo:** reconocer un objeto 10 (hasta 12 veces), **tomar un objeto 50** (hasta 12),
**dejarlo en su lugar 40** (hasta 12), más una bonificación por el primer objeto tomado. Restan:
dejar caer o lanzar un objeto (40) y que una persona entregue un objeto en la mano (100). La mesa
se puede correr hasta 50 cm; la superficie extra, hasta 5 cm. Se puede pedir ayuda para abrir el
lavavajillas o la leche sin perder puntos.

### 4.2 Interacción con personas (recepcionista)

Milo es el recepcionista de una fiesta: recibe a dos invitados en la puerta (opcional: oír el
timbre y **abrir la puerta**, 200 puntos por invitado), les pregunta el nombre y su bebida
favorita, los lleva al living, les ofrece asiento y los presenta mirándolos. **El segundo invitado
trae una bolsa:** el robot la recibe de su mano (**50 puntos**), sigue al dueño de casa por la casa
(200) y la deja en el suelo donde le indique (50).

Para la entrega de la bolsa el reglamento pide: que el robot avise que está listo y deje la pinza
quieta **a menos de 10 cm de la mano** de la persona; que sea la persona quien complete la entrega
(el robot no debe estirarse hacia ella); y que el robot avise antes de cerrar ("cierro la pinza en
3 segundos"). Si el robot le pide al invitado que deje la bolsa sobre su estructura, no hay puntos.
La bolsa se anuncia en los días de preparación; según la lista mínima, es liviana y con asas
rígidas verticales: hay que ver si su asa cabe en la pinza y cuánto pesa.

### 4.3 Lavar ropa

En la lavandería hay una lavadora de carga frontal (cerrada, con 2–4 poleras dentro), un canasto
con 6–8 poleras y una mesa con una polera arrugada. El robot lleva ropa a la mesa y **dobla al
menos una** (800 puntos; tomar una polera del canasto, 100). Opcionales: abrir la lavadora,
sacar ropa de la lavadora y usar el canasto para llevarla (300 c/u), doblar más poleras y
apilarlas. El reglamento la define como prueba de **objetos deformables y agarre con dos manos**:
con un solo brazo y una pinza de 49 mm, tomar una polera es posible, doblarla bien no.

### 4.4 Robot de servicio general (GPSR)

El robot parte fuera de la casa, va a un punto de instrucciones y un árbitro le da **3 órdenes
habladas** generadas al azar por el [generador oficial](https://github.com/RoboCupAtHome/CommandGenerator)
y reescritas por un modelo de lenguaje (por ejemplo, traer un objeto de un lugar a una persona).
Puntos por entender cada orden (80), mostrar un plan (100) y cumplirla (250 c/u), más 200 si las
intercala. Para el brazo, es tomar y entregar objetos de la lista en cualquier mueble de la casa.

### 4.5 Final

Para los mejores equipos, en una sola casa: la casa está en su estado normal salvo **al menos 8
problemas** que el robot tiene que encontrar y resolver solo, sin repetir el tipo de problema:
basura en el suelo al basurero, objetos fuera de su lugar de vuelta a su lugar, personas que
levantan la mano con un pedido, **cerrar el lavavajillas** (300), **mover el canasto de ropa**
junto a la lavadora (600) y **abrir la puerta del departamento** a un invitado que espera detrás
(600). Cada problema resuelto vale 650 (encontrarlo y decirlo, 150). Los equipos pueden proponer
tareas propias aprobadas por el comité.

### 4.6 Desafío abierto

Los dos primeros días, cada equipo puede mostrar 8 minutos de su investigación (más 2 de
preguntas), reordenando la casa si quiere. No da puntos para la competencia, pero tiene su propio
premio. Es un buen lugar para mostrar algo propio del brazo (abrir la puerta por la manilla, el
agarre guiado por la cámara).

## 5. Qué puede tomar la pinza

**Lo que la limita hoy:**

- La pinza **EBG-20** abre hasta **49 mm**; `agarre_node` acepta objetos de hasta **45 mm** de ancho
  (deja margen para que cada dedo apriete 2 mm).
- Las almohadillas de los dedos miden **21 mm**: un objeto muy alto se toma solo por su parte de
  arriba.
- Hoy agarra **solo desde arriba**, con la pinza vertical, y deja las puntas de los dedos **5 mm
  sobre la mesa**: un objeto más bajo que ~1 cm (llave, tarjeta, servilleta plana) no se alcanza.
- En la simulación sostuvo hasta **0.5 kg** sin resbalar; con 1 kg resbala 2–3 mm. La carga del
  brazo real no se ha medido.

La medida que importa es el **ancho por donde se toma**: para un objeto acostado, el lado angosto de
lo que se ve desde arriba; para uno parado, su canto. Las medidas de los objetos YCB están en la
[sección 7](#7-medidas-de-los-objetos-ycb).

| | Objetos (ancho por donde se toman, en mm) |
| --- | --- |
| ✅ **Caben** | tenedor 27 · cuchillo 20 · plumón 20 · destornilladores 34 · llave inglesa 14 · tijeras 15 · candado 29 · prensa chica 26 · dado 22 · pelota de golf 43 · cuchara (por el mango) |
| ⚠️ **Por una parte** | taza o tazón (asa o borde) · bowl (borde) · jarro (asa) · martillo (mango; pesa ~0.6 kg) · espátula (mango) · bolsa (asa rígida) · servilleta y esponja (se aprietan) |
| ⚠️ **Justo** | caja de azúcar parada 45 · caja de pudín parada 38 · caja de gelatina parada 30 · frutilla 42–46 · huevo de chocolate (además es frágil) |
| ❌ **No caben** | **lata de Red Bull** (~53 en la real de 250 ml) · **tarro de Pringles** ~75 · **cubo Rubik** 57 · latas (sopa 68, atún 86) · botellas (mostaza 59, cloro 68) · frutas (manzana 72, naranja 72, limón 54, ciruela 54) · vaso de plástico 57 · caja de galletas o cereal 72 · objeto pesado de 1–1.5 kg · bandeja y canasto (dos manos) · doblar ropa |
| ❓ **Medir** | **caja de Colgate**: su canto mide entre ~35 y ~50 mm según el tamaño |

**Lo que sale de esta tabla:**

1. **De los 4 objetos comunes, solo la caja de Colgate podría caber.** En *Pick and Place* la
   superficie extra siempre tiene 2 objetos comunes, así que esa parte queda casi fuera con la
   pinza actual.
2. **Nuestro Red Bull simulado es más angosto que el real.** El modelo que mandó el equipo mide
   45.5 mm de diámetro y 134 mm de alto; una lata real de 250 ml mide unos 53 × 135 mm. En la
   simulación cabría y en la realidad no. Conviene medir la lata real y corregir la escala del
   modelo.
3. **Los cubiertos, los plumones y las herramientas delgadas caben**, y el tenedor ya se agarra en
   la simulación (2 de 4 en la prueba del 9-nov, `scripts/prueba_objetos.sh`).
4. Para la **vajilla** (que va al lavavajillas) hay que tomarla por el borde o el asa, algo que
   `agarre_node` todavía no hace.

## 6. Qué falta para el resto

Ordenado por lo que más objetos abre:

| Qué | Para qué objetos | Dónde |
| --- | --- | --- |
| **Pinza de más carrera** (dedos más largos, otra pinza, o la misma con cremallera más larga) | Red Bull, Pringles, Rubik, latas, frutas, botellas: casi todo lo que no cabe. Decisión del equipo de hardware | — |
| **Agarre de lado** (pinza horizontal) | cajas y objetos parados (Colgate, cajas angostas), sacar y meter en repisas de 30 cm | P15 en [`PENDIENTES.md`](../PENDIENTES.md) |
| **Agarre por el borde o el asa** | taza, tazón, bowl, jarro, bolsa | P15 |
| **Bajar hasta tocar la mesa** (con control de fuerza o un tope) | llave, tarjeta, servilleta, objetos de menos de 1 cm | P15 |
| **Objetos de la competencia en la simulación** | Colgate, llaves y lata de bebida no tienen modelo; el Red Bull está mal escalado | P14 |
| **Carga del brazo real** (pesar y medir torques) | el objeto pesado de 1–1.5 kg, el martillo | P05 |

Para el **video de calificación** (6-dic), lo que ya funciona en el brazo es: la cámara con el
detector de objetos, el agarre desde arriba de objetos delgados (tenedor) y el acercamiento a la
manija de una puerta. Con las habilidades del resto del equipo (navegación, voz, reconocimiento de
personas) se juntan las 5. Para manipular en el video, lo seguro son cubiertos, plumón, dado y la
caja de Colgate si cabe.

## 7. Medidas de los objetos YCB

Medidas a lo largo de los tres ejes principales de cada objeto (el más largo primero), sacadas de
sus mallas 3D (`google_16k` de YCB) el 11-nov-2026. No incluyen el peso: el sitio de YCB no
respondía. Cuatro objetos no tienen esa malla (copa de vino, llave, plumón chico, sartén) y el
tarro de Pringles se midió con la malla de Berkeley (la que usa nuestro catálogo).

| Objeto YCB | Medidas (mm) | | Objeto YCB | Medidas (mm) |
| --- | --- | --- | --- | --- |
| 001 tarro de Pringles | 246 × 80 × 78 | | 029 plato | 261 × 260 × 27 |
| 002 tarro de café | 153 × 118 × 114 | | 030 tenedor | 198 × 27 × 15 |
| 003 caja de galletas | 214 × 163 × 72 | | 031 cuchara | 196 × 42 × 21 |
| 004 caja de azúcar | 177 × 96 × 45 | | 032 cuchillo | 215 × 20 × 15 |
| 005 lata de sopa | 103 × 68 × 68 | | 033 espátula | 311 × 84 × 33 |
| 006 botella de mostaza | 192 × 96 × 59 | | 035 taladro | 216 × 183 × 57 |
| 007 lata de atún | 86 × 86 × 34 | | 036 bloque de madera | 207 × 101 × 97 |
| 008 caja de pudín | 120 × 98 × 38 | | 037 tijeras | 202 × 88 × 15 |
| 009 caja de gelatina | 90 × 74 × 30 | | 038 candado | 78 × 62 × 29 |
| 010 lata de carne | 117 × 107 × 57 | | 040 plumón grande | 121 × 20 × 19 |
| 011 plátano | 197 × 74 × 38 | | 042 llave inglesa | 203 × 61 × 14 |
| 012 frutilla | 55 × 46 × 42 | | 043 destornillador cruz | 216 × 34 × 34 |
| 013 manzana | 76 × 75 × 72 | | 044 destornillador plano | 216 × 35 × 34 |
| 014 limón | 69 × 54 × 54 | | 048 martillo | 331 × 132 × 33 |
| 015 durazno | 64 × 60 × 58 | | 050 prensa mediana | 87 × 77 × 26 |
| 016 pera | 104 × 67 × 66 | | 053 pelota de fútbol chica | 127 × 125 × 125 |
| 017 naranja | 73 × 72 × 72 | | 054 pelota de softball | 96 × 95 × 95 |
| 018 ciruela | 57 × 54 × 54 | | 055 pelota de béisbol | 73 × 73 × 72 |
| 019 jarro | 243 × 189 × 133 | | 056 pelota de tenis | 67 × 67 × 67 |
| 021 cloro | 254 × 98 × 68 | | 057 pelota de ráquetbol | 56 × 56 × 56 |
| 022 limpiavidrios | 270 × 97 × 83 | | 058 pelota de golf | 43 × 43 × 42 |
| 024 bowl | 161 × 160 × 55 | | 061 ladrillo de espuma | 79 × 65 × 62 |
| 025 tazón | 120 × 110 × 97 | | 062 dado | 22 × 21 × 18 |
| 026 esponja | 111 × 79 × 18 | | 065-a vaso chico | 62 × 57 × 57 |
| 028 tapa de sartén | 268 × 268 × 75 | | 077 cubo Rubik | 68 × 67 × 60 |

Ojo al leerla: la medida es la caja que encierra la malla, así que un objeto curvo sale más ancho
de lo que es (el plátano mide 74 mm por su curva, pero su cuerpo tiene ~38 mm), y el cubo Rubik
sale con 68 mm por imperfecciones del escaneo (el oficial mide 57 mm).

## 8. Fuentes

- Llamada a participación 2027: <https://athome.robocup.org/call-for-participation-robocuphome-2027/>
  y <https://athome.robocup.org/call-for-participation/> (revisadas el 11-nov-2026).
- Reglamento: <https://github.com/RoboCupAtHome/RuleBook>, rama principal (última versión
  publicada: 2026.2, 11-may-2026); capítulos `pages/general_rules/Scenario.tex`,
  `pages/general_rules/Robots.tex` y `pages/general_rules/OpenChallenge.tex`, las pruebas en
  `tasks/` (`PickAndPlaceChallenge`, `HumanRobotInteractionChallenge`, `DoingLaundry`, `GPSR`,
  `Finals`) y sus puntajes en `scoresheets/`.
- Objetos YCB: <https://www.ycbbenchmarks.com/object-models/>; mallas desde
  `http://ycb-benchmarks.s3.amazonaws.com/data/google/<objeto>_google_16k.tgz`.
- Pinza y brazo: [`docs/hardware/README.md`](../hardware/README.md); agarre en la simulación:
  [`BRAZO.md`](../../ros2_ws/src/andesrobot_arm/docs/BRAZO.md) y P15 en
  [`PENDIENTES.md`](../PENDIENTES.md).
