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
6. [La lista de objetos probable](#6-la-lista-de-objetos-probable)
7. [Lista para entrenar el detector](#7-lista-para-entrenar-el-detector)
8. [Qué falta para el resto](#8-qué-falta-para-el-resto)
9. [Medidas de los objetos YCB](#9-medidas-de-los-objetos-ycb)
10. [Fuentes](#10-fuentes)

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
La bolsa oficial se anuncia en los días de preparación; según la lista mínima, es liviana y con
asas rígidas verticales. **Para preparar la prueba, el equipo acordó usar una bolsa de papel de
supermercado:** la [bolsa de papel kraft de Edipac](https://tienda.edipac.cl/bolsas/12-bolsa-papel-chica-7806510000645.html),
de 315 × 160 × 410 mm (ancho × fondo × alto, sin contar el asa), 21 L, ~55 g, con dos asas
reforzadas de papel torcido. Por ahora es una referencia para el grupo: las fotos para entrenar y
probar se sacarán con las bolsas que compremos. Las asas de papel son delgadas (unos pocos milímetros) y caben
de sobra en la pinza; lo que hay que cuidar es que el papel no se rompa ni se arrugue al apretar
(apretar con poca fuerza) y el peso de lo que lleve adentro. El set YCB no trae bolsas: las fotos
para entrenar el detector las tiene que conseguir el equipo de visión. En la simulación la bolsa
del equipo (`bolsa_equipo`) es una caja con la foto de una bolsa negra, sin asas: para practicar
la entrega en Gazebo hará falta un modelo de bolsa de papel con asas.

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
[sección 9](#9-medidas-de-los-objetos-ycb).

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

## 6. La lista de objetos probable

La lista oficial de 2027 todavía no existe: el comité la publica en los días de preparación, justo
antes de la competencia. La más cercana es la **plantilla oficial de la liga**,
[RoboCupAtHome/CompetitionTemplate](https://github.com/RoboCupAtHome/CompetitionTemplate)
(`objects/objects.md` y `objects/test_objects.md`, actualizada en abril de 2026), que es la que usa
el generador de órdenes de GPSR. Casi todos sus objetos son del set YCB. Aquí va cruzada con la
pinza (medidas de la [sección 9](#9-medidas-de-los-objetos-ycb); las marcadas "sin medir" son
estimaciones).

| Categoría | Objeto (nombre oficial) | Por donde se toma (mm) | ¿Cabe? |
| --- | --- | --- | --- |
| Vajilla (`dish`) | `cup` | 120 × 110 | ⚠️ por el borde o el asa |
| | `bowl` | 161, borde delgado | ⚠️ por el borde |
| | `plate` | 261, borde | ⚠️ difícil: es plano |
| | `fork` | 27 | ✅ (probado en la simulación) |
| | `knife` | 20 | ✅ |
| | `spoon` | 42 (mango más delgado) | ✅ por el mango |
| Comida (`food`) | `chocolate_jello` | 38 parada | ✅ |
| | `strawberry_jello` | 30 parada | ✅ |
| | `sugar` | 45 parada | ⚠️ justo |
| | `tuna` | 86 de diámetro, 34 de alto | ❌ desde arriba, ⚠️ de lado |
| | `tomato_soup` | 68 | ❌ |
| | `mustard` | 59 | ❌ |
| | `spam` | 57 | ❌ |
| | `coffee_grounds` | 114 | ❌ |
| Frutas (`fruit`) | `strawberry` | 42–46 | ⚠️ justo |
| | `banana` | ~38 de cuerpo | ⚠️ probable |
| | `plum`, `lemon` | 54 | ❌ |
| | `peach`, `pear` | 58, 66 | ❌ |
| | `orange`, `apple` | 72 | ❌ |
| Juguetes (`toy`) | `dice` | 22 | ✅ |
| | `rubiks_cube` | 57 | ❌ |
| | `tennis_ball`, `baseball`, `soccer_ball` | 67, 73, 125 | ❌ |
| Snacks (`snack`) | `pringles`, `cheezit`, `cornflakes` | ~75, 72, ~70 | ❌ |
| Bebidas (`drink`) | `juice_pack` | ~40, sin medir | ⚠️ probable |
| | `cola`, `milk`, `red_wine` | ~66, ~70, ~75, sin medir | ❌ |
| | `orange_juice`, `tropical_juice`, `iced_tea` | botella o caja, sin medir | ❌ probable |
| Limpieza (`cleaning_supply`) | `sponge` | 18 de grosor | ⚠️ de lado o apretándola |
| | `cleanser` | 68 | ❌ |
| De pruebas específicas | `bag_one`, `bag_two`, `bag_three` (para practicar: bolsa de papel de supermercado) | el asa de papel, pocos mm | ✅ |
| | `dishwasher_tab` | ~20, sin medir | ✅ |

Además, los **4 objetos comunes** de la sección 3: Red Bull (~53 mm) ❌, Pringles ❌, Rubik ❌ y
Colgate ❓ (medirla).

**Resumen:** caben bien unos 8 (tenedor, cuchillo, cuchara, dado, caja de gelatina, caja de pudín,
pastilla de lavavajillas y el asa de un bolso); a medias unos 9 (taza, bowl y plato por el borde,
caja de azúcar, frutilla, plátano, cajita de jugo, esponja y lata de atún de lado; necesitan el
agarre de lado o por el borde de la [sección 8](#8-qué-falta-para-el-resto)); el resto, unos 24, son
más anchos que lo que abre la pinza. La lista real de 2027 puede cambiar, sobre todo los objetos
locales que se compran en la sede.

## 7. Lista para entrenar el detector

Lo que el detector de objetos (YOLO) debería reconocer en la casa, **no solo lo que la pinza puede
tomar**: en GPSR y en la final el robot tiene que encontrar objetos, nombrarlos, decir dónde están
y elegirlos por color, tamaño o posición, aunque no los tome. Y en las pruebas aparecen muebles y
partes de la casa que el robot tiene que ubicar.

**Cómo conviene armar las clases.** Las órdenes usan el **nombre oficial de cada objeto**
(`apple`, `tuna`, `dice`) o su **categoría** (`fruit`, `food`, `toy`). El modelo actual
(`dataset_reglamento3`, 27 clases) mezcla los dos niveles: tiene clases por objeto (`fork`,
`pringles`, `rubik_cube`) y clases generales (`fruit`, `food`, `drink`, `tool`), así que no puede
distinguir una manzana de una naranja ni una lata de atún de una de sopa. Lo recomendable es **una
clase por objeto** con su nombre oficial y pasar a la categoría en el código (una tabla
objeto → categoría), que es como la publica el comité.

**A. Objetos de la plantilla oficial** (39 objetos conocidos más 4 de pruebas específicas: 3 bolsas y la pastilla del lavavajillas), con su categoría:

| Categoría | Objetos | En el modelo actual |
| --- | --- | --- |
| `dish` | cup, bowl, plate, fork, knife, spoon | ✅ todos por separado |
| `food` | chocolate_jello, strawberry_jello, sugar, tuna, tomato_soup, mustard, spam, coffee_grounds | solo `food` general |
| `fruit` | apple, orange, pear, peach, plum, lemon, banana, strawberry | solo `fruit` general |
| `toy` | dice, rubiks_cube, tennis_ball, baseball, soccer_ball | solo `rubik_cube` |
| `snack` | pringles, cheezit, cornflakes | `pringles` y `cereal_box` |
| `drink` | juice_pack, cola, milk, orange_juice, tropical_juice, red_wine, iced_tea | solo `drink` general (y `redbull_can`) |
| `cleaning_supply` | cleanser, sponge | solo `cleaning_supply` general |
| pruebas | bag_one, bag_two, bag_three (para practicar: bolsa de papel de supermercado; no hay en YCB, fotos propias), dishwasher_tab | `bag`; la pastilla no |

**B. Objetos comunes (siempre en la competencia):** redbull_can, pringles, rubiks_cube,
colgate_box. Los cuatro ya están en el modelo actual.

**Cuántas clases son.** El núcleo son **43 clases**: los 39 objetos conocidos de la plantilla,
`redbull_can` y `colgate_box` (pringles y rubiks_cube ya están entre los 39), una clase `bag` (la
bolsa de papel de supermercado; las 3 bolsas de la plantilla son ejemplos de la misma clase) y
`dishwasher_tab`. Lo de C suma unas 8 más (con la basura como una sola clase) y lo de D es
opcional, unas 25 si se incluye todo.

**Lista corta para empezar con el brazo (17 clases).** Para implementar la manipulación no hace
falta esperar el modelo completo: basta un primer modelo con los objetos que el brazo toma ahora
o pronto, y los que tiene que ver en sus pruebas aunque todavía no los tome. Las imágenes son las
de referencia de la liga (plantilla oficial y reglamento, enlazadas a una versión fija de sus
repositorios de GitHub); los cubiertos y la vajilla de la plantilla son de un mismo juego de
enlozado rojo. La bolsa de la imagen es una de las tres de la plantilla (bolsas reutilizables);
nosotros practicaremos con una bolsa de papel kraft de 315 × 160 × 410 mm con asas de papel
torcido (sección 4.2).

**Prioridad 1: el brazo los toma (9).** Caben en la pinza (asa de la bolsa, cubiertos, dado, cajas
angostas paradas, pastilla); la bolsa es la entrega de la prueba de los invitados.

|   |   |   |   |   |
| --- | --- | --- | --- | --- |
| <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/test_objects/carry_my_luggage/bag_one.jpg" width="90" alt="bag"><br>`bag`<br>bolsa (usaremos una de papel de supermercado) | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/fork.png" width="90" alt="fork"><br>`fork`<br>tenedor | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/knife.png" width="90" alt="knife"><br>`knife`<br>cuchillo | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/spoon.png" width="90" alt="spoon"><br>`spoon`<br>cuchara | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/toys%21toy/dice.png" width="90" alt="dice"><br>`dice`<br>dado |
| <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/food/chocolate_jello.png" width="90" alt="chocolate_jello"><br>`chocolate_jello`<br>caja de pudín | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/food/strawberry_jello.png" width="90" alt="strawberry_jello"><br>`strawberry_jello`<br>caja de gelatina | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/test_objects/clean_the_table/dishwasher_tab.jpg" width="90" alt="dishwasher_tab"><br>`dishwasher_tab`<br>pastilla de lavavajillas | <img src="https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/ebdf34d223ddc3ae89ac3f16a23ad28024adfdf6/images/colgate.jpeg" width="90" alt="colgate_box"><br>`colgate_box`<br>caja de pasta dental Colgate |   |

**Prioridad 2: el brazo los tiene que ver (8).** Están en Pick and Place (vajilla, objetos comunes,
desayuno): la vajilla se tomará por el borde y los demás hay que ubicarlos aunque no quepan.

|   |   |   |   |   |
| --- | --- | --- | --- | --- |
| <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/cup.png" width="90" alt="cup"><br>`cup`<br>taza | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/bowl.png" width="90" alt="bowl"><br>`bowl`<br>bowl | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/dishes%21dish/plate.png" width="90" alt="plate"><br>`plate`<br>plato | <img src="https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/ebdf34d223ddc3ae89ac3f16a23ad28024adfdf6/images/red_bull.jpg" width="90" alt="redbull_can"><br>`redbull_can`<br>lata de Red Bull | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/snacks%21snack/pringles.png" width="90" alt="pringles"><br>`pringles`<br>tarro de Pringles |
| <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/toys%21toy/rubiks_cube.png" width="90" alt="rubiks_cube"><br>`rubiks_cube`<br>cubo Rubik | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/snacks%21snack/cornflakes.jpg" width="90" alt="cornflakes"><br>`cornflakes`<br>cereal | <img src="https://raw.githubusercontent.com/RoboCupAtHome/CompetitionTemplate/af2972f0670afbb068701f1dc5281c0efac5045c/objects/known_objects/drinks%21drink/milk.jpg" width="90" alt="milk"><br>`milk`<br>leche |   |   |

El resto de la lista entra después, en el modelo completo.

**C. Lo que exige el reglamento y la plantilla no trae:** servilleta (*napkin*), bandeja
(`tray`, ya está), canasto de ropa (*laundry basket*), polera (*t-shirt*; hoy `clothing`), un
objeto pesado de 1–1.5 kg (por ejemplo una botella de 1.5 L), un objeto frágil (por ejemplo un
huevo de chocolate), un objeto deformable (paño o toalla) y la basura: el comité elige una
categoría como basura en los días de preparación, así que conviene tener ejemplos típicos (lata
vacía, vaso de papel, papel arrugado, botella plástica).

Imágenes de referencia con licencia libre, de Wikimedia Commons (cada una enlaza a su página, con
la licencia completa). Son solo para que el grupo sepa qué buscar: la bolsa de la imagen tiene
asas planas y la de Edipac las tiene de papel torcido, y la botella no dice si es de 1.5 L. Para
entrenar hacen falta fotos propias de los objetos reales.

| Imagen | Clase | Qué es | Licencia y autor (Wikimedia Commons) |
| --- | --- | --- | --- |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/8/81/Albert_Heijn_paper_shopping_bag_%282019%29_05.jpg/250px-Albert_Heijn_paper_shopping_bag_%282019%29_05.jpg" width="90" alt="bag"> | `bag` | bolsa de papel kraft | [CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:Albert_Heijn_paper_shopping_bag_(2019)_05.jpg), Donald Trung Quoc Don (Chữ Hán: 徵國單) - W… |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/4/40/Newark_Park_sausage_roll_flat_white_coffee_caf%C3%A9_marquee_Ozleworth_Gloucestershire_England.jpg/250px-Newark_Park_sausage_roll_flat_white_coffee_caf%C3%A9_marquee_Ozleworth_Gloucestershire_England.jpg" width="90" alt="napkin"> | `napkin` | servilleta | [CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:Newark_Park_sausage_roll_flat_white_coffee_caf%C3%A9_marquee_Ozleworth_Gloucestershire_England.jpg), Acabashi |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/8/8d/Tiffany_-_silver_serving_tray_-_HLAMBG.png/250px-Tiffany_-_silver_serving_tray_-_HLAMBG.png" width="90" alt="tray"> | `tray` | bandeja | [CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:Tiffany_-_silver_serving_tray_-_HLAMBG.png), Eric Polk |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/6/67/Clothes_in_white_laundry_basket.jpg/250px-Clothes_in_white_laundry_basket.jpg" width="90" alt="laundry_basket"> | `laundry_basket` | canasto de ropa | [CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:Clothes_in_white_laundry_basket.jpg), Corn cheese |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/6/61/BrownIEditShirt-plainBackground.png/250px-BrownIEditShirt-plainBackground.png" width="90" alt="t_shirt"> | `t_shirt` | polera | [CC BY-SA 3.0](https://commons.wikimedia.org/wiki/File:BrownIEditShirt-plainBackground.png), Daniel Furon under contract for the Wiki… |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/a/ad/Bottle_of_Water.jpg/250px-Bottle_of_Water.jpg" width="90" alt="heavy_object"> | `heavy_object` | objeto pesado: botella de agua | [CC0](https://commons.wikimedia.org/wiki/File:Bottle_of_Water.jpg), Jiafei Slay Queen |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/6/62/Brought_some_Kinder_Surprise_eggs_home_for_the_girls%2C_they_were_a_big_hit.jpg/250px-Brought_some_Kinder_Surprise_eggs_home_for_the_girls%2C_they_were_a_big_hit.jpg" width="90" alt="fragile_object"> | `fragile_object` | objeto frágil: huevo de chocolate | [CC BY 2.0](https://commons.wikimedia.org/wiki/File:Brought_some_Kinder_Surprise_eggs_home_for_the_girls,_they_were_a_big_hit.jpg), Robert Nelson |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/6/61/Viskestykke.jpg/250px-Viskestykke.jpg" width="90" alt="cloth"> | `cloth` | deformable: paño de cocina | [Public domain](https://commons.wikimedia.org/wiki/File:Viskestykke.jpg), Nillerdk |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/8/89/Crushed_coka_cola_can.jpg/250px-Crushed_coka_cola_can.jpg" width="90" alt="trash"> | `trash` | basura: lata aplastada | [CC BY-SA 4.0](https://commons.wikimedia.org/wiki/File:Crushed_coka_cola_can.jpg), क्षत्रिय117 |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/b/b3/Cups-disposable-paper-nongovernment-Tamil_Nadu.JPG/250px-Cups-disposable-paper-nongovernment-Tamil_Nadu.JPG" width="90" alt="trash"> | `trash` | basura: vaso de papel | [CC BY-SA 3.0](https://commons.wikimedia.org/wiki/File:Cups-disposable-paper-nongovernment-Tamil_Nadu.JPG), Tha-uzhavan |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/e/ea/Paperball_2.jpeg/250px-Paperball_2.jpeg" width="90" alt="trash"> | `trash` | basura: papel arrugado | [Copyrighted free use](https://commons.wikimedia.org/wiki/File:Paperball_2.jpeg), Nikhilesh Haval |
| <img src="https://thumb.wikimedia.org/wikipedia/commons/thumb/5/55/Empty_Plastic_Bottle.jpg/250px-Empty_Plastic_Bottle.jpg" width="90" alt="trash"> | `trash` | basura: botella plástica vacía | [CC0](https://commons.wikimedia.org/wiki/File:Empty_Plastic_Bottle.jpg), Echendu Tracy |

**D. Otros objetos YCB que el comité podría elegir** (la plantilla usa solo una parte del set):
mug (tazón), pitcher_base (jarro), wine_glass (copa), cups (vasos de plástico apilables),
skillet y skillet_lid (sartén y su tapa), spatula, windex_bottle, scissors, large_marker y
small_marker (plumones), padlock (candado), key (llave), power_drill (taladro), hammer, wrench,
screwdrivers, clamps (prensas), wood_block, foam_brick, golf_ball, racquetball, softball,
lego_duplo, toy_airplane, timer, marbles (bolitas), chain (cadena).

**E. Objetos locales:** productos comprados en la sede (Alemania en 2027); no se pueden conocer
antes. Sirve entrenar con fotos de productos alemanes comunes de esas mismas categorías y,
durante la competencia, sumar fotos de los objetos reales (se entregan para entrenar el primer
día de preparación, de a cinco por equipo y por una hora).

**F. Muebles y partes de la casa** (no se toman, pero el robot tiene que ubicarlos): mesa de
comedor, mesa de centro, silla, sillón, estante o librero, despensa, lavavajillas (puerta y
bandeja), lavadora de carga frontal, basurero, puerta y **manilla** (ya hay un modelo propio de la
manija), y decoración (planta, cuadro, espejo, reloj, vela, libro). Muchos están en modelos
preentrenados (COCO trae `chair`, `couch`, `dining table`, `potted plant`, `book`, `clock`), así
que quizás baste con uno de esos para los muebles y dejar el modelo propio para los objetos.

**Fotos para entrenar:** fotos reales de cada objeto desde varias vistas, distancias y luces
(la competencia es con luz natural), sobre mesas, en repisas y en el suelo, juntos y apilados.
Si además se usa la simulación, sumar fotos de los modelos 3D de la mesa simulada para que el
detector funcione también en Gazebo: hoy el modelo nuevo reconoce 9 de los 26 objetos simulados
(P14 en [`PENDIENTES.md`](../PENDIENTES.md)).

## 8. Qué falta para el resto

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

## 9. Medidas de los objetos YCB

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

## 10. Fuentes

- Llamada a participación 2027: <https://athome.robocup.org/call-for-participation-robocuphome-2027/>
  y <https://athome.robocup.org/call-for-participation/> (revisadas el 11-nov-2026).
- Reglamento: <https://github.com/RoboCupAtHome/RuleBook>, rama principal (última versión
  publicada: 2026.2, 11-may-2026); capítulos `pages/general_rules/Scenario.tex`,
  `pages/general_rules/Robots.tex` y `pages/general_rules/OpenChallenge.tex`, las pruebas en
  `tasks/` (`PickAndPlaceChallenge`, `HumanRobotInteractionChallenge`, `DoingLaundry`, `GPSR`,
  `Finals`) y sus puntajes en `scoresheets/`.
- Imágenes de referencia de la sección 7 C: Wikimedia Commons, <https://commons.wikimedia.org>
  (autor y licencia en cada fila de la tabla).
- Lista de objetos de la plantilla oficial: <https://github.com/RoboCupAtHome/CompetitionTemplate>
  (`objects/objects.md` y `objects/test_objects.md`, abril de 2026).
- Objetos YCB: <https://www.ycbbenchmarks.com/object-models/>; mallas desde
  `http://ycb-benchmarks.s3.amazonaws.com/data/google/<objeto>_google_16k.tgz`.
- Pinza y brazo: [`docs/hardware/README.md`](../hardware/README.md); agarre en la simulación:
  [`BRAZO.md`](../../ros2_ws/src/andesrobot_arm/docs/BRAZO.md) y P15 en
  [`PENDIENTES.md`](../PENDIENTES.md).
