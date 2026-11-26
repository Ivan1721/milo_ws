// brazo_mega.ino — programa del Arduino Mega que mueve el brazo de Milo (versión 5: rampa de
// aceleración + cableado de POSITIVO COMÚN en los TB6600 (v3) + servo de la garra de 270° por
// microsegundos, 0..297° (v4/v5, del equipo, 25-nov-2026); los motores paso a paso, igual que v3).
//
// En palabras simples: el PC (ROS 2 + MoveIt, o el driver del control de Xbox) decide a qué posición va cada articulación y se la
// manda por el cable USB; este programa genera los pulsos para los 6 drivers TB6600 de los motores
// paso a paso y mueve el servo de la garra. El Arduino NO sabe de ángulos ni de radianes: solo
// cuenta PASOS. La conversión (pasos por vuelta del motor x micropasos del driver x reducción
// 38.4:1) la hace el PC, con los números de ros2_ws/src/andesrobot_arm/config/brazo_real.yaml.
//
// Cableado de los TB6600 (v3, del equipo, prueba del brazo armado en Chile, nov-2026): cada
// señal del driver entra a un optoacoplador con dos bornes (PUL+/PUL-, DIR+/DIR-). Hay dos formas de cablearlo:
//   - POSITIVO COMÚN (el del brazo armado): PUL+ y DIR+ de los 6 drivers juntos a +5 V del
//     Arduino, y los pines del Arduino a PUL- y DIR-. El opto se enciende con el pin en LOW: las
//     señales son ACTIVAS EN BAJO (STEP en reposo = HIGH, cada pulso es un LOW de PULSO_US).
//   - CÁTODO COMÚN (el de la v2): PUL- y DIR- a GND, y los pines a PUL+ / DIR+; activas en alto.
// Se elige con POSITIVO_COMUN (abajo). En los dos, "sentido positivo" = opto de DIR encendido,
// así que el signo de los pasos (y config/brazo_real.yaml) no cambia al pasar de uno al otro.
//
// Cómo se mueve un motor paso a paso con un TB6600: el pin DIR dice el sentido y cada pulso en el pin STEP (PUL) avanza un paso (un micropaso, según los interruptores del
// driver). Más pulsos por segundo = más rápido. Las 6 articulaciones se mueven a la vez: en cada
// vuelta de loop() se revisa si a cada una ya le toca su próximo pulso (sin delay largos).
//
// Rampa de aceleración (v2): un motor paso a paso no puede pasar de quieto a 2000 pasos/s de un
// golpe: el rotor no alcanza a seguir los pulsos y "pierde pasos" (el Arduino cree que llegó y el
// brazo quedó en otra parte). Por eso la velocidad de cada articulación cambia de a poco: como
// mucho `aceleracion` pasos/s por cada segundo (cada TICK_US se recalcula). Dos reglas:
//   - M (probar a mano): perfil trapezoidal. Acelera hasta velPrueba, sigue a esa velocidad y
//     frena a tiempo para llegar justo: con velocidad v necesita v²/(2·aceleracion) pasos para
//     parar, así que nunca va más rápido que sqrt(2·aceleracion·pasos_que_faltan).
//   - G (ROS, cada 50 ms): la velocidad sigue a la que pide el PC (lo que falta / el tiempo que
//     queda para el plazo del G), con la misma rampa. No frena "para llegar justo" a cada G: su
//     destino está solo 50 ms adelante y con esa regla nunca pasaría de ~400 pasos/s. Las
//     trayectorias de MoveIt ya vienen con aceleración acotada y terminan con velocidad 0.
//   Si el destino queda del otro lado (cambio de sentido), primero frena y después da la vuelta.
//   X y el perro guardián paran EN SECO (son paradas de emergencia).
//
// Comandos (una línea de texto terminada en "\n", 115200 baudios; el Arduino contesta otra línea):
//
//   P                      PONG brazo_mega v5                 ¿estás ahí?
//   E                      E p1 p2 p3 p4 p5 p6 mov servo       estado: posición de cada articulación
//                                                             [pasos], mov = 1 si alguna se mueve,
//                                                             servo = ángulo o -1 si está suelto
//   G ms p1 p2 p3 p4 p5 p6 OK G                               ir a esas posiciones [pasos] y llegar
//                                                             todas a la vez en ms milisegundos
//                                                             (sin pasar de VEL_MAX ni de la rampa)
//   M j pasos              OK M j destino                     mover SOLO la articulación j (1..6)
//                                                             esa cantidad de pasos (+ o -), con
//                                                             velPrueba: para probar una por una
//   V pasos_por_s          OK V n                             cambiar velPrueba (la de M)
//   A pasos_por_s2         OK A n                             cambiar la aceleración (rampa)
//   X                      OK X                               parar todo donde está (en seco)
//   Z                      OK Z                               la posición actual pasa a ser 0
//                                                             (solo quieto)
//   S grados               OK S grados                        mover el servo de la garra (0..SERVO_MAX)
//   D                      OK D                               soltar el servo
//   otra cosa              ERR motivo
//
// Seguridad:
//   - Al encender no mueve nada. Las posiciones arrancan en 0: el Arduino no sabe dónde está el
//     brazo (no hay finales de carrera todavía), así que hay que encenderlo con el brazo en su pose
//     de cero, o usar Z cuando esté ahí.
//   - Perro guardián: si el PC manda G (lo hace ROS, unas 20 veces por segundo) y deja de hablar
//     WATCHDOG_MS, el brazo se detiene (por ejemplo si se cae el nodo o se desconecta el USB).
//     M no tiene perro guardián (es para probar a mano desde el Monitor Serie).
//   - Los drivers quedan siempre habilitados (ENA sin conectar): si se deshabilitan, J2 y J3 caen
//     por el peso del brazo.
//   - El servo se engancha recién con el primer S; se alimenta con una fuente de 5-6 V aparte
//     (GND común), no del pin 5V del Arduino.
//
// Pines: los de la imagen del cableado (conversación del 23-nov-2026, ver firmware/README.md),
// los mismos que usó el equipo en la prueba del 24-nov.
// Para cargarlo y probarlo: firmware/README.md.

#include <Servo.h>

const int N = 6;                                     // articulaciones J1..J6
//                              J1  J2  J3  J4  J5  J6
const uint8_t PIN_STEP[N] = {45, 47, 49, 51, 53, 43};  // a PUL- (positivo común) o PUL+
const uint8_t PIN_DIR[N]  = {44, 46, 48, 50, 52, 42};  // a DIR- (positivo común) o DIR+
const int PIN_SERVO = 9;                             // señal del servo de la garra
const int PIN_LED = LED_BUILTIN;                     // pin 13: encendido mientras algo se mueve

// true = positivo común (señales activas en bajo, el brazo armado); false = cátodo común (v2).
const bool POSITIVO_COMUN = true;
const bool OPTO_ON = !POSITIVO_COMUN;   // nivel del pin que enciende el opto del TB6600
const bool OPTO_OFF = POSITIVO_COMUN;   // nivel de reposo

const long BAUDIOS = 115200;
const unsigned int PULSO_US = 5;      // ancho del pulso STEP (el TB6600 pide al menos 2.5 us)
const unsigned int DIR_US = 5;        // espera después de cambiar DIR, antes del pulso
const float VEL_MAX = 2000.0;         // [pasos/s] tope (con 1/8 y 38.4:1, ~11.7 °/s)
// Velocidad a la que se puede arrancar, parar o dar la vuelta sin rampa (un motor paso a paso
// arranca sin problema a pocas decenas de pasos/s; con 1/8 de micropaso, 100 pasos/s son 12.5
// pasos enteros por segundo).
const float VEL_INICIO = 100.0;
// Al llegar al destino a esta velocidad o menos, para en seco (400 pasos/s con 1/8 = 50 pasos
// enteros/s). Con un umbral más bajo, por lo discreto de los ticks llegaba un poco más rápido,
// se pasaba, volvía y se volvía a pasar sin terminar nunca (medido el 23-nov, firmware v2 inicial).
const float VEL_PARADA = 400.0;
// Se frena como si la aceleración fuera un 10 % menor (margen para llegar despacio).
const float MARGEN_FRENO = 0.9;
const unsigned long TICK_US = 2000;   // cada cuánto se recalculan las velocidades (500 veces/s)
const unsigned long G_MIN_US = 20000; // plazo mínimo para calcular la velocidad de G
const unsigned long WATCHDOG_MS = 500;
// Servo de la garra (v4/v5, del equipo, 25-nov-2026): es de 270°. La librería Servo con write() no
// pasa de 180°, así que se manda por MICROSEGUNDOS, lineal: 0° = SERVO_US_MIN, SERVO_RANGO° =
// SERVO_US_MAX. v5: pulso ampliado a 400..2600 µs (con 500..2500 la garra no llegaba a sus topes);
// misma escala que 500..2500 = 270° (7.4 µs/°), así que 0° = 400 µs y 297° = 2600 µs.
// OJO: con v4, 0° era 500 µs; ahora 500 µs = ~14°. Los topes reales de la garra se fijan en el PC
// (garra: de config/brazo_real.yaml y servo_min / servo_max de brazo_driver.yaml).
const int SERVO_RANGO = 297;          // [grados] los que caben en SERVO_US_MIN..SERVO_US_MAX
const int SERVO_US_MIN = 400;         // [µs] pulso de 0°
const int SERVO_US_MAX = 2600;        // [µs] pulso de SERVO_RANGO°
const int SERVO_MIN = 0;              // [grados] límites que se dejan usar
const int SERVO_MAX = SERVO_RANGO;    // [grados]
const int LARGO_MAX = 96;             // largo máximo de una línea (un G con 6 números cabe)

float velPrueba = 200.0;              // [pasos/s] la de M (con 1/8 y 38.4:1, ~1.2 °/s)
// [pasos/s²] 4000 = de 0 a VEL_MAX en 0.5 s (~0.41 rad/s² en la articulación con 1/8 y 38.4:1;
// MoveIt pide 1 rad/s² x 0.15 de escala en el robot real = 0.15 rad/s²). Probable que haya que
// bajarla con el brazo real si pierde pasos al arrancar: comando A.
float aceleracion = 4000.0;

long pos[N];                          // posición actual de cada articulación [pasos]
long destino[N];                      // a dónde va [pasos]
float vel[N];                         // [pasos/s] velocidad actual (>= 0; el sentido es dirMov)
unsigned long intervalo[N];           // [us] entre pulsos a esa velocidad (lo calcula rampas)
int dirMov[N];                        // +1 / -1: sentido en que se está moviendo
bool porM[N];                         // el último movimiento de esa articulación vino de M
unsigned long plazoG[N];              // [us] (micros) cuándo tiene que llegar al destino de G
unsigned long ultimo[N];              // [us] momento programado del último pulso
int sentidoPin[N];                    // +1 / -1: lo último escrito en DIR (0 = nada aún)
bool modoG = false;                   // hay movimiento de G (con perro guardián)
unsigned long ultimoComando = 0;      // [ms] cuándo llegó el último comando
unsigned long ultimoTick = 0;         // [us]

Servo garra;
int angulo = -1;                      // -1 = servo suelto

// Escritura rápida de los pines STEP y DIR: directo en el registro del puerto (~0.2 us) en vez
// de digitalWrite (~5 us, que revisa el número de pin cada vez). Con las 6 articulaciones a
// 2000 pasos/s son 12000 pulsos por segundo: con digitalWrite el Mega no alcanzaba (medido el
// 23-nov: un movimiento de 3.5 s tardaba 5.1 s).
volatile uint8_t *regStep[N], *regDir[N];
uint8_t mascaraStep[N], mascaraDir[N];

inline void escribir(volatile uint8_t *reg, uint8_t mascara, bool alto) {
  uint8_t sreg = SREG;                // sin interrupciones mientras se cambia el registro
  cli();
  if (alto) *reg |= mascara; else *reg &= ~mascara;
  SREG = sreg;
}

char linea[LARGO_MAX + 1];
int largo = 0;
bool descartando = false;             // la línea fue demasiado larga: ignorar hasta el "\n"

// ¿Alguna articulación no llegó o todavía va con velocidad?
bool moviendo() {
  for (int j = 0; j < N; j++) {
    if (pos[j] != destino[j] || vel[j] > 0.0) return true;
  }
  return false;
}

// Parada de emergencia: todo queda donde está, sin velocidad.
void parar() {
  for (int j = 0; j < N; j++) {
    destino[j] = pos[j];
    vel[j] = 0.0;
    intervalo[j] = 0;
  }
  modoG = false;
}

void setup() {
  for (int j = 0; j < N; j++) {
    // Primero el nivel de reposo y DESPUÉS salida: con positivo común, un pin que arranca en
    // LOW es un paso falso (y un opto de DIR encendido) en el instante de encender.
    digitalWrite(PIN_STEP[j], OPTO_OFF ? HIGH : LOW);
    digitalWrite(PIN_DIR[j], OPTO_OFF ? HIGH : LOW);
    pinMode(PIN_STEP[j], OUTPUT);
    pinMode(PIN_DIR[j], OUTPUT);
    regStep[j] = portOutputRegister(digitalPinToPort(PIN_STEP[j]));
    regDir[j] = portOutputRegister(digitalPinToPort(PIN_DIR[j]));
    mascaraStep[j] = digitalPinToBitMask(PIN_STEP[j]);
    mascaraDir[j] = digitalPinToBitMask(PIN_DIR[j]);
    pos[j] = 0;
    destino[j] = 0;
    vel[j] = 0.0;
    intervalo[j] = 0;
    dirMov[j] = 1;
    porM[j] = false;
    plazoG[j] = 0;
    sentidoPin[j] = 0;
    ultimo[j] = 0;
  }
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(BAUDIOS);
  // Aviso de que arrancó (el Arduino se reinicia la primera vez que el PC abre el puerto).
  Serial.println(F("LISTO brazo_mega v5"));
}

// Lee el siguiente número entero de la línea a partir de *p (salta espacios). Devuelve false si
// no hay un número ahí.
bool leerEntero(char **p, long *valor) {
  while (**p == ' ') (*p)++;
  char *fin;
  long v = strtol(*p, &fin, 10);
  if (fin == *p) return false;
  *valor = v;
  *p = fin;
  return true;
}

void atender(char *texto) {
  ultimoComando = millis();
  char comando = texto[0];
  char *p = texto + 1;
  long a, b;
  if (comando == 'P') {
    Serial.println(F("PONG brazo_mega v5"));
  } else if (comando == 'E') {
    Serial.print('E');
    for (int j = 0; j < N; j++) {
      Serial.print(' ');
      Serial.print(pos[j]);
    }
    Serial.print(' ');
    Serial.print(moviendo() ? 1 : 0);
    Serial.print(' ');
    Serial.println(angulo);
  } else if (comando == 'G') {
    long ms, nuevo[N];
    bool bien = leerEntero(&p, &ms) && ms > 0;
    for (int j = 0; j < N && bien; j++) bien = leerEntero(&p, &nuevo[j]);
    if (!bien) {
      Serial.println(F("ERR uso: G ms p1 p2 p3 p4 p5 p6"));
      return;
    }
    unsigned long plazo = micros() + (unsigned long)ms * 1000UL;
    for (int j = 0; j < N; j++) {
      destino[j] = nuevo[j];
      plazoG[j] = plazo;
      porM[j] = false;
    }
    modoG = true;
    Serial.println(F("OK G"));
  } else if (comando == 'M') {
    if (!leerEntero(&p, &a) || !leerEntero(&p, &b) || a < 1 || a > N) {
      Serial.println(F("ERR uso: M j pasos  (j = 1..6)"));
      return;
    }
    int j = a - 1;
    destino[j] = destino[j] + b;      // relativo al destino (si estaba quieta, a la posición)
    if (vel[j] == 0.0) destino[j] = pos[j] + b;
    porM[j] = true;
    Serial.print(F("OK M "));
    Serial.print(a);
    Serial.print(' ');
    Serial.println(destino[j]);
  } else if (comando == 'V') {
    if (!leerEntero(&p, &a) || a < 1) {
      Serial.println(F("ERR uso: V pasos_por_s"));
      return;
    }
    velPrueba = constrain((float)a, 1.0, VEL_MAX);
    Serial.print(F("OK V "));
    Serial.println((long)velPrueba);
  } else if (comando == 'A') {
    if (!leerEntero(&p, &a) || a < 100) {
      Serial.println(F("ERR uso: A pasos_por_s2  (100 o mas)"));
      return;
    }
    aceleracion = constrain((float)a, 100.0, 100000.0);
    Serial.print(F("OK A "));
    Serial.println((long)aceleracion);
  } else if (comando == 'X') {
    parar();
    Serial.println(F("OK X"));
  } else if (comando == 'Z') {
    if (moviendo()) {
      Serial.println(F("ERR Z solo con el brazo quieto (X primero)"));
      return;
    }
    for (int j = 0; j < N; j++) {
      pos[j] = 0;
      destino[j] = 0;
    }
    Serial.println(F("OK Z"));
  } else if (comando == 'S') {
    if (!leerEntero(&p, &a)) {
      Serial.println(F("ERR uso: S grados"));
      return;
    }
    int pedido = constrain((int)a, SERVO_MIN, SERVO_MAX);
    if (angulo < 0) garra.attach(PIN_SERVO, SERVO_US_MIN, SERVO_US_MAX);   // engancha recién ahora
    garra.writeMicroseconds(map(pedido, 0, SERVO_RANGO, SERVO_US_MIN, SERVO_US_MAX));
    angulo = pedido;
    Serial.print(F("OK S "));
    Serial.println(angulo);
  } else if (comando == 'D') {
    garra.detach();
    angulo = -1;
    Serial.println(F("OK D"));
  } else {
    Serial.print(F("ERR comando desconocido: "));
    Serial.println(texto);
  }
}

void leerSerie() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linea[largo] = '\0';
      if (descartando) {
        // Una sola respuesta para toda la línea larga (probado 23-nov: antes el resto de la
        // línea se tomaba como otro comando y daba una segunda respuesta, que descuadra al PC).
        Serial.println(F("ERR linea demasiado larga"));
        descartando = false;
      } else if (largo > 0) {
        atender(linea);
      }
      largo = 0;
    } else if (descartando) {
      // seguir tirando caracteres hasta el fin de línea
    } else if (largo < LARGO_MAX) {
      linea[largo++] = c;
    } else {
      descartando = true;
      largo = 0;
    }
  }
}

// Recalcula la velocidad de cada articulación (la rampa): ver las reglas al comienzo.
// Cuidando el tiempo: el Mega no tiene unidad de punto flotante, y una raíz cuadrada o una
// división le toman ~30-40 us. Con las 6 articulaciones, 500 veces por segundo, eso era ~25 % del
// procesador y no alcanzaba para dar 12000 pulsos por segundo (medido el 23-nov). Por eso la raíz
// solo se calcula cuando de verdad hay que frenar, y la división solo cuando cambia la velocidad.
void rampas(unsigned long ahora) {
  float dv = aceleracion * (TICK_US / 1000000.0);   // cuánto puede cambiar la velocidad ahora
  float k = 2.0 * MARGEN_FRENO * aceleracion;       // frenable = sqrt(k · d)
  for (int j = 0; j < N; j++) {
    long falta = destino[j] - pos[j];
    if (falta == 0 && vel[j] <= VEL_PARADA) {
      vel[j] = 0.0;                    // llegó (despacio): se para
      intervalo[j] = 0;
      continue;
    }
    int lado = falta > 0 ? 1 : (falta < 0 ? -1 : 0);
    float objetivo;
    if (lado != 0 && lado != dirMov[j] && vel[j] > VEL_INICIO) {
      objetivo = 0.0;                  // va para el otro lado: primero frenar
    } else {
      if (lado != 0 && vel[j] <= VEL_INICIO) dirMov[j] = lado;   // (re)arranca hacia el destino
      float d = labs(falta);
      // Lo más rápido que puede ir y todavía frenar en lo que falta (con margen): sqrt(k·d).
      // Se aplica con limitarFreno: solo si el objetivo pasa de eso (comparando cuadrados, sin
      // la raíz) se calcula la raíz.
      bool limitarFreno;
      if (porM[j]) {
        // Trapezoidal: velPrueba, sin pasar de lo frenable.
        objetivo = velPrueba;
        limitarFreno = true;
      } else {
        // G: lo que falta en el tiempo que queda para el plazo (al menos G_MIN_US). Mientras el
        // PC siga mandando G (antes del plazo) no se frena "para llegar justo" (ver arriba);
        // vencido el plazo sin un G nuevo, es el destino final: se frena para llegar justo.
        long queda = (long)(plazoG[j] - ahora);
        float t = (queda > (long)G_MIN_US ? queda : (long)G_MIN_US) / 1000000.0;
        objetivo = d / t;
        limitarFreno = queda <= 0;
      }
      objetivo = min(objetivo, VEL_MAX);
      if (limitarFreno && objetivo * objetivo > k * d) objetivo = sqrt(k * d);
    }
    float antes = vel[j];
    if (vel[j] < objetivo) vel[j] = min(objetivo, vel[j] + dv);
    else vel[j] = max(objetivo, vel[j] - dv);
    // El intervalo entre pulsos: solo si cambió la velocidad (a velocidad constante, nada).
    if (vel[j] != antes || intervalo[j] == 0) {
      intervalo[j] = vel[j] > 0.0 ? (unsigned long)(1000000.0 / vel[j]) : 0;
    }
  }
}

// Da los pulsos que tocan. Cada articulación tiene su "próximo momento" de paso, que avanza de a
// un intervalo exacto (ultimo += intervalo), no desde cuándo se revisó: si no, el tiempo de cada
// vuelta de loop() se sumaba a cada paso y los motores iban 8-14 % más lentos que lo pedido
// (medido el 23-nov con firmware/prueba_exigente.py). Si se atrasó más de un intervalo se vuelve
// a contar desde ahora, para no dar una ráfaga de pasos de golpe.
void pulsos(unsigned long ahora) {
  for (int j = 0; j < N; j++) {
    unsigned long iv = intervalo[j];
    if (iv == 0) {
      ultimo[j] = ahora;               // quieta: el primer paso será un intervalo después
      continue;
    }
    // Pasos solo hacia el destino, salvo si viene frenando después de pasarse (va rápido).
    if (destino[j] == pos[j] && vel[j] <= VEL_PARADA) continue;
    if (ahora - ultimo[j] < iv) continue;
    if (ahora - ultimo[j] >= 2 * iv) ultimo[j] = ahora - iv;
    int s = dirMov[j];
    if (s != sentidoPin[j]) {
      // Sentido positivo = opto de DIR encendido (LOW con positivo común, HIGH con cátodo común).
      escribir(regDir[j], mascaraDir[j], s > 0 ? OPTO_ON : OPTO_OFF);
      sentidoPin[j] = s;
      delayMicroseconds(DIR_US);
    }
    escribir(regStep[j], mascaraStep[j], OPTO_ON);      // pulso: opto encendido PULSO_US
    delayMicroseconds(PULSO_US);
    escribir(regStep[j], mascaraStep[j], OPTO_OFF);
    pos[j] += s;
    ultimo[j] += iv;
  }
}

bool ledEncendido = false;

void loop() {
  leerSerie();
  unsigned long ahora = micros();
  if (ahora - ultimoTick >= TICK_US) {
    // Cada TICK_US exactos (si no, el atraso de cada vuelta alargaba los ticks y la aceleración
    // real quedaba menor que la pedida); si se atrasó mucho, se vuelve a contar desde ahora.
    ultimoTick = (ahora - ultimoTick >= 4 * TICK_US) ? ahora : ultimoTick + TICK_US;
    rampas(ahora);
  }
  bool mov = moviendo();
  // Perro guardián: con G, si el PC dejó de hablar, se para (ver Seguridad arriba).
  if (modoG && mov && millis() - ultimoComando > WATCHDOG_MS) parar();
  if (!mov) modoG = false;
  pulsos(micros());
  // El LED solo se escribe cuando cambia (escribirlo en cada vuelta le quita tiempo a los pulsos).
  if (mov != ledEncendido) {
    digitalWrite(PIN_LED, mov ? HIGH : LOW);
    ledEncendido = mov;
  }
}
