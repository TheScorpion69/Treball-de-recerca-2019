/*******************************************************************************
  Treball de recerca Novembre 2019
  Marc Boné

  El codi següent controla un braç robòtic amb 6DOF (degrees of freedon, graus de
  llibertat), és a dir, sis articulacions. Les articulacions estan numerades del 0
  al 5 començant per la base, per tant la base serà l'articulació número 0 i el
  ganxo la número 5.

  El protocol seguit per rebre les dades a través de Bluetooth és el següent:
  El dispositiu enviarà un nombre de 4 o menys digits, acabant el seu enviament
  amb un salt de línia, el primer digit indicarà quin motor es desitja moure,
  seguit dels 3 digits que indicaran els graus als que es vol arribar.
  Les posicions inicials estan calibrades per a aquest robot en concret, s'han de
  calibrar novament si es vol fer servir amb un altre braç de 6DOF.
*******************************************************************************/

#include<Servo.h>

int rpos[6] = {45, 160, 0, 150, 180, 45}; //Array amb les posicions inicials de cada servo motor,
//dependran de la calibració dels motors en el muntatge del braç.

class myServo {
    int startpos;
    Servo servo;
    bool smooth; // En moure els motors més de 45 graus es farà que accelerin al començament i frenin al final del recorregut,
                 // això evita que els moviments siguin massa bruscs, aquesta variable serveix per a no aplicar això als dos últims
                 // motors, ja que el seu moviment és tant curt que no val la pena alentir-los, no passarà res perque vagin ràpid.

  public:

    void startingpos () { //Aquesta funció posa el servo motor a la seva posició inicial.
      servo.write(startpos);
    }

    void init(int pin, int restpos, bool smooth1 = true) { //Aquesta funció inicialitza els servo motors i les variables necessàries pel seu control.
      servo.attach(pin);
      smooth = smooth1;
      startpos = restpos;
    }

    int f(double x) {
      return 80 * x * x - 80 * x + 30;
    }
    // Aquesta funció retorna el valor del delay entre graus, és a dir, controla la velocitat dels moviments.
    // És la funció que he calculat préviament per a tenir 30ms a l'inici i al final i 10ms en el tram més ràpid.

    void move(int deg) { // Aquesta funció controla el moviment dels servo motors, és la part central del codi.
      int current_deg = servo.read();
      if (deg > current_deg) {
        int desp = deg - current_deg;
        for (int i = 0; i < desp; ++i) {
          servo.write(current_deg + i);
          if (desp > 45 && smooth) delay(f((double)(i / desp)));
          else delay (15);
        }
      } else {
        int desp = current_deg - deg;
        for (int i = 0; i < desp; ++i) {
          servo.write(current_deg - i);
          if (desp > 45 && smooth) delay(f((double)(i / desp)));
          else delay(15);
        }
      }
    }
};

myServo a[6];

void setup() {
  Serial.begin(9600); // S'inicia la comunicació per Bluetooth.
  while (!Serial);

  int pins[6] = {3, 5, 6, 9, 10, 11}; // Array amb els pins en els que estan connectats els servo motors, depén del circuit.

  for (int i = 0; i < 6; ++i) { // Inicialització de tots els servo motors.
    a[i].init(pins[i], rpos[i], i != 4 && i != 5);
  }

  for (int i = 5;  i >= 0; --i) { // Es posen tots els servo motors a la seva respectiva posició inicial.
    a[i].startingpos();
    delay(50);
  }
}

void loop() {
  int inputdeg = 0; // Inicialització de totes les variables que rebran la informació a través de Bluetooth.
  char buff[4];
  char motor;

  while (Serial.available() <= 0); // Bucle que atura el codi fins que arribi algun valor a través de Bluetooth.

  do { //Bucle que atura el codi fins que el valor rebut sigui vàlid, eliminant així els salts de línia que podrien interferir amb les dades.
    motor = Serial.read() - '0';
  } while (motor < 0 || motor > 9);

  const int n = Serial.readBytesUntil('\n', buff, 3); // Aquesta variable contindrà el nombre de digits dels graus que es volen moure.
                                                      // La mateixa instrucció també posarà els valors dels graus en l'array buff, inicialitzat préviament.

  for (int i = 0; i < n; ++i) inputdeg = inputdeg * 10 + (buff[i] - '0'); // Aquest Bucle converteix els caracters de l'array buff en int (integers) i els introdueix a la variable inputdeg

  a[motor].move(inputdeg); // Instrucció que envia a la funció move de la classe creada quin motor s'ha de moure i la informació necessària per fer-ho.
}
