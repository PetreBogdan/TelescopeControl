

#include <catalogs.h> // Incarcarea cataloagelor

/*Variabile ce se pot modifica datorita specificatiilor hardware*/

const unsigned long STEP_DELAY = 18699;                // Intarzierea dintre pasi pentru a urmari cerul
const unsigned long MICROSTEPS_PER_DEGREE_RA  = 12800; //Micropasi pe grad
const unsigned long MICROSTEPS_PER_DEGREE_DEC = MICROSTEPS_PER_DEGREE_RA; 

const unsigned long MICROSTEPS_RA  = 32;              //Valoarea de micropasi
const unsigned long MICROSTEPS_DEC = MICROSTEPS_RA;  

const long SERIAL_SPEED = 9600;         //Viteaza seriala
long MAX_RANGE = 1800;                  //Maxim 30 de grade la inceput

int RA_DIR   = HIGH;                    //Directia acelor de ceasornic
int DEC_DIR  = HIGH;                    

unsigned int  SLOW_SPEED      = 8;      // Viteza initiala de tracking
const    int  SLOW_SPEED_INC  = 4;      // Incrementarea vitezei de tracking

unsigned long STEP_DELAY_SLEW = 1200;   //Intarzierea in modul de SLEW

boolean SIDE_OF_PIER_WEST     = true;   //Telescopul se afla in partea de vest la inceput
boolean POWER_SAVING_MODE  = true;   //Indicele pentru modul de sleep al axei DEC


//Interfata pinilor Arduino
const int raDir    =  4;
const int raStep   =  3;
const int raButton =  6;            
const int raEnableMicrosteps  = 2;
const int decDir   = 12;
const int decStep  = 11;
const int decButton=  7;
const int decEnableMicrosteps = 9;
const int decSleep = 10;
const int SW = 8; //Butonul SW din joystick 
const int X_pin = 0; //axa X
const int Y_pin = 1; //axa y

/*Variabile ce nu se pot modifica*/

const unsigned long MICROSTEPS_PER_HOUR  = MICROSTEPS_PER_DEGREE_RA * 360 / 24; //micropasi pe ora pentru RA

const int CMR = (STEP_DELAY*16/8/2)-1; //Compare Match Register pentru intreruperile axei RA

unsigned long raPressTime  = 0;    // Timpul de cand s-a apasat butonul pentru RA
unsigned long decPressTime = 0;    // Timpul de cand s-a apasat butonul pentru DEC
unsigned long bothPressTime= 0;    // Timpul pentru apasarea ambelor butoane pentru a schimba Side of Pier
unsigned long swPressTime = 0;    //Timpul de cand s-a apasat switch-ul joystick-ului

unsigned long decLastTime  = 0;    // Timpul de cand s-a schimbat starea semnalului DEC
boolean raStepStatus    = false;// true = HIGH, false = LOW
boolean decStepStatus   = false;// true = HIGH, false = LOW

boolean SLEWING = false; // true atunci cand suntem in modul Slew pentru a opri trackingul

const long DAY_SECONDS =  86400; // secunde intr-o zi
const long NORTH_DEC   = 324000; // 90°

// Current coords in Secs (default to true north)
long currRA  = 0;     //default pentru LX200
long currDEC = NORTH_DEC;

int raSpeed  = 1;   // Viteza de inceput al axei RA
int decSpeed = 0;   // Viteza de inceput al axei DEC (este oprit la inceput)

// Serial Input
char input[20];     // Stocheaza strigurile de la intrare
int  in = 0;        // Caracterul curent din string

long inRA    = 0; //Variabile pentru coordonate noi
long inDEC   = 0;

String lx200RA = "00:00:00#"; //Coordonatele in formatul LX200
String lx200DEC= "+90*00:00#";

//Variabilele pentru acceleratie
unsigned long MAX_DELAY = 16383; //limita pentru delayMicroseconds()
unsigned long decStepDelay   = MAX_DELAY; 
unsigned long decTargetDelay = STEP_DELAY/SLOW_SPEED;
unsigned int  decPlayIdx = 0;

String _TelescopeControl = "TelescopeControl";
const unsigned long _ver = 100000;

void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println(_TelescopeControl);
  // Declararea pinilor de output pentru motoare
  pinMode(raStep, OUTPUT);
  pinMode(raDir,  OUTPUT);
  pinMode(raEnableMicrosteps, OUTPUT);
  pinMode(decStep, OUTPUT);
  pinMode(decDir,  OUTPUT);
  pinMode(decEnableMicrosteps, OUTPUT);
  pinMode(decSleep, OUTPUT);
  // La inceput axa DEC este in Sleep
  decSleepMode(true);
  // Directia este in sensul acelor de ceasornic pentru RA
  digitalWrite(raDir, RA_DIR);
  // Nu se trimit semnale de pas pentru motoare 
  digitalWrite(raStep,  LOW);
  digitalWrite(decStep, LOW);
  // Aplicarea modului de micropasi pentru motoare (1/32)
  digitalWrite(raEnableMicrosteps, HIGH);
  digitalWrite(decEnableMicrosteps, HIGH);
  // Aplicarea rezistentei interne pentru butoane, fara sa fie nevoie de rezistente in schema
  pinMode(raButton,  INPUT_PULLUP);
  pinMode(decButton, INPUT_PULLUP);
  // Aprinderea ledului pentru 0.5 secunde la pornire
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  RaTimer(CMR); //inceperea trackingului pentru axa RA
  lx200DEC[3] = char(223); // Codul in ASCII pentru grade
  Serial.println("READY!");

  pinMode(SW, INPUT); //switch-ul joystick-ulului
  digitalWrite(SW, HIGH); //Setat pe 1 logic, 0 logic apasat
}



//Timer pentru trackingul axei RA
void RaTimer(int cmr) {
  /*  STEP_DELAY/2 = 9349 => 1000000/9349 =  106.9575Hz 
   *  CMR = 16000000/(prescaler*(1000000/(STEP_DELAY/2)))-1;
   *  => CMR = 145.0959 with prescaler=1024, 18699  with prescaler=0
   *  let's go for the finest one
   */
  /* Using 8-but Timer 2 (CMR less than 256)
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  OCR2A = CMR; // compare match register
  TCCR2A |= (1 << WGM21); // turn on CTC mode ( Clear Timer on Compare Match)
  TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
  */
  cli(); // disable interrupts
  // Set Timer1 (16-bit)
  TCCR1A = 0; // reset register to 0
  TCCR1B = 0; // 
  TCNT1  = 0; // initialize counter
  OCR1A = cmr; // compare match register
  TCCR1B |= (1 << WGM12); // CTC mode ( Clear Timer on Compare Match)
  //TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 prescaler
  //TCCR1B |= (1 << CS11) | (1 << CS10);  // 64 prescaler
  TCCR1B |= (1 << CS11);   // 8 prescaler
  TIMSK1 |= (1 << OCIE1A); // timer compare interrupt 
  sei(); // enable interrupts
}

//Schimbarea starii motorului pentru tracking
ISR(TIMER1_COMPA_vect){
  if (!SLEWING) { // change pulse only when not slewing
    raStepStatus = !raStepStatus;
    digitalWrite(raStep, (raStepStatus ? HIGH : LOW));
  }
}


//Schimbarea starii axei DEC, si schimbarea vitezei
void dec_Move() {

  unsigned long dt  = micros() - decLastTime; // timpul ce a trecut de la ultimul semnal
  long rttcp = (decStepDelay/2) - dt;         // timpul ramas pentru a schimba pulsul

  if ( rttcp < 200 && rttcp > 0 ) { //mai putin de 200 us pentru a schimba semnalul
    delayMicroseconds(rttcp); // Asteapta dupa cele 200us
    decStepStatus = !decStepStatus;
    digitalWrite(decStep, (decStepStatus ? HIGH : LOW));
    decLastTime = micros(); // Reseteaza timpul
    if (decPlayIdx<=100) { // Accelereaza primele 100 de pulsuri
      // Schimba timpul de delay de la delay-ul maxim pana la delay-ul dorit
      unsigned int i = decPlayIdx/2; // 0, 0, 1,1,2,2,3,3,..., 50, 50
      decStepDelay = MAX_DELAY-( (MAX_DELAY-decTargetDelay)*i/50); 
      decPlayIdx++; 
    }
  } else if ( rttcp < 0 ) { // eroare
    decLastTime = micros(); //reseteaza timpul
    Serial.println("Timpul pentru DEC este prea mare");
  }
}

//Calculul si miscarea axelor in secunde
int calculateRaDecSec(long raSec, long decSec) {

  // Mai mult de 12 ore atunci sensul este invers
  if (abs(raSec) > DAY_SECONDS/2) {
    raSec = raSec+(raSec>0?-1:1)*DAY_SECONDS;
    Serial.print("New RA in secs ");
    Serial.println(raSec);
  }

  // Verifica range-ul maximF
  if ( (abs(decSec) > (MAX_RANGE*60)) || ( abs(raSec) > (MAX_RANGE*4)) ) {
    return 0; //Nu se afla in range-ul maxim atunci returneaza 0 si nu face nimic
  }
  
  // Directiile pentru Slew
  digitalWrite(raDir,  (raSec  > 0 ? RA_DIR :(RA_DIR ==HIGH?LOW:HIGH)));
  digitalWrite(decDir, (decSec > 0 ? DEC_DIR:(DEC_DIR==HIGH?LOW:HIGH)));


  // Calculul micropasilor necesari
  unsigned long raSteps  = (abs(raSec) * MICROSTEPS_PER_HOUR) / 3600;
  unsigned long decSteps = (abs(decSec) * MICROSTEPS_PER_DEGREE_DEC) / 3600;

  // Calculul pasilor intregi si micropasilor pentru ajustare
  unsigned long raFullSteps   = raSteps / MICROSTEPS_RA;              //trunchiaza rezultatul
  unsigned long raMicroSteps  = raSteps - raFullSteps * MICROSTEPS_RA; // restul
  unsigned long decFullSteps  = decSteps / MICROSTEPS_DEC;         
  unsigned long decMicroSteps = decSteps - decFullSteps * MICROSTEPS_DEC; 

  // Dezactivarea micropasilor pentru modul de pas intreg
  digitalWrite(raEnableMicrosteps, LOW);
  digitalWrite(decEnableMicrosteps, LOW);

  // Pasii intregi
  Serial.println("Moving full steps:");
  Serial.print("raFullSteps = ");
  Serial.println(raFullSteps);
  Serial.print("decFullSteps = ");
  Serial.println(decFullSteps);
  unsigned long slewTime = micros(); //Durata timpului de SLEW pentru ajustarea axei RA
  slewRaDec(raFullSteps, decFullSteps);

  // Activarea micropasillor
  digitalWrite(raEnableMicrosteps, HIGH);
  digitalWrite(decEnableMicrosteps, HIGH);

 //Ajustarile finale de micropasi
  Serial.println("Moving microsteps:");
  Serial.print("raMicroSteps = ");
  Serial.println(raMicroSteps);
  Serial.print("decMicroSteps = ");
  Serial.println(decMicroSteps);
  slewRaDec(raMicroSteps, decMicroSteps);

  //Daca durata de slew este mai mare de 5 secunde atunci se ajusteaza axa RA
  //Se face acest lucru deoarece telescopul nu mai face tracking si se ajusteaza acest lucru
  slewTime = micros() - slewTime; // Timpul de SLEW
  if ( slewTime > (5 * 1000000) ) {
    Serial.println("Cautarea a durat mai mult de 5 secunde.");
    Serial.print("Se ajusteaza axa RA ");
    Serial.println(slewTime/1000000);
    calculateRaDecSec(slewTime / 1000000, 0); 
  }

  // Reseteaza directia axei RA
  digitalWrite(raDir,  RA_DIR);

  // Succes
  return 1;
}

//Miscarea axelor calculate in pasi in functia de mai sus
void slewRaDec(unsigned long raSteps, unsigned long decSteps) {
  digitalWrite(LED_BUILTIN, HIGH);
  SLEWING = true;

  //Trezirea motorului DEC daca este nevoie
  if (decSteps != 0) {
    decSleepMode(false);
  }

  unsigned long delaySlew = 0; 
  unsigned long delayLX200Micros = 0; //Masoara timpul de intarziere provenit de la comunicatia cu LX200
  in = 0; //index
    
  for (unsigned long i = 0; (i < raSteps || i < decSteps) ; i++) {
    if ((i<100)) { // Accelereaza primii 100 de pasi
      delaySlew = MAX_DELAY-( (MAX_DELAY-STEP_DELAY_SLEW)/100*i);
    } else if ( (i>raSteps-100 && i<raSteps)|| (i>decSteps-100 && i<decSteps)) {
      delaySlew = STEP_DELAY_SLEW*2; //de doua ori mai rapid pentru ultimii 100 pasi
    } else { 
      delaySlew = STEP_DELAY_SLEW; // full speed
    } 
    
    if (i < raSteps)  { digitalWrite(raStep,  HIGH); }
    if (i < decSteps) { digitalWrite(decStep, HIGH); }
    delayMicroseconds(delaySlew);
    
    if (i < raSteps)  { digitalWrite(raStep,  LOW);  }
    if (i < decSteps) { digitalWrite(decStep, LOW);  }
    
    // Update la coordonate pentru LX200
    delayLX200Micros = 0;
    if (Serial.available() > 0) {
      delayLX200Micros = micros();
      input[in] = Serial.read();
      if (input[in] == '#' && in > 1 ) {
        if (input[in-1] == 'R') { // :GR# comanda de update la axa RA
          Serial.print(lx200RA);
        } else if (input[in-1] == 'D') { // :GD# pentru axa DEC
          Serial.print(lx200DEC);
        } else if (input[in-1] == 'Q') { // :Q# STOP
          break;
        }
        in = 0;
      } else {
        if (in++ >5) in = 0;
      }
      delayLX200Micros = micros()-delayLX200Micros;
      if (delayLX200Micros>delaySlew) Serial.println("Comunicarea LX200 este prea inceata");
    } 
    delayMicroseconds(delaySlew-delayLX200Micros);
  }
  // Seteaza DEC inapoi in modul SLEEP
  if (decSteps != 0) {
    decSleepMode(true);
  }
  digitalWrite(LED_BUILTIN, LOW);
  SLEWING = false;
}


//Functia de SLEEP pentru DEC
void decSleepMode(boolean b) {
  if (POWER_SAVING_MODE) {
    if (b == false) { // wake up!
      digitalWrite(decSleep, HIGH);
      delayMicroseconds(2000); // DRV8825 are nevoie de 1,7ms pentru a se trezi
    } else {
      digitalWrite(decSleep, LOW); // LOW pentru sleep
    }
  } else {
    digitalWrite(decSleep, HIGH);
  }
}


//Protocolul LX200
void lx200(String s) { //Comenzi de tip :#
  if (s.substring(1,3).equals("GR")) { // :GR#  
    // Trimite coordonatele RA
    Serial.print(lx200RA);
  } else if (s.substring(1,3).equals("GD")) { // :GD# 
    // Trimite coordonatele pentru DEC
    Serial.print(lx200DEC);
  } else if (s.substring(1,3).equals("GV")) { //Ia datele dispozitivului
    char c = s.charAt(3); 
    if ( c == 'P') {// GVP Ia numele dispozitivului
       Serial.print(_TelescopeControl);  
    } else if (c == 'N') { // GVN Ia versiunea firmwareului, setata ca 100000
       Serial.print(_ver);  
    }
    Serial.print('#');
  } else if (s.substring(1,3).equals("Sr")) { // :SrHH:MM:SS# or :SrHH:MM.T# 
    // Seteaza coordonatele initiale pentru RA
    long hh = s.substring(3,5).toInt();
    long mi = s.substring(6,8).toInt();
    long ss = 0;
    if (s.charAt(8) == '.') { // :SrHH:MM.T#
      ss = (s.substring(9,10).toInt())*60/10;
    } else {
      ss = s.substring(9,11).toInt();
    }
    inRA = hh*3600+mi*60+ss;
    Serial.print(1); // Eroare, nu s-a putut extrage infromatia
  } else if (s.substring(1,3).equals("Sd")) { // :SdsDD*MM:SS# or :SdsDD*MM#
    // Coordonatele initiale pentru DEC
    long dd = s.substring(4,6).toInt();
    long mi = s.substring(7,9).toInt();
    long ss = 0;
    if (s.charAt(9) == ':') { ss = s.substring(10,12).toInt(); }
    inDEC = (dd*3600+mi*60+ss)*(s.charAt(3)=='-'?-1:1);

    if (currDEC == NORTH_DEC) { // Seteaza initial pozitia telescopului catre nordul real
      // Sincronizarea initiala a coordonatelor
      currRA  = inRA;
      currDEC = inDEC;
      updateLx200Coords(currRA, currDEC);
    }
    Serial.print(1); 
  } else if (s.substring(1,3).equals("MS")) { // :MS# misca telescopul
    
    long deltaraSec  = currRA-inRA;
    long deltadecSec = currDEC-inDEC;
    
    Serial.print(0); // Miscarea este posibila daca se returneaza 0 
   
    if (calculateRaDecSec(deltaraSec, deltadecSec) == 1) { // Daca s-a terminat cu succes         
      currRA  = inRA;
      currDEC = inDEC;
      updateLx200Coords(currRA, currDEC); // recompute strings
    } else { 
      Serial.print("1Range_too_big#");
    }
  } else if (s.substring(1,3).equals("CM")) { // :CM# sincronizeaza telescopul

    currRA  = inRA;
    currDEC = inDEC;
    Serial.print("Synced#");
    updateLx200Coords(currRA, currDEC); 
  }
}


//Update la coordonate pentru protocolul LX200
void updateLx200Coords(long raSec, long decSec) {
  unsigned long pp = raSec/3600;
  unsigned long mi = (raSec-pp*3600)/60;
  unsigned long ss = (raSec-mi*60-pp*3600);
  lx200RA = "";
  if (pp<10) lx200RA.concat('0');
  lx200RA.concat(pp);lx200RA.concat(':');
  if (mi<10) lx200RA.concat('0');
  lx200RA.concat(mi);lx200RA.concat(':');
  if (ss<10) lx200RA.concat('0');
  lx200RA.concat(ss);lx200RA.concat('#');

  pp = abs(decSec)/3600;
  mi = (abs(decSec)-pp*3600)/60;
  ss = (abs(decSec)-mi*60-pp*3600);
  lx200DEC = "";
  lx200DEC.concat(decSec>0?'+':'-');
  if (pp<10) lx200DEC.concat('0');
  lx200DEC.concat(pp);lx200DEC.concat(char(223)); 
  if (mi<10) lx200DEC.concat('0'); 
  lx200DEC.concat(mi);lx200DEC.concat(':');
  if (ss<10) lx200DEC.concat('0');
  lx200DEC.concat(ss);lx200DEC.concat('#');
 } 


//Scrie pe serial informatiile 
void printInfo() {
  Serial.print("Pozitia curenta:: ");
  printCoord(currRA, currDEC);
  Serial.print("Side of Pier: ");
  Serial.println(SIDE_OF_PIER_WEST?"W":"E");
  Serial.print("Viteza de urmarire: ");
  Serial.println(SLOW_SPEED);
  Serial.print("Range maxim: ");
  Serial.println(MAX_RANGE/60);
  Serial.print("Modul Sleep DEC: ");
  Serial.println(POWER_SAVING_MODE?"enabled":"disabled");
}


//Comenzile interne
void tgoto(String s) {
  //sleep, range, speed, side, info
   if (s.substring(1,6).equals("sleep")) {
    POWER_SAVING_MODE = (s.charAt(0) == '+')?true:false;
    if (POWER_SAVING_MODE) {
      Serial.println("Modul de Sleep a fost activat");
      digitalWrite(decSleep, LOW); // Pune axa DEC in modul sleep
    } else {
      Serial.println("Modul de Sleep a fost dezactivat");
      digitalWrite(decSleep, HIGH); // Scoate axa DEC din modul sleep
    }
                         
  } else if (s.substring(1,6).equals("range")) {
    int d = (s.charAt(0) == '+')?15:-15;
    if (MAX_RANGE+d > 0 ) {
      MAX_RANGE = MAX_RANGE+d*60;
      Serial.print("Noul range in grade este: ");
      Serial.println(MAX_RANGE/60);
    } else {
      Serial.println("Nu se poate seta un range mai mic ca 0"); //nu poate sa seteze un range mai mic de 0
    }
  } else if (s.substring(1,6).equals("speed")) {
    int d = (s.charAt(0) == '+')?(SLOW_SPEED_INC):(-SLOW_SPEED_INC);
    if (SLOW_SPEED+d > 0 ) {
      SLOW_SPEED  = SLOW_SPEED+d;
      decTargetDelay = STEP_DELAY/SLOW_SPEED; 
      Serial.print("Viteza de urmarire setata la: ");
      Serial.println(SLOW_SPEED);
    } else {
      Serial.println("Can't set speed to zero");
    }
  } else if (s.substring(1,5).equals("side")) {
    changeSideOfPier();
  } else if (s.substring(1,5).equals("info")) {
    printInfo();
  } else { 
    //Comenzile de Move sau de Set
    long deltaraSec  = 0; // Secundele sa mute axa RA
    long deltadecSec = 0; // Secundele sa mute axa DEC

    if ((s.charAt(0) == '+' || s.charAt(0) == '-') && (s.charAt(5) == '+' || s.charAt(5) == '-')) { // semnele diferentelor de coordonate
      // toInt() converteste string in int
      if (!s.substring(1, 5).equals("0000")) {
        deltaraSec = s.substring(1, 5).toInt() * (s.charAt(0) == '+' ? +1 : -1) * 4;
        if (deltaraSec == 0) { Serial.println("Problema la coordonatele RA"); return; }
      }
      if (!s.substring(6, 10).equals("0000")) {
        deltadecSec = s.substring(6, 10).toInt() * (s.charAt(5) == '+' ? +1 : -1) * 60;
        if (deltadecSec == 0) { Serial.println("Problema la coordonatele DEC"); return; }
      }
      long tmp_inRA = currRA + deltaraSec;  // calculeaza noua pozitie
      long tmp_inDEC = currDEC + deltadecSec;
      if ( tmp_inRA<0 || tmp_inRA>86400 || abs(tmp_inDEC)>324000 ) { //daca trece de 24 de ore sau 90 de grade
        Serial.println("Values out of range"); return; 
      } else {
        inRA = tmp_inRA ;
        inDEC = tmp_inDEC;      
      }
    } else { // Decodifica coordonatele pentru variabilele de coordonate curente
      if (s.charAt(1) == 'M' || s.charAt(1) == 'm') { // Catalogul Messier
        int m = s.substring(2,5).toInt(); // toInt() returns 0 if conversion fails
        if (m == 0 || m > 110) { Serial.println("Obiectul Messier nu se gaseste in catalog"); return; } 
        //Citeste din librarie
        inRA = (long) pgm_read_dword(&(Messier[m].ra));
        inDEC= (long) pgm_read_dword(&(Messier[m].dec));
        Serial.print(s[0]=='s'?"Set ":"Goto ");
        Serial.print("M");Serial.println(m);
      } else if (s.charAt(1) == 'S' || s.charAt(1) == 's') { // catalogul de stele
        int n = s.substring(2,5).toInt(); 
        if (n < 0 || n > 244) { Serial.println("Steaua selectata nu se gaseste in catalog"); return; } 
        //Citeste din librarie
        inRA = (long) pgm_read_dword(&(Stars[n].ra));
        inDEC= (long) pgm_read_dword(&(Stars[n].dec));
        Serial.print(s[0]=='s'?"Set ":"Goto ");
        Serial.print("Star ");Serial.println(n);
      } else if (s.charAt(1) == 'N' || s.charAt(1) == 'n') { // Catalogul NGC
        int n = s.substring(2,6).toInt(); 
        if (n < 0 || n > 7840) { Serial.println("Obiectul NGC nu s-a gasit in catalog"); return; } 
        //Algoritmul de cautare binara
        int ngcElem = ngcLookup(n);
        if (ngcElem>0) {
          //Citeste din librarie
          inRA  = (long) pgm_read_dword(&(NGCs[ngcElem].ra ));
          inDEC = (long) pgm_read_dword(&(NGCs[ngcElem].dec));
          Serial.print(s[0]=='s'?"Set ":"Goto ");
          Serial.print("NGC");Serial.println(n);
        } else {
          Serial.println("Obiectul NGC nu s-a gasit in catalog");
          return;
        }
      } else { // HHMMSSdDDMMSS d este semnul pentru axa DEC
        inRA  = s.substring(1, 3).toInt() * 60 * 60 + s.substring(3, 5).toInt() * 60 + s.substring(5, 7).toInt();
        inDEC = (s.charAt(7) == '+' ? 1 : -1) * (s.substring(8, 10).toInt() * 60 * 60 + s.substring(10, 12).toInt() * 60 + s.substring(12, 14).toInt());
        if (inRA == 0 || inDEC == 0) { Serial.println("Eroare la conversia coordonatelor"); return; }
      }
      
      // Variabilele inRA si inDEC sunt setate
      if (s.charAt(0) == 's') { 
          currRA  = inRA;
          currDEC = inDEC;
          Serial.print("Pozitia curenta: ");
          printCoord(currRA, currDEC);
        } else { //Diferenta de coordonate pentru SLEW
          deltaraSec  = currRA-inRA;
          deltadecSec = currDEC-inDEC;
        }
      }

      //Verifica daca delta este 0
      if (deltaraSec != 0 || deltadecSec != 0) { 
        //Miscarea axelor
        Serial.println("moving...");
        if ( calculateRaDecSec(deltaraSec, deltadecSec) == 1) { // Cu succes
          Serial.println("Done!");
          // Seteaza noua pozitie
          currRA  = inRA;
          currDEC = inDEC;
          Serial.print("Pozitia curenta: ");
          printCoord(currRA, currDEC);
        } else{ 
          Serial.print("Eroare de nerespectarea range-ului: "); //eroare de nerespectarea Rangeului
          Serial.println(MAX_RANGE/60);
        }
      }
  }
}

//Functia de printare a coordonatelor
void printCoord(long raSec, long decSec) {
  long pp = raSec/3600;
  Serial.print(pp);
  Serial.print("h");
  long mi = (raSec-pp*3600)/60;
  if (mi<10) Serial.print('0');
  Serial.print(mi);
  Serial.print("'");
  long ss = (raSec-mi*60-pp*3600);
  if (ss<10) Serial.print('0');
  Serial.print(ss);
  Serial.print("\" ");
  pp = abs(decSec)/3600;
  Serial.print((decSec>0?pp:-pp));
  Serial.print("°");
  mi = (abs(decSec)-pp*3600)/60;
  if (mi<10) Serial.print('0');
  Serial.print(mi);
  Serial.print("'");
  ss = (abs(decSec)-mi*60-pp*3600);
  if (ss<10) Serial.print('0');
  Serial.print(ss);
  Serial.println("\"");
}


//Schimbarea Side of Pier
void changeSideOfPier() {
  SLEWING = true; // Se opresc motoarele
  SIDE_OF_PIER_WEST = !SIDE_OF_PIER_WEST;
  Serial.print("Side of Pier: ");
  Serial.println(SIDE_OF_PIER_WEST?"W":"E");

  //Schimba directia axei DEC
  DEC_DIR = (DEC_DIR==HIGH?LOW:HIGH);
  
  // Aprinde ledul pentru 3 secunde
  if (SIDE_OF_PIER_WEST) { // turn on led for 3 secs
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(3000);digitalWrite(LED_BUILTIN, LOW);
  } else { //Becul se aprinde si se stinge de 2 ori dupa terminarea schimbarii
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); digitalWrite(LED_BUILTIN, LOW);
    delay(1000); digitalWrite(LED_BUILTIN, HIGH);
    delay(1000); digitalWrite(LED_BUILTIN, LOW);
  }
  bothPressTime = 0; //Dureaza o secunda aceasta mmiscare si este nevoie de resetarea timpului
  digitalWrite(raDir, RA_DIR); // Seteaza directia axei RA la cea initiala pentru tracking
  SLEWING = false;
}


//Miscarea axelor manual folosind Joystickul
void manualMode() {
  SLEWING = true;
  
    decSleepMode(false);

 //Pasi normali, micropasi sunt prea inceti

  while (analogRead(X_pin) >700){
    digitalWrite(raDir, HIGH);
    digitalWrite(raEnableMicrosteps, LOW);
    digitalWrite(raStep,  HIGH);
    delayMicroseconds(STEP_DELAY_SLEW);
    digitalWrite(raStep,  LOW);
    
  }


  while (analogRead(X_pin) < 300){
    digitalWrite(raDir, LOW);
    digitalWrite(raEnableMicrosteps, LOW);
    digitalWrite(raStep,  HIGH);
    delayMicroseconds(STEP_DELAY_SLEW);
    digitalWrite(raStep,  LOW);
    
  }

  while (analogRead(Y_pin) > 700){
    digitalWrite(decDir, HIGH);
    digitalWrite(decEnableMicrosteps, LOW);
    digitalWrite(decStep,  HIGH);
    delayMicroseconds(STEP_DELAY_SLEW);
    digitalWrite(decStep,  LOW);
    
  }

  while (analogRead(Y_pin) < 300){
    digitalWrite(decDir, LOW);
    digitalWrite(decEnableMicrosteps, LOW);
    digitalWrite(decStep,  HIGH);
    delayMicroseconds(STEP_DELAY_SLEW);
    digitalWrite(decStep,  LOW);
    
  }


  
  
  digitalWrite(raEnableMicrosteps, HIGH);
  digitalWrite(decEnableMicrosteps, HIGH);

  
    decSleepMode(true);
 
  
  digitalWrite(raDir, RA_DIR);
  SLEWING=false;
  
}
 
//Functia main loop()
void loop() {
  
  //Porneste axa DEC daca este nevoie
  if (decSpeed != 0) {
    dec_Move();
  }

  // Schimba Side of Pier cand cele doua butoane sunt apasate
  if ( (digitalRead(raButton) == LOW) && digitalRead(decButton) == LOW) {
    if (bothPressTime == 0) { bothPressTime = micros(); }
    if ( (micros() - bothPressTime) > (1000000) ) { 
      changeSideOfPier();
      //Reseteaza viteza motoarelor
      raSpeed = 1;
      RaTimer(CMR/raSpeed); // reset ra to 1x
      decSpeed = 0; // Opreste axa DEC
      decSleepMode(true);
    }
  } else {
    // Daca nu sunt ambele butoane apasate reseteaza timpul
    bothPressTime = 0;  
  }
  
  //Apasarea butonului RA, nu face nimic daca s-a apasat prea repede
  if ( (digitalRead(raButton) == LOW)  && (micros() - raPressTime) > (300000) ) {
    raPressTime = micros();
    // 1x 
    if (raSpeed == 1) {
      raSpeed = SLOW_SPEED;
      digitalWrite(raDir, RA_DIR);
    } else if  (raSpeed == SLOW_SPEED) {
      raSpeed = (SLOW_SPEED - 2);
      digitalWrite(raDir, (RA_DIR==HIGH?LOW:HIGH)); // Schimba directia pentru ajustare
    } else if  (raSpeed == (SLOW_SPEED - 2)) {
      raSpeed = 1;
      digitalWrite(raDir, RA_DIR);
    }
    RaTimer(CMR/raSpeed); //Repara intreruperile
  }
  
  //Apasarea butonului DEC, nu face nimic daca s-a apasat prea repede 
  if (digitalRead(decButton) == LOW && (micros() - decPressTime) > (300000) ) {
    decPressTime = micros();  
    // 0x 
    if (decSpeed == 0) {
      decSpeed = SLOW_SPEED;
      decSleepMode(false); // porneste motorul
      digitalWrite(decDir, DEC_DIR);
    } else if (decSpeed == SLOW_SPEED) {
      decSpeed = (SLOW_SPEED - 1);
      digitalWrite(decDir, (DEC_DIR==HIGH?LOW:HIGH));
    } else if  (decSpeed == (SLOW_SPEED - 1)) {
      decSpeed = 0; // Se opreste
      decSleepMode(true); // Modul Sleep
    }
    decStepDelay = MAX_DELAY; 
    decPlayIdx = 0;
    decLastTime = micros();
  }

  //Verifica daca exista ceva pe serial
  if (Serial.available() > 0) {
    input[in] = Serial.read(); 

    //Nu se iau in considerare spatiile libere
    if (input[in] == ' ') return; 
    
    
    if (input[in] == char(6)) { Serial.print("P"); return; } // Modul telescopului (mod Polar)

    if (input[in] == '#' || input[in] == '\n') { //verifica daca este comanda interna sau lx200
      if ((input[0] == '+' || input[0] == '-' 
        || input[0] == 's' || input[0] == 'g')) { //comanda interna
        tgoto(input);
      } else if (input[0] == ':') { // lx200
        lx200(input);
      } else {
        //Comanda invalida
        if (in > 0) {
          String s = input;
          Serial.print(s.substring(0,in));
          Serial.println(" unknown. Expected lx200 or internal commands");
        }
      }
      in = 0; 
    } else {
      if (in++>20) in = 0; //incrementeaza sau reseteaza pozitia caracterului
    } 
  }

  if (digitalRead(SW) == LOW && (micros() - decPressTime) > (300000) ) { //daca switch-ul joystick-ului a apasat
    
    
    swPressTime = micros(); //reseteaza timpul
    manualMode(); //modul manual
    Serial.println("Modul manual terminat, este nevoie de resetarea pozitiei.");
   }
}
