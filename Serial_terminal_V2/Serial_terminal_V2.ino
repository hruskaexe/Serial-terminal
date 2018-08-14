/*
  Serial_terminal.ino - code for load serial communication data to display.
  Created by Tomáš Hruška, August 9, 2018.
  Released into the public domain.
*/

/* při kompilaci je nutno mít sériovou komunikaci odpojenou
   přijatá data se ukládají do pole receivedData, délka přijatých dat lze přizpůsobit změnou x a délkou pole receivedData
   použitá LED dioda - dvoubarevná, červená a zelená barva, barvy se mění dle směru průchodu el. proudu

   Popis programu:
   - 3 tlačítka s deboucem - nahoru, dolu, OK
   - menu pro výběr baud rate nebo zadání vlastního (6 čísel)
   - menu zvolení detekce CR, LF, CR + LF, žádná
   - stisknutí tlačítka OK při komunikaci - pozastaví displej
   - když je ve stavu pozastavení - držení tlačítka OK načte nový řádek a znova pozastaví
                                  - pouhé stisknutí spustí načítání komunikace dále
   - stisknutí tlačítka nahoru při komunikaci - vrátí se na začátek programu na výběr baud rate
   - stisknutí tlačítka dolu při komunikaci - vrátí se na výběr typu detekce
   - při pozastavení a spuštění - pokračuje ve zpracovávání přijatých dat, po tom načte nová data
                                - při zpracování všech přijatých dat načte vždy aktuální data
   - zelená LED značí příjímaní a zpracovávání přijatých dat, červená zastavení komunikace
*/

#include <ST7565.h>            //knihovna pro LCD display
//#define SERIAL_BUFFER_SIZE 256 //možnost zvětšení buffru pro seriovou komunikaci - klasicky se rovná 64

//=====POČÁTEČNÍ DEFINOVÁNÍ======================================================================================================
//===============================================================================================================================

//=====definování pinů======================
const byte buttonPin1 = 10;         //pin talčítka 1 - dolu
const byte buttonPin2 = 11;         //pin talčítka 2 - OK
const byte buttonPin3 = 12;        //pin talčítka 3 - nahoru
const byte ledPin1 = 8;           //pin 1 LED diody - anoda (+)
const byte ledPin2 = 9;           //pin 2 LED diody - katoda (-)
ST7565 glcd(3, 4, 5, 6, 7);       //piny LCD displeje

//=======definování proměnných==============
byte buttonState1, buttonState2, buttonState3; // stav tlačítek - 0 = rozepnuto, 1 = sepnuto
byte detekce;                                  // stav detekce - 1 = CR, 2 = LF, 3 = CR + LF, 4 = žadná
short vyberX = 0;                                   // horizontální pozice znaků pro LCD display
byte vyberY = 1;                                   // vertikální pozice znaků pro LCD display
byte selectNum[6];                             // pole pro uložení vybraných čísel při volbě vlastního Baud Rate
char selectChar[6];                           // pole pro uložení konverze "selectNum" z typu int na char
short pom, pom2, pom3, i, j;                          // pomocné proměnné pro podmínky a smyčky
long baudRate;                                // zvolená Baud Rate rychlost sériové komunikace
byte debounceDelay = 25;                 // čekající doba pro Debounce tlačítek (v ms) (při registraci zákmitu tlačítka zvětšit)
const byte displayDelay = 5;                   // čekací doba LCD displeje mezi zapsání na displej a vymazání mezipaměti displeje (v ms)
const byte delayReload = 60;                //čekání pro pomalejší pohyb s šípkou a zamezení rychlých změn na displeji při zmáčknutí tlačítka

short x = 189;                                  //celkový počet znaků displeje, volně nastavitelný, výsledný počet znaků vypsatelných na displej = x - 21
char receivedData[189];                         //ukládání přijatých dat, velikost pole se musí rovnat x

char line0[21], line1[21], line2[21], line3[21], line4[21], line5[21], line6[21], line7[21];  //řádky displeje - jeden řádek 21 znaků
byte reloadDisplay;                            //pomocná podmínek pro obnovení displeje
bool baudRateEnable = 1;						            //využije se při návratu do menu z komunikace když se definuje pouze typ detekce
unsigned long lastDebounceTime = 0;			      //proměnná pro uložení času odepnutí tlačítka pro debounce
unsigned long lastCommunicationTime = 0;	    //čas poslední komunikace
short lastChar;                               //číslo prvku umístění posledního přijatého znaku
byte choiceMemory1, choiceMemory2 = 0;            //paměť zvolených voleb

void setup() {

  //definování tlačítek
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);

  //definování LED diody
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  //inicializace LCD displeje
  glcd.begin(0x3);
  glcd.clear();
}

void loop() {

  ledRed(); //rozsvícení LED červeně

  //=====VÝBĚR BAUD RATE==========================================================================================================
  //===============================================================================================================================
  if (baudRateEnable == 1) { //podmínka končící před začátkem volby typu detekce, využije se při návratu do menu z komunikace
    reloadDisplay = 1; //načte data na displej
    selectNum[0] = 48; //znaky ASCII znamenající nuly
    selectNum[1] = 48; //při volbě vlastního baud rate jsou nuly výchozími hodnotami
    selectNum[2] = 48;
    selectNum[3] = 48;
    selectNum[4] = 48;
    selectNum[5] = 48;
    while (i == 0) {
      if (reloadDisplay == 1) {
        //============ výpis na display======
        glcd.drawstring(0, 1, (char *)" 1200"); //(sloupec, řádek, hodnota)
        glcd.drawstring(0, 2, (char *)" 4800");
        glcd.drawstring(0, 3, (char *)" 9600");
        glcd.drawstring(0, 4, (char *)" 19200");
        glcd.drawstring(0, 5, (char *)" 38400");
        glcd.drawstring(0, 6, (char *)" 57600          zadat");
        glcd.drawstring(0, 7, (char *)" 115200          jiny");
        glcd.drawstring(0, 0, (char *)"Vyber Baud Rate:");
        glcd.drawchar(vyberX, vyberY, 26); // znak pro šipku

        glcd.display();
        delay(displayDelay);
        glcd.clear();
        reloadDisplay = 0;
      }

      //==============pohyb s šipkou======
      buttonState1 = readButtonState(buttonPin1); //tlačítko dolu
      if (buttonState1 == 1) {
        reloadDisplay = 1;
        vyberY = vyberY + 1; //další řádek šipky
        if (vyberY > 7 && vyberX == 0) {
          vyberY = 7;
          vyberX = 87;
        }
        if (vyberY > 7 && vyberX > 0) {
          vyberY = 1;
          vyberX = 0;
        }
        delay(delayReload);
      }
      buttonState3 = readButtonState(buttonPin3); //tlačítko nahoru
      if (buttonState3 == 0 && pom == 5) pom = 2;
      if (buttonState3 == 1 && pom != 5) {
        reloadDisplay = 1;
        vyberY = vyberY - 1; //předchozí řádek šipky
        if (vyberY < 1) {
          vyberY = 7;
          vyberX = 87;
        }
        if (vyberX > 0 && vyberY < 7) {
          vyberY = 7;
          vyberX = 0;
        }
        delay(delayReload);
      }
      //=======zapamatování zvolené komunikační rychlosti====
      buttonState2 = readButtonState(buttonPin2); //tlačítko OK
      if (buttonState2 == 1) {
        i = 1;
        pom = 2;
        if (vyberY == 1) {
          baudRate = 1200;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 2) {
          baudRate = 4800;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 3) {
          baudRate = 9600;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 4) {
          baudRate = 19200;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 5) {
          baudRate = 38400;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 6) {
          baudRate = 57600;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 7 && vyberX == 0) {
          baudRate = 115200;
          choiceMemory1 = vyberY;
        }
        if (vyberY == 7 && vyberX > 0) { //jestliže byl zvolen vlastní baud rate
          baudRate = 0;
          vyberX = 30;        //pozice šipky v X
          reloadDisplay = 1;  //obnov displej
          j = 0;	            //pozice v poli selectNum
          choiceMemory1 = vyberY + 1;
        }
      }
    }

    //=====VOLBA VLASTNÍHO BAUD RATE================================================================================================
    //===============================================================================================================================

    while (baudRate == 0) {

      //==============pohyb s šipkou======
      buttonState1 = readButtonState(buttonPin1); //tlačítko dolu
      if (buttonState1 == 1 && vyberX > -1) {
        selectNum[j] = selectNum[j] - 1;
        if (selectNum[j] == 47) selectNum[j] = 57;
        reloadDisplay = 1;
        delay(delayReload);
      }

      buttonState3 = readButtonState(buttonPin3); //tlačítko nahoru
      if (buttonState3 == 1 && vyberX > -1) {
        selectNum[j] = selectNum[j] + 1;
        if (selectNum[j] == 58) selectNum[j] = 48;
        reloadDisplay = 1;
        delay(delayReload);
      }

      buttonState2 = readButtonState(buttonPin2); //tlačítko OK
      if (buttonState2 == 0 && pom == 2) pom = 1;
      if (buttonState2 == 1 && pom == 1) {
        vyberX = vyberX - 6;
        pom = 2;
        reloadDisplay = 1;
        j++;
      }

      //============ výpis na display======
      if (reloadDisplay == 1) {
        glcd.drawstring(0, 0, (char *)"Zadej Baud Rate:");
        glcd.drawchar(30, 1, selectNum[0]);
        glcd.drawchar(24, 1, selectNum[1]);
        glcd.drawchar(18, 1, selectNum[2]);
        glcd.drawchar(12, 1, selectNum[3]);
        glcd.drawchar(6, 1, selectNum[4]);
        glcd.drawchar(0, 1, selectNum[5]);
        if (vyberX > -1) glcd.drawchar(vyberX, 2, 24);  //znak pro šipku

        glcd.display();
        delay(displayDelay);
        glcd.clear();
        reloadDisplay = 0;
      }

      //konverze pole int "selectNum" do pole typu char "selectChar" pro konverzi do jediné proměnné typu long "baudRate"
      if (vyberX < 0) {
        selectChar[0] = char(selectNum[5]);
        selectChar[1] = char(selectNum[4]);
        selectChar[2] = char(selectNum[3]);
        selectChar[3] = char(selectNum[2]);
        selectChar[4] = char(selectNum[1]);
        selectChar[5] = char(selectNum[0]);

        baudRate = atol(selectChar); //konverze pole hodnot do jedné proměnné (za sebou)

        if (baudRate > 0) pom = 2;

        if (baudRate == 0) {
          vyberX = 30;  //pozice šipky v X
          reloadDisplay = 1; //obnov displej
          j = 0;  //pozice v poli selectNum
          pom = 2;
        }
      }
    }
  }



  //=====VÝBĚR TYPU DETEKCE========================================================================================================
  //===============================================================================================================================

  reloadDisplay = 1; //obnov displej
  if (choiceMemory2 == 0) {
    vyberY = 1;  //pozice šipky v Y
    vyberX = 0;
  }
  else vyberY = choiceMemory2;

  while (baudRate > 0 && i == 1) {

    if (reloadDisplay == 1) {
      glcd.drawstring(0, 0, (char *)"Vyber typ detekce:");
      glcd.drawstring(0, 1, (char *)" carriage return");
      glcd.drawstring(0, 2, (char *)" line feed");
      glcd.drawstring(0, 3, (char *)" CR i LF");
      glcd.drawstring(0, 4, (char *)" zadna");
      glcd.drawchar(0, vyberY, 26);     //znak pro šipku

      glcd.display();
      delay(displayDelay);
      glcd.clear();
      reloadDisplay = 0;
    }

    //==============pohyb s šipkou======
    buttonState1 = readButtonState(buttonPin1); //tlačítko dolu
    if (buttonState1 == 0 && pom == 5) pom = 2;
    if (buttonState1 == 1 && pom != 5) {
      reloadDisplay = 1;
      vyberY = vyberY + 1;
      if (vyberY > 4) {
        vyberY = 1;
      }
      delay(delayReload);
    }

    buttonState3 = readButtonState(buttonPin3); //tlačítko nahoru
    if (buttonState3 == 1) {
      reloadDisplay = 1;
      vyberY = vyberY - 1;
      if (vyberY < 1) {
        vyberY = 4;
      }
      delay(delayReload);
    }
    buttonState2 = readButtonState(buttonPin2); //tlačítko OK
    if (buttonState2 == 0 && pom == 2) pom = 1;
    if (buttonState2 == 1 && pom == 1) {
      detekce = vyberY; //uložení typu detekce
      i = 2; // ukončení smyčky
      choiceMemory2 = vyberY;
    }
  }



  //=====INICIALIZACE SÉRIOVÉ KOMUNIKACE===========================================================================================
  //===============================================================================================================================
  //===============================================================================================================================

  Serial.begin(baudRate); //inicializace sériové rychlosti komunikace
  delay(10); //čekání po inicializaci
  ledGreen(); //rozsvícení LED zeleně
  pom = 0; //pro stopIfButton2
  pom3 = 0;

  //vymazání všech polí (nahrazení mezerami) - prázdný znak musí být vždy mezera!
  for (byte j = 0; j < 21; j++) {
    line0[j] = ' ';
    line1[j] = ' ';
    line2[j] = ' ';
    line3[j] = ' ';
    line4[j] = ' ';
    line5[j] = ' ';
    line6[j] = ' ';
    line7[j] = ' ';
  }
  for (short j = 0; j < x; j++) {
    receivedData[j] = ' ';
  }
  reloadDisplayProcedure(); //-----------------------------------------------------------
  lastCommunicationTime = millis();
  glcd.drawstring(0, 0, (char *)"Cekani na data . . ."); //vypíše hlášku
  glcd.display();
  delay(displayDelay);
  glcd.clear();

  //=====ZAČÁTEK SMYČKY SÉRIOVÉ KOMUNIKACE=========================================================================================
  //===============================================================================================================================

  while (i == 2) { //nekonečná komunikační smyčka dokud nenastane stisknutí tlačítka nahoru nebo dolu - návrat do menu

    if (millis() - lastCommunicationTime > 2000) {           //při nepřijetí dat po dobu 1 s možno pozastavit
      if (readButtonState(buttonPin2) == 0 && pom3 == 2) {
        pom3 = 0;
      }
      if (readButtonState(buttonPin2) == 1 && pom3 == 0) {
        stopIfButton2();
        pom3 = 2;
      }
    }
    returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
    returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
    if (pom3 == 1) return;

    if (Serial.available() > 0) {
      for (byte j = 0; j < 21; j++) {  //posun řádků na displeji pro uvolnění místa
        line0[j] = line1[j];
        line1[j] = line2[j];
        line2[j] = line3[j];
        line3[j] = line4[j];
        line4[j] = line5[j];
        line5[j] = line6[j];
        line6[j] = line7[j];
      }
      Serial.readBytes(receivedData, x - 21); //načtení dat ze sériové komunikace
      reloadDisplay = 1;
    }

    //=====KDYŽ SE PŘIJMOU DATA======================================================================================================
    //===============================================================================================================================

    if (reloadDisplay == 1) {
      Serial.end();

      for (short j = x - 21 ; j < x; j++) { //projede prvky pole pro poslední řádek na displeji a nahradí mezerami
        receivedData[j] = ' ';					  //poslední řádek musí být vždy mezery! jinak nastanou nechtěné nekonečné smyčky
      }

      for (short j = 0; j < x; j++) {
        if (detekce == 1 && receivedData[j] == 10) {                       //znak 10 nahradí mezerou pro nerušení na displeji při carriage return
          receivedData[j] = ' ';
        }
        if (detekce == 2 && receivedData[j] == 13) {                       //znak 13 nahradí mezerou pro nerušení na displeji při line feed
          receivedData[j] = ' ';
        }
        if (detekce == 4 && (receivedData[j] == 10 || receivedData[j] == 13)) {   //znak 10 a 13 nahradí mezerou pro nerušení na displeji při žádné detekci
          receivedData[j] = ' ';
        }
      }
      for (short k = 21; k < x - 21; k++) {  //hledá umístení posledního znaku
        if (receivedData[k] != ' ') {
          lastChar = k;
        }
      }
      pom2 = 1;
      lineShift(); //posune na další řádek podle typu detekce znaků 13 a 10
      if (pom3 == 1) return;

      if (pom2 == 1) {  //jestliže v proceduře lineShift() nebyl displej nikdy vypsán (detekce = 4 - žádná)
        for (byte j = 0; j < 21; j++) {
          line7[j] = receivedData[j];
        }
        reloadDisplayProcedure(); //-----------------------------------------------------------
        stopIfButton2();      //jestliže je stisknutý tlačítko OK tak zamrzne displej
        returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
        returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
        if (pom3 == 1) return;
        pom2 = 0;
      }

      //posun řádků na displeji v případě dlouhých přijatých dat za hranici jednoho řádku
      if (lastChar > 20) {
        for (short p = 21; p < lastChar; p = p + 21) {
          for (byte j = 0; j < 21; j++) {
            line0[j] = line1[j];
            line1[j] = line2[j];
            line2[j] = line3[j];
            line3[j] = line4[j];
            line4[j] = line5[j];
            line5[j] = line6[j];
            line6[j] = line7[j];
          }
          for (byte k = 0; k < 21; k++) {
            line7[k] = receivedData[k + p];    //zapisuje od prvního přijatého řádku k poslednímu
          }
          reloadDisplayProcedure(); //---------------------------------------------------------
          stopIfButton2();      //jestliže je stisknutý tlačítko OK tak zamrzne displej
          returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
          returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
          if (pom3 == 1) return;
        }
      }
      Serial.begin(baudRate); //inicializace sériové rychlosti komunikace
      delay(10);
      lastCommunicationTime = millis();
      reloadDisplay = 0; //ukončení podmínky
      pom3 = 0;
      for (short j = 0; j < x; j++) {
        receivedData[j] = ' ';         //vymazání dat
      }
    }

  }
}

//=====KONEC PROGRAMU============================================================================================================
//===============================================================================================================================










//===============================================================================================================================
//=====FUNKCE A PROCEDURY========================================================================================================
//===============================================================================================================================

//=====ČTENÍ TLAČÍTKA===========================================================
//==============================================================================
byte readButtonState(byte buttonPin) {
  bool button;                                   // pomocná proměnná pro funci "readButtonState"

  button = digitalRead(buttonPin);
  if (button == HIGH) {
    delay(debounceDelay);  //čekací debounce - aby vyhodnotilo sepnutí ihned
    if (button == HIGH) {
      return 1;             //při sepnutí vrátí 1
    }
  }

  //kontinuální debounce při rozepnutém stavu, nezdržuje program, není potřeba vyhodnotit ihned
  if (button == LOW && lastDebounceTime == 0) {
    lastDebounceTime = millis(); //zapíše čas arduina od doby spuštění
  }
  if (button == LOW && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = 0;
    return 0;             //při rozepnutí vrátí 0
  }
}

//=====ROZSVÍCENÍ LED DIODY ZELENĚ==============================================
//==============================================================================
void ledGreen() {
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, HIGH);
}

//=====ROZSVÍCENÍ LED DIODY ČERVENĚ=============================================
//==============================================================================
void ledRed() {
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, LOW);
}

//=====NÁVRAT DO MENU NA VÝBĚR BAUD RATE PŘI STISKNUTÍ TLAČÍTKA NAHORU==========
//==============================================================================
void returnIfButton3() {
  buttonState3 = readButtonState(buttonPin3); //tlačítko nahoru (odepnutí = 0, sepnutí = 1)
  if (buttonState3 == 1) {                    //vrátí program na úplný začátek, na výběr boud rate
    ledRed(); //led rozsvítí červeně
    Serial.end(); //ukončení seriové komunikace

    vyberY = choiceMemory1;
    vyberX = 0;
    if (choiceMemory1 == 8) {
      vyberY = 7;
      vyberX = 87;
    }

    baudRateEnable = 1; //vrácení na úplný začátek, na výběr baud rate
    i = 0;  //ukončení smyčky komunikace
    pom3 = 1;
    pom = 5;
  }
}

//=====NÁVRAT DO MENU NA VÝBĚR TYPU DETEKCE PŘI STISKNUTÍ TLAČÍTKA DOLU=========
//==============================================================================
void returnIfButton1() {
  buttonState1 = readButtonState(buttonPin1); //tlačítko dolu (odepnutí = 0, sepnutí = 1)
  if (buttonState1 == 1) {                    //vrátí program pro volbu typu detekce
    ledRed(); //led rozsvítí červeně
    Serial.end();  //ukončení seriové komunikace

    baudRateEnable = 0; //vrácení na výběr typu detekce
    i = 1;  //ukončení smyčky komunikace
    pom3 = 1;
    pom = 5;
  }
}

//=====STOP DISPLEJE JESTLIŽE JE SEPNUTÉ TLAČÍTKO OK============================
//==============================================================================
void stopIfButton2() {
  //stisknutí tlačítka OK pozastaví vypisování na display, po druhém stisknutí vrátí zpět
  //po dlouhém stisku ze stavu pozastavení spustí program a ihned po vypsání další řádku na displej znovu pozastaví

  buttonState2 = readButtonState(buttonPin2); //tlačítko OK (odepnutí = 0, sepnutí = 1)
  if (buttonState2 == 1) {
    pom = 2;
    ledRed(); //rozsvícení LED červeně
  }
  while (pom > 1) { //zůstane ve smyčce dokud se tlačítko nezmáčkne znova
    buttonState2 = readButtonState(buttonPin2); //tlačítko OK (odepnutí = 0, sepnutí = 1)
    if (buttonState2 == 0 && pom == 2) {
      pom = 3;
    }
    if (readButtonState(buttonPin1) == 1 || readButtonState(buttonPin3) == 1) { //návrat do menu když je pauza
      returnIfButton3();
      returnIfButton1();
      buttonState2 = 1;
      return;
    }
    if (buttonState2 == 1 && pom == 3) {
      ledGreen(); //rozsvícení LED zeleně
      pom = 0;
    }
  }
}

//=====ZAPSÁNÍ DAT NA DISPLEJ===================================================
//==============================================================================
void reloadDisplayProcedure() {
  glcd.drawstring(0, 0, line0); //řádek 0
  glcd.drawstring(0, 1, line1);
  glcd.drawstring(0, 2, line2);
  glcd.drawstring(0, 3, line3);
  glcd.drawstring(0, 4, line4);
  glcd.drawstring(0, 5, line5);
  glcd.drawstring(0, 6, line6);
  glcd.drawstring(0, 7, line7); //řádek 7

  glcd.display();
  delay(displayDelay);
  glcd.clear();
}

//=====POSUNUTÍ ŘÁDKŮ DLE VYBRANÉHO TYPU DETEKCE ZNAKŮ 10 A 13==================
//==============================================================================
void lineShift() {

  byte character1;
  byte character2;
  if (detekce == 1) { //detekce carriage return
    character1 = 13;
    character2 = 13;
  }
  if (detekce == 2) { //detekce line feed
    character1 = 10;
    character2 = 10;
  }
  if (detekce == 3) { //detekce CR i LF
    character1 = 10;
    character2 = 13;
  }


  for (short j = 0; j < lastChar; j++) {                                                 //projede celé pole
    if (pom3 == 1) return;

    if ((receivedData[j] == character1 || receivedData[j] == character2) && detekce != 4) {   //když najde znak
      short p = 0;								  //vymazání počítadla řádků
      for (short m = j; m > -1; m = m - 21) {      //smyčka postupující po řádcích
        if (m < 21) {			   				  //jestliže znak je na prvním řádku
          for (byte k = 0; k < 21; k++) {        //vymaž line7
            line7[k] = ' ';
          }
          for (short k = p; k < j; k++) {
            line7[k - p] = receivedData[k];      //přepíše data po konci znaku na začátek pole
          }
          if (m > 0) {              //jestliže znak není první na displeji (nevypíše prázdný řádkek)
          reloadDisplayProcedure(); //-----------------------------------------------------------
          stopIfButton2();		  //jestliže je stisknutý tlačítko OK tak zamrzne displej
          returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
          returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
          if (pom3 == 1) return;
          }

          if (m > 0) {                          //jestliže znak není první na displeji
            for (byte k = 0; k < 21; k++) {         //posune řádky
              line0[k] = line1[k];
              line1[k] = line2[k];
              line2[k] = line3[k];
              line3[k] = line4[k];
              line4[k] = line5[k];
              line5[k] = line6[k];
              line6[k] = line7[k];
            }
          }
          for (short k = j + 1; k < x; k++) {
            receivedData[k - j - 1] = receivedData[k];		  //přepíše data po konci znaku na začátek pole
          }
          lastChar = lastChar - j - 1;
          for (short k = lastChar; k < x; k++) {
            receivedData[k] = ' ';                            //po posledním přijatém znaku jsou prázdné mezery
          }

          for (byte k = 0; k < 21; k++) {
            line7[k] = receivedData[k];      //přepíše data do line7
          }
          byte pom6 = 0;
          for (byte k = 0; k < 21; k++) {
            if (line7[k] == character1 || line7[k] == character2) {  //jestliže pro první řádek 21 znaků - nově zalomeného řádku je znak další
              pom6 = 1;
            }
          }
          if (pom6 == 0) {             //pouze pokud v prvním řádku 21 znaků není znak
            reloadDisplayProcedure(); //-----------------------------------------------------------
            stopIfButton2();      //jestliže je stisknutý tlačítko OK tak zamrzne displej
            returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
            returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
            if (pom3 == 1) return;

          }

          j = -1;  //smyčka hledá znaky od začátku pole od znova
          pom2 = 0; //na displej bylo zapsáno
        }
        if (m > 20) {		//jestliže znak je dál než na prvním řádku 21 znaků
          if (pom2 == 1) {  //jestliže na displej nebylo zapsáno
            for (byte k = 0; k < 21; k++) {
              line7[k] = receivedData[k + p];    //zapisuje od prvního přijatého řádku k předposlednímu
            }
            reloadDisplayProcedure(); //-----------------------------------------------------------
            stopIfButton2();      //jestliže je stisknutý tlačítko OK tak zamrzne displej
            returnIfButton1(); //stisknuté tlačítko dolu - návrat do volby typu detekce
            returnIfButton3(); //stisknuté tlačítko nahoru - návrat začátek programu pro volbu velikosti baud rate
            if (pom3 == 1) return;
          }
          pom2 = 1;		//následně na displej zapisuj

          for (byte k = 0; k < 21; k++) {         //posune řádky
            line0[k] = line1[k];
            line1[k] = line2[k];
            line2[k] = line3[k];
            line3[k] = line4[k];
            line4[k] = line5[k];
            line5[k] = line6[k];
            line6[k] = line7[k];
          }
        }
        p = p + 21; //při opakování smyčky další řádek, počítadlo řádků
      }
    }

  }
}
