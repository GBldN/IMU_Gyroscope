/* ################# PROGRAMME POUR TESTER LES VALEURS D'ATTITUDE CALCULEES A PARTIR D'UNE CENTRALE INERTIELLE #################
 * 
 * Programme pour une platine d'expérimentation gyroscopique à 3 cardans dont 2 sont équipés de capteurs de rotations afin de proposer :
 *    - la mesure la plus précise possible des angles « réels » entre les cardans correspondant au ROULIS et au TANGAGE,
 *    - le calcul des angles à partir de la centrale,
 *    - l’affichage sur un écran LCD des 6 angles (3 pour le ROULIS et 3 pour le TANGAGE) dans l’ordre suivant :
 *      . L’angle « réel » ;
 *      . L’angle calculé à partir de l’accéléromètre ;
 *      . L’angle calculé avec le gyromètre,
 * 
 * Matériel utilisé : 
 *    - Un Arduino UNO
 *    - Une centrale inertielle Grove_IMU_6 DOF : https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/
 *    - Un écran LCD RGB BLACKLIGHT : https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/
 *    - Deux potentiomètres Grove Rotary_Angle_Sensor : https://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/
 * 
 * Ce programme permet : 
 *    - d'acquérir les valeurs mesurées par la centrale inertielle, 
 *    - de calculer les angles de ROULIS, TANGAGE à partir de l'accéléromètre
 *    - de calculer les angles de ROULIS, TANGAGE et LACET à partir du gyromètre
 *    - de calculer les angles de ROULIS et TANGAGE mécaniques entre les cardans (à partir de potentiomètres)
 *    - Afficher les 3 angles sur un écran LCD et sur le moniteur série
 * 
 * L'ensemble des ressources sont disponibles sur le github https://github.com/GBldN/IMU_Gyroscope
 * 
 * version=1.0
 * author=gael.balduini@gmail.com
 * licence=CC-by-nc-sa
 * maintainer=Arduino <info@arduino.cc>
 * url=https://github.com/GBldN/IMU_Gyroscope
 */

/* ##### DEFINITION DES MACROS ##### */

/* ##### DECLARATION DES CONSTANTES ##### */
  #define       f_ech               100               //Choix d'une fréquence en Hz pour le calcul à intervalle régulier entre 1 et 500 Hz (ATTENTION ! 1000 doit être un multiple de f_ech)
  
  #define       pleine_echelle_acc  16                //réglage de la gamme de mesure de l'accéléromètre ±2g ; ±4g ; ±8g ou ±16g
  #define       sensi_acc           488               //Sensibilité de l'accéléromètre en µg/LSB   (réglé par défault pour 4g mais se personnalise en fonction de la pleine échelle
  #define       pleine_echelle_gyr  2000              //réglage de la gamme de mesure du gyromètre ± 250; 500; 1000; 2000 °/s (2; 41; 42; 83; 167; 333 tr/min) ou ±125°/s pour le LM6DS3 
  #define       sensi_gyr           70000             //Sensibilité du gyromètre       en µdps/LSB (réglé par défault pour 2000 °/s mais se personnalise en fonction de la pleine échelle)
 
    
/* ##### DEFINITION DES CONNEXIONS ##### */
  #define       Bp1Pin              4                 //Port de connexion de l'entrée du bouton poussoir d'etalonnage
  #define       Bp2Pin              5                 //Port de connexion de l'entrée du bouton poussoir de choix du filtre
  #define       Potar0Pin           A0                //Port de connexion du potentiomètre
  #define       Potar1Pin           A1                //Port de connexion du potentiomètre
  
/* ##### DEFINITION DES CONNEXIONS ET DES VARIABLES ##### */
/* -- Intégrations des bibiothèques -- */
  #include <Wire.h>                                   //Intégration de la biblitothèque "wire" pour la communication I2C
  #include "rgb_lcd.h"                                //Intégration de la biblitothèque "rgb_lcd" pour la commande de l'écran GROVE I2C
  rgb_lcd lcd;                                        //Création de la fonction nommée ici "lcd"
  
/* -- Définition des variables pour les entrée et mesures -- */
    
  int16_t       ax_brut, ay_brut, az_brut;            //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
  int16_t       gx_brut, gy_brut, gz_brut;            //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
  int32_t       ax_offset, ay_offset, az_offset;      //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
  int32_t       gx_offset, gy_offset, gz_offset;      //Variables pour le stockage des valeurs d'offsets du gyrocope
  
/* -- Définition des variables pour les calcul des angles -- */
  float         Ax_reel, Ay_reel, Az_reel;            //Variables pour le calcul des valeurs réelle de l'accéléromètre valeur_brute * sensibilité / précision
  float         Gx_reel, Gy_reel, Gz_reel;            //Variables pour le calcul des valeurs réelle du gyromètre
  float         Gx_prec, Gy_prec, Gz_prec;            //Variables pour le stockage de la valeur précédente pour le calcul des angles avec le gyromètre
  
  float         roulis_gyr, tangage_gyr, lacet_gyr;   //Variables pour le calcul des angles d'inclinaisons avec le gyromètre : ROULIS, TANGAGE et LACET
  float         roulis_acc, tangage_acc;              //Variables pour le calcul des angles d'inclinaisons avec l'accéléromètre : ROULIS et TANGAGE

/* -- Définition des variables pour le calcul du temps -- */
  uint32_t      t0;                                   //Variable pour stocker le temps absolu ( fontion millis() avec débordement après 50 jours environ)
  uint32_t      t1;                                   //Variable pour stocker le temps absolu ( fontion millis() avec débordement après 50 jours environ)
  
  
/* ---------------------------------------------------- *
 *  ROUTINE D'INITIALISATION  (exécutée une seule fois) *
 * ---------------------------------------------------- */
void setup() 
{
/* -- Configuration des broches en ENTREE ou SORTIE -- */
  pinMode(Bp1Pin, INPUT);
  pinMode(Bp2Pin, INPUT);

/* -- configuration des fonctions spéciales */
  Serial.begin(115200);                               //Initialisation de la bibliothèque Moniteur série
  Wire.begin();                                       //Initialisation de la connexion pour l'I2C

/* -- Affichage temporaire du programme actif dans le µC -- */
  lcd.begin(16, 2);                                   //Démarrage et configuration de la fonction LCD pour un écran 2 lignes 16 caractères
  lcd.clear();                                        //Effacement de l'écran
  lcd.setRGB(0, 255, 0);                              //Réglage de la couleur de l'écran en VERT
  lcd.setCursor(0, 0);                                //Positionnement du curseur (caractère,ligne)
  lcd.print(" PLATINE MESURE ");                      //Ecriture du titre du programme sur le LCD
  lcd.setCursor(0, 1);
  lcd.print("CENTRALE INERTIELLE");
  delay(1000); 

/* -- Paramétrage de la centrale et étalonnage -- */  
  reglage_LM6DS3();
  delay(200);
  etalonnage();

/* -- Initialisation du temps initial pour les calculs à intevalle régulier de l'intégration numérique et de l'affichage sur le LCD -- */
  t0 = millis();
  t1 = millis();
}
  
/* -------------------------------------------- *
 *  BOUCLE PRINCIPALE (exécutée en permanence)  *
 * -------------------------------------------- */
void loop()
{
/* -- Lancement de la procédure d'étalonnage en cas d'appui sur le bouton poussoir -- */
  if ( digitalRead(Bp1Pin) == HIGH ) etalonnage();    //Appel de la routine d'étalonnage si appui sur le bouton poussoir

/* -- Lecture des valeurs brutes de la centrale -- */
  lecture_valeurs_brutes();

/* -- Calcul des valeurs réelles d'accélérations en g et de vitesses angulaires en °/S (dps) en fonction des valeurs brutes et la sensibilité donnée µg/LSB et µdps/LSB  -- */
  Ax_reel = float(ax_brut - ax_offset) * sensi_acc / 1000000; 
  Ay_reel = float(ay_brut - ay_offset) * sensi_acc / 1000000;
  Az_reel = float(az_brut - az_offset) * sensi_acc / 1000000;
  Gx_reel = float(gx_brut - gx_offset) * sensi_gyr / 1000000;
  Gy_reel = float(gy_brut - gy_offset) * sensi_gyr / 1000000;
  Gz_reel = float(gz_brut - gz_offset) * sensi_gyr / 1000000;

/* -- Calcul des angles de ROULIS et TANGAGE à partir de l'accéléromètre -- */
//CHOIX N°1
  float A_norme = sqrt(pow(Ax_reel,2) + pow(Ay_reel,2) + pow(Az_reel,2));
  //roulis_acc  = asin( Ay_reel / A_norme) * RAD_TO_DEG ;
  if ( Az_reel >= 0) roulis_acc  = atan2( Ay_reel ,   sqrt( pow(Ax_reel, 2) + pow(Az_reel, 2)) ) * RAD_TO_DEG;
  else               roulis_acc  = atan2( Ay_reel , - sqrt( pow(Ax_reel, 2) + pow(Az_reel, 2)) ) * RAD_TO_DEG;
  tangage_acc = atan2( - Ax_reel , Az_reel ) * RAD_TO_DEG;

/*
//CHOIX N°2
    roulis_acc = atan2( Ay_reel , Az_reel ) * RAD_TO_DEG;
    //tangage_acc  = asin( -Ax_reel / A_norme) * RAD_TO_DEG ;
    if ( Az_reel >= 0) tangage_acc  = atan2(   Ax_reel ,   sqrt( pow(Ay_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;
    else               tangage_acc  = atan2(   Ax_reel , - sqrt( pow(Ay_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;

//CHOIX N°3
    roulis_acc = atan2( Ay_reel , Az_reel ) * RAD_TO_DEG ;
    if ( Az_reel >= 0)
    {
      roulis_acc  = atan2( Ay_reel , sqrt( pow(Ax_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;
      tangage_acc = atan2( Ax_reel , sqrt( pow(Ay_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;
    }
    else
    {
      roulis_acc  = atan2( Ay_reel , -sqrt( pow(Ax_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;
      tangage_acc = atan2( Ax_reel , -sqrt( pow(Ay_reel,2) + pow(Az_reel,2)) ) * RAD_TO_DEG;
    }
*/

/* -- Initialisation des angles calculés avec le gyromètre si Stabilisation horizontale -- */
  if ( A_norme > 0.99 && A_norme < 1.01 )
  {
    if ( roulis_acc  > -0.1 && roulis_acc  < 0.1 ) roulis_gyr  = 0;
    if ( tangage_acc > -0.1 && tangage_acc < 0.1 ) tangage_gyr = 0;
  }


/* -- Calcul des vitesses angulaires autour des axes de ROULIS, TANGAGE et LACET en fonction de l'inclinaison  -- */ 
  float roulis_vitesse  = Gx_reel * cos(tangage_gyr * DEG_TO_RAD) +   Gz_reel * sin(tangage_gyr * DEG_TO_RAD);
  float tangage_vitesse = Gy_reel;
  float lacet_vitesse   = Gy_reel * sin(roulis_gyr * DEG_TO_RAD)  + ( -Gx_reel * sin(tangage_gyr * DEG_TO_RAD) + Gz_reel * cos(tangage_gyr * DEG_TO_RAD) ) * cos(roulis_gyr * DEG_TO_RAD);
   
/* -- Calcul des angles de ROULIS ET TANGAGE à partir du gyromètre à chaque période (voir explications des calculs) -- */
  uint16_t delta = millis() - t0;                                                       //Calcul de la période
  if ( delta >= ( 1000 / f_ech) )                                                       //Calcul à intervalle régulier défini par le réglage de la fréquence d'échantillonnage f_ech
  {
    lacet_gyr   += lacet_vitesse   * float(delta) / 1000;
    roulis_gyr  += roulis_vitesse  * float(delta) / 1000;
    tangage_gyr += tangage_vitesse * float(delta) / 1000;
    t0 = millis();
  }
  
/* -- Modulation de l'angle calculé avec le gyromètre pour qu'il reste entre -180° et +180° -- */
  if (roulis_gyr  <  180) roulis_gyr += 360;
  if (roulis_gyr  >= 180) roulis_gyr -= 360;
  if (tangage_gyr <  180) tangage_gyr += 360;
  if (tangage_gyr >= 180) tangage_gyr -= 360;
  
/* -- Mesure et calcul des angles de la platine avec les potentiomètres -- */
  int16_t roulis  = round( 90 * ( 2 * float(analogRead(Potar0Pin)) - ( 158 + 876 ) ) / ( 158 - 876) );
  int16_t tangage = round( 90 * ( 2 * float(analogRead(Potar1Pin)) - ( 885 + 161 ) ) / ( 885 - 161) );

/* -- AFFICHAGE SUR LE MONITEUR SERIE -- */
  /*
    Serial.print(" BRUTES :");
    Serial.print(" aX = ");   Serial.print(ax_brut);
    Serial.print(" - aY = "); Serial.print(ay_brut);
    Serial.print(" - aZ = "); Serial.print(az_brut);
    Serial.print(" / gX = ");   Serial.print(gx_brut);
    Serial.print(" - gY = "); Serial.print(gy_brut);
    Serial.print(" - gZ = "); Serial.print(gz_brut);
  */
  Serial.print(" | RELLES : ");
  /*
    Serial.print("(g)");
    Serial.print(" aX = ");   Serial.print(Ax_reel,2);
    Serial.print(" - aY = "); Serial.print(Ay_reel,2);
    Serial.print(" - aZ = "); Serial.print(Az_reel,2);
    Serial.print(" /(deg/s)");
    Serial.print(" gX = ");   Serial.print(Gx_reel,1);
    Serial.print(" - gY = "); Serial.print(Gy_reel,1);
    Serial.print(" - gZ = "); Serial.print(Gz_reel,0);
  */
  Serial.print(" norme = "); Serial.print(A_norme); Serial.print( "g ");
  Serial.print(" | ANGLES :");
  Serial.print(" ROULIS Pot = ");                                           Serial.print(roulis);         Serial.print("° ");
  Serial.print(" Acc = ");          if (roulis_acc > 0) Serial.print(" ");   Serial.print(roulis_acc , 0);   Serial.print("° ");
  Serial.print(" - Gyr = ");        if (roulis_gyr > 0) Serial.print(" ");   Serial.print(roulis_gyr , 0);   Serial.print("° ");
  Serial.print(" / TANGAGE Pot = ");                                        Serial.print(tangage);        Serial.print("° ");
  Serial.print(" - Acc = ");        if (tangage_acc > 0) Serial.print(" ");  Serial.print(tangage_acc , 0);  Serial.print("° ");
  Serial.print(" - Gyr = ");        if (tangage_gyr > 0) Serial.print(" ");  Serial.print(tangage_gyr , 0);  Serial.print("° ");
  Serial.println();

/* -- AFFICHAGE SUR L'ECRAN LCD tous les 200 ms-- */
  if ( ( millis() - t1 ) > 200 )
  {
    lcd.setCursor(0, 0);
    lcd.print("R:");
    lcd.print(roulis);
    lcd.print("   ");
    lcd.setCursor(6, 0);
    lcd.print(";");
    lcd.print(roulis_acc , 0);
    lcd.print("   ");
    lcd.setCursor(11, 0);
    lcd.print(";");
    lcd.print(roulis_gyr , 0);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(tangage);
    lcd.print("   ");
    lcd.setCursor(6, 1);
    lcd.print(";");
    lcd.print(tangage_acc , 0);
    lcd.print("   ");
    lcd.setCursor(11, 1);
    lcd.print(";");
    lcd.print(tangage_gyr , 0);
    lcd.print("    ");
    t1= millis();
  }

}
 
  
/* -------------------------------------------- *
 * ROUTINE DE LECTURE DES VALEUR DE X REGISTRES *
 * -------------------------------------------- */
/* Remarque *pTableau correspond au pointeur vers le tableau voir https://www.locoduino.org/spip.php?article106 */
void LECTURE_REGISTRES(uint8_t Add_module, uint8_t Add_Registre, uint8_t Nregistres, uint8_t * pTableau) //Lecture, sur la liaison l'I2C, de plusieurs registres successifs
{
  Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
  Wire.write(Add_Registre);                                                            //Définition de l'adresse du PREMIER registre
  Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
   
  Wire.requestFrom(Add_module, Nregistres);                                            //Requette de lecture des n octects
  uint8_t index=0;                                                                     //initilisation d'une variable de comptage
  while (Wire.available()) pTableau[index++]=Wire.read();                              //Tant que des octects sont disponibles enregistrer la valeur de chaque octect dans le tableau et incrémenter l'index
}
   
/* ------------------------------------------------ *
 * ROUTINE D'ECRITURE D'UNE VALEUR DANS UN REGISTRE *
 * ------------------------------------------------ */
void ECRITURE_REGISTRE(uint8_t Add_module, uint8_t Add_Registre, uint8_t Valeur)     //Ecriture, sur la liaison l'I2C, d'un une valeur d'un octect dans un registre
{ 
  Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
  Wire.write(Add_Registre);                                                            //Définition de l'adresse du registre
  Wire.write(Valeur);                                                                  //Ecriture de la valeur
  Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
}

/* ------------------------------------------------- *
*   ROUTINE DE LECTURE DES VALEURS BRUTES DU CAPTEUR *
* -------------------------------------------------- */
void lecture_valeurs_brutes()
{ 
  //Variable locale pour déclarer un pointeur vers un tableau dynamique de 6 valeurs
  uint8_t pTampon[6];

  LECTURE_REGISTRES( 0x6A , 0x28 , 6 , pTampon );
  ax_brut= (pTampon[0] << 8 | pTampon[1] );
  ay_brut= (pTampon[2] << 8 | pTampon[3] );
  az_brut= (pTampon[4] << 8 | pTampon[5] ); 

  LECTURE_REGISTRES( 0x6A , 0x22 , 6 , pTampon );
  gx_brut=( pTampon[0] << 8 | pTampon[1] );
  gy_brut=( pTampon[2] << 8 | pTampon[3] );
  gz_brut=( pTampon[4] << 8 | pTampon[5] );
}

/* ------------------------------------------------------------------------ *
 * ROUTINE DE REGLAGE DE LA CENTRALE INERTIELLE STMicroelectronics LSM6DS3  *
 *      Utilise la fonction de communication I2C ECRITURE_REGISTRE()        *
 * ------------------------------------------------------------------------ */
void reglage_LM6DS3()
{
  /* -- REGISTRE : FUNC_CFG_ACCESS (01h) : Désactive (/active) les fonctions embarquées */
  ECRITURE_REGISTRE( 0x6A , 0x01 , 0b00000000 );
  /* -- REGISTRE : ORIENT_CFG_G (0Bh) Angular rate sensor sign and orientation register */
  ECRITURE_REGISTRE( 0x6A , 0x0B , 0b00000000 );
  /* -- REGISTRE : CTRL1_XL (10h) Linear acceleration sensor control register 1 */
  ECRITURE_REGISTRE( 0x6A , 0x10 , 0b01100100 ); /* ±16g et autres réglages par défaut */
  /* -- REGISTRE : CTRL2_G (11h) Angular rate sensor control register 2 */
  ECRITURE_REGISTRE( 0x6A , 0x11 , 0b01101100 ); /* ±2000°/s et autres réglages par défaut */
  /* -- REGISTRE : CTRL3_C (12h) Control register 3 */
  ECRITURE_REGISTRE( 0x6A , 0x12 , 0b00000110 );
  /* -- REGISTRE : CTRL4_C (13h) Control register 4 */
  ECRITURE_REGISTRE( 0x6A , 0x13 , 0b10000000 );
  /* -- REGISTRE : CTRL5_C (14h) Control register 5 */
  ECRITURE_REGISTRE( 0x6A , 0x14 , 0b00000000 );
  /* -- REGISTRE : CTRL6_C (15h) Control register 6 */
  ECRITURE_REGISTRE( 0x6A , 0x15 , 0b00000000 );
  /* -- REGISTRE : CTRL7_G (16h) Angular rate sensor control register 7 */
  ECRITURE_REGISTRE( 0x6A , 0x16 , 0b00000000 );
  /* -- REGISTRE : CTRL8_XL (17h) Linear acceleration sensor control register 8 */
  ECRITURE_REGISTRE( 0x6A , 0x17 , 0b00000000 );     
  /* -- REGISTRE : CTRL9_XL (18h) Linear acceleration sensor control register 9 (r/w) */
  ECRITURE_REGISTRE( 0x6A , 0x18 , 0b00111000 );
  /* -- REGISTRE : CTRL10_C (19h) Control register 10 (r/w) */
  ECRITURE_REGISTRE( 0x6A , 0x19 , 0b00111000 );
}
  
  
/* ------------------------------------------------ *
 *     ROUTINE D'ETALONNAGE = calcul des offsets    *
 * ------------------------------------------------ */
void etalonnage()
{
  Serial.println();
  Serial.println(" ################### ETALONNAGE EN COURS ###################" );
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ETALONNAGE EN COURS ");
  lcd.setCursor(0, 1);
  lcd.print("! Ne pas bouger !     "); 
  
  uint16_t nombre_iterations = 500;                                                        //Variable pour définir le nombre d'itération du calcul
  ax_offset = 0;
  ay_offset = 0;
  az_offset = 0;
  gx_offset = 0;
  gy_offset = 0;
  gz_offset = 0;

  for (int i = 1; i <= nombre_iterations; i++)
  {
    lecture_valeurs_brutes();

    ax_offset += ax_brut;                                                              //Sommation de chaque valeur pour le calcul de la moyenne
    ay_offset += ay_brut;
    az_offset += az_brut;
    gx_offset += gx_brut;
    gy_offset += gy_brut;
    gz_offset += gz_brut;
  }

  ax_offset /= nombre_iterations;  //Calcul de la moyenne des valeurs mesurées pendant les itérations
  ay_offset /= nombre_iterations;
  az_offset /= nombre_iterations;
  az_offset -= 32767 / pleine_echelle_acc;
  gx_offset /= nombre_iterations;
  gy_offset /= nombre_iterations;
  gz_offset /= nombre_iterations;

  Serial.println();
  Serial.println(" ################### ETALONNAGE REALISE ###################" );
  Serial.println();
  Serial.print(" OFFSETS ACCELEROMETRE : ");
  Serial.print(" X = "); Serial.print(ax_offset); Serial.print(" | Y = "); Serial.print(ay_offset); Serial.print(" | Z = "); Serial.print(az_offset);
  Serial.println();

  Serial.print(" OFFSETS GYROMETRE     : ");
  Serial.print(" X = "); Serial.print(gx_offset); Serial.print(" | Y = "); Serial.print(gy_offset); Serial.print(" | Z = "); Serial.print(gz_offset);
  Serial.println();
  Serial.println();
  Serial.print(" ------- RELACHEZ LE BOUTON ------- ");


  Serial.println();
  Serial.println();

  while (digitalRead(Bp1Pin))
  {
    //Boucle d'attente du relachement du bouton
  }
  roulis_gyr = 0;                                                                     //initialisation des calcul d'angles issues du gyroscope
  tangage_gyr = 0;

  lcd.clear();
}
