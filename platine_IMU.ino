



  /* ##### DEFINITION DES MACROS POUR LA PERSONNALISATION DU MATERIEL ##### */
    #define       f_ech               50                           //Choix d'une fréquence en Hz pour le calcul à intervalle régulier entre 1 et 500 Hz (ATTENTION ! 1000 doit être un multiple de f_ech) 
    #define       pleine_echelle_acc  2                             //réglage de la gamme de mesure de l'accéléromètre ±2g ; ±4g ; ±8g ou ±16g
    #define       pleine_echelle_gyr  125                           //réglage de la gamme de mesure du gyromètre ± 125; 250; 500; 1000; 2000 °/s (21; 41; 42; 83; 167; 333 tr/min) pour le LM6DS3 
  
  /* Macro pour modifier les bits spécifiques d'un mots binaires grâce à un masque (0 pour les bits à modifier 1 pour ceux à laisser) */
    #define modif_bit( mot_depart, ajout, masque) ( ( mot_depart & masque ) | ajout )
  
  /* ##### DEFINITION DES CONNEXIONS ##### */
    #define       Bp1Pin              4                             //Port de connexion de l'entrée du bouton poussoir d'etalonnage
    #define       Bp2Pin              5                             //Port de connexion de l'entrée du bouton poussoir de choix du filtre
    #define       Potar0Pin           A0                            //Port de connexion du potentiomètre
    #define       Potar1Pin           A1                            //Port de connexion du potentiomètre
  
  /* ##### DEFINITION DES CONNEXIONS ET DES VARIABLES ##### */
  /* -- Intégrations des bibiothèques -- */
    #include <Wire.h>                                               //Intégration de la biblitothèque "wire" pour la communication I2C
    #include "rgb_lcd.h"                            //Intégration de la biblitothèque "rgb_lcd" pour la commande de l'écran GROVE I2C
    rgb_lcd lcd;                                    //Création de la fonction nommée ici "lcd"
  
  /* -- Définition des variables pour les entrée et mesures -- */
    int32_t       Angle0, Angle1;                                   //Variable pour le calcul de l'angle moyen à partir des potentiomètres
    uint8_t       index = 0;                                        //index pour calculer la moyenne glissante
    uint32_t      sensi_acc         = 61;                           //Sensibilité de l'accéléromètre en milli g/LSB   (réglé par défault pour ± 2   g )
    uint32_t      sensi_gyr         = 8750;                         //Sensibilité du gyromètre       en milli dps/LSB (réglé par défault pour ± 250 °/s)
    
    uint8_t       IMU_add           = 0x6A;                         //Adresse I2c de la centrale inertielle de départ (par defaut celle de la LM6DS3)
    uint8_t       reg_gyr           = 0x22;                         //Adresse de départ de lecture des valeurs du gyromètre de départ (par defaut celle de la LM6DS3)
    uint8_t       reg_acc           = 0x28;                         //Adresse de départ de lecture des valeurs de l'accéléromètre de départ (par defaut celle de la LM6DS3)
    
    
    int16_t       ax_brut, ay_brut, az_brut;                        //Variables pour la lecture des valeurs brutes d'accélérations (accéléromètre)
    int16_t       gx_brut, gy_brut, gz_brut;                        //Variables pour la lecture des valeurs brutes des vitesses angulaires (gyromètre)
    int32_t       ax_offset, ay_offset, az_offset;                  //Variables pour le stockage des valeurs d'offsets de l'accéléromètre
    int32_t       gx_offset, gy_offset, gz_offset;                  //Variables pour le stockage des valeurs d'offsets du gyrocope
    
    /* -- Définition des variables pour les calcul des angles -- */
    float         Ax_reel, Ay_reel, Az_reel;                        //Variables pour le calcul des valeurs réelle de l'accéléromètre valeur_brute * sensibilité / précision
    float         Gx_reel, Gy_reel, Gz_reel;                        //Variables pour le calcul des valeurs réelle du gyromètre
    float         Gx_prec, Gy_prec, Gz_prec;                        //Variables pour le stockage de la valeur précédente pour le calcul des angles avec le gyromètre
    
    float         roulis_gyr, tangage_gyr, lacet_gyr;               //Variables pour le calcul des angles d'inclinaisons avec le gyromètre : ROULIS, TANGAGE et LACET
    float         roulis_acc, tangage_acc;                          //Variables pour le calcul des angles d'inclinaisons avec l'accéléromètre : ROULIS et TANGAGE
    
    volatile long t0;                                               //Variable pour stocker le temps absolu ( fontion millis() avec débordement après 50 jours environ)
    volatile long t1;                                               //Variable pour stocker le temps absolu ( fontion millis() avec débordement après 50 jours environ)
  
  
  
  /**************************************************************
        ROUTINE D'INITIALISATION  (exécutée une seule fois)
   **************************************************************/
  void setup() 
  {
  /* -- Configuration des broches en ENTREE ou SORTIE -- */
    pinMode(Bp1Pin, INPUT);
    pinMode(Bp2Pin, INPUT);
  
  /* -- configuration des fonctions spéciales */
    Serial.begin(115200);                                           //Initialisation de la bibliothèque Moniteur série
    Wire.begin();                                                   //Initialisation de la connexion pour l'I2C
  
  /* -- Affichage temporaire du programme actif dans le µC -- */
    lcd.begin(16, 2);                               //Démarrage et configuration de la fonction LCD pour un écran 2 lignes 16 caractères
    lcd.clear();                                    //Effacement de l'écran
    lcd.setRGB(0, 255, 0);                          //Réglage de la couleur de l'écran en VERT
    lcd.setCursor(0, 0);                            //Positionnement du curseur (caractère,ligne)
    lcd.print(" PLATINE MESURE ");                 //Ecriture du titre du programme sur le LCD
    lcd.setCursor(0, 1);
    lcd.print("CENTRALE INERTIELLE");
    delay(1000); 
  
  /* -- Paramétrage de la centrale et étalonnage -- */  
    reglage_LM6DS3();
    delay(200);
    etalonnage();
  
  /* -- Initialisation du temps initial pour les calculs à intervalle régulier et l'affichage LCD -- */
    t0 = millis();
    t1 = millis();
  }
  
  /**************************************************************
             BOUCLE PRINCIPALE (exécutée en permanence)
   **************************************************************/
  void loop()
  {
    /* -- Lancement de la procédure d'étalonnage en cas d'appui sur le bouton poussoir -- */
    if ( digitalRead(Bp1Pin) == HIGH ) etalonnage();                                      //Appel de la routine d'étalonnage si appui sur le bouton poussoir
  
    /* -- Lecture des valeurs brutes de la centrale -- */
    uint8_t pTampon[6];                                                                   //Variable pour décleer le pointeur vers le stockage d'un tableau dynamique de 6 valeurs pour stocker 6 octets
  
    LECTURE_REGISTRES( IMU_add , reg_acc , 6 , pTampon );
    ax_brut = (pTampon[0] << 8 | pTampon[1] );                                            //Addition binaire des 2 octets
    ay_brut = (pTampon[2] << 8 | pTampon[3] );
    az_brut = (pTampon[4] << 8 | pTampon[5] );
  
    LECTURE_REGISTRES( IMU_add , reg_gyr , 6 , pTampon );
    gx_brut = ( pTampon[0] << 8 | pTampon[1] );
    gy_brut = ( pTampon[2] << 8 | pTampon[3] );
    gz_brut = ( pTampon[4] << 8 | pTampon[5] );
  
    /* -- Calcul des valeurs réelles d'accélérations et de vitesses angulaires  en fonction des valeurs brutes -- */
    Ax_reel = float(ax_brut - ax_offset) * sensi_acc / 1000000;                           //Calcul des valeurs d'accélérations en g en fonction de la sensibilité du capteur
    Ay_reel = float(ay_brut - ay_offset) * sensi_acc / 1000000;                           //Les valeurs brutes sont codées sur 16 bits (0 et 65536)
    Az_reel = float(az_brut - az_offset) * sensi_acc / 1000000;                           //La sensibilité du capteur dépend de la valeur pleine échelle choisie en haut et définie dans les réglages du capteur
    Gx_reel = float(gx_brut - gx_offset) * sensi_gyr / 1000000;                           //Calcul des valeurs de vitesses angulaires en °/s
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
  
   
    float roulis_vitesse  = Gx_reel * cos(tangage_gyr * DEG_TO_RAD) +   Gz_reel * sin(tangage_gyr * DEG_TO_RAD);
    float tangage_vitesse = Gy_reel;
    float lacet_vitesse   = Gy_reel * sin(roulis_gyr * DEG_TO_RAD)  + ( -Gx_reel * sin(tangage_gyr * DEG_TO_RAD) + Gz_reel * cos(tangage_gyr * DEG_TO_RAD) ) * cos(roulis_gyr * DEG_TO_RAD);
     
    /* -- Calcul des angles de ROULIS ET TANGAGE à partir du gyromètre (voir explications des calculs) -- */
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
    //int16_t roulis  = round( -0.2557 * float(analogRead(Potar0Pin)) + 132.56 );     // 0 = 130°  ; 526 = 0° ; 1023 = -130° eq y = -0,2557x + 132,56
    //int16_t tangage = round(  0.2521 * float(analogRead(Potar1Pin)) - 130.93 );     //  0 = -131°  ; 533 = 0° ; 1023 = 132°  eQ y = 0,2521x - 130,93
  
    //int16_t roulis  = map( analogRead(Potar0Pin), 0 , 1023 ,  130 , -133 );     // 0 = 130°  ; 530 = 0° ; 1023 = -133° eq y = -0,2557x + 132,56
    //int16_t tangage = map( analogRead(Potar1Pin), 0 , 1023 , -131 , -132 );     //  0 = -131°  ; 532 = 0° ; 1023 = 132°  eQ y = 0,2521x - 130,93
    /* Potar 1 : -90° = 876 et +90° = 158 | Potar 2 : -90° = 161 et +90° = 885 */
    int16_t roulis  = round( 90 * ( 2 * float(analogRead(Potar0Pin)) - ( 158 + 876 ) ) / ( 158 - 876) );
    int16_t tangage = round( 90 * ( 2 * float(analogRead(Potar1Pin)) - ( 885 + 161 ) ) / ( 885 - 161) );
  
    //int32_t roulis  = analogRead(Potar0Pin);
    //int32_t tangage = analogRead(Potar1Pin);
  
  
  
  
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
  
  
  /* -- AFFICHAGE SUR L'ECRAN LCD -- */
    if ( ( millis() - t1 ) > 200 )
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("R:");
      lcd.print(roulis);
      lcd.print(";");
      lcd.print(roulis_acc , 0);
      lcd.print(";");
      lcd.print(roulis_gyr , 0);
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(tangage);
      lcd.print(";");
      lcd.print(tangage_acc , 0);
      lcd.print(";");
      lcd.print(tangage_gyr , 0);
      t1= millis();
    }
  
  }
  
  
  
  
  /****************************************************
          ROUTINE D'ETALONNAGE calcul des offsets
   ****************************************************/
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
      uint8_t pTampon[6];                                                                   //Variable pour décleer le pointeur vers le stockage d'un tableau dynamique de 6 valeurs pour stocker 6 octets
  
      LECTURE_REGISTRES( IMU_add , reg_acc , 6 , pTampon );
      ax_brut = (pTampon[0] << 8 | pTampon[1] );                                            //Addition binaire des 2 octets
      ay_brut = (pTampon[2] << 8 | pTampon[3] );
      az_brut = (pTampon[4] << 8 | pTampon[5] );
  
      LECTURE_REGISTRES( IMU_add , reg_gyr , 6 , pTampon );
      gx_brut = ( pTampon[0] << 8 | pTampon[1] );
      gy_brut = ( pTampon[2] << 8 | pTampon[3] );
      gz_brut = ( pTampon[4] << 8 | pTampon[5] );
  
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
  
  
  
  
  
  /* -------------------------------------------- *
     ROUTINE DE LECTURE DES VALEUR DE X REGISTRES
     -------------------------------------------- */
  /* Remarque sur l'utilisation des pointeurs *pTableau correspond au pointeur vers le tableau voir https://www.locoduino.org/spip.php?article106 */
  void LECTURE_REGISTRES(uint8_t Add_module, uint8_t Add_Registre, uint8_t Nregistres, uint8_t * pTableau) //Lecture, sur la liaison l'I2C, de plusieurs registres successifs
  {
    Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
    Wire.write(Add_Registre);                                                            //Définition de l'adresse du PREMIER registre
    Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
  
    Wire.requestFrom(Add_module, Nregistres);                                            //Requette de lecture des n octets
    uint8_t index = 0;                                                                   //initilisation d'une variable de comptage
    while (Wire.available()) pTableau[index++] = Wire.read();                            //Tant que des octets sont disponibles enregistrer la valeur de chaque octet dans le tableau et incrémenter l'index
  
  }
  
  /* ------------------------------------------------ *
     ROUTINE D'ECRITURE D'UNE VALEUR DANS UN REGISTRE
     ------------------------------------------------ */
  void ECRITURE_REGISTRE(uint8_t Add_module, uint8_t Add_Registre, uint8_t Valeur)     //Ecriture, sur la liaison l'I2C, d'un une valeur d'un octet dans un registre
  {
    Wire.beginTransmission(Add_module);                                                  //Initialisation de la connexion avec le module (Adresse : du module)
    Wire.write(Add_Registre);                                                            //Définition de l'adresse du registre
    Wire.write(Valeur);                                                                  //Ecriture de la valeur
    Wire.endTransmission();                                                              //Arrêt de l'envoie d'informations sur l'I2C
  }
  
  /* -------------------------------------------------------------------------------- *
     ROUTINE DE REGLAGE DE LA CENTRALE INERTIELLE LSM6DS3 (par l'accès aux registres)
     -------------------------------------------------------------------------------- */
  
  
  void reglage_LM6DS3()
  {
    IMU_add  = 0x6A;                                   //Adresse de la centrale inertielle pour l'accès aux registres
    //reg_temp = 0x20;                                   //Addresse de départ de lecture des valeurs sur 2 octets de la température  32 et 33 (0x20 et 0x21)
    reg_gyr  = 0x22;                                   //Addresse de départ de lecture des valeurs sur 6 octets du gyromètre       34 à  39 (0x22 à  0x27)
    reg_acc  = 0x28;                                   //Addresse de départ de lecture des valeurs sur 6 octets de l'accéléromètre 40 à  45 (0x28 à  0x2D)
  
    uint8_t val_reg_acc, val_reg_gyr;                  //Variables locales pour les réglages
  
    /* -- FUNC_CFG_ACCESS (01h) : Désactive (/active) les fonctions embarquées */
    ECRITURE_REGISTRE( IMU_add , 0x01 , 0b00000000 );
  
  
    /* -- REGISTRE : ORIENT_CFG_G (0Bh) Angular rate sensor sign and orientation register
       OCTET A ENVOYER : | 0 | 0 | SignX_G | SignY_G | SignZ_G | Orient_2 | Orient_1 | Orient_0 |
          - Sign#_G : angular rate sign. Default value: 0 (0: positive sign; 1: negative sign)
          - Orient [2:0]: Directional user-orientation selection. Default value: 000
  
                            | 000 | 001 | 010 | 011 | 100 | 101
             Pitch: tangage |  X  |  X  |  Y  |  Y  |  Z  |  Z
             Roll : Roulis  |  Y  |  Z  |  X  |  Z  |  X  |  Y
             Yaw  : Lacet   |  Z  |  Y  |  Z  |  X  |  Y  |  X
  
       CHOIX DU REGLAGE : Sens positif pour chaque axe ; ROULIS = axe X, TANGAGE = axe Y et LACET = axe Z  => 0b00000010
    */
    ECRITURE_REGISTRE( IMU_add , 0x0B , 0b00000000 );
  
  
    /* -- REGISTRE WHO_AM_I (0Fh) : Lecture seule, renvoie 0b01101010 (6Ah) */
  
    /* -- REGISTRE : CTRL1_XL (10h) Linear acceleration sensor control register 1
       | ODR_XL3 | ODR_XL2 | ODR_XL1 | ODR_XL0 | FS_XL1 | FS_XL0 | BW_XL1 | BW_XL0  |
          - ODR_XL [3:0] : Outpout Data Rate Default value: 0000                            (débit des données)       ## fontion du bit XL_HM_MODE du registre CTRL6 (15h)
          - FS_XL [1:0]  : full-scale Default value: 00                                      (sensibilité)
          - BW_XL [1:0}  : Anti-aliasing filter bandwidth selection. Default value: 00 (00: 400 Hz; 01: 200 Hz; 10: 100 Hz; 11: 50 Hz) #Filtre passe bas
  
             |  ODR_XL :  XL_HM_MODE = 1  :  XL_HM_MODE = 0  | Sensibilité | antialiaising |
             | 0 0 0 0 :    PWR down      :    PWR down      | 00 : ±2  g  | 00 : 400 Hz   |
             | 0 0 0 1 : 13 Hz low pwr    :  13Hz High perf  | 01 : ±16 g  | 01 : 200 Hz   |
             | 0 0 1 0 : 26 Hz  low pwr   :  26Hz  High perf | 10 : ±4  g  | 10 : 100 Hz   |
             | 0 0 1 1 : 52 Hz  low pwr   :  52Hz  High perf | 11 : ±8  g  | 11 :  50 Hz   |
             | 0 1 0 0 : 104 Hz  normal   :  104Hz High perf |             |               |
             | 0 1 0 1 : 208 Hz  normal   :  104Hz High perf |             |               |
             | 0 1 1 0 : 416 Hz High perf : 416Hz High perf  |             |               |
             | 0 1 1 1 : 833 Hz High perf : 833Hz High perf  |             |               |
             | 1 0 0 0 : 1.7kHz High perf : 1.7kHz High perf |             |               |
             | 1 0 0 1 : 3.3kHz High perf : 3.3kHz High perf |             |               |
             | 1 0 1 0 : 6.6kHz High perf : 6.6kHz High perf |             |               |
             | 1 0 X X : Not available    :  Not available   |             |               |
  
       CHOIX DU REGLAGE :
            - ODR           = 416 Hz      0b0110####                                                # Mode High perf donc XL_HM_MODE = 0  reg 15h
            - sensibilité   = choisie au début du programme dans la valeur "pleine_echelle_acc"
            - antialiaising = 400 Hz (0b######00)                                                 1
    */
  #define OdrAndAntialiaising_acc 0b01100000
    if (pleine_echelle_acc == 2)     {
      val_reg_acc  = 0b00000000;
      sensi_acc = 61;
    }
    if (pleine_echelle_acc == 4)     {
      val_reg_acc  = 0b00001000;
      sensi_acc = 122;
    }
    if (pleine_echelle_acc == 8)     {
      val_reg_acc  = 0b00001100;
      sensi_acc = 244;
    }
    if (pleine_echelle_acc == 16)    {
      val_reg_acc  = 0b00000100;
      sensi_acc = 488;
    }
    ECRITURE_REGISTRE( IMU_add , 0x10 , modif_bit( OdrAndAntialiaising_acc , val_reg_acc , 0b11110011 ) );     //la macro modif bit permet de ne modifier que les bits à 0 du masque
  
    /* -- REGISTRE : CTRL2_G (11h) Angular rate sensor control register 2
       OCTET A ENVOYER : | ODR_G3 | ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | FS_125 | 0 |
           - ODR_G [3:0]  : Gyroscope output data rate selection. Default value: 0000        (débit des données)              ## fontion du bit G_HM_MODE du registre CTRL7_G (16h)
           - FS_G  [1:0]  : Gyroscope full-scale selection. Default value: 00                (sensibilité)
           - FS_125       : Gyroscope full-scale at 125 dps. Default value: 0                (forçage sensibilité à 125 °/s)
  
             |  ODR_G  :   G_HM_MODE=1    :    G_HM_MODE=0   |Sensibilité 3 bits|
             | 0 0 0 0 :    PWR down      :    PWR down      | X X 1 : ±125 °/s  |
             | 0 0 0 1 : 12.5Hz low pwr   : 12.5Hz High perf | 0 0 0 : ±245 °/s  |
             | 0 0 1 0 : 26 Hz  low pwr   :  26Hz  High perf | 0 1 0 : ±500 °/s  |
             | 0 0 1 1 : 52 Hz  low pwr   :  52Hz  High perf | 1 0 0 : ±1000 °/s |
             | 0 1 0 0 : 104 Hz  normal   :  104Hz High perf | 1 1 0 : ±2000 °/s |
             | 0 1 0 1 : 208 Hz  normal   :  104Hz High perf |                   |
             | 0 1 1 0 : 416 Hz High perf : 416Hz High perf  |                   |
             | 0 1 1 1 : 833 Hz High perf : 833Hz High perf  |                   |
             | 1 0 0 0 : 1.7kHz High perf : 1.7kHz High perf |                   |
             | 1 0 0 1 : 3.3kHz High perf : 3.3kHz High perf |                   |
             | 1 0 1 0 : 6.6kHz High perf : 6.6kHz High perf |                   |
             | 1 0 X X : Not available    : Not available    |                   |
       CHOIX DU REGLAGE :
            - ODR         = 416 Hz; 0b0110####                                                   #Mode high perf donc G_HM_MODE = 0 reg 16h
            - sensibilité = choisie au début du programme dans la valeur "pleine_echelle_gyr"
    */
  #define ODR_gyr 0b01100000                                                                      //Choix du débit de donnée seuls les 4 bits de poids fort sont à modifier 
    if (pleine_echelle_gyr == 125)    {
      val_reg_gyr = 0b00000010;  //datasheet = 4370  - ATTENTION PAR CALCUL = 3814.75
      sensi_gyr = 4370;
    }
    if (pleine_echelle_gyr ==  245) {
      val_reg_gyr = 0b00000000;  //datasheet = 8750  - ATTENTION PAR CALCUL = 7476.92
      sensi_gyr = 8750;
    }
    if (pleine_echelle_gyr ==  500) {
      val_reg_gyr = 0b00000100;  //datasheet = 17500 - ATTENTION PAR CALCUL = 15259  17500
      sensi_gyr = 15259;
    }
    if (pleine_echelle_gyr == 1000) {
      val_reg_gyr = 0b00001000;  //datasheet = 35000 - ATTENTION PAR CALCUL = 30518
      sensi_gyr = 35000;
    }
    if (pleine_echelle_gyr == 2000) {
      val_reg_gyr = 0b00001100;  //datasheet = 70000 - ATTENTION PAR CALCUL = 61036.1
      sensi_gyr = 70000;
    }
    ECRITURE_REGISTRE( IMU_add , 0x11 , modif_bit( ODR_gyr, val_reg_gyr , 0b11110001 ) );           //la fonction modif bit permet de ne modifier que les bits à 0 du masque
  
    /* -- REGISTRE : CTRL3_C (12h) Control register 3
       OCTET A ENVOYER : | BOOT | BDU | H_LACTIVE | PP_OD | SIM | IF_INC | BLE | SW_RESET |
           - BOOT        : Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content)
           - BDU         : Block Data Update. Default value: 0 (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
           - H_LACTIVATE : Interrupt activation level. Default value: 0 (0: interrupt output pads active high; 1: interrupt output pads active low)
           - SIM         : SPI Serial Interface Mode selection. Default value: 0 (0: 4-wire interface; 1: 3-wire interface).
           - IF_INC      : Register address automatically incremented during a multiple byte access with a serial interface (I2C or SPI). Default value: 1 (0: disabled; 1: enabled)
           - BLE         : Big/Little Endian Data selection. Default value 0 (0: data LSB @ lower address; 1: data MSB @ lower address)
           - SW_RESET    : Software reset. Default value: 0 (0: normal mode; 1: reset device) This bit is cleared by hardware after next flash boot.
  
       CHOIX DU REGLAGE :  0b00000110
           - BLE =1 pour inverser les octets des valeurs brutes afin d'être identique des IMU TDK InvenSense
                 Les données brutes d'accélérations et de vitesses angulaires sont codées sur 2 octets, par défaut
                   . dans les IMU de TDK InvenSense le premier octet correspond aux bits de poids fort et les second aux bits de poids faibles
                   . dans les IMU de STMicroelectronics c'est l'inverse.
                 Comme j'utilise la même routine pour extraire les données brutes, et que seul le LM6DS3 possède ce réglage je l'applique ici.
    */
    ECRITURE_REGISTRE( IMU_add , 0x12 , 0b00000110 );
  
  
  
    /* -- REGISTRE : CTRL4_C (13h) Control register 4
       OCTET A ENVOYER : | XL_BW_SCAL_ODR | SLEEP_G | INT2_on_INT1 | FIFO_TEMP_EN | DRDY_MASK  | I2C_disable | MODE3_EN | STOP_ON_FTH |
           - XL_BW_SCAL_ODR : Accelerometer bandwidth selection. Default value: 0                                ## Relation avec les réglages d'ODR_XL dans le CTRL1_XL (10h)
           - SLEEP_G        : Gyroscope sleep mode enable. Default value: 0 (0: disabled; 1: enabled)
           - I2C_disable    : Disable I2C interface. Default value: 0 (0: both I2C and SPI enabled; 1: I2C disabled, SPI only)
           - MODE3_EN       : Enable auxiliary SPI interface (Mode 3, refer to Table 2). Default value: 0 (0: auxiliary SPI disabled; 1: auxiliary SPI enabled)
  
             |       ODR       |  Analog filter BW (XL_HM_MODE = 0) |
             |                 | XL_BW_SCAL_ODR = 0 | XL_BW_SCAL_ODR = 1 |
             | 6.66 - 3.33 kHz |  Filter not used   |                    |
             |   1.66 kHz      |       400 Hz       |     Bandwidth is   |
             |    833 Hz       |       400 Hz       |   determined by    |
             |     416 Hz      |       200 Hz       | setting BW_XL[1:0] |
             |     208 Hz      |       100 Hz       |    CTRL1_XL (10h)  |
             |   104 - 13 Hz   |       150 Hz       |                    |
  
       CHOIX DU REGLAGE :   0b10000000
           - XL_BW_SCAL_ODR = 1 pour régler la bande passante avec le registre 10h
    */
    ECRITURE_REGISTRE( IMU_add , 0x13 , 0b10000000 );
  
    /* -- REGISTRE : CTRL5_C (14h) Control register 5
       OCTET A ENVOYER : | ROUNDING2 | ROUNDING1 | ROUNDING0 | 0 | ST1_G | ST0_G | ST1_XL | ST0_XL |
           - ROUNDING[2:0] Circular burst-mode (rounding) read from the output registers. Default value: 000
                             000 No rounding
                             001 Accelerometer only
                             010 Gyroscope only
                             011 Gyroscope + accelerometer
                             100 Registers from SENSORHUB1_REG (2Eh) to SENSORHUB6_REG (33h) only
                             101 Accelerometer + registers from SENSORHUB1_REG (2Eh) to SENSORHUB6_REG (33h)
                             110 Gyroscope + accelerometer + registers from SENSORHUB1_REG (2Eh) to SENSORHUB6_REG (33h) and registers from SENSORHUB7_REG (34h) to SENSORHUB12_REG (39h)
                             111 Gyroscope + accelerometer + registers from SENSORHUB1_REG (2Eh) to SENSORHUB6_REG (33h)
       CHOIX DU REGLAGE : défaut
    */
    ECRITURE_REGISTRE( IMU_add , 0x14 , 0b00000000 );
  
  
    /* -- REGISTRE : CTRL6_C (15h) Control register 6
       | TRIG_EN | LVL_EN | LVL2_EN | XL_HM_MODE | 0 | 0 | 0 | 0 |
           - XL_HM_MODE : High-performance operating mode disable for accelerometer. Default value: 0                    ## Modifie le mode de l' ODR_C du registre CTRL1_XL (10h)
  
       CHOIX DU REGLAGE : 0b00000000
             Mode high per donc XL_HM_MODE = 0 (voir Registre CTRL1_XL (10h)
    */
    ECRITURE_REGISTRE( IMU_add , 0x15 , 0b00000000 );
  
  
    /* -- REGISTRE : CTRL7_G (16h) Angular rate sensor control register 7
       | G_HM_MODE | HP_EN_G | HPCF_G1 | HPCF_G0 | HP_G_RST | ROUNDING_STATUS| 0 | 0 |
           - G_HM_MODE       : High-performance operating mode disable for gyroscope. Default value: 0                   ## Modifie le mode de l' ODR_G du registre CTRL2_G (11h)
           - HP_EN_G         : Gyroscope high-pass filter enable. Default value: 0 (0: HPF disabled; 1: HPF enabled)
           - HPM_G[1:0]      : Gyroscope high-pass filter cutoff frequency selection. Default value: 00 (00 = 0.0081 Hz ; 01 = 0.0324 Hz ; 10 = 2.07 Hz ; 11 = 16.32 Hz)
           - HP_G_RST        : Gyro digital HP filter reset. Default: 0Gyro digital HP filter reset. Default: 0 (0: gyro digital HP filter reset OFF; 1: gyro digital HP filter reset ON)
           - ROUNDING_STATUS : Source register rounding function enable on STATUS_REG (1Eh), FUNC_SRC (53h) and WAKE_UP_SRC (1Bh) registers. Default value: 0 (0: disabled; 1: enabled)
  
       CHOIX DU REGLAGE : 0b11000000
            - G_HM_MODE  = Mode high perf donc G_HM_MODE = 0 (voir Registre CTRL2_XL (11h))
            - HP_EN_G = 1  Activation du filtre passe haut
            - HPM_G[1:0] = 00  Valeur la plus faible possible pour filtrer les vibrations et éviter la dérive !!!! A VERIFIER
    */
    ECRITURE_REGISTRE( IMU_add , 0x16 , 0b00000000 );
  
  
  
    /* -- REGISTRE : CTRL8_XL (17h) Linear acceleration sensor control register 8
       | LPF2_XL_EN | HPCF_XL1 | HPCF_XL0 | 0 | 0 | HP_SLOPE_XL_EN | 0 | LOW_PASS_ON_6D |
           - LPF2_XL_EN      : Accelerometer low-pass filter LPF2 selection. Refer to Figure 5  (diagramme de choix des filtres de l'accélomètre )
           - HPCF_XL[1:0]    : Accelerometer slope filter and high-pass filter configuration and cutoff setting. It is also used to select the cutoff frequency of the LPF2 filter
                               This low-pass filter can also be used in the 6D/4D functionality by setting the LOW_PASS_ON_6D bit to 1
           - HP_SLOPE_XL_EN  : Accelerometer slope filter / high-pass filter selection. Refer to Figure 5.
           - LOW_PASS_ON_6D  : Low-pass filter on 6D function selection. Refer to Figure 5.                         ## POUR LES FONCTIONS EMBARQUEES
  
             | HPCF_XL[1:0] | Applied filter | HP filter cutoff frequency [Hz] | LPF2 digital filter cutoff frequency [Hz] |
             |      0 0     |  Slope (pente) |             ODR_XL/4            |               ODR_XL/50                   |
             |      0 1     |    High-pass   |             ODR_XL/100          |               ODR_XL/100                  |
             |      1 0     |    High-pass   |             ODR_XL/9            |               ODR_XL/9                    |
             |      1 1     |    High-pass   |             ODR_XL/400          |               ODR_XL/400                  |
  
       CHOIX DU REGLAGE : 0b11100100      filtre passe haut de l'accéléromètre le plus bas possible donc /400
           - LPF2_XL_EN = 1
           - HPCF_XL[1:0] = 11
           - HP_SLOPE_XL_EN = 1
           - LOW_PASS_ON_6D = 0
  
    */
    ECRITURE_REGISTRE( IMU_add , 0x17 , 0b00000000 );
  
    /* -- REGISTRE : CTRL9_XL (18h) Linear acceleration sensor control register 9 (r/w)
        | 0 | 0 | Zen_XL | Yen_XL | Xen_XL | SOFT_EN | 0 | 0 |
            - #en_XL      : Accelerometer Z-axis output enable. Default value: 1 (0: #-axis output disabled; 1: #-axis output enabled)
            - SOFT_EN     : Enable soft-iron correction algorithm for magnetometer(1). Default value: 0 (disabled)
    */
    ECRITURE_REGISTRE( IMU_add , 0x18 , 0b00111000 );
  
    /* -- REGISTRE : CTRL10_C (19h) Control register 10 (r/w)
        | 0 | 0 | Zen_G | Yen_G | Xen_G | FUNC_EN | PEDO_RST_STEP | SIGN_MOTION_EN |
            - #en_G          : Gyroscope yaw axis (#) output enable. Default value: 1 (0: #-axis output disabled; 1: #-axis output enabled)
            - SOFT_EN        : Enable embedded functionalities   and accelerometer HP and LPF2 filters (refer to Figure 5). Default value: 0 (disable)
                                                               (pedometer, tilt, significant motion, sensor hub and ironing)
            - PEDO_RST_STEP  : Reset pedometer step counter. Default value: 0 (0: disabled; 1: enabled)
            - SIGN_MOTION_EN : Enable significant motion function. Default value: 0 (0: disabled; 1: enabled)
    */
    ECRITURE_REGISTRE( IMU_add , 0x19 , 0b00111000 );
  }
