# IMU_Gyroscope
<p align="center">
<img src="Platine IMU_Bleutooth.jpg" alt="Moteur à l'arret" height=200>
<img src="Platine IMU_Bluetooth 2.jpg" alt="Moteur en rotation" height=200> 
</p>

## Présentation

La platine proposée est un gyroscope à 3 cardans dont 2 sont équipés de capteurs de rotations afin de proposer :
- la mesure la plus précise possible des angles « réels » entre les cardans correspondant au ROULIS et au TANGAGE,
- le calcul des angles à partir de la centrale,
- l’affichage sur un écran LCD des 6 angles (3 pour le ROULIS et 3 pour le TANGAGE) dans l’ordre suivant :
  - L’angle « réel » ; 
  - L’angle calculé à partir de l’accéléromètre ; 
  -	L’angle calculé avec le gyromètre,

## Matériel utilisé
- Un Arduino UNO
- Une centrale inertielle Grove_IMU_6 DOF : [wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope](https://wiki.seeedstudio.com/Grove-6-Axis_AccelerometerAndGyroscope/) 
- Un écran LCD RGB BLACKLIGHT : [wiki.seeedstudio.com/Grove-LCD_RGB_Backlight](https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/)
- Deux potentiomètres  Grove Rotary_Angle_Sensor : [wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor](https://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/)

## Modèle numérique 3D
Elle est conçue pour utiliser des modules Grove que ce soit pour les centrales inertielles ou les potentiomètres.

La conception de la platine reste à optimiser en particulier le design qui reste grossier mais surtout les liaisons pivots et la solution de blocage qui sont loin d’être optimales.
La mesure de l’ange à partir du potentiomètre n’est pas des plus précise du fait de l’offset présent lorsque la platine est horizontale et du manque de linéarité du potentiomètre.
Les modèles SolidWorks, les fichiers stl pour l’impression ainsi que le programme arduino commenté sont proposés dans les fichiers associés.



Pour tout renseignement complémentaire [gael.balduini@gmail.com](mailto:gael.balduini@gmail.com)





