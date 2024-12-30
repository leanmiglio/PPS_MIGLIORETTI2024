// INCLUSION DE LIBRERIAS A UTILIZAR
  #include <AccelStepper.h>
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  #include <Encoder.h>
  #include <math.h>
//




bool mensajeMostrado = false;
unsigned long mensajeStartTime = 0; // Temporización para el mensaje
bool enMovimiento = false;    // Indica si los motores están en movimiento
bool mostrarMensaje = false;  // Indica si se debe mostrar el mensaje de "Punto Alcanzado"
bool movimientoCompletado = false;   // Indica si el movimiento se ha completado
unsigned long tiempoMensaje = 0;     // Para controlar el tiempo de mostrar el mensaje
unsigned long mensajeCompletoStartTime = 0; 
bool mensajeCompletoMostrado = false;

unsigned long mostrarMensajeStartTime = 0;
const unsigned long mostrarMensajeDuration = 2000;




// Declaraciones de funciones


// DEFINICION DE PIN'S DE INPUT/OUTPUT
  //DEFINICION DE PIN'S DISPLAY, BUSSER Y ENCODER
    LiquidCrystal_I2C lcd(0x27, 16, 2);
    Encoder myEnc(51, 52);
    const int SwitchEncPin = 53;
    #define BUZZER 5

  /// DEFINIMOS LOS PINES DE CONTROL DE LOS MOTORES
    // PINES DE HABILITACION
      #define ENAPIN1 39
      #define ENAPIN2 41
      #define ENAPIN3 43
      #define ENAPIN4 45
    // PINES DE PULSO Y DIRECCION
      #define PULPIN1 37
      #define PULPIN2 35
      #define PULPIN3 33
      #define PULPIN4 31
      #define DIRPIN1 36
      #define DIRPIN2 34
      #define DIRPIN3 32
      #define DIRPIN4 30
  // DEFINIMOS LOS PINES DE LOS FINALES DE CARRERA
      #define LIMMIN1 22
      #define LIMMAX1 23 
      #define LIMMIN2 24
      #define LIMMAX2 25
      #define LIMMIN3 26
      #define LIMMAX3 27
      #define LIMMIN4 28
      #define LIMMAX4 29

  // DEFINICION A MAS BAJO NIVEL DE LOS PIN'S DE LOS SWITCHS
      #define LIMIT_SWITCH_PIN PINA
      #define LIMIT_SWITCH_DDRA DDRA
      #define LIMIT_SWITCH_PORT PORTA

    // DEFINICION A MAS BAJO NIVEL DE LOS PIN'S PARA DIR Y PUL EN PUERTO C
      #define MOTOR_PIN_DIR_PUL PORTC
      #define MOTOR_PIN_DDR DDRC

  // Indicadores visuales (LED)
    #define RED_LED_PIN 48
    #define YELLOW_LED_PIN 49
    #define GREEN_LED_PIN 47

  // PULSADORES
    #define EMERGENCY_STOP_PIN 2
    #define BOTON1 40
    #define BOTON2 42
//

//CONTROL DEL ROBOT-VARIABLES
  //Variables y constantes del motor
    const int stepsPerRevolution = 200;
    const int RETROTRAER_STEPS[4]= {40,30,30,10}; // Cantidad de pasos para retroceder
    //const int retrotraer_1 = 40; 
    //const int retrotraer_2 = 30;
    //const int retrotraer_3 = 30;
    //const int retrotraer_4 = 10;
    const uint8_t MOTOR1_DIR_MASK = 0b00000001;
    const uint8_t MOTOR1_PUL_MASK = 0b00000010;
    const uint8_t MOTOR2_DIR_MASK = 0b00000100;
    const uint8_t MOTOR2_PUL_MASK = 0b00001000;
    const uint8_t MOTOR3_DIR_MASK = 0b00010000;
    const uint8_t MOTOR3_PUL_MASK = 0b00100000;
    const uint8_t MOTOR4_DIR_MASK = 0b01000000;
    const uint8_t MOTOR4_PUL_MASK = 0b10000000;

    AccelStepper MOTORS[4]= {
      AccelStepper (AccelStepper::DRIVER, PULPIN1, DIRPIN1),
      AccelStepper (AccelStepper::DRIVER, PULPIN2, DIRPIN2),
      AccelStepper (AccelStepper::DRIVER, PULPIN3, DIRPIN3),
      AccelStepper (AccelStepper::DRIVER, PULPIN4, DIRPIN4),
    };

    //Relaciones de transmision
      const float relacionMOTOR1 = 475.0 / 32.0;
      const float relacionMOTOR2 = 1200.0 / 44.0;
      const float relacionMOTOR3 = 700.0 / 90.0;
    //

    // VARIABLES DE CINEMATICA
      //Longitud de eslabones
        float L1 = 260;
        float L2 = 210;
        float d1 = 120;
      //

      //Variables para los limites de los angulos/desplazamientos
        // Límites de movimiento
          const float q1Min = -90.0 * DEG_TO_RAD ;
          const float q1Max = 90.0 * DEG_TO_RAD ;
          const float q2Min = -135.0 * DEG_TO_RAD ;
          const float q2Max = 135.0 * DEG_TO_RAD ;
          const float q3Min = 0.0;
          const float q3Max = 440.0;
          long pasosMotor1 = 0, pasosMotor2 = 0, pasosMotor3 = 0;
        //

        float q1 = 0, q2 = 0, q3 = 0;  

        float X_destino = 0.0;
        float Y_destino = 0.0;
        float Z_destino = 0.0;
      //

      //
          unsigned long PTGStartTime = 0;
          bool PTGMostrar = false;
          bool iniciarPTG = false;
          bool PTGCompleta = false;
      //

      // Variables para gestión no bloqueante
        unsigned long lastRunTime = 0;
        const unsigned long runInterval = 20; // Intervalo de actualización
      //
    //
  //

  //Variables de los Switchs
      const long delay_switchs = 250;

    const uint8_t LIMIT_SWITCH_MASKS[4][2] = {
     {0b00000001, 0b00000010}, // Switches para el motor 1
     {0b00000100, 0b00001000}, // Switches para el motor 2
     {0b00010000, 0b00100000}, // Switches para el motor 3
     {0b01000000, 0b10000000}  // Switches para el motor 4
     };
  //

  // Variables MODO MANUAL

    //Variables estado motores
    //

    //Variables Mover motores individualmente
    int ang_m1 = 0;
    int pasos_m1 = 0;

    int ang_m2 = 0;
    int pasos_m2 = 0;

    int ang_m3 = 0;
    int pasos_m3 = 0;

    int ang_m4 = 0;
    int pasos_m4 = 0;
    //

  //

  // Variables de SET ORIGEN
   // Variable global para el tiempo no bloqueante
      unsigned long origenAlcanzadoStartTime = 0;
      bool origenAlcanzadoMostrar = false;
   //

    bool iniciarEstablecerOrigen = false;
    bool EstablecerOrigenCompleta = false;

    bool iniciarIrAlOrigen = false;
    bool IrAlOrigenCompleta = false;
  //

  // Variables RECALIBRACION

   // Variable global para el tiempo no bloqueante
      unsigned long recalibracionAbsolutaStartTime = 0;
      bool recalibracionAbsolutaMostrar = false;
   //

    bool iniciarRecalibracionAbsoluta = false;
    bool recalibracionAbsolutaCompleta = false;
  //


  //Variables de SetYCalibracion
      long calib_m1 = 0;
      long calib_m2 = 0;
      long calib_m3 = 0;
      long calib_m4 = 0;

      const int verif_speed=330; //VELOCIDAD PARA "SET Y CALIBRACION"

      uint8_t global_first_verify;
      uint8_t global_first_retro;
      int global_steps_retro;
      bool verificacionCompleta = false;
      bool iniciarVerificacion = false;  // flag para controlar cuándo iniciar la verificación

    volatile int POSICIONES_VYC[4][2] = {
      {0, 0}, // Switches para el motor 1
      {0, 0}, // Switches para el motor 2
      {0, 0}, // Switches para el motor 3
      {0, 0}  // Switches para el motor 4
      };
  //

  //Variables y constantes de movimiento
      const int normalSpeed = 400; //VELOCIDAD DEFAULT DEL ROBOT
      const int slowSpeed = 20;
      const int maxSpeed = 200;
      const int acceleration = 100;
      const int VELOCIDAD_MIN = 1;
      const int VELOCIDAD_MAX = 1000;
      const int ACELERACION_MIN = 1;
      const int ACELERACION_MAX = 1000;

      // Variables "X, Y, Z" para la opción "PuntoToGo"  DEFAULTT
      int X = 140;
      int Y = 100;
      int Z = 120;
  //

  //Variables y constantes de interpolacion
   long pasosEntreSwitches[4] = {0, 0, 0, 0};  // Un valor para cada motor
   const int PASOS_PRUEBA[4]= {500,200,300,100};
  //

  // Flags, auxiliares, generales, etc
    bool flag6 = false;
    bool flagProteccion = false;
    bool mostrandoPasos = false;
    unsigned long mostrarPasosStartTime = 0;
    int motorMostrandoIndex = 0;
    const unsigned long tiempoMostrarMotor = 2000;  // Tiempo de 2 segundos para mostrar cada motor
    bool  moveFlag = true;
    bool flagSTR = false;
    bool proteccion = false;
    char direccion='L'; ///por defecto nuestro robot calibra hacia L


    bool emergencyStop = false;
    bool ldr_proteccion = false;
    bool resumePressed = false;

    // Variables para el Buzzer (EMERGENCIA)
    unsigned long buzzerEmergencyEndTime = 0; // Tiempo para el zumbido de emergencia
    const unsigned long buzzerEmergencyInterval = 500; // Intervalo del zumbido (en ms)

    // Variables para el Buzzer (DISPLAY)
    unsigned long buzzerStartTime = 0;
    unsigned long buzzerDuration = 100;  // Duración del zumbido en milisegundos
    bool isBuzzerOn = false;

    //Variables para control del Encoder
    long oldPosition = 0; // Para almacenar la posición anterior del encoder
    unsigned long lastEncoderChangeTime = 0; // Última vez que se cambió la posición del encoder
  //





//

//CONSTANTES Y VARIABLES ASOCIADAS AL DISPLAY LCD

  //MENU ESTATUS GENERAL

    //ESTADOS:
    enum MenuState { MAIN_MENU,
                        SUB_MENU_HOME,
                        SUB_MENU_PUNTOTOGO,
                        SUB_MENU_MODELO,
                          CONFIRM_MODELO,
                        SUB_MENU_AJUSTE_PARAMETROS,
                        SUB_MENU_SETCALIBRACION,
                          SUB_MENU_CAMBIAR_DIRECCION, 
                        SUB_MENU_SET_ORIGEN, 
                        SUB_MENU_MOV_PARALELO, 
                        SUB_MENU_ESTABLECER_RUTA,
                        SUB_MENU_MODO_MANUAL, 
                          SUB_MENU_ESTADO_MOTORES,
                          SUB_MENU_MOV_INDIVIDUAL,
                            SUB_MENU_M1,
                            SUB_MENU_M2,
                            SUB_MENU_M3,
                            SUB_MENU_M4
                        } 
                        
                        currentMenuState = MAIN_MENU;
    //


    //MENU PRINCIPAL ----SECCIONES----
    String mainMenu[9] = {"Home",
                          "PuntoToGo", 
                          "Set y Calibracion", 
                          "Ajuste de parametros", 
                          "Modelo de movimiento", 
                          "Modo Manual", 
                          "SET Origen", 
                          "Ej.M Paralelo", 
                          "Rutas"};
    int menuIndex = 0;
    //

    // SUBMENU: "MODELO DE MOVIMIENTO"
      int MODELO = 0; // 0 = Cinemática Directa, 1 = Cinemática Inversa
      String modeloMovimientoMenu[3] = {"Cinematica Directa", "Cinematica Inversa", "<- ATRAS"}; //TEXTO VISUALIZADO EN EL MENU
      int modeloMovimientoIndex = 0; // Índice del menú "Modelo de Movimiento"
      bool confirmChange = false; // Estado para confirmar cambio de modelo
    //

    // SUBMENU "AJUSTE DE PARAMETROS"
      String ajusteParametrosMenu[5] = {"Velocidad", "Aceleracion", "Curva de Aceleracion", "Inicio Set", "<- ATRAS"}; //TEXTO VISUALIZADO EN EL MENU
      int ajusteParametrosIndex = 0; // Índice del menú de Ajuste de Parametros
      bool editingParametros = false; // Estado para saber si se está editando una variable de Ajuste de Parametros

      int velocidad = 50;  // Rango 0 a 100
      int aceleracion = 50; // Rango 0 a 100

      //SUB-SUBMENU "Curvas de aceleracion"
        String curvasAceleracion[5] = {"Tipo 1", "Tipo 2", "Tipo 3", "Tipo 4", "<- ATRAS"}; //TEXTO VISUALIZADO EN EL MENU
        int curvaAceleracionIndex = 0; // Índice para la curva de aceleración
      //
    //

    //SUBMENU "SET Y CALIBRACION"
    String setCalibracionMenu[3] = {"Cambiar Direccion", "Ejecutar", "<- ATRAS"}; //TEXTO VISUALIZADO EN EL MENU
    int setCalibracionIndex = 0; // Índice del menú "Set y Calibracion"
    String inicioSetCalibracion = "LEFT"; // Opciones: LEFT or RIGHT
      //SUB-SUBMENU "CAMBIAR DIRECCION"
        String cambiarDireccionMenu[3] = {"LEFT", "RIGHT", "<- ATRAS"};
        int cambiarDireccionIndex = 0; // Índice para el submenú "Cambiar Direccion"
      //
    //

    //SUBMENU "PUNTO TO GO"
      String puntoToGoMenu[5] = {"X", "Y", "Z", "Confirmar", "<- ATRAS"};
      int puntoToGoIndex = 0; // Índice del menú de PuntoToGo
      bool editing = false; // Estado para saber si se está editando una variable

      // Límites de las variables
      const int minValXPTG = 50;
      const int maxValXPTG = 470;
      const int minValYPTG = 50;
      const int maxValYPTG = 470;
      const int minValZPTG = 120;
      const int maxValZPTG = 560;


    //

    //SUBMENU "HOME"
      String homeMenu[3] = {"Ejecutar", "NO", "<- ATRAS"};
      int homeMenuIndex = 0; // Índice del menú de HOME
    //

    //SUBMENU "MODO MANUAL"
      String modoManualMenu[3] = {"Estado Motores", "Mover individualmente", "<- ATRAS"};
      int modoManualIndex = 0; // Índice del menú "Modelo de Movimiento"

       //SUB-SUBMENU ESTADO MOTORES
          String estadoMotoresMenu[3] = {"Liberar", "Energizar", "<- ATRAS"};
          int estadoMotoresIndex = 0; // Índice del menú de HOME
       //

       //SUB-SUBMENU MOVIMIENTO INDIVIDUAL
          String movIndividualMenu[5] = {"MOTOR 1", "MOTOR 2", "MOTOR 3", "MOTOR 4", "<- ATRAS"};
          int movIndividualIndex = 0; // Índice del menú de HOME

            //SUB-SUB-SUBMENU MOTORES
              //MOTOR1
                String motor1Menu[4] = {"Ang", "Pasos", "Mover", "<- ATRAS"};
                int motor1Index = 0; 
                bool editingM1 = false;
                const int minValANG1 = -90;
                const int maxValANG1 = 90;
                const int minValPAS1 = -1336;
                const int maxValPAS1 = 1336;

              //MOTOR2
                String motor2Menu[4] = {"Ang", "Pasos", "Mover", "<- ATRAS"};
                int motor2Index = 0; 
                bool editingM2 = false;
                const int minValANG2 = -220;
                const int maxValANG2 = 220;
                const int minValPAS2 = -6000;
                const int maxValPAS2 = 6000;

              //MOTOR3
                String motor3Menu[4] = {"Ang", "Pasos", "Mover", "<- ATRAS"};
                int motor3Index = 0;  
                bool editingM3 = false;
                const int minValANG3 = -135;
                const int maxValANG3 = 135;
                const int minValPAS3 = -1050;
                const int maxValPAS3 = 1050
                
                ;

              //MOTOR4
                String motor4Menu[4] = {"Ang", "Pasos", "Mover", "<- ATRAS"};
                int motor4Index = 0;
                bool editingM4 = false;
                const int minValANG4 = -6000;
                const int maxValANG4 = 6000;
                const int minValPAS4 = -6000;
                const int maxValPAS4 = 6000;
              //
            //
        //
    //

    
    //SUBMENU "SET ORIGEN"
      String setOrigenMenu[4] = {"Ir a Origen", "Establecer nuevo Origen", "Ir a Origen Absoluto", "<- ATRAS"};
      int setOrigenIndex = 0; // Índice del menú de HOME
    //

    //SUBMENU "EJECUTAR MOVIMIENTO PARALELO"
      String movParaleloMenu[4] = {"Establecer plano", "Establecer puntos", "Ejecutar", "<- ATRAS"};
      int movParaleloIndex = 0; // Índice del menú de HOME

      //SUB-SUBMENU ESTABLECER PLANO
          String planoParalelo[3] = {"X Constante (YZ)", "Y Constante (XZ)", "<- ATRAS"};
          int planoParaleloIndex = 0; // Índice del menú de HOME
       //

        //SUB-SUBMENU ESTABLECER PUNTOS
          // X CONSTANTE
            String paraleloX[5] = { "Est. Origen","Est. Destino", "<- ATRAS", "Menu Principal"};
            int paraleloXIndex = 0; // Índice del menú de PuntoToAdd
            bool editingParaleloX = false; // Estado para saber si se está editando una variable
          
            // SUB-SUB-SUBMENU ESTABLECER ORIGEN

            //

            // SUB-SUB-SUBMENU ESTABLECER DESTINO


          // Y CONSTANTE
            String paraleloY[5] = {"Est. Origen","Est. Destino", "<- ATRAS"};
            int paraleloYIndex = 0; // Índice del menú de PuntoToAdd
            bool editingParaleloY = false; // Estado para saber si se está editando una variable
          //
        //


      // Límites de las variables
        const int minValXParalelo = 20;
        const int maxValXParalelo = 60;
        const int minValYParalelo = 20;
        const int maxValYParalelo = 60;
        const int minValZParalelo = 20;
        const int maxValZParalelo = 60;
      //
    //
    //

    //SUBMENU "ESTABLECER RUTA"
      String rutaMenu[5] = {"Nueva ruta", "Ver rutas", "Añadir puntos", "Ejecutar rutas", "<- ATRAS"};
      int rutaMenuIndex = 0; // Índice del menú de HOME

      //SUB-SUBMENU AÑADIR PUNTOS
      String puntoToAdd[6] = {"X", "Y", "Z", "Sel. Ruta", "Añadir", "<- ATRAS"};
        int puntoToAddIndex = 0; // Índice del menú de PuntoToAdd
        bool editingAdd = false; // Estado para saber si se está editando una variable

        // Límites de las variables
        const int minValXAdd = 50;
        const int maxValXAdd = 470;
        const int minValYAdd = 50;
        const int maxValYAdd = 470;
        const int minValZAdd = 120;
        const int maxValZAdd = 560;
      //
    //


  //

  //DESPLAZAMIENTO DE TEXTO EN EL DISPLAY
    unsigned long lastScrollTime = 0; // Tiempo de la última actualización del desplazamiento
    int scrollIndex = 0; // Índice actual de desplazamiento
    String currentScrollingText = ""; // Texto actualmente en desplazamiento
    bool isScrolling = false; // Estado de si el texto debe desplazarse

  //
//

// Variables para temporización no bloqueante
  unsigned long lastDebounceTime = 0; // Tiempo de la última interacción con el encoder
  const unsigned long debounceDelay = 300; // Retraso de debounce para el botón
  unsigned long buzzerEndTime = 0; // Tiempo de finalización para el BUZZER
  unsigned long puntoToGoEndTime = 0; // Tiempo de finalización para la función PUNTOTOGO
  bool isPuntoToGoRunning = false; // Estado de ejecución de PUNTOTOGO
//




///-----------------------------------------------------FUNCIONES------------------------------------------------------------------

void selectValuesMotor(int motorIndex, char replace_direction, char sentido_retraccion) {
  global_first_verify = (replace_direction == 'L') ? LIMIT_SWITCH_MASKS[motorIndex][0] : LIMIT_SWITCH_MASKS[motorIndex][1];
  global_first_retro = (sentido_retraccion == 'R') ? LIMIT_SWITCH_MASKS[motorIndex][1] : LIMIT_SWITCH_MASKS[motorIndex][0];
  global_steps_retro = RETROTRAER_STEPS[motorIndex];

  // Mostrar el mensaje en el LCD de forma no bloqueante
    static unsigned long displayStartTime = 0;
    displayStartTime = millis(); // Marca el tiempo actual para la visualización del mensaje
}


//FUNCION CHEQUEO PARADA DE EMERGENCIA
  ISR(TIMER1_COMPA_vect) {
      moveFlag = true; // Establece la bandera de movimiento
  }


  void checkEmergencyStop() {
      if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {  // Si el botón de emergencia está presionado
          emergencyStop = true;  // Activa la parada de emergencia

          // Mostrar "EMERGENCIA" en el LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("EMERGENCIA");
          lcd.setCursor(0, 1);
          lcd.print("Stopped!");

          // Detener los motores
          for (int i = 0; i < 4; i++) {
              MOTORS[i].stop();  // Detener el motor
              digitalWrite(ENAPIN1 + (i * 2), HIGH);  // Deshabilitar el motor
          }

          // Iniciar el zumbido de emergencia
      } else {
          if (emergencyStop) {
              emergencyStop = false;

              // Mostrar mensaje en el LCD para reiniciar
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Reinicie el");
              lcd.setCursor(0, 1);
              lcd.print("sistema");

              // Regresar al menú principal
              currentMenuState = MAIN_MENU;
              updateMenu();
          }
      }
  }
//






void setup() {

  //CONFIGURACION DE LOS PINES BUZZER Y ENCODER INPUT/OUTPUT
    pinMode(SwitchEncPin, INPUT_PULLUP); 
    pinMode(BUZZER, OUTPUT);  // Configura el pin del zumbador como salida
    digitalWrite(BUZZER, HIGH);  // Asegura que el buzzer esté apagado
  //

  // PARADA DE EMERGENCIA
    //sei(); cli();
    pinMode(EMERGENCY_STOP_PIN, INPUT); 
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), checkEmergencyStop, FALLING); // Asocia la interrupcion al pin de la parada de emergencia
  //

  
  // CONFIGURACION INPUT/OUTPUT PINES
    pinMode(EMERGENCY_STOP_PIN, OUTPUT); //pin del led de parada de emergencia definido como OUTPUT
    digitalWrite(EMERGENCY_STOP_PIN, LOW); // led parada de emergencia inicio apagado
    DDRA = 0x00;  // Confugura todos los pines del puerto A como entradas ---> switchs
    DDRC = 0xFF;  // Configura todos los pines del puerto C como salidas ---> DIR Y PUL
  //

  // SETEO DE LAS VELOCIDADES MIN Y MAX DE LOS MOTORES x default
    for (int i = 0; i < 4; i++) {
      MOTORS[i].setMaxSpeed(1000);
      MOTORS[i].setAcceleration(100);
    }
  //

  // LCD
    lcd.init();  // Inicializa el LCD
    lcd.backlight();  // Enciende la luz de fondo del LCD
    updateMenu();  // Muestra el menú inicial
  //

}


//FUNCION QUE MUEVE LOS MOTORES HASTA SU DETECCION
  void giroSTR(char replace_direction, char sentido_retraccion) {

    //CICLO FOR PARA LOS 4 MOTORES
      for (int motorIndex = 0; motorIndex < 3; motorIndex++) {
          AccelStepper &num_motor = MOTORS[motorIndex];
          bool firstSwitchDetected = false;
          bool secondSwitchDetected = false;

          // Seleccionamos los valores correspondientes para este motor
          selectValuesMotor(motorIndex, replace_direction, sentido_retraccion);

          // Invertimos los switches del motor 1
          if (motorIndex == 0) {
              // Intercambiar los switches del motor 1
              uint8_t temp = global_first_verify;
              global_first_verify = global_first_retro;
              global_first_retro = temp;
          }

          digitalWrite(ENAPIN1 + (motorIndex * 2), LOW);  // Habilita el motor correspondiente

          //DISPLAY LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Motor ");
          lcd.print(motorIndex + 1);
          lcd.setCursor(0, 1);
          lcd.print("Verificando...");

          // Reiniciamos la posición del motor para contar pasos
          num_motor.setCurrentPosition(0);

          // Fase 1: Mover en la dirección inicial hasta detectar el primer switch
            num_motor.setSpeed(verif_speed); // Ajusta la velocidad de verificación
            unsigned long debounceStart = millis();

            while (!firstSwitchDetected) {
                num_motor.runSpeed();  // Mueve el motor en el primer sentido

                // Verificamos si el switch ha sido presionado (lógica inversa)
                if ((PINA & global_first_verify) != 0) {  // Si el switch es activado
                    if (millis() - debounceStart > 50) {  // Debounce no bloqueante
                        num_motor.stop();  // Detenemos el motor
                        firstSwitchDetected = true;

                        // Retrocedemos una cantidad de pasos
                        num_motor.move(-global_steps_retro);
                        while (num_motor.distanceToGo() != 0) {
                            num_motor.run();  // Retrocedemos
                        }

                        // Cambiamos de dirección para la siguiente verificación
                        lcd.clear();
                        lcd.setCursor(0, 0);
                        lcd.print("Motor ");
                        lcd.print(motorIndex + 1);
                        lcd.setCursor(0, 1);
                        lcd.print("Verif otro sw.");
                    }
                }
            }
            // Reiniciamos la posición antes de mover en el sentido contrario
            num_motor.setCurrentPosition(0);
          //

          // Fase 2: Mover en el sentido contrario hasta detectar el segundo switch
            num_motor.setSpeed(-verif_speed);  // Cambiamos de dirección
            debounceStart = millis();  // Reiniciamos debounce para el segundo switch

            while (!secondSwitchDetected) {
                num_motor.runSpeed();  // Mueve el motor en el sentido contrario

                // Verificamos si el segundo switch ha sido presionado
                if ((PINA & global_first_retro) != 0) {  // Si el segundo switch es activado
                    if (millis() - debounceStart > 50) {  // Debounce no bloqueante
                        num_motor.stop();  // Detenemos el motor
                        secondSwitchDetected = true;

                                            pasosEntreSwitches[motorIndex] = abs(num_motor.currentPosition());

                        // Retrocedemos una pequeña cantidad de pasos
                        num_motor.move(global_steps_retro);
                        while (num_motor.distanceToGo() != 0) {
                            num_motor.run();  // Retrocedemos
                        }

                        // Guardamos la posición final del motor
                        POSICIONES_VYC[motorIndex][flagSTR ? 1 : 0] = num_motor.currentPosition();
                    }
                }
            }
          //

          // Apagar el motor una vez que termine la verificación
          digitalWrite(ENAPIN1 + (motorIndex * 2), HIGH);  // Deshabilita el motor
          verificacionCompleta = true;  // Marcar la verificación como completa
      }
    //

    // Al terminar la verificación de todos los motores, iniciar la visualización de los pasos
      motorMostrandoIndex = 0;
      mostrandoPasos = true;
      mostrarPasosStartTime = millis();
      flagSTR = !flagSTR;  // Alterna entre la primera y segunda verificación
    //
  }
//

// FUNCION PARA MOSTRAR LOS PASOS DE TODOS LOS MOTORES "SET Y VERIFICACION" EN EL LCD -> SIN BLOQUEAR EL MENU
  void mostrarPasosMotores() {
      lcd.clear();
      lcd.setCursor(0, 0);
      for (int i = 0; i < 4; i++) {
          lcd.setCursor(0, 0);
          lcd.print("Motor ");
          lcd.print(i + 1);
          lcd.setCursor(0, 1);
          lcd.print("Pasos: ");
          lcd.print(pasosEntreSwitches[i]);

          // Espera breve no bloqueante para mostrar los pasos de cada motor (controlado con millis)
          unsigned long startTime = millis();
          while (millis() - startTime < 2000) {  // Mostrar cada motor durante 2 segundos
              // Loop vacío para evitar bloqueos del sistema
          }

          lcd.clear();  // Limpiar la pantalla para mostrar el siguiente motor
      }

      verificacionCompleta = false;  // Reiniciar el estado para la próxima verificación
  }
//

//FUNCION PARA LA EJECUTACION DE LA VERIFICACION Y CALIBRACION
void VYC() {     

    stopBuzzer(); //detiene el buzzer una vez ejecutada la operacion
    static unsigned long startTime = 0;

    if (millis() - startTime >= 1000) {  // Muestra el mensaje durante 1 segundo
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Calibrando...");
        startTime = millis();  // Actualiza el tiempo para que el mensaje dure 1 segundo
    }

    //verificacion ycalibracion
    uint8_t switchStates = ~LIMIT_SWITCH_PIN;
    int verif_speed=100;
    String replace_L=String('L');
    String replace_R=String('R');


    char replace_direction = '\0';
    char sentido_retraccion ='\0';

    switch (direccion) {
      case 'L':
        
        replace_direction='L';
        sentido_retraccion='R';
        break;

      case 'R':
        replace_direction='R';
        sentido_retraccion='L';
        break;

       //// LLAVE FINAL SWITCH
      default:
      Serial.println("Direccion invalida");
      return;
  }

  giroSTR(replace_direction, sentido_retraccion);    //EHECUTA PARA REALIZAR LA PRIMERA VERIFICACION
  char temp = replace_direction;                     //INVERTIMOS EL SENTIDO
  replace_direction = sentido_retraccion;
  sentido_retraccion = temp;

}


// FUNCION PARA VERIFICAR ESTADO DEL BUZZER
  void updateLCDForDebugging() {
      if (isBuzzerOn) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Buzzer ON!");
          lcd.setCursor(0, 1);
          lcd.print("Time: ");
          lcd.print(millis() - buzzerStartTime);
      } else {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Buzzer OFF");
      }
  }
//



void loop() {
  //Lectura continua de la posicion del encoder
  long newPosition = myEnc.read();  // Lee la posición del encoder sin dividir



    // Control de inicio de PUNTOTOGO
    if (iniciarPTG && !PTGCompleta) {
        PUNTOTOGO();
    } else if (PTGCompleta && millis() - mostrarMensajeStartTime >= mostrarMensajeDuration) {
        // Volver a mostrar el menú principal u otras acciones después de mostrar el mensaje
        lcd.clear();
        iniciarPTG = false;
        PTGCompleta = false;
    }

    moverMotoresSincronizados(); // Ejecuta el movimiento y controla el mensaje sin bloquear

    if (mensajeCompletoMostrado && millis() - mensajeCompletoStartTime > 2000) { // 2 segundos de pausa
      mensajeCompletoMostrado = false;
      lcd.clear(); // Limpia el mensaje
      // Continuar al menú principal o lo que sea necesario
      updateMenu(); //
  }
    //


  // Ejecucion de la verificacion y calibracion
    if (iniciarVerificacion && !verificacionCompleta) {
        // Llama a la función de verificación de switches si no está completa
        giroSTR('L', 'R');  // Usamos la dirección inicial para la verificación
    } else if (verificacionCompleta) {
        // Si la verificación está completa, mostramos los pasos
        mostrarPasosMotores();

        // Una vez que se muestran los pasos, desactivamos la verificación
        iniciarVerificacion = false;  // Reinicia la bandera para la próxima ejecución
    }
  //

  // Ejecucion de origenes
    if (iniciarIrAlOrigen && !IrAlOrigenCompleta) {
      //LLama a la funcion de "Ir al Origen"
      IrAlOrigen();
    } else if (IrAlOrigenCompleta) {
      iniciarIrAlOrigen = false; //Reiniciamos la bandera
    }

    // Lógica no bloqueante para mostrar el mensaje "Origen Alcanzado" durante 2 segundos
      if (origenAlcanzadoMostrar && millis() - origenAlcanzadoStartTime >= 2000) {
        lcd.clear();  // Limpiar el mensaje después de 2 segundos
        origenAlcanzadoMostrar = false;  // Desactivar la visualización del mensaje
        IrAlOrigenCompleta = true;  // Marcar la función como completada
      }
    //


    // Ejecucion de recalibracion
    if (iniciarRecalibracionAbsoluta && !recalibracionAbsolutaCompleta) {
      //LLama a la funcion de "Ir al Origen"
      RecalibrarAbsoluto();
    } else if (recalibracionAbsolutaCompleta) {
      iniciarRecalibracionAbsoluta = false; //Reiniciamos la bandera
    }

    // Lógica no bloqueante para mostrar el mensaje "Origen Alcanzado" durante 2 segundos
      if (recalibracionAbsolutaMostrar && millis() - recalibracionAbsolutaStartTime >= 2000) {
        lcd.clear();  // Limpiar el mensaje después de 2 segundos
        recalibracionAbsolutaMostrar = false;  // Desactivar la visualización del mensaje
        recalibracionAbsolutaCompleta = true;  // Marcar la función como completada
      }
    //

  //



  // Controlar el zumbido breve del buzzer de forma no bloqueante
    if (isBuzzerOn && millis() - buzzerStartTime >= buzzerDuration) {
        digitalWrite(BUZZER, HIGH);  // Apagar el buzzer después de que haya pasado la duración
        isBuzzerOn = false;
    }
  //

  // Zumbido periódico durante la emergencia

  //

  
  // Detecta cambios en la posición del encoder
  if (newPosition != oldPosition) {

    // DETECCION DE CAMBIOS EN EL DISPLAY LCD DEBIDO A LA INTERACCION DEL ENCODER
     if (abs(newPosition - oldPosition) >= 2) { // Detectar cambios de al menos 2 pasos

      // Si estamos en el MENU "MENU PRINCIPAL"
        if (currentMenuState == MAIN_MENU) {
          menuIndex = (menuIndex + (newPosition > oldPosition ? 1 : -1) + 9) % 9;  // Mueve el índice circularmente
        }
      //

      // Si estamos en el SUBMENU "HOME"
        else if (currentMenuState == SUB_MENU_HOME) {
          homeMenuIndex = (homeMenuIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3;  // Mueve el índice circularmente
        }
      //

      // Si estamos en el SUBMENU "PUNTOTOGO"
        else if (currentMenuState == SUB_MENU_PUNTOTOGO) {
          if (!editing) {
            puntoToGoIndex = (puntoToGoIndex + (newPosition > oldPosition ? 1 : -1) + 5) % 5;  // Mueve el índice circularmente
          } else {
            // Si estamos editando, ajusta el valor de la variable seleccionada
            adjustVariable(newPosition);
          }
        }
      //

      // Si estamos en el SUBMENU "MODELO DE MOVIMIENTO"
        else if (currentMenuState == SUB_MENU_MODELO) {
          modeloMovimientoIndex = (modeloMovimientoIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3;  // Mueve el índice circularmente
        }
      

        // Si estamos en el SUB-SUBMENU de confirmación de cambio de modelo
          else if (currentMenuState == CONFIRM_MODELO) {
            confirmChange = !confirmChange; // Alternar entre SI / NO
          }
        //
      //

      // Si estamos en el SUBMENU "AJUSTE DE PARAMETROS"
        else if (currentMenuState == SUB_MENU_AJUSTE_PARAMETROS) {
          if (!editingParametros) {
            ajusteParametrosIndex = (ajusteParametrosIndex + (newPosition > oldPosition ? 1 : -1) + 5) % 5;  // Mueve el índice circularmente
          } else {
            // Ajusta el valor de la variable seleccionada
            adjustParametros(newPosition);
          }
        }
      //

      // Si estamos en el SUBMENU "SET Y CALIBRACION"
        else if (currentMenuState == SUB_MENU_SETCALIBRACION) {
          setCalibracionIndex = (setCalibracionIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3;  // Mueve el índice circularmente
          }

        // Si estamos en el SUB-SUBMENU "CAMBIAR DIRECCION"
          else if (currentMenuState == SUB_MENU_CAMBIAR_DIRECCION) {
          cambiarDireccionIndex = (cambiarDireccionIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3;  // Mueve el índice circularmente
          }
        //
      //

      // Si estamos en el SUBEMNU "SET ORIGEN"
          else if(currentMenuState == SUB_MENU_SET_ORIGEN){
          setOrigenIndex = (setOrigenIndex + (newPosition > oldPosition ? 1 : -1) + 4) % 4; //Mueve el indice circularmente
          }
      //


      // Si estamos en el SUBMENU "MODO MANUAL"
         else if(currentMenuState == SUB_MENU_MODO_MANUAL){
          modoManualIndex = (modoManualIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3; //Mueve el indice circularmente
          }

          // Si estamos en el SUB-SUBMENU "ESTADO MOTORES"
           else if(currentMenuState == SUB_MENU_ESTADO_MOTORES){
            modoManualIndex = (modoManualIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3; //Mueve el indice circularmente
            }
          //

        // Si estamos en el SUB-SUBMENU "MOVIMIENTO INDIVIDUAL"
            else if(currentMenuState == SUB_MENU_MOV_INDIVIDUAL){
            movIndividualIndex = (movIndividualIndex + (newPosition > oldPosition ? 1 : -1) + 5) % 5; //Mueve el indice circularmente
            }

            // Si estamos en M1
              else if(currentMenuState == SUB_MENU_M1){
                if (!editingM1) {
                motor1Index = (motor1Index + (newPosition > oldPosition ? 1 : -1) + 4) % 4;  // Mueve el índice circularmente
                } else {
                // Si estamos editando, ajusta el valor de la variable seleccionada
                adjustVariableMOTOR(newPosition);
                }
              }
            //

            // Si estamos en M2
              else if(currentMenuState == SUB_MENU_M2){
                if (!editingM2) {
                motor2Index = (motor2Index + (newPosition > oldPosition ? 1 : -1) + 4) % 4;  // Mueve el índice circularmente
                } else {
                // Si estamos editando, ajusta el valor de la variable seleccionada
                adjustVariableMOTOR(newPosition);
                }
              }
            //

            // Si estamos en M3
              else if(currentMenuState == SUB_MENU_M3){
                if (!editingM3) {
                motor3Index = (motor3Index + (newPosition > oldPosition ? 1 : -1) + 4) % 4;  // Mueve el índice circularmente
                } else {
                // Si estamos editando, ajusta el valor de la variable seleccionada
                adjustVariableMOTOR(newPosition);
                }
              }
            //

            // Si estamos en M4
              else if(currentMenuState == SUB_MENU_M4){
                if (!editingM4) {
                motor4Index = (motor1Index + (newPosition > oldPosition ? 1 : -1) + 4) % 4;  // Mueve el índice circularmente
                } else {
                // Si estamos editando, ajusta el valor de la variable seleccionada
                adjustVariableMOTOR(newPosition);
                }
              }
            //

        //

      //
    
      // Si estamos en el SUBEMNU "ESTABLECER RUTA"
        else if(currentMenuState == SUB_MENU_ESTABLECER_RUTA){
          rutaMenuIndex = (rutaMenuIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3; //Mueve el indice circularmente
        }
      //

      // Si estamos en el SUBEMNU "EJECUTAR MOVIMIENTO PARALELO"
        else if(currentMenuState == SUB_MENU_MOV_PARALELO){
          movParaleloIndex = (movParaleloIndex + (newPosition > oldPosition ? 1 : -1) + 3) % 3; //Mueve el indice circularmente
        }
      //


      //ACTUALIZACION CONTINUA Y LECTURA DEL ENCODER Y EL DISPLAY
        oldPosition = newPosition; // Actualiza la posición anterior
        updateMenu();  // Actualiza la pantalla solo cuando hay un cambio
       //

    }
    //

  }
  //

  // Detección del botón del encoder con DELAY --> sin bloquear
    if (!digitalRead(SwitchEncPin) && (millis() - lastDebounceTime) > debounceDelay) {
      lastDebounceTime = millis(); // Actualizar tiempo de debounce
      // Maneja la selección según el estado actual del menú
      if (currentMenuState == MAIN_MENU) {
        handleMainMenuSelection();

      } else if (currentMenuState == SUB_MENU_HOME) {
        handleHomeSelection();

      } else if (currentMenuState == SUB_MENU_PUNTOTOGO) {
        handlePuntoToGoSelection();

      } else if (currentMenuState == SUB_MENU_MODELO) {
        handleModeloSelection();
         } else if (currentMenuState == CONFIRM_MODELO) {
         handleConfirmModeloSelection();

      } else if (currentMenuState == SUB_MENU_SETCALIBRACION) {
        handleSetCalibracionSelection();
        } else if (currentMenuState == SUB_MENU_CAMBIAR_DIRECCION) {
        handleCambiarDireccionSelection();

      } else if (currentMenuState == SUB_MENU_AJUSTE_PARAMETROS) {
        handleAjusteParametrosSelection();

      } else if (currentMenuState == SUB_MENU_MODO_MANUAL) {
        handleModoManualSelection();
         } else if (currentMenuState == SUB_MENU_ESTADO_MOTORES) {
         handleEstadoMotoresSelection();

         } else if (currentMenuState == SUB_MENU_MOV_INDIVIDUAL) {
         handleMovimientoMotoresSelection();

           } else if (currentMenuState == SUB_MENU_M1) {
           handleM1Selection();
           } else if (currentMenuState == SUB_MENU_M2) {
           handleM2Selection();
           } else if (currentMenuState == SUB_MENU_M3) {
           handleM3Selection();
           } else if (currentMenuState == SUB_MENU_M4) {
           handleM4Selection();

      } else if (currentMenuState == SUB_MENU_SET_ORIGEN) {
        handleSetOrigenSelection();

      } else if (currentMenuState == SUB_MENU_MOV_PARALELO) {
        handleMovParaleloSelection();

      } else if (currentMenuState == SUB_MENU_ESTABLECER_RUTA) {
        handleEstablecerRutaSelection();


      }
    }
  //


  // Actualizar el desplazamiento del texto de forma continua
    updateScroll();
  //

  //CONTROLADORES NO BLOQUIANTES
    // Controlar el zumbido breve del buzzer de forma no bloqueante
      if (isBuzzerOn && millis() - buzzerStartTime >= buzzerDuration) {
          digitalWrite(BUZZER, HIGH);  // Apagar el buzzer después de que haya pasado la duración
          isBuzzerOn = false;
      }
    //


    // Controlar la ejecución de PUNTOTOGO de forma no bloqueante
    if (isPuntoToGoRunning && millis() >= puntoToGoEndTime) {
      isPuntoToGoRunning = false;
      currentMenuState = MAIN_MENU; // Vuelve al menú principal después de PUNTOTOGO
      updateMenu();
    }
  //

} //FIN DEL VOID LOOP ---------------------------------------------------------------------------



// Función para ajustar las variables X, Y, Z
void adjustVariable(long newPosition) {

  // Invertir la dirección
    int increment = (newPosition < oldPosition) ? 1 : -1; // Invertimos el sentido de ajuste
  //

  // Calcular la velocidad del giro del encoder
    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - lastEncoderChangeTime;
  //

  // Ajuste de escala de acuerdo a la velocidad del giro
    int scale = 1;
    if (timeDiff < 100) { // Si el tiempo entre cambios es menor a 100 ms
      scale = 5; // Incremento más rápido
    } else if (timeDiff < 200) { // Si es menor a 200 ms
      scale = 3; // Incremento medio
    }
  //

  // Ajusta el valor según la escala
  if (puntoToGoIndex == 0) { // Editando X
    X = constrain(X + (increment * scale), minValXPTG, maxValXPTG);
  } else if (puntoToGoIndex == 1) { // Editando Y
    Y = constrain(Y + (increment * scale), minValYPTG, maxValYPTG);
  } else if (puntoToGoIndex == 2) { // Editando Z
    Z = constrain(Z + (increment * scale), minValZPTG, maxValZPTG);
  }
  //

  oldPosition = newPosition; // Actualiza la posición anterior
  lastEncoderChangeTime = currentTime; // Actualiza el tiempo del cambio
  updateMenu();  // Actualiza la pantalla para mostrar el valor ajustado
} 


// Funcion para ajustar movimiento de motores individualmente
  void adjustVariableMOTOR(long newPosition) {

    // Invertir la dirección
    int increment = (newPosition < oldPosition) ? 1 : -1; // Invertimos el sentido de ajuste
    //

    // Calcular la velocidad del giro del encoder
      unsigned long currentTime = millis();
      unsigned long timeDiff = currentTime - lastEncoderChangeTime;
   //

    // Ajuste de escala de acuerdo a la velocidad del giro
      int scale = 1;
      if (timeDiff < 100) { // Si el tiempo entre cambios es menor a 100 ms
        scale = 5; // Incremento más rápido
      } else if (timeDiff < 200) { // Si es menor a 200 ms
        scale = 3; // Incremento medio
      }
    //

    switch (currentMenuState) {
        case SUB_MENU_M1:
            if (motor1Index == 0) {  // Editar Ángulo para M1
                ang_m1 = constrain(ang_m1 + (increment * scale), minValANG1, maxValANG1);
            } else if (motor1Index == 1) {  // Editar Pasos para M1
                pasos_m1 = constrain(pasos_m1 + (increment * scale), minValPAS1, maxValPAS1);
            }
            break;
        case SUB_MENU_M2:
            if (motor2Index == 0) {  // Editar Ángulo para M1
                ang_m2 = constrain(ang_m2 + (increment * scale), minValANG2, maxValANG2);
            } else if (motor1Index == 1) {  // Editar Pasos para M1
                pasos_m2 = constrain(pasos_m2 + (increment * scale), minValPAS2, maxValPAS2);
            }
            break;
        case SUB_MENU_M3:
            if (motor3Index == 0) {  // Editar Ángulo para M1
                ang_m3 = constrain(ang_m3 + (increment * scale), minValANG3, maxValANG3);
            } else if (motor1Index == 1) {  // Editar Pasos para M1
                pasos_m3 = constrain(pasos_m3 + (increment * scale), minValPAS3, maxValPAS3);
            }
            break;
        case SUB_MENU_M4:
            if (motor4Index == 0) {  // Editar Ángulo para M1
                ang_m4 = constrain(ang_m4 + (increment * scale), minValANG4, maxValANG4);
            } else if (motor1Index == 1) {  // Editar Pasos para M1
                pasos_m4 = constrain(pasos_m4 + (increment * scale), minValPAS4, maxValPAS4);
            }
            break;
    }


    oldPosition = newPosition; // Actualiza la posición anterior
    lastEncoderChangeTime = currentTime; // Actualiza el tiempo del cambio
    updateMenu();  // Actualiza la pantalla para mostrar el valor ajustado
  }

//





// Función para ajustar los parámetros de "Ajuste de Parametros"
void adjustParametros(long newPosition) {

  int increment = (newPosition < oldPosition) ? 1 : -1; // Ajustar la dirección
  
  // Ajustar las variables seleccionadas
    if (ajusteParametrosIndex == 0) { // Editando Velocidad
      velocidad = constrain(velocidad + increment, 0, 100);
    } else if (ajusteParametrosIndex == 1) { // Editando Aceleracion
      aceleracion = constrain(aceleracion + increment, 0, 100);
    } else if (ajusteParametrosIndex == 2) { // Seleccionando Curva de Aceleracion
      curvaAceleracionIndex = (curvaAceleracionIndex + (increment > 0 ? 1 : -1) + 4) % 4; // Cambiar entre 4 tipos
    } else if (ajusteParametrosIndex == 3) { // Seleccionando Inicio Set y Calibracion
      inicioSetCalibracion = (inicioSetCalibracion == "LEFT") ? "RIGHT" : "LEFT"; // Alternar entre LEFT y RIGHT
    }
  //

  oldPosition = newPosition; // Actualiza la posición anterior
  updateMenu();  // Actualiza la pantalla para mostrar el valor ajustado
}

// Función para desplazar texto en el LCD de manera continua sin bloquear
void scrollText(int row, String message, int delayTime = 300) {

  int len = message.length();
  static unsigned long initialPauseTime = 0; // Tiempo de pausa inicial
  static bool initialPauseDone = false; // Estado de la pausa inicial

  // Si el mensaje es menor o igual al ancho de la pantalla, mostrarlo directamente
    if (len <= 16) {
      lcd.setCursor(0, row);
      lcd.print("> " + message); // Mostrar con el símbolo ">"
      isScrolling = false; // No se requiere desplazamiento
      initialPauseDone = false; // Resetear el estado de pausa inicial
      return; // Salir de la función
    }
  //

  // Establecer el texto actual que se está desplazando
  currentScrollingText = message;
  isScrolling = true;

  // Pausa inicial antes de comenzar el desplazamiento
    if (!initialPauseDone) {
      if (millis() - initialPauseTime < 1000) { // Pausa inicial de 1000 ms (1 segundo)
        lcd.setCursor(0, row);
        lcd.print("> " + message.substring(0, 16)); // Mostrar la primera parte del texto
        return; // Salir de la función hasta que la pausa inicial termine
      } else {
        initialPauseDone = true; // Pausa inicial completada
        initialPauseTime = millis(); // Reiniciar el tiempo para el desplazamiento normal
      }
    }
  //

  // Si ha pasado el tiempo de delayTime, actualizar el desplazamiento
  if (millis() - lastScrollTime >= delayTime) {
    lastScrollTime = millis(); // Actualizar el tiempo de la última actualización

    // Si el desplazamiento ha llegado al final del texto, reiniciar
      if (scrollIndex > len - 15) { // Desplazamiento hasta el final menos el ancho de la pantalla
        scrollIndex = 0;
        initialPauseDone = false; // Reiniciar el estado de la pausa inicial para la próxima vez
        initialPauseTime = millis(); // Establecer el tiempo de inicio de la pausa inicial
        return; // Pausa antes de reiniciar
      }
    //

    // Mostrar la parte del mensaje correspondiente
    lcd.setCursor(0, row);
    lcd.print("> " + currentScrollingText.substring(scrollIndex, scrollIndex + 16));
    scrollIndex++; // Incrementar el índice de desplazamiento
  }
}

// Función para actualizar el desplazamiento de texto
void updateScroll() {
  if (isScrolling) {
    scrollText(1, currentScrollingText); // Actualiza el texto desplazándose
  }
}

// Función para actualizar el menú en el display
void updateMenu() {

  lcd.clear();
  lcd.setCursor(0, 0);

  //MENU PRINCIPAL
    if (currentMenuState == MAIN_MENU) {
      lcd.setCursor(0, 0);
      lcd.print("Menu:");
      scrollIndex = 0; // Reiniciar el índice de desplazamiento
      scrollText(1, mainMenu[menuIndex]); // Iniciar desplazamiento si es necesario
  //

  //SUBMENU HOME
    } else if (currentMenuState == SUB_MENU_HOME) {
      scrollText(0, "HOME:");
      scrollText(1, homeMenu[homeMenuIndex]);
  //

  //SUBMENU PUNTO TO GO
    } else if (currentMenuState == SUB_MENU_PUNTOTOGO) {
      scrollText(0, "PuntoToGo:");
      scrollIndex = 0; // Reiniciar el índice de desplazamiento

      if (!editing) {
        scrollText(1, puntoToGoMenu[puntoToGoIndex]); // Iniciar desplazamiento si es necesario
      } else {
        switch (puntoToGoIndex) {
          case 0:
            scrollText(1, "X = " + String(X));
            break;
          case 1:
            scrollText(1, "Y = " + String(Y));
            break;
          case 2:
            scrollText(1, "Z = " + String(Z));
            break;
        }
      }
  //

  // SUBMENU MODELO MOVIMIENTO
      } else if (currentMenuState == SUB_MENU_MODELO) {
        scrollText(0, "Modelo de Mov:");
        scrollIndex = 0; // Reiniciar el índice de desplazamiento
        scrollText(1, modeloMovimientoMenu[modeloMovimientoIndex]); // Iniciar desplazamiento si es necesario


      // SUB-SUBMENU CONFIRMAR MODELO
        } else if (currentMenuState == CONFIRM_MODELO) {
          scrollText(0, "Cambiar a:");
          scrollText(1, confirmChange ? "SI" : "NO"); // Mostrar "SI" o "NO"
      //
  //

  // SUBMENU SET Y CALIBRACION
    } else if (currentMenuState == SUB_MENU_SETCALIBRACION) {
        scrollText(0, "Set y Calib:");
        scrollText(1, setCalibracionMenu[setCalibracionIndex]);
  

    // SUB-SUBMENU CAMBIAR DIRECCION
      } else if (currentMenuState == SUB_MENU_CAMBIAR_DIRECCION) {
          scrollText(0, "Cambiar Direcc:");
          scrollText(1, cambiarDireccionMenu[cambiarDireccionIndex]);
    //
  //

  //SUBMENU AJUSTE DE PARAMETROS
    } else if (currentMenuState == SUB_MENU_AJUSTE_PARAMETROS) {
      scrollText(0, "Ajuste Param:");
      scrollIndex = 0; // Reiniciar el índice de desplazamiento
      if (!editingParametros) {
        scrollText(1, ajusteParametrosMenu[ajusteParametrosIndex]); // Muestra las opciones
      } else {
        // Mostrar los valores editables según la selección
        switch (ajusteParametrosIndex) {
          case 0:
            scrollText(1, "Velocidad: " + String(velocidad));
            break;
          case 1:
            scrollText(1, "Aceleracion: " + String(aceleracion));
            break;
          case 2:
            scrollText(1, "Curva: " + curvasAceleracion[curvaAceleracionIndex]);
            break;
          case 3:
            scrollText(1, "Inicio: " + inicioSetCalibracion);
            break;
        }
      }
    
  //

  //SUBMENU SET ORIGEN
    } else if (currentMenuState == SUB_MENU_SET_ORIGEN) {
      scrollText(0, "SetOrigen:");
      scrollText(1, setOrigenMenu[setOrigenIndex]);
      scrollIndex = 0; // Reiniciar el índice de desplazamiento
  //

  //SUBMENU MODO MANUAL
    } else if (currentMenuState == SUB_MENU_MODO_MANUAL) {
    scrollText(0, "Modo Manual:");
    scrollText(1, modoManualMenu[modoManualIndex]);
    scrollIndex = 0; // Reiniciar el índice de desplazamiento

      // SUB-SUBMENU ESTADO MOTORES
        } else if (currentMenuState == SUB_MENU_MODO_MANUAL) {
        scrollText(0, "Estado Motores:");
        scrollText(1, estadoMotoresMenu[estadoMotoresIndex]);
        scrollIndex = 0; // Reiniciar el índice de desplazamiento
      //


      // SUB-SUBMENU MOVIMIENTO INDIVIDUAL
        } else if (currentMenuState == SUB_MENU_MOV_INDIVIDUAL) {
        scrollText(0, "Mov Ind:");
        scrollText(1, movIndividualMenu[movIndividualIndex]);
        scrollIndex = 0; // Reiniciar el índice de desplazamiento
      //

        // SUB-SUB-SUBMENU MOTOR 1
          } else if (currentMenuState == SUB_MENU_M1) {
          scrollText(0, "M1:");
            if (!editingM1) {
            scrollText(1, motor1Menu[motor1Index]); // Iniciar desplazamiento si es necesario
            } else {
              switch (motor1Index) {
                case 0:
                  scrollText(1, "ANG = " + String(ang_m1));
                  break;
                case 1:
                  scrollText(1, "PAS = " + String(pasos_m1));
                  break;
              }
            }

        //

        // SUB-SUB-SUBMENU MOTOR 2
          } else if (currentMenuState == SUB_MENU_M2) {
          scrollText(0, "M2:");
            if (!editingM2) {
            scrollText(1, motor2Menu[motor2Index]); // Iniciar desplazamiento si es necesario
            } else {
              switch (motor2Index) {
                case 0:
                  scrollText(1, "ANG = " + String(ang_m2));
                  break;
                case 1:
                  scrollText(1, "PAS = " + String(pasos_m2));
                  break;
              }
            }

        // SUB-SUB-SUBMENU MOTOR 3
          } else if (currentMenuState == SUB_MENU_M3) {
          scrollText(0, "M3:");
            if (!editingM3) {
            scrollText(1, motor3Menu[motor3Index]); // Iniciar desplazamiento si es necesario
            } else {
              switch (motor3Index) {
                case 0:
                  scrollText(1, "ANG = " + String(ang_m3));
                  break;
                case 1:
                  scrollText(1, "PAS = " + String(pasos_m3));
                  break;
              }
            }

          // SUB-SUB-SUBMENU MOTOR 4
          } else if (currentMenuState == SUB_MENU_M4) {
          scrollText(0, "M4:");
            if (!editingM4) {
            scrollText(1, motor4Menu[motor4Index]); // Iniciar desplazamiento si es necesario
            } else {
              switch (motor4Index) {
                case 0:
                  scrollText(1, "ANG = " + String(ang_m4));
                  break;
                case 1:
                  scrollText(1, "PAS = " + String(pasos_m4));
                  break;
              }
            }
          }

  //

  //SUBMENU ESTABLECER MOV PARALELO

  //

  //SUBMENU ESTABLECER RUTAS



  //

}


// FUNCIONES HANDLE PARA MANEJO DE LOS MENUS / SUBMENUS / SUB-SUB.........

  // FUNCION MANEJO DEL MENU PRINCIPAL
  void handleMainMenuSelection() {
    playBuzzer();
      // Entra al SUBMENU "HOME"
        if (menuIndex == 0) {
          currentMenuState = SUB_MENU_HOME;
          homeMenuIndex = 0;
          updateMenu();
      //

      //Entra al SUBMENU "PuntoToGo"
        } else if (menuIndex == 1) {
          currentMenuState = SUB_MENU_PUNTOTOGO;
          puntoToGoIndex = 0;
          updateMenu();
      //

      // Entra al SUBMENU "Set y Calibracion"
        } else if (menuIndex == 2) { // Añadido para "Set y Calibracion"
          currentMenuState = SUB_MENU_SETCALIBRACION;
          setCalibracionIndex = 0;
          updateMenu();
      //

      //Entra al SUBMENU "Ajuste de Parametros"
        } else if (menuIndex == 3) {
          currentMenuState = SUB_MENU_AJUSTE_PARAMETROS;
          ajusteParametrosIndex = 0;
          updateMenu();
      //

      //Entra al SUBMENU "Modelo de Movimiento"
        } else if (menuIndex == 4) {
          currentMenuState = SUB_MENU_MODELO;
          modeloMovimientoIndex = 0;
        updateMenu();
      //

      //Entra al SUBMENU "MODO MANUAL"
        } else if (menuIndex == 5) {
          currentMenuState = SUB_MENU_MODO_MANUAL;
          modoManualIndex = 0;
        updateMenu();
      //

      //Entra al SUBMENU "SET ORIGEN"
        } else if (menuIndex == 6) {
          currentMenuState = SUB_MENU_SET_ORIGEN;
          setOrigenIndex = 0;
        updateMenu();
      //

      //Entra al SUBMENU "MOV PARALELO"
        } else if (menuIndex == 7) {
          currentMenuState = SUB_MENU_MOV_PARALELO;
          movParaleloIndex = 0;
        updateMenu();
      //

      //Entra al SUBMENU "ESTABLECER RUTA"
        } else if (menuIndex == 8) {
          currentMenuState = SUB_MENU_ESTABLECER_RUTA;
          rutaMenuIndex = 0;
        updateMenu();
      //

    }
  }



  // Función para manejar la selección en el SUBMENU "HOME"
  void handleHomeSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar
    if (homeMenuIndex == 0) { // SI seleccionado
      HOMMING(); // Llamar a la función HOMMING
      currentMenuState = MAIN_MENU; // Vuelve al menú principal después de HOMMING
    } else if (homeMenuIndex == 2) { // VOLVER ATRAS seleccionado
      currentMenuState = MAIN_MENU;
    }
    updateMenu();
  }

  // Función para manejar la selección en el SUBMENU "PuntoToGo"
  void handlePuntoToGoSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar
    if (puntoToGoIndex == 3) {
      // Si se selecciona "Confirmar", se ejecuta PUNTOTOGO
        iniciarPTG = true;
        PTGCompleta = false;  // Reinicia para comenzar el proceso
    } else if (puntoToGoIndex == 4) {
      // Si se selecciona "Volver Atras", vuelve al menú principal
      currentMenuState = MAIN_MENU;
      updateMenu();
    } else {
      // Entra en modo de edición para la variable seleccionada
      editing = !editing; // Cambia el estado de edición
      updateMenu();
    }
  }

  // Función para manejar la selección en el SUBMENU "Modelo de Movimiento"
  void handleModeloSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar
    if (modeloMovimientoIndex == 2) {
      // Si se selecciona "Volver Atras", vuelve al menú principal
      currentMenuState = MAIN_MENU;
      updateMenu();
    } else {
      // Confirma el cambio de modelo
      currentMenuState = CONFIRM_MODELO;
      confirmChange = false; // Empieza con la opción en "NO"
      updateMenu();
    }
  }

  // Función para manejar la seleccion en el SUB-SUBMENU "Cambio de modelo"
  void handleConfirmModeloSelection() {
    if (confirmChange) {
      // Cambia la variable MODELO
      MODELO = (modeloMovimientoIndex == 0) ? 0 : 1; // 0 = Cinematica Directa, 1 = Cinematica Inversa
    }
    // Regresa al menú "Modelo de Movimiento"
    currentMenuState = SUB_MENU_MODELO;
    updateMenu();
  }

  // Función para manejar la selección en el SUBMENU "Set y Calibracion"
    void handleSetCalibracionSelection() {
        playBuzzer(); // Reproduce sonido al seleccionar
        if (setCalibracionIndex == 0) { // CAMBIAR DIRECCION INICIO
            // Cambiar la dirección de inicio
            currentMenuState = SUB_MENU_CAMBIAR_DIRECCION;
            updateMenu();
        } else if (setCalibracionIndex == 1) { // EJECUTAR
            // Ejecuta la función VyC
            stopBuzzer();
            digitalWrite(BUZZER, HIGH);  // Apagar el buzzer
            iniciarVerificacion = true;
            currentMenuState = MAIN_MENU; // Vuelve al menú principal después de ejecutar
            updateMenu();
        } else if (setCalibracionIndex == 2) { // VOLVER ATRAS
            // Vuelve al menú principal
            currentMenuState = MAIN_MENU;
            updateMenu();
        }
    }


    // Función para manejar la selección en el SUB-SUBMENU "Cambiar Direccion"
      void handleCambiarDireccionSelection() {
        playBuzzer(); // Reproduce sonido al seleccionar
        if (cambiarDireccionIndex == 0) { // LEFT
          direccion = 'L';
        } else if (cambiarDireccionIndex == 1) { // RIGHT
          direccion = 'R';
        } else if (cambiarDireccionIndex == 2) { // VOLVER ATRAS
          // Volver al menú "Set y Calibracion"
          currentMenuState = SUB_MENU_SETCALIBRACION;
          updateMenu();
          return; // Salir de la función para evitar cambiar de menú al seleccionar
        }
        
        // Después de seleccionar una opción (LEFT o RIGHT), volver al menú "Set y Calibracion"
        currentMenuState = SUB_MENU_SETCALIBRACION;
        updateMenu();
      }
    //
  //

  // Función para manejar la selección en el SUBMENU "Ajuste de Parametros"
  void handleAjusteParametrosSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar
    if (ajusteParametrosIndex == 4) {
      // Si se selecciona "Volver Atras", vuelve al menú principal
      currentMenuState = MAIN_MENU;
      updateMenu();
    } else {
      // Entra en modo de edición para la variable seleccionada
      editingParametros = !editingParametros; // Cambia el estado de edición
      updateMenu();
    }
  }

  //Funcion para menjar la seleccion en el SUBMENU "Modo Manual"
    void handleModoManualSelection() {
      playBuzzer(); // Reproduce sonido al seleccionar
      if (modoManualIndex == 0) {
        currentMenuState = SUB_MENU_ESTADO_MOTORES;
        updateMenu();
      } else if (modoManualIndex == 1) {
        currentMenuState = SUB_MENU_MOV_INDIVIDUAL;
        updateMenu();
      } else {
      currentMenuState = MAIN_MENU;
      updateMenu();
      }
    }

    //Funcion para manejar la seleccion en el SUB-SUBMENU "ESTADO MOTORES"
      void handleEstadoMotoresSelection() {
        playBuzzer(); // Reproduce sonido al seleccionar


      }
    //

    //Funcion para manejar la seleccion en el SUB-SUBMENU "MOV MOTORES"
      void handleMovimientoMotoresSelection() {
        playBuzzer(); // Reproduce sonido al seleccionar
      if (movIndividualIndex == 0) {
        currentMenuState = SUB_MENU_M1;
        updateMenu();
      } else if (movIndividualIndex == 1) {
        currentMenuState = SUB_MENU_M2;
        updateMenu();
        } else if (movIndividualIndex == 2) {
        currentMenuState = SUB_MENU_M3;
        updateMenu();
        } else if (movIndividualIndex == 3) {
        currentMenuState = SUB_MENU_M4;
        updateMenu();
      } else {
      currentMenuState = SUB_MENU_MODO_MANUAL;
      updateMenu();
      }

      }


      // Funcion para manejar la seleccion en el SUB-SUB-SUBMENU "Motor 1"
        void handleM1Selection() {
          playBuzzer(); // Reproduce sonido al seleccionar
          if (motor1Index == 2) {
            // Si se selecciona "Confirmar", se ejecuta MOVER_MOTOR_INDIVIDUAL
            MOVER_MOTOR_INDIVIDUAL(0, ang_m1, pasos_m1); // Llama a la función para el motor 1
            editingM1 = false;
          } else if (motor1Index == 3) {
            // Si se selecciona "Volver Atras", vuelve al menú principal
            currentMenuState = SUB_MENU_MOV_INDIVIDUAL;
            updateMenu();
          } else {
            // Entra en modo de edición para la variable seleccionada
            editingM1 = !editingM1; // Cambia el estado de edición
            updateMenu();
          }


        }
      //

      // Funcion para manejar la seleccion en el SUB-SUB-SUBMENU "Motor 2"
        void handleM2Selection() {
          playBuzzer(); // Reproduce sonido al seleccionar
            if (motor2Index == 2) {
            // Si se selecciona "Confirmar", se ejecuta MOVER_MOTOR_INDIVIDUAL
            MOVER_MOTOR_INDIVIDUAL(1, ang_m2, pasos_m2); // Llama a la función para el motor 2
            editingM2 = false;
          } else if (motor2Index == 3) {
            // Si se selecciona "Volver Atras", vuelve al menú principal
            currentMenuState = SUB_MENU_MOV_INDIVIDUAL;
            updateMenu();
          } else {
            // Entra en modo de edición para la variable seleccionada
            editingM2 = !editingM2; // Cambia el estado de edición
            updateMenu();
          }


        }
      //

      // Funcion para manejar la seleccion en el SUB-SUB-SUBMENU "Motor 3"
        void handleM3Selection() {
          playBuzzer(); // Reproduce sonido al seleccionar
          if (motor3Index == 2) {
            // Si se selecciona "Confirmar", se ejecuta MOVER_MOTOR_INDIVIDUAL
            MOVER_MOTOR_INDIVIDUAL(2, ang_m3, pasos_m3); // Llama a la función para el motor 3
            editingM3 = false;
          } else if (motor3Index == 3) {
            // Si se selecciona "Volver Atras", vuelve al menú principal
            currentMenuState = SUB_MENU_MOV_INDIVIDUAL;
            updateMenu();
          } else {
            // Entra en modo de edición para la variable seleccionada
            editingM3 = !editingM3; // Cambia el estado de edición
            updateMenu();
          }

        }
      //

      // Funcion para manejar la seleccion en el SUB-SUB-SUBMENU "Motor 4"
        void handleM4Selection() {
          playBuzzer(); // Reproduce sonido al seleccionar
          if (motor4Index == 2) {
            // Si se selecciona "Confirmar", se ejecuta MOVER_MOTOR_INDIVIDUAL
            MOVER_MOTOR_INDIVIDUAL(3, ang_m4, pasos_m4); // Llama a la función para el motor 4
            editingM4 = false;
          } else if (motor4Index == 3) {
            // Si se selecciona "Volver Atras", vuelve al menú principal
            currentMenuState = SUB_MENU_MOV_INDIVIDUAL;
            updateMenu();
          } else {
            // Entra en modo de edición para la variable seleccionada
            editingM4 = !editingM4; // Cambia el estado de edición
            updateMenu();
          }

        }
      //
    //

  //

  void handleSetOrigenSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar
    if (setOrigenIndex == 0) { 
      //Si se selecciona "Ir al Origen"
      stopBuzzer();
      digitalWrite(BUZZER, HIGH);  // Apagar el buzzer
      iniciarIrAlOrigen = true;
      currentMenuState = MAIN_MENU; // Vuelve al menú principal después de ejecutar
      updateMenu();

    } else if (setOrigenIndex == 3) {
      // Si se selecciona "Volver Atras", vuelve al menú principal
      currentMenuState = MAIN_MENU;
      updateMenu();

      } else if (setOrigenIndex == 2) {
      stopBuzzer();
      digitalWrite(BUZZER, HIGH);  // Apagar el buzzer
      iniciarRecalibracionAbsoluta = true;
      currentMenuState = MAIN_MENU;
      updateMenu();

    } else {
      // Si se selecciona "Establecer nuevo origen"
      iniciarEstablecerOrigen = true; // Cambia el estado de la flag para iniciar
      currentMenuState = MAIN_MENU; // Vuelve al menú principal después de ejecutar
      updateMenu();
    }
  }

  void handleMovParaleloSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar

  }

  void handleEstablecerRutaSelection() {
    playBuzzer(); // Reproduce sonido al seleccionar

  }

//


// FUNCIONES PARA TOCAR/BLOQUEAR UN ZUMBIDO BREVE SIN BLOQUEAR EL CÓDIGO
  unsigned long lastBuzzerTime = 0;
  const unsigned long buzzerCooldown = 500;  // 500 ms de espera antes de permitir otro sonido

  void playBuzzer() {
      if (!isBuzzerOn && millis() - lastBuzzerTime > buzzerCooldown) {
          digitalWrite(BUZZER, LOW);  // Encender el buzzer
          buzzerStartTime = millis();
          isBuzzerOn = true;
          lastBuzzerTime = millis();  // Actualiza el tiempo del último sonido
      }
  }

  void stopBuzzer() {
      if (isBuzzerOn && millis() - buzzerStartTime >= buzzerDuration) {
          digitalWrite(BUZZER, HIGH);  // Apagar el buzzer
          isBuzzerOn = false;
      }
  }
//

// Definición de la función HOMMING
void HOMMING() {
  // Aquí puedes agregar la lógica real para la función HOMMING
  lcd.clear();
  scrollText(0, "HOMMING:");
  scrollText(1, "Ejecutando...");
  delay(2000);  // Simula el tiempo que tarda la ejecución (2 segundos)
}









///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool verificarLimites(float X, float Y, float Z) {
  return (X >= minValXPTG && X <= maxValXPTG) &&
         (Y >= minValYPTG && Y <= maxValYPTG) &&
         (Z >= minValZPTG && Z <= maxValZPTG);
}

bool cinematicaInversa(float X, float Y, float Z, float &q1, float &q2, float &q3) {

  // Calcular q
  q2 = acos((X*X + Y*Y - L1*L1 - L2*L2) / (2 * L1 * L2));
  q1 = atan2(Y, X) - atan2(L2 * sin(q2), L1 + L2 * cos(q2));

  // Calcular q3 basado en Z (DESPLAZAMIENTO VERTICAL)
  q3 = Z - d1;

  return true;
}

// Configuración inicial de movimiento
void PUNTOTOGO() {
  float q1, q2, q3;
  
  if (!cinematicaInversa(X, Y, Z, q1, q2, q3)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Punto Invalido");
        mostrarMensajeStartTime = millis();
        mostrarMensaje = true;
        enMovimiento = false;
        return;
  }

  // Convertir los ángulos a pasos
  long pasosMotor1 = q1 * relacionMOTOR1;
  long pasosMotor2 = q2 * relacionMOTOR2;
  long pasosMotor3 = q3 * relacionMOTOR3;

  // Calcular el máximo de pasos para determinar las velocidades proporcionales
  long maxPasos = max(abs(pasosMotor1), max(abs(pasosMotor2), abs(pasosMotor3)));
  
  // Configurar velocidades para que los motores terminen al mismo tiempo
  MOTORS[0].setMaxSpeed(500.0 * (float(abs(pasosMotor1)) / maxPasos));
  MOTORS[1].setMaxSpeed(500.0 * (float(abs(pasosMotor2)) / maxPasos));
  MOTORS[2].setMaxSpeed(500.0 * (float(abs(pasosMotor3)) / maxPasos));

  // Configurar aceleraciones
  MOTORS[0].setAcceleration(100);
  MOTORS[1].setAcceleration(100);
  MOTORS[2].setAcceleration(100);

  // Establecer los targets para cada motor
  MOTORS[0].moveTo(pasosMotor1);
  MOTORS[1].moveTo(pasosMotor2);
  MOTORS[2].moveTo(pasosMotor3);

  // Inicia el movimiento sincronizado
    enMovimiento = true;
    mostrarMensaje = false;

}

void moverMotoresSincronizados() {
    if (enMovimiento) {
        if (MOTORS[0].distanceToGo() == 0 && MOTORS[1].distanceToGo() == 0 && MOTORS[2].distanceToGo() == 0) {
            enMovimiento = false;
            mostrarMensaje = true;
            mostrarMensajeStartTime = millis();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Punto Alcanzado");
        } else {
            MOTORS[0].run();
            MOTORS[1].run();
            MOTORS[2].run();
        }
    } else if (mostrarMensaje && millis() - mostrarMensajeStartTime >= 2000) {
        mostrarMensaje = false;
        lcd.clear();
        currentMenuState = MAIN_MENU; // Vuelve al menú principal después del mensaje
        updateMenu(); // Vuelve al menú principal después del mensaje
    }
}


// 
void MOVER_MOTOR_INDIVIDUAL(int motorIndex, int ang, int pasos) {
    AccelStepper &motor = MOTORS[motorIndex]; // Selecciona el motor correspondiente

    // Configurar velocidad y aceleración
    motor.setMaxSpeed(400);
    motor.setAcceleration(100);

    // Mover el motor a la posición calculada basada en el ángulo y pasos
    int targetSteps = ang + pasos;
    motor.moveTo(targetSteps);
    digitalWrite(ENAPIN1 + (motorIndex * 2), LOW);

    // Mover el motor de forma no bloqueante
    while (motor.distanceToGo() != 0) {
        motor.run();
    }

    // Detener y deshabilitar el motor una vez que alcanza la posición
    motor.stop();
    digitalWrite(ENAPIN1 + (motorIndex * 2), HIGH);
}
//

void RecalibrarAbsoluto() {
  static int motorIndex = 0;
  static bool procesoCompleto = false;
  static bool motorEnMovimiento = false;
  static long pasosAjusteMotor[3] = {0, -1040, -1625}; // Ajustes individuales
  static int motoresOrden[3] = {1, 2, 0}; // Orden de ejecución: motor 2, motor 3, motor 1
  const int velocidad = 500; // Velocidad común para cada motor
  
  // Desenergizar motor 4 siempre
  digitalWrite(ENAPIN4, HIGH);

  // Si el proceso ya se completó, mostrar mensaje y resetear variables
  if (procesoCompleto) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ajuste Completo");
    recalibracionAbsolutaStartTime = millis(); 
    recalibracionAbsolutaCompleta = true;
    procesoCompleto = false;
    motorIndex = 0; // Reiniciar para futuras ejecuciones
    return;
  }

  // Mostrar mensaje de inicio solo al comenzar
  if (motorIndex == 0 && !motorEnMovimiento) {
    lcd.clear();
    scrollText(0, "Ajustando:");
    scrollText(1, "Posicion Absoluta");
  }

  int motorActual = motoresOrden[motorIndex];
  digitalWrite(ENAPIN1 + motorActual * 2, LOW); // Energizar el motor actual

  // Configurar velocidad y aceleración del motor si no está en movimiento
  if (!motorEnMovimiento) {
    MOTORS[motorActual].setMaxSpeed(velocidad);
    MOTORS[motorActual].setAcceleration(100);
    MOTORS[motorActual].move(pasosAjusteMotor[motorIndex]);
    motorEnMovimiento = true; // Indica que el motor está en movimiento
  }

  // Ejecutar movimiento
  if (MOTORS[motorActual].distanceToGo() != 0) {
    MOTORS[motorActual].run();
  } else {
    // El motor ha llegado al destino, desenergizar y pasar al siguiente
    digitalWrite(ENAPIN1 + motorActual * 2, HIGH); // Desenergizar el motor
    motorIndex++;
    motorEnMovimiento = false; // Preparado para el siguiente motor

    // Si completó todos los motores, marca el proceso como completo
    if (motorIndex >= 3) {
      procesoCompleto = true;
    }
  }
}


void IrAlOrigen() {
  static int motorIndex = 0;
  static unsigned long lastActionTime = 0;
  static bool retrayendo = false; // Indica si el motor está en retracción
  static int pasosRetraccion[3] = {90, 30, 50}; // Pasos de retracción para cada motor
  static int motoresOrden[3] = {1, 2, 0}; // Orden de los motores
  static int switchesOrigen[3] = {LIMMIN2, LIMMIN3, LIMMAX1}; // Pines de switches ajustados
  static bool procesoCompleto = false;
  static int speedsOrigen[3] = {400, 400, 300}; // Velocidades individuales para motor 2, motor 3, motor 1

  // Desenergiza motor 4 siempre
  digitalWrite(ENAPIN4, HIGH);

  // Si el proceso ya terminó, no hace nada
  if (procesoCompleto) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Origen Alcanzado");
    origenAlcanzadoStartTime = millis(); 
    IrAlOrigenCompleta = true; 
    procesoCompleto = false;
    motorIndex = 0; // Reiniciar para futuras ejecuciones
    return;
  }

  // Configuración inicial del motor si es la primera vez
  if (motorIndex == 0 && !retrayendo) {
    lcd.clear();
    scrollText(0, "IR AL ORIGEN:");
    scrollText(1, "Ejecutando...");
  }

  int motorActual = motoresOrden[motorIndex];
  int switchPin = switchesOrigen[motorIndex];

  // Configuración de velocidad y aceleración individual
  MOTORS[motorActual].setMaxSpeed(5000);
  MOTORS[motorActual].setAcceleration(50);  // Aceleración común

  // Comprobar si el switch está presionado
  if (!retrayendo && digitalRead(switchPin) != HIGH) {
    MOTORS[motorActual].setSpeed(speedsOrigen[motorIndex]); // Ajustado para moverse hacia el switch
    MOTORS[motorActual].runSpeed();
  } else if (!retrayendo) {
    // Switch presionado, iniciar retracción
    MOTORS[motorActual].stop();
    MOTORS[motorActual].move(-pasosRetraccion[motorIndex]); // Ajustado para invertir la dirección
    retrayendo = true;
  }

  // Ejecutar la retracción si es el caso
  if (retrayendo && MOTORS[motorActual].distanceToGo() != 0) {
    MOTORS[motorActual].run();
  } else if (retrayendo) {
    // Retracción completada, desactivar motor y pasar al siguiente
    digitalWrite(ENAPIN1 + motorActual * 2, HIGH);
    retrayendo = false;
    motorIndex++;

    // Verificar si todos los motores han terminado
    if (motorIndex >= 3) {
      procesoCompleto = true; // Termina el proceso si es el último motor
    }
  }
}