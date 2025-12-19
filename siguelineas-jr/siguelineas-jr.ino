#include <Arduino.h>
#include <LittleFS.h>
#include <QTRSensors.h>
#include "BluetoothSerial.h"
#include "Motors.h"

// Constantes para el bluetooth
const char* name = "LineFollowerJR";

// Led indicador del ESP32
const int LED_BUILTIN = 2;

// Constantes para el control del brillo del led indicador
const int LED_BT_CLIENT_CONNECTED_DELAY = 9;
const int LED_BT_INITIALIZED_DELAY = 1;

// Constantes para el control de la medición de los sensores
const int MEASURE_DELAY = 250;

// Constantes para las entradas y salidas
const uint8_t INPUT_1 = 15;
const uint8_t INPUT_2 = 16;
const uint8_t INPUT_3 = 17;
const uint8_t INPUT_4 = 18;
const uint8_t INPUT_5 = 19;

// Constantes para las direcciones de los archivos de configuración
const char* KP_PATH = "/kp.txt";
const char* KI_PATH = "/ki.txt";
const char* KD_PATH = "/kd.txt";
const char* SPEED_PATH = "/speed.txt";

// Variables para el control de los motores
int left_motor_speed, right_motor_speed;
int speed = 0;

// Variables para recibir los datos convertidos
float kp = 0.0, ki = 0.0, kd = 0.0;

// Constante para el SETPOINT del PID
const int SETPOINT = 2000;

// Variables para el algoritmo del PID
int error, last_input, integral, derivative;

// // Variables para los parámetros del PID
String strKp;
String strKi;
String strKd;
String strSpeed;

// Variables para el control del led indicador
unsigned long previous_led_millis = 0;
int led_brightness = 0;
int brightness_step = 1;

//variables para el control de la medida de los sensores
unsigned long previous_measure_millis = 0;

// Variable for bt status
int bt_status = 0;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

bool newData = false;

const uint8_t sensorCount = 5;
uint16_t sensorValues[sensorCount];

QTRSensors qtr;

BluetoothSerial SerialBT;

Motors motors;

void initLittleFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("Ocurrió un error al iniciar el sistema de archivos");
  }

  Serial.println("Sistema de archivos cargado");
}

String readFile(const char* path) {
  Serial.printf("Leyendo archivo: %s\r\n", path);

  File file = LittleFS.open(path);

  if (!file || file.isDirectory()) {
    Serial.println("Error al abrir el archivo");
    return String();
  }

  String fileContent;

  while (file.available()) {
    fileContent = file.readStringUntil('\n');
    break;
  }

  return fileContent;
}

void writeFile(const char* path, const char* data) {
  Serial.printf("Escribiendo archivo: %s\r\n", path);

  File file = LittleFS.open(path, FILE_WRITE);

  if (!file) {
    Serial.println("Error al abrir el archivo");
    return;
  }

  if (file.print(data)) {
    Serial.println("Archivo actualizado");
  } else {
    Serial.println("Error al actualizar el archivo");
  }
}

void initBluetooth() {
  Serial.println("Iniciando Bluetooth");
  SerialBT.begin(name);                         // Initialize bt with the name of the device
  SerialBT.register_callback(BT_EventHandler);  // Register callback function for BT events
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (SerialBT.available() > 0 && newData == false) {
    rc = SerialBT.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");  // get the first part - the string
  kp = atof(strtokIndx);                // convert this part to an integer

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  ki = atof(strtokIndx);           // convert this part to an integer

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  kd = atof(strtokIndx);           // convert this part to an integer

  strtokIndx = strtok(NULL, ",");
  speed = atoi(strtokIndx);  // convert this part to a float
}

void showParsedData() {
  Serial.print("kp: ");
  Serial.print(kp, 3);
  Serial.print(", ki: ");
  Serial.print(ki, 3);
  Serial.print(", kd: ");
  Serial.print(kd, 3);
  Serial.print(", vel: ");
  Serial.println(speed);
}

// Bluetooth Event Handler CallBack Function Definition
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_START_EVT) {
    bt_status = event;
    Serial.println("Bluetooth inicializado");
  } else if (event == ESP_SPP_SRV_OPEN_EVT) {
    bt_status = event;
    Serial.println("Cliente conectado");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    bt_status = event;
    Serial.println("Cliente desconectado");
  } else if (event == ESP_SPP_DATA_IND_EVT) {
    Serial.println("Datos recibidos");
    recvWithStartEndMarkers();

    if (newData == true) {
      // Se tiene que enviar una cadena con el siguiente formato "<kp, ki, kd, vel>"
      strcpy(tempChars, receivedChars);
      // this temporary copy is necessary to protect the original data
      //   because strtok() used in parseData() replaces the commas with \0
      parseData();
      showParsedData();
      newData = false;

      char str[32];
      dtostrf(kp, 8, 3, str);
      writeFile(KP_PATH, str);

      dtostrf(ki, 8, 3, str);
      writeFile(KI_PATH, str);

      dtostrf(kd, 8, 3, str);
      writeFile(KD_PATH, str);

      sprintf(str, "%d", speed);
      writeFile(SPEED_PATH, str);
    }
  }
}

void setup() {
  pinMode(INPUT_1, INPUT);
  pinMode(INPUT_2, INPUT);
  pinMode(INPUT_3, INPUT);
  pinMode(INPUT_4, INPUT);
  pinMode(INPUT_5, INPUT);

  Serial.begin(115200);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ INPUT_1, INPUT_2, INPUT_3, INPUT_4, INPUT_5 }, sensorCount);

  delay(50);

  initBluetooth();

  initLittleFS();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  strKp = readFile(KP_PATH);
  strKi = readFile(KI_PATH);
  strKd = readFile(KD_PATH);
  strSpeed = readFile(SPEED_PATH);

  kp = strKp.toFloat();
  ki = strKi.toFloat();
  kd = strKd.toFloat();
  speed = strSpeed.toInt();

  Serial.print("kp: ");
  Serial.print(kp, 3);
  Serial.print(", ki: ");
  Serial.print(ki, 3);
  Serial.print(", kd: ");
  Serial.print(kd, 3);
  Serial.print(", vel: ");
  Serial.println(speed);

  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < sensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);

  // motors.motors(100, 100);
}

void loop() {
  if (millis() - previous_measure_millis >= MEASURE_DELAY) {
    previous_measure_millis = millis();
    uint16_t entrada = qtr.readLineBlack(sensorValues);

    error = entrada - SETPOINT;

    derivative = entrada - last_input;

    integral += error;
    integral = constrain(integral, -1000, 1000);

    int diff = (kp * error) + (ki * integral) + (kd * derivative);

    last_input = entrada;

    diff = constrain(diff, -255, 255);

    Serial.print(kp);
    Serial.print(", ");
    Serial.print(ki);
    Serial.print(", ");
    Serial.print(kd);
    Serial.print(", ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.print(error);
    Serial.print(", ");
    Serial.println(diff);
    // print the sensor values as numbers from 0 to 1000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // entrada
    // for (uint8_t i = 0; i < sensorCount; i++) {
    //   Serial.print(sensorValues[i]);
    //   Serial.print('\t');
    // }
    // Serial.println(entrada);
  }

  if (millis() - previous_led_millis >= LED_BT_CLIENT_CONNECTED_DELAY && bt_status == ESP_SPP_SRV_OPEN_EVT) {
    previous_led_millis = millis();

    led_brightness += brightness_step;
    if (led_brightness == 0 || led_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, led_brightness);
  }

  if (millis() - previous_led_millis >= LED_BT_INITIALIZED_DELAY && (bt_status == ESP_SPP_START_EVT || bt_status == ESP_SPP_CLOSE_EVT)) {
    previous_led_millis = millis();

    led_brightness += brightness_step;
    if (led_brightness == 0 || led_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, led_brightness);
  }
}
