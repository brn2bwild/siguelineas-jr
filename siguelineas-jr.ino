#include <Arduino.h>
#include <LittleFS.h>
#include "BluetoothSerial.h"

// Constantes para el bluetooth
const char* name = "LineFollowerJR";

// Led indicador del ESP32
const int LED_BUILTIN = 2;

// Constantes para el control del brillo del led indicador
const int LED_BT_CLIENT_CONNECTED_DELAY = 9;
const int LED_BT_INITIALIZED_DELAY = 1;

// Constantes para las entradas y salidas
const uint8_t LED_PIN = 2;

// Constantes para las direcciones de los archivos de configuración
const char* KP_PATH = "/kp.txt";
const char* KI_PATH = "/ki.txt";
const char* KD_PATH = "/kd.txt";
const char* SPEED_PATH = "/speed.txt";

// Variables para el control de los motores
int left_motor_speed, right_motor_speed;

// variables to hold the parsed data
float kp = 0.0, ki = 0.0, kd = 0.0;
int speed = 0;

// // Variables para los parámetros del PID
String strKp;
String strKi;
String strKd;
String strSpeed;

// Variables para el control del led indicador
unsigned long previous_led_millis = 0;
int leg_brightness = 0;
int brightness_step = 1;

// Variable for bt status
int bt_status = 0;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

bool newData = false;

BluetoothSerial SerialBT;

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
  Serial.begin(115200);

  initBluetooth();

  initLittleFS();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

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
}

void loop() {
  if (millis() - previous_led_millis >= LED_BT_CLIENT_CONNECTED_DELAY && bt_status == ESP_SPP_SRV_OPEN_EVT) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }

  if (millis() - previous_led_millis >= LED_BT_INITIALIZED_DELAY && (bt_status == ESP_SPP_START_EVT || bt_status == ESP_SPP_CLOSE_EVT)) {
    previous_led_millis = millis();

    leg_brightness += brightness_step;
    if (leg_brightness == 0 || leg_brightness == 255) {
      brightness_step = -brightness_step;
    }

    analogWrite(LED_BUILTIN, leg_brightness);
  }
}
