#include "WifiPAMIS.h"
#include <WiFiWebServer_RTL8720.h>
#include <RTL8720HttpClient.h>
#include <string>
#include <vector>

char ssid[] = "Club Tech";    // your network SSID (name)
char pass[] = "V~kmM5U145fH";       // your network password

char serverAddress[] = "192.168.0.101";  // server address
int port = 8080;

IPAddress ip(192, 168, 0, 130);  // Adresse IP statique
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient      client;
WiFiHttpClient  httpClient(client, serverAddress, port);

int status = WL_IDLE_STATUS;     // the Wifi radio's status

Message rxMsg[SIZE_FIFO];
int FIFO_ecriture = 0;

void Wifi_task(void *pvParameters){
    (void) pvParameters;

    while(1){
      WifiLoop();
    }
}

void init_Wifi_task(){
    xTaskCreate(Wifi_task, "Wifi_task", 2048, NULL, tskIDLE_PRIORITY+2, NULL);
    
} 

void setup_Wifi(){
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();

  Serial.print("Current Firmware Version = ");
  Serial.println(fv);

  if (fv != LATEST_RTL8720_FIRMWARE)
  {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);

    // Connect to WPA/WPA2 network. 2.4G and 5G are all OK
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(1000);
  }
  Serial.println("WiFi.begin(ssid, pass);                 ended");
  // Configurer l'adresse IP statique
  WiFi.config(ip, gateway, subnet);
  Serial.println("WiFi.config(ip, gateway, subnet);       ended");
  Serial.println();
  // you're connected now, so print out the data
  printWifiStatus();

  // init_Wifi_task(); Serial.println("init_Wifi_task;                 ended");
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to:
  // you're connected now, so print out the data
  Serial.print(F("You're connected to the network, IP = "));
  Serial.println(WiFi.localIP());

  Serial.print(F("SSID: "));
  Serial.print(WiFi.SSID());

  // print the received signal strength:
  int32_t rssi = WiFi.RSSI();
  Serial.print(F(", Signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}

void WifiLoop() {
    static uint32_t waitingTime = 500, pastTimeMs = 0;

    if((millis()- pastTimeMs) >= waitingTime){
    // Serial.println("making GET request");
    if (httpClient.get("/pamis")) {
        // read the status code and body of the response
        int statusCode = httpClient.responseStatusCode();
        String response = httpClient.responseBody();

        Serial.print("Status code: ");
        Serial.println(statusCode);

       remplirMessageStruct(response);

       // Close the connection to free up the socket
      httpClient.stop();
    } /*else {
        // Serial.println("HTTP request failed");
    }//*/
    pastTimeMs = millis();
    }
}

// Custom function to get lines from a String
void getLines(const String& input, std::vector<String>& lines) {
    String currentLine;

    for (unsigned int i = 0; i < input.length(); i++) {
        char c = input.charAt(i);
        if (c == '\n') {
            lines.push_back(currentLine);
            currentLine = "";
        } else {
            currentLine += c;
        }
    }

    // Add the last line if it's not empty
    if (currentLine.length() > 0) {
        lines.push_back(currentLine);
    }
}

void extractValuesFromString(const String& input, char separator, int &nb_msg, int &robot_id, int &order, int &arg1, int &arg2, int &arg3) {
  int values[6] = {0}; // Array to store extracted values
  int currentIndex = 0; // Index for the values array

  int startIndex = 0; // Start index for each value
  int length = input.length();

  for (int i = 0; i < length; i++) {
    if (input[i] == separator || i == length - 1) {
      // Extract substring between startIndex and i (or i + 1 for the last value)
      String valueStr = input.substring(startIndex, (i == length - 1) ? i + 1 : i);
      
      // Convert the substring to an integer and store it in the values array
      values[currentIndex++] = valueStr.toInt();

      // Update the startIndex for the next value
      startIndex = i + 1;
    }
  }

  // Assign values to variables
  nb_msg = values[0];
  robot_id = values[1];
  order = values[2];
  arg1 = values[3];
  arg2 = values[4];
  arg3 = values[5];
}

void remplirMessageStruct(const String& response)
{
  static int nb_msg = 0, robot_id = 0, order = 0, arg1 = 0, arg2 = 0, arg3 = 0;
  static int past_nb_liste_cmd = 0;

  // Use the custom function to get lines
  std::vector<String> lines;
  getLines(response, lines);

  // Iterate through lines
  int nb_lignes = 0, past_nb_msg = 0;
  for (String& ligne : lines) {// On lit une ligne complète             
    // Serial.println("*");
    // Serial.println(ligne.c_str());
    past_nb_msg = nb_msg;
    robot_id = 0; order = 0; arg1 = 0; arg2 = 0; arg3 = 0; // Initialize variables before parsing

    if(nb_lignes == 1){
      //On sait maintenant que c'est une liste de commande
      Serial.println("liste de commande !");
      if(past_nb_liste_cmd == nb_msg){
        //Alors cette liste de commande à deja été traiter
        Serial.println("cette liste de commande à deja été traiter");
        FIFO_ecriture = (FIFO_ecriture!=0)?(FIFO_ecriture-1):SIZE_FIFO;//On supprime la premiere instruction inutile du buffer
        return;
      }
      else{
        past_nb_liste_cmd = nb_msg;
      }
    }

    // int errorCode = sscanf(ligne.c_str(), "%hd,%hd,%hd,%hd,%hd,%hd",
    //                        &nb_msg,
    //                        &robot_id,
    //                        &order,
    //                        &arg1,
    //                        &arg2,
    //                        &arg3);
    // Serial.print("errorCode sscanf : "); Serial.println(errorCode);
    extractValuesFromString(ligne, ',', nb_msg, robot_id, order, arg1, arg2, arg3);

    if (past_nb_msg != nb_msg)
    { // Nouveau msg      
      Serial.println ("*************************************************");
      Serial.print("Commande n°");
      Serial.print(nb_msg);   Serial.print(" : id ");
      Serial.print(robot_id); Serial.print(", order ");
      Serial.print(order);    Serial.print(", arg1 ");
      Serial.print(arg1);     Serial.print(", arg2 ");
      Serial.print(arg2);     Serial.print(", arg3 ");
      Serial.println(arg3);
      Serial.println ("*************************************************");
      rxMsg[FIFO_ecriture] = (Message){nb_msg, robot_id, order, arg1, arg2, arg3};
      FIFO_ecriture = (FIFO_ecriture + 1) % SIZE_FIFO;
    }
    nb_lignes ++;
  }

}
