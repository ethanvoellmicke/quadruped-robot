#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "command_receiver.h"
#include "gait_controller.h"
#include "servo_controller.h"

namespace
{
const char WIFI_NAME[] = "xxxxxxxx";
const char WIFI_PASSWORD[] = "xxxxxxxx";
constexpr unsigned int UDP_PORT = 5005;
constexpr size_t COMMAND_BUFFER_SIZE = 96;
constexpr unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

WiFiUDP udp;
char commandBuffer[COMMAND_BUFFER_SIZE];
unsigned long lastWiFiAttemptMs = 0;
bool udpStarted = false;
bool previousWiFiConnected = false;

void trimInPlace(char *text)
{
  if (text == nullptr) return;

  char *start = text;
  while (*start != '\0' && isspace(static_cast<unsigned char>(*start)))
  {
    ++start;
  }

  if (start != text)
  {
    memmove(text, start, strlen(start) + 1);
  }

  const size_t length = strlen(text);
  if (length == 0) return;

  size_t end = length;
  while (end > 0 && isspace(static_cast<unsigned char>(text[end - 1])))
  {
    --end;
  }
  text[end] = '\0';
}

void lowercaseInPlace(char *text)
{
  for (size_t i = 0; text[i] != '\0'; ++i)
  {
    text[i] = static_cast<char>(tolower(static_cast<unsigned char>(text[i])));
  }
}

const char *modeName(GaitMode mode)
{
  switch (mode)
  {
    case GaitMode::Stand: return "stand";
    case GaitMode::Crawl: return "crawl";
    case GaitMode::Trot: return "trot";
    default: return "unknown";
  }
}

bool parseFloatArgument(const char *command, const char *prefix, float &value)
{
  const size_t prefixLength = strlen(prefix);
  if (strncmp(command, prefix, prefixLength) != 0)
  {
    return false;
  }

  const char *argument = command + prefixLength;
  while (*argument == ' ') ++argument;

  if (*argument == '\0')
  {
    return false;
  }

  char *endPointer = nullptr;
  const float parsed = strtof(argument, &endPointer);

  if (endPointer == argument)
  {
    return false;
  }

  while (*endPointer != '\0' && isspace(static_cast<unsigned char>(*endPointer)))
  {
    ++endPointer;
  }

  if (*endPointer != '\0')
  {
    return false;
  }

  value = parsed;
  return true;
}

void processCommand(char *command)
{
  trimInPlace(command);
  lowercaseInPlace(command);

  if (command[0] == '\0') return;

  if (strcmp(command, "stand") == 0 || strcmp(command, "stop") == 0)
  {
    setGaitMode(GaitMode::Stand);
    Serial.println("Command: stand");
    return;
  }

  if (strcmp(command, "crawl") == 0 || strcmp(command, "walk") == 0)
  {
    setGaitMode(GaitMode::Crawl);
    Serial.println("Command: crawl");
    return;
  }

  if (strcmp(command, "trot") == 0)
  {
    setGaitMode(GaitMode::Trot);
    Serial.println("Command: trot");
    return;
  }

  if (strcmp(command, "status") == 0)
  {
    printRobotStatus();
    return;
  }

  float value = 0.0f;

  if (parseFloatArgument(command, "speed", value))
  {
    setGaitSpeed(value);
    Serial.print("Speed set to ");
    Serial.println(getGaitSpeed(), 3);
    return;
  }

  if (parseFloatArgument(command, "height", value))
  {
    setGroundHeight(value);
    Serial.print("Height set to ");
    Serial.println(getGroundHeight(), 3);
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(command);
}

void connectWiFiNonBlocking()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  const unsigned long now = millis();
  if (now - lastWiFiAttemptMs < WIFI_RETRY_INTERVAL_MS)
  {
    return;
  }

  lastWiFiAttemptMs = now;
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_NAME);

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
}

void readSerialCommands()
{
  static size_t length = 0;

  while (Serial.available() > 0)
  {
    const char character = static_cast<char>(Serial.read());

    if (character == '\r') continue;

    if (character == '\n')
    {
      commandBuffer[length] = '\0';
      processCommand(commandBuffer);
      length = 0;
      continue;
    }

    if (length + 1 < COMMAND_BUFFER_SIZE)
    {
      commandBuffer[length++] = character;
    }
    else
    {
      length = 0;
      Serial.println("Serial command too long; discarded");
    }
  }
}

void readUdpCommands()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  while (true)
  {
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;

    const int bytesToRead =
      packetSize < static_cast<int>(COMMAND_BUFFER_SIZE - 1)
      ? packetSize
      : static_cast<int>(COMMAND_BUFFER_SIZE - 1);

    const int bytesRead = udp.read(
      reinterpret_cast<uint8_t *>(commandBuffer),
      bytesToRead
    );

    while (udp.available() > 0) udp.read();

    if (bytesRead > 0)
    {
      commandBuffer[bytesRead] = '\0';
      processCommand(commandBuffer);
    }
  }
}
}

void initializeCommandReceiver()
{
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Wi-Fi module not detected; serial commands still work");
    return;
  }

  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000UL)
  {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    udp.begin(UDP_PORT);
    udpStarted = true;
    previousWiFiConnected = true;
    Serial.print("Wi-Fi connected. Nano IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Listening for text commands on UDP port ");
    Serial.println(UDP_PORT);
  }
  else
  {
    Serial.println("Wi-Fi not connected yet; gait and serial control remain active");
    lastWiFiAttemptMs = millis();
  }
}

void updateCommandReceiver()
{
  readSerialCommands();

  connectWiFiNonBlocking();

  const bool isConnected = WiFi.status() == WL_CONNECTED;

  if (isConnected && !udpStarted)
  {
    udp.begin(UDP_PORT);
    udpStarted = true;
    Serial.print("UDP listener active. IP: ");
    Serial.println(WiFi.localIP());
  }
  else if (!isConnected && previousWiFiConnected)
  {
    udp.stop();
    udpStarted = false;
    Serial.println("Wi-Fi disconnected; local gait continues");
  }

  previousWiFiConnected = isConnected;
  readUdpCommands();
}

void printRobotStatus()
{
  Serial.print("mode=");
  Serial.print(modeName(getGaitMode()));
  Serial.print(" speed=");
  Serial.print(getGaitSpeed(), 3);
  Serial.print(" height=");
  Serial.print(getGroundHeight(), 3);
  Serial.print(" wifi=");
  Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
}
