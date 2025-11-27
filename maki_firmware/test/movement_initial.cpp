
#include <Arduino.h>
#include "maki_controller.hpp"

#define DirectionPin 10
#define BaudRate 1000000
#define ID1 1
#define ID2 2

void setup() {
    Serial.begin(115200);
    Serial.println("Maki Gripper Test");
    delay(2000);    

    // Inicia comunicación 
    maki_controller.begin(BaudRate, DirectionPin, &Serial);

    // Modo sin fin
    maki_controller.setEndless(ID1, ON);
    maki_controller.setEndless(ID2, ON);
}

void loop() {

    int pos = maki_controller.readPosition(ID1);

    Serial.println("");
    Serial.print("Pos: ");
    Serial.println(pos);

    // Motor 1 izquierda rápida
    maki_controller.ledStatus(ID1, ON);
    maki_controller.turn(ID1, LEFT, 200);
    delay(1000);
    maki_controller.ledStatus(ID1, OFF);

    // Motor 2 izquierda lenta
    maki_controller.ledStatus(ID2, ON);
    maki_controller.turn(ID2, LEFT, 300);
    delay(1000);
    maki_controller.ledStatus(ID2, OFF);

    // Motor 1 derecha rápida
    maki_controller.ledStatus(ID1, ON);
    maki_controller.turn(ID1, RIGHT, 600);
    delay(2000);
    maki_controller.ledStatus(ID1, OFF);

    // Motor 2 derecha lenta
    maki_controller.ledStatus(ID2, ON);
    maki_controller.turn(ID2, RIGHT, 300);
    delay(2000);
    maki_controller.ledStatus(ID2, OFF);
    
}
