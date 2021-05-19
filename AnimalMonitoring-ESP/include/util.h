#ifndef UTIL_H
#define UTIL_H

#define SERIAL_DEBUG

template <typename T>
void SerialDebugger(T message)
{
    #ifdef SERIAL_DEBUG
        Serial.println(message);
    #endif
}

template <typename T1, typename T2>
void SerialDebugger(T1 message1, T2 message2)
{
    #ifdef SERIAL_DEBUG
        Serial.print(message1);
        Serial.println(message2);
    #endif
}


template <typename T1, typename T2, typename T3>
void SerialDebugger(T1 message1, T2 message2, T3 message3)
{
    #ifdef SERIAL_DEBUG
        Serial.print(message1);
        Serial.print(message2);
        Serial.println(message3);
    #endif
}

template <typename T1, typename T2, typename T3, typename T4>
void SerialDebugger(T1 message1, T2 message2, T3 message3, T4 message4)
{
    #ifdef SERIAL_DEBUG
        Serial.print(message1);
        Serial.print(message2);
        Serial.print(message3);
        Serial.println(message4);
    #endif
}

template <typename T1, typename T2, typename T3, typename T4, typename T5>
void SerialDebugger(T1 message1, T2 message2, T3 message3, T4 message4, T5 message5)
{
    #ifdef SERIAL_DEBUG
        Serial.print(message1);
        Serial.print(message2);
        Serial.print(message3);
        Serial.print(message4);
        Serial.println(message5);
    #endif
}
#endif