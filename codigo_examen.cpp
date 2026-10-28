/*Imaginemos un crucero, con dos semaforos para autos y uno para peatones.
    Tres semaforos V0, V1, V2, dos funcionan igual (V0 y V1), estos tienen dos leds uno verde y uno rojo, 
    el otro solo un azul. Hay 6 estados posibles:
        S0: V0=Verde, V1=Rojo, V2=Apagado
        S1: V0=Parpadeando, V1=Rojo, V2=Apagado
        S2: V0=Rojo, V1=Verde, V2=Apagado
        S3: V0=Rojo, V1=Parpadeando, V2=Apagado
        S4: V0=Rojo, V1=Rojo, V2=Apagado
        S5: V0=Rojo, V1=Rojo, V2=Encendido
    Este ultimo estado depende de un switch, al llegar a S4 se comprueba el estado del switch,
    si esta encendido ocurre S5 y si esta apagado regresa a S0.
    Esto se debe guardar en la memoria flash y a pesar de agregar estados en codigo debe funcionar igual.
    La velocidad de cambio esta dada por un dipswich de 8 bits, el cual se enmascarara y utilizaremos solo 2 de ellos.
    Tabla de estados:
        00 = Sistema Apagado
        01 = 1s "Normal"
        10 = 2s "Baja concurrencia"
        11 = 500ms "Hora pico"
    Esta velocidad se divide entre cambios de color no entre cambios de estado.
    Ejemplo: 01
    Entre S0 y S2 pasa 1s, entre S2 y S4 pasa 1s.
    Ese segundo se divide entre S0 y S1 con un respectivo velocidad/2.
    La tabla de estados así como las respectivas tablas de salidas deben estar almacenadas en la memoria Flash del micro
    controlador.
    Tabla de transicion:
        S0 -> S1
        S1 -> S2
        S2 -> S3
        S3 -> S4
        S4 -> S5
        S5 -> S0
    Tabla de salidas:
        S0: V0=Verde, V1=Rojo, V2=Apagado
        S1: V0=Parpadeando, V1=Rojo, V2=Apagado
        S2: V0=Rojo, V1=Verde, V2=Apagado
        S3: V0=Rojo, V1=Parpadeando, V2=Apagado
        S4: V0=Rojo, V1=Rojo, V2=Apagado
        S5: V0=Rojo, V1=Rojo, V2=Encendido
    Se requiere consultar las tablas almacenadas en 
    Si se requiriera modificar el comportamiento del semáforo, debe depender únicamente de los valores de esas tablas. Las
    respectivas salidas del semáforo como del cruce peatonal deben ser LEDs. La solicitud de cruce debe realizarse a través
    del DIP switch.
    El modo de los semáforos puede ser seleccionado a través de DIP switch, así como las velocidades almacenadas en la
    Flash.
    El parpadeo de la luz de atención debe durar en total (incluyendo encendidos y apagados) lo especificado por el modo
    de funcionamiento.
*/

/*Implementacion personal:
    Usando un dip swich con entradas PB1-PB5 y PC0-PC2; utilizando como salidas PD0-PD4.
    Del dipswich usaremos el pin PC2 para el cruce peatonal, y los pines PB1 y PB2 para las velocidades.
    Leds: El pin PD4 es el paso al peaton, PD3 y PD1 son avance, PD2 y PD0 son detenerse.
    Tabla de transicion actualizado:
    |Donde esta   |Destino      |Destino        |
    |Estado actual|Swich apagado|Swich encendido|
        S0 -> S1 ó S1
        S1 -> S2 ó S2
        S2 -> S3 ó S3
        S3 -> S4 ó S4
        S4 -> S0 ó S5
        S5 -> S0 ó S0
*/
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>

#define DIP_VELOCIDAD_0 PB1
#define DIP_VELOCIDAD_1 PB2
#define DIP_CRUCE PC2
 
#define LED_ROJO_V0 PD2
#define LED_AVANCE_V1 PD1
#define LED_ROJO_V1 PD0
#define LED_AVANCE_V0 PD3
#define LED_PEATON PD4

#define LEDS_MASK 0x1F
#define FASES_PARPADEO 16

enum estados { S0, S1, S2, S3, S4, S5, NUM_ESTADOS };
enum leds    { APAGADO, ROJO, VERDE, PARPADEANDO, AZUL };

/*La tabla de cambios:
    La primer columna es el estado en el que estas.
    La segunda el estado a donde iras si el swich esta apagado.
    La tercera a donde iras si el swich esta encendido.*/
const uint8_t tablaCambios[NUM_ESTADOS][3] PROGMEM = {
    {S0, S1, S1},
    {S1, S2, S2},
    {S2, S3, S3},
    {S3, S4, S4},
    {S4, S0, S5},
    {S5, S0, S0}
};

/* Columna 0: V0 | Columna 1: V1 | Columna 2: V2 (peatón) */
const uint8_t tablaSalidas[NUM_ESTADOS][3] PROGMEM = {
    {VERDE, ROJO, APAGADO},
    {PARPADEANDO, ROJO, APAGADO},
    {ROJO, VERDE, APAGADO},
    {ROJO, PARPADEANDO, APAGADO},
    {ROJO, ROJO, APAGADO},
    {ROJO, ROJO, AZUL}
};

/*Tabla de leds:
Configura que led corresponde a cada bit, nos ayuda con las mascaras*/
const uint8_t tablaLeds[3][2] PROGMEM = {
    {(1 << LED_ROJO_V0), (1 << LED_AVANCE_V0)},
    {(1 << LED_ROJO_V1), (1 << LED_AVANCE_V1)},
    {0,(1 << LED_PEATON)}
};

/* Duracion en microsegundos de un cambio de color completo, dada por el valor
   del dipswich PB2 y PB1 
   00 apagado, 
   01 (1s) normal, 
   10 (2s) baja, 11 
   la (500ms) la hora pico */
const uint16_t tablaVelocidades[4] PROGMEM = {0, 1000, 2000, 500};

/Una variable de bandera que nos dice cuando se solicito paso/
volatile uint8_t banderaPeatonal = 0;

/Divide los retardos para estar checando pc2 (la bandera de arriba)/
static void retardoChecando(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        if (!(PINC & (1 << DIP_CRUCE))) banderaPeatonal = 1;
        _delay_ms(1);
    }
}

static uint8_t leerVelocidad(void) {
    return (uint8_t)((~PINB >> DIP_VELOCIDAD_0) & 0x03); // 0x03 = 00000011
}

/* Lee la tabla de salidas y arma el patron del puerto D y la mascara de parpadeo */
static void armarSalida(uint8_t estado, uint8_t *patron, uint8_t *blink) {
    uint8_t p = 0, b = 0;

    for (uint8_t s = 0; s < 3; s++) {
        uint8_t color = pgm_read_byte(&tablaSalidas[estado][s]);
        uint8_t rojo  = pgm_read_byte(&tablaLeds[s][0]);
        uint8_t verde = pgm_read_byte(&tablaLeds[s][1]);

        switch (color) {
            case ROJO: p |= rojo; break;
            case VERDE:
            case AZUL: p |= verde; break;
            case PARPADEANDO: p |= verde; b |= verde; break;
            default: break;
        }
    }
    *patron = p;
    *blink  = b;
}

int main(void) {
    DDRD  |= LEDS_MASK;
    PORTD &= ~LEDS_MASK;

    DDRB  &= ~((1 << DIP_VELOCIDAD_0) | (1 << DIP_VELOCIDAD_1));
    PORTB |=  (1 << DIP_VELOCIDAD_0) | (1 << DIP_VELOCIDAD_1);
    DDRC  &= ~(1 << DIP_CRUCE);
    PORTC |=  (1 << DIP_CRUCE);

    uint8_t estado = S0;

    while (1) {
        uint16_t t = pgm_read_word(&tablaVelocidades[leerVelocidad()]);

        if (t == 0) {                         // 00 = sistema apagado
            PORTD &= ~LEDS_MASK;
            estado = S0;
            banderaPeatonal = 0;
            _delay_ms(10);
            continue;
        }

        uint8_t patron, blink;
        armarSalida(estado, &patron, &blink);
        PORTD = (PORTD & ~LEDS_MASK) | (patron & LEDS_MASK);

        uint16_t tEstado = t / 2;

        if (blink == 0) {
            retardoChecando(tEstado);
        } else {
            uint16_t sub = tEstado / FASES_PARPADEO;
            for (uint8_t i = 0; i < FASES_PARPADEO; i++) {
                PORTD ^= blink;
                retardoChecando(sub);
            }
        }

        estado = pgm_read_byte(&tablaCambios[estado][1 + banderaPeatonal]);

        if (estado == S0) banderaPeatonal = 0; // la solicitud ya se atendió
    }
    return 0;
}