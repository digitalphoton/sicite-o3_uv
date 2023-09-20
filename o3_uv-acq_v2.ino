#include <Adafruit_ILI9341.h>
//#include <SD.h>

#include <Adafruit_BMP280.h>
#include <Wire.h>

#include <math.h>

#define PORT_FILTER_CTRL PORTD
#define DDR_FILTER_CTRL DDRD
#define PIN_FILTER_CTRL PD4

#define PORT_RELAY_CTRL PORTD
#define DDR_RELAY_CTRL DDRD
#define PIN_RELAY_CTRL PD3

#define TFT_CS 7
#define TFT_DC 6
#define TFT_RST 8

#define SD_CS 10

#define VAL_BUFF_SIZE 10

#define LOW_GAIN (1000.0/1000.0*4.0*3.3)
#define HIGH_GAIN (2200.0/14200.0)

#define TUBE_LENGTH_M 0.52      // m
#define ABS_COEF 1.42529e-05    // m^2/µg

#define R_CONST 8.3144621       // J / (mol * K)
#define MM_O3 47.997            // g/mol

#define _KELVIN(T_C) ((T_C) + 273.15)
#define _PPB(C_UGM3, T, P) ((C_UGM3) * ((T) / (P)) * ((R_CONST) / (MM_O3)) * 1000)

typedef enum {

    STANDBY,
    DEBUG_INFO,
    DISP_VALUE

} Actions;

// -----------------------------------------------------------------------------

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP280 bmp;

Actions currentAction = STANDBY;
float g_vcc = 5.0;
uint8_t g_adcCount = 0;
float g_refVoltage = 0.0;

float g_adcLowValue = 0.0;
unsigned long g_adcLowCount = 0ul;

float g_adcHighValue = 0.0;
unsigned long g_adcHighCount = 0ul;

float g_valueBuffer[VAL_BUFF_SIZE];
uint8_t g_valueCount;
float g_currentValue = 0.0;

bool g_isCardPresent = true;

void setup() {

    Serial.begin(115200);
    tft.begin();

    tft.startWrite();
    tft.setRotation(1);
    tft.setCursor(0,0);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);
    tft.println();
    tft.println("Inicializando...");
    tft.endWrite();

    /*Serial.println("Inicializando cartão...");
    Serial.flush();

    if( !SD.begin(SPI_FULL_SPEED, SD_CS) ) {

        g_isCardPresent = false;
        Serial.println("Cartão SD ausente, prosseguindo sem log");

    }*/

    Serial.println("Configurando GPIOs");

    bitSet(DDRD, PD2);

    bitSet(DDR_RELAY_CTRL, PIN_RELAY_CTRL);

    bitSet(DDR_FILTER_CTRL, PIN_FILTER_CTRL);
    bitSet(PORT_FILTER_CTRL, PIN_FILTER_CTRL);

    //bitSet(DDRD, PD5);

    Serial.println("Configurando ADC...");

    adcSetup();

    // definir entrada do ADC como o pino A0
    ADMUX &= ~(0b1111 << MUX0);

    Serial.println("Configurando Timer 1...");

    timer1Setup(0xffff, 0x7fff);

    /*String fio = "referencia; delta; ugm3; ppb;";
    if(g_isCardPresent) {

        Serial.println("Abrindo arquivo de log...");
        File arquivo = SD.open("log.txt", FILE_WRITE);
        if(arquivo) {

            Serial.println("Arquivo aberto com sucesso.");
            arquivo.println(fio);
            arquivo.close();

        }
        else {

            g_isCardPresent = false;
            Serial.println("Erro ao abrir o arquivo, log desabilitado nessa sessão.");

        }

    }
    Serial.println(fio);*/

    Serial.println("Configurando Sensor BMP280");
    bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

    Serial.print("VCC = ");
    Serial.print(g_vcc);
    Serial.println(" V");

    Serial.println("referencia [V]; delta (amplificado) [V]; temperatura [°C]; pressao [kPa]; concentracao [ug/m^3]; concentracao [ppb]");

}

void loop() {

    switch(currentAction) {

        default:
        case STANDBY:

            break;

        case DEBUG_INFO:

            printDebugInfo();
            currentAction = STANDBY;
            break;
        
        case DISP_VALUE:

            printValue();
            currentAction = STANDBY;
            break;

    }

}

void timer1Setup(uint16_t regA, uint16_t regB) {

    // CTC, TOP = OCR1A
    TCCR1A = (0b00 << COM1A0) | (0b00 << COM1A1) | (0b00 << WGM10);
    TCCR1B = (0b01 << WGM12) | (0b101 << CS10);

    // Interrupts: COMPB, COMPA
    TIMSK1 = (1 << OCIE1B) | (1 << OCIE1A);

    OCR1A = regA;
    OCR1B = regB;

    TCNT1 = 0x0000;

}
void adcSetup(void) {

    // Prescaler = 128
    ADCSRA = _BV(ADEN) + (0b101 << ADPS0);

    // Trigger source: free running mode
    ADCSRB = (0b000 << ADTS0);

    // Reference: AVcc, Source: Internal reference
    ADMUX = (0b01 << REFS0) + (0b1110 << MUX0);

    _delay_ms(2);   // Necessário para a referência estabilizar

    // Start a conversion
    bitSet(ADCSRA, ADSC);

    while(bit_is_set(ADCSRA, ADSC));

    // Calculate Vcc based on the measurement of the internal reference
    g_vcc = 1.1 * 1024.0 / ADC;

    // Enable ADC interrupt request
    bitSet(ADCSRA, ADIE);

}

void printDebugInfo(void) {

    Serial.print("Value: ");

    float media = 0.0;
    for(int i = 0; i < VAL_BUFF_SIZE; i++) {

        media += g_valueBuffer[i];

    }
    media /= VAL_BUFF_SIZE;

    Serial.print(media, 3);
    Serial.print(" V");

    Serial.println();

}
void printValue(void) {

    g_refVoltage /= HIGH_GAIN;
    Serial.print(g_refVoltage, 6);
    Serial.print("; ");


    Serial.print(g_valueBuffer[g_valueCount], 6);
    Serial.print("; ");

    float temp = bmp.readTemperature();
    Serial.print(temp, 2);
    Serial.print("; ");

    float press = bmp.readPressure();
    Serial.print(press / 1000.0, 6);
    Serial.print("; ");

    float ugm3 = calcConc(g_valueBuffer[g_valueCount]);
    Serial.print( ugm3 );
    Serial.print("; ");

    float ppb = _PPB(ugm3, _KELVIN(temp), press);
    Serial.print( ppb );
    Serial.println("; ");

    tft.startWrite();

    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);

    tft.println();
    tft.println("Atual: ");
    tft.print("\n    ");
    tft.print( (int)ugm3 );
    tft.print(" ug/m^3");
    tft.print("\n    ");
    tft.print( (int)ppb );
    tft.print(" ppb");
    tft.print("\n\n");

    tft.endWrite();

    /*if(g_isCardPresent) {

        File arquivo = SD.open("log.txt", FILE_WRITE);

        arquivo.print(g_refVoltage, 6);
        arquivo.print("; ");
        arquivo.print(g_valueBuffer[g_valueCount], 6);
        arquivo.print("; ");
        arquivo.print(ugm3, 2);
        arquivo.print("; ");
        arquivo.print(ppb, 2);
        arquivo.println("; ");

        arquivo.close();
        SPI.transfer(0x00);

    }*/

}

float calcConc(float delta) {

    return -log( 1 - delta/(LOW_GAIN*g_refVoltage) ) / (TUBE_LENGTH_M * ABS_COEF);

}

// Interrupções ----------------------------------------------------------------

ISR(TIMER1_COMPA_vect) {

    // Interromper leitura do ADC
    bitClear(ADCSRA, ADIE);
    bitClear(PORTD, PD2);

    if(g_adcCount == 0) return;

    g_adcLowValue /= g_adcLowCount;
    g_adcHighValue /= g_adcHighCount;

    if( bit_is_clear(PORT_FILTER_CTRL, PIN_FILTER_CTRL) ) {

        g_valueBuffer[g_valueCount] += g_adcLowValue;
        bitSet(PORT_FILTER_CTRL, PIN_FILTER_CTRL);
        bitSet(PORT_RELAY_CTRL, PIN_RELAY_CTRL);

        currentAction = DISP_VALUE;

    }
    else {

        g_valueBuffer[g_valueCount] = - g_adcLowValue;
        g_refVoltage = g_adcHighValue;

        bitClear(PORT_FILTER_CTRL, PIN_FILTER_CTRL);
        bitClear(PORT_RELAY_CTRL, PIN_RELAY_CTRL);

        if(g_valueCount >= VAL_BUFF_SIZE) {

            g_valueCount = 0;

        }

    }

}
ISR(TIMER1_COMPB_vect) {

    ADMUX &= ~(0b1111 << MUX0);

    // Iniciar leitura do ADC
    bitSet(PORTD, PD2);
    bitSet(ADCSRA, ADIE);
    bitSet(ADCSRA, ADSC);

    g_adcLowValue = 0.0;
    g_adcLowCount = 0ul;
    g_adcHighValue = 0.0;
    g_adcHighCount = 0ul;
    
}

ISR(ADC_vect) {

    switch( g_adcCount % 2 ) {

        default:
        case 0:

            g_adcLowValue += g_vcc * ADC / 1024.0;
            g_adcLowCount++;

            ADMUX &= ~(0b1111 << MUX0);
            ADMUX |= (0b0001 << MUX0);
            break;

        case 1:

            g_adcHighValue += g_vcc * ADC / 1024.0;
            g_adcHighCount++;

            ADMUX &= ~(0b1111 << MUX0);
            break;

    }

    bitSet(ADCSRA, ADSC);
    g_adcCount++;
    //bitToggle(PORTD, PD5);

}
