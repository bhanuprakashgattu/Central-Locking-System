#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SET_BIT(PORT,PIN)  PORT |= (1<<PIN)
#define CLR_BIT(PORT,PIN)  PORT &= ~(1<<PIN)

#define START_ADC  ADCSRA |= (1<<ADSC)
#define ENABLE_ADC ADCSRA |= (1<<ADEN)

volatile uint8_t Flag1=0;
volatile uint8_t Flag2=0;

void ADC_init()
{
    ENABLE_ADC;
    ADMUX |= ((1<<REFS0)); // reference voltage and left adjust
    ADCSRA |= ((1<<ADPS1)|(1<<ADPS2)); // division factor
}

uint16_t READ_ADC()
{
    START_ADC;    // starts ADC
    while(ADCSRA & (1<<ADSC)); // pooling ADC
    return ADC; // returns ADC values
}

void DoorsLocked()
{
    SET_BIT(PORTD,PD4);
    SET_BIT(PORTD,PD5);
    SET_BIT(PORTD,PD6);
    SET_BIT(PORTD,PD7);
}

void DoorsOpened()
{
    CLR_BIT(PORTD,PD4);
    CLR_BIT(PORTD,PD5);
    CLR_BIT(PORTD,PD6);
    CLR_BIT(PORTD,PD7);
}

void Init_CentralLockingSystem()
{
    ADC_init();
    SET_BIT(DDRD,PD4); //Right Front
    SET_BIT(DDRD,PD5); // Left Front
    SET_BIT(DDRD,PD6);
    SET_BIT(DDRD,PD7);

    CLR_BIT(DDRD,PD2);  // Engine Switch
    SET_BIT(PORTD,PD2);  // PULL-UP enabled

    CLR_BIT(DDRD,PD3);  // doorLocking Switch
    SET_BIT(PORTD,PD3);  // PULL-UP enabled

    SREG |= (1<<7); //Global inetrrupt

    EICRA |= ((1<<ISC10)|(1<<ISC00)); //the falling edge will trigger  an interrupt
    EIMSK |= ((1<<INT1)|(1<<INT0));   //PD2=INT0,PD3=INT1;
}

int main()
{
    int speed=0,speedFlag=0;
    Init_CentralLockingSystem();

    while(1)
    {
        if(Flag1)
        {
            if(Flag2==1 || speedFlag==1)
            {
                DoorsLocked();
                _delay_ms(2000);
                if(Flag2==1)
                    speedFlag=0;
            }
            else
            {
                if(Flag1==1)
                {
                    speed = READ_ADC();
                    if(speed>500)
                        speedFlag=1;
                        
                }
                DoorsOpened();
            }
        }
        else
        {
           DoorsOpened();
        }

    }

}
ISR(INT0_vect) //engine switch
{
    Flag1=!Flag1;
}
ISR(INT1_vect) //
{
    if(Flag1==1)
        Flag2=!Flag2;

}
