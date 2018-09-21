#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL // 16 MHz, clock_ms externo, define necessario para usar delay corretamente
#define OVF_T0_1_ms (F_CPU/(1000UL*256))
#define BAUDRATE 9600        //taxa de transmissao em bps
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
// DHT no pino PC0       => A0
#define DHT_PIN PC0
#define THERMISTOR PC1   //A1

#define LED_YELLOW_D PD3
#define LED_BLUE_B PB3
#define BUTTON_D PD2
#define LED_GREEN_B PB0
#define MOTOR LED_BLUE_B //A saida para do LED e do MOTOR eh a mesma

#define BETA 1/50.0
#define ALFA 1/90.0
// Motor no pino  PB3 (pino 11 do arduino)
// Led com PWM no pino PD3 (pino 3 do arduino)
enum {
  INIT = 0,
  CTRL = 1,
  STOP = 2
};
// Variaveis globais
volatile const uint8_t ovf_T0_1ms = OVF_T0_1_ms;
volatile uint8_t count_ovf_T0 = 0;
volatile uint64_t timer_ms = 0;
volatile uint8_t state = INIT;
volatile bool bounce_aux = false;
uint64_t t_first_edge = 0;
//#######################################
enum PWM_CHANNEL{
  //Canal por nome
  CHAN_MOTOR = 0b00000001,
  CHAN_LED   = 0b00000010,
};

//Funcoes
uint8_t readDHT_byte();//funcao auxiliar de readDHT(data[])
bool readDHT(uint8_t data[]);
uint8_t readTherm();

uint64_t clock_ms(); //retorna o tempo corrido de execucao em ms

void usart_initialize();
void usart_transmision(uint8_t data);
void usart_transmisionString(uint8_t *data);
uint8_t usart_reception();
void usart_flush();

void pwm_initialize(){
  // Definir o modo de operação para FastPWM
  TCCR2A &= ~(1 << WGM22);
  TCCR2A |= (1 << WGM21 | 1 << WGM20);

  // Definir a fonte do clk (clk_i/256) => 16e6/256 = 62.5KHz
  TCCR2B &= ~(1 << CS20)
  TCCR2B |= (1 << CS21) | (1 << CS22);

  // Definido para 0% do ciclo de trabalho
  OCR2A = 0x00;
  OCR2B = 0x00;

  //Configura os pinos do PWM como saida
  DDRB |= 1 << LED_BLUE_B; // OC2A
  DDRD |= 1 << LED_YELLOW_D; // OC2B
}
// Ativar canais PWM
void pwm_enable (enum PWM_CHANNEL channel) {
  if (channel & CHAN_MOTOR) TCCR2A |= 1 << COM2A1;
  if (channel & CHAN_LED) TCCR2A |= 1 << COM2B1;
}
// Desativar canais PWM
void pwm_disable (enum PWM_CHANNEL channel) {
  if (channel & CHAN_OC2A) TCCR2A &= ~(1 << COM2A1);
  if (channel & CHAN_OC2B) TCCR2A &= ~(1 << COM2B1);
}
// Define o ciclo de trabalho no canal PWM_CHANNEL
void pwm_dutycycle (enum PWM_CHANNEL channel, uint8_t dutycycle){
  if (channel & CHAN_OC2A) OCR2A = dutycycle;
  if (channel & CHAN_OC2B) OCR2B = dutycycle;
}

void setup(){
  DDRB |= 1 << LED_GREEN_B; //configura LED_GREEN_B como saida
  DDRD &= !(1 << BUTTON_D); //configura BUTTON_D como entrada
  DDRC |= 1 << DHT_PIN;


  PORTC|= !(1 << DHT_PIN);      //coloca DHT_PIN para nivel alto
  PORTB&= !(1 << LED_GREEN_B);  //pino LED_GREEN_B para nivel baixo


  //configura interrupcao para borda de descida no BUTTON_D
  EICRA &= !(1 << ISC00);
  EICRA |=  (1 << ISC01);
  EIMSK |=  (1 << INT0); //acionar apenas INT0

  //configuracoes do contador T1
  //Modo normal de operacao e sem prescaler.
  TCCR1A = 0x0;
  TCCR1B |= 1 << CS10; //no prescaler.
  //coloca um valor para Math A, que corresponda a 1ms.
  OCR1AH = (F_CPU/1000UL) >> 8;
  OCR1AL = (F_CPU/1000UL) << 8;
  TIMSK1 |= 1 << OCIE1A; //habilita interrupcao para Math A.


  usart_initialize();
  pwm_initialize();
  pwm_enable( CHAN_LED | CHAN_MOTOR );

  sei();//habilita interrupcoes, chave global
}

int main(){
  setup();
  // index 0 => Humidade %, parte inteira
  // index 1 => Humidade %, parte decimal
  uint8_t dht_data[5];
  uint64_t start_timer;
  uint64_t timer;
  uint8_t duty_MOTOR = 0, duty_LED = 255;
  uint8_t T;
  int z;
  uint8_t U;

  while(true){
      switch (state) {
        case INIT:
          duty_MOTOR = 0;
          duty_LED = 0;
          PORTB &= ~(1 << LED_GREEN_B);
          pwm_dutycycle(CHAN_LED,0);
          pwm_dutycycle(CHAN_MOTOR,0);
          start_timer = clock_ms();
        break;
        case CTRL:
          PORTB |= (1 << LED_GREEN_B);
          timer = clock_ms() - start_timer;
          readDHT(dht_data);
          U = dht_data[0];
          T = readTherm();
          duty_LED = (U/90.0)*256;

          if( timer < 10000){
            //comportamento 1
            z = (3.5/100.0)*(timer/1000.0)*256;
          }else if(timer > 10000 && timer < 15000){
            //comportamento 2
            z = (35/100.0)*256;
          }else if(timer > 15000 && timer < 20000){
            //comportamento 3
            z = ((((65-32)/5.0)/100.0)*(timer/1000.0 - 15) + 35/100.0)*256;
          }else if(timer > 20000 && timer < 25000){
            //comportamento 4
            z = (65/100.0)*256;
          }else if(timer > 25000 && timer <= 30000){
            //comportamento 5
            z = (((-65.0/5)/100.0)*(timer/1000.0 - 25) + 65/100.0)*256;
          }else{
            //finish
            state = INIT;
          }

          duty_MOTOR = z*U*ALFA + BETA*T;
          pwm_dutycycle(CHAN_MOTOR, duty_MOTOR);
          pwm_dutycycle(CHAN_LED, duty_LED);

          usart_transmision(duty_MOTOR);

        break;
        default:

        break;
      }
  }

  return 0;
}

//Interrupcao responsavel por atualizar o estado da maquina de estados
//esta associada a interrupcao gerada pelo botao e eh realizado
//tratamento do bounce.
ISR(INT0_vect){
  if(bounce_aux){//primeira chamada da interrupcao
   t_first_edge = clock_ms();
   bounce_aux = false;
  }
  else{
    if((clock_ms() - t_first_edge) > 100){//Teste se houve 100ms de diferenca entre interrupcao atual e a ultima
      bounce_aux = true;
      state = (state == INIT)?CTRL:INIT;
    }
  }
}
ISR(TIMER1_COMPA_vect){
  TCNT1H = 0;
  TCNT1L = 0;
  timer_ms++;
}
ISR(USART_RX_vect){
  uint8_t data = usart_reception();
  usart_flush();
  if(data == '1')state = CTRL;
  else if(data == '0')state = INIT;
  else {
    PORTB |=  (1 << LED_GREEN_B);
    _delay_ms(500);
    PORTB &= ~(1 << LED_GREEN_B);
    _delay_ms(500);
  }
}
ISR(USART_TX_vect){//interrupcao: transmissao completa

}

uint64_t clock_ms(){ return timer_ms; }

//Realiza a conversao ADC e retorna o valor em Graus Celcius
uint8_t readTherm(){
  ADCSRA |= 0b10000111; //divide o clock_ms por 128 (o clock_ms de conversao sera 12Mhz/128)

  ADMUX  |= 0b01000000;//usa o Vcc como ref
  ADMUX  |= 0b00000001; //converte do pino A1
  ADCSRA |= 0b01000000; // inicia a conversao A/D
  while(!(ADCSRA & 0b00010000)); //Aguarda a conversao ser concluida
  return (ADC*27)/512;
}
uint8_t readDHT_byte(){
  uint8_t data = 0;
  for(uint8_t i = 0; i < 8; i++){
    while( !(PINC & (1<<DHT_PIN)));//espera sair da zona LOW
    _delay_us(30);
    if( PINC & (1<<DHT_PIN) )//se ler nivel Alto nessa linha, entao o bit eh 1
      data |= 1 << (7-i);
    while( PINC & (1<<DHT_PIN) );//espera receber o nivel LOW indicando o proximo bit
  }
  return data;
}
bool readDHT(uint8_t data[]){
  uint8_t dht_in = 0;
  //Pedido de transmissao de dados
  DDRC |= 1 << DHT_PIN;
  PORTC&= !(1<<DHT_PIN);
  _delay_ms(18); //ate 18ms
  PORTC|= (1<<DHT_PIN);
  _delay_us(1);

  //Confirmacao do DHT
  DDRC &= !(1<<DHT_PIN);
  _delay_us(80); //ate 80us
  dht_in = PINC & (1<<DHT_PIN);
  if(dht_in)
    return false;

  _delay_us(80);//ate 80us
  dht_in = PINC & (1<<DHT_PIN);
  if(!dht_in)
    return false;

  //inicio da transmissao dos dados
  _delay_us(80);
  for(uint8_t i = 0; i < 5; i++){
    data[i] = readDHT_byte();
  }

  uint8_t checksum = data[0] + data[2];

  if(checksum != data[4])
    return false;

  return true;
}
void usart_initialize(){
  //habilita a comunicacao USART
  //habilita entrada (Rx) e saida(Tx)
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (3<<UCSZ00);

  // definir a taxa de transmissao
  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  //formato do frame 8bits de dado e 1stop bits
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);//ativa o receptor e o transmissor
  UCSR0C = (3<<UCSZ00);
  //Habilita interrupcoes locais
  UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);
}
void usart_transmisionString(uint8_t *data){
  while(*data != 0x00){
    usart_transmision(*data);
    data++;
  }
}
//Coloca um byte no buffer de saida de dado
void usart_transmision(uint8_t data){
  //Aguarda a ultima transmissao ser concluida
  while( !(UCSR0A & (1<<UDRE0)));
  //coloca o byte no buffer
  UDR0 = data;
}
//Aguarda a recepcao ser concluida
uint8_t usart_reception(){
  //aguardar buffer a finalizacao da recepcao
  while ( !(UCSR0A & (1 << RXC0)) );
  return UDR0;
}
//Limpa o buffer de entrada de dados
void usart_flush(){
  uint8_t trash;
  while ( (UCSR0A & (1<<RXC0))) trash = UDR0;
}
