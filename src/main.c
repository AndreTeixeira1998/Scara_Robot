/*******************************************************
 *
 * main.c
 *
 *  Created on: 16/11/2018
 *      Author: Andreia Seabra e André Teixeira
 * 
 * main.c v2
 * 
 * Movimento dos dois motores nas interrupçoes
mak * Para fazer movimentar os motores fornecer-lhes:
 *   - motorx.step --- numero de steps
 *   - motorx.t_between_step --- tempo entre cada step csec
 *   - motorx.dir --- indica a direçao do morotor
 * main.c v2
 *  
 * Funções DegreetoRad , vice-versa
 *         Distancia entre dois pontos
 *         DirectKinematics
 * 
 * main.c v10.1
 *  -Reta (check)
 ******************************************************/
#include <avr/interrupt.h> /*Interrupts library */
#include <avr/io.h> /* Registers */
#include <util/delay.h> /* Delays library */
#include <avr/eeprom.h> 

//#include "printf_tools.h" 
//#include "serial_printf.h" 
#include <math.h>

#include "uart.h" 
//#define DEBUG

//pins
#define M1STEP PC0
#define M1DIR PC1
#define M2STEP PC2
#define M2DIR PC3
#define MS1 PB5
#define MS2 PB4
#define MS3 PB3

#define FimCurso1 PD2
#define FimCurso2 PD3

#define TCONT 100

/*Testing...*/
#define NSTEP1 10000
#define NSTEP2 10000

#define ERROR 50

/*Motor variables*/
#define CLOCKWISE 1
#define COUNTERCLOCKWISE -1
#define NCOIL 5

/*Using half step mode, this motor has 360/5.625*64 = 4096 steps*/
#define DEGREE_PER_STEP (1.8/pow(2, motor1.modo)) 

/*Arm variables*/
#define ARM_SIZE 20
#define FOREARM_SIZE 20

/*Size of small straight*/
#define D_SCALE 0.1
#define UP 1
#define DOWN 0


#define TPREDEFI 10
#define TINICIAL 1000

#define FULLSTEP 0
#define HALFSTEP 1
#define QUARTERSTEP 2
#define EIGHTSTEP 3
#define SIXTEENSTEP 4

//MODES
#define WRITE 1
#define WALK 0

#define TRUE 1
#define FALSE 0
#define T1BOTTOM 65536-2500 /*10ms*/

// REPOSITORY PARAMETERS
#define END_OF_THE_LINE 30
#define ALPHABET 25
#define KATAKANA 25
#define INTERFACE 0
#define TRABALHAR 1
#define HATARAKU 2
#define DATASIZE 0 //PRIMEIRA POSICAO DO VETOR CONTEM O TAMANHO DA INFORMACAO
#define WORD_POINT_X count_word+1
#define WORD_POINT_Y count_word+2
#define WORD_MODE count_word+3

//EEPROM
#define SIG8 0b01010101

uint8_t EEMEM signature;
uint8_t EEMEM ADD_FOREARM;
uint8_t EEMEM ADD_ARM;


uint8_t FOREARM=FOREARM_SIZE;
uint8_t ARM=ARM_SIZE;

uint8_t repository1[ALPHABET][END_OF_THE_LINE]={ // TAMANHO, POSX, POSY, MODO
    {15 , 50 ,50 ,WALK, 65 ,100 ,WRITE, 80 ,50  ,WRITE, 75, 75, WALK , 55,  75,WRITE},                                              //a
    {24 , 70 ,50 ,WALK, 70 ,100 ,WRITE, 80 ,100 ,WRITE, 80, 82, WRITE, 70, 75,WRITE, 80, 70, WRITE, 80, 50, WRITE, 70,50,WRITE },   //b 
    {12 , 70 ,50 ,WALK, 50 ,50  ,WRITE, 50 ,100 ,WRITE, 70, 100,WRITE},                                                             //c
    {21 , 60 ,50 ,WALK, 60 ,100 ,WRITE, 70 ,100 ,WRITE, 75, 90, WRITE, 75, 55,WRITE, 70, 50, WRITE, 60, 50, WRITE},                 //d
    {18 , 70 ,50 ,WALK, 50 ,50  ,WRITE, 50 ,100 ,WRITE, 70, 100,WRITE, 50, 75,WALK,  60, 75, WRITE},                                //e
    {15 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 70 ,100 ,WRITE, 50, 75, WALK,  60, 75,WRITE},                                               //f
    {21 , 67 ,65 ,WALK, 67 ,75  ,WRITE, 75 ,75  ,WRITE, 75, 50, WRITE, 55, 50,WRITE, 55, 100,WRITE, 75,100,WRITE},                  //g
    {18 , 52 ,50 ,WALK, 52 ,100 ,WRITE, 52 ,75  ,WALK,  70, 75, WRITE, 70, 50,WALK,  70, 100,WRITE},                                //h
    {18 , 50 ,50 ,WALK, 70 ,50  ,WRITE, 60 ,50  ,WALK,  60, 100,WRITE, 50, 100,WALK, 70, 100,WRITE },                               //i
    {12 , 50 ,70 ,WALK, 50 ,50  ,WRITE, 62 ,50  ,WRITE, 62, 100,WRITE},                                                             //j
    {18 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 50 ,80  ,WALK,  60, 100,WRITE, 50, 80,WALK,  60, 50, WRITE},                                //k
    {9  , 50 ,100,WALK, 50 ,50  ,WRITE, 90 ,50  ,WRITE},                                                                            //l
    {15 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 60 ,80  ,WRITE, 70, 100,WRITE, 70, 50,WRITE},                                               //m
    {12 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 65 ,50  ,WRITE, 65, 100,WRITE},                                                             //n
    {15 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 65 ,100 ,WRITE, 65, 50, WRITE, 50, 50,WRITE},                                               //o
    {15 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 65 ,100 ,WRITE, 65, 75, WRITE, 50, 75,WRITE},                                               //p
    {21 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 65 ,100 ,WRITE, 65, 50, WRITE, 50, 50,WRITE, 60, 65, WALK, 70,30,WRITE },                   //q
    {18 , 50 ,50 ,WALK, 50 ,100 ,WRITE, 65 ,100 ,WRITE, 65, 80, WRITE, 50, 80,WRITE, 62, 50, WRITE},                                //r
    {18 , 50 ,50 ,WALK, 65 ,50  ,WRITE, 65 ,75  ,WRITE, 50, 75, WRITE, 50, 100,WRITE,65, 100,WRITE},                                //s
    {12 , 60 ,50 ,WALK, 60 ,100 ,WRITE, 50 ,100 ,WALK,  70, 100,WRITE},                                                             //t
    {12 , 50 ,100,WALK, 50 ,50  ,WRITE, 90 ,50  ,WRITE, 90, 100,WRITE},                                                             //u
    {12 , 50 ,100,WALK, 57 ,50  ,WRITE, 65 ,100 ,WRITE},                                                                            //v
    {12 , 50 ,50 ,WALK, 65 ,100 ,WRITE, 50 ,100 ,WALK,  65, 50, WRITE},                                                             //x
    {12 , 55 ,75 ,WALK, 60 ,100 ,WRITE, 50 ,100 ,WALK,  57, 50, WRITE},                                                             //y
    {12 , 50 ,100,WALK, 65 ,100 ,WRITE, 50 ,50  ,WRITE, 65, 50, WRITE},                                                             //z
};

/*
uint8_t repository2[KATAKANA][END_OF_THE_LINE+1]={ 
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //a
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //b 
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //c
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //d
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //e
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //f
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //g
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //h
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //i
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //j
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //k
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //l
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //m
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //n
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //o
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //p
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //q
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //r
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //s
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //t
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //u
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //v
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //x
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //y
    { END_OF_THE_LINE, 1,2,3,4,5,6, 7,8,9}, //z
};
*/
typedef struct{
    volatile uint16_t move; /*TRUE OR FALSE*/
    volatile uint16_t nmove;
    volatile uint16_t t_until_nmove; /*centiseconds*/
    volatile uint16_t t_between_step; /*centiseconds*/
    volatile uint16_t step;
    volatile int dir;
    volatile uint16_t modo;
}StepMotor;

typedef struct{
    float x;
    float y;
    double theta1;
    double theta2;
}Point;

StepMotor motor1 = {0,0,100,0,0,CLOCKWISE,SIXTEENSTEP};
StepMotor motor2 = {0,0,100,0,0,CLOCKWISE,SIXTEENSTEP};
Point Origin = {0, 0};
Point MyPos = {20,0}; //Origin

volatile uint16_t coil[NCOIL]={0,              
                        (1<<MS1),
                        (1<<MS2),
                        (1<<MS1)|(1<<MS2),
                        (1<<MS1)|(1<<MS2)|(1<<MS3)
                        };

/*kUSUME*/
volatile uint16_t counttimer=0;

int p(float value){
    return (int)(value*1000);
}

/*void putLong(long x)
{
    if(x < 0)
    {
        putchar('-');
        x = -x;
    }
    if (x >= 10) 
    {
        putLong(x / 10);
    }
    putchar(x % 10+'0');
}
*/
/*void putDouble(double x, int p)
{
    long d;
    if (x<0) {
        putchar('-');
        x=-x;
    }
    d = x;
    putLong(d);
    putchar('.');
    while (p--) {
        x = (x - d) * 10;
        d = x;
        putchar('0'+d);
    }
    putchar(' ');
}

*/

void sobe(void){
	while(OCR0A > 7){
		OCR0A -= 1;
		_delay_ms(100);
	}
}

void desce(void){
	while(OCR0A < 37){
		OCR0A += 1;
		_delay_ms(100);
	}
}

void tc0_init(void){
	DDRD |= (1<<PD6); //initialize PORTD for output
	TCCR0A  = (1<<WGM00)|(1<<WGM01);   //fast pwm,TOP=0xff 
	TCCR0B  = (1<<CS12)|(1<<CS10);     //presc 1024 
	TIMSK0  = (1<<OCIE0A)|(1<<TOIE0); //Enable interrupts 
	OCR0A = 18;
}

ISR (TIMER0_COMPA_vect){
      PORTD &= ~(1<<PD6);   //clear output
	 // printf("beep");
}

ISR (TIMER0_OVF_vect){
      PORTD |= (1<<PD6);   //set output
	  //printf("boop\n");
}

void tc1_init(void){
    /*Iinicialização do timer 1*/

    TCCR1B = 0; //Stop timer 1 
    TIFR1 = (7<<TOV1)|(1<<ICF1); //Clear all pending interrupts
    TCCR1A = 0; /* Normal mode */
    TCNT1 = T1BOTTOM; /* Load botom value */
    TIMSK1 = (1<<TOIE1); /* Enable interrupts by overflow */
    TCCR1B = 3; /* timer prescaler = 100b */
    sei();
}

ISR(TIMER1_OVF_vect){ /* No interrupt do timer1 são executados os movimentos do motor*/
    TCNT1 = T1BOTTOM;
                                       //ALTEREI ISTO
    motor1.t_until_nmove--; 
    motor2.t_until_nmove--;
    //printf("%d -- %d \n\r", motor1.t_until_nmove, motor2.t_until_nmove);
    /*Move motor1*/
    
    //printf("%d", motor2.dir);
    /*DIREÇAO 1*/
    if(motor1.dir == CLOCKWISE){ //printf("CLOCKWISE\n\r");
        PORTC |=(1<<M1DIR);
        //motor1.stepdir--;        
    }
    else{/*COUNTERCLOCKWISE movement*/ //printf("COUNTERCLOCKWISE\n\r");
        PORTC &=~(1<<M1DIR);
        //motor1.stepdir++;
    }

    /*DIREÇAO 2*/
    if(motor2.dir == CLOCKWISE){ 
        PORTC |=(1<<M2DIR);
     
    }

    else{/*COUNTERCLOCKWISE movement*/ //printf("COUNTERCLOCKWISE\n\r");
        PORTC &=~(1<<M2DIR);

    }


    if((motor1.step!=0) && (motor1.move==TRUE)){
        PORTC |=(1<<M1STEP); /*Movement*/
        _delay_us(1);
        PORTC &=~(1<<M1STEP);
        motor1.step--;
        motor1.move = FALSE;
        //printf("--Motor1 %d-- ", motor1.step);
    }
    

    if((motor2.step!=0) && (motor2.move==TRUE)){ 
        PORTC |=(1<<M2STEP); /*Movement*/
        _delay_us(1);
        PORTC &=~(1<<M2STEP);
        motor2.step--;
        //printf("--Motor1 %d-- ", motor1.step);
        motor2.move=FALSE; 
    } 

    /*Verifica se o motor deve mover na proxima interrupçao*/
    if(0==motor1.t_until_nmove){
        motor1.t_until_nmove=motor1.t_between_step;
        motor1.move=TRUE;
    }
    if(0==motor2.t_until_nmove){
        motor2.t_until_nmove=motor2.t_between_step;
        motor2.move=TRUE;
    }

}                

void start_timer(int count){
    counttimer = count;
}

int get_timer(void){
    if(0 == counttimer){
        return 1;
    }
    return 0;
}

void hw_init(void){   //MUDEI ISTO
  
    DDRC = DDRC| (1<<M1STEP)|(1<<M1DIR)|(1<<M2STEP)|(1<<M2DIR); 
    DDRB = DDRB|(1<<MS1)|(1<<MS2)|(1<<MS3);
    DDRD = DDRD & ~((1<<FimCurso1) | (1<<FimCurso2));
    PORTD |=(1<<FimCurso1) | (1<<FimCurso2);  
   
    EICRA = (2<<ISC00)|(2<<ISC10);
    EIMSK = (1<<INT0)|(1<<INT1);
    sei();
    //PORTC = PORTC| (1<<M1STEP)|(1<<M1DIR)|(1<<M2STEP)|(1<<M2DIR); 
    PORTC = 0;  
    PORTB |= coil[motor1.modo];

}
ISR(INT0_vect){
    motor1.step = 0 ;
}
ISR(INT1_vect){
    motor2.step = 0 ;
}

float dist(Point p, Point l){
    return sqrt(pow(l.x-p.x,2)+pow(l.y-p.y,2));
}

float RadtoDegree(float angle){
    return angle*180/M_PI;
}

float DegreetoRad(float angle){
    return angle*M_PI/180;
}

float LawOfCos(float a, float b, float c){
    //printf("lawofcos %d %d %d\n\r",(int)a, (int)b, (int)c);
    return acos( ( pow(a,2) + pow(b,2) - pow(c,2) ) / (2 * a *b) );
}

void EquationOfaStraightLine(Point MyPos, Point TargetPos, float *m, float *b){
    /*
    * y = mx + b
    */

    //printf("Equation of a straight line\n\r");
    //printf("MyPos.x %d , MyPos.y %d , TargetPos.x %d , TargetPos.y %d\n\r ", (int)(1000*MyPos.x), (int)(1000*MyPos.y), (int)(1000*TargetPos.x), (int)(1000*TargetPos.y));
    if(MyPos.x == TargetPos.x){
        *m = 0;
        *b = MyPos.x;
    }
    else{
        *m = (TargetPos.y - MyPos.y) / (TargetPos.x-MyPos.x);
        *b = MyPos.y - (*m) * MyPos.x;
    }

}

int abs(int number){
   
    if(number<0)
        return -number;
    else
        return number;
}

int LCM (int a, int b){
    /*
    * It is known that LCM = a * b / gdc(a,b)
    * Where LCM is least common multiple and
    * GDC is greatest common divisor
    */
    if(0 == a){
       return b;
    }
    else if( 0 == b){
        return a;
    }
    else{
        int gdc=0, lcm, i;

        for(i=1; i <= a && i<=b; i++){
            if( (0 == a%i) && (0 == b%i))
                gdc = i;
        }

        lcm = a * b / gdc ;
        return lcm;
    }
    return -1;
}

void ChooseDir(volatile int *dir, float angle_ini, float angle_fin){

    //printf("  Angle init %d Angle final %d", (int)(angle_ini*1000), (int)(angle_fin*1000));
    if(angle_fin >= angle_ini){
        *dir = CLOCKWISE;
    //    printf(" CLOCKWISE \n\r");
    }
    else{
        *dir = COUNTERCLOCKWISE;
    //    printf(" COUNTERCLOCKWISE \n\r");
    }
}

void DirectKinematics(double theta1, double theta2, float *x, float *y){
    /*Angles most come in degree*/

    double theta1_rad = DegreetoRad(theta1);
    double theta2_rad = DegreetoRad(theta2);
    *x = FOREARM * cos(theta1_rad) + ARM * cos(theta1_rad + theta2_rad);
    *y = FOREARM * sin(theta1_rad) + ARM * sin(theta1_rad + theta2_rad);
    
}

void InverseKinematics(Point robot, Point target, double *theta1, double *theta2){

    //printf("%d tx %d ty %d rx %d ry", (int)target.x, (int)target.y, (int)robot.x, (int)robot.y);
    float length = dist(robot, target);
    //printf("---%d distancia---", (int)length);
    if(length > (ARM + FOREARM)){
        printf("Too far\n\r");
    }
    else{
        float alpha = LawOfCos(length, FOREARM, ARM);
        //printf("%d alpha\n\r", (int)(alpha*1000));
        float beta = LawOfCos(ARM, FOREARM, length);
        //printf("%d beta\n\r", (int)(beta*1000));
        float gama = atan2 (target.y-robot.y,target.x-robot.x);
        //printf("%d gama\n\r", (int)(gama*1000));

        *theta1 = RadtoDegree(gama - alpha);
        *theta2 = RadtoDegree(M_PI - beta);
    }
}

void CalcTime_between_step(uint16_t m1_step, uint16_t m2_step, volatile uint16_t *t1, volatile uint16_t *t2){

    int t_total = LCM( m1_step, m2_step);
    
    /* Quando o numero de steps e nulo igualou-se o tempo a 1,
       mas na verdade poderia-se igual a qualquer outro valor,
       apenas e necessária esta atribuição pois nao e possivel
       dividir um numero por zero, tornando assim o valor de
       t_between_step previsel para todos os casos
    */
    (0 == m1_step) ? (*t1 = 1) : (*t1 = t_total / m1_step);
    (0 == m2_step) ? (*t2 = 1) : (*t2 = t_total / m2_step);

}
Point CalcSubTarget(int flag, float m, float b, int dir){
    
    /* dir - UP MyPos.x or y < Target.x or y
           - DOWN MyPos.x or y > Target.x or y*/
    Point SubTarget;

    if(UP == dir){
        if(0==flag){
            SubTarget.x = MyPos.x + D_SCALE;
            SubTarget.y = m * SubTarget.x + b;
        }
        else{
            //printf("%d---- m real\n\r", (int)(m*100000000));

            if(!m){
                SubTarget.y = MyPos.y + D_SCALE;
                SubTarget.x = MyPos.x;            
            }
            else{
                SubTarget.y = MyPos.y + D_SCALE;
                SubTarget.x = 1/m * SubTarget.y - b/m;
            }
        }
    }
    else{ 
        if(0==flag){
            SubTarget.x = MyPos.x - D_SCALE;
            SubTarget.y = m * SubTarget.x + b;
        }
        else{
            //printf("%d---- m real\n\r", (int)(m*100000000));

            if(!m){
                SubTarget.y = MyPos.y - D_SCALE;
                SubTarget.x = MyPos.x;            
            }
            else{
                SubTarget.y = MyPos.y - D_SCALE;
                SubTarget.x = 1/m * SubTarget.y - b/m;
            }
        }
    }

    InverseKinematics(Origin, SubTarget, &SubTarget.theta1, &SubTarget.theta2);

    return SubTarget;

}

void Calc_move(Point SubTarget ){
    /*
    * Preparado para utizar x ou y para controlar o comprimento das mini-retas:
    *   - flag = 0 -> controla em x;
    *   - flag = 1 -> controla em y;
    * 
    * Calcula o Target para a semi-reta (SubTarget);
    * Decide a direção de cada motor;
    * Calcula o numero de steps
    * Através do nº de steps obtem o t_between steps para cada motor
    * Utiliza o nºde steps efetuados para calcular a posição real
    * Atualiza a posiçãp para a real após o movimento
    * 
    */
    double theta1,theta2;
    
    StepMotor fake_m1 = {};
    StepMotor fake_m2 = {};

    ChooseDir(&fake_m1.dir, MyPos.theta1, SubTarget.theta1);
    ChooseDir(&fake_m2.dir, MyPos.theta2, SubTarget.theta2);

    /*Calculate the number of steps*/
    theta1 = MyPos.theta1 - SubTarget.theta1;
    theta2 = MyPos.theta2 - SubTarget.theta2;

    fake_m1.step = abs((int) (theta1 / 0.1125+0.5));
    fake_m2.step = abs((int) (theta2 / 0.1125+0.5));

    CalcTime_between_step(fake_m1.step, fake_m2.step, &fake_m1.t_between_step, &fake_m2.t_between_step);

    /*Updates MyPos*/

    MyPos.theta1 = fake_m1.step * DEGREE_PER_STEP * fake_m1.dir + MyPos.theta1;
    MyPos.theta2 = fake_m2.step * DEGREE_PER_STEP * fake_m2.dir + MyPos.theta2;

    DirectKinematics(MyPos.theta1, MyPos.theta2, &MyPos.x, &MyPos.y);

    /*Atualizaçao os motores*/

    /*Certerficar-me que não fazem nada enquanto os atualizo*/
    motor1.t_until_nmove = 10000;
    motor1.move = FALSE;

    motor2.t_until_nmove = 10000;
    motor2.move = FALSE;

    /* Atribui os valores pretendidos*/
    motor1.dir = fake_m1.dir;
    motor1.t_between_step = fake_m1.t_between_step;
    motor1.step = fake_m1.step;

    motor2.dir = fake_m2.dir;
    motor2.t_between_step = fake_m2.t_between_step;
    motor2.step = fake_m2.step;
    
    /*Inicia o movimento*/

    motor1.t_until_nmove = 1;
    motor2.t_until_nmove = 1;

    #ifdef DEBUG
        //printf(" -- SubTarget.theta1: %d SubTarget.theta2 %d SubTarget.x %d SubTarget.y %d-- \n\r" , (int)(SubTarget.theta1*1000), (int)(1000*SubTarget.theta2), (int)(1000*SubTarget.x),  (int)(1000*SubTarget.y));
        //printf(" --- dir1 %d dir2 %d---- ", fake_m1.dir, fake_m2.dir );
        printf(" -- Angulos a mover %d %d --", (int)(theta1*1000), (int)(theta2*1000));
        printf(" -- Step1 %d  Step2 %d --", fake_m1.step, fake_m2.step);
        //printf(" -- fake_m1.t_between_step %d -- fake_m2.t_between_step %d -- \n\r", fake_m1.t_between_step, fake_m2.t_between_step);
        printf(" -- MyPos.theta1: %d MyPos.theta2 %d MyPos.x %d MyPos.y %d-- \n\r" , (int)(MyPos.theta1*1000), (int)(1000*MyPos.theta2), (int)(1000*MyPos.x),  (int)(1000*MyPos.y));
        printf(" -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- \n\r");
    #endif
}
void Make_move(Point TargetPos, int mode){
    puts("*");
    /*
    * Calcula a reta que liga os pontos
    * Decide o eixo que controla a variação
    * Utiliza um ciclo que calcula o proximo movimento:
    *   - Sempre que ambos os motores terminarem o seu moviemento
    *   - Enquanto nao atingir a posição final
    */
    Point SubTarget;
    float m,b; /*Used to make the equation of a straight line*/  
    int flag=-1; 
    static int8_t oldmode=99;
    
    if(oldmode != mode){
        if(mode==WRITE) {
            desce();
            _delay_ms(100);
        } else {
            sobe();
            _delay_ms(100);
        }
    } 
    oldmode=mode;
    printf("%d\n\r", oldmode);
    /*Decide se divide a reta em x ou em y*/

    if(  abs( TargetPos.x - MyPos.x ) >= abs( TargetPos.y - MyPos.y) ){
        flag = FALSE; /* Dividir em x -> y = f(x)*/
        //printf("--Equação em y = f(x)--\n\r");
    }
    else{
        flag = TRUE; /* Dividir em y -> x = F(y)*/
        //printf("--Equação em x = f(y)--\n\r");
    }
    
    EquationOfaStraightLine(MyPos,TargetPos,&m,&b);
    //printf("m : %d , b : %d", (int)(m*1000), (int)(b*1000));
    

    while( ( (MyPos.x < TargetPos.x) && (0==flag) ) || ( (MyPos.y < TargetPos.y) && (1==flag) ) ){

        if((0==motor1.step) && (0==motor2.step)){
            //printf("FIRST WHILE\n\r");
            SubTarget = CalcSubTarget(flag,m,b,UP);
            Calc_move(SubTarget);
        }
        else{
            //printf("-- Motor1 steps left %d Motor2 steps left %d --\n\r", motor1.step, motor2.step);
        }
    }
    while( ( (MyPos.x > TargetPos.x) && (0==flag) ) || ( (MyPos.y > TargetPos.y) && (1==flag) ) ){
        
        if((0==motor1.step) && (0==motor2.step)){
            
            //printf("SECOND WHILE\n\r");
            SubTarget = CalcSubTarget(flag,m,b,DOWN);
            Calc_move(SubTarget);
        }
        else{
            //printf("-- Motor1 steps left %d Motor2 steps left %d --\n\r", motor1.step, motor2.step);
        }   
    }
    

}int main(void){
    int mode=WALK, word=0, count_word=0, state=INTERFACE, nstate=INTERFACE;
    char input, input_word, f_input, fa_input;
    //float word[50][50];
    hw_init();
    //usart_init();
    uart_init();
    //printf_init();
    tc0_init();
    tc1_init();
    printf("\n\n\n\rScara arm starting (ง ͡ʘ ͜ʖ ͡ʘ)ง !\n\n\n\r");

    
    if(eeprom_read_byte(&signature) != SIG8){
    
        puts("EnTrAnDo Em MoDo CoNfIgUrAcAo");
        puts("\tComprimento Braço (0 a 99 cm)");
        f_input=(getchar()-'0')*10 + getchar()-'0'; //because we are getting a number

        puts("\tComprimento Antebraco (0 a 99 cm)");
        fa_input=(getchar()-'0')*10 + getchar()-'0'; //again , getting a number
        eeprom_update_byte(&ADD_FOREARM, fa_input);
        eeprom_update_byte(&ADD_ARM,f_input);
        eeprom_update_byte(&signature,SIG8);
    }
    
    ARM = eeprom_read_byte(&ADD_FOREARM);
    FOREARM = eeprom_read_byte(&ADD_ARM);
    //printf("%d %d", ARM, FOREARM);
    /*POsition inicialization*/
    sobe();
    _delay_ms(1000);

    motor1.step=NSTEP1;
    motor1.t_between_step=1;
    motor1.t_until_nmove=1;
    motor1.dir=COUNTERCLOCKWISE;
    motor2.step=NSTEP2;
    motor2.t_between_step=1;
    motor2.t_until_nmove=1;
    motor2.dir=COUNTERCLOCKWISE;
    while (motor1.step || motor2.step); //printf("%d %d\n", motor1.step, motor2.step);

    MyPos.x = 20; //check if it is right
    MyPos.y =0;

    /*
    desce();
    _delay_ms(1000);
*/

    InverseKinematics(Origin, MyPos, &MyPos.theta1, &MyPos.theta2);
    //Point TargetPos
    //int x;
    /* TESTE DIRECTKINEMATICS
    DirectKinematics(MyPos.theta1,MyPos.theta2, &x, &y);
    printf(" %d %d \n\r ", (int)x,(int)y);
    */
    //printf("%d x %d y %d theta1 %d theta2", (int)(MyPos.x*1000), (int)(1000*MyPos.y), (int)(MyPos.theta1*1000), (int)(MyPos.theta2*1000));

    /*
    Point TargetPos = {10,15};
    InverseKinematics(Origin, TargetPos, &TargetPos.theta1, &TargetPos.theta2);
    Make_move(TargetPos, mode);
    */

    while(1){
        switch(state){
            case INTERFACE: {
                puts("Language?");
                puts("Portuguese (p) / japanese (j)");
                input=getchar();
                if (input == 'J' || input == 'j'){
                    puts("Yokoso!");
                    puts("Tegami? :)");
                    input_word=getchar();
                    word=22;
                    nstate = HATARAKU;
                } 
                else{
                    puts("Olá!");
                    puts("Qual é a letra, Oh Gordo/a?!");
                    input_word=getchar();
                    nstate = TRABALHAR;  
                
                    switch(input_word){
                        case 'a': {word=0; puts("Afirmativo! Aqui vai um A");} break;
                        case 'b': {word=1; puts("Afirmativo! Aqui vai um B");} break;
                        case 'c': {word=2; puts("Afirmativo! Aqui vai um C");} break;
                        case 'd': {word=3; puts("Afirmativo! Aqui vai um D");} break;
                        case 'e': {word=4; puts("Afirmativo! Aqui vai um E");} break;
                        case 'f': {word=5; puts("Afirmativo! Aqui vai um F");} break;
                        case 'g': {word=6; puts("Afirmativo! Aqui vai um G");} break;
                        case 'h': {word=7; puts("Afirmativo! Aqui vai um H");} break;
                        case 'i': {word=8; puts("Afirmativo! Aqui vai um I");} break;
                        case 'j': {word=9; puts("Afirmativo! Aqui vai um J");} break;
                        case 'k': {word=10; puts("Afirmativo! Aqui vai um K");} break;
                        case 'l': {word=11; puts("Afirmativo! Aqui vai um L");} break;
                        case 'm': {word=12; puts("Afirmativo! Aqui vai um M");} break;
                        case 'n': {word=13; puts("Afirmativo! Aqui vai um N");} break;
                        case 'o': {word=14; puts("Afirmativo! Aqui vai um O");} break;
                        case 'p': {word=15; puts("Afirmativo! Aqui vai um P");} break;
                        case 'q': {word=16; puts("Afirmativo! Aqui vai um Q");} break;
                        case 'r': {word=17; puts("Afirmativo! Aqui vai um R");} break;
                        case 's': {word=18; puts("Afirmativo! Aqui vai um S");} break;
                        case 't': {word=19; puts("Afirmativo! Aqui vai um T");} break;
                        case 'u': {word=20; puts("Afirmativo! Aqui vai um U");} break;
                        case 'v': {word=21; puts("Afirmativo! Aqui vai um V");} break;
                        case 'x': {word=22; puts("Afirmativo! Aqui vai um X");} break;
                        case 'y': {word=23; puts("Afirmativo! Aqui vai um Y");} break;
                        case 'z': {word=24; puts("Afirmativo! Aqui vai um Z");} break;
                        default:
                        puts("NANI?!");
                        word=22; //faz um X :)
                    } 
                }         
            }
            break;
            
            case TRABALHAR:{
                
                Point TargetPos = { (float) repository1[word][WORD_POINT_X]/10.0 , (float) repository1[word][WORD_POINT_Y] /10.0};
                InverseKinematics(Origin, TargetPos, &TargetPos.theta1, &TargetPos.theta2);
                //printf("%d x %d y %d theta1 %d theta2", (int)(TargetPos.x*1000), (int)(1000*TargetPos.y), (int)(TargetPos.theta1*1000), (int)(TargetPos.theta2*1000));
                Make_move(TargetPos, repository1[word][WORD_MODE]);
                count_word+=3;
            
                if(count_word >= repository1[word][DATASIZE]){
                nstate = INTERFACE;
                count_word=0;
                }
            }
            break;
            
            case HATARAKU: {
            
                Point TargetPos = { (float) repository1[word][WORD_POINT_X] /10.0 , (float) repository1[word][WORD_POINT_Y] /10.0};
                InverseKinematics(Origin, TargetPos, &TargetPos.theta1, &TargetPos.theta2);
                Make_move(TargetPos, repository1[word][WORD_MODE]);
                count_word+=3;
                
                if(count_word >= repository1[word][DATASIZE]){
                nstate = INTERFACE;
                count_word=0;
                }
            }
            break;
            
        default:
            state = ERROR;
        } 
        
        state = nstate;
        
    }
  
}
