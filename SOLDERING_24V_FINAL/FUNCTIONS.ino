/////////////////////////////////////////// ФУНКЦИИ ////////////////////////////////////////

//ФУНКЦИЯ СЧИТЫВАЕТ АНАЛОГОВЫЙ ВХОД samples КОЛИЧЕСТВО РАЗ И ВОЗВРАЩАЕТ МЕДИАННОЕ ЗНАЧЕНИЕ
int16_t median(uint8_t samples){
  int16_t raw[samples];//МАССИВ ДЛЯ ХРАНЕНИЯ ДАННЫХ 
  int16_t tem=0; uint8_t i,j;  //ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
for(i=0; i<samples; i++){raw[i]=analogRead(A0);}//ЧИТАЕМ ДАННЫЕ С А0 И ПИШЕМ ИХ В ЯЧЕЙКИ МАССИВА
for(i=0; i<samples; i++){//СОРТИРУЕМ МАССИВ ПО ВОЗРАСТАНИЮ ЗНАЧЕНИЙ В ЯЧЕЙКАХ
  for(j=0; j<samples-1; j++){//АНОМАЛЬНО БОЛЬШИЕ И МАЛЫЕ В НАЧАЛО И КОНЕЦ МАССИВА
    if(raw[j]>raw[j+1]){tem=raw[j]; raw[j]=raw[j+1]; raw[j+1]=tem;}}}
//return raw[samples/2];//ВОЗВРАЩАЕМ ИЗ ФУНКЦИИ ЗНАЧЕНИЕ СРЕДНЕЙ ЯЧЕЙКИ МАССИВА
return raw[samples>>1];//>>1 ВОЗВРАЩАЕМ результат деления на 2 // &0b1 Остаток от деления на 2
}

void clmil(){timer1=mill;f.w1=true;}//ОБНУЛЯЕМ ТАЙМЕР, ПОДНИМАЕМ ФЛАГ

void clful(){myEnc.write(0);oldPos=0;newPos=0;timer1=mill;f.w1=true;}//ОБНУЛЯЕМ ЭНКОДЕР, ТАЙМЕР

void key5(){btn5=digitalRead(5);if(btn5!=oldbtn5){delay(10);btn5=digitalRead(5);
if(btn5==LOW&&oldbtn5==HIGH){timer1=mill;f.w1=true;f.w4=true;f.w6=!f.w6;
if(f.w6==true){ar=uint8_t(8);}else{ar=uint8_t(7);}}oldbtn5=btn5;}}

void digprint1(){
if(graf>-10&&graf<10){lcd.print(' ');}
if(graf>0){lcd.print('+');}if(graf==0){lcd.print(' ');}lcd.print(graf);
}
void digprint2(){
if(graf<10){lcd.print("  ");}else{lcd.print(' ');}lcd.print(graf);
}
void digprint3(){
if(graf<10){lcd.print("  ");}if(graf>=10&&graf<100){lcd.print(' ');}lcd.print(graf);
}
void menu0(){
lcd.setCursor(5,0);
switch(m0){
    case 0:trig=temp0;temp=temp0;lcd.print('1');break;
    case 1:trig=temp1;temp=temp1;lcd.print('2');break;
    case 2:trig=temp2;temp=temp2;lcd.print('3');break;}
}
void menu1(){
uint8_t i;
switch(m1){
    case 0:trig1=wait;break;
    case 1:trig1=down;break;
    case 2:trig1=tempwait;break;
    case 3:for(i=0;i<3;i++){if(sharp==zen[i]){trig1=i;}}break;}
lcd.setCursor(0,0);
if(m1==0){lcd.write(ar);}else{lcd.print(' ');}lcd.print("WAIT");graf=wait;digprint2();
lcd.setCursor(8,0);
if(m1==1){lcd.write(ar);}else{lcd.print(' ');}lcd.print("DOWN");graf=down;digprint2();
lcd.setCursor(0,1);
if(m1==2){lcd.write(ar);}else{lcd.print(' ');}lcd.print("TMPW ");graf=tempwait;digprint3();
lcd.setCursor(10,1);
if(m1==3){lcd.write(ar);}else{lcd.print(' ');}lcd.print("TS");graf=sharp;digprint2();
}
void menu2(){
switch(m2){
    case 0:trig2=KpSet;break;
    case 1:trig2=KiSet;break;
    case 2:trig2=autoret;break;
    case 3:trig2=led;break;}
lcd.setCursor(0,0);
if(m2==0){lcd.write(ar);}else{lcd.print(' ');}lcd.print("Kp=");Kp=((double)KpSet)/10.0;
lcd.print(Kp,1);//graf=KpSet;digprint2();Kp=((double)KpSet)/10.0;
lcd.setCursor(8,0);
if(m2==1){lcd.write(ar);}else{lcd.print(' ');}lcd.print("Ki=");Ki=((double)KiSet)/10.0;
lcd.print(Ki,1);//graf=KiSet;digprint2();Ki=((double)KiSet)/10.0;
lcd.setCursor(0,1);
if(m2==2){lcd.write(ar);}else{lcd.print(' ');}lcd.print("RET");graf=autoret;digprint2();
lcd.setCursor(8,1);
if(m2==3){lcd.write(ar);}else{lcd.print(' ');}lcd.print("LED ");
if(led==1){lcd.print("ON ");}else{lcd.print("OFF");}
}
void bigdigit(){//ФУНКЦИЯ ФОРМИРОВАНИЯ БОЛЬШИХ ЦИФР ИЗ СИМВОЛОВ ПСЕВДОГРАФИКИ
uint8_t a[3]={0,1,2},i,d1,d2,d3,d4,d5,d6,e1,e2,e3;
a[0]=temp/100;
a[1]=temp/10%10;
a[2]=temp%10;
for(i=0;i<3;i++){
switch(i){
  case 0: e1=7, e2=8, e3=9; break;
  case 1: e1=10,e2=11,e3=12;break;
  case 2: e1=13,e2=14,e3=15;break;}
switch(a[i]){
  case 0: d1=1, d2=8, d3=6,   d4=1, d5=3, d6=6;break;
  case 1: d1=32,d2=2, d3=6,   d4=32,d5=32,d6=6;break;
  case 2: d1=2, d2=8, d3=6,   d4=1, d5=4, d6=5;break;
  case 3: d1=2, d2=4, d3=6,   d4=7, d5=3, d6=6;break;
  case 4: d1=1, d2=3, d3=6,   d4=32,d5=32,d6=6;break;
  case 5: d1=1, d2=4, d3=5,   d4=7, d5=3, d6=6;break;
  case 6: d1=1, d2=4, d3=5,   d4=1, d5=3, d6=6;break;
  case 7: d1=2, d2=8, d3=6,   d4=32,d5=2, d6=6;break;
  case 8: d1=1, d2=4, d3=6,   d4=1, d5=3, d6=6;break;
  case 9: d1=1, d2=4, d3=6,   d4=7, d5=3, d6=6;break;}
lcd.setCursor(e1,0);lcd.write(d1);
lcd.setCursor(e2,0);lcd.write(d2);
lcd.setCursor(e3,0);lcd.write(d3);
lcd.setCursor(e1,1);lcd.write(d4);
lcd.setCursor(e2,1);lcd.write(d5);
lcd.setCursor(e3,1);lcd.write(d6);}
}

void bigtime(){//ФУНКЦИЯ ФОРМИРОВАНИЯ БОЛЬШИХ ЧАСОВ ИЗ СИМВОЛОВ ПСЕВДОГРАФИКИ
uint8_t a[4]={0,1,2,3},i,d1,d2,d3,d4,d5,d6,e1,e2,e3;
wt=rtc.getTime();
a[0]=wt.hour/10;//inputdata/10;//
a[1]=wt.hour%10;//inputdata%10;//
a[2]=wt.min/10;//inputdata/10;//
a[3]=wt.min%10;//inputdata%10;//
for(i=0;i<4;i++){
switch(i){
  case 0: e1=0, e2=1, e3=2; break;
  case 1: e1=3, e2=4, e3=5; break;
  case 2: e1=7, e2=8, e3=9; break;
  case 3: e1=10,e2=11,e3=12;break;}
switch(a[i]){
  case 0: d1=1, d2=8, d3=6,   d4=1, d5=3, d6=6;break;
  case 1: d1=32,d2=2, d3=6,   d4=32,d5=32,d6=6;break;
  case 2: d1=2, d2=8, d3=6,   d4=1, d5=4, d6=5;break;
  case 3: d1=2, d2=4, d3=6,   d4=7, d5=3, d6=6;break;
  case 4: d1=1, d2=3, d3=6,   d4=32,d5=32,d6=6;break;
  case 5: d1=1, d2=4, d3=5,   d4=7, d5=3, d6=6;break;
  case 6: d1=1, d2=4, d3=5,   d4=1, d5=3, d6=6;break;
  case 7: d1=2, d2=8, d3=6,   d4=32,d5=2, d6=6;break;
  case 8: d1=1, d2=4, d3=6,   d4=1, d5=3, d6=6;break;
  case 9: d1=1, d2=4, d3=6,   d4=7, d5=3, d6=6;break;}
lcd.setCursor(e1,0);lcd.write(d1);
lcd.setCursor(e2,0);lcd.write(d2);
lcd.setCursor(e3,0);lcd.write(d3);
lcd.setCursor(e1,1);lcd.write(d4);
lcd.setCursor(e2,1);lcd.write(d5);
lcd.setCursor(e3,1);lcd.write(d6);}
//lcd.setCursor(13,0);lcd.print(rtc.getTemp(),0);lcd.print((char)223);
lcd.setCursor(14,1);if(wt.sec<10){lcd.print('0');}lcd.print(wt.sec);
}

void bigdigitres(){//СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ БОЛЬШИХ ЦИФР В ГЛАВНОМ МЕНЮ
byte v1[8]={ 7, 7, 7, 7, 7, 7, 7, 7};
byte v2[8]={ 7, 7, 0, 0, 0, 0, 0, 0};
byte v3[8]={ 0, 0, 0, 0, 0, 0,31,31};
byte v4[8]={31,31, 0, 0, 0, 0,31,31};
byte v5[8]={28,28, 0, 0, 0, 0,28,28};
byte v6[8]={28,28,28,28,28,28,28,28};
byte v7[8]={ 0, 0, 0, 0, 0, 0, 7, 7};
byte v8[8]={31,31, 0, 0, 0, 0, 0, 0};
lcd.createChar(1,v1);lcd.createChar(2,v2);
lcd.createChar(3,v3);lcd.createChar(4,v4);
lcd.createChar(5,v5);lcd.createChar(6,v6);
lcd.createChar(7,v7);lcd.createChar(8,v8);
}
void arrow0(){//ПЕРЕЗАПИСЫВАЕМ СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ БОЛЬШИХ ЦИФР В ГЛАВНОМ МЕНЮ
byte v7[8]={ 0, 0, 0, 0, 0, 0, 7, 7};
byte v8[8]={31,31, 0, 0, 0, 0, 0, 0};
lcd.createChar(7,v7);lcd.createChar(8,v8);}

void arrow1(){
byte v7[8]={24,28,30,31,30,28,24,0};//ТРЕУГОЛЬНИК УКАЗАТЕЛЬ ДЛЯ МЕНЮ 1 И 2
byte v8[8]={24,28,14, 7,14,28,24,0};//СТРЕЛОЧКА УКАЗАТЕЛЬ ДЛЯ МЕНЮ 1 И 2
lcd.createChar(7,v7);lcd.createChar(8,v8);}


void set_time(){wt=rtc.getTime();//ЧИТАЕМ ТЕКУЩЕЕ ЗНАЧЕНИЕ ВРЕМЕНИ И ДАТЫ
sethour=wt.hour;setmin=wt.min;setsec=wt.sec;\
setdate=wt.date;setmon=wt.mon;setyear=(wt.year-2000);setdow=wt.dow;}

void time_set(){//ЗАПИСЫВАЕМ ТЕКУЩЕЕ ЗНАЧЕНИЕ ВРЕМЕНИ И ДАТЫ
rtc.setTime(sethour,setmin,setsec);rtc.setDate(setdate,setmon,(setyear+2000));rtc.setDOW(setdow);}




// пример диммера на симисторе с внешним детектором нуля
// или готовый китайский модуль https://ali.ski/xwpCCe
// одноканальный диммер без таймера, работает на пропуске
// полуволн синусоиды по прерываниям с детектора нуля
// подходит для моторов и обогревателей, лампы мерцают жутко!
// Алгоритм Брезенхема https://habr.com/ru/post/254719/
// диммер принимает значения 0-255
// переключения оптимизированы

//#define INT_NUM 0     // соответствующий ему номер прерывания

//int Out;// ЦЕЛОЧИСЛЕННАЯ ПЕРЕМЕННАЯ
//void setup() {
//pinMode(2, INPUT_PULLUP);// пин детектора нуля
//pinMode(11, OUTPUT);// управляющий пин симистора
//digitalWrite(11, LOW);
//attachInterrupt(0, isr, RISING);// для самодельной схемы RISING для китайского модуля FALLING

//void loop() {
  
//Out=map(analogRead(A0), 0,1023, 0,255);// УСТАНАВЛИВАЕМ ЗНАЧЕНИЕ Output В ПРЕДЕЛАХ 0-255

// y=x++; тоже самое, что y=x;x=x+1;\
   y=++x; тоже самое, что y=(x+1);\
   y+=x;  тоже самое, что y=y+x;\
// ++ (плюс плюс) инкремент: a++ равносильно a = a + 1
// — (минус минус) декремент: a — равносильно a = a – 1
// += составное сложение: a += 10 равносильно a = a + 10
// -= составное вычитание: a -= 10 равносильно a = a – 10
// *= составное умножение: a *= 10 равносильно a = a * 10
// /= составное деление: a /= 10 равносильно a = a / 10
// %= прибавить остаток от деления: a %= 10 равносильно a = a + a % 10
// &= составное битовое И: a &= b равносильно a = a & b
// ^= составное исключающее ИЛИ: a ^= b равносильно a = a ^ b
// |= составное ИЛИ: a |= b равносильно a = a | b



//void isr(){// ФУНКЦИЯ ОБРАБОТКИ ПРЕРЫВАНИЯ ОТСЛЕЖИВАЕТ ПЕРЕХОД ЧЕРЕЗ "0" И \
  static byte count,last,lastVal;\
  int val=(++count * Out) >> 8;// РЕГИСТРОВЫЙ СДВИГ x >> 8 = x /(2^8) = x / 256\
if(lastVal!=(val!=last)){digitalWrite(11, val!=last);}\
lastVal=(val!=last);last=val;}
/*
Пример:

Serial.print(78) передается как "78"
Serial.print(1.23456) передается как "1.23"
Serial.print(byte(78)) передается как "N" (т.к. в таблице ASCII "N" под 78 номером)
Serial.print('N') передается как "N"
Serial.print("Hello world.") передается как "Hello world."
С помощью второго опционально параметра можно задать базис (систему счисления) для чисел. 
Допустимые значения BYTE, BIN (двоичный), OCT (восьмиричный), DEC (десятеричный), 
HEX (шестнадцатеричный). Для вещественных (дробных) чисел второй параметр задает количество 
знаков после запятой. Пример:

Serial.print(78, BYTE) выводит "N"
Serial.print(78, BIN) выводит  "1001110"
Serial.print(78, OCT) выводит "116"
Serial.print(78, DEC) выводит "78"
Serial.print(78, HEX) выводит "4E"
Serial.println(1.23456, 0) выводит "1"
Serial.println(1.23456, 2) выводит "1.23"
Serial.println(1.23456, 4) выводит "1.2346"
*/
