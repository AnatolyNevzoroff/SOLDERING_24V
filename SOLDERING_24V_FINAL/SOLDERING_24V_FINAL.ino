/////////////////////////////////// ПАЯЛЬНИК 24V ВЕРСИЯ 1.22 //////////////////////////////////////
///////////////////// СКЕТЧ АНАТОЛИЯ НЕВЗОРОВА // CODE BY ANATOLY NEVZOROFF ///////////////////////
  
#define p60000 60000L
#define p1000 1000L//ИНТЕРВАЛ 1000 mS (1 СЕКУНДА), ТИП ДАННЫХ "L" (long)
#define p500 500L
#define p400 400L
#define p100 100L
#define p50 50L
#define ledOFF lcd.noBacklight()
#define ledON lcd.backlight()
#define hot 54//БЕЗОПАСНАЯ ТЕМПЕРАТУРА ДЛЯ СМЕНЫ ЖАЛА
//#define SERIAL_PORT_ENABLE //ЗАКОМЕНТИРОВАТЬ ДЛЯ ОТКЛЮЧЕНИЯ СЕРИЙНОГО ПОРТА И ВЫВОДА ДАННЫХ 

/////////////////////////////////// ПОДКЛЮЧАЕМЫЕ БИБЛИОТЕКИ ///////////////////////////////////////
#include <EEPROM.h>//ЧТЕНИЕ И ЗАПИСЬ ПЕРЕМЕННЫХ В ЭНЕРГОНЕЗАВИСИМУЮ ПАМЯТЬ EEPROM
#include <Encoder.h>//ОБРАБОТКА УГЛА ПОВОРОТА ЭНКОДЕРА
#include <Wire.h>//ОБЕСПЕЧЕНИЕ ПЕРЕДАЧИ ДАННЫХ ПО ШИНЕ I2C (IIC) (ЭКРАН И ЧАСЫ)
#include <LiquidCrystal_I2C.h>//ПЕЧАТЬ СИМВОЛОВ НА ЖИДКОКРИСТАЛИЧЕСКОМ ЭКРАНЕ ЧЕРЕЗ I2C
#include <PID_v1.h>//ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОГО ДИФФЕРЕНЦИРОВАНИЯ ДЛЯ ИНЕРТНЫХ СИСТЕМ
#include <DS3231.h>//МОДУЛЬ ЧАСОВ (RTC)

LiquidCrystal_I2C lcd(0x27,16,2);//УСТАНАВЛИВАЕМ АДРЕС, ЧИСЛО СИМВОЛОВ, СТРОК У ЖКИ ЭКРАНА
Encoder myEnc(3,4);// DT и CLK ИЛИ S1 и S2 ВЫВОДЫ ЭНКОДЕРА
DS3231 rtc(SDA,SCL);Time wt;//ПОДКЛЮЧАЕМ МОДУЛЬ ЧАСОВ; ИНИЦИАЛИЗИРУЕМ ОБЪЕКТ wt ДЛЯ КЛАССА ЧАСОВ

/////////////////////////////////// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ /////////////////////////////////////////

//unsigned long от 0 до 4294967295
uint32_t mill,timer1,timer2,timer3,timer4,timer5;//ТАЙМЕРЫ ДЛЯ ФУНКЦИИ millis()

//int от -32768 до 32767  
int16_t pwm,inputdata,temp,temp0,temp1,temp2,tempwait,trig,graf;//
int16_t freq[4]={1370,1170,970,870};//МАССИВ ДЛЯ ФУНКЦИИ tone()

//byte от 0 до 255
uint8_t temp00,temp01,temp10,temp11,temp20,temp21,ar,sharp,zen[3]={1,5,10};
//uint8_t w0,w1,w2=1,w3,w4,w5,w6,w7,w8,w9;
uint8_t led,str,KpSet,KiSet;//hot=54,
uint8_t poz[8]={0,3,6,14,0,3,8,15};//МАССИВ ДЛЯ ПОЗИЦИОНИРОВАНИЯ КУРСОРА В МЕНЮ УСТАНОВКИ ВРЕМЕНИ

//char от -128 до 127
int8_t fr,m0,m1,om1,m2,m3,menu,wait,down,autoret,trig1,trig2,trig3,loc,oldPos,newPos;
int8_t setime,sethour,setmin,setsec,setdate,setmon,setyear,setdow;

bool flag,btn5,oldbtn5,btn6,oldbtn6,btn7,oldbtn7,save,power;//=true

struct Flags{// СТРУКТУРА ДЛЯ ЭКОНОМИИ ПАМЯТИ НА ФЛАГАХ
  bool w0: 1;// ОТРАБОТКА ПРИ НАЖАТИИ НА КНОПКУ, ПОВОРОТЕ ЭНКОДЕРА, СМЕЩЕНИИ РТУТНОГО ДАТЧИКА
  bool w1: 1;// ВЫВОД НА ЭКРАН ДАННЫХ
  bool w2: 1;// ЗАГРУЗКА СОСТОЯНИЙ ПЕРЕМЕННЫХ ПРИ ВКЛЮЧЕНИИ / ОТКЮЧЕНИИ ПИТАНИЯ
  bool w3: 1;// ПЕРЕКЛЮЧАТЕЛЬ ПОДСВЕТКИ ЭКРАНА
  bool w4: 1;// ПЕРЕКЛЮЧЕНИЕ ФУНКЦИЙ ПРИ НАЖАТИИ ЭНКОДЕРА
  bool w5: 1;// ОДНОКРАТНАЯ ЗАГРУЗКА ПЕРЕМЕННЫХ ПРИ ПЕРЕХОДЕ МЕЖДУ МЕНЮ
  bool w6: 1;// ПЕРЕКЛЮЧЕНИЕ ВИДА УКАЗАТЕЛЯ И ФУНКЦИЙ ПРИ НАЖАТИИ И ПОВОРОТЕ ЭНКОДЕРА
  bool w7: 1;// ВКЛЮЧЕНИЕ ВТОРОГО ОТСЧЁТА ДЛЯ ОТКЛЮЧЕНИЯ СИЛОВОЙ ЧАСТИ ПАЯЛЬНИКА
  bool w8: 1;// КОНТРОЛЬ СОСТОЯНИЯ СВЕТОДИОДА "POWER" В ТЕЧЕНИЕ timer
};Flags f;

double Setpoint;//ТРЕБУЕМАЯ ТЕМПЕРАТУРА
double Input;   //ДАННЫЕ О ТЕКУЩЕЙ ТЕМПЕРАТУРЕ
double Output;  //ШИМ СИГНАЛ С ЦИФРОВОГО ВЫХОДА
double Kp;      //АГРЕССИВНОСТЬ НАБОРА ТЕМПЕРАТУРЫ Setpoint, ЧЕМ БОЛЬШЕ ТЕМ БЫСТРЕЕ 3.8
double Ki;      //"ПРЕДСКАЗЫВАЕТ" ЗНАЧЕНИЕ Input, ЧЕМ БОЛЬШЕ ТЕМ ИНЕРЦИОННЕЕ 0.4
double Kd=0;    //УСРЕДНЯЕТ, СГЛАЖИВАЕТ Output ПРЕДОТВРАЩАЕТ КОЛЕБАНИЯ
PID myPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);

//УКАЗЫВАЕМ ВСЕ ФУНКЦИИ ДЛЯ БОЛЕЕ БЫСТРОЙ КОМПИЛЯЦИИ, ДЛЯ ВЛАДЕЛЬЦЕВ МОЩНЫХ ПК НЕАКТУАЛЬНО
void menu0();void menu1();void menu2();void digprint1();void digprint2();void digprint3();
void key5();void bigdigit();void bigdigit();void bigdigitres();void arrow0();void arrow1();
void clmil();void clful();int16_t median(uint8_t samples);void bigtime();void set_time();
void time_set();//void key_MENU();

//////////////////////////////////////////// SETUP ////////////////////////////////////////////////
void setup(){

// ПЕРЕВОДИМ ПИНЫ D9 и D10 ТАЙМЕРА №1 НА ЧАСТОТУ - 31.4 кГц 
TCCR1A = 0b00000001;//8bit
TCCR1B = 0b00000001;//x1 phase correct
// ПЕРЕВОДИМ ПИНЫ D9 и D10 ТАЙМЕРА №1 НА ЧАСТОТУ - 30 Гц (как возможный вариант)\
TCCR1A = 0b00000001;//8bit\
TCCR1B = 0b00000101;//x1024 phase correct

Wire.begin();lcd.init();myPID.SetMode(AUTOMATIC);myPID.SetOutputLimits(0,255);rtc.begin();

#ifdef SERIAL_PORT_ENABLE 
Serial.begin(9600); 
#endif

pinMode(3,INPUT);       //ПОВОРОТ ЭНКОДЕРА ВЛЕВО
pinMode(4,INPUT);       //ПОВОРОТ ЭНКОДЕРА ВПРАВО
pinMode(5,INPUT);       //КНОПКА ПОДМЕНЮ SUBMENU KEY (Кнопка SW энкодера)
pinMode(6,INPUT_PULLUP);//КНОПКА ВКЛЮЧЕНИЯ POWER/STANDBY KEY
pinMode(7,INPUT_PULLUP);//ВХОД ДЛЯ РТУТНОГО ДАТЧИКА
//pinMode(8,OUTPUT);      
//pinMode(9,OUTPUT);
pinMode(10,OUTPUT);     //ВЫХОД ДЛЯ ШИМ СИГНАЛА / LED INDICATOR (Ораньжевый светодиод)
pinMode(11,OUTPUT);     //ВЫХОД ДЛЯ ПЬЕЗОПИЩАЛКИ (tone() РАБОТАЕТ НА ПИНАХ 3 И 11 ВТОРОГО ТАЙМЕРА)
pinMode(12,OUTPUT);     //ВЫХОД ДЛЯ ИНДИКАТОРА ВКЛЮЧЕНИЯ POWER LED ON/OFF (Красный светодиод)
pinMode(13,OUTPUT);     //ВЫХОД ДЛЯ БЛОКА ПИТАНИЯ POWER RELAY ON/OFF ВКЛЮЧЕНО - LOW
digitalWrite(13,HIGH);  //ОТКЛЮЧАЕМ ПИТАНИЕ БП НА 24V или PORTB|=1<<5;
analogReference(DEFAULT);//ВКЛЮЧАЕМ ВНУТРЕННЕЕ ОПОРНОЕ НАПРЯЖЕНИЕ 5,0 ВОЛЬТ (для 1,1 В INTERNAL)

////////////////////////////////// ЧИТАЕМ ПЕРЕМЕННЫЕ ИЗ EEPROM ////////////////////////////////////
//for(uint8_t Rd=0;Rd<20;Rd++){EEPROM.update(Rd,0);}//ОЧИСТКА ПАМЯТИ ДЛЯ САМОЙ ПЕРВОЙ ЗАГРУЗКИ
temp00=EEPROM.read(0);temp01=EEPROM.read(1);temp0=word(temp00,temp01);
temp10=EEPROM.read(2);temp11=EEPROM.read(3);temp1=word(temp10,temp11);
temp20=EEPROM.read(4);temp21=EEPROM.read(5);temp2=word(temp20,temp21);
tempwait=EEPROM.read(6);led=EEPROM.read(14);KpSet=EEPROM.read(7);KiSet=EEPROM.read(8);
sharp=EEPROM.read(10);wait=EEPROM.read(11);down=EEPROM.read(12);autoret=EEPROM.read(13);
bigdigitres();//ЗАГРУЖАЕМ СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ БОЛЬШИХ ЦИФР
f.w2=true;//ДЛЯ НАЧАЛЬНОЙ РАЗОВОЙ ЗАГРУЗКИ ЗНАЧЕНИЙ В РЕЖИМЕ "POWER OFF"
//if(autoret<10){autoret=15;}//ДЛЯ САМОЙ ПЕРВОЙ ЗАГРУЗКИ ИНАЧЕ АВТОВЫХОД ИЗ МЕНЮ БУДЕТ МГНОВЕННЫМ
Kp=(double)KpSet/10.0;Ki=(double)KiSet/10.0;//ЗАДАЁМ КОЭФФИЦИЕНТЫ ПИД ПОСЛЕ ОТКЛЮЧЕНИЯ ПИТАНИЯ

}////////////////////////////////////////////// END SETUP /////////////////////////////////////////

void loop(){

// ЧТЕНИЕ ПОЛОЖЕНИЯ ЭНКОДЕРА И ПЕРЕЗАПИСЬ ПЕРЕМЕННОЙ mill
newPos=myEnc.read()/4;mill=millis();

///////////// КНОПКА ВКЛЮЧЕНИЯ POWER ON/OFF ЛОГИЧЕСКИЙ "0" НАЖАТА "1" НЕ НАЖАТА ///////////////
//ХОРОШИЙ АЛГОРИТМ, НО ИСПОЛЬЗУЕТ delay(40)ПОТЕРЯ 40 МИЛЛИСЕКУНД
btn6=digitalRead(6);if(btn6!=oldbtn6){delay(40);btn6=digitalRead(6);\
if(!btn6&&oldbtn6){power=!power;f.w2=true;clmil();}oldbtn6=btn6;}
//ХОРОШИЙ АЛГОРИТМ, НО ТРЕБУЕТ 5! ПЕРЕМЕННЫХ\
btn6=digitalRead(6);if(btn6!=oldbtn6){timer4=mill+p50;flag=true;oldbtn6=btn6;}\
if(flag==true&&mill>=timer4){flag=false;btn6=digitalRead(6);\
if(btn6==LOW){power=!power;f.w2=1;clmil();}}
//КОМПРОМИСНЫЙ, НО НЕ ЛУЧШИЙ ВАРИАНТ\
if(flag==false&&btn6!=digitalRead(6)){timer4=millis()+p50;flag=true;}\
if(flag==true&&millis()>=timer4){flag=false;btn6=digitalRead(6);\
if(btn6==LOW){power=!power;f.w2=1;clmil();}}

//////////////////////////////////////////// POWER ON ////////////////////////////////////////////
if(power==true){
//ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER ON"
if(f.w2==true){menu=0;f.w0=true;f.w1=true;f.w2=false;f.w4=false;f.w5=false;f.w7=false;fr=3;
timer3=mill;ledON;lcd.noCursor();lcd.noBlink();digitalWrite(12,HIGH);digitalWrite(13,LOW);
lcd.clear();}

//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ ВКЛЮЧЕНИЯ БЛОКА ПИТАНИЯ 24В ПАЯЛЬНИКА 
if(fr>-1&&mill>timer3){timer3=mill+p100;tone(11,freq[fr],90);fr--;}

///////////////////////////////////////// ЯДРО ПАЯЛЬНИКА /////////////////////////////////////////
inputdata=median(15);//ЧИТАЕМ ОТФИЛЬТРОВАННЫЕ СРЕДНЕ-МЕДИАННЫЕ ЗНАЧЕНИЯ С ПОРТА "А0"

//МИНИМАЛЬНАЯ ЗАЩИТА ОТ ПЕРЕГРЕВА, ТЕРМОРЕЗИСТОР ПОВРЕЖДЁН, ОБРЫВ КАБЕЛЯ, ПЛОХОЙ КОНТАКТ И Т.Д.
//ПРОВЕРЯЕМ ЗНАЧЕНИЯ С ПОРТА "А0" НА ЗАВЕДОМО ЗАВЫШЕННЫЕ
if(inputdata>1022){digitalWrite(13,HIGH);lcd.clear();
lcd.setCursor(0,0);lcd.print(F("SOLDERING IRON"));lcd.setCursor(5,1);lcd.print(F("ERROR!"));
delay(2000);power=false;f.w2=true;}

Input=map(inputdata,0,1024,20,350);//ПЕРЕВОДИМ ПОЛУЧЕННЫЕ С "А0" ЗНАЧЕНИЯ В ТЕМПЕРАТУРУ
Setpoint=temp;//ОТСЫЛАЕМ ЗАДАННУЮ ТЕМПЕРАТУРУ В ПИД
myPID.SetTunings(Kp,Ki,Kd);//УСТАНАВЛИВАЕМ КОЭФФИЦИЕНТЫ ПИД
#ifdef SERIAL_PORT_ENABLE 
Serial.print("Kp ");Serial.print(Kp);Serial.print(" ");Serial.print("KpSet ");Serial.println(KpSet);
Serial.print("Ki ");Serial.print(Ki);Serial.print(" ");Serial.print("KiSet ");Serial.println(KiSet);
#endif
myPID.Compute();//ПРОПОРЦИОНАЛЬНО-ИНТЕГРАЛЬНОЕ ДИФФЕРЕНЦИРОВАНИЕ ТЕМПЕРАТУРЫ (ПИД)
analogWrite(10,Output);//РЕЗУЛЬТАТ ПИД ПРЕОБРАЗУЕМ В ШИМ

//КОНТРОЛЬ ДАТЧИКА ПОЛОЖЕНИЯ РУЧКИ ПАЯЛЬНИКА
btn7=digitalRead(7);if(btn7!=oldbtn7){clmil();f.w0=true;oldbtn7=btn7;}

//ВХОД В МЕНЮ 1 ИЛИ 2 
if(f.w4==true&&oldbtn5==LOW&&mill-timer1>p400){timer1=mill;++menu;
if(menu>2){menu=0;}f.w1=true;f.w4=false;f.w5=true;lcd.clear();}

/////////////////////////////////////////// ГЛАВНОЕ МЕНЮ /////////////////////////////////////////
if(menu==0){if(f.w5==true){f.w5=false;arrow0();clmil();}

btn5=digitalRead(5);if(btn5!=oldbtn5){delay(10);btn5=digitalRead(5);
if(btn5==LOW&&oldbtn5==HIGH){timer1=mill;++m0;if(m0>2){m0=0;}f.w0=true;f.w4=true;}oldbtn5=btn5;}
//ЗАДЕРЖКА КОМАНДЫ СМЕНЫ ТЕМПЕРАТУРЫ ДЛЯ ВОЗМОЖНОСТИ ПЕРЕПРЫГНУТЬ ЧЕРЕЗ ЗНАЧЕНИЕ, ГЛАВНОЕ НЕ \
ПРЕВЫШАТЬ 500 МС МЕЖДУ ОДИНОЧНЫМИ НАЖАТИЯМИ :-)) ИЛИ ЕСЛИ НЕ ОТПУСКАТЬ КНОПКУ 400 МС. \
(ЧТО НАСТУПАЕТ РАНЬШЕ НА 100 МС) КОМАНДА НЕ УСПЕВАЕТ ПОМЕНЯТЬ ТЕМПЕРАТУРУ И ПРОИСХОДИТ ПЕРЕХОД В\
СЛЕДУЮЩЕЕ МЕНЮ, А ПЕРЕХОД НА СЛЕДУЮЩЕЕ ЗНАЧЕНИЕ ТЕМПЕРАТУРЫ МИНУСУЕТСЯ В ПЕРВОМ МЕНЮ.
if(f.w4==true&&mill-timer1>p500){f.w4=false;clmil();}

//МЕНЯЕМ ТЕМПЕРАТУРУ В ЗАВИСИМОСТИ ОТ ВЫБРАННОГО ЗНАЧЕНИЯ Т1, Т2, Т3
if(newPos!=oldPos){trig=trig+(newPos*sharp);trig=constrain(trig,150,340);f.w0=true;clful();
switch(m0){
    case 0:temp0=trig;break;
    case 1:temp1=trig;break;
    case 2:temp2=trig;break;}}

/////////////////////////////////////// ВЫВОДИМ ДАННЫЕ НА ЭКРАН ///////////////////////////////////
//НОМЕР ОДНОГО ИЗ ТРЁХ ЗНАЧЕНИЙ ТЕМПЕРАТУРЫ, ТРЕБУЕМУЮ ТЕМПЕРАТУРУ (БОЛЬШИЕ ЦИФРЫ), ЕСЛИ \
ПОСЛЕ УХОДА В СОН ПОСТУПИЛ СИГНАЛ С ДАТЧИКА ПОЛОЖЕНИЯ ДАЁМ ЗУММЕР И ВКЛЮЧАЕМ ИНДИКАТОР "POWER"
if(f.w1==true){menu0();bigdigit();f.w1=false;if(f.w7==true){f.w7=false;tone(11,1500,250);
digitalWrite(12,HIGH);}}
//КАЖДЫЕ 0.5 СЕКУНДЫ
if(mill-timer5>p500){timer5=mill;
//ОТОБРАЖАЕМ ФАКТИЧЕСКУЮ ТЕМПЕРАТУРУ 
lcd.setCursor(0,0);graf=Input;digprint3();lcd.write(B11011111);
//СКВАЖНОСТЬ ШИМ ПЕРЕВОДИМ В ПРОЦЕНТЫ
pwm=map(Output,0,255,0,100);lcd.setCursor(0,1);graf=pwm;digprint3();lcd.print("% ");
//ТЕМПЕРАТУРА ВНУТРИ КОРПУСА
lcd.print(rtc.getTemp(),0);}
}
                                  
/////////////////////////////////////////// МЕНЮ № 1 /////////////////////////////////////////////
if(menu==1){
if(f.w5==true){arrow1();f.w5=false;f.w6=true;ar=uint8_t(8);--m0;if(m0<0){m0=2;}}
key5();
if(f.w6==true&&newPos!=oldPos){om1=m1;m1=m1+newPos;if(m1>3){m1=0;}if(m1<0){m1=3;}clful();}
if(f.w6==false&&newPos!=oldPos){trig1=trig1+newPos;clful();
switch(m1){
    case 0:wait=constrain(trig1,1,99);break;
    case 1:down=constrain(trig1,1,99);break;
    case 2:tempwait=constrain(trig1,80,150);break;
    case 3:if(trig1>2){trig1=0;}if(trig1<0){trig1=2;};sharp=zen[trig1];break;}}
if(f.w1==true){menu1();f.w1=false;}}

/////////////////////////////////////////// МЕНЮ № 2 /////////////////////////////////////////////
if(menu==2){
if(f.w5==true){f.w5=false;f.w6=true;ar=uint8_t(8);}
key5();
if(f.w6==true&&newPos!=oldPos){m2+=newPos;if(m2>3){m2=0;}if(m2<0){m2=3;}clful();}
if(f.w6==false&&newPos!=oldPos){trig2+=newPos;clful();
switch(m2){
    case 0:KpSet=constrain(trig2,0,99);break;
    case 1:KiSet=constrain(trig2,0,99);break;
    case 2:autoret=constrain(trig2,3,30);break;
    case 3:if(trig2>1){trig2=0;}if(trig2<0){trig2=1;};led=trig2;break;}}
if(f.w1==true){menu2();f.w1=false;}
}
/////////////// ЕСЛИ НЕТ АКТИВНОСТИ В ЗАДАННЫЙ ИНТЕРВАЛ УХОДИМ В "СПЯЩИЙ РЕЖИМ" /////////////////
if(f.w0==true&&mill-timer1>(wait*p60000)){timer1=mill;temp=tempwait;f.w0=false;f.w7=true;
lcd.setCursor(5,0);lcd.print('W');tone(11,900,250);bigdigit();
}
//В "СПЯЩЕМ РЕЖИМЕ"
if(f.w7==true){
//МИГАЕМ ИНДИКАТОРОМ "POWER"
if(mill-timer3>(f.w8?p1000:p500)){timer3=mill;f.w8=!f.w8;digitalWrite(12,f.w8);
}
//ЕСЛИ ПО ПРЕЖНЕМУ НЕТ АКТИВНОСТИ В ЗАДАННЫЙ ИНТЕРВАЛ, ПЕРЕХОДИМ В РЕЖИМ "POWER OFF"
if(mill-timer1>(down*p60000)){power=0;f.w2=true;f.w7=false;}
}
/////////////////////////// АВТОВОЗВРАТ В ГЛАВНОЕ МЕНЮ ИЗ МЕНЮ 1 ИЛИ 2 ///////////////////////////
if(menu!=0&&mill-timer1>(autoret*p1000)){menu=0;f.w5=true;lcd.clear();}

}///////////////////////////////////////// END POWER ON //////////////////////////////////////////

///////////////////////////////////// POWER OFF (STANDBY) ////////////////////////////////////////
if(power==false){
//ЗАДАЁМ НАЧАЛЬНЫЕ ЗНАЧЕНИЯ ДЛЯ РЕЖИМА "POWER OFF"
if(f.w2==true){menu=0;fr=0;save=true;f.w2=false;f.w3=led;f.w5=true;f.w8=true;arrow0();
digitalWrite(13,HIGH);digitalWrite(12,LOW);analogWrite(10,0);lcd.clear();clmil();}
//timer3=mill;timer4=mill;timer5=mill;f.w7=true;

//ПОДАЁМ ЗВУКОВОЙ СИГНАЛ ОТКЛЮЧЕНИЯ ПАЯЛЬНИКА (БЛОКА ПИТАНИЯ 24В) f.w8=false;noTone(11);
if(fr<5&&mill>timer4+p100){timer4=mill;tone(11,freq[fr],90);++fr;}

//ПЕРЕКЛЮЧЕНИЕ МЕЖДУ ГЛАВНЫМ МЕНЮ И МЕНЮ УСТАНОВКИ ВРЕМЕНИ
if(f.w4==true&&oldbtn5==LOW&&mill-timer1>p400){timer1=mill;++menu;
if(menu>1){menu=0;}f.w1=true;f.w4=false;f.w5=true;lcd.clear();}

///////////////////////////// ГЛАВНОЕ МЕНЮ В РЕЖИМЕ "POWER OFF" /////////////////////////////////
if(menu==0){
//СБРАСЫВАЕМ МИГАНИЕ И ПОДЧЁРКИВАНИЕ КУРСОРА noTone(11);
if(f.w5==true){f.w4=false;f.w7=false;lcd.noCursor();lcd.noBlink();f.w5=false;
}
//ОБРАБАТЫВАЕМ НАЖАТИЕ НА ЭНКОДЕР
btn5=digitalRead(5);if(btn5!=oldbtn5){delay(10);btn5=digitalRead(5);
if(btn5==LOW&&oldbtn5==HIGH){timer1=mill;f.w4=true;}oldbtn5=btn5;
}
//ПЕРЕКЛЮЧАТЕЛЬ ПОДСВЕТКИ ЭКРАНА КРАТКОВРЕМЕННЫМ НАЖАТИЕМ НА ЭНКОДЕР
if(f.w4==true&&mill-timer1>p400){timer1=mill;f.w3=!f.w3;(f.w3?ledON:ledOFF);f.w4=false;
}
//СЧИТЫВАЕМ ТЕКУЩУЮ ТЕМПЕРАТУРУ ПАЯЛЬНИКА 1 РАЗ В СЕКУНДУ
if(f.w8==true&&mill>timer2+p1000){timer2=mill;inputdata=median(15);
temp=map(inputdata,0,1024,20,350);
//ЕСЛИ ПАЯЛЬНИК ГОРЯЧИЙ, ВЫВОДИМ ПРЕДУПРЕЖДАЮЩУЮ НАДПИСЬ И ТЕКУЩУЮ ТЕМПЕРАТУРУ
if(temp>hot){lcd.setCursor(0,0);lcd.print("TIP IS");lcd.setCursor(0,1);lcd.print("HOT!!!");
bigdigit();f.w7=true;}
//ЕСЛИ ПАЯЛЬНИК ОСТЫЛ, УПРАВЛЯЕМ ПОДСВЕТКОЙ ЭКРАНА, ПОДАЁМ ЗВУКОВОЙ СИГНАЛ И ОТКЛЮЧАЕМ ИНДИКАТОР
else if(f.w7=true){(f.w3?ledON:ledOFF);f.w7=false;f.w8=false;tone(11,1300,350);
digitalWrite(10,LOW);lcd.clear();}
}
//ПОКА ПАЯЛЬНИК ГОРЯЧИЙ МИГАЕМ ИНДИКАТОРОМ if(f.w8==false){}noTone(11);
if(f.w7==true&&mill-timer5>(f.w6?p400:p500)){timer5=mill;f.w6=!f.w6;digitalWrite(10,f.w6);
}
//ЕСЛИ ПАЯЛЬНИК ХОЛОДНЫЙ ВЫВОДИМ ЧАСЫ И ТЕМПЕРАТУРУ ПАЯЛЬНИКА 1 РАЗ В СЕКУНДУ
if(f.w8==false&&mill-timer3>p1000){timer3=mill;inputdata=median(15);
temp=map(inputdata,0,1024,20,350);bigtime();lcd.setCursor(13,0);lcd.print('+');lcd.print(temp);}
//graf=temp;digprint2();}
}

///////////////////////////////// МЕНЮ УСТАНОВКИ ВРЕМЕНИ И ДАТЫ //////////////////////////////
if(menu==1){
//СЧИТЫВАЕМ И ПЕЧАТАЕМ ТЕКУЩИЕ ЗНАЧЕНИЯ ВРЕМЕНИ И ДАТЫ
if(f.w5==true){f.w5=false;f.w6=true;setime=0;clmil();set_time();
lcd.setCursor(0,0);lcd.print(rtc.getTimeStr());
lcd.setCursor(9,0);lcd.print("SAVE-0+");
lcd.setCursor(0,1);lcd.print(rtc.getDateStr());
lcd.setCursor(11,1);lcd.print("DAY ");lcd.print(wt.dow);
lcd.setCursor(poz[loc],str);lcd.blink();}
//КНОПКОЙ ЭНКОДЕРА МЕНЯЕМ МИГАНИЕ НА ПОДЧЁРКИВАНИЕ КУРСОРА
btn5=digitalRead(5);if(btn5!=oldbtn5){delay(10);btn5=digitalRead(5);
if(btn5==LOW&&oldbtn5==HIGH){clmil();f.w4=true;f.w6=!f.w6;
if(f.w6==true){lcd.blink();lcd.noCursor();}else{lcd.noBlink();lcd.cursor();}}oldbtn5=btn5;}
//ПОВОРОТОМ ЭНКОДЕРА МЕНЯЕМ ПОЗИЦИЮ КУРСОРА
if(f.w6==true&&newPos!=oldPos){loc+=newPos;
if(loc<0){loc=7;str=1;}if(loc>7){loc=0;str=0;}if(loc>3){str=1;}if(loc<4){str=0;}clful();}
//ПОВОРОТОМ ЭНКОДЕРА МЕНЯЕМ ЗНАЧЕНИЕ В ПОЗИЦИИ КУРСОРА
if(f.w6==false&&newPos!=oldPos){trig3+=newPos;clful();
switch(loc){
    case 0:sethour=constrain(trig3,0,23);break;
    case 1:setmin=constrain(trig3,0,59);break;
    case 2:setsec=constrain(trig3,0,59);break;
    case 3:setime=constrain(trig3,-1,1);break;
    case 4:setdate=constrain(trig3,1,31);break;
    case 5:setmon=constrain(trig3,1,12);break;
    case 6:setyear=constrain(trig3,20,99);break;
    case 7:setdow=constrain(trig3,1,7);break;}}
if(f.w1==true){
switch(loc){
    case 0:trig3=sethour;break;
    case 1:trig3=setmin;break;
    case 2:trig3=setsec;break;
    case 3:trig3=setime;break;
    case 4:trig3=setdate;break;
    case 5:trig3=setmon;break;
    case 6:trig3=setyear;break;
    case 7:trig3=setdow;break;}
lcd.setCursor(poz[loc],str);if(trig3<10&&loc!=3&&loc!=7){lcd.print('0');}
lcd.print(trig3);
//if(loc!=3){lcd.print(trig3);}else{if(w3==0){lcd.print('N');}if(w3==1){lcd.print('Y');}}
lcd.setCursor(poz[loc],str);
//ЗАПИСЫВАЕМ ТЕКУЩИЕ ЗНАЧЕНИЯ ВРЕМЕНИ И ДАТЫ, ВЫВОДИМ СООБЩЕНИЕ ОБ ЭТОМ
if(setime!=0){time_set();lcd.clear();lcd.print(F("TIME & DATE SAVE"));}
f.w1=false;}
//ВОЗВРАЩАЕМСЯ В ОСНОВНОЕ МЕНЮ
if(setime!=0&&mill-timer1>(p1000+p1000)){f.w5=true;menu=0;lcd.clear();}
}
//delay(2000);clmil();timer1=mill;
}//////////////////////////////////////// END POWER OFF ///////////////////////////////////////////

///////////////////////////////////////// EEPROM UPDATE ///////////////////////////////////////////
if(save==true){//
//ПЕРЕЗАПИСЫВАЕМ ЗНАЧЕНИЯ В EEPROM ПРИ КАЖДОМ ПЕРЕХОДЕ В РЕЖИМ "POWER OFF"
temp00=highByte(temp0);//старший байт
temp01=lowByte(temp0);//младший байт
EEPROM.update(0,temp00);EEPROM.update(1,temp01);
temp10=highByte(temp1);temp11=lowByte(temp1);EEPROM.update(2,temp10);EEPROM.update(3,temp11);
temp20=highByte(temp2);temp21=lowByte(temp2);EEPROM.update(4,temp20);EEPROM.update(5,temp21);
EEPROM.update(6,tempwait);EEPROM.update(7,KpSet);EEPROM.update(8,KiSet);
EEPROM.update(10,sharp);EEPROM.update(11,wait);EEPROM.update(12,down);EEPROM.update(13,autoret);
EEPROM.update(14,led);
save=false;}

///////////////////////////////////////////////////////////////////////////////////////////////////
}/////////////////////////////////////////// END LOOP /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////





////////////////// ПЕРЕВОДИМ ПИНЫ D3 и D11 ТАЙМЕРА №2 НА ЧАСТОТУ - 31.4 кГц //////////////////////\
TCCR2B = 0b00000001;//БИТНОСТЬ 8x1 (0 - 255)void cl();\
TCCR2A = 0b00000001;//Режим ШИМ С ФАЗОВОЙ КОРРЕКЦИЕЙ (Phase Correct PWM) (На Fast PWM не поменять)
/////////////////// ПЕРЕВОДИМ ПИНЫ D3 и D11 ТАЙМЕРА №2 НА ЧАСТОТУ - 30 Гц ////////////////////////\
TCCR2B = 0b00000111;//x1024\
TCCR2A = 0b00000001;//phase correct

////////////////////////////////////////////// END ////////////////////////////////////////////////
// bool     = boolean одно из двух true=1...255 или false=0
// int8_t   = char          от        -128 до 127
// uint8_t  = byte          от           0 до 255
// int16_t  = int           от      -32768 до 32767
// uint16_t = word          от           0 до 65535 
// int32_t  = long          от -2147483648 до 2147483647 
// uint32_t = unsigned long от           0 до 4294967295

/////////////////////////////// МЕНЮ ПРЕДВАРИТЕЛЬНОЙ НАСТРОЙКИ /////////////////////////////////
/*
if(menu==10){

btn5=digitalRead(5);if(btn5!=oldbtn5){delay(10);btn5=digitalRead(5);
if(!btn5&&oldbtn5){timer1=mill;f.w4=true;}oldbtn5=btn5;}

#ifdef SERIAL_PORT_ENABLE

//ЭТАП 1. ДЛЯ НАСТРОЙКИ ТЕМПЕРАТУРЫ С ТЕРМОРЕЗИСТОРА НАПРЯМУЮ УПРАВЛЯЕМ ШИМ
if(newPos!=oldPos){pwm+=newPos;pwm=constrain(pwm,0,255);analogWrite(10,pwm);\
myEnc.write(0);oldPos=0;newPos=0;}

//ЭТАП 2. ЗАДАЕМ ТЕМПЕРАТУРУ ДЛЯ НАСТРОЙКИ ПИД\
if(newPos!=oldPos){temp=temp+newPos;temp=constrain(temp,100,340);Setpoint=temp;\
myEnc.write(0);oldPos=0;newPos=0;}

inputdata=median(15);
Input=map(inputdata,0,1024,20,350);
//АКТИВИРУЕМ ПРИ НАСТРОЙКЕ ПИД \
myPID.Compute();\
analogWrite(10,Output);\
pwm=map(Output,0,255,0,100);

//ВЫВОДИМ РЕЗУЛЬТАТЫ НА ЭКРАН КАЖДЫЕ 0,5 СЕК.
if(mill-timer1>p500){timer1=mill;\
lcd.setCursor(0,0);lcd.print("A0 ");graf=inputdata;digprint3();\
lcd.setCursor(8,0);lcd.print("TEMP ");graf=Input;digprint3();\
lcd.setCursor(0,1);lcd.print("PWM ");lcd.print(pwm);lcd.print(' ');\
lcd.setCursor(8,1);lcd.print("TSEL ");graf=Setpoint;digprint3();\
}

//Serial.print("A0 ");Serial.print(analogRead(A0));Serial.print("  ");\
if(mill-timer2>p100){timer2=mill;\
Serial.print("Median "); Serial.print(inputdata);Serial.print("  ");\
Serial.print("Input ");Serial.print(Input);Serial.print("  ");\
Serial.print("Setpoint ");Serial.print(Setpoint);Serial.print("  ");\
Serial.print("PWM ");Serial.print(pwm);Serial.print("  ");\
Serial.print("Output ");Serial.println(Output); 
//lcd.write(B11011111);lcd.write(B11011111);clful();
}
*/
