/////////////////////////////////////////// ФУНКЦИИ ////////////////////////////////////////

//ФУНКЦИЯ СЧИТЫВАЕТ АНАЛОГОВЫЙ ВХОД samples КОЛИЧЕСТВО РАЗ И ВОЗВРАЩАЕТ СРЕДНЕМЕДИАННОЕ ЗНАЧЕНИЕ
int16_t median(uint8_t samples) {
  int16_t raw[samples];//МАССИВ ДЛЯ ХРАНЕНИЯ ДАННЫХ
  int16_t tem = 0; uint8_t i, j; //ВРЕМЕННЫЕ ЛОКАЛЬНЫЕ ПЕРЕМЕННЫЕ
  for (i = 0; i < samples; i++) {
    raw[i] = analogRead(A0); //ЧИТАЕМ ДАННЫЕ С А0 И ПИШЕМ ИХ В ЯЧЕЙКИ МАССИВА
  }
  for (i = 0; i < samples; i++) { //СОРТИРУЕМ МАССИВ ПО ВОЗРАСТАНИЮ ЗНАЧЕНИЙ В ЯЧЕЙКАХ
    for (j = 0; j < samples - 1; j++) { //АНОМАЛЬНО БОЛЬШИЕ И МАЛЫЕ В НАЧАЛО И КОНЕЦ МАССИВА
      if (raw[j] > raw[j + 1]) {
        tem = raw[j];
        raw[j] = raw[j + 1];
        raw[j + 1] = tem;
      }
    }
  }
  return raw[samples >> 1]; //>>1 ВОЗВРАЩАЕМ РЕЗУЛЬТАТ ДЕЛЕНИЯ НА 2
}

void clmil() { //ОБНУЛЯЕМ ТАЙМЕР, ПОДНИМАЕМ ФЛАГ
  timer1 = mill;  
  f.w1 = true;
}

void clful() { //ОБНУЛЯЕМ ЭНКОДЕР, ТАЙМЕР
  myEnc.write(0);  
  oldPos = 0;
  newPos = 0;
  timer1 = mill;
  f.w1 = true;
}

void key5() { //ПЕРЕХОД В МЕНЮ 1 И 2
  btn5 = digitalRead(5); if (btn5 != oldbtn5) {
    delay(10); btn5 = digitalRead(5);
    if (btn5 == LOW && oldbtn5 == HIGH) {
      timer1 = mill; f.w1 = true; f.w4 = true; f.w6 = !f.w6;
      if (f.w6 == true) {
        ar = uint8_t(8);
      } else {
        ar = uint8_t(7);
      }
    } oldbtn5 = btn5;
  }
}

//3 ФУНКЦИИ "ПРАВИЛЬНОГО" ФОРМИРОВАНИЯ ЦИФР НА ЭКРАНЕ
void digprint1() {
  if (graf > -10 && graf < 10) {
    lcd.print(' ');
  }
  if (graf > 0) {
    lcd.print('+');
  } if (graf == 0) {
    lcd.print(' ');
  } lcd.print(graf);
}
void digprint2() {
  if (graf < 10) {
    lcd.print("  ");
  } else {
    lcd.print(' ');
  } lcd.print(graf);
}
void digprint3() {
  if (graf < 10) {
    lcd.print("  ");
  } if (graf >= 10 && graf < 100) {
    lcd.print(' ');
  } lcd.print(graf);
}
void menu0() { //ФУНКЦИЯ ПЕЧАТИ НОМЕРА ТЕКУЩЕЙ ТЕМПЕРАТУРЫ
  lcd.setCursor(5, 0);
  switch (m0) {
    case 0: trig = temp0; temp = temp0; lcd.print('1'); break;
    case 1: trig = temp1; temp = temp1; lcd.print('2'); break;
    case 2: trig = temp2; temp = temp2; lcd.print('3'); break;
  }
}
void menu1() { //ОТОБРАЖЕНИЕ ЗНАЧЕНИЙ В МЕНЮ 1
  uint8_t i;
  switch (m1) {
    case 0: trig1 = wait; break;
    case 1: trig1 = down; break;
    case 2: trig1 = tempwait; break;
    case 3: for (i = 0; i < 3; i++) {
        if (sharp == zen[i]) {
          trig1 = i;
        }
      } break;
  }
  lcd.setCursor(0, 0);
  if (m1 == 0) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("WAIT"); graf = wait; digprint2();
  lcd.setCursor(8, 0);
  if (m1 == 1) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("DOWN"); graf = down; digprint2();
  lcd.setCursor(0, 1);
  if (m1 == 2) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("TMPW "); graf = tempwait; digprint3();
  lcd.setCursor(10, 1);
  if (m1 == 3) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("TS"); graf = sharp; digprint2();
}
void menu2() { //ОТОБРАЖЕНИЕ ЗНАЧЕНИЙ В МЕНЮ 2
  switch (m2) {
    case 0: trig2 = KpSet; break;
    case 1: trig2 = KiSet; break;
    case 2: trig2 = autoret; break;
    case 3: trig2 = led; break;
  }
  lcd.setCursor(0, 0);
  if (m2 == 0) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("Kp="); Kp = ((double)KpSet) / 10.0;
  lcd.print(Kp, 1);
  lcd.setCursor(8, 0);
  if (m2 == 1) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("Ki="); Ki = ((double)KiSet) / 10.0;
  lcd.print(Ki, 1);
  lcd.setCursor(0, 1);
  if (m2 == 2) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("RET"); graf = autoret; digprint2();
  lcd.setCursor(8, 1);
  if (m2 == 3) {
    lcd.write(ar);
  } else {
    lcd.print(' ');
  } lcd.print("LED ");
  if (led == 1) {
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }
}
void bigdigit() { //ФУНКЦИЯ ФОРМИРОВАНИЯ БОЛЬШИХ ЦИФР ИЗ СИМВОЛОВ ПСЕВДОГРАФИКИ
  uint8_t a[3] = {0, 1, 2}, i, d1, d2, d3, d4, d5, d6, e1, e2, e3; //ЛОКАЛЬНЫЙ МАССИВ И ПЕРЕМЕННЫЕ
  a[0] = temp / 100;
  a[1] = temp / 10 % 10;
  a[2] = temp % 10;
  for (i = 0; i < 3; i++) {
    switch (i) {
      case 0: e1 = 7, e2 = 8, e3 = 9; break;
      case 1: e1 = 10, e2 = 11, e3 = 12; break;
      case 2: e1 = 13, e2 = 14, e3 = 15; break;
    }
    switch (a[i]) {
      case 0: d1 = 1, d2 = 8, d3 = 6, d4 = 1, d5 = 3, d6 = 6; break;
      case 1: d1 = 32, d2 = 2, d3 = 6, d4 = 32, d5 = 32, d6 = 6; break;
      case 2: d1 = 2, d2 = 8, d3 = 6, d4 = 1, d5 = 4, d6 = 5; break;
      case 3: d1 = 2, d2 = 4, d3 = 6, d4 = 7, d5 = 3, d6 = 6; break;
      case 4: d1 = 1, d2 = 3, d3 = 6, d4 = 32, d5 = 32, d6 = 6; break;
      case 5: d1 = 1, d2 = 4, d3 = 5, d4 = 7, d5 = 3, d6 = 6; break;
      case 6: d1 = 1, d2 = 4, d3 = 5, d4 = 1, d5 = 3, d6 = 6; break;
      case 7: d1 = 2, d2 = 8, d3 = 6, d4 = 32, d5 = 2, d6 = 6; break;
      case 8: d1 = 1, d2 = 4, d3 = 6, d4 = 1, d5 = 3, d6 = 6; break;
      case 9: d1 = 1, d2 = 4, d3 = 6, d4 = 7, d5 = 3, d6 = 6; break;
    }
    lcd.setCursor(e1, 0); lcd.write(d1);
    lcd.setCursor(e2, 0); lcd.write(d2);
    lcd.setCursor(e3, 0); lcd.write(d3);
    lcd.setCursor(e1, 1); lcd.write(d4);
    lcd.setCursor(e2, 1); lcd.write(d5);
    lcd.setCursor(e3, 1); lcd.write(d6);
  }
}

void bigtime() { //ФУНКЦИЯ ФОРМИРОВАНИЯ БОЛЬШИХ ЧАСОВ ИЗ СИМВОЛОВ ПСЕВДОГРАФИКИ
  uint8_t a[4] = {0, 1, 2, 3}, i, d1, d2, d3, d4, d5, d6, e1, e2, e3;
  wt = rtc.getTime();
  a[0] = wt.hour / 10;
  a[1] = wt.hour % 10;
  a[2] = wt.min / 10;
  a[3] = wt.min % 10;
  for (i = 0; i < 4; i++) {
    switch (i) {
      case 0: e1 = 0, e2 = 1, e3 = 2; break;
      case 1: e1 = 3, e2 = 4, e3 = 5; break;
      case 2: e1 = 7, e2 = 8, e3 = 9; break;
      case 3: e1 = 10, e2 = 11, e3 = 12; break;
    }
    switch (a[i]) {
      case 0: d1 = 1, d2 = 8, d3 = 6,   d4 = 1, d5 = 3, d6 = 6; break;
      case 1: d1 = 32, d2 = 2, d3 = 6,   d4 = 32, d5 = 32, d6 = 6; break;
      case 2: d1 = 2, d2 = 8, d3 = 6,   d4 = 1, d5 = 4, d6 = 5; break;
      case 3: d1 = 2, d2 = 4, d3 = 6,   d4 = 7, d5 = 3, d6 = 6; break;
      case 4: d1 = 1, d2 = 3, d3 = 6,   d4 = 32, d5 = 32, d6 = 6; break;
      case 5: d1 = 1, d2 = 4, d3 = 5,   d4 = 7, d5 = 3, d6 = 6; break;
      case 6: d1 = 1, d2 = 4, d3 = 5,   d4 = 1, d5 = 3, d6 = 6; break;
      case 7: d1 = 2, d2 = 8, d3 = 6,   d4 = 32, d5 = 2, d6 = 6; break;
      case 8: d1 = 1, d2 = 4, d3 = 6,   d4 = 1, d5 = 3, d6 = 6; break;
      case 9: d1 = 1, d2 = 4, d3 = 6,   d4 = 7, d5 = 3, d6 = 6; break;
    }
    lcd.setCursor(e1, 0); lcd.write(d1);
    lcd.setCursor(e2, 0); lcd.write(d2);
    lcd.setCursor(e3, 0); lcd.write(d3);
    lcd.setCursor(e1, 1); lcd.write(d4);
    lcd.setCursor(e2, 1); lcd.write(d5);
    lcd.setCursor(e3, 1); lcd.write(d6);
  }
    lcd.setCursor(14, 1); if (wt.sec < 10) {
    lcd.print('0');
  } lcd.print(wt.sec);
}

void bigdigitres() { //ФОРМИРУЕМ СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ БОЛЬШИХ ЦИФР В ГЛАВНОМ МЕНЮ
  byte v1[8] = { 7, 7, 7, 7, 7, 7, 7, 7};
  byte v2[8] = { 7, 7, 0, 0, 0, 0, 0, 0};
  byte v3[8] = { 0, 0, 0, 0, 0, 0, 31, 31};
  byte v4[8] = {31, 31, 0, 0, 0, 0, 31, 31};
  byte v5[8] = {28, 28, 0, 0, 0, 0, 28, 28};
  byte v6[8] = {28, 28, 28, 28, 28, 28, 28, 28};
  byte v7[8] = { 0, 0, 0, 0, 0, 0, 7, 7};
  byte v8[8] = {31, 31, 0, 0, 0, 0, 0, 0};
  lcd.createChar(1, v1); lcd.createChar(2, v2);
  lcd.createChar(3, v3); lcd.createChar(4, v4);
  lcd.createChar(5, v5); lcd.createChar(6, v6);
  lcd.createChar(7, v7); lcd.createChar(8, v8);
}
void arrow0() { //ПЕРЕЗАПИСЫВАЕМ СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ БОЛЬШИХ ЦИФР В ГЛАВНОМ МЕНЮ
  byte v7[8] = { 0, 0, 0, 0, 0, 0, 7, 7};
  byte v8[8] = {31, 31, 0, 0, 0, 0, 0, 0};
  lcd.createChar(7, v7); lcd.createChar(8, v8);
}

void arrow1() { //ПЕРЕЗАПИСЫВАЕМ СИМВОЛЫ ПСЕВДОГРАФИКИ ДЛЯ ТРЕУГОЛЬНИКА ИЛИ СТРЕЛОЧКИ
  byte v7[8] = {24, 28, 30, 31, 30, 28, 24, 0}; //ТРЕУГОЛЬНИК УКАЗАТЕЛЬ ДЛЯ МЕНЮ 1 И 2
  byte v8[8] = {24, 28, 14, 7, 14, 28, 24, 0}; //СТРЕЛОЧКА УКАЗАТЕЛЬ ДЛЯ МЕНЮ 1 И 2
  lcd.createChar(7, v7); lcd.createChar(8, v8);
}


void set_time() {
  wt = rtc.getTime(); //ЧИТАЕМ ТЕКУЩЕЕ ЗНАЧЕНИЕ ВРЕМЕНИ И ДАТЫ
  sethour = wt.hour; setmin = wt.min; setsec = wt.sec; \
  setdate = wt.date; setmon = wt.mon; setyear = (wt.year - 2000); setdow = wt.dow;
}

void time_set() { //ЗАПИСЫВАЕМ ТЕКУЩЕЕ ЗНАЧЕНИЕ ВРЕМЕНИ И ДАТЫ
  rtc.setTime(sethour, setmin, setsec); rtc.setDate(setdate, setmon, (setyear + 2000)); rtc.setDOW(setdow);
}

///////////////////////////////////////// END /////////////////////////////////////////
