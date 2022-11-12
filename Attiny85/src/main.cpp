#include "Setup.h"

#include <avr/pgmspace.h>
//#include <Wire.h>

#include "Power.h"
#include "SlaveI2C.h"
#include "Storage.h"
#include "counter.h"
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Для логирования раскомментируйте LOG_ON в Setup.h
#if defined(LOG_ON)
TinyDebugSerial mySerial;
#endif

#define FIRMWARE_VER 26 // Передается в ESP и на сервер в данных.

/*
Версии прошивок

26 - 2022.11.13 - dontsov
    1. Тестовая прошивка для 32мс импульсов

25 - 2022.11.09 - dontsov
    1. Единая прошивка Ватериуса и modkam zigbee версии

24 - 2022.02.22 - neitri, dontsovcmc
	1. Передача флага о том, что пробуждение по кнопке
	2. Передача количества включений режима настройки
	3. Убрано измерение напряжение, пусть его считает ESP

23 - ветка "8times" - 8 раз в секунду проверка входов

22 - 2021.07.13 - dontsovcmc
	1. переписана работа с watchdog: чип перезагрузится в случае сбоя

21 - 2021.07.01 - dontsovcmc
	1. переписана работа с watchdog
	2. поле voltage стало uint16 (2 байта от uint32 пустые для совместимости с 0.10.3)
	3. период пробуждения 15 мин, от ESP получит 1440 или другой.

20 - 2021.05.31 - dontsovcmc
	1. atmelavr@3.3.0
	2. конфигурация для attiny45

19 - 2021.04.03 - dontsovcmc
	1. WDTCR = _BV( WDCE ); в resetWatchdog

18 - 2021.04.02 - dontsovcmc
	1. WDTCR |= _BV( WDIE ); в прерывании

17 - 2021.04.01 - dontsovcmc
	1. Рефакторинг getWakeUpPeriod

16 - 2021.03.29 - dontsovcmc
	1. Отключение подтягивающих резисторов в I2C (ошибка в tinycore)
	2. Отключение ESP с задержкой 100мс после получения команды на сон (потребление ESP ниже на 7мкА).

15 - 2021.02.07 - kick2nick
	Время пробуждения ESP изменено с 1 суток (1440 мин.) на настриваемое значение
	1. Добавил период пробуждения esp.
	2. Добавил команду приема периода пробуждения по I2C.

14 - 2020.11.09 - dontsovcmc
	1. поддержка attiny84 в отдельной ветке

13 - 2020.06.17 - dontsovcmc
	1. изменил формулу crc
	2. поддержка версии на 4 счетчика (attiny84)
	   -D BUILD_WATERIUS_4C2W

12 - 2020.05.15 - dontsovcmc
	1. Добавил команду T для переключения режима пробуждения
	2. Добавил отправку аналогового уровня замыкания входа в ЕСП
	3. Исправил инициализацию входов. Кажется после перезагрузки +1 импульс
	4. Добавил crc при отправке данных

11 - 2019.10.20 - dontsovcmc
	1. Обновил алгоритм подсчёта импульсов.
	   Теперь импульс: 1 раз замыкание + 3 раза разомкнуто. Период 250мс +- 10%.

10 - 2019.09.16 - dontsovcmc
	1. Замеряем питание пока общаемся с ESP
	2. Время настройки 10 минут.

9 - 2019.05.04 - dontsovcmc
	1. USIWire заменен на Wire

8 - 2019.04.05 - dontsovcmc
	1. Добавил поддержку НАМУР. Теперь чтение состояния analogRead
	2. Добавил состояние входов.

7 - 2019.03.01 - dontsovcmc
	1. Обновил фреймворк до Platformio Atmel AVR 1.12.5
	2. Время аварийного отключения ESP 120сек.
	   Даже при отсутствии связи ESP раньше в таймауты уйдет и пришлет "спим".
*/

// Счетчики импульсов

// Waterius Classic: https://github.com/dontsovcmc/waterius
//
//                                +-\/-+
//       RESET   (D  5/A0)  PB5  1|    |8  VCC
//  *Counter1*   (D  3/A3)  PB3  2|    |7  PB2  (D  2/ A1)         SCL   *Button*
//  *Counter0*   (D  4/A2)  PB4  3|    |6  PB1  (D  1)      MISO         *Power ESP*
//                          GND  4|    |5  PB0  (D  0)      MOSI   SDA
//                                +----+
//
// https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_x5.md


// включение подтягивающего резистора и АЦП
#define ADC_START_T()    \
	PORTB |= _BV(PB4);   \
	ADCSRA |= _BV(ADEN); \
	ADCSRA |= _BV(ADSC);

// выключение подтягивающего резистора и АЦП
#define ADC_STOP_T()      \
	ADCSRA &= ~_BV(ADEN); \
	PORTB &= ~_BV(PB4);

// ожидание завершения преобразования АЦП;
#define ADC_CHECK_T()          \
	while (ADCSRA & _BV(ADSC)) \
		;

// чтение значения АЦП из регистра
#define ADC_VALUE_T() ADCH


static CounterB counter0(4, 2); // Вход 1, Blynk: V0, горячая вода PB4 ADC2
static CounterB counter1(3, 3); // Вход 2, Blynk: V1, холодная вода (или лог) PB3 ADC3

static ButtonB button(2);  // PB2 кнопка (на линии SCL)
						   // Долгое нажатие: ESP включает точку доступа с веб сервером для настройки
						   // Короткое: ESP передает показания
static ESPPowerPin esp(1); // Питание на ESP

// Данные
#ifdef MODKAM_VERSION
struct Header info = {FIRMWARE_VER, 0, 0, 0, WATERIUS_2C, {CounterState_e::CLOSE, CounterState_e::CLOSE}, {0, 0}, {0, 0}, 0, 0};

bool flag_new_counter_value = false;
#else
struct Header info = {FIRMWARE_VER, 0, 0, 0, 0, 0, WATERIUS_2C, {CounterState_e::CLOSE, CounterState_e::CLOSE}, {0, 0}, {0, 0}, 0, 0};
#endif

uint32_t wakeup_period;


//Кольцевой буфер для хранения показаний на случай замены питания или перезагрузки
//Кольцовой нужен для того, чтобы превысить лимит записи памяти в 100 000 раз
//Записывается каждый импульс, поэтому для 10л/импульс срок службы памяти 10 000м3
// 100к * 20 = 2 млн * 10 л / 2 счетчика = 10 000 000 л или 10 000 м3
static EEPROMStorage<Data> storage(20); // 8 byte * 20 + crc * 20

SlaveI2C slaveI2C;


/* Вектор прерываний сторожевого таймера watchdog */
ISR(WDT_vect)
{
	// для экономии времени АЦП запускается сразу из прерывания
	ADC_START_T();
}

// Проверяем входы на замыкание.
// Замыкание засчитывается только при повторной проверке.
inline void counting(uint8_t a)
{
	if (counter0.is_impuls(a))
	{
		info.data.value0++; //нужен т.к. при пробуждении запрашиваем данные
		info.adc.adc0 = counter0.adc;
		info.states.state0 = counter0.state;
		storage.add(info.data);
	}
}

//Запрос периода при инициализции. Также период может изменится после настройки.
// Настройка. Вызывается однократно при запуске.
void setup()
{


	noInterrupts();
	info.service = MCUSR; // причина перезагрузки

	//-----------------------------------------------
	// настройка системной частоты
	CLKPR = _BV(CLKPCE);
	// CLKPR = 2;           // установка предделителя частоты 1/4 (2 МГц)
	CLKPR = 3; // установка предделителя частоты 1/8 (1 МГц)
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");

	// настройка входов порта
	MCUCR &= ~_BV(PUD); // включение подтягивающих резисторов разрешено
	DDRB = 0;			// все пины работают на вход
	PORTB = 0;			// подтягивающие резисторы отключены
	PORTB |= _BV(PB5);	// включение подтягивающего резистора на Reset
						//-----------------------------------------------
						// настройка ADC (возврат к стандартным настройкам)
	// сброс: источник опорного напряжения Vcc, смещение вправо, порт ADC0 (PB5)
	ADMUX = 0;
	ADMUX |= _BV(ADLAR); // смещение влево (старшие 8 разрядов находятся в ADCH)
	ADMUX |= _BV(MUX1);	 // для ADC2 (PB4)
	// ADMUX |= _BV(MUX1) | _BV(MUX0); // для ADC3 (PB3)
	//  сброс: выключено АЦП и флаги прерывния, предделитель = 2
	ADCSRA = 0x1; // предделитель = 2
	// ADCSRA |= _BV(ADEN);              // АЦП включено
	// ADCSRA |= _BV(ADIF) | _BV(ADIE);  // разрешение прерывания по окончании преобразования
	// сброс: free running mode
	ADCSRB = 0;

	// выключение цифрового входа для портов ADC2 (PB4) и ADC3 (PB3)
	DIDR0 |= _BV(ADC2D) | _BV(ADC3D);

	//-----------------------------------------------
	// настройка сторожевого таймера
	// сброс значения вызвавшего нештатную перезагрузку МК
	MCUSR = 0; // без этого не работает после перезагрузки по watchdog

	// включение сторожевого таймера с указанием периода срабатывания
	wdt_enable(WDT_PERIOD_COUNTING);
	// включение прерывания для WDT
	WDTCR |= _BV(WDIE);
	//-----------------------------------------------
	// TCCR0B = 0;
	// TCCR1  = 0;
	interrupts();


	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	uint16_t size = storage.size();
	if (storage.get(info.data))
	{ //не первая загрузка
		info.resets = EEPROM.read(size);
		info.resets++;
		EEPROM.write(size, info.resets);
	}
	else
	{
		EEPROM.write(size, info.resets);					// 0
		EEPROM.write(size + 1, info.setup_started_counter); // 0
	}

	wakeup_period = WAKEUP_PERIOD_DEFAULT;

	LOG_BEGIN(9600);
	LOG(F("==== START ===="));
	LOG(F("MCUSR"));
	LOG(info.service);
	LOG(F("RESET"));
	LOG(info.resets);
	LOG(F("EEPROM used:"));
	LOG(size + 2);
	LOG(F("Data:"));
	LOG(info.data.value0);
	LOG(info.data.value1);
}

// Главный цикл, повторящийся раз в сутки или при настройке вотериуса
void loop()
{
	static uint8_t adc = 0xff;
	static uint8_t count = 0;       // счётчик 1 секунды
	static uint32_t count_sec = 0;

	power_all_disable(); // Отключаем все лишнее: ADC, Timer 0 and 1, serial interface
	power_adc_enable(); // Включаем ADC

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	wdt_enable(WDT_PERIOD_COUNTING); // сторожевой таймер работает по основному циклу
	WDTCR |= _BV(WDIE);

	sleep_cpu(); // переход в deep sleep
	WDTCR |= _BV(WDIE);

	count_sec = 0;
	while (true)
	{
		++count;

		ADC_CHECK_T();
		adc = ADC_VALUE_T(); // чтение значения АЦП в буфер
		ADC_STOP_T();
		sleep_cpu(); // переход в deep sleep

		counting(adc);
		WDTCR |= _BV(WDIE);

		if (count >= ONE_SECOND)
		{
			++count_sec;
			if (count_sec > wakeup_period || button.pressed()) 
			{
				break;
			}
		}
	}

	ADC_STOP_T();
	power_all_enable();
	wdt_enable(WDT_PERIOD_WAKEUP); // для общения с ESP
	WDTCR |= _BV(WDIE);
	LOG_BEGIN(9600);
	LOG(F("Data:"));
	LOG(info.data.value0);
	LOG(info.data.value1);


	// Если пользователь нажал кнопку SETUP, ждем когда отпустит
	// иначе ESP запустится в режиме программирования (кнопка на i2c и 2 пине ESP)
	// Если кнопка не нажата или нажата коротко - передаем показания
	unsigned long wake_up_limit;
	if (button.long_pressed())
	{ // wdt_reset внутри wait_release
		LOG(F("SETUP pressed"));
		slaveI2C.begin(SETUP_MODE);
		wake_up_limit = SETUP_TIME_MSEC; // 10 мин при настройке

		uint16_t setup_started_addr = storage.size() + 1;
		info.setup_started_counter = EEPROM.read(setup_started_addr);
		info.setup_started_counter++;
		EEPROM.write(setup_started_addr, info.setup_started_counter);
	}
	else
	{
		if (count_sec < wakeup_period)
		{
			LOG(F("Manual transmit wake up"));
			slaveI2C.begin(MANUAL_TRANSMIT_MODE);
		}
		else
		{
			LOG(F("wake up for transmitting"));
			slaveI2C.begin(TRANSMIT_MODE);
		}
		wake_up_limit = WAIT_ESP_MSEC; // 15 секунд при передаче данных
	}

	esp.power(true);
	LOG(F("ESP turn on"));

	while (!slaveI2C.masterGoingToSleep() && !esp.elapsed(wake_up_limit))
	{

		wdt_reset();

#ifdef MODKAM_VERSION
		info.voltage = readVcc(); // Текущее напряжение

		if (flag_new_counter_value) {
			flag_new_counter_value = false;

			storage.add(info.data);  // вдруг записали новое значение
		}
#endif
		ADC_START_T();
		ADC_CHECK_T();
		counting(ADC_VALUE_T());
		ADC_STOP_T();

		delayMicroseconds(30000); 

		if (button.long_pressed())
		{		   // wdt_reset внутри wait_release
			break; // принудительно выключаем
		}
	}

	slaveI2C.end(); // выключаем i2c slave.

	if (!slaveI2C.masterGoingToSleep())
	{
		LOG(F("ESP wake up fail"));
	}
	else
	{
		LOG(F("Sleep received"));
	}

	delayMicroseconds(20000);

	esp.power(false);
}
