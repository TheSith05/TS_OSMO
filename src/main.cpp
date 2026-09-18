// Подкидываем библиотеки
#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>
#include <Stamp.h>
#include <FastLED.h>
#include <GTimer.h>

// Настройка пинов
#define SENSOR_PIN 2
#define NUM_LEDS 6
#define DATA_PIN 0
#define RESET_PIN 6 // пин сброса
#define LED_REFRESH_RATE 100

#define STEP_6(set) set##_0, set##_1, set##_2, set##_3, set##_4, set##_5
#define LITERS 5 // С каким шагом писать на флешку

// А вот и создание объектов
GyverDBFile db(&LittleFS, "/data.db");
SettingsGyver sett("Продвинутый OSMOS", &db);
Datime dt_0, dt_1, dt_2, dt_3, dt_4, dt_5;
CRGB leds[NUM_LEDS];
GTimerCb<millis> tmr_LedUpdate;

// База данных
DB_KEYS(
    kk,
    total_liters, // Хранит общий пробег
    boot_pulses,  // Хранит общиие количество пульсов

    btn_reset, // Кнопка сброса
    btn_start, // Кнопка настройки
    btn_clear, // Обнулить показатели

    conf_reset,
    conf_start,
    conf_clear,
    flag_first_start, // Флаг первой настройки
    param_show_sw,    // Для свитча показа настроек
    vol_filters,      // То что будет хранить количество фильтров

    number_of_filters, // Хранит сколько у меня фильтров
    pulses,            // Полученные пульсы
    db_ssid,
    db_pass,

    STEP_6(ID),
    STEP_6(name),
    STEP_6(resource),
    STEP_6(date),
    STEP_6(run));

// Структура для хранения связанных данных
struct Struc_1
{
    size_t db_ID;
    size_t db_name;
    size_t db_resource;
    Datime *dt; // указатель на экземпляр
    size_t db_date;
    size_t db_run;
    boolean flag_reload;
};

// Массив из 6 элементов
Struc_1 filters[6] = {
    {kk::ID_0, kk::name_0, kk::resource_0, &dt_0, kk::date_0, kk::run_0, false},
    {kk::ID_1, kk::name_1, kk::resource_1, &dt_1, kk::date_1, kk::run_1, false},
    {kk::ID_2, kk::name_2, kk::resource_2, &dt_2, kk::date_2, kk::run_2, false},
    {kk::ID_3, kk::name_3, kk::resource_3, &dt_3, kk::date_3, kk::run_3, false},
    {kk::ID_4, kk::name_4, kk::resource_4, &dt_4, kk::date_4, kk::run_4, false},
    {kk::ID_5, kk::name_5, kk::resource_5, &dt_5, kk::date_5, kk::run_5, false}};

struct Struc_2
{
    const char *name; // лучше const char*, чтобы не плодить String (экономия памяти)
    int liters;
};

Struc_2 FirstSetings[6] = {
    {"Механика 5мкм", 10000},
    {"Сорбцион", 20000},
    {"Механика 1 мкм", 13000},
    {"Мембрана", 5000},
    {"Постфильтр", 5000},
    {"Минерализатор", 1000}};

struct Struc_3
{
    int remainder;   // остаток в литрах, может быть < 0 (перерасход)
    int gauge_value; // что показывать на шкале (по модулю при перерасходе)
    uint32_t color;  // цвет шкалы
    bool exhausted;  // ресурс исчерпан
};

int selected_filter_idx = 0; // Переменная для хранения выбора (по умолчанию 0 - первый фильтр)
int val;                     // Переменная количества фильтров в операвтивке.
int calcLiters; //Сколько вылили литров на калибровку.

float totalLiters = 0.0;      // Общий пробег в оперативке
float allLiters = 0.0;        // Пробег на момент установки.
float lastSavedLiters = 0.0;  // Хранит последнее сохраненнение пробега в оперативке
float PULSES_PER_LITER = 0.0; // Стартовое значение, которое мы будем калибровать
uint32_t bootPulses = 0;      // пульсы С ЭТОЙ загрузки — только для экрана калибровки

bool flag_confirm_start = false;
bool flag_confirm_reset = false;
bool flag_confirm_clear = false;
bool flag_notice = false;
bool flag_notice_reset = false;

// Настройки собственной Wi-Fi сети (Точки доступа)
String AP_SSID = "OsmoFilter";
String AP_PASS = "12345678"; // Пароль (минимум 8 символов!)

volatile uint32_t lastPulseTime = 0;

//-------------------
// ==========================================
// МЬЮТЕКС
// ==========================================
// portMUX_TYPE - системный тип. flowMutex - наше имя.
// portMUX_INITIALIZER_UNLOCKED - состояние "открыто".
portMUX_TYPE flowMutex = portMUX_INITIALIZER_UNLOCKED;

// Черновик для импульсов (изменяется в прерывании, поэтому volatile)
volatile uint32_t isrPulseCount = 0;

// ==========================================
// делаем универсальную финкцию для вебморды и лампочек, где считаем остаток цвет и т.д.
// ==========================================
Struc_3 calcFilterState(int i)
{
    Struc_3 cfs; // Объект нашей структуры
    float runAtInstall = db[filters[i].db_run];
    int resource = db[filters[i].db_resource];

    // Защита от деления на ноль, если ячейка resource вдруг пустая
    if (resource <= 0)
    {
        cfs.remainder = 0;
        cfs.gauge_value = 0;
        cfs.color = 0xF44336;
        cfs.exhausted = true;
        return cfs;
    }

    int last = round(resource - (totalLiters - runAtInstall));
    cfs.remainder = last;
    cfs.exhausted = (last <= 0);
    cfs.gauge_value = cfs.exhausted ? abs(last) : last;

    if (cfs.exhausted)
    {
        cfs.color = 0xF44336;
    }
    else
    {
        float percent = (float)last / resource * 100.0;
        cfs.color = 0x4CAF50;
        if (percent <= 15.0)
            cfs.color = 0xF44336;
        else if (percent <= 50.0)
            cfs.color = 0xFF9800;
    }
    return cfs;
}

// ==========================================
// Обновляем адресную ленту
// ==========================================
void updateLEDs()
{
    // Мигание каждые 300 мс
    bool isBlinking = (millis() / 300) % 2;

    for (int i = 0; i < val; i++)
    {
        Struc_3 st = calcFilterState(i);

        if (st.exhausted)
        {
            leds[i] = isBlinking ? CRGB::Red : CRGB::Black;
        }
        else
        {
            int resource = db[filters[i].db_resource];
            float ratio = constrain((float)st.remainder / resource, 0.0f, 1.0f);
            leds[i] = CHSV((uint8_t)(ratio * 96), 255, 255);
        }
    }
    FastLED.show();
}

// ==========================================
// ОБРАБОТЧИК ПРЕРЫВАНИЙ (ISR)
// ==========================================
void IRAM_ATTR pulseCounterISR()
{
    uint32_t currentTime = micros(); // Получаем текущее время в микросекундах

    // Если с прошлого импульса прошло больше 2000 микросекунд (защита от наводок)
    if (currentTime - lastPulseTime > 2000)
    {
        portENTER_CRITICAL_ISR(&flowMutex);
        isrPulseCount = isrPulseCount + 1;
        portEXIT_CRITICAL_ISR(&flowMutex);

        lastPulseTime = currentTime; // Запоминаем время успешного срабатывания
    }
}

// ==========================================
// ФУНКЦИЯ ОБРАБОТКИ ДАННЫХ
// ==========================================
void processWaterFlow()
{
    uint32_t newPulses = 0;

    portENTER_CRITICAL(&flowMutex);
    if (isrPulseCount > 0)
    {
        newPulses = isrPulseCount;
        isrPulseCount = 0;
    }
    portEXIT_CRITICAL(&flowMutex);

    if (newPulses > 0)
    {
        bootPulses += newPulses;                     // для калибровки
        totalLiters += newPulses / PULSES_PER_LITER; // накапливаем ЛИТРЫ напрямую
        // === УМНОЕ СОХРАНЕНИЕ ВО FLASH ===
        // Если разница между текущим пробегом и сохраненным
        if (totalLiters - lastSavedLiters >= (float)LITERS)
        {
            lastSavedLiters = totalLiters; 

            // Записываем данные в базу
            db[kk::total_liters] = totalLiters;

            // Форсируем сохранение на флешку
            db.update();
        }
    }
}

// === БИЛДЕР (Отрисовка) ===
void build(sets::Builder &b)
{
    // ==========================================
    // Начальное окно настроки
    // ==========================================
    if (db[kk::flag_first_start] == 0)
    {
        if (b.beginGroup("Данные для калибровки"))
        {
            b.LabelFloat(kk::total_liters, "Всего литров", totalLiters);
            b.LabelNum(kk::boot_pulses, "Пульсов с включения", bootPulses);
            if (b.Button(kk::btn_clear, "Обнулить показатели", sets::Colors::Green))
            {
                // Флаг для апдейтера
                flag_confirm_clear = true;
            }
            b.endGroup(); // Обязательно закрываем группу
        }
        if (b.beginGroup("Настройка"))
        {
            b.Input(kk::db_ssid, "Название сети", &AP_SSID);
            b.Number(kk::db_pass, "Пароль от сети", &AP_PASS);
            b.Number("Сколько пролили?", &calcLiters);
            if (b.Number(kk::number_of_filters, "Количество фильтров", &val, 1, 6))
            {
                b.reload();
            }
            b.Number("Текущий пробег", &allLiters);
            for (int i = 0; i < val; i++)
            {
                b.Input(filters[i].db_name, "Ступень", FirstSetings[i].name);
                b.Number(filters[i].db_resource, "Ресурс", &FirstSetings[i].liters);
            }
            if (b.Button(kk::btn_start, "Сохранить настройки", sets::Colors::Green))
            {
                // Флаг для апдейтера
                flag_confirm_start = true;
            }
            b.endGroup();
        }
    }
    else
    {
        // ==========================================
        // То что видим всегда
        // ==========================================
        if (b.beginGroup("Общий пробег"))
        {
            b.LabelFloat(kk::total_liters, "Всего литров", totalLiters);
            b.endGroup(); 
        }

        // 2. Вторая группа: Фильтр обратного осмоса
        if (b.beginGroup("Состояние фильтров"))
        {
            for (int i = 0; i < val; i++)
            {
                Struc_3 st = calcFilterState(i);

                filters[i].dt->set(db[filters[i].db_date]); 
                b.Label(filters[i].db_name, "Установлен: " + filters[i].dt->dateToString());

                if (!st.exhausted)
                {
                    b.LinearGauge(filters[i].db_ID, "Остаток", 0, db[filters[i].db_resource],
                                  "л", st.remainder, st.color);
                }
                else
                {
                    b.LinearGauge(filters[i].db_ID, "ПЕРЕРАСХОД", -1, 0,
                                  "л", st.gauge_value, st.color);
                }
            }
            if (b.Switch(kk::param_show_sw, "Показать настройки"))
            {
                b.reload();
            }
            b.endGroup();
        }
        // То что покажет интерфейс если выбрать показать
        if (db[kk::param_show_sw].toBool() == true)
        {
            if (b.beginGroup("Сохраненные нстройки"))
            {
                b.LabelNum("Импульсов на литр", PULSES_PER_LITER);
                b.LabelNum("Количесвто фильтров", val);
                for (int i = 0; i < val; i++)
                {
                    int res = db[filters[i].db_resource];
                    b.LabelNum(db[filters[i].db_name].toText(), res);
                }
            }
            b.endGroup();
        }
        // Сброс фильтров
        if (b.beginGroup("Обслуживание"))
        {

            // === Собираем строку ===
            String names;
            for (int i = 0; i < val; i++)
            {
                if (i > 0)
                    names += ';';
                names += db[filters[i].db_name].toString();
            }

            // Формируем строку опций. Важно, чтобы порядок совпадал с массивом filters!
            // Результат выбора автоматически запишется в selected_filter_idx
            b.Select("Выберите фильтр", names, &selected_filter_idx);

            // Кнопка сброса
            if (b.Button(kk::btn_reset, "Сбросить ресурс", sets::Colors::Red))
            {
                // Флаг для апдейтера
                flag_confirm_reset = true;
            }
            b.endGroup();
        }
    }
    bool res_reset;
    if (b.Confirm(kk::conf_reset, "Установить ресурс картриджа на 100%?", &res_reset))
    {
        if (res_reset)
        {
            // Пишем новые данные В БАЗУ ДАННЫХ по ключу из структуры
            db[filters[selected_filter_idx].db_date] = sett.rtc.getUnix();
            db[filters[selected_filter_idx].db_run] = totalLiters; // Запоминаем текущий общий пробег

            db[kk::total_liters] = totalLiters; // Переписываем показания т.к. пробег мог быть сброшен когда он не равен 5 литрам
            lastSavedLiters = totalLiters;

            db.update(); // Форсируем сохранение файла
            flag_notice_reset = true; //подмаем флаг убираем уведомление
            b.reload();
        }
    }

    bool res_start;
    if (b.Confirm(kk::conf_start, "Сохранить настроки?", &res_start))
    {
        if (res_start)
        {
            for (int i = 0; i < val; i++)
            {
                db[filters[i].db_date] = sett.rtc.getUnix();
            }
            if (calcLiters > 0)
            {
                db[kk::pulses] = bootPulses / calcLiters;
            }
            db[kk::total_liters] = allLiters;
            db[kk::flag_first_start] = 1;
            db.update(); // Форсируем сохранение файла
            b.reload();
            ESP.restart(); // Перезагрузка
        }
    }
}

void update(sets::Updater &upd)
{
    // ==========================================
    // ОБНОВЛЯЕМ ГЛАВНЫЕ СЧЕТЧИКИ В РЕАЛЬНОМ ВРЕМЕНИ ---
    // ==========================================
    upd.update(kk::total_liters, totalLiters);
    upd.update(kk::boot_pulses, bootPulses);

    for (int i = 0; i < val; i++)
    {
        Struc_3 st = calcFilterState(i);
        // Отправляем вычисленные данные и цвет в вебморду
        upd.update(filters[i].db_ID, st.gauge_value);
        upd.updateColor(filters[i].db_ID, st.color);

        // Переход в перерасход — просим браузер перестроить страницу
        if (st.exhausted && !filters[i].flag_reload)
        {
            filters[i].flag_reload = true;
            sett.reload();
        }
        if (!st.exhausted)
            filters[i].flag_reload = false; // ресурс восстановлен
    }

    // ==========================================
    // 4. Окна подтверждения и уведомления
    // ==========================================
    if (flag_confirm_clear)
    {
        totalLiters = 0;
        bootPulses = 0;
        lastSavedLiters = 0;        
        db[kk::total_liters] = 0.0; 
        db.update();
        flag_confirm_clear = false;
    }
    if (flag_confirm_reset)
    {
        flag_confirm_reset = false;
        upd.confirm(kk::conf_reset);
    }
    if (flag_confirm_start)
    {
        flag_confirm_start = false;
        upd.confirm(kk::conf_start);
    }
    if (flag_notice_reset)
    {
        flag_notice_reset = false;
        // Можно выводить уведомления, если сброс ресурса успешен
        upd.notice("Ресурс картриджа сброшен на 100%");
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(SENSOR_PIN, INPUT);
    pinMode(RESET_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseCounterISR, FALLING);

    // Небольшая защита от дребезга/наводки: жмём 300 мс
    delay(300);
    bool factoryReset = !digitalRead(RESET_PIN);

    // 1. Файловая система и БД
    LittleFS.begin(true);

    if (factoryReset) // Сброс при необходимости
    {
        LittleFS.remove("/data.db"); 
        Serial.println("FACTORY RESET!");
    }

    db.begin();

    // 2. Все init и выгрузка в оперативку
    db.init(kk::db_ssid, "OsmoFilter");
    db.init(kk::db_pass, "12345678");
    AP_SSID = db[kk::db_ssid];
    AP_PASS = db[kk::db_pass];

    for (int i = 0; i < 6; i++)
    {
        db.init(filters[i].db_name, FirstSetings[i].name);
        db.init(filters[i].db_resource, FirstSetings[i].liters);
        db.init(filters[i].db_date, 0);
        db.init(filters[i].db_run, 0.0);
    }
    db.init(kk::flag_first_start, 0);
    db.init(kk::total_liters, 0.0);
    db.init(kk::number_of_filters, 6);
    db.init(kk::pulses, 450);

    db[kk::param_show_sw] = false;

    totalLiters = db[kk::total_liters];
    val = db[kk::number_of_filters];
    PULSES_PER_LITER = db[kk::pulses];
    lastSavedLiters = totalLiters;

    // 3. Только теперь Wi-Fi
    WiFi.mode(WIFI_AP);
    //WiFi.setTxPower(WIFI_POWER_8_5dBm); // вместо дефолтных ~19 дБм
    WiFi.softAP(AP_SSID.c_str(), AP_PASS.c_str());

    
    sett.setVersion("v1.1.0");

    sett.setProjectInfo("ESP32 TS_OSMO", "https://github.com/TheSith05/TS_OSMO/");

    sett.begin();
    sett.onBuild(build);
    sett.onUpdate(update);
    sett.config.updateTout = 500;

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(100);
    tmr_LedUpdate.startInterval(LED_REFRESH_RATE, updateLEDs);
}

void loop()
{
    sett.tick();
    tmr_LedUpdate.tick();
    // Вызываем обработку импульсов при каждом проходе цикла!
    processWaterFlow();
}
