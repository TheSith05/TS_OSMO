#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>
#include <Stamp.h>
#include <FastLED.h>
#include <GTimer.h>

// Укажите данные вашей сети
#define WIFI_SSID "Keenetic giga 2.4"
define WIFI_PASS "Si"
#define SENSOR_PIN 2
#define NUM_LEDS 6
#define DATA_PIN 0
#define LED_REFRESH_RATE 100

GyverDBFile db(&LittleFS, "/data.db");
SettingsGyver sett("Продвинутый OSMOS", &db);
Datime dt_0, dt_1, dt_2, dt_3, dt_4, dt_5;
CRGB leds[NUM_LEDS];
GTimerCb<millis> tmr_LedUpdate;

// База данных
DB_KEYS(
    kk,
    total_liters,
    total_pulses,

    btn_reset,
    btn_start,
    conf_reset,
    conf_start,
    conf_save,
    flag_first_start,

    ID_0,
    ID_1,
    ID_2,
    ID_3,
    ID_4,
    ID_5,

    run_0,
    run_1,
    run_2,
    run_3,
    run_4,
    run_5,

    date_0,
    date_1,
    date_2,
    date_3,
    date_4,
    date_5);

// Структура для хранения связанных данных
struct Filter
{
    size_t db_ID;
    const char *name; // лучше const char*, чтобы не плодить String (экономия памяти)
    int liters;
    Datime *dt; // указатель на экземпляр
    size_t db_date;
    size_t db_run;
};

// Массив из 6 элементов (порядок зафиксирован)
Filter filters[6] = {
    {kk::ID_0, "PP 5мкм", 1000, &dt_0, kk::date_0, kk::run_0},
    {kk::ID_1, "Уголь", 1200, &dt_1, kk::date_1, kk::run_1},
    {kk::ID_2, "PP 1мкм", 1500, &dt_2, kk::date_2, kk::run_2},
    {kk::ID_3, "Мембрана", 1300, &dt_3, kk::date_3, kk::run_3},
    {kk::ID_4, "Постфильтр", 700, &dt_4, kk::date_4, kk::run_4},
    {kk::ID_5, "Минерализатор", 500, &dt_5, kk::date_5, kk::run_5}};

int selected_filter_idx = 0; // Переменная для хранения выбора (по умолчанию 0 - первый фильтр)

float totalLiters = 0.0;
float lastSavedLiters = 0.0;    // <--- Хранит последнее сохраненное во Flash значение
float PULSES_PER_LITER = 450.0; // Стартовое значение, которое мы будем калибровать

bool flag_confirm_save = false;
bool flag_confirm_start = false;
bool flag_confirm_reset = false;
bool flag_notice = false;
bool flag_notice_reset = false;

volatile uint32_t lastPulseTime = 0;

//-------------------
// ==========================================
// ПЕРЕМЕННЫЕ И МЬЮТЕКС (ЗАМОК)
// ==========================================
// portMUX_TYPE - системный тип. flowMutex - наше имя.
// portMUX_INITIALIZER_UNLOCKED - состояние "открыто".
portMUX_TYPE flowMutex = portMUX_INITIALIZER_UNLOCKED;

// Черновик для импульсов (изменяется в прерывании, поэтому volatile)
volatile uint32_t isrPulseCount = 0;

// Главные "бухгалтерские" счетчики
uint32_t totalPulses = 0;
//----------------------

// === Собираем строку ===
String getNamesString()
{
    String result = "";
    for (int i = 0; i < 6; i++)
    {
        if (i > 0)
            result += ';';
        result += filters[i].name;
    }
    return result;
}

String names = getNamesString(); // Получаем строку

// Обновленная функция принимает float, так как totalLiters у вас float
void updateLEDs()
{
    // Мигание каждые 300 мс
    bool isBlinking = (millis() / 300) % 2;

    for (int i = 0; i < NUM_LEDS; i++)
    {
        // Вычисляем остаток: ресурс конкретного фильтра минус общая потраченная вода
        float remainder = filters[i].liters - totalLiters;

        if (remainder < 0)
        {
            // Ресурс исчерпан: мигаем красным
            leds[i] = isBlinking ? CRGB::Red : CRGB::Black;
        }
        else
        {
            // Вычисляем долю остатка (от 0.0 до 1.0)
            float ratio = remainder / (float)filters[i].liters;

            // Защита границ, на всякий случай
            ratio = constrain(ratio, 0.0f, 1.0f);

            // 96 - зелёный (новый фильтр), 0 - красный (пора менять)
            uint8_t hue = ratio * 96;

            leds[i] = CHSV(hue, 255, 255);
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
    if (currentTime - lastPulseTime > 15000)
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
        totalPulses += newPulses;
        totalLiters = totalPulses / PULSES_PER_LITER;
        // === УМНОЕ СОХРАНЕНИЕ ВО FLASH ===
        // Если разница между текущим пробегом и сохраненным >= 1 литру
        if (totalLiters - lastSavedLiters >= 1.0)
        {
            lastSavedLiters = totalLiters; // Обновляем якорь

            // Записываем данные в базу только раз в литр!
            db[kk::total_pulses] = totalPulses;
            db[kk::total_liters] = totalLiters;

            // Форсируем сохранение на флешку[cite: 8]
            db.update();
        }
    }
}

// === БИЛДЕР (Отрисовка) ===
void build(sets::Builder &b)
{
    // 1. Первая группа
    if (b.beginGroup("Общий пробег"))
    {
        b.LabelFloat(kk::total_liters, "Всего литров", totalLiters);
        b.LabelNum(kk::total_pulses, "Всего пульсов", totalPulses);
        b.endGroup(); // Обязательно закрываем группу
    }

    // 2. Вторая группа: Фильтр обратного осмоса
    if (b.beginGroup("Фильтр обратного осмоса (Демо)"))
    {
        for (int i = 0; i < 6; i++)
        {
            // Читаем из базы данных пробег на момент установки И дату установки
            float run_at_install = db[filters[i].db_run];
            // Вычисляем остаток: Ресурс - (Общий пробег - Пробег при установке)
            int last_liter = filters[i].liters - (totalLiters - run_at_install);
            // Передаем экземпляру DT время в UNIX формате
            filters[i].dt->set(db[filters[i].db_date]);

            b.Label(filters[i].name, "Установлен: " + filters[i].dt->dateToString());

            if (last_liter >= 0)
            {
                // === ВЫЧИСЛЯЕМ ЦВЕТ ПРЯМО ТУТ ===
                float percent = ((float)last_liter / filters[i].liters) * 100.0;
                uint32_t gauge_color = 0x4CAF50; // Мягкий зеленый
                if (percent <= 15.0)
                    gauge_color = 0xF44336; // Красный
                else if (percent <= 50.0)
                    gauge_color = 0xFF9800; // Желтый

                // ПЕРЕДАЕМ ЦВЕТ В КОНЦЕ![cite: 1]
                b.LinearGauge(filters[i].db_ID, "Остаток", 0, filters[i].liters, "л", last_liter, gauge_color);
            }
            else
            {
                // Тут мы уже передаем красный (sets::Colors::Red)[cite: 1]
                b.LinearGauge(filters[i].db_ID, "ПЕРЕРАСХОД", 0, abs(last_liter), "л", abs(last_liter), sets::Colors::Red);
            }
        }
        b.endGroup();
    }

    if (b.beginGroup("Обслуживание"))
    {
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

    if (db[kk::flag_first_start] == 0)
    {
        if (b.beginGroup("Начать использование"))
        {
            if (b.Button(kk::btn_start, "Установить дату", sets::Colors::Green))
            {
                // Флаг для апдейтера
                flag_confirm_start = true;
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

            db.update(); // Форсируем сохранение файла[cite: 8]
            flag_notice_reset = true;
            b.reload();
        }
    }

    bool res_start;
    if (b.Confirm(kk::conf_start, "Обновитть все даты?", &res_start))
    {
        if (res_start)
        {
            for (int i = 0; i < 6; i++)
            {
                db[filters[i].db_date] = sett.rtc.getUnix();
            }
            db[kk::flag_first_start] = 1;
            db.update(); // Форсируем сохранение файла[cite: 8]
            b.reload();
        }
    }
}

void update(sets::Updater &upd)
{
    // ==========================================
    // --- ДОБАВЛЕНО: ОБНОВЛЯЕМ ГЛАВНЫЕ СЧЕТЧИКИ В РЕАЛЬНОМ ВРЕМЕНИ ---
    // ==========================================
    upd.update(kk::total_liters, totalLiters);
    upd.update(kk::total_pulses, totalPulses);

    for (int i = 0; i < 6; i++)
    {
        // Читаем пробег на момент установки
        float run_at_install = db[filters[i].db_run];
        // Считаем остаток
        int last_liter = filters[i].liters - (totalLiters - run_at_install);

        uint32_t gauge_color;
        int value_to_send;

        if (last_liter >= 0)
        {
            // === РЕЖИМ: НОРМА (Остаток >= 0) ===
            value_to_send = last_liter;
            float percent = ((float)last_liter / filters[i].liters) * 100.0;

            gauge_color = 0x4CAF50; // Мягкий зеленый
            if (percent <= 15.0)
            {
                gauge_color = 0xF44336; // Красный
            }
            else if (percent <= 50.0)
            {
                gauge_color = 0xFF9800; // Оранжевый
            }
        }
        else
        {
            // === РЕЖИМ: ПЕРЕРАСХОД (Остаток < 0) ===
            // Отправляем значение по модулю, чтобы шкала "ПЕРЕРАСХОД" (построенная в build)
            // всегда была заполнена на 100%
            value_to_send = abs(last_liter);
            gauge_color = 0xF44336; // Жестко красный цвет
        }

        // Отправляем вычисленные данные и цвет в вебморду
        upd.update(filters[i].db_ID, value_to_send);
        upd.updateColor(filters[i].db_ID, gauge_color);
    }

    // ==========================================
    // 4. Окна подтверждения и уведомления
    // ==========================================
    if (flag_confirm_save)
    {
        flag_confirm_save = false;
        upd.confirm(kk::conf_save);
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

    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseCounterISR, FALLING);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    LittleFS.begin(true);
    db.begin();
    for (int i = 0; i < 6; i++)
    {
        db.init(filters[i].db_date, 0); // Инициализируем даты нулями
        db.init(filters[i].db_run, 0);  // (или оставьте вашу ручную инициализацию)
    }
    db.init(kk::flag_first_start, 0);
    // Инициализируем общие счетчики (создаст ячейки, если их еще нет)
    db.init(kk::total_liters, 0.0);
    db.init(kk::total_pulses, 0ul); // 0ul означает unsigned long (uint32_t)

    // --- ДОБАВЛЕНО ---
    // Выгружаем сохраненный пробег из БД в наши рабочие переменные!
    totalLiters = db[kk::total_liters];
    totalPulses = db[kk::total_pulses];

    // Синхронизируем якорь со стартовым пробегом
    lastSavedLiters = totalLiters;

    // Устанавливаем версию прошивки. Она будет видна в веб-интерфейсе!
    sett.setVersion("v1.2.0-beta");

    // Задаем имя проекта и кликабельную ссылку
    sett.setProjectInfo("ESP32 SuperMini", "https://github.com/");

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