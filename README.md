# AEM2ABIT

Прошивка для платы [CanFilter на STM32G474](https://github.com/Sergey1560/canfilter_g4): CAN-шлюз, который принимает данные широкополосного
лямбда-контроллера **AEM X-Series UEGO** (30-0310) и передаёт их в формате ADLM ЭБУ **ABIT**.

```
 AEM UEGO ──CAN 500k──► FDCAN2 ──► преобразование ──► FDCAN1 ──CAN 500k──► ABIT
           ID 0x180 (29 бит)                                 ID 0x211 (11 бит)
```

## Что делает

- Слушает шину AEM на FDCAN2 и принимает только extended-кадр `0x180` длиной 8 байт.
  Стандартные и remote-кадры отбрасываются аппаратным фильтром.
- Пересчитывает лямбду и O₂ в масштаб ABIT и сразу отправляет кадр `0x211` в FDCAN1.
  Всё происходит в прерывании приёма, задержка — один кадр.
- Показывает состояние на трёх светодиоде.
- Если контроллер ушёл в bus-off, автоматически восстанавливает связь.

## Формат кадров

### Вход: AEM, ID `0x180` (extended), 8 байт, ~100 Гц

| Байты | Поле | Масштаб |
|---|---|---|
| 0–1 | Lambda, big-endian, unsigned | 0.0001 λ/бит |
| 2–3 | Oxygen, big-endian, signed | 0.001 %/бит |
| 4 | Напряжение питания | 0.1 В/бит |
| 6, бит 7 | Data valid | 1 — данные достоверны |
| 6, бит 5 | Free-air calibration | |
| 6, бит 1 | LSU detected | |
| 7, бит 6 | Sensor fault | 1 — неисправность датчика |

Описание сигналов лежит в [dbc/aem_lambda.dbc](dbc/aem_lambda.dbc), документация AEM — в [doc/aem_30-0310.pdf](doc/aem_30-0310.pdf).

### Выход: ABIT, ID `0x211` (standard), 8 байт

| Байты | Поле | Масштаб |
|---|---|---|
| 0–1 | Lambda, little-endian, unsigned | 0.001 λ/бит (AEM / 10, с округлением) |
| 2–3 | Oxygen, little-endian, unsigned | 0.01 %/бит (AEM / 10, с округлением; отрицательные значения → 0) |
| 4–6 | — | 0 |
| 7, бит 0 | Готовность датчика | 1 — AEM Data valid и нет Sensor fault |

Описание протокола ABIT — [doc/abit_adlm.pdf](doc/abit_adlm.pdf).

Пример: AEM `35 4E 14 5F …` (λ = 1.3646, O₂ = 5.215 %) → ABIT `55 05 0A 02 00 00 00 01`
(λ = 1.365, O₂ = 5.22 %).

## Светодиоды (WS2812B)

| № | Цвет | Значение |
|---|---|---|
| 0 | зелёный | AEM сообщает, что данные достоверны |
| 1 | красный | неисправность датчика |
| 2 | мигает зелёным, 1 Гц | прошивка работает |

## Железо

- MCU: **STM32G474**, кварц HSE 8 МГц, ядро 170 МГц (Range 1 boost), FDCAN тактируется от PLLQ на 85 МГц.
- Обе шины работают на 500 кбит/с: 85 tq на бит, точка выборки 75.3 %, SJW = 21.

| Сигнал | Пин |
|---|---|
| FDCAN1 (ABIT) RX / TX | PB8 / PB9, AF9 |
| FDCAN1 включение трансивера | PB7 |
| FDCAN1 silent | PC13 |
| FDCAN2 (AEM) RX / TX | PB12 / PB13, AF9 |
| FDCAN2 silent | PB14 |
| WS2812B data | PB6 (TIM8_CH1 + DMA1 Ch1) |

Скорость шины задаётся аргументом `fdcan_init()` в [can.c](Lib/Mylib/can.c) (`CAN_BD_125` … `CAN_BD_1000`),
сегменты бита — в [can.h](Lib/Mylib/can.h).

## Структура

```
Src/
  main.c          инициализация: тактирование, светодиоды, CAN
  led.c           таймеры обновления WS2812B (TIM6, 10 Гц) и мигания (TIM7)
Lib/Mylib/
  can.c / can.h   FDCAN: настройка, фильтры, прерывания, преобразование AEM → ABIT
  rcc.c           тактирование 170 МГц от HSE
  ws2812b.c       драйвер WS2812B на таймере с DMA
  dwt.c           счётчик тактов DWT (метки времени в логах)
  log.h           макросы логирования через SEGGER RTT
Lib/CMSIS         заголовки CMSIS / STM32G4
Lib/Segger        SEGGER RTT и SystemView
Startup/          startup и linker script
dbc/              DBC-описание кадра AEM
dump/             записи шины AEM (candump) при заглушенном и работающем двигателе
```

## Сборка

Нужны CMake ≥ 3.22 и `arm-none-eabi-gcc`. Путь к тулчейну задаётся переменной `GCC_TC_PATH`
в [CMakeLists.txt](CMakeLists.txt).

```sh
cmake -B build
cmake --build build
```

На выходе получаются `build/aem2abit.elf`, `.bin` и `.srec`.

Опция `-DSYSTEMVIEW_ENABLED=ON` подключает SEGGER SystemView.

## Отладка

Логи идут через SEGGER RTT, канал 0. Читать их можно в J-Link RTT Viewer или в Ozone
(в репозитории есть [ozone.jdebug](ozone.jdebug)). Подробный вывод CAN включается
макросом `CAN_DEBUG` в [can.h](Lib/Mylib/can.h). В рабочей прошивке его лучше выключить:
он печатает каждый кадр прямо из прерывания.
