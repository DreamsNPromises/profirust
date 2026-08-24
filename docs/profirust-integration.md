# profirust: руководство по интеграции PROFIBUS-мастера

Проект: PROFIBUS-master модуль на RP2040 (Pico) + шлюз на Raspberry Pi 4.
Документ описывает, как устроена библиотека, что уже есть и что писать, и целевую архитектуру интеграции.

## 1. Что умеет profirust (v0.6.0)

- Циклический обмен с DP-V0 slave: чтение входов, запись выходов (process image).
- Жизненный цикл slave: параметризация → конфигурация → обмен → диагностика → offline-детект.
- Множество slave на одной шине, много-мастерная сеть (token ring).
- Диагностика (стандартная + расширенная), live-list, сканер шины (`DpScanner`), статистика (feature `statistics`).
- Бодрейты 9.6 kbit/s … 12 Mbit/s (12 Mbit/s ненадёжен, см. roadmap).

**Чего нет:**
- DP-V1 (ациклический доступ к параметрам, SAP 50/51) — в roadmap.
- Работа в роли slave (passive station) — в roadmap.
- Семантического маппинга "байты → теги" — это пишем мы.

## 2. Архитектура: 3 слоя

```
┌─────────────────────────────────────────────────┐
│ DP   (src/dp)      DpMaster, Peripheral, ...    │  ← здесь живут наши данные
│ FDL  (src/fdl)     FdlActiveStation, Telegram   │  ← токен-ринг, телеграммы (парсинг внутри)
│ PHY  (src/phy)     Rp2040Phy, SerialPortPhy...  │  ← UART + RS-485
└─────────────────────────────────────────────────┘
```

Слои связаны через trait `fdl::FdlApplication` (src/fdl/mod.rs:38): `DpMaster` реализует его, а `FdlActiveStation::poll()` вызывает. Для нескольких приложений — `poll_multi()`.

## 3. Ключевые типы

### PHY — `Rp2040Phy` (src/phy/rp2040.rs)
Конструктор: `Rp2040Phy::new(uart, dir_pin, per_clock, timer, &mut buffer[512], BAUD)`. UART 8E1, `dir_pin` переключает RS-485 TX/RX.

### FDL — `FdlActiveStation` (src/fdl/active.rs)
```rust
let fdl = fdl::FdlActiveStation::new(
    fdl::ParametersBuilder::new(MASTER_ADDRESS, BAUDRATE)
        .watchdog_timeout(Duration::from_secs(1))
        .slot_bits(1000)          // T_sl в битах; должен быть > max_tsdr всех slave
        .highest_station_address(3)
        .max_retry_limit(1)
        .build_verified(&dp_master)
);
fdl.set_online();                 // войти в token ring
fdl.poll(now, &mut phy, &mut dp_master);   // главный цикл
```
`build_verified()` проверяет, что `slot_bits` > max Tsdr каждого slave (src/fdl/parameters.rs:253). Для динамической конфигурации на Pico бери большой `slot_bits` (например 1000+) и не `build_verified`, либо пересобирай.

### DP — `DpMaster` / `Peripheral`
```rust
let mut storage: [dp::PeripheralStorage; MAX_SLAVES] = Default::default(); // максимум 124
let mut dp_master = dp::DpMaster::new(&mut storage[..]);

let handle = dp_master.add(
    dp::Peripheral::new(ADDR, options, &mut buf_inputs[..], &mut buf_outputs[..])
        .with_diag_buffer(&mut buf_diag[..])
);
dp_master.enter_operate();
```
- **Чтение входов:** `dp_master.get_mut(handle).pi_i()` — свежие байты от slave.
- **Запись выходов:** `pi_q_mut()` — байты уходят slave в следующем цикле.
- **Доступ ко всем:** `iter()`, `iter_mut()`.
- **События:** `take_last_events()` → `DpEvents { cycle_completed, peripheral }`, событие `PeripheralEvent::DataExchanged` (src/dp/master.rs:40).
- **Состояние slave:** `is_live()`, `is_running()`.

### `PeripheralOptions` (src/dp/peripheral.rs:3)
`ident_number`, `user_parameters: &[u8]`, `config: &[u8]`, `max_tsdr`, `fail_safe`, `sync_mode`, `freeze_mode`, `groups`. **Генерируется из GSD** (см. §5). Это `&[u8]` — ссылки с lifetime `'a`, данные должны жить в памяти (на Pico — в статическом буфере/арене).

## 4. Несколько slave

Да, поддерживается. `DpMaster` хранит массив `PeripheralStorage`; за один токен мастер опрашивает всех slave по очереди (цикл обмена), по завершении всех — `cycle_completed = true`.

- Объявить: `[dp::PeripheralStorage; N]`, по одному `add()` на каждое устройство (пример в доке `src/lib.rs:20`).
- Каждый `Peripheral` — свой буфер `pi_i`/`pi_q`, свой handle. Идентификация данных — по handle/address.
- Пример с несколькими приложениями (DpMaster + DpScanner): `examples/multi-application.rs`.
- Сканер для авто-обнаружения slave на шине: `dp::scan::DpScanner` (`examples/dp-scanner.rs`), live-list: `fdl::live_list::LiveList` (`examples/live-list.rs`).

## 5. GSD-файлы и gsdtool

- `gsd-parser` (workspace crate, **только std**) парсит GSD → `GenericStationDescription` (модули, слоты, `user_prm_data`, диагностика, бодрейты).
- `gsdtool` — интерактивный CLI **для хоста**: `config-wizard <file.gsd>` ведёт по параметрам/модулям и печатает готовый Rust-код `PeripheralOptions` + размеры буферов.
- **Важно:** `gsd-parser`/`gsdtool` не могут работать на Pico (no_std). Разбор GSD должен идти на шлюзе (Raspberry Pi, std) или на этапе сборки; на Pico уходит готовая конфигурация.

## 6. DP-V0 vs DP-V1

- Любой DP-V1 slave **обязан** поддерживать циклический DP-V0 обмен — обратная совместимость по спецификации. Так что циклический I/O с любым устройством profirust осилит.
- **Не реализовано:** ациклический доступ к параметрам DP-V1 (SAP 50/51). Для Phoenix IL PB BK DP/V1 это ок (куpler отдаёт I/O циклически). Для SEW DFP21B доступ к параметрам привода — либо DP-V1 (недоступно), либо **PPO-канал** поверх циклики (профиль PROFIdrive: запросы параметра вкладываются в process data с handshake-битами). PPO реализуемо в нашем middleware, но нужно читать документацию устройства.
- Вывод: для параметризации приводов может понадобиться своя прослойка поверх циклики. Для обычного I/O ничего делать не нужно.

## 7. Текущий `examples/rp-pico/src/main.rs` (ваш форк, ветка dev-rp2040)

| Блок | Что делает | Куда смотреть |
|---|---|---|
| overclock | 240 МГц для 12 Mbit/s | `mod overclock` |
| UART | GPIO0 TX / GPIO1 RX, драйв 12 мА | main.rs:72 |
| dir_pin | GPIO2 — RS-485 направление | main.rs:83 |
| GPIO6-8 | логирование ошибок в логический анализатор | main.rs:88 |
| USB CDC | core 1, USB-сериал: сливает SPSC-очередь логов в USB | main.rs:124 |
| DP master | 1 slave (адрес 3), `[PeripheralStorage; 1]`, цикл poll 32× | main.rs:158 |

`MAX_SLAVES` сейчас = 1 — для наших задач увеличить. USB сейчас только на TX логов; для шлюза нужен двунаправленный протокол.

## 8. Что уже есть vs что писать

| Задача | Статус |
|---|---|
| Циклический обмен (чтение входов / запись выходов) | ✅ profirust |
| Идентификация данных по slave | ✅ handle/address |
| Параметризация/конфигурация/диагностика/offline | ✅ profirust |
| Сканер шины, live-list, статистика | ✅ profirust |
| GSD-парсинг + генерация конфига | ✅ на шлюзе (std) |
| Маппинг "теги → (slave, offset, bit, тип)" | ❌ наша работа (на шлюзе) |
| Протокол шлюз ↔ Pico (фрейминг, сообщения) | ❌ наша работа |
| Динамическая конфигурация от шлюза на Pico | ❌ наша работа |
| Логи через протокол | ❌ наша работа |
| PPO-канал для приводов (если понадобится) | ❌ наша работа, device-specific |

## 9. Целевая архитектура интеграции

```
Raspberry Pi (шлюз, Rust, std)                 RP2040 (Pico, no_std, profirust)
┌──────────────────────────────┐   USB CDC    ┌──────────────────────────────────┐
│ gsd-parser: GSD → конфиг     │ ◄──────────► │ протокол: приём конфига,         │
│ маппинг тегов (decode/encode)│  (12 Mbit/s) │ отдача данных, приём выходов     │
│ Modbus/MQTT/...              │              │ DpMaster + [Peripheral; N]       │
└──────────────────────────────┘              │ арена памяти под pi_i/pi_q/conf  │
                                              └──────────────────────────────────┘
```

**Конфигурация — динамически от шлюза** (по нашему решению):
1. Шлюз парсит GSD → для каждого slave строит `PeripheralOptions`: адрес, `ident_number`, `user_parameters`, `config`, `max_tsdr`, размеры буферов.
2. Шлюз шлёт конфиг на Pico; Pico складывает его в статическую арену и создаёт `Peripheral` (буферы `pi_i`/`pi_q` и конфиги — это `&'a [u8]`, указывают на арену).
3. Pico запускает цикл обмена. Данные кладутся в `pi_i`; выходы шлюз шлёт по протоколу → Pico пишет в `pi_q_mut()`.

**Маппинг — на шлюзе:** тег → `(slave_addr, byte_offset, bit_offset, тип, порядок байт)`. Pico не знает про теги, он гоняет сырые байты. Шлюз декодирует `pi_i` по своей конфигурации (как в вашем Modbus-примере, только вместо "40001" — "slave 3, offset 0, I16").

**Протокол — делаем свой** (рекомендую). Готовые варианты не подходят:
- MQTT/HTTP — тяжело для no_std MCU, избыточно для USB-линка.
- Modbus RTU — по смыслу не то (это прикладной протокол для устройств, не для шины MCU↔хост).
- CBOR/postcard (serde) — viable, но тянет serde и генерацию схем; для простого моста оверхед.

**Свой фрейм-протокол** в общей crate (no_std-совместимой, чтобы лежала и на Pico, и в шлюзе):
- Фрейм: `MAGIC | len | type | payload | crc16`. Типы сообщений:
  - `CONFIG_UPLOAD` (шлюз→Pico): список периферии.
  - `DATA_PUSH` (Pico→шлюз): `slave_addr | len | pi_i байты | status(online/running/diag)`.
  - `OUTPUT_WRITE` (шлюз→Pico): `slave_addr | pi_q байты`.
  - `STATS/DIAG` (Pico→шлюз): статистика, события Online/Offline/Diagnostics.
  - `LOG` (Pico→шлюз): логи в этом же канале.
- Логи: текущий `logger_atomic` (SPSC-очередь + core1) оставляем, но вместо прямого TX в USB — кладём в `LOG`-фрейм. Один USB-канал, мультиплексирование по `type`. Отдельный USB-порт не нужен.
- Шлюзу логи приятно писать в файл/в stderr; данные — в теги.

**Скорость передачи на шлюз — не проблема.**
- Объём данных мал: process image на slave обычно 1..32 байт. Даже на 12 Mbit/s шине и 10 slave ≈ 1 кбайт за цикл обмена (~1 мс) → ≤ ~1 Мбайт/с на пике, но реально куда меньше.
- USB Full-Speed CDC на RP2040 даёт сотни кбайт/с — про запас.
- **Главное:** скорость вверх по линку **не привязана к бодрейту шины**. Pico сам решает, как часто слать `DATA_PUSH`: раз в N циклов / по таймеру / по изменению. В вашем Modbus-примере `poll_rate = 500` (мс) — аналогично делаем push-rate конфигурируемым. Шина работает всегда (данные свежие в `pi_i`), а шлюз получает столько, сколько переварит.

**Память на Pico (реализовано в `profibus-module/pico/src/config.rs`):**
- Арена `static mut` под `pi_i`/`pi_q`/диагностику (8192 б) и под байты `user_parameters`/`config` (4096 б), раздаётся при приёме конфига.
- `PeripheralStorage` — `[dp::PeripheralStorage; 16]` (см. `dp_control::MAX_SLAVES`).
- FDL `ParametersBuilder` с запасом по `slot_bits` (1000).

## 9а. Реализованный проект `profibus-module`

Отдельный workspace `profibus-module` (сейчас лежит внутри profirust: `profirust/profibus-module`):

| Путь | Что это |
|---|---|
| `protocol/` | no_std крейт протокола (frame + config + message), без зависимостей |
| `pico/` | прошивка RP2040: `main.rs` (core0 = DP-цикл, core1 = USB TX/RX), `config.rs` (арена), `host_link.rs` (фрейминг/очереди), `dp_bridge.rs` (конфиг → `Peripheral`, отправка данных по интервалу, события), плюс перенесённые `time.rs`/`overclock.rs`/`logger_atomic.rs`/`panic_handler.rs` |
| `.cargo/config.toml` | `flip-link` + `-Tlink.x` (в `pico/.cargo/config.toml`) + `linker gcc` для хоста |

Сборка: `cd pico && ./build.sh` (собирает и копирует `.uf2` на Pico). Зависимость от profirust — пока path `../..` (модуль лежит внутри profirust); для другой машины — git-URL форка (см. комментарий в `pico/Cargo.toml`).

## 10. Карта файлов для изменений

| Что менять | Файлы |
|---|---|
| Прошивка Pico (всё) | `profibus-module/pico/src/*` |
| Фрейм-протокол | `profibus-module/protocol/src/{frame,message,config}.rs` |
| Парсер конфига + арена | `profibus-module/pico/src/config.rs` |
| USB-канал (ввод/вывод, не только логи) | `profibus-module/pico/src/{main.rs,gw_link.rs}` |
| GSD → конфиг на шлюзе | `gsd-parser` (готов) + код в шлюзе |
| Маппинг тегов | код в шлюзе |
| PPO-канал приводов (если нужно) | middleware на Pico или шлюзе поверх process image |
| Доработка самого стека (DP-V1 и т.п.) | `src/dp/`, `src/fdl/` (в форке profirust) |

## 11. Команды

```bash
# gsdtool: конфигурация из GSD (на хосте!)
cargo run -p gsdtool -- config-wizard path/to/peripheral.gsd

# сборка Pico + прошивка (в profibus-module/pico)
./build.sh

# проверка протокола на хосте (из корня profibus-module)
cargo build -p protocol --target x86_64-unknown-linux-gnu
```

## 12. Итог: план работ

1. ✅ Общий crate `protocol` с фрейм-протоколом (frame + config + message, no_std) — в `profibus-module/protocol`.
2. ✅ Pico: приём конфига, арена, создание N `Peripheral`, цикл + `DATA_PUSH`/`OUTPUT_WRITE`/`EVENT`/`STATS`/`LOG` — в `profibus-module/pico`. Собирается и прошивается.
3. ⏳ На шлюзе: GSD→конфиг, маппинг тегов, приём/отправка по протоколу (следующий шаг).
4. ⏳ Обкатка на реальных устройствах (Phoenix, SEW); для SEW при необходимости PPO-прослойка.
