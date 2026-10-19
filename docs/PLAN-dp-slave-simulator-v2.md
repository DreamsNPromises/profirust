# План: DP-V0 slave-симулятор на RP2040 (пошаговый)

Цель: вторая RP2040 на той же шине, что и тестируемый master, эмулирует N
DP-V0 slave. Минимальный стенд: 2 slave, на каждый — LED (вход мастера →
выход slave) и кнопка (вход slave → мастер). Проверка — существующим
`examples/rp-pico` (доработанным на N периферий и 1 байт выходов).

Спецификация: [felser.ch/profibus-manual](https://www.felser.ch/profibus-manual/)
(`zeitverhaeltnisse.html`, `telegrammformate_2.html`, `ms0_zyklischer_datenaustausch.html`).
Источник истины по тому, что принимает master — код `src/dp/peripheral.rs`.

---

## 0. Зафиксированные решения

| Вопрос | Решение |
|--------|---------|
| Протокол | DP-V0, без DP-V1 |
| SYNC/FREEZE | не реализуем (master их не шлёт; `Global_Control` шлёт только Clear/Operate) |
| Железо | 2×RP2040. Для стенда допустимо прямое UART TX↔RX + GND, без трансиверов |
| Профиль slave | ident `0x00F0`, prm `[0x20]`, cfg = `DI8 + DO8` (см. §6), 1 байт входов, 1 байт выходов |
| N | 2 (расширяемо до 8; таблица + статические буферы) |
| Скорость | сначала 500 кбит/с, затем 1.5/3/6/12 Мбит/с (master уже проверен на 12 Мбит/с с реальным coupler'ом при 240 МГц) |
| Библиотека | новый модуль `src/dp/slave.rs` (без отдельного `fdl/passive.rs`) |
| Firmware | новый crate `examples/rp-pico-slave/` |
| I/O | LED = `pi_q[0].bit0`, кнопка = `pi_i[0].bit0` (active low) |

---

## 1. Пошаговый план

### Шаг 1. Библиотека: `DpSlave` + unit-тесты (~1 файл)

Файлы: `src/dp/slave.rs` (новый), `src/dp/mod.rs` (экспорт).

Задачи:
1. Типы `SlaveState`, `SlaveConfig`, `DpSlave`, константа `MAX_REPLY_LEN` — сигнатуры в §4.
2. `handle_telegram(now, &Telegram, out) -> usize`: FDL-status, Set_Prm, Chk_Cfg,
   Slave_Diag, Global_Control, Data_Exch, неизвестный SAP → RS. Форматы — §5.
3. FCB-дедуп (§5.7), watchdog `tick(now)`, `set_ext_diag`/`clear_ext_diag`.
4. Unit-тесты на telegram-уровне: вход — собранные байты запроса, выход — байты ответа.

Приёмка:
```bash
cargo test -p profirust --features phy-simulator slave
cargo clippy -p profirust --features phy-simulator
```
Обязательные кейсы — приложение B.1.

### Шаг 2. Host-интеграция: `DpMaster` ↔ N × `DpSlave` на `SimulatorPhy`

Файлы: `src/dp/slave.rs` (тестовый модуль) или `tests/slave_integration.rs`.

Задачи:
1. Харнесс: `phy_control` владеет bus_time, `phy_master` и `phy_slaves` — `duplicate()`.
2. Цикл: `advance_bus_time(1us)` → `fdl_master.poll()` → `receive_telegram` →
   `dispatch` по N slave → **`advance_bus_time_min_tsdr()`** → `transmit_data`.
   Без этого шага `SimulatorBus` паникует (ответ раньше 11 бит).
3. Сценарии B.2.

Приёмка: `cargo test -p profirust --features phy-simulator slave` — зелёный;
master доходит до `is_running()` на 4 slave'ах, `cycle_completed` идёт.

### Шаг 3. Firmware-каркас: 1 slave на реальной шине

Файлы: `examples/rp-pico-slave/` (новый crate: копия обвязки из `examples/rp-pico`
— `time.rs`, `overclock.rs`, `logger_atomic.rs`, `panic_handler.rs`, `build.sh`).

Задачи:
1. `main.rs`: UART (GPIO0/1), dir (GPIO2), PHY, USB-логи (как в master-примере).
2. Главный цикл core0: `poll_transmission → receive_telegram → dispatch → transmit`.
3. Один `DpSlave`: addr 3, профиль из §0. LED/кнопки пока нет, pi_i/pi_q — константы.
4. Порядок действий и формула паузы перед TX — §3.

Приёмка: master-пример (с `config = DI8+DO8`, `pi_q = 1 байт`) доходит до
`Peripheral #3 becomes ready for data exchange`, `is_running() == true`,
`Inputs RAW` читается. Скорость 500 кбит/с.

### Шаг 4. LED + кнопка для одного slave

Файлы: `examples/rp-pico-slave/src/main.rs`, `examples/rp-pico/src/main.rs` (master).

Задачи:
1. Slave: после каждого `handle_telegram` (или раз в цикл) `led.set(pi_q[0] & 1)`;
   `pi_i[0] = !button.is_low()`.
2. Master: `buffer_outputs = [0u8; 1]`, в цикле писать в `pi_q[0]` счётчик/мигалку,
   печатать `pi_i[0]`.

Приёмка: master пишет 0/1 → LED гаснет/горит; нажатие кнопки меняет `Inputs RAW`
в логе master'а.

### Шаг 5. N slave (2, затем 4)

Файлы: `examples/rp-pico-slave/src/slaves.rs` (таблица), `main.rs`;
`examples/rp-pico/src/main.rs` (N периферий).

Задачи:
1. Таблица `SlaveDef { address, ident, prm, cfg, pi_i_len, pi_q_len, led, button }`.
2. Статические буферы на N slave, `DpSlave::new` в цикле.
3. Диспетчер: вызвать `handle_telegram` у всех, вернуть ответ первого
   ненулевого (адреса уникальны; broadcast отвечает 0).
4. Пина: slave 3 → LED GPIO6, кнопка GPIO8; slave 4 → LED GPIO7, кнопка GPIO9.
5. Master: N периферий, каждому свой `pi_i`/`pi_q`, в цикле мигать по-разному.

Приёмка: оба slave `is_running()`, LED независимы, кнопки независимы.

### Шаг 6. Инъекция отказов (опционально, но полезно)

Файлы: `examples/rp-pico-slave/src/faults.rs`, `main.rs`.

Задачи: флаги `silent`, `corrupt_crc`, `delay_us`, `drop_every_n`, `force_rs`,
`prm_fault`, `cfg_fault`; переключение — USB-командами (core1 → SPSC → core0) или
кнопкой-переключателем сценариев. Применяются в `dispatch` после сборки ответа.

Приёмка: master видит Offline/retry/ParameterError/ConfigError.
Этот шаг — только если хочется проверять отказы на железе; базово они
проверяются в host-тестах (Шаг 2).

### Шаг 7. 12 Мбит/с

Задачи: убедиться в `overclock.rs` (240 МГц), поднять скорость, замерить
логическим анализатором от «конец запроса» до «первый байт ответа»; он должен
быть в `[min TSDR, max TSDR]` = `[11, 800]` бит при 12 Мбит/с.

Приёмка: длинный прогон без потерь и без retry; замеры в логе.

---

## 2. Архитектура firmware

```
core0 (bus, hot path):
  loop {
    now = time::now();
    if phy.poll_transmission(now) { continue; }        // Tset + выталкивание TX
    n = phy.receive_telegram(now, |res| match res {
        Ok(t) => dispatch(&mut slaves, now, &t, &mut tx_buf),
        Err(_) => 0,                                    // framing error: игнор
    }).unwrap_or(0);
    if n > 0 {
        wait_tsdr(now, min_tsdr_bits);                  // §3, только если Tset < 11 бит
        phy.transmit_data(time::now(), |buf| { buf[..n].copy_from_slice(&tx_buf[..n]); (n, ()) });
    }
    for s in slaves.iter_mut() { s.tick(now); }         // watchdog
    // LED <- pi_q, pi_i <- кнопки
  }

core1: USB CDC, логи (как в master-примере). Команды — опционально.
```

`dispatch()` нельзя вызывать вне `receive_telegram`-closure: телеграмма
заимствует буфер PHY. Внутри closure PHY не трогать; ответ копируется в `tx_buf`.

---

## 3. Тайминги (Felser `zeitverhaeltnisse.html`)

| Биты | ≤187.5k | 500k | 1.5M | 3M | 6M | 12M |
|------|---------|------|------|----|----|-----|
| TQUI | 0 | 0 | 0 | 3 | 6 | 9 |
| TSET | 1 | 1 | 1 | 4 | 8 | 16 |
| TRDY | ≤11 | ≤11 | ≤11 | ≤11 | ≤11 | ≤11 |
| min TSDR | 11 | 11 | 11 | 11 | 11 | 11 |
| max TSDR | 60 | 100 | 150 | 250 | 450 | 800 |
| TSL | 100 | 200 | 300 | 400 | 600 | 1000 |

`Rp2040Phy::transmit_data` ставит `start_tx = now + tset_us`, т.е. первый байт
уйдёт не раньше `now + Tset`. Значит перед вызовом:
```
tset_us  = ceil(tset_bits * 1e6 / rate)
need_us  = ceil(11 * 1e6 / rate)
if tset_us < need_us { busy_wait(need_us - tset_us) }
```
При 12 Мбит/с Tset = 16 бит ≥ 11 → пауза не нужна. При 6 Мбит/с с учётом
округления вверх — тоже проходит; при ≤3 Мбит/с пауза обязательна.

---

## 4. API `src/dp/slave.rs`

```rust
pub const MAX_REPLY_LEN: usize = 256;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SlaveState { #[default] Wprm, Wcfg, Dxchg }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlaveConfig<'a> {
    pub address: u8,
    pub ident_number: u16,
    pub user_parameters: Option<&'a [u8]>, // None = принимать любые
    pub config: Option<&'a [u8]>,          // None = принимать любую
    pub fail_safe: bool,
}

pub struct DpSlave<'a> { /* поля см. ниже */ }

impl<'a> DpSlave<'a> {
    pub fn new(cfg: SlaveConfig<'a>, pi_i: &'a mut [u8], pi_q: &'a mut [u8]) -> Self;
    pub fn address(&self) -> u8;
    pub fn state(&self) -> SlaveState;
    pub fn min_tsdr_bits(&self) -> u8;      // из последнего Set_Prm, по умолчанию 11
    pub fn pi_i(&self) -> &[u8];
    pub fn pi_i_mut(&mut self) -> &mut [u8];
    pub fn pi_q(&self) -> &[u8];
    pub fn pi_q_mut(&mut self) -> &mut [u8];

    /// Вернуть длину ответа, записанного в `out`; 0 = ответа нет.
    pub fn handle_telegram(&mut self, now: Instant, t: &Telegram, out: &mut [u8]) -> usize;
    pub fn tick(&mut self, now: Instant);   // watchdog
    pub fn set_ext_diag(&mut self, bytes: &[u8]); // выставит DH на следующем Data_Exch
    pub fn clear_ext_diag(&mut self);
}
```

Поля: `cfg, state, fcb, last_sap, last_reply[MAX_REPLY_LEN], last_reply_len,
has_last_reply, master_address, groups, min_tsdr_bits, wd_on, wd_factors,
wd_timestamp, prm_fault, cfg_fault, prm_req, diag_pending, ext_diag[32],
ext_diag_len, pi_i, pi_q`. Память ≈ 340 Б + буферы: на 8 slave < 4 КБ.

Алгоритм `handle_telegram`:
```
1. Data(dt) = t иначе 0
2. FDL-status и dt.da == address → ответ §5.5 (без FCB)
3. Request{fcb,req} = dt.fc иначе 0
4. dt.da != address && dt.da != 127 → 0
5. sap = dt.dsap
6. fcv && has_last_reply && fcb == self.fcb && sap == last_sap → вернуть last_reply
7. 61→Set_Prm, 62→Chk_Cfg, 60→Diag, 58&&da==127→Global_Control, None→Data_Exch,
   56/57/59→опционально, иначе→RS
8. если fcv: сохранить fcb, sap, ответ в last_reply
```

---

## 5. Протокол (справочник)

### 5.1 Матрица

| Запрос | SAP | Ответ |
|--------|-----|-------|
| FDL_Status | — | Data `Response{Slave, Ok}`, пустой PDU |
| Set_Prm | 61 | **SC (0xE5)** всегда |
| Chk_Cfg | 62 | **SC (0xE5)** всегда |
| Slave_Diag | 60 | Data `Response{Slave, Ok}`, dsap=62, ssap=60, PDU 6 б + ext |
| Global_Control | 58, da=127 | **нет ответа** |
| Data_Exch | None | Data `Response{Slave, DL/DH}` c PI_I; PI_I пуст → SC |
| прочее SRD | — | Data `Response{Slave, RS}` |

Ответ: `da = req.sa`, `sa = self.address`, `resp.dsap = req.ssap`,
`resp.ssap = req.dsap`.

Ошибки Set_Prm/Chk_Cfg — **только битами диагностики**, не сменой ответа:
master в `WaitForParam`/`WaitForConfig` принимает только SC
(`src/dp/peripheral.rs`).

### 5.2 Set_Prm PDU

```
[0]     Station Status: bit7 Lock_Req, bit5 Sync_Req, bit4 Freeze_Req, bit3 WD_On
[1]     WD_Fact_1    (если WD_On)
[2]     WD_Fact_2    (если WD_On)
[3]     Min_TSDR (биты, >= 11)
[4],[5] Ident_Number (BE)
[6]     Group_Mask
[7..]   User_Prm_Data
```
Всегда SC. Иначе: ident/prm не сошлись → `prm_fault=true`, остаться в WPRM.
Ок → `prm_fault=false, prm_req=false, state=WCFG, groups=pdu[6]`,
`master_address=req.sa`, сохранить WD и `min_tsdr`.

### 5.3 Chk_Cfg

Всегда SC. `pdu != cfg.config` → `cfg_fault=true`, остаться в WCFG.
Ок → `cfg_fault=false, state=DXCHG`.

### 5.4 Slave_Diag

Master читает: `pdu[0..2]` = u16 **LE** флагов, `pdu[3]` = master (255=нет),
`pdu[4..6]` = ident **BE**, `pdu[6..]` = ext-diag.

```
[0] = status1 (младший байт DiagnosticFlags::bits())
[1] = status2 (старший байт)
[2] = status3: bit0 = Ext_Diag_Overflow
[3] = master_address.unwrap_or(255)
[4..6] = ident.to_be_bytes()
[6..] = ext_diag[..len]
```

| Бит | Имя | Когда |
|-----|-----|-------|
| s1.b1 | STATION_NOT_READY | `state != Dxchg` |
| s1.b2 | CONFIGURATION_FAULT | `cfg_fault` |
| s1.b3 | EXT_DIAG | `ext_diag_len > 0` |
| s1.b6 | PARAMETER_FAULT | `prm_fault` |
| s2.b0 | PARAMETER_REQUIRED | `prm_req` (reset, watchdog) |
| s2.b1 | STATUS_DIAGNOSTICS | `ext_diag_len > 0` |
| s2.b2 | PERMANENT_BIT | **всегда 1** |
| s2.b3 | WATCHDOG_ON | `wd_on` |
| s2.b4/5 | FREEZE/SYNC_MODE | не используем |

После reset: `Wprm, prm_req=true, master=None`.

### 5.5 Data_Exch, FDL-status

Data_Exch в DXCHG: `pi_q.copy_from_slice(dt.pdu)` (если длина совпала);
ответ `DL`, либо `DH` если `diag_pending` (сбросить); PDU = `pi_i`;
`pi_i.is_empty()` → SC. Не в DXCHG → `RS`.

FDL-status (`is_fdl_status_request()`, `da == address`) → Data
`Response{Slave, Ok}`, пустой PDU (нужен для `live_list`/gap-poll).

### 5.6 Global_Control (Felser `telegrammformate_2.html`)

`da=127`, SAP=58, SDN, PDU = 2 байта.

| Бит control | Значение |
|---|---|
| 0, 6, 7 | 0 (reserved) |
| 1 | Clear_Data |
| 2 | UNFREEZE |
| 3 | FREEZE |
| 4 | UNSYNC |
| 5 | SYNC |

`pdu[1]`: бит0=Group1 … бит6=Group7, бит7=Group8; `0` = все группы.
Ответа нет. Clear → `pi_q.fill(0)`. SYNC/FREEZE — игнорировать.

### 5.7 FCB

- `fcv==false` (`FrameCountBit::Inactive`/`First`) → дедупа нет.
  Первый Set_Prm и его retry — с `First` (fcv=0).
- `fcv==true` и `(sap, fcb)` совпал с последним → вернуть сохранённый ответ.
- Master гоняет **один** FCB через все SAP → достаточно одного слота кэша.

### 5.8 Watchdog

`wd_on && state==Dxchg && now - wd_timestamp > f1*f2*10 мс` →
`fail_safe ? pi_q.fill(0)`, `state=Wprm`, `prm_req=true`, `diag_pending=true`.
`wd_timestamp = now` при каждом телеграмме с нашим адресом и при Data_Exch.

---

## 6. Профиль slave

```
address = 3 и 4
ident   = 0x00F0
prm     = [0x20]
cfg     = [0x42,0x00,0xBE,0x81,   // IB IL 24 DI 8  -> 1 байт входов
           0x82,0x00,0xBD,0x81]   // IB IL 24 DO 8  -> 1 байт выходов
pi_i_len = 1   // кнопка: bit0
pi_q_len = 1   // LED: bit0
```
Свой минимальный набор байт тоже допустим (сравнение на равенство), но реальные
идентификаторы модулей осмысленнее и совместимы с gsdtool.

Пины (пример): slave 3 — LED GPIO6, кнопка GPIO8; slave 4 — LED GPIO7,
кнопка GPIO9. Если кнопка одна — завести её в `pi_i` всех slave'ов (или разные
биты одного байта). Кнопка: GPIO с pull-up на GND, active low. UART: GPIO0 TX,
GPIO1 RX, GPIO2 dir (при прямом соединении не используется).

Master-пример: `user_parameters = Some(&[0x20])`, `config = cfg выше`,
`pi_i = 1`, `pi_q = 1`; в цикле писать мигалку в `pi_q[0]`, печатать `pi_i[0]`.

---

## Приложение B. Тесты

### B.1 Unit (`src/dp/slave.rs`)

1. diag до Set_Prm: `Prm_Req|Station_Not_Ready|Permanent`, master=255.
2. Set_Prm ок → SC, WCFG, master записан; плохой ident → SC, WPRM, Prm_Fault.
3. Chk_Cfg ок → SC, DXCHG; плохой → SC, Cfg_Fault, WCFG.
4. Diag в DXCHG: `Station_Not_Ready=0`, master/ident верные.
5. Data_Exch: PDU == pi_i; pi_q обновился из запроса; пустой pi_i → SC.
6. Data_Exch в WPRM → RS.
7. Повтор с тем же FCB (fcv=1) → тот же ответ, даже если pi_i изменили.
8. Смена FCB → новый ответ.
9. Global_Control: 0 байт, фильтр по group mask, Clear обнуляет pi_q.
10. FDL-status: 6 байт, `Slave/Ok`.
11. Watchdog → WPRM, Prm_Req, pi_q=0.
12. `set_ext_diag` → следующий Data_Exch = DH; diag разбирается
    `ExtendedDiagnostics::iter_diag_blocks`.

### B.2 Интеграция на `SimulatorPhy`

1. 1 slave: `is_running()`, `cycle_completed`.
2. Эхо: `pi_q` master → `pi_i` slave → `pi_i` master.
3. N=4: разные адреса, все running.
4. ext-diag → `PeripheralEvent::Diagnostics`.
5. `prm_fault` → `ParameterError`; `cfg_fault` → `ConfigError`.
6. Дроп каждого k-го ответа → мастер выходит на связь через retry.
7. Принудительный RS → ValidateConfig → восстановление.
8. Watchdog → Prm_Req → перепараметризация.

---

## Приложение C. Открытые вопросы

1. Пины LED/кнопок на твоей плате — какие GPIO свободны?
2. Достаточно ли N=2, или сразу закладывать 4?
3. Прямое UART-соединение или всё же трансиверы на обоих концах?
4. Нужен ли шаг 6 (инъекция отказов на железе) или хватит host-тестов?
