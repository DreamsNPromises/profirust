# План: DP-V0 slave-симулятор на RP2040 (финальный)

Цель: вторая RP2040 на той же шине, что и тестируемый profirust-master
(`examples/rp-pico`), эмулирует N DP-V0 slave. Минимальный стенд: 2 слейва, у
каждого LED (выход master'а → LED слейва) и кнопка (вход слейва → master).
Проверка: доработанным `examples/rp-pico` + host-тестами на `SimulatorPhy`.

---

## 1. Зафиксированные решения

| Вопрос | Решение |
|--------|---------|
| Протокол | DP-V0. Без DP-V1, без SYNC/FREEZE |
| Библиотека | новый модуль `src/dp/slave.rs` (`DpSlave` + диспетчер `dispatch`) |
| Firmware | новый crate `examples/rp-pico-slave/` (копия обвязки из `examples/rp-pico`) |
| N слейвов | 2; структура допускает любое число (таблица + статические буферы) |
| Скорости | hardware smoke на B9600 (скорость текущего master-примера), затем B500000, затем 1.5/3/6/12 Мбит/с |
| Watchdog | реализуется (master-пример ставит 1 с; в host-тестах — 50–100 мс) |
| Clear_Data | парсится, обнуляет `pi_q`; отдельных тестов не требует |
| Инъекция отказов | в базовой версии не делается (отдельно позже, при необходимости) |
| Профиль слейва | см. §8 |
| Соединение | сначала прямое UART TX↔RX + GND (код от этого не зависит) |

---

## 2. Правила для исполнителя

- **Источник истины по протоколу — код master'а**:
  - `src/dp/peripheral.rs` — какие ответы принимаются в каких состояниях
    (главный ориентир!);
  - `src/dp/master.rs` — порядок запросов, Global_Control;
  - `src/fdl/telegram.rs` — форматы телеграмм/FC/FCB, `TelegramTx`;
  - `src/fdl/live_list.rs` — формат FDL-status обмена;
  - `src/dp/diagnostics.rs` — формат ext-diag блоков;
  - `src/phy/simulator.rs` — что паникует (тайминги шины);
  - `src/phy/rp2040.rs` — поведение Tset/dir-пина.
- **Не трогать**: `dp/master.rs`, `dp/peripheral.rs`, `fdl/*`, `phy/*`.
  Правки в `examples/rp-pico/src/main.rs` — только данные (адреса, размеры
  буферов, cfg, скорость, число периферий), не логика.
- Новый модуль — no_std-чистый (только core + `log`); буферы через
  `managed::ManagedSlice` (паттерн `Peripheral::new`).
- Приёмка каждого шага: `cargo fmt --check`, `cargo clippy -p profirust --
  -D warnings`, `cargo test -p profirust` (+ `--features phy-simulator` где
  уместно). Для firmware — сборка под `thumbv6m-none-eabi` (build.sh).
- Порядок шагов обязателен: каждый опирается на предыдущий и имеет
  независимую приёмку.

---

## 3. Архитектура

### 3.1 Библиотека (`src/dp/slave.rs`)

`DpSlave` — состояние одного виртуального слейва (без PHY, без участия в
token ring). `dispatch()` — диспетчер над срезом слейвов: телеграмму видят
все, ответ возвращает первый ненулевой (адреса уникальны; broadcast-SDN
ответов не требует).

### 3.2 Host-харнесс (интеграционные тесты на `SimulatorPhy`)

Три дубликата одного `SimulatorPhy`: `phy_ctrl` (владелец bus_time),
`phy_master`, **один** `phy_slave` на всё slave-устройство (как в firmware):

```
loop {
    now = phy_ctrl.advance_bus_time(1us);
    if !phy_master_busy {
        fdl_master.poll(now, &mut phy_master, &mut dp_master);
    }

    // телеграмма появляется на шине постепенно — поллим до полной
    if !phy_slave.poll_transmission(now) {
        let reply = phy_slave.receive_telegram(now, |res| match res {
            Ok(t)  => dp::slave::dispatch(&mut slaves, now, &t, &mut tx_buf),
            Err(_) => 0,                       // битая телеграмма — молчим
        }).unwrap_or(0);
        if reply > 0 {
            // +11 бит от конца TX master'а (иначе SimulatorPhy паникует)
            let now = phy_ctrl.advance_bus_time_min_tsdr();
            phy_slave.transmit_data(now, |buf| {
                buf[..reply].copy_from_slice(&tx_buf[..reply]); (reply, ())
            });
        }
    }
    for s in slaves.iter_mut() { s.tick(now); }
}
```

`receive_telegram` вернёт `Some` только когда последний байт «вышел на
линию», поэтому `advance_bus_time_min_tsdr()` после него даёт ровно
допустимые 11 бит.

### 3.3 Firmware (`examples/rp-pico-slave/`; core0 — горячий путь)

```
loop {
    let now = time::now();
    if phy.poll_transmission(now) { continue; }
    let n = phy.receive_telegram(now, |res| match res {
        Ok(t)  => dispatch(&mut slaves, now, &t, &mut tx_buf), // PHY тут не трогать!
        Err(_) => 0,
    }).unwrap_or(0);
    if n > 0 {
        wait_tsdr(now);              // см. §6
        phy.transmit_data(time::now(), |buf| {
            buf[..n].copy_from_slice(&tx_buf[..n]); (n, ())
        });
    }
    for s in slaves.iter_mut() { s.tick(now); }
    // LED <- pi_q; pi_i <- кнопки   (дёшево, без блокировок)
}
```

- `dispatch()` нельзя звать вне closure `receive_telegram`: телеграмма
  заимствует буфер PHY. Ответ копируется в локальный `tx_buf`, передача —
  после closure.
- Цикл не должен блокироваться дольше времени заполнения RX-FIFO UART
  (32 байта; на 12 Мбит/с ≈ 29 мкс). Никакого вывода в горячем пути; весь
  лог — через SPSC-очередь в core1 (как в master-примере).
- core1 — USB CDC + логи (копия `logger_atomic`-подхода из master-примера).

---

## 4. API `src/dp/slave.rs`

```rust
pub const MAX_REPLY_LEN: usize = 64;   // 6 (diag) + ext_diag; данные ≤ pi_i.len()

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SlaveState { #[default] Wprm, Wcfg, Dxchg }

#[derive(Debug, PartialEq, Eq)]
pub struct SlaveConfig<'a> {
    pub address: u8,
    pub ident_number: u16,
    /// None = принимать любые user-prm байты
    pub user_parameters: Option<&'a [u8]>,
    /// None = принимать любую конфигурацию
    pub config: Option<&'a [u8]>,
    /// при истечении watchdog обнулять pi_q
    pub fail_safe: bool,
}

pub struct DpSlave<'a> { /* … */ }

impl<'a> DpSlave<'a> {
    /// pi_i/pi_q — как в Peripheral::new: Into<managed::ManagedSlice<'a, u8>>
    pub fn new<PII, PIQ>(cfg: SlaveConfig<'a>, pi_i: PII, pi_q: PIQ) -> Self
    where
        PII: Into<managed::ManagedSlice<'a, u8>>,
        PIQ: Into<managed::ManagedSlice<'a, u8>>;

    pub fn address(&self) -> u8;
    pub fn state(&self) -> SlaveState;
    /// min TSDR (биты) из последнего Set_Prm; до параметризации — 11
    pub fn min_tsdr_bits(&self) -> u8;
    pub fn pi_i(&self) -> &[u8];
    pub fn pi_i_mut(&mut self) -> &mut [u8];
    pub fn pi_q(&self) -> &[u8];
    pub fn pi_q_mut(&mut self) -> &mut [u8];

    /// Длина ответа, записанного в `out`; 0 = не отвечать.
    /// Не-Data телеграммы (Token, SC, Response-Data) — игнор (0).
    pub fn handle_telegram(&mut self, now: Instant, t: &Telegram, out: &mut [u8]) -> usize;

    /// watchdog (см. §5.8)
    pub fn tick(&mut self, now: Instant);

    /// выставить ext-diag: следующий Data_Exch ответит DH, Slave_Diag отдаст байты
    pub fn set_ext_diag(&mut self, bytes: &[u8]);
    pub fn clear_ext_diag(&mut self);
}

/// Диспетчер: телеграмму получают все слейвы, возвращается первый ненулевой ответ.
pub fn dispatch(slaves: &mut [DpSlave], now: Instant, t: &Telegram, out: &mut [u8]) -> usize;
```

Поля `DpSlave`: `cfg, state, fcb, last_sap, last_reply[MAX_REPLY_LEN],
last_reply_len, has_last_reply, master_address, groups, min_tsdr_bits,
wd_on, wd_factors, wd_timestamp, prm_fault, cfg_fault, prm_req,
diag_pending, ext_diag[32], ext_diag_len, pi_i, pi_q`.

Алгоритм `handle_telegram` (только `Telegram::Data`; прочее → 0):

```
1. fc — Request? иначе → 0
2. req = FdlStatus && dt.da == address → ответ §5.5 (без FCB-кэша)
3. dt.da != address && dt.da != 127 → 0
4. sap = dt.dsap;  fcb/fcv из dt.fc
5. fcv && has_last_reply && fcb == self.fcb && sap == last_sap
       → вернуть сохранённый last_reply (запрос НЕ выполнять)
6. sap: 61 → Set_Prm §5.2;  62 → Chk_Cfg §5.3;  60 → Slave_Diag §5.4;
       58 && da == 127 → Global_Control §5.6 (ответа нет);
       None → Data_Exch §5.5;  прочее → RS
7. fcv → перезаписать кэш (fcb, sap, ответ)
8. любая телеграмма с dt.da == address → wd_timestamp = now
```

---

## 5. Протокольный справочник

### 5.1 Матрица запрос/ответ

| Запрос | SAP | Ответ |
|--------|-----|-------|
| FDL-status (`req = FdlStatus`, без SAP) | — | Data `Response{Slave, Ok}`, **без SAP, пустой PDU**, в любом состоянии |
| Set_Prm | 61 | **SC (0xE5)** всегда; ошибки — битами диагностики |
| Chk_Cfg | 62 | **SC (0xE5)** всегда; ошибки — битами диагностики |
| Slave_Diag | 60 | Data `Response{Slave, Ok}`, dsap=62, ssap=60, PDU ≥ 6 байт + ext |
| Global_Control (SDN, da=127) | 58 | **нет ответа** |
| Data_Exch | — (None) | Data `Response{Slave, DL}` с `pi_i`; `diag_pending` → `DH` (сбросить); пустой `pi_i` → SC; не в DXCHG → `RS` |
| прочий SRD | — | Data `Response{Slave, RS}` |

Адресация ответа: `da = req.sa`, `sa = self.address`,
`resp.dsap = req.ssap`, `resp.ssap = req.dsap`. (Master в
`do_await_data_response` требует `sa == слейв && da == master && fc =
Response`; SC принимается без адресов.)

### 5.2 Set_Prm (PDU)

```
[0]   Station Status: bit7 Lock_Req (ставится всегда — игнорировать),
      bit5 Sync_Req, bit4 Freeze_Req (игнорировать), bit3 WD_On
[1,2] WD_Fact_1/2      (если WD_On)
[3]   Min_TSDR (биты)  — сохранить
[4,5] Ident_Number (BE)
[6]   Group_Mask
[7..] User_Prm_Data
```
Всегда SC. Ident или user-prm не совпали с `cfg` → `prm_fault = true`,
остаться в WPRM (master узнает через Slave_Diag → `ParameterError`).
OK → `prm_fault=false, prm_req=false, state=WCFG, groups=pdu[6],
master_address=req.sa`, сохранить WD-факторы и `min_tsdr`. Повторный
Set_Prm в любом состоянии обрабатывается заново.

### 5.3 Chk_Cfg

Всегда SC. `pdu != cfg.config` → `cfg_fault = true`, остаться в WCFG (из WPRM
состояние не менять). OK → `cfg_fault=false`; в DXCHG переходить **только из
WCFG**. Слейв, отвергнувший Set_Prm, остаётся в WPRM: иначе он с `prm_fault`
формально готов к Data_Exch (см. B.5).

### 5.4 Slave_Diag (ответ)

```
[0],[1] DiagnosticFlags::bits() в LE (младший байт в [0])
[2]     status3: bit0 = Ext_Diag_Overflow (остальное 0)
[3]     master_address.unwrap_or(255)
[4..6]  ident.to_be_bytes()
[6..]   ext_diag[..ext_diag_len]
```

| Бит | Когда |
|-----|-------|
| s1.b1 STATION_NOT_READY | `state != Dxchg` |
| s1.b2 CONFIGURATION_FAULT | `cfg_fault` |
| s1.b3 EXT_DIAG | `ext_diag_len > 0` |
| s1.b6 PARAMETER_FAULT | `prm_fault` |
| s2.b0 PARAMETER_REQUIRED | `prm_req` (после сброса/WD) |
| s2.b1 STATUS_DIAGNOSTICS | `ext_diag_len > 0` |
| s2.b2 PERMANENT_BIT | **всегда 1** |
| s2.b3 WATCHDOG_ON | `wd_on` |

Формат ext-diag блоков — как парсит `ExtDiagBlockIter` (`src/dp/
diagnostics.rs`): заголовок = 2 старших бита тип (00 device, 01 identifier,
10 channel), 6 младших — длина блока с заголовком. Для симуляции достаточно
одного блока, напр. identifier-based `[0x42, 0x01]` («проблема у модуля 0»).

### 5.5 Data_Exch, FDL-status

- Data_Exch в DXCHG: если `pdu.len() == pi_q.len()` → скопировать в `pi_q`;
  иначе → `pi_q` не трогать, `cfg_fault = true`, ответ **RS**.
  Ответ: `DL` (или `DH` при `diag_pending` — сбросить), PDU = `pi_i`;
  пустой `pi_i` → SC.
- Data_Exch не в DXCHG → RS.
- FDL-status (в любом состоянии) → `Response{Slave, Ok}`, пустой PDU, без
  SAP-байт (нужно для live-list/gap-poll master'а).

### 5.6 Global_Control (SDN, da=127, SAP=58, PDU 2 байта)

`pdu[0]`: bit1 Clear_Data (→ `pi_q.fill(0)`), прочее игнорируем.
`pdu[1]` — групповая маска (0 = все; иначе применять при
`groups & pdu[1] != 0`). Ответа нет.

### 5.7 FCB

- `fcv == false` → дедупа нет (первый запрос master'а идёт с
  `FrameCountBit::First`).
- `fcv == true` и `(sap, fcb)` совпали с кэшем → вернуть сохранённый ответ,
  запрос не выполнять (даже если `pi_i` уже другой — это и есть семантика
  FCB). Смена FCB → выполнить и перезаписать кэш.
- Один слот кэша на слейва достаточно: master гоняет один FCB через все
  SAP, FCB циклится только после успешного ответа; retry шлёт тот же FCB.
- FDL-status в кэш не пишется (приходит с `FrameCountBit::Inactive`).

### 5.8 Watchdog

`wd_on && state == Dxchg && now - wd_timestamp > f1 * f2 * 10 мс` →
(`fail_safe ? pi_q.fill(0)`), `state=Wprm, prm_req=true, diag_pending=true`.
`wd_timestamp` обновляется любой корректной телеграммой с `da == address`
(любой SAP). Broadcast WD не сбрасывает.

**Важно для тестов:** пока master активен, WD не истечёт — он шлёт
diag-запросы даже Offline-перифериям. Срабатывание проверяется unit-тестом
(`tick` с подтянутым `now`) и host-сценарием «master замолчал».

---

## 6. Тайминги (felser.ch/profibus-manual/zeitverhaeltnisse.html)

| Биты | ≤187.5k | 500k | 1.5M | 3M | 6M | 12M |
|------|---------|------|------|----|----|-----|
| TSET | 1 | 1 | 1 | 4 | 8 | 16 |
| min TSDR | 11 | 11 | 11 | 11 | 11 | 11 |
| max TSDR | 60 | 100 | 150 | 250 | 450 | 800 |
| TSL | 100 | 200 | 300 | 400 | 600 | 1000 |

`Rp2040Phy::transmit_data` ставит `start_tx = now + ceil(Tset_bits/rate)`.
Первый байт ответа должен уйти не раньше min TSDR после последнего байта
запроса, поэтому в firmware перед `transmit_data`:

```
min_bits = max(11, min_tsdr из Set_Prm)     // см. DpSlave::min_tsdr_bits()
tset_us  = ceil(tset_bits  * 1e6 / rate)   // Baudrate::tset_bits()
need_us  = ceil(min_bits   * 1e6 / rate)
if tset_us < need_us { busy_wait(need_us - tset_us) }   // по таймеру, ≤ единицы мкс
```

При 12 Мбит/с Tset=16 ≥ 11 → паузы нет. На ≤3 Мбит/с пауза обязательна.
Ориентир `now` — момент чтения последнего байта из FIFO; реальная задержка
получится ≥ вычисленной (safe), запас до max TSDR большой (200 мкс на 500k).

---

## 7. Профиль стенда

```
address  = 3 и 4                (master = 2, как в примере)
ident    = 0x00F0               (совпадает с текущим master-примером)
user-prm = [0x20]
cfg      = [0x42, 0x00, 0xBE, 0x81,    // модуль DI8 → 1 байт входов
             0x82, 0x00, 0xBD, 0x81]    // модуль DO8 → 1 байт выходов
pi_i_len = 1   // кнопка: bit0
pi_q_len = 1   // LED: bit0
```

cfg — просто байты, сравнение на равенство с обеих сторон; подойдёт и любая
другая корректная последовательность той же длины.

Пины (заменить при необходимости — это константы в `slaves.rs`):
слейв 3 — LED GPIO6, кнопка GPIO8; слейв 4 — LED GPIO7, кнопка GPIO9.
Кнопка: GPIO с pull-up на GND, active low. UART: GPIO0 TX, GPIO1 RX,
GPIO2 dir (настраивать как в master-примере).

Master-пример (`examples/rp-pico/src/main.rs`): `user_parameters =
Some(&[0x20])`, `config` = cfg выше, `buffer_outputs = [0u8; 1]`; в цикле
писать мигалку/счётчик в `pi_q[0]`, печатать `pi_i[0]`.

---

## 8. Шаги

### Шаг 1. Библиотека: `DpSlave` + `dispatch` + unit-тесты

Файлы: `src/dp/slave.rs` (новый), `src/dp/mod.rs` (экспорт).
Реализовать §4–§5, тесты — приложение A.
Приёмка: `cargo test -p profirust slave`; clippy/fmt/doc чисто.

### Шаг 2. Host-интеграция: `DpMaster` ↔ N × `DpSlave` на `SimulatorPhy`

Файлы: интеграционные тесты (в `src/dp/slave.rs` или `tests/`).
Хэрнесс — §3.2. Сценарии — приложение B (watchdog у master'а настраивать
50–100 мс). Master — 4 периферии, `slot_bits` по умолчанию.
Приёмка: все сценарии B зелёные; `is_running()` на 4 слейвах,
`cycle_completed` идёт; soak (B.8) чистый.

### Шаг 3. Firmware-каркас: 1 слейв на реальной шине

Файлы: `examples/rp-pico-slave/` — новый crate: копия обвязки master-примера
(`time.rs`, `overclock.rs`, `logger_atomic.rs`, `panic_handler.rs`,
`build.rs`, `build.sh`, `memory.x`, `Cargo.toml`, `.cargo/config.toml`,
`Embed.toml`), `main.rs` — §3.3, один `DpSlave` (addr 3), pi_i/pi_q —
константы. Master-пример дорабатывается по §7.
Приёмка: сначала B9600, затем B500000 — в логе master'а
`Peripheral #3 becomes ready for data exchange`, `is_running() == true`,
`Inputs RAW` читается.

### Шаг 4. LED + кнопка одного слейва

Файлы: обе `main.rs`. Слейв: LED ← `pi_q[0] & 1`, `pi_i[0] = !button.is_low()`.
Приёмка: master пишет 0/1 → LED гаснет/горит; кнопка меняет `Inputs RAW`.

### Шаг 5. N слейвов (2)

Файлы: `examples/rp-pico-slave/src/slaves.rs` — таблица
`SlaveDef { address, led_pin, button_pin, ... }` + массив `DpSlave`,
построенный над статическими буферами (ManagedSlice); `main.rs`; master-пример
— 2 периферии.
Приёмка: оба слейва `is_running()`, LED и кнопки независимы.

### Шаг 6. 1.5 / 3 / 6 Мбит/с

`BAUDRATE` в обоих прошивках (у master-примера — `max_tsdr` в options
зависит от скорости, см. его match).
Приёмка: длительный прогон без retry/timeout, `statistics()` чистая.

### Шаг 7. 12 Мбит/с

240 МГц (overclock.rs уже так), Tset=16 ≥ 11 → пауза не нужна. Следить за
лагом цикла (лог `poll() rate` в master-примере). Замерить логическим
анализатором от конца запроса до первого байта ответа — должно лежать в
`[11, 800]` бит. Если цикл не успевает — RX-байты в IRQ-обработчик в
кольцевой буфер (минимальный ISR) как крайняя мера.
Приёмка: длинный прогон без потерь и retry; замеры в логе. CHANGELOG +
rustdoc с примером в `src/dp/slave.rs`.

---

## Приложение A — unit-тесты (`src/dp/slave.rs`)

Вход — собранные байты запроса, выход — байты ответа.

1. diag до Set_Prm: `Prm_Req | Station_Not_Ready | Permanent`, master=255;
   PERMANENT_BIT всегда установлен.
2. Set_Prm OK → SC, WCFG, master записан; плохой ident → SC, WPRM, prm_fault.
3. Chk_Cfg OK → SC, DXCHG; плохой → SC, cfg_fault, WCFG.
4. Diag в DXCHG: Station_Not_Ready=0, master/ident верные, dsap/ssap = 62/60.
5. Data_Exch: PDU ответа == pi_i; pi_q обновился из запроса; пустой pi_i → SC.
6. Data_Exch в WPRM и в WCFG → RS.
7. Не-Data телеграммы (Token, ShortConfirmation, Response-Data) → 0.
8. Повтор с тем же FCB (fcv=1) → тот же ответ, даже если pi_i изменили.
9. Смена FCB → новый ответ.
10. FDL-status в любом состоянии: 6-байтовая телеграмма, `Response{Slave, Ok}`,
    без SAP, пустой PDU.
11. Global_Control: ответа нет, фильтр по group mask, Clear обнуляет pi_q.
12. Watchdog (`tick` с подтянутым now) → WPRM, Prm_Req, pi_q=0 при fail_safe.
13. `set_ext_diag` → следующий Data_Exch = DH; блоки разбираются
    `ExtendedDiagnostics::iter_diag_blocks`.
14. Data_Exch с длиной PDU != pi_q.len() → RS, cfg_fault, pi_q нетронут.

## Приложение B — host-сценарии (`SimulatorPhy`)

1. 1 слейв: полный startup — Online → Configured → `is_running()`,
   `cycle_completed`.
2. Эхо: `pi_q` master → `pi_q` слейва; `pi_i` слейва → `pi_i` master.
3. N=4 с разными адресами: все running; токены/чужие телеграммы не мешают.
4. ext-diag → `PeripheralEvent::Diagnostics`.
5. prm_fault (несовпадение в SlaveConfig) → `ParameterError`;
   cfg_fault → `ConfigError`.
6. RS на Data_Exch (слейв сброшен в WPRM) → master ValidateConfig →
   восстановление.
7. Watchdog (50 мс): master замолчал на > WD-период → слейв WPRM/Prm_Req;
   master ожил → перепараметризация → DXCHG.
8. Soak: ~10k циклов — 0 retry/timeout/crc по `DpStatistics`.
