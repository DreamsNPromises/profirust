# План v3: DP-V0 slave-симулятор на RP2040

Этот документ — самодостаточная версия плана (переработка
`PLAN-dp-slave-simulator-v2.md`). Сначала — критическая оценка v2 (что
подтверждается кодом profirust, а что ошибочно или недоопределено), затем
исправленный план. Все ссылки на код проверены по текущему состоянию репозитория,
тайминги — по felser.ch/profibus-manual/zeitverhaeltnisse.html.

---

## Часть I. Критическая оценка плана v2

### I.1 Что в v2 верно и подтверждается кодом (оставляем без изменений)

1. **Ошибки Set_Prm/Chk_Cfg сообщаются только битами диагностики, ответ всегда
   SC.** Проверено: `Peripheral::receive_reply` в `src/dp/peripheral.rs` в
   состояниях `WaitForParam`/`WaitForConfig` принимает *только* `ShortConfirmation`,
   всё остальное игнорируется. Ошибка выясняется позже, в `ValidateConfig`,
   чтением `Slave_Diag`.
2. **Формат ответа Slave_Diag.** `dsap = 62 (SAP_MASTER_MS0)`, `ssap = 60
   (SAP_SLAVE_DIAGNOSIS)`, PDU: `[0..2]` — u16 флагов **LE** (master парсит
   `from_le_bytes`), `[3]` — адрес master'а (255 = нет), `[4..6]` — ident **BE**,
   `[6..]` — ext-diag. Совпадает с `Peripheral::handle_diagnostics_response`
   (проверяются и dsap, и ssap, и длина ≥ 6).
3. **RS (SapNotEnabled) на Data_Exch вне DXCHG.** Master в `DataExchange`
   реагирует на `ResponseStatus::SapNotEnabled` переходом в `ValidateConfig` —
   рабочий путь восстановления.
4. **FDL-status обязателен.** `LiveList`/gap-poll мастера рассылает
   `FdlStatus`-запросы на все адреса; ответ должен быть `Data` c
   `fc = Response{Slave, Ok}`, **без SAP-байт и с пустым PDU** (см. тест
   `live_list_application` в `src/fdl/live_list.rs` и `TelegramTx::
   send_fdl_status_response`). Ответ нужен в любом состоянии слейва.
5. **Ответ SC при Data_Exch с пустым pi_i** — master это явно обрабатывает.
6. **Таблица таймингов (§3 v2) верна** — сверена с Felser
   (zeitverhaeltnisse.html) и с `Baudrate::tset_bits()/tqui_bits()` в
   `src/lib.rs`: TSET 1/1/1/4/8/16, TQUI 0/0/0/3/6/9, min TSDR = 11, max TSDR
   60/100/150/250/450/800, TSL 100/200/300/400/600/1000.
7. **`advance_bus_time_min_tsdr()` — реальная ловушка.** `SimulatorBus::
   enqueue_telegram` паникует, если «не-slave» передаёт раньше, чем
   `timestamp + length*11 + min_tsdr` бит. Учесть обязательно.
8. **Одного слота FCB-кэша достаточно.** Master гоняет один FCB на все SAP
   (`Peripheral.fcb`), FCB циклится только после *успешного* ответа; retry
   шлёт тот же FCB. Кэш ключуется `(sap, fcb)`.
9. **Трюк «пауза перед TX» в firmware корректен.** `Rp2040Phy::transmit_data`
   действительно ставит `start_tx = now + tset_us` (ceil), поэтому busy-wait
   `ceil(min_tsdr_bits/rate) - tset_us` даёт нужную задержку; при 12 Мбит/с
   Tset = 16 бит ≥ 11 бит, пауза не нужна.
10. **Промежуточные выводы про матрицу SAP/ответов** (Set_Prm 61 → SC,
    Chk_Cfg 62 → SC, Global_Control 58/broadcast → без ответа, Data_Exch без
    SAP → DL/DH/SC/RS) — верны.
11. Разумная поэтапность и идея «источник истины — код master'а».

### I.2 Ошибки и пробелы v2 (исправлено в v3)

1. **Алгоритм `handle_telegram` не говорит, что делать с не-Data телеграммами.**
   `Telegram` — это enum: `Data`, `Token`, `ShortConfirmation`. Слейв на шине
   *видит* токены (master гоняет их по кольцу даже в одиночестве) и, если
   прошивка упадёт на `Token`, всё сломается. Диспетчер обязан явно
   игнорировать `Token`, `ShortConfirmation` и любые `Data` с
   `fc = Response` (master-to-master). В §5.9 ниже это зафиксировано.
2. **Не определено поведение при несовпадении длины PDU Data_Exch и `pi_q`.**
   «Копировать, если длина совпалась» — а если нет? В v3: не трогать `pi_q`,
   ответить RS и выставить `cfg_fault` — детектируемая и диагностируемая
   ошибка стенда (master уйдёт в ConfigError).
3. **Сценарий Watchdog (B.2.8) в v2 недостижим.** Master в состоянии
   `Offline` у периферии *продолжает* слать diag-запросы каждому циклу
   (`Peripheral::transmit_telegram`: Offline + retry_count==0 →
   diagnostics request). Значит WD слейва не истечёт, пока master жив — даже
   если слейв «молчит». Чтобы WD сработал, должен замолчать **master**
   (в host-тесте — перестать поллить и подвинуть bus_time). Кроме того,
   1-секундный watchdog из master-примера — это >1 с симулированного bus-time
   с шагом 1 мкс (миллион итераций); для host-теста WD надо настраивать
   коротким (50–100 мс).
4. **В host-харнессе (Шаг 2) не показано, что телеграмма «ещё в полёте».**
   После `fdl_master.poll()` телеграмма появляется на шине *постепенно*
   (`current_cursor()` отсчитывает байты по bus_time). Хэрнесс должен в цикле
   мелкими шагами двигать время и звать `receive_telegram` у слейва, пока не
   придёт полная телеграмма; только тогда TX master'а завершён и можно делать
   `advance_bus_time_min_tsdr()` + `transmit_data()`. Ровно 11 бит — хватает
   (проверка в симуляторе на строгое `<`, равенство допустимо).
5. **Против yourself: один SimulatorPhy на всё slave-устройство, а не по физу
   на слейва.** Диспетчер и так разруливает адреса; один PHY проще, точнее
   повторяет firmware (там PHY один) и не заставляет дренажировать N приёмных
   курсоров. В v2 wording «phy_slaves» вводит в заблуждение.
6. **API `DpSlave::new(cfg, pi_i: &mut [u8], pi_q: &mut [u8])` неудачен.**
   Голые `&mut`-заимствования делают таблицу из N слейвов над статическими
   буферами мучительной (unsafe `static mut` или `split_at_mut`-гимнастика).
   В остальном крейте принят паттерн `Into<managed::ManagedSlice<'a, u8>>`
   (`Peripheral::new`) — надо использовать его же: и `&mut [u8]` принимает, и
   массивы, и `Vec` в host-тестах.
7. **TSDR-пауза: противоречие §3 (фиксированные 11 бит) и §4
   (`min_tsdr_bits()` из Set_Prm).** В v3: ждать `max(11, min_tsdr из
   последнего Set_Prm)`; до параметризации — 11 (master из примера шлёт 11).
8. **Выпало из v1 ограничение FIFO UART (32 байта).** Главный цикл firmware не
   должен блокироваться дольше времени заполнения RX-FIFO (на 12 Мбит/с 32
   байта ≈ 29 мкс): USB-лог — только через SPSC-очередь (как в master-примере),
   никакой печати в горячем пути, LED/кнопки/tick — копеечные операции.
9. **Clear недостижим у master'а.** `DpMaster::enter_state` паникует
   (`todo!`) для любого состояния, кроме Operate, т.е. `enter_clear()` у
   master-примера вызвать нельзя, и «Clear → pi_q.fill(0)» на этом стенде
   никогда не проверится живьём против master'а. Слейву достаточно *парсить*
   бит Clear_Data (безопасно), но не надо строить вокруг него сценарии.
   SYNC/FREEZE — корректно выброшено (master их не шлёт, биты в его
   `Global_Control` для Operate всегда 0x00).
10. **Скорость первого железного smoke-теста.** v2 стартует с 500 кбит/с, но
    текущий master-пример отлажен на B9600. Первый hardware-прогон лучше делать
    на скорости существующего примера (минимум переменных), потом 500k,
    потом выше. Это замечание к порядку, не к цели.
11. **Не хватает общих сценариев приёмки:** soak-прогон на host-сим (десятки
    тысяч циклов, 0 retry/timeout/CRC), offline → re-online с
    перепараметризацией, нечувствительность к токенам/чужим телеграммам.
    Добавлены в B.2.
12. **Мелочи:** нет упоминаний `cargo fmt`/`cargo doc`/no_std-чистоты нового
    модуля; не сказано, что приём ошибок телеграммы (`Err` из
    `receive_telegram`) у слейва — просто «отбросить буфер, не отвечать»;
    Lock_Req (бит 7 station status, master ставит его всегда) — просто
    игнорируем, кроме запоминания адреса master'а.

**Вердикт:** v2 — сильный план, протокольная часть практически полностью
совпадает с ожиданиями master-кода. Основные правки: обработка не-Data
телеграмм, поведение при несовпадении длины, недостижимый сценарий WD,
уточнение host-харнесса (1 PHY, «полёт» телеграммы), ManagedSlice-API,
min-TSDR из Set_Prm, FIFO-ограничение, порядок скоростей. Всё остальное
заимствуется из v2 почти дословно.

---

## Часть II. План v3

Цель: вторая RP2040 на той же шине эмулирует N DP-V0 slave. Минимальный стенд:
2 слейва, на каждого LED (выход master'а → выход слейва) и кнопка (вход слейва
→ master). Проверка — доработанным `examples/rp-pico`, а также host-тестами
`DpMaster` ↔ N×`DpSlave` на `SimulatorPhy`.

### 1. Зафиксированные решения

| Вопрос | Решение |
|--------|---------|
| Протокол | DP-V0, без DP-V1, без SYNC/FREEZE |
| Clear_Data | парсим, обнуляем pi_q; живых тестов с этим master'ом нет (`enter_clear()` у него `todo!()`) |
| Железо | 2×RP2040. Для стенда допустимо прямое UART TX↔RX + GND, без трансиверов |
| Профиль слейва | ident `0x00F0`, user-prm `[0x20]`, cfg = DI8+DO8 (см. §7), 1 байт входов, 1 байт выходов |
| N | 2, расширяемо до 8 (таблица + статические буферы) |
| Скорости | hardware smoke — на скорости текущего master-примера (B9600), затем 500k, затем 1.5/3/6/12 Мбит/с |
| Библиотека | новый модуль `src/dp/slave.rs` (без отдельного fdl/passive); диспетчер — функция `dispatch` там же |
| Firmware | новый crate `examples/rp-pico-slave/` |
| I/O | LED = `pi_q[0].bit0`, кнопка = `pi_i[0].bit0` (active low) |
| Watchdog | реализуем (master-пример ставит 1 с); в host-тестах настраиваем 50–100 мс |

### 2. Правила для исполнителя

- **Источник истины по протоколу — код master'а**: `src/dp/peripheral.rs`
  (какие ответы принимаются в каких состояниях), `src/dp/master.rs`
  (порядок запросов), `src/fdl/telegram.rs` (форматы), `src/fdl/live_list.rs`
  (FDL-status), `src/phy/simulator.rs` (что паникует), `src/phy/rp2040.rs`
  (Tset). Спецификация Felser — вторичный справочник.
- Не трогать master-часть стека (`dp/master.rs`, `dp/peripheral.rs`,
  `fdl/*`). Правки в `examples/rp-pico/src/main.rs` — только данные
  (адреса, размеры буферов, cfg, скорость, N периферий), не логика.
- Новый модуль `src/dp/slave.rs` — no_std-чистый (только core + `log`);
  буферы через `managed::ManagedSlice`, как везде в крейте.
- Приёмка каждого шага: `cargo fmt --check`, `cargo clippy -- -D warnings`,
  `cargo test -p profirust` (+ `--features phy-simulator` где уместно),
  `cargo doc -p profirust --no-deps` без предупреждений. Для firmware —
  `cargo build --release` под `thumbv6m-none-eabi` (через build.sh примера).

### 3. Архитектура

#### 3.1 Библиотека (`src/dp/slave.rs`)

`DpSlave` — состояние одного виртуального слейва (без PHY). `dispatch()` —
диспетчер над срезом слейвов: отдаёт телеграмму всем, возвращает первый
ненулевой ответ (адреса уникальны, broadcast-SDN ответов не требует).

#### 3.2 Host-харнесс (интеграционные тесты)

Один `SimulatorPhy` «control» (владелец bus_time) + `phy_master` + **один**
`phy_slave_device` — дубликаты. Слейвы живут за одним физом, как в firmware:

```
loop {
    now = phy_ctrl.advance_bus_time(1us);
    if !phy_master_busy { fdl_master.poll(now, &mut phy_master, &mut dp_master); }

    // слейв-устройство: телеграмма приходит ПОСТЕПЕННО — поллим, пока полная
    if !phy_slave.poll_transmission(now) {
        reply = phy_slave.receive_telegram(now, |res| match res {
            Ok(t)  => dp::slave::dispatch(&mut slaves, now, &t, &mut tx_buf),
            Err(_) => 0,                       // битая телеграмма — молчим
        }).unwrap_or(0);
        if reply > 0 {
            now = phy_ctrl.advance_bus_time_min_tsdr();   // +11 бит от конца TX master'а
            phy_slave.transmit_data(now, |buf| { buf[..reply].copy_from_slice(&tx_buf[..reply]); (reply, ()) });
        }
    }
    for s in slaves.iter_mut() { s.tick(now); }
}
```

Важно: `receive_telegram` вернёт `Some` только когда последний байт «вышел на
линию» (см. `current_cursor()`), поэтому `advance_bus_time_min_tsdr()` после
него даёт ровно допустимые 11 бит (проверка симулятора — на строгое `<`).

#### 3.3 Firmware (`examples/rp-pico-slave/`, core0 — горячий путь)

```
loop {
    now = time::now();
    if phy.poll_transmission(now) { continue; }
    n = phy.receive_telegram(now, |res| match res {
        Ok(t)  => dispatch(&mut slaves, now, &t, &mut tx_buf),   // внутри closure PHY не трогать!
        Err(_) => 0,
    }).unwrap_or(0);
    if n > 0 {
        wait_tsdr(now, slaves_min_tsdr_bits());   // §6; только если Tset < min TSDR
        phy.transmit_data(time::now(), |buf| { buf[..n].copy_from_slice(&tx_buf[..n]); (n, ()) });
    }
    for s in slaves.iter_mut() { s.tick(now); }
    // LED <- pi_q; pi_i <- кнопки (копеечно, без блокировок)
}
```

`dispatch()` нельзя звать вне closure `receive_telegram` — телеграмма
заимствует буфер PHY; ответ копируется в локальный `tx_buf`, передача — после
closure.

core1 — USB CDC/логи (как в master-примере, `logger_atomic` + SPSC).
**Никакого вывода в горячем пути**: цикл не должен блокироваться дольше
времени заполнения RX-FIFO UART (32 байта; на 12 Мбит/с ≈ 29 мкс).

### 4. API `src/dp/slave.rs`

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
    /// при истечении watchdog обнулять pi_q (fail-safe поведение)
    pub fail_safe: bool,
}

pub struct DpSlave<'a> { /* … */ }

impl<'a> DpSlave<'a> {
    /// pi_i/pi_q — как в Peripheral::new: Into<managed::ManagedSlice<'a, u8>>
    pub fn new<PII, PIQ>(cfg: SlaveConfig<'a>, pi_i: PII, pi_q: PIQ) -> Self
    where PII: Into<managed::ManagedSlice<'a, u8>>,
          PIQ: Into<managed::ManagedSlice<'a, u8>>;
    pub fn address(&self) -> u8;
    pub fn state(&self) -> SlaveState;
    /// min TSDR (биты) из последнего Set_Prm, до параметризации — 11
    pub fn min_tsdr_bits(&self) -> u8;
    pub fn pi_i(&self) -> &[u8];
    pub fn pi_i_mut(&mut self) -> &mut [u8];
    pub fn pi_q(&self) -> &[u8];
    pub fn pi_q_mut(&mut self) -> &mut [u8];

    /// Длина ответа, записанного в `out`; 0 = не отвечать.
    /// Token/ShortConfirmation/Response-телеграммы игнорирует (0).
    pub fn handle_telegram(&mut self, now: crate::time::Instant, t: &Telegram, out: &mut [u8]) -> usize;

    /// watchdog: проверить истечение (см. §5.8)
    pub fn tick(&mut self, now: crate::time::Instant);
    /// выставить ext-diag; следующий Data_Exch ответит DH, а Slave_Diag отдаст байты
    pub fn set_ext_diag(&mut self, bytes: &[u8]);
    pub fn clear_ext_diag(&mut self);
}

/// Диспетчер: отдать телеграмму всем слейвам, вернуть первый ненулевой ответ.
pub fn dispatch(slaves: &mut [DpSlave], now: Instant, t: &Telegram, out: &mut [u8]) -> usize;
```

Состояние (поля): `cfg, state, fcb, last_sap, last_reply[MAX_REPLY_LEN],
last_reply_len, has_last_reply, master_address, groups, min_tsdr_bits, wd_on,
wd_factors, wd_timestamp, prm_fault, cfg_fault, prm_req, diag_pending,
ext_diag[32], ext_diag_len, pi_i, pi_q`. ≈130 Б + буферы на слейва; 8 слейвов —
единицы КБ, для RP2040 несущественно.

Алгоритм `handle_telegram` (только `Telegram::Data`; всё прочее — 0):

```
1.  dt = t.as_data() иначе → 0
2.  fc — Request? иначе → 0                    (чужие ответы/мастера — мимо)
3.  req = FdlStatus && dt.da == address → ответ §5.5 (без FCB-кэша)
4.  dt.da != address && dt.da != 127 → 0
5.  sap = dt.dsap;  fcb/fcv из dt.fc
6.  fcv && has_last_reply && fcb == self.fcb && sap == last_sap → вернуть last_reply
7.  sap: 61→Set_Prm §5.2, 62→Chk_Cfg §5.3, 60→Slave_Diag §5.4,
        58 && da==127 → Global_Control §5.6 (ответа нет),
        None → Data_Exch §5.5, прочее → RS
8.  fcv → сохранить (fcb, sap, ответ) в кэш
9.  любая телеграмма с dt.da == address → wd_timestamp = now   (§5.8)
```

### 5. Протокольный справочник

#### 5.1 Матрица

| Запрос | SAP | Ответ |
|--------|-----|-------|
| FDL-status (`req = FdlStatus`, без SAP) | — | Data `Response{Slave, Ok}`, **без SAP, пустой PDU**, в любом состоянии |
| Set_Prm | 61 | **SC (0xE5)** всегда; ошибки — битами диагностики |
| Chk_Cfg | 62 | **SC (0xE5)** всегда; ошибки — битами диагностики |
| Slave_Diag | 60 | Data `Response{Slave, Ok}`, `dsap=62, ssap=60`, PDU ≥ 6 байт + ext |
| Global_Control (SDN, da=127) | 58 | **нет ответа** |
| Data_Exch | — (None) | Data `Response{Slave, DL}` c `pi_i`; `diag_pending` → `DH` (сбросить); пустой `pi_i` → SC; не в DXCHG → `RS` |
| прочий SRD (55–59 и т.п.) | — | Data `Response{Slave, RS}` |

Адресация ответа: `da = req.sa`, `sa = self.address`,
`resp.dsap = req.ssap`, `resp.ssap = req.dsap` (master в
`do_await_data_response` требует `sa == слейв && da == мастер && fc = Response`;
SC принимается без адресов).

#### 5.2 Set_Prm (PDU)

```
[0]   Station Status: bit7 Lock_Req (master ставит всегда — игнорируем),
      bit5 Sync_Req, bit4 Freeze_Req (игнорируем), bit3 WD_On
[1,2] WD_Fact_1/2     (если WD_On)
[3]   Min_TSDR (биты; сохранить, использовать в firmware §6)
[4,5] Ident_Number (BE)
[6]   Group_Mask
[7..] User_Prm_Data
```
Всегда SC. Ident или user-prm не совпали с `cfg` → `prm_fault = true`,
остаться в WPRM (master узнает через Slave_Diag → `ParameterError`).
OK → `prm_fault=false, prm_req=false, state=WCFG, groups=pdu[6],
master_address=req.sa`, сохранить WD и `min_tsdr`.
Повторный Set_Prm в любом состоянии — просто обрабатывается заново
(так master перепараметризирует после сброса).

#### 5.3 Chk_Cfg

Всегда SC. `pdu != cfg.config` → `cfg_fault = true`, остаться в WCFG
(`ConfigError` у master'а). OK → `cfg_fault=false, state=DXCHG`.

#### 5.4 Slave_Diag (ответ)

```
[0],[1] флаги: DiagnosticFlags::bits() в LE (младший байт [0])
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
| s2.b2 PERMANENT_BIT | **всегда 1** (иначе master объявит диагностику некорректной!) |
| s2.b3 WATCHDOG_ON | `wd_on` |

После сброса/WD: `Wprm, prm_req=true, master=None`.
Формат ext-diag блоков — как парсит master (`src/dp/diagnostics.rs`,
`ExtDiagBlockIter`): заголовок = старшие 2 бита тип (00 device, 01
identifier, 10 channel), младшие 6 — длина блока с заголовком. Для
симуляции достаточно одного блока, напр. identifier-based `[0x42, 0x01]`
(модуль 0 «проблемный») или device-based `[0x00|len, ...]`.

#### 5.5 Data_Exch, FDL-status

- Data_Exch в DXCHG: если `pdu.len() == pi_q.len()` → скопировать в `pi_q`;
  иначе → `pi_q` не трогать, `cfg_fault = true`, ответ **RS** (см. I.2.2).
  Ответ: `DL` (или `DH` при `diag_pending` — сбросить), PDU = `pi_i`;
  `pi_i` пуст → SC.
- Data_Exch не в DXCHG → `RS`.
- FDL-status (в любом состоянии) → `Response{Slave, Ok}`, пустой PDU, без
  SAP-байт (нужно для live-list/gap-poll; проверено тестами `live_list.rs`).

#### 5.6 Global_Control (SDN, da=127, SAP=58, PDU 2 байта)

`pdu[0]`: bit1 Clear_Data, bit3 FREEZE, bit5 SYNC (у master'а для Operate
всегда 0x00). `pdu[1]` — групповая маска (0 = все). Ответа нет. Clear_Data →
`pi_q.fill(0)`. SYNC/FREEZE игнорируем. Блок памяти SYNC/FREEZE не заводим.

#### 5.7 FCB

- `fcv == false` → дедупа нет (первый запрос master'а идёт с `FrameCountBit::First`).
- `fcv == true` и `(sap, fcb)` совпали с кэшем → вернуть сохранённый ответ,
  запрос не выполнять (даже если `pi_i` уже другой — это и есть семантика FCB).
- Смена FCB → выполнить и перезаписать кэш. Один слот кэша на слейва
  достаточно (см. I.1.8). FDL-status в кэш не пишется (master шлёт его с
  `FrameCountBit::Inactive`).

#### 5.8 Watchdog

`wd_on && state == Dxchg && now - wd_timestamp > f1 * f2 * 10 мс` →
(`fail_safe ? pi_q.fill(0)`), `state=Wprm, prm_req=true, diag_pending=true`.
`wd_timestamp` обновляется любой корректной телеграммой с `da == address`
(любой SAP). Broadcast (da=127) WD **не** сбрасывает (неважно на практике —
master адресует слейвов чаще; зафиксировано как решение).

**Важно для тестов (см. I.2.3):** пока master жив, WD не истечёт — он шлёт
diag-запросы даже Offline-перифериям. Проверять срабатывание надо unit-тестом
(`tick` с подтянутым `now`) и host-сценарием «master замолчал».

#### 5.9 Что игнорируем

`Token` (слейвы не участвуют в кольце), `ShortConfirmation` (это ответ, слейв
их не получает от master'а, но десериализатор может отдать), любые `Data` с
`fc = Response`, `Data` с `da != address && da != 127`. Ошибки парсинга
(`Err` из `receive_telegram`) — отбросить буфер, не отвечать. SAP 55/56/57/59
→ RS (master их не использует).

### 6. Тайминги (Felser `zeitverhaeltnisse.html`; сверено с `Baudrate::*`)

| Биты | ≤187.5k | 500k | 1.5M | 3M | 6M | 12M |
|------|---------|------|------|----|----|-----|
| TQUI | 0 | 0 | 0 | 3 | 6 | 9 |
| TSET | 1 | 1 | 1 | 4 | 8 | 16 |
| min TSDR | 11 | 11 | 11 | 11 | 11 | 11 |
| max TSDR | 60 | 100 | 150 | 250 | 450 | 800 |
| TSL | 100 | 200 | 300 | 400 | 600 | 1000 |

`Rp2040Phy::transmit_data` ставит `start_tx = now + ceil(Tset_bits / rate)`.
Первый байт ответа должен уйти не раньше `min TSDR` после последнего байта
запроса, поэтому перед `transmit_data`:

```
min_bits = max(11, slaves_min_tsdr_bits())         // из Set_Prm, §5.2
tset_us  = ceil(tset_bits  * 1e6 / rate)
need_us  = ceil(min_bits   * 1e6 / rate)
if tset_us < need_us { busy_wait(need_us - tset_us) }   // ≤ пару мкс, timer-based
```

При 12 Мбит/с Tset=16 ≥ 11 → паузы нет. При 6M Tset=8 < 11 → wait ~0.5 мкс.
Ориентир `now` — момент чтения последнего байта из FIFO; реальная задержка
будет ≥ вычисленной (safe), но следить, чтобы суммарно не выйти за max TSDR
(на 500k — 200 мкс, запас огромный). На 12M обязательна верификация
логическим анализатором (Шаг 8).

### 7. Профиль стенда

```
address  = 3 и 4                (master = 2, как в примере)
ident    = 0x00F0               (совпадает с текущим master-примером)
user-prm = [0x20]              (как в текущем master-примере)
cfg      = [0x42, 0x00, 0xBE, 0x81,    // модуль DI8  → 1 байт входов
             0x82, 0x00, 0xBD, 0x81]    // модуль DO8  → 1 байт выходов
pi_i_len = 1   // кнопка: bit0
pi_q_len = 1   // LED: bit0
```

cfg — просто байты, сравнение на равенство с обеих сторон; подойдут и другие
последовательности (можно сгенерить через gsdtool). Пины (пример): слейв 3 —
LED GPIO6, кнопка GPIO8; слейв 4 — LED GPIO7, кнопка GPIO9. Кнопка: GPIO с
pull-up на GND, active low. UART: GPIO0 TX, GPIO1 RX, GPIO2 dir (при прямом
соединении не используется, но настраиваем как в master-примере).

Master-пример (`examples/rp-pico/src/main.rs`): `user_parameters =
Some(&[0x20])`, `config` = cfg выше, `buffer_outputs = [0u8; 1]`, в цикле
писать мигалку/счётчик в `pi_q[0]`, печатать `pi_i[0]`.

### 8. Пошаговый план

#### Шаг 1. Библиотека: `DpSlave` + `dispatch` + unit-тесты

Файлы: `src/dp/slave.rs` (новый), `src/dp/mod.rs` (экспорт `slave`).
Реализовать §4–§5, unit-тесты — приложение B.1.
Приёмка: `cargo test -p profirust slave`, `clippy -D warnings`, `fmt`, `doc`.

#### Шаг 2. Host-интеграция: `DpMaster` ↔ N × `DpSlave` на `SimulatorPhy`

Файлы: тест в `src/dp/slave.rs` или `tests/slave_integration.rs`.
Хэрнесс — §3.2 (один slave-side PHY, «полёт» телеграммы, 11-битный сдвиг).
Сценарии — B.2. Для WD-сценария master'у настраивать короткий watchdog
(50–100 мс).
Приёмка: тесты зелёные; master доходит до `is_running()` на 4 слейвах,
`cycle_completed` идёт; soak (B.2.9) чистый.

#### Шаг 3. Firmware-каркас: 1 слейв на реальной шине

Файлы: `examples/rp-pico-slave/` (копия обвязки master-примера: `time.rs`,
`overclock.rs`, `logger_atomic.rs`, `panic_handler.rs`, `build.rs`,
`build.sh`, `memory.x`, `Cargo.toml`, `.cargo/config.toml`).
`main.rs` — §3.3, один `DpSlave` (addr 3), pi_i/pi_q — константы.
Master-пример дорабатывается по §7 (cfg, 1 байт выходов).
Приёмка: сначала на скорости текущего master-примера (B9600), затем B500000:
`Peripheral #3 becomes ready for data exchange`, `is_running() == true`,
`Inputs RAW` читается.

#### Шаг 4. LED + кнопка одного слейва

Файлы: обе `main.rs`. Слейв: LED ← `pi_q[0] & 1`, `pi_i[0] = !button.is_low()`.
Master: мигалка в `pi_q[0]`, печать `pi_i[0]`.
Приёмка: master пишет 0/1 → LED гаснет/горит; кнопка меняет `Inputs RAW`.

#### Шаг 5. N слейв (2, затем 4)

Файлы: `examples/rp-pico-slave/src/slaves.rs` (таблица `SlaveDef { address,
led_pin, button_pin, pi_i, pi_q }` + массив `DpSlave`, построенный из
статических буферов через ManagedSlice), `main.rs`; master-пример — N
периферий.
Приёмка: оба/все слейва `is_running()`, LED и кнопки независимы.

#### Шаг 6. Инъекция отказов (опционально для железа, обязательна для host-тестов)

Файлы: `examples/rp-pico-slave/src/faults.rs`: флаги `silent`,
`corrupt_reply_crc`, `delay_us`, `drop_every_n`, `force_rs`, `prm_fault`,
`cfg_fault`, `set_ext_diag`; переключение — USB-команды (core1 → SPSC →
core0) или кнопкой-сценариями. Применяются в `dispatch` после сборки ответа.
Приёмка на железе: master показывает Offline/retry/ParameterError/
ConfigError/Diagnostics. Базово всё это уже покрыто B.2 — шаг нужен, если
хочется гонять отказы на живой шине.

#### Шаг 7. 1.5 / 3 / 6 Мбит/с

`BAUDRATE` в обоих прошивках; следить за `tset/min-TSDR`-паузой (§6) и
лагом цикла (лог `poll() rate` уже есть в master-примере).
Приёмка: длительный прогон без retry/timeout; `statistics()` чистая.

#### Шаг 8. 12 Мбит/с + замеры

240 МГц (overclock.rs уже так), Tset=16 ≥ 11 → пауза не нужна. Замерить
логическим анализатором от конца запроса до первого байта ответа — должно
лежать в `[min TSDR, max TSDR] = [11, 800]` бит. Если цикл не успевает —
RX-байты в IRQ-обработчик в кольцевой буфер (минимальный ISR), как крайняя
мера.
Приёмка: длинный прогон без потерь и retry; замеры в логе.

Финал: запись в CHANGELOG, доки в `src/dp/slave.rs` (rustdoc + пример),
`cargo doc` чистый.

### 9. Приложение B — тесты

#### B.1 Unit (`src/dp/slave.rs`, telegram-уровень)

1. diag до Set_Prm: `Prm_Req | Station_Not_Ready | Permanent`, master=255,
   PERMANENT_BIT всегда установлен.
2. Set_Prm OK → SC, WCFG, master записан; плохой ident → SC, WPRM, prm_fault.
3. Chk_Cfg OK → SC, DXCHG; плохой → SC, cfg_fault, WCFG.
4. Diag в DXCHG: Station_Not_Ready=0, master/ident верные, dsap/ssap = 62/60.
5. Data_Exch: PDU ответа == pi_i; pi_q обновился из запроса; пустой pi_i → SC.
6. Data_Exch в WPRM → RS; в WCFG → RS.
7. **Не-Data телеграммы**: Token, ShortConfirmation, Response-Data → 0
   (ответа нет).
8. Повтор с тем же FCB (fcv=1) → тот же ответ, даже если pi_i изменили.
9. Смена FCB → новый ответ.
10. FDL-status в любом состоянии: 6-байтовая телеграмма, `Response{Slave, Ok}`,
    без SAP, пустой PDU.
11. Global_Control: 0 байт ответа, фильтр по group mask, Clear обнуляет pi_q.
12. Watchdog (`tick` с подтянутым now) → WPRM, Prm_Req, pi_q=0 при fail_safe.
13. `set_ext_diag` → следующий Data_Exch = DH; diag разбирается
    `ExtendedDiagnostics::iter_diag_blocks` (формат §5.4).
14. Data_Exch с длиной PDU != pi_q.len() → RS, cfg_fault, pi_q нетронут.

#### B.2 Host-интеграция (`SimulatorPhy`)

1. 1 слейв: полный startup — Online → Configured → `is_running()`,
   `cycle_completed` идёт.
2. Эхо: `pi_q` master → `pi_q` слейва; `pi_i` слейва → `pi_i` master.
3. N=4 с разными адресами: все running, токены и чужие телеграммы не
   ломают слейвов.
4. ext-diag → событие `PeripheralEvent::Diagnostics`.
5. prm_fault → `ParameterError`; cfg_fault → `ConfigError`.
6. Дроп каждого k-го ответа → retry вытянет (тот же FCB → тот же ответ);
   при k > retry-limit — Offline, затем самовосстановление через
   re-discovery (слейв жив) — перепараметризация.
7. Принудительный RS → ValidateConfig → восстановление.
8. Watchdog (короткий, 50 мс): **master замолчал** на > WD-период → слейв
   WPRM/Prm_Req; master ожил → перепараметризация → DXCHG.
9. Soak: ~10k циклов — 0 retry/timeout/crc по `DpStatistics`.

#### B.3 Hardware (стенд)

1. Шаги 3–4 приёмки §8 на B9600 и B500000.
2. Оба слейва на шине: независимые LED/кнопки.
3. (Опционально) прогон host-утилит из репозитория против стенда, напр.
   `examples/dp-scanner` через USB-RS485 — слейвы обнаруживаются.
4. 12 Мбит/с: замеры TSDR анализатором, soak без ошибок.

### 10. Открытые вопросы (ответить до Шага 5)

1. Какие GPIO под LED/кнопки на твоей плате свободны (конфликт с
   отладочными пинами logger_atomic GPIO6–8 у master-примера)?
2. N=2 сразу или сразу 4? (структура не меняется, вопрос только в буферах)
3. Прямое UART-соединение или трансиверы на обоих концах? (для >500k —
   лучше трансиверы)
4. Нужен ли Шаг 6 (инъекция отказов на железе) или хватит host-тестов?
