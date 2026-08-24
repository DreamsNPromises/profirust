# gsdtool Guide

CLI-утилита для работы с GSD-файлами PROFIBUS, в первую очередь в контексте
стека `profirust`. Построена поверх крейта `gsd-parser`, который выполняет
весь собственно разбор.

### Установка

```bash
cargo install gsdtool
```

### Команды

#### `gsdtool dump <file.gsd>`

Разбирает GSD-файл и выгружает полученный `GenericStationDescription` как
debug-вывод Rust (`{:#?}`). Удобно, чтобы посмотреть, что извлёк парсер.

#### `gsdtool config-wizard <file.gsd>`

Интерактивно генерирует конфигурацию периферии для profirust:

1. Показывает предупреждения разбора, если они есть.
2. Спрашивает глобальные параметры устройства через промпты `dialoguer` —
   списки выбора для параметров с записями `PrmText`, проверяемый численный
   ввод для диапазонов `MinMax`. Невидимые/неизменяемые параметры
   пропускаются.
3. Проходит по слотам (fuzzy-select). Для модульных станций выбирается до
   `max_module` модулей; компактные станции автоматически берут свой единственный
   модуль. Параметры каждого выбранного модуля спрашиваются так же.
4. Считает потоки config/PRM байтов через `gsd-parser::PrmBuilder` и выводит
   размеры буферов I/O из config-байтов (флаги byte/word, input/output).
5. Печатает готовый к вставке Rust-код:

```rust
let options = profirust::dp::PeripheralOptions {
    ident_number: 0x4711,
    user_parameters: Some(&[...]),
    config: Some(&[...]),
    max_tsdr: match BAUDRATE { ... },
    ...
};
let mut buffer_inputs = [0u8; 4];
let mut buffer_outputs = [0u8; 4];
let mut buffer_diagnostics = [0u8; 57];
```

#### `gsdtool diagnostics <file.gsd>`

Декодирует device-based расширенную диагностику. Вставьте сырые байты
диагностики (принимается форма среза `fmt::Debug`, например `[160, 0, ...]`).
Утилита переводит их в битовую проекцию (`bitvec`) и интерпретирует по секции
`Unit_Diag` из GSD:

- **bits**: текст для каждого установленного диагностического бита;
- **not_bits**: текст для каждого *сброшенного* бита, объявленного там;
- **areas**: диапазон бит извлекается как целое и ищется его значение.

### Пример сессии

<pre><font color="#A6E22E"><b>❯</b></font> gsdtool diagnostics si0380a7.gsd
Diagnostics Data (as fmt::Debug slice): [160, 0, 0, 32, 72, 100, 255, 255, 255, 255, 255, 255, 0, 41, 0, 128, 0, 0]

Bit 29: ====== Segment DP2 ======
Bit 127: A/B shorted, too much resist.
Area 40-47: 100 = Reflection error rate: 100%
</pre>
