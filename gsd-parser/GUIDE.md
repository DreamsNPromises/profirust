# gsd-parser Guide

Библиотека, разбирающая GSD-файлы (Generic Station Description) устройств
PROFIBUS в типизированную структуру Rust: `GenericStationDescription`.

## Как это работает под капотом

Разбор идёт в два этапа:

1. **Грамматика** — `src/gsd.pest` — PEG-грамматика, компилируемая на этапе
   сборки через `pest_derive`. Она токенизирует файл в дерево pest-пар:
   инструкции `PrmText`, `Module`, `SlotDefinition`, присваивания вида
   `Setting = value`.

2. **Семантика** — `parser.rs::parse_inner()` за один проход обходит это
   дерево и заполняет поля `GenericStationDescription`:
   - скалярные ключи маппятся напрямую (`9.6_supp` -> `SupportedSpeeds::B9600`,
     `maxtsdr_93.75` -> `MaxTsdr::b93750` и т.д.);
   - `Ext_User_Prm_Data_Ref/Const` собираются в `UserPrmData`
     (смещение + типизированное описание параметра). Если в файле есть только
     устаревшие `User_Prm_Data`/`User_Prm_Data_Len` — используется их вариант;
   - `Module` сохраняются по reference-номеру, после чего `SlotDefinition`
     разрешают свои ссылки (allowed/default) на эти модули.

Не каждая проблема фатальна. Некритичные несоответствия (слот ссылается на
несуществующий модуль, compact-станция объявила `Max_Module != 1`)
собираются как предупреждения — используйте
`parse_from_file_with_warnings()`. Жёсткие ошибки прерывают разбор с
`pest::error::Error`, содержащим путь к файлу, строку и столбец.

## Ключевые типы

| Тип                         | Назначение                                          |
|-----------------------------|-----------------------------------------------------|
| `GenericStationDescription` | Полное разобранное описание устройства              |
| `Module`                    | Периферийный модуль: имя + config-байты + PRM       |
| `Slot`                      | Слот станции: модуль по умолчанию + разрешённые     |
| `UserPrmData`               | Раскладка байтов пользовательских параметров        |
| `UnitDiag`                  | Диагностические биты/not-биты/области с текстами    |
| `SupportedSpeeds`           | Bitflags поддерживаемых скоростей                   |
| `MaxTsdr`                   | Max station delay responder для каждой скорости     |

## Сборка пользовательских параметров

`PrmBuilder` превращает раскладку `UserPrmData` в готовые байты параметров:
стартует с константных данных и значений по умолчанию всех параметров,
далее значения переопределяются:

```rust
let mut prm = gsd_parser::PrmBuilder::new(&gsd.user_prm_data)?;
prm.set_prm("Measuring units per revolution", 4096)?;
prm.set_prm_from_text("Code sequence", "Increasing clockwise (0)")?;
let bytes: Vec<u8> = prm.into_bytes();
```

Каждая запись проверяется на диапазон типа (`UserPrmDataType`) и ограничение
параметра (`MinMax` / набор значений).

## Использование

```toml
[dependencies]
gsd-parser = "0.6"
```

```rust
let (gsd, warnings) =
    gsd_parser::parse_from_file_with_warnings("device.gsd");
println!("{} от {}, ident 0x{:04x}",
    gsd.model, gsd.vendor, gsd.ident_number);
```

## Тесты

Снапшот-тесты построены на [insta.rs](https://insta.rs). Положите свои
`.gsd`-файлы в `tests/data/`, выполните `cargo insta test` и примите
снапшоты через `cargo insta review`; дальше обычный `cargo test` ловит
регрессии.
