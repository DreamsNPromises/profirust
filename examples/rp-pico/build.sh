#!/usr/bin/env bash
set -euo pipefail

# Имя бинарного файла (можно передать как аргумент, по умолчанию dp-master-pico)
BINARY="${1:-dp-master-pico}"

# Директория скрипта
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔨 Сборка проекта..."
cargo build --release

# Путь к ELF-файлу
ELF_FILE="target/thumbv6m-none-eabi/release/$BINARY"
UF2_FILE="$BINARY.uf2"

# Конвертация
echo "🔄 Конвертация $ELF_FILE -> $UF2_FILE"
elf2uf2-rs "$ELF_FILE" "$UF2_FILE"

# Определение точки монтирования (попробуем /media/$USER/RPI-RP2, но можно подставить свою)
MOUNT_POINT="/media/$USER/RPI-RP2"
if [ ! -d "$MOUNT_POINT" ]; then
    # Альтернативный поиск через lsblk (например, для систем с /mnt)
    MOUNT_POINT=$(lsblk -o MOUNTPOINT -nr | grep -i "RPI-RP2" | head -1 || true)
    if [ -z "$MOUNT_POINT" ]; then
        echo "❌ Не удалось найти точку монтирования RPI-RP2. Подключите Pico в режиме bootloader (нажмите BOOTSEL + сброс)."
        exit 1
    fi
fi

# Копирование
echo "📂 Копирование $UF2_FILE в $MOUNT_POINT"
cp "$UF2_FILE" "$MOUNT_POINT/"

echo "✅ Готово! Pico прошивается."