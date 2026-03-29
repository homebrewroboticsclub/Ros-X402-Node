# DEV_AI — контекст для агентов (rospy_x402)

## Единая точка входа экосистемы (KYR + x402 + teleop)

Запуск, RAID, частые команды и индекс доков связки: **[../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md)**, **[../br_bringup/README.md](../br_bringup/README.md)**.

## Назначение

ROS 1 (Noetic) пакет: REST API над возможностями робота с оплатой по протоколу **x402** (Solana), ROS-сервис исходящих платежей, библиотека `rospy_x402.x402`.

## Структура

- Нода: `scripts/x402_ex_server.py` → модули в `src/rospy_x402/` (`server.py`, `config.py`, `x402/*`, демо-callables).
- Конфиг эндпоинтов: `config/endpoints.example.json`.
- Документация: [DOC/README.md](DOC/README.md).

## Обязанности при правках

1. **Документация** — синхронно обновлять `DOC/*` (архитектура, протокол, диаграммы). Новый крупный блок (например, новый тип `ros_action`) — отдельный файл в `DOC/` + ссылки в `DOC/README.md`, [README.md](README.md), этом файле.
2. **Тесты** — новый функционал сопровождать тестами; прогон всего пакета:
   ```bash
   cd /home/ubuntu/ros_ws && source devel/setup.bash
   catkin_make run_tests --pkg rospy_x402
   ```
   При отсутствии зарегистрированных тестов в CMake — добавить `rostest`/`pytest` и включить в `catkin_add_nosetests` или аналог.
3. **Коммит** — осмысленные сообщения (что изменилось, влияние на API/конфиг).

## Полезные ссылки

- [DOC/ARCHITECTURE.md](DOC/ARCHITECTURE.md)
- [DOC/X402_PROTOCOL.md](DOC/X402_PROTOCOL.md)
