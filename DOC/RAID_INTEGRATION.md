# Интеграция с RAID App (робот)

Контракты HTTP на стороне RAID описаны в репозитории `x402_raid_app`. Здесь — что делает пакет **rospy_x402** и какие параметры задавать.

## Порядок шагов

1. **Enroll (A)** — `POST /api/robots/enroll` с флот-секретом; в ответе `id` (robot UUID) и `teleopSecret`. При повторе с тем же `enrollmentKey` запись обновляется, `id` стабилен.
2. **Help (B)** — `POST /api/robots/{robotId}/teleop/help` с `X-Robot-Teleop-Secret` (реализовано в `EscalationManager`).
3. **Push allowlist (опционально)** — RAID вызывает **точный** URL из `operatorRegistryUrl` при sync: `POST` с заголовком `X-Raid-To-Robot-Secret` и телом `{"allowedTeleoperatorIds": ["uuid", ...]}`. Внешняя спецификация: `ROBOT_OPERATOR_SYNC.md` в RAID.

Если на RAID **не задан** `ROBOT_FLEET_ENROLLMENT_SECRET`, enroll вернёт **503** — это конфигурация сервера.

## Персистентность

По умолчанию файл состояния: `~/.ros/raid_robot_state.json` (поля `id`, `teleopSecret`). Переопределение: переменная окружения `RAID_STATE_FILE` или rosparam `~raid_state_file`.

Список операторов после sync: `~/.ros/raid_operator_allowlist.json` (рядом со state), либо `RAID_ALLOWLIST_FILE` / `~raid_allowlist_file`.

## Приоритет учётных данных

1. Непустые rosparam `~raid_robot_id` и `~raid_teleop_secret` (и/или `RAID_ROBOT_ID`, `RAID_TELEOP_SECRET` в окружении).
2. Иначе чтение сохранённого state-файла.
3. Иначе **авто-enroll**, если заданы **оба**: `ROBOT_FLEET_ENROLLMENT_SECRET` (или `~robot_fleet_enrollment_secret`) и `RAID_ENROLLMENT_KEY` (или `~raid_enrollment_key`), плюс **LAN-доступный** `~raid_enroll_host` / `RAID_ENROLL_HOST` (тот же адрес, по которому RAID достучится до HTTP робота, например до `GET /health` на порту REST x402).

## Таблица параметров и env

| Имя | Назначение |
|-----|------------|
| `~raid_app_url` / база URL | Все вызовы к RAID. В LAN при mDNS на RAID: часто `http://raid-app.local:3000`. |
| `ROBOT_FLEET_ENROLLMENT_SECRET` / `~robot_fleet_enrollment_secret` | Только enroll и прочие мутации `/api/robots` с флот-авторизацией на стороне RAID. |
| `RAID_ENROLLMENT_KEY` / `~raid_enrollment_key` | Стабильный ключ устройства (идемпотентный enroll). |
| `~raid_enroll_host`, `RAID_ENROLL_HOST` | Хост в теле enroll (`host`) — не `localhost`, если RAID на другой машине. |
| `~raid_enroll_http_port` | Порт HTTP робота в теле enroll (`port`); по умолчанию совпадает с портом REST x402 из `endpoints` JSON. |
| `~raid_enroll_rosbridge_host`, `~raid_enroll_rosbridge_port` | Поля `rosbridgeHost` / `rosbridgePort` в enroll (по умолчанию хост как у HTTP, порт **9090**). |
| `~raid_robot_name` | Опционально `name` в enroll. |
| `RAID_TO_ROBOT_SECRET` / `~raid_to_robot_secret` | Проверка входящего `X-Raid-To-Robot-Secret` на эндпоинте sync. |
| `~raid_operator_sync_path` | Путь POST на REST x402 (по умолчанию `/raid/operator-allowlist`). Полный `operatorRegistryUrl` при enroll: `http://<raid_enroll_host>:<raid_enroll_http_port><path>`. |

Секреты **не** коммитить; использовать `~/.rospy_x402.env`, `ROSPY_X402_ENV_FILE` или systemd `Environment=`.

## Поведение teleop/help

Успех: HTTP **200** или **201**. Ответ **200** с `duplicate: true` — заявка уже открыта, обрабатывается как успех. **401** — неверные `robotId` / `teleopSecret`, ретраи бессмысленны до исправления конфигурации или повторного enroll.

## Входящий WebSocket (rosbridge)

JWT оператора до rosbridge не доходит; RAID пробрасывает заголовки/query с UUID оператора. Стандартный rosbridge их не проверяет — см. [ROSBRIDGE_AND_RAID.md](../../br-kyr/DOC/ROSBRIDGE_AND_RAID.md) в KYR.
