# Интеграция с RAID App (робот)

Контракты HTTP на стороне RAID описаны в репозитории `x402_raid_app`. Здесь — что делает пакет **rospy_x402** и какие параметры задавать.

## Порядок шагов

1. **Enroll (A)** — `POST /api/robots/enroll` с флот-секретом; в ответе `id` (robot UUID) и `teleopSecret`. При повторе с тем же `enrollmentKey` запись обновляется, `id` стабилен.
2. **Help (B)** — `POST /api/robots/{robotId}/teleop/help` с `X-Robot-Teleop-Secret` (реализовано в `EscalationManager`).
3. **Push allowlist (опционально)** — RAID вызывает **точный** URL из `operatorRegistryUrl` при sync: `POST` с заголовком `X-Raid-To-Robot-Secret` и телом `{"allowedTeleoperatorIds": ["uuid", ...]}`. Внешняя спецификация: `ROBOT_OPERATOR_SYNC.md` в RAID.

Если на RAID **не задан** `ROBOT_FLEET_ENROLLMENT_SECRET`, enroll вернёт **503** — это конфигурация сервера.

## Когда робот саморегистрируется (авто-enroll)

Запрос **выполняется один раз при старте** ноды `x402_ex_server`, **до** поднятия HTTP-сервера x402, и **только если** после шагов ниже всё ещё нет пары `robotId` + `teleopSecret`:

1. Взяты непустые `~raid_robot_id` и `~raid_teleop_secret` (или `RAID_ROBOT_ID` / `RAID_TELEOP_SECRET` в окружении) — тогда enroll **не** вызывается.
2. Иначе прочитан `~/.ros/raid_robot_state.json` (или `RAID_STATE_FILE`) — если там есть `id` и `teleopSecret`, enroll **не** вызывается.
3. Иначе, если заданы **и** флот-секрет **и** `RAID_ENROLLMENT_KEY` **и** `raid_enroll_host` / `RAID_ENROLL_HOST`, выполняется `POST …/api/robots/enroll`.

Повторный старт с уже сохранённым state **не** дергает RAID для enroll, пока вы не удалите файл состояния и не сбросите rosparam.

### Уже зарегистрирован в RAID (тот же `enrollmentKey`)

На стороне RAID enroll **идемпотентен**: тот же ключ обновляет ту же запись, **`id` не меняется**. Наша нода при **успешном** enroll всегда перезаписывает state-файл актуальными `id` и `teleopSecret` из ответа (если RAID вернёт новый `teleopSecret`, он сохранится).

Если робот **не** вызывает enroll (потому что кредиты уже из launch или из state), а вы на RAID сменили секреты или удалили робота, старый `teleopSecret` в state может стать невалидным — тогда нужно либо выставить новые значения в launch/env, либо удалить state и перезапустить ноду при доступном RAID (и корректном `ROBOT_FLEET_ENROLLMENT_SECRET`).

### RAID недоступен

| Ситуация | Поведение |
|----------|-----------|
| **Первый** запуск: нет state, enroll нужен, RAID не отвечает | В логе `RAID enroll failed: …`. Пара `robotId`/`teleopSecret` остаётся пустой; нода всё равно стартует. `teleop/help` и эскалация будут падать, пока не появятся кредиты (ручной ввод, успешный enroll позже). |
| **Повторный** запуск: state на диске уже есть, enroll не вызывается | Старт без обращения к RAID. Кредиты для help берутся из файла. **Вызов** `teleop/help` к RAID при эскалации всё равно требует, чтобы RAID был доступен в момент вызова. |
| Первый запуск: enroll упал, **потом** RAID снова доступен | Перезапустите `x402_ex_server` (или весь launch): при отсутствии валидной пары снова попытка enroll. Либо один раз задайте `raid_robot_id` / `raid_teleop_secret` вручную. |

Ретраев с backoff при старте **нет** — одна попытка enroll за запуск. При необходимости backoff добавляют на уровне systemd/скрипта обёртки.

Пример значений для тестового стенда (должны совпадать с `.env` на сервере RAID): см. [`.env.example`](../.env.example) в пакете `rospy_x402` (`ROBOT_FLEET_ENROLLMENT_SECRET`, `RAID_TO_ROBOT_SECRET`).

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
| `~raid_app_url` / `RAID_APP_URL` / база URL | Все вызовы к RAID. Приоритет: непустой rosparam, иначе `RAID_APP_URL` из `.env`, иначе умолчание `http://192.168.20.53:3000` (резерв при сбое mDNS). Для mDNS: `raid_app_url:="http://raid-app.local:3000"` или `RAID_APP_URL=http://raid-app.local:3000`. |
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
