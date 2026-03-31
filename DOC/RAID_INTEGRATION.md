# Интеграция с RAID App (робот)

Контракты HTTP на стороне RAID описаны в репозитории `x402_raid_app`. Здесь — что делает пакет **rospy_x402** и какие параметры задавать.

## Порядок шагов

1. **Enroll (A)** — `POST /api/robots/enroll` с флот-секретом; в ответе `id` (robot UUID) и `teleopSecret`. При повторе с тем же `enrollmentKey` запись обновляется, `id` стабилен.
2. **Help (B)** — `POST /api/robots/{robotId}/teleop/help` с `X-Robot-Teleop-Secret` (реализовано в `EscalationManager`). Тело JSON и поле `metadata.situation_report`: см. [RAID_APP_TELEOP_HELP_SPEC.md](RAID_APP_TELEOP_HELP_SPEC.md).
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
| `~raid_app_url` / `RAID_APP_URL` / база URL | Все вызовы к RAID. Приоритет: непустой rosparam, иначе `RAID_APP_URL` из `.env`, иначе умолчание `http://raid-app.local:3000`. Если mDNS недоступен, задайте IP в launch или `.env`, например `http://192.168.1.100:3000`. |
| `ROBOT_FLEET_ENROLLMENT_SECRET` / `~robot_fleet_enrollment_secret` | Только enroll и прочие мутации `/api/robots` с флот-авторизацией на стороне RAID. |
| `RAID_ENROLLMENT_KEY` / `~raid_enrollment_key` | Стабильный ключ устройства (идемпотентный enroll). |
| `~raid_enroll_host`, `RAID_ENROLL_HOST` | Хост в теле enroll (`host`) — не `localhost`, если RAID на другой машине. |
| `~raid_enroll_http_port` | Порт HTTP робота в теле enroll (`port`); по умолчанию совпадает с портом REST x402 из `endpoints` JSON. |
| `~raid_enroll_rosbridge_host`, `~raid_enroll_rosbridge_port` | Поля `rosbridgeHost` / `rosbridgePort` в enroll (по умолчанию хост как у HTTP, порт **9090**). |
| `~raid_robot_name` | Опционально `name` в enroll. |
| `RAID_TO_ROBOT_SECRET` / `~raid_to_robot_secret` | Проверка входящего `X-Raid-To-Robot-Secret` на эндпоинте sync. |
| `~raid_operator_sync_path` | Путь POST на REST x402 (по умолчанию `/raid/operator-allowlist`). Полный `operatorRegistryUrl` при enroll: `http://<raid_enroll_host>:<raid_enroll_http_port><path>`. |

Секреты **не** коммитить; использовать `~/.rospy_x402.env`, `ROSPY_X402_ENV_FILE` или systemd `Environment=`.

**Почему нет enroll в логах:** файл `rospy_x402/.env` в `.gitignore` — на роботе после `git pull` его нет, пока не скопируете с машины разработки или не создадите из `.env.example`. Нужен пакет **python3-dotenv** (`sudo apt install python3-dotenv`). Нода мержит несколько `.env` подряд (пакет `config/.env`, пакет `.env`, cwd, `~/.rospy_x402.env`, `ROSPY_X402_ENV_FILE` — последний побеждает). Либо перед `roslaunch` выполните `export ROBOT_FLEET_ENROLLMENT_SECRET=...` и т.д. — дочерний процесс унаследует окружение.

## Поведение teleop/help

Успех: HTTP **200** или **201**. Ответ **200** с `duplicate: true` — заявка уже открыта, обрабатывается как успех. **401** — неверные `robotId` / `teleopSecret`, ретраи бессмысленны до исправления конфигурации или повторного enroll.

В теле запроса в `metadata` передаются `task_id`, `error_context` и текстовое поле **`situation_report`** (свободное описание состояния и причины эскалации). Контракт для RAID: [RAID_APP_TELEOP_HELP_SPEC.md](RAID_APP_TELEOP_HELP_SPEC.md).

Если в JSON ответа **`POST …/teleop/help`** сразу есть **`teleopGrantPayload`** + **`teleopGrantSignature`**, робот передаёт их в KYR. Иначе, при наличии **`id`** заявки и **`~raid_session_grant_poll`**, робот **поллит** `GET …/teleop/session-grant?helpRequestId=…` (после Accept оператора в RAID). При успехе — тот же поток в KYR. Иначе — фолбэк mock. Порядок шагов и ошибки RAID: [ROBOT_TELEOP_KYR_RAID_GRANT.md](ROBOT_TELEOP_KYR_RAID_GRANT.md). Полный цикл: [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md).

ROS: сервис `/x402/request_help` (`rospy_x402/RequestHelp`) — третье поле запроса `situation_report`.

## Оплата оператору: сессия закрыта, балансы SOL не изменились

Цепочка **`/teleop_fetch/end_session`** → `close_session` → **`/x402/complete_teleop_payment`** может завершиться **без перевода в сеть**:

1. **Частая причина:** до обновления RAID робот использует **mock-грант** с `operator_pubkey: "pending_from_raid"`. В **SignedReceipt** то же значение — `pay_operator_from_receipt_payload` **намеренно не шлёт SOL** (защита от отправки на невалидный адрес). Нужен ответ RAID с **`teleopGrantPayload` / подпись** и реальным **Solana base58** оператора ([RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md)).
2. **Логи:** после правок `teleop_fetch` пишет **WARN** `complete_teleop_payment: success but NO on-chain transfer` и в логах `rospy_x402` — `Operator payment skipped: receipt has placeholder operator_pubkey`. В логах `x402_ex_server` ищите `Sent x402 payment` / ошибки RPC.
3. **Проверка кошелька и RPC:** на кошельке робота должны быть **SOL** (сумма + комиссия), в `.env` / launch — рабочий **`SOLANA_PRIVATE_KEY`** и RPC (например Helius).
4. **Ручной тест перевода** (подставьте реальный pubkey оператора, одна строка JSON в `receipt_payload`):

```bash
rosservice call /x402/complete_teleop_payment "receipt_payload: '{\"operator_pubkey\":\"<OPERATOR_BASE58>\",\"started_at_sec\":0,\"ended_at_sec\":10}'"
```

При настроенном `teleop_operator_payment_flat_sol` на ноде `x402_server` уйдёт фиксированная сумма независимо от полей времени в JSON.

## Входящий WebSocket (rosbridge)

JWT оператора до rosbridge не доходит; RAID пробрасывает заголовки/query с UUID оператора. Стандартный rosbridge их не проверяет — см. [ROSBRIDGE_AND_RAID.md](../../br-kyr/DOC/ROSBRIDGE_AND_RAID.md) в KYR.
