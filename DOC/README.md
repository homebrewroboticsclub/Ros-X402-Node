# rospy_x402 — индекс документации

Документация по пакету в **`DOC/`**. Корневые [README.md](../README.md) и [DEV_AI.md](../DEV_AI.md) отсылают сюда.

## REST-сервер и архитектура кода

- [ARCHITECTURE.md](ARCHITECTURE.md) — узлы, модули `rospy_x402`, конфиги, точки расширения.

## Протокол x402 и интеграция

- [X402_PROTOCOL.md](X402_PROTOCOL.md) — V2: 402-ответ, discovery, верификация платежей, формат конфига.

## Диаграммы

- [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) — Mermaid: потоки и верификация.

## Планирование / спринт (робот-воркспейс)

- [SPRINT_STATUS_ROS_WORKSPACE.md](SPRINT_STATUS_ROS_WORKSPACE.md) — какие семафоры и тесты относятся к rospy_x402 и что проверено автотестами.

## Публикация репозитория

- Чеклист по всей четвёрке пакетов: [../../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md](../../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md).

## RAID App (enroll, help, allowlist)

- [RAID_INTEGRATION.md](RAID_INTEGRATION.md) — флот-секрет, enroll, персистентность, sync операторов, параметры ноды `x402_ex_server`.
- [RAID_APP_TELEOP_HELP_SPEC.md](RAID_APP_TELEOP_HELP_SPEC.md) — контракт `POST …/teleop/help` для разработчиков RAID (в т.ч. поле `situation_report`).
- [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md) — ответ RAID: подписанный SessionGrant, Solana `operator_pubkey`, пост-оплата на роботе (`/x402/complete_teleop_payment`).
- [ROBOT_TELEOP_KYR_RAID_GRANT.md](ROBOT_TELEOP_KYR_RAID_GRANT.md) — ответ RAID: порядок **help → Accept → GET session-grant**; на роботе поллинг в `raid_session_grant_client` + `EscalationManager`. События эскалации (старт help, POST RAID, poll гранта, пересылка в `teleop_fetch`, ошибки) дополнительно пишутся в **`~/.kyr/dashboard_events.jsonl`** для KYR веб-дашборда (см. `br-kyr/DOC/BLACKBOX_DASHBOARD.md`).
- [PEAQ_RAID_CLAIM.md](PEAQ_RAID_CLAIM.md) — peaq-клейм через RAID: KYR context, HTTP GET, merge в датасет; см. спеки в `br-vr-dev-sinc/DOC/`.

---

При изменении кода обновляйте соответствующие разделы и при появлении нового функционального блока — новый файл в `DOC/` + ссылка в этом индексе, в README и DEV_AI.
