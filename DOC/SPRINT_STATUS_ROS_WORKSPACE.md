# Спринт: семафоры и тесты — зона ответственности **rospy_x402**

Состояние на **2026-03-31**. Пакет закрывает **ROS-интеграцию x402, RAID HTTP, эскалацию телеопа, пост-оплату оператору** и дополнительное логирование в `~/.kyr/dashboard_events.jsonl`. Облачные таблицы receipts/incidents, Hugging Face, quality score, внешняя БД инцидентов — **вне репозитория**.

## Краткий вывод

| Категория | Комментарий |
|-----------|-------------|
| Реализовано | `EscalationManager`: `POST …/teleop/help`, poll `session-grant`, передача grant в `teleop_fetch`, optional Peaq claim → `set_peaq_dataset_claim`, события в dashboard log; RAID клиенты, тесты эскалации |
| Частично | П.7 (`raid_task_id` / `payment_id` в SessionRecord): на роботе есть `task_id` в grant/event и цепочка x402; **именованные поля спринта в облачном SessionRecord** — зона Backend/DATA_NODE |
| Не реализовано в репо | REST `GET /api/v1/receipts`, связь receipt↔incident, авто `TELEOP_TAKEOVER`, инциденты, RTT gate, Cosmos, recovery extractor, HF, 500+ external incidents |

---

## Семафоры → rospy_x402

| # | Deliverable | Роль пакета | Статус |
|---|-------------|-------------|--------|
| 1 | Receipt emission service | x402 REST — **платежи/робот-демо**, не облачный receipt CRUD | **Н/Д** как HTTP receipts API; пересечение с KYR receipt только через `close_session` / оплату |
| 2 | Receipt–incident | Нет | **Н/Д** |
| 3 | Event↔recording | Косвенно: эскалация несёт `task_id`, датасет — в teleop_fetch | **Частично** только как данные для downstream |
| 4 | `TELEOP_TAKEOVER` авто | Нет публикации такого события в облако из rospy_x402 | **Не реализовано** |
| 5 | Incident auto-generation | Нет | **Н/Д** |
| 6 | RTT preflight | Нет | **Н/Д** |
| 7 | `raid_task_id` + `payment_id` | Grant/RAID контекст; merge Peaq в metadata — см. `escalation_service`, `PEAQ_RAID_CLAIM.md` | **🟡 PARTIAL**: робот/RAID; облачный SessionRecord — вне репо |
| 8 | KYR stats | Нет эндпоинта облака | **Н/Д** |
| 9–11 | Handoff / VR | Нет | **Н/Д** |
| 17 | peaq ClaimRegistry real calls | HTTP GET peaq claim, proxy в датасет | **🟡**: реализована **робот-часть**; `claim_id` в облачном SessionRecord — Backend |

Остальные пункты (12–16, 18–21, 22 тесты 12–24) — **не rospy_x402**.

---

## Тесты спринта → rospy_x402

| # | Тест | Статус |
|---|------|--------|
| 1–6, 8–16, 18–24 | Требуют облако / VR / датасеты / HF | **Н/Д** или ручная интеграция вне пакета |
| 7 | `raid_task_id` / `payment_id` в session | Проверка через реальный upload в DATA_NODE и содержимое `metadata.json` / API — **не покрыта unit-тестами rospy_x402** |
| 17 | peaq claim в SessionRecord | Аналогично: робот кладёт claim в metadata path (teleop_fetch); облачная запись — вне репо |

---

## Автотесты пакета rospy_x402

Прогон **2026-03-31**:

```bash
cd /home/ubuntu/ros_ws && source devel/setup.bash
catkin build rospy_x402 --no-status --catkin-make-args run_tests
```

Результат: **34 tests OK** (`test_escalation_service`, `test_raid_*`, `test_dashboard_events_log`, и др.).

---

## Логи окружения

В `~/.kyr/dashboard_events.jsonl` фиксировались виды: `help_request_start`, `help_raid_post_ok`, `grant_poll_ok` / `grant_inline_ok`, `grant_mock_fallback`, `session_close` (от KYR) — **без** `TELEOP_TAKEOVER`.

---

## Замечания для PM / Backend

- Спринтовый **Receipt emission** (таблица + HMAC + GET) — отдельный сервис; rospy_x402 даёт **x402 payment flow** и **связку с RAID**, а не CRUD receipts.
- Для п.7 и теста 7 нужна явная сверка полей **`task_id` из RAID** vs **`raid_task_id` / `payment_id`** в контракте DATA_NODE (см. документацию teleop_fetch и Backend).
