# RAID App — расширение `POST …/teleop/help` (поле `situation_report`)

**Аудитория:** разработчики RAID App (`x402_raid_app` или эквивалент).  
**Источник на роботе:** пакет `rospy_x402`, `EscalationManager._request_grant_from_raid` → HTTP `POST` на URL ниже.

## Эндпоинт и заголовки (без изменений)

- **Метод:** `POST`
- **Путь:** `/api/robots/{robotId}/teleop/help` (`robotId` — UUID из enroll).
- **Заголовки:**
  - `Content-Type: application/json`
  - `X-Robot-Teleop-Secret` — секрет робота из enroll

## Тело запроса JSON

Робот отправляет объект вида:

```json
{
  "message": "Need assistance",
  "metadata": {
    "task_id": "string",
    "error_context": "string",
    "situation_report": "string"
  }
}
```

| Поле | Обязательность | Описание |
|------|----------------|----------|
| `message` | да | Краткая метка (как раньше). |
| `metadata` | да | Объект с контекстом заявки. |
| `metadata.task_id` | да | Идентификатор задачи / сессии на стороне робота. |
| `metadata.error_context` | да | Строка (часто JSON) с машиночитаемыми деталями ошибки; может быть пустой. |
| `metadata.situation_report` | **новое**, рекомендуется | Свободный текст UTF-8: **текущее состояние робота**, **что делал недавно**, **почему нужен телеоператор**. Может быть длинным (тысячи символов). Старые клиенты могут не слать ключ — трактовать как `""`. |
| `metadata.kyr_peaq_context` | **новое**, опционально | Объект JSON от KYR для привязки peaq-клейма на RAID (см. `br-vr-dev-sinc/DOC/RAID_APP_PEAQ_CLAIM_SPEC.md`). Если нет — не падать. |

## Что сделать на стороне RAID

1. **Принять** `metadata.situation_report` в теле `POST …/teleop/help` (парсинг JSON).
2. **Сохранить** в модели заявки о помощи (help request) и отдавать в UI/API оператору вместе с `task_id` / `error_context`.
3. **Обратная совместимость:** если поля нет — не падать; считать пустой строкой.
4. **Ограничения (рекомендация):** лимит длины на уровне API/БД (например 32–64 KiB), при превышении — `413` или обрезка с пометкой в логе — по политике продукта.
5. **Кодировка:** UTF-8; не интерпретировать как HTML без экранирования в UI.

## Связанный ROS API на роботе

Сервис `rospy_x402/RequestHelp` (`/x402/request_help`): поле `situation_report` дублируется в `metadata.situation_report` HTTP-запроса.

Документация робота: [RAID_INTEGRATION.md](RAID_INTEGRATION.md).
