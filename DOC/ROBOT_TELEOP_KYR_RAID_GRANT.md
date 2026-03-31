# Интеграция телеопа робота с RAID: SessionGrant, KYR и оплата оператору

**Аудитория:** разработчик ПО на роботе (`teleop_fetch`, `rospy_x402`, KYR, `EscalationManager`).  
**Сторона RAID:** репозиторий `x402_raid_app`.  
**Связанные документы:** [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md), [TELEOP_FETCH.md](TELEOP_FETCH.md).

**Реализация на роботе:** после `POST …/teleop/help`, если в ответе нет inline `teleopGrantPayload`, `EscalationManager` вызывает **`raid_session_grant_client.poll_raid_session_grant`** (параметры `~raid_session_grant_poll`, `~raid_session_grant_timeout_sec`, `~raid_session_grant_interval_sec` на ноде `x402_server`).

---

## 1. Симптомы на стороне робота

В логах после закрытия телеоп-сессии:

- `complete_teleop_payment: success but NO on-chain transfer`
- `No valid operator Solana pubkey in receipt`
- упоминание **`pending_from_raid`**

**Значение:** в **SignedReceipt** KYR нет валидного Solana-адреса оператора. Он должен был прийти из **подписанного SessionGrant**, выданного RAID, а не из локального mock.

---

## 2. Что делает RAID и чего он не делает

| Действие | Где |
|----------|-----|
| Принимает запрос помощи | `POST /api/robots/{robotId}/teleop/help` + секрет робота |
| После **accept** оператором подписывает SessionGrant с **`operator_pubkey`** (кошелёк оператора из БД RAID) | Запись в БД + выдача через `GET …/teleop/session-grant` |
| Перевод SOL оператору | **Не выполняет.** Платёж делает **робот** через `/x402/complete_teleop_payment` и свой настроенный плательщик |

Итог: если KYR открыт с **mock-грантом** (`operator_pubkey: pending_from_raid`), receipt будет без реального адреса — это **ошибка порядка шагов или доверия подписи на роботе**, а не «устаревшие данные» в первом `POST …/help`.

---

## 3. Обязательный порядок шагов (робот)

```text
1) POST …/teleop/help
      → сохранить helpRequest.id (и при необходимости teleopGrantPollUrl из ответа)

2) Дождаться, пока оператор нажмёт Accept в RAID (человек/UI).

3) GET …/teleop/session-grant?helpRequestId=<uuid из шага 1>
      → тот же заголовок секрета робота, что и для help
      → ответ 200: teleopGrantPayload (строка JSON), teleopGrantSignature (base58), grantSignerPublicKey

4) Распарсить teleopGrantPayload в объект (UTF-8, байт-в-байт как строка — вариант A спеки).
      → проверить: operator_pubkey — валидный base58 Solana, НЕ "pending_from_raid"

5) Передать payload + signature в KYR open_session (как в вашем стеке: receive_grant / аналог).
      → доверить на KYR публичный ключ подписанта: grantSignerPublicKey (см. §5)

6) Телеоп-сессия → close_session → complete_teleop_payment с receipt от KYR
```

**Запрещённый для оплаты сценарий:** вызвать KYR `open_session` с **внутренним mock-грантом**, а **потом** (или никогда) не подменить его данными с RAID.

---

## 4. HTTP-контракт (робот → RAID)

### 4.1 Секрет

Заголовок: **`X-Robot-Teleop-Secret: <teleopSecret>`**  
(или `Authorization: Bearer <teleopSecret>` — как уже договорено с RAID.)

Значение **`teleopSecret`** выдаётся при регистрации робота в RAID (enroll / админка).

### 4.2 Запрос помощи

```http
POST /api/robots/{robotId}/teleop/help
X-Robot-Teleop-Secret: <secret>
Content-Type: application/json

{ "message": "…", "metadata": { "task_id": "…", … } }
```

В ответе (при включённом подписании гранта на RAID):

- **`id`** / **`helpRequest.id`** — UUID заявки; **обязателен** для шага 3.
- **`teleopGrantPollUrl`** — относительный путь, например  
  `/api/robots/{robotId}/teleop/session-grant?helpRequestId={id}`  
  (к базе RAID добавить схему и хост, например `https://raid.example`).

### 4.3 Получение гранта (только после accept оператором)

```http
GET /api/robots/{robotId}/teleop/session-grant?helpRequestId=<uuid>
X-Robot-Teleop-Secret: <secret>
```

**Успех 200** (пример полей):

| Поле | Тип | Описание |
|------|-----|----------|
| `teleopGrantPayload` | string | Ровно UTF-8 JSON SessionGrant; **не пересобирать** JSON для проверки подписи на KYR |
| `teleopGrantSignature` | string | Ed25519 подпись в **base58** над **сырыми UTF-8 байтами** `teleopGrantPayload` |
| `grantSignerPublicKey` | string | Solana base58 публичный ключ **подписанта гранта** (RAID). Должен быть в `trusted_raid_keys` на KYR |

**Ошибки 404 (тело JSON, поле `error`):**

| error | Когда |
|-------|--------|
| `grant_not_ready` | Заявка ещё в статусе open (оператор не принял) |
| `grant_unconfigured` | На RAID не задан `TELEOP_GRANT_SIGNING_SECRET_KEY` |
| `grant_absent` | У оператора нет `wallet_public_key` в БД RAID |

Стратегия робота: после help **поллить** `session-grant` с backoff, пока не 200 или не таймаут по политике продукта.

---

## 5. Доверие подписи на KYR (`trusted_raid_keys`)

Подписывает грант **отдельный** ключ RAID (Ed25519 / Solana keypair), **не** кошелёк оператора.

На стороне KYR должен быть разрешён публичный ключ, совпадающий с:

- **`grantSignerPublicKey`** из ответа `session-grant`, или
- **`teleopGrantSignerPublicKey`** из **`GET /health`** на том же инстансе RAID.

Если ключ не добавлен или неверен, KYR может отклонить грант и остаться на mock → в receipt снова **`pending_from_raid`**.

---

## 6. Содержимое SessionGrant (после `JSON.parse(teleopGrantPayload)`)

Ожидаемые поля (имена **как в JSON RAID**):

| Поле | Описание |
|------|----------|
| `session_id` | UUID прокси-сессии телеопа в RAID (**`session.id`** из ответа оператору на **accept**). Должен согласовываться с тем, как KYR/teleop_fetch вяжут сессию |
| `robot_id` | UUID робота в RAID |
| `task_id` | Строка из `metadata.task_id` запроса help (может быть пустой) |
| `operator_pubkey` | **Solana base58** получателя SOL |
| `valid_until_sec` | Unix-время истечения гранта |
| `scope_json` | JSON-строка (политика; RAID может добавлять подсказки по сумме оплаты) |

Если **`operator_pubkey`** отсутствует или равен запасному литералу mock — **не** вызывать `complete_teleop_payment` с ожиданием on-chain перевода оператору.

---

## 7. Самопроверка с рабочей станции

Подставьте `RAID_BASE`, `robotId`, `secret`, `helpRequestId` (после accept):

```bash
curl -sS -H "X-Robot-Teleop-Secret: ${SECRET}" \
  "${RAID_BASE}/api/robots/${ROBOT_ID}/teleop/session-grant?helpRequestId=${HELP_ID}"
```

Далее:

```bash
# Пример: показать operator_pubkey из payload (jq)
curl -sS -H "X-Robot-Teleop-Secret: ${SECRET}" \
  "${RAID_BASE}/api/robots/${ROBOT_ID}/teleop/session-grant?helpRequestId=${HELP_ID}" \
  | jq -r '.teleopGrantPayload | fromjson | .operator_pubkey'
```

Ожидается непустая base58-строка, не `pending_from_raid`.

---

## 8. Чеклист перед эскалацией к команде RAID

- [ ] После **accept** `GET session-grant` возвращает **200** с непустыми `teleopGrantPayload` и `teleopGrantSignature`.
- [ ] В распарсенном payload поле **`operator_pubkey`** валидно для Solana.
- [ ] На KYR в **`trusted_raid_keys`** добавлен **`grantSignerPublicKey`** с RAID.
- [ ] **KYR `open_session`** вызывается **после** получения гранта с RAID, с **этими** payload/signature, а не только с mock.
- [ ] Для оплаты на роботе настроены RPC, баланс плательщика и сервис **`/x402/complete_teleop_payment`** (это вне RAID).

Если пункты выполнены, а on-chain перевод не идёт — смотреть логи Solana/x402 на **роботе** (ошибка RPC, сумма, подпись транзакции).

---

## 9. Ссылки

- Полный цикл и диаграмма: [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md)  
- HTTP help с робота: [TELEOP_FETCH.md](TELEOP_FETCH.md)  
- OpenAPI инстанса RAID: `https://<raid-host>/docs`
