import json
import logging
import time
import urllib.error
import urllib.request
from typing import Any, Dict, Optional


logger = logging.getLogger(__name__)


class SolanaRpcClient:
    def __init__(
        self,
        endpoint: str,
        timeout: int = 10,
        max_retries: int = 3,
        backoff_factor: float = 0.2,
    ):
        self._endpoint = endpoint
        self._timeout = timeout
        self._request_id = 0
        self._max_retries = max_retries
        self._backoff_factor = backoff_factor

    @property
    def endpoint(self) -> str:
        return self._endpoint

    def _next_id(self) -> int:
        self._request_id += 1
        return self._request_id

    def call(self, method: str, params: Optional[list] = None) -> Dict[str, Any]:
        payload = json.dumps(
            {"jsonrpc": "2.0", "id": self._next_id(), "method": method, "params": params or []}
        ).encode("utf-8")
        req = urllib.request.Request(
            self._endpoint,
            data=payload,
            headers={"Content-Type": "application/json"},
        )

        attempt = 0
        while True:
            try:
                with urllib.request.urlopen(req, timeout=self._timeout) as response:
                    data = response.read().decode("utf-8")
                    logger.debug(
                        "Solana RPC call method=%s params=%s status=%s len=%s",
                        method,
                        params,
                        response.status,
                        len(data),
                    )
                    return json.loads(data)
            except urllib.error.HTTPError as exc:
                attempt += 1
                if exc.code in {429, 503} and attempt <= self._max_retries:
                    delay = self._backoff_factor * (2 ** (attempt - 1))
                    logger.warning(
                        "Solana RPC rate limited (status=%s). Retrying in %.2fs (attempt %d/%d)",
                        exc.code,
                        delay,
                        attempt,
                        self._max_retries,
                    )
                    time.sleep(delay)
                    continue
                logger.error(
                    "Solana RPC HTTP error method=%s status=%s: %s",
                    method,
                    exc.code,
                    exc,
                )
                raise
            except urllib.error.URLError as exc:
                attempt += 1
                if attempt <= self._max_retries:
                    delay = self._backoff_factor * (2 ** (attempt - 1))
                    logger.warning(
                        "Solana RPC connection error (%s). Retrying in %.2fs (attempt %d/%d)",
                        exc,
                        delay,
                        attempt,
                        self._max_retries,
                    )
                    time.sleep(delay)
                    continue
                logger.error("Failed to reach Solana RPC endpoint: %s", exc)
                raise

    def get_signatures_for_address(
        self,
        address: str,
        limit: int = 20,
        commitment: str = "confirmed",
    ) -> Dict[str, Any]:
        return self.call(
            "getSignaturesForAddress",
            [address, {"limit": limit, "commitment": commitment}],
        )

    def get_transaction(
        self,
        signature: str,
        commitment: str = "confirmed",
    ) -> Dict[str, Any]:
        return self.call(
            "getTransaction",
            [signature, {"encoding": "jsonParsed", "commitment": commitment}],
        )

    def wait_for_signature(self, signature: str, max_attempts: int = 10, delay_seconds: float = 2.0) -> Dict[str, Any]:
        for attempt in range(max_attempts):
            result = self.get_transaction(signature)
            if result.get("result"):
                return result
            logger.debug("Waiting for signature %s (attempt %d)", signature, attempt + 1)
            time.sleep(delay_seconds)
        return {}

