import importlib
import inspect
import json
import logging
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlparse

import rospy

from .config import EndpointConfig, EndpointPricing, ServerConfig, load_config
from .x402 import (
    PaymentRequest,
    PaymentVerificationError,
    X402Client,
    X402Error,
)

logger = logging.getLogger(__name__)


class X402RestServer:
    def __init__(
        self,
        config_path: str,
        rpc_endpoint: str,
        server_shutdown_timeout: float = 1.0,
    ):
        self._config_path = config_path
        self._server_config: ServerConfig = load_config(config_path)
        self._rpc_endpoint = rpc_endpoint
        self._server_shutdown_timeout = server_shutdown_timeout
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._x402_client = X402Client(rpc_endpoint=rpc_endpoint)
        self._facilitator_url = self._server_config.facilitator_url

    def start(self) -> None:
        if self._httpd:
            raise RuntimeError("Server already running")

        handler_cls = self._create_handler_class()
        address = (self._server_config.listen_host, self._server_config.listen_port)
        self._httpd = ThreadingHTTPServer(address, handler_cls)
        rospy.loginfo("x402 REST server listening on %s:%s", *address)

        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._httpd:
            return

        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread:
            self._thread.join(timeout=self._server_shutdown_timeout)
        self._httpd = None
        self._thread = None
        rospy.loginfo("x402 REST server stopped")

    def reload(self) -> None:
        rospy.loginfo("Reloading x402 REST configuration from %s", self._config_path)
        self._server_config = load_config(self._config_path)
        self._facilitator_url = self._server_config.facilitator_url

    @property
    def x402_client(self) -> X402Client:
        return self._x402_client

    @property
    def facilitator_url(self) -> Optional[str]:
        return self._facilitator_url

    # Internal helpers -----------------------------------------------------

    def _create_handler_class(self):
        server = self
        endpoint_map = self._build_endpoint_map()

        class RequestHandler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            _max_log_payload = 2048

            def do_GET(self):  # noqa: N802
                self._handle("GET")

            def do_POST(self):  # noqa: N802
                self._handle("POST")

            def do_PUT(self):  # noqa: N802
                self._handle("PUT")

            def do_DELETE(self):  # noqa: N802
                self._handle("DELETE")

            def log_message(self, format_: str, *args: Any) -> None:
                rospy.loginfo("x402_rest: " + format_, *args)

            def _handle(self, method: str) -> None:
                endpoint = endpoint_map.get((self._normalized_path(self.path), method))
                if not endpoint:
                    self._send_json(
                        HTTPStatus.NOT_FOUND, {"error": "Endpoint not found"}
                    )
                    return

                try:
                    payload = self._parse_body()
                except ValueError as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": f"Invalid JSON payload: {exc}"},
                    )
                    return

                headers_log = self._sanitize_headers(self.headers)
                payload_log = self._truncate_for_log(payload)
                logger.info(
                    "Incoming request method=%s path=%s headers=%s body=%s",
                    method,
                    self.path,
                    headers_log,
                    payload_log,
                )

                try:
                    response = server._process_endpoint_request(
                        endpoint, payload, self.headers
                    )
                    self._send_json(HTTPStatus.OK, response)
                except PaymentVerificationError as exc:
                    payload = self._safe_error_payload(exc)
                    self._send_json(HTTPStatus.PAYMENT_REQUIRED, payload)
                except X402Error as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": str(exc)},
                    )
                except Exception as exc:  # pylint: disable=broad-except
                    rospy.logerr("Unhandled error processing request: %s", exc)
                    self._send_json(
                        HTTPStatus.INTERNAL_SERVER_ERROR,
                        {"error": "Internal server error"},
                    )

            def _parse_body(self) -> Dict[str, Any]:
                length = int(self.headers.get("Content-Length") or "0")
                if length == 0:
                    return {}
                raw_body = self.rfile.read(length).decode("utf-8")
                if not raw_body:
                    return {}
                return json.loads(raw_body)

            def _send_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
                payload_log = self._truncate_for_log(payload)
                logger.info(
                    "Outgoing response status=%s path=%s body=%s",
                    status.value,
                    self.path,
                    payload_log,
                )
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status.value)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _truncate_for_log(self, data: Any) -> str:
                """
                Reduce size of logged payloads to keep logs readable.
                """
                try:
                    text = json.dumps(data)
                except (TypeError, ValueError):
                    text = str(data)
                if len(text) <= self._max_log_payload:
                    return text
                truncated = text[: self._max_log_payload] + "...<truncated>"
                return truncated

            @staticmethod
            def _sanitize_headers(headers) -> Dict[str, str]:
                masked_headers: Dict[str, str] = {}
                for key, value in headers.items():
                    if key.lower() in {"authorization", "proxy-authorization"}:
                        masked_headers[key] = "***redacted***"
                    else:
                        masked_headers[key] = value
                return masked_headers

            @staticmethod
            def _safe_error_payload(exc: Exception) -> Dict[str, Any]:
                message = str(exc)
                try:
                    parsed = json.loads(message)
                    if isinstance(parsed, dict):
                        return parsed
                except json.JSONDecodeError:
                    pass
                return {"error": message}

            @staticmethod
            def _normalized_path(path: str) -> str:
                return urlparse(path).path

        return RequestHandler

    def _build_endpoint_map(self) -> Dict[Tuple[str, str], EndpointConfig]:
        return {
            (endpoint.path, endpoint.http_method): endpoint
            for endpoint in self._server_config.endpoints
        }

    def _process_endpoint_request(
        self,
        endpoint: EndpointConfig,
        body: Dict[str, Any],
        headers,
    ) -> Dict[str, Any]:
        self._ensure_payment(endpoint, body, headers)
        result = self._invoke_action(endpoint, body)
        return {"status": "ok", "data": result}

    def _ensure_payment(
        self,
        endpoint: EndpointConfig,
        body: Dict[str, Any],
        headers,
    ) -> None:
        pricing: Optional[EndpointPricing] = endpoint.x402_pricing
        if not pricing:
            return

        reference = headers.get("X-X402-Reference") or body.get("x402_reference")
        session = None
        if reference:
            session = self._x402_client.get_session(reference)

        if not session:
            receiver_account = pricing.receiver_account or (self._x402_client.public_key or "")
            if not receiver_account:
                raise X402Error("Receiver account is not configured for paid endpoint.")

            request = PaymentRequest(
                receiver_account=receiver_account,
                amount=pricing.amount,
                asset_symbol=pricing.asset_symbol,
            )
            session = self._x402_client.create_payment_session(
                request, pricing.payment_window_sec
            )
            raise PaymentVerificationError(
                json.dumps(
                    {
                        "message": "Payment session created. Complete payment to continue.",
                        "reference": session.request.reference,
                        "amount": pricing.amount,
                        "asset": pricing.asset_symbol,
                        "receiver": receiver_account,
                        "expires_in_sec": pricing.payment_window_sec,
                    }
                )
            )

        self._x402_client.verify_payment(session.request.reference)

    def _invoke_action(self, endpoint: EndpointConfig, body: Dict[str, Any]) -> Any:
        action = endpoint.ros_action
        if action.type != "python_call":
            raise X402Error(f"Unsupported action type: {action.type}")

        module = importlib.import_module(action.module)
        attr_path = action.callable.split(".")
        callable_obj = module
        for attr in attr_path:
            callable_obj = getattr(callable_obj, attr)
        args = list(action.args or [])
        kwargs = dict(action.kwargs or {})

        signature = inspect.signature(callable_obj)
        if "body" in signature.parameters:
            kwargs.setdefault("body", body)
        elif any(
            param.kind == inspect.Parameter.VAR_KEYWORD
            for param in signature.parameters.values()
        ):
            kwargs.setdefault("body", body)

        if callable(callable_obj):
            return callable_obj(*args, **kwargs)

        raise X402Error(
            f"Configured callable {action.callable} in module "
            f"{action.module} is not callable"
        )

