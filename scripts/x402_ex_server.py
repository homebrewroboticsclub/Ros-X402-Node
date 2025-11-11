#!/usr/bin/env python3
import json
import logging
import os
import signal
import sys
import time
import urllib.error
import urllib.request
from typing import Dict, Optional

import rospkg
import rospy
from rospy_x402.server import X402RestServer
from rospy_x402.x402 import (
    KeyManager,
    PaymentRequest,
    PaymentSubmissionError,
    PaymentTimeoutError,
    PaymentVerificationError,
    X402Client,
    X402Error,
)
from rospy_x402.srv import (
    x402_buy_service,
    x402_buy_serviceRequest,
    x402_buy_serviceResponse,
)


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("x402_ex_server")


def _load_headers(headers_json: str) -> Dict[str, str]:
    if not headers_json:
        return {}
    try:
        data = json.loads(headers_json)
        if not isinstance(data, dict):
            raise ValueError("Headers must be a JSON object")
        return {str(key): str(value) for key, value in data.items()}
    except json.JSONDecodeError as exc:
        raise ValueError(f"Failed to parse headers JSON: {exc}") from exc


def _http_request(
    url: str, method: str, payload: str, headers: Dict[str, str]
) -> str:
    data = payload.encode("utf-8") if payload else None
    request = urllib.request.Request(url, data=data, method=method.upper())
    for key, value in headers.items():
        request.add_header(key, value)

    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            body = response.read().decode("utf-8")
            return body
    except urllib.error.HTTPError as exc:
        error_body = exc.read().decode("utf-8")
        raise RuntimeError(
            f"External service returned {exc.code}: {error_body}"
        ) from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Failed to reach external service: {exc}") from exc


class X402BuyServiceHandler:
    def __init__(
        self,
        x402_client: X402Client,
        receiver_account: str,
        asset_symbol: str,
        payment_window_sec: int,
        poll_interval_sec: float,
    ):
        self._x402_client = x402_client
        self._receiver_account = receiver_account
        self._asset_symbol = asset_symbol
        self._payment_window_sec = payment_window_sec
        self._poll_interval_sec = poll_interval_sec

    def __call__(self, request: x402_buy_serviceRequest) -> x402_buy_serviceResponse:
        try:
            headers = _load_headers(request.headers_json)
        except ValueError as exc:
            logger.error("Invalid headers JSON: %s", exc)
            return x402_buy_serviceResponse(
                success=False,
                response_body="",
                error_message=str(exc),
                payment_signature="",
            )

        signature = ""
        payee_account = (request.payer_account or "").strip()
        asset_symbol = request.asset_symbol or self._asset_symbol or "SOL"

        if payee_account:
            try:
                signature = self._x402_client.send_payment(payee_account, request.amount)
                rospy.loginfo(
                    "Payment to %s completed signature=%s amount=%s %s",
                    payee_account,
                    signature,
                    request.amount,
                    asset_symbol,
                )
            except PaymentSubmissionError as exc:
                logger.error("Outgoing payment failed: %s", exc)
                return x402_buy_serviceResponse(
                    success=False,
                    response_body="",
                    error_message=str(exc),
                    payment_signature="",
                )
        else:
            receiver_account = self._receiver_account or (self._x402_client.public_key or "")
            if not receiver_account:
                return x402_buy_serviceResponse(
                    success=False,
                    response_body="",
                    error_message="Receiver account is not configured.",
                    payment_signature="",
                )

            payment_request = PaymentRequest(
                receiver_account=receiver_account,
                amount=request.amount,
                asset_symbol=asset_symbol,
            )

            session = self._x402_client.create_payment_session(
                payment_request, self._payment_window_sec
            )
            rospy.loginfo(
                "Created incoming payment session reference=%s amount=%s %s receiver=%s",
                session.request.reference,
                session.request.amount,
                session.request.asset_symbol,
                payment_request.receiver_account,
            )

            signature = self._wait_for_payment(session.request.reference)
            if not signature:
                info_payload = json.dumps(
                    {
                        "reference": session.request.reference,
                        "receiver": payment_request.receiver_account,
                        "amount": session.request.amount,
                        "asset": session.request.asset_symbol,
                        "error": "Payment not confirmed in time.",
                    }
                )
                return x402_buy_serviceResponse(
                    success=False,
                    response_body=info_payload,
                    error_message="Payment not confirmed in time.",
                    payment_signature="",
                )

        try:
            response_body = _http_request(
                url=request.endpoint,
                method=request.method or "GET",
                payload=request.payload,
                headers=self._inject_signature_header(headers, signature),
            )
        except RuntimeError as exc:
            return x402_buy_serviceResponse(
                success=False,
                response_body="",
                error_message=str(exc),
                payment_signature=signature,
            )

        return x402_buy_serviceResponse(
            success=True,
            response_body=response_body,
            error_message="",
            payment_signature=signature,
        )

    def _wait_for_payment(self, reference: str) -> Optional[str]:
        deadline = time.time() + self._payment_window_sec
        while time.time() < deadline and not rospy.is_shutdown():
            try:
                session = self._x402_client.verify_payment(reference)
                if session.signature:
                    return session.signature
            except PaymentVerificationError:
                pass
            except PaymentTimeoutError:
                break

            time.sleep(self._poll_interval_sec)

        return None

    @staticmethod
    def _inject_signature_header(headers: Dict[str, str], signature: str) -> Dict[str, str]:
        if signature:
            headers = dict(headers)
            headers.setdefault("X-X402-Payment-Signature", signature)
        return headers


def main() -> None:
    rospy.init_node("x402_ex_server")

    config_path = rospy.get_param("~config_path", "")
    if not config_path:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("rospy_x402")
        config_path = os.path.join(package_path, "config", "endpoints.example.json")

    rpc_endpoint = rospy.get_param(
        "~solana_rpc_endpoint", "https://api.mainnet-beta.solana.com"
    )
    key_manager = KeyManager()
    try:
        key_material = key_manager.load_from_prompt()
    except ValueError as exc:
        rospy.logerr("Failed to load private key: %s", exc)
        sys.exit(1)

    receiver_account = rospy.get_param(
        "~service_receiver_account", key_material.public_key_b58
    )
    asset_symbol = rospy.get_param("~service_asset_symbol", "SOL")
    payment_window_sec = int(
        rospy.get_param("~service_payment_window_sec", 300)
    )
    poll_interval_sec = float(
        rospy.get_param("~service_poll_interval_sec", 5.0)
    )

    rest_server = X402RestServer(config_path, rpc_endpoint)
    rest_server.x402_client.set_key_material(key_material)

    os.environ["ROSPY_X402_CONFIG"] = config_path
    if rest_server.facilitator_url:
        rospy.loginfo("Configured external facilitator at %s", rest_server.facilitator_url)

    try:
        rest_server.start()
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logerr("Failed to start REST server: %s", exc)
        sys.exit(1)

    buy_handler = X402BuyServiceHandler(
        rest_server.x402_client,
        receiver_account=receiver_account,
        asset_symbol=asset_symbol,
        payment_window_sec=payment_window_sec,
        poll_interval_sec=poll_interval_sec,
    )

    rospy.Service("x402_buy_service", x402_buy_service, buy_handler)
    rospy.loginfo("Loaded x402 key. Public key: %s", key_material.public_key_b58)
    rospy.loginfo("x402_ex_server node started")

    def shutdown_handler(signum, frame):  # noqa: D401
        rospy.loginfo("Received shutdown signal (%s)", signum)
        rest_server.stop()
        rospy.signal_shutdown("Signal received")

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except X402Error as exc:
        rospy.logerr("x402 error: %s", exc)
        sys.exit(2)

