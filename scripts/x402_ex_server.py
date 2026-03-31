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

# Paths of .env files successfully merged (last file wins per key). Set at import time.
_DOTENV_LOADED_PATHS: list = []
_DOTENV_IMPORT_FAILED: bool = False


def _load_dotenv() -> None:
    """
    Merge several optional .env files (override=True in order — later wins).
    Do not stop at the first file: an empty ~/.env must not block package .env.
    """
    global _DOTENV_LOADED_PATHS, _DOTENV_IMPORT_FAILED
    try:
        from dotenv import load_dotenv
    except ImportError:
        _DOTENV_IMPORT_FAILED = True
        return

    env_file = os.environ.get("ROSPY_X402_ENV_FILE", "").strip()
    ordered: list = []
    try:
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("rospy_x402")
        ordered.append(os.path.join(pkg_path, "config", ".env"))
        ordered.append(os.path.join(pkg_path, ".env"))
    except (rospkg.ResourceNotFound, rospkg.ROSPkgException):
        pass
    ordered.append(os.path.join(os.getcwd(), ".env"))
    ordered.append(os.path.join(os.path.expanduser("~"), ".rospy_x402.env"))
    if env_file:
        ordered.append(env_file)

    seen = set()
    for path in ordered:
        if not path or path in seen or not os.path.isfile(path):
            continue
        seen.add(path)
        load_dotenv(path, override=True)
        _DOTENV_LOADED_PATHS.append(path)


_load_dotenv()
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
    CompleteTeleopPayment,
    CompleteTeleopPaymentRequest,
    CompleteTeleopPaymentResponse,
    x402_buy_service,
    x402_buy_serviceRequest,
    x402_buy_serviceResponse,
)
from rospy_x402.teleop_operator_payment import pay_operator_from_receipt_payload


from rospy_x402.escalation_service import EscalationManager
from rospy_x402.raid_integration import (
    build_operator_registry_url,
    credentials_from_state,
    default_allowlist_path,
    default_state_path,
    enroll_robot,
    load_raid_state,
    normalized_sync_path,
    parse_enroll_credentials,
    save_raid_state,
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
            endpoint = (request.endpoint or "").strip()
            if not endpoint:
                return x402_buy_serviceResponse(
                    success=True,
                    response_body="",
                    error_message="",
                    payment_signature=signature,
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
                url=(request.endpoint or "").strip(),
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

    config_path = rospy.get_param(
        "~config_path",
        os.environ.get("ROSPY_X402_CONFIG", "").strip(),
    )
    if not config_path:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("rospy_x402")
        config_path = os.path.join(package_path, "config", "endpoints.example.json")

    # RPC: rosparam overrides; else Helius if HELIUS_API_KEY set; else default public
    default_rpc = "https://api.mainnet-beta.solana.com"
    helius_key = (os.environ.get("HELIUS_API_KEY") or "").strip()
    if helius_key:
        default_rpc = f"https://mainnet.helius-rpc.com/?api-key={helius_key}"
    rpc_endpoint = (rospy.get_param("~solana_rpc_endpoint", "") or "").strip() or default_rpc
    rospy.loginfo("Solana RPC: %s", rpc_endpoint.split("?")[0] + ("... (Helius)" if "helius" in rpc_endpoint.lower() else ""))

    key_manager = KeyManager()
    key_material = key_manager.load_from_env()
    if not key_material:
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

    if _DOTENV_IMPORT_FAILED:
        rospy.logwarn(
            "python3-dotenv is not installed; .env files are ignored. "
            "Install: sudo apt install python3-dotenv"
        )
    elif _DOTENV_LOADED_PATHS:
        rospy.loginfo("rospy_x402 dotenv merged from: %s", " | ".join(_DOTENV_LOADED_PATHS))
    else:
        rospy.logwarn(
            "No .env file found for rospy_x402 (package config/.env and .env, cwd .env, "
            "~/.rospy_x402.env, ROSPY_X402_ENV_FILE). "
            "Either copy .env.example to the package .env on this machine, or "
            "export ROBOT_FLEET_ENROLLMENT_SECRET / RAID_* before roslaunch."
        )

    # rosparam > env RAID_APP_URL > static IP (mDNS often breaks in Docker / some LANs)
    raid_app_url = (rospy.get_param("~raid_app_url", "") or "").strip()
    if not raid_app_url:
        raid_app_url = (os.environ.get("RAID_APP_URL") or "").strip()
    if not raid_app_url:
        raid_app_url = "http://192.168.20.53:3000"
    state_path = (
        (os.environ.get("RAID_STATE_FILE") or "").strip()
        or (rospy.get_param("~raid_state_file", "") or "").strip()
        or default_state_path()
    )
    allowlist_path = (
        (os.environ.get("RAID_ALLOWLIST_FILE") or "").strip()
        or (rospy.get_param("~raid_allowlist_file", "") or "").strip()
        or default_allowlist_path(state_path)
    )
    fleet_secret = (
        (os.environ.get("ROBOT_FLEET_ENROLLMENT_SECRET") or "").strip()
        or (rospy.get_param("~robot_fleet_enrollment_secret", "") or "").strip()
    )
    enroll_key = (
        (os.environ.get("RAID_ENROLLMENT_KEY") or "").strip()
        or (rospy.get_param("~raid_enrollment_key", "") or "").strip()
    )
    raid_to_robot_secret = (
        (os.environ.get("RAID_TO_ROBOT_SECRET") or "").strip()
        or (rospy.get_param("~raid_to_robot_secret", "") or "").strip()
    )
    sync_path = normalized_sync_path(
        rospy.get_param("~raid_operator_sync_path", "/raid/operator-allowlist")
    )

    listen_port = int(rest_server._server_config.listen_port)
    enroll_http_port = int(rospy.get_param("~raid_enroll_http_port", listen_port))
    enroll_host = (
        (rospy.get_param("~raid_enroll_host", "") or "").strip()
        or (os.environ.get("RAID_ENROLL_HOST") or "").strip()
    )

    registry_url = None
    if raid_to_robot_secret and enroll_host:
        registry_url = build_operator_registry_url(
            enroll_host, enroll_http_port, sync_path
        )

    raid_robot_id = (
        (rospy.get_param("~raid_robot_id", "") or "").strip()
        or (os.environ.get("RAID_ROBOT_ID") or "").strip()
    )
    raid_teleop_secret = (
        (rospy.get_param("~raid_teleop_secret", "") or "").strip()
        or (os.environ.get("RAID_TELEOP_SECRET") or "").strip()
    )

    if not raid_robot_id or not raid_teleop_secret:
        saved = load_raid_state(state_path)
        if saved:
            creds = credentials_from_state(saved)
            if creds:
                raid_robot_id, raid_teleop_secret = creds
                rospy.loginfo("Loaded RAID credentials from %s", state_path)

    if (not raid_robot_id or not raid_teleop_secret) and fleet_secret and enroll_key:
        if not enroll_host:
            rospy.logerr(
                "RAID auto-enroll skipped: set ~raid_enroll_host or RAID_ENROLL_HOST "
                "(LAN-reachable address for RAID HTTP health)."
            )
        else:
            rb_host = (rospy.get_param("~raid_enroll_rosbridge_host", "") or "").strip()
            if not rb_host:
                rb_host = enroll_host
            rb_port = int(rospy.get_param("~raid_enroll_rosbridge_port", 9090))
            robot_name = (rospy.get_param("~raid_robot_name", "") or "").strip() or None
            try:
                data = enroll_robot(
                    raid_app_url,
                    fleet_secret,
                    enroll_key,
                    enroll_host,
                    enroll_http_port,
                    rosbridge_host=rb_host,
                    rosbridge_port=rb_port,
                    name=robot_name,
                    operator_registry_url=registry_url,
                )
                raid_robot_id, raid_teleop_secret = parse_enroll_credentials(data)
                save_raid_state(state_path, raid_robot_id, raid_teleop_secret)
                rospy.loginfo("RAID enroll ok; credentials saved to %s", state_path)
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logerr("RAID enroll failed: %s", exc)

    if not raid_robot_id or not raid_teleop_secret:
        rospy.logwarn(
            "RAID credentials still missing: teleop/help will fail until you set "
            "~raid_robot_id + ~raid_teleop_secret, or enroll (fleet secret + "
            "RAID_ENROLLMENT_KEY + RAID_ENROLL_HOST in env and reachable %s), "
            "or place id/teleopSecret in %s.",
            raid_app_url,
            state_path,
        )

    if raid_to_robot_secret:
        rest_server.raid_operator_sync_path = sync_path
        rest_server.raid_to_robot_secret = raid_to_robot_secret
        rest_server.raid_allowlist_file = allowlist_path
        rospy.loginfo(
            "RAID operator sync enabled at POST %s (allowlist file %s)",
            sync_path,
            allowlist_path,
        )
    else:
        rest_server.raid_operator_sync_path = None
        rest_server.raid_to_robot_secret = None
        rest_server.raid_allowlist_file = None

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

    def _complete_teleop_payment_cb(
        req: CompleteTeleopPaymentRequest,
    ) -> CompleteTeleopPaymentResponse:
        rate = float(rospy.get_param("~teleop_operator_payment_sol_per_sec", 1e-6))
        ok, msg, sig = pay_operator_from_receipt_payload(
            rest_server.x402_client, req.receipt_payload, rate
        )
        return CompleteTeleopPaymentResponse(ok, msg, sig)

    rospy.Service(
        "/x402/complete_teleop_payment",
        CompleteTeleopPayment,
        _complete_teleop_payment_cb,
    )

    try:
        escalation_manager = EscalationManager(
            raid_app_url=raid_app_url,
            robot_id=raid_robot_id,
            teleop_secret=raid_teleop_secret,
            x402_client=rest_server.x402_client,
        )
        rospy.loginfo("EscalationManager initialized.")
    except Exception as exc:  # pylint: disable=broad-except
        rospy.logwarn("EscalationManager failed to initialize: %s", exc)

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

