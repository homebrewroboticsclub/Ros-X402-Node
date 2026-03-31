import json
import logging
import time
import requests
import rospy
from typing import Dict, Any, Optional, Tuple

from rospy_x402.srv import RequestHelp, RequestHelpResponse

from rospy_x402.raid_peaq_client import extract_peaq_claim_object, fetch_peaq_claim
from rospy_x402.raid_teleop_grant import extract_signed_grant_from_raid_help_response
from rospy_x402.teleop_operator_payment import pay_operator_from_receipt_payload

# Try to import teleop_fetch services if available, fallback otherwise
try:
    from teleop_fetch.srv import ReceiveGrant, EndSession
except ImportError:
    ReceiveGrant = None  # type: ignore
    EndSession = None  # type: ignore

try:
    from teleop_fetch.srv import SetPeaqDatasetClaim
except ImportError:
    SetPeaqDatasetClaim = None  # type: ignore

try:
    from KYR.srv import CloseSession, GetPeaqIssuanceMetadata
except ImportError:
    CloseSession = None  # type: ignore
    GetPeaqIssuanceMetadata = None  # type: ignore

logger = logging.getLogger(__name__)

class EscalationManager:
    """
    Handles escalation to RAID_APP when the robot needs help.
    """
    def __init__(self, raid_app_url: str, robot_id: str, teleop_secret: str, x402_client=None):
        self.raid_app_url = raid_app_url.rstrip('/')
        self.robot_id = robot_id
        self.teleop_secret = teleop_secret
        self.x402_client = x402_client
        
        # ROS Service for manual/external triggering
        try:
            self.srv = rospy.Service('/x402/request_help', RequestHelp, self.handle_request_help)
        except NameError:
            logger.warning("RequestHelp service type not available. Run catkin_make and source.")
        
        # ROS Proxy to teleop_fetch to forward the grant
        self.teleop_grant_proxy = None
        if ReceiveGrant is not None:
            try:
                self.teleop_grant_proxy = rospy.ServiceProxy("/teleop_fetch/receive_grant", ReceiveGrant)
            except Exception as e:
                logger.warning("ReceiveGrant proxy not created: %s", e)

        # Subs for session completion to trigger post-pay
        self.kyr_close_proxy = None
        if CloseSession is not None:
            try:
                self.kyr_close_proxy = rospy.ServiceProxy("/kyr/close_session", CloseSession)
            except Exception as e:
                logger.warning("CloseSession proxy not created: %s", e)

        self._set_peaq_claim_proxy = None
        if SetPeaqDatasetClaim is not None:
            try:
                self._set_peaq_claim_proxy = rospy.ServiceProxy(
                    "/teleop_fetch/set_peaq_dataset_claim",
                    SetPeaqDatasetClaim,
                )
            except Exception as e:
                logger.warning("SetPeaqDatasetClaim proxy not created: %s", e)

    def handle_request_help(self, req) -> RequestHelpResponse:
        """
        ROS Service handler for manual escalation.
        1. Formats HelpNeededEvent
        2. Calls RAID_APP
        3. Forwards received SessionGrant to teleop_fetch
        """
        sr_len = len(req.situation_report or "")
        logger.info(
            "Escalation requested for task %s (situation_report length=%d chars)",
            req.task_id,
            sr_len,
        )

        event = {
            "type": "HelpNeededEvent",
            "timestamp": int(time.time()),
            "task_id": req.task_id,
            "error_context": req.error_context,
            "situation_report": req.situation_report or "",
        }

        # Step 1 & 2: Call RAID_APP (Synchronous for simplicity here, could be async)
        try:
            grant_payload, signature = self._request_grant_from_raid(event)
        except Exception as e:
            msg = f"Failed to get grant from RAID: {e}"
            logger.error(msg)
            return RequestHelpResponse(success=False, message=msg)

        # Step 3: Forward to teleop_fetch
        if not self.teleop_grant_proxy:
            return RequestHelpResponse(success=False, message="Teleop proxy not initialized")

        try:
            rospy.wait_for_service('/teleop_fetch/receive_grant', timeout=5.0)
            res = self.teleop_grant_proxy(grant_payload=grant_payload, signature=signature)
            if res.success:
                return RequestHelpResponse(success=True, message="Grant received and forwarded to teleop successfully")
            else:
                return RequestHelpResponse(success=False, message=f"Teleop rejected grant: {res.message}")
        except rospy.ServiceException as e:
            msg = f"Failed to forward grant to teleop_fetch: {e}"
            logger.error(msg)
            return RequestHelpResponse(success=False, message=msg)


    def _request_grant_from_raid(self, event: Dict[str, Any]) -> Tuple[str, str]:
        """
        Calls RAID APP to request help and obtain a teleop grant.
        """
        import uuid
        
        if not self.robot_id or not self.teleop_secret:
            raise ValueError("RAID robot_id or teleop_secret not configured.")

        url = f"{self.raid_app_url}/api/robots/{self.robot_id}/teleop/help"
        headers = {
            "Content-Type": "application/json",
            "X-Robot-Teleop-Secret": self.teleop_secret
        }
        payload = {
            "message": "Need assistance",
            "metadata": {
                "task_id": event.get("task_id", "unknown"),
                "error_context": event.get("error_context", ""),
                "situation_report": event.get("situation_report", ""),
            },
        }
        if rospy.get_param("~raid_send_kyr_peaq_context", True):
            kyr_ctx = self._kyr_peaq_context_dict(event)
            if kyr_ctx is not None:
                payload["metadata"]["kyr_peaq_context"] = kyr_ctx

        logger.info("POST %s ...", url)
        response = requests.post(url, json=payload, headers=headers, timeout=10.0)

        if response.status_code == 401:
            raise ValueError(
                "RAID teleop/help rejected credentials (401). "
                "Check raid_robot_id and raid_teleop_secret (or re-enroll)."
            )
        if response.status_code not in (200, 201):
            response.raise_for_status()

        data = response.json()
        if response.status_code == 200 and data.get("duplicate"):
            logger.info("RAID teleop/help returned duplicate=true (request already open).")
        else:
            logger.info("RAID teleop/help status=%s", response.status_code)

        try:
            claim = self._maybe_obtain_peaq_claim(data)
            if claim:
                self._push_peaq_claim_to_dataset(claim)
            else:
                logger.info(
                    "No peaq claim to merge (RAID omitted inline/GET or wrong JSON keys; "
                    "expect peaq_claim or peaqClaim). help top-level keys: %s",
                    sorted(data.keys()) if isinstance(data, dict) else "n/a",
                )
        except Exception as e:
            logger.warning("peaq claim fetch/merge skipped (fail-open): %s", e)

        signed = extract_signed_grant_from_raid_help_response(data)
        if signed:
            logger.info("Using signed SessionGrant from RAID teleop/help response.")
            return signed[0], signed[1]

        # Fallback until RAID returns teleopGrantPayload + teleopGrantSignature (see DOC).
        grant = {
            "session_id": self._help_request_id(data) or str(uuid.uuid4()),
            "robot_id": self.robot_id,
            "task_id": event.get("task_id", "unknown"),
            "operator_pubkey": "pending_from_raid",
            "valid_until_sec": int(time.time()) + 3600,
            "scope_json": json.dumps({"allowed_actions": ["*"]}),
        }

        payload_str = json.dumps(grant)
        dummy_sig = "dummy_signature_base58_encoded"

        return payload_str, dummy_sig

    def _kyr_peaq_context_dict(self, event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if GetPeaqIssuanceMetadata is None:
            return None
        try:
            rospy.wait_for_service("/kyr/get_peaq_issuance_metadata", timeout=2.0)
            proxy = rospy.ServiceProxy("/kyr/get_peaq_issuance_metadata", GetPeaqIssuanceMetadata)
            res = proxy(
                task_id=str(event.get("task_id", "") or ""),
                error_context=str(event.get("error_context", "") or ""),
            )
            if not res.success or not (res.context_json or "").strip():
                logger.warning("KYR get_peaq_issuance_metadata: %s", res.message)
                return None
            return json.loads(res.context_json)
        except (rospy.ROSException, rospy.ServiceException, json.JSONDecodeError) as e:
            logger.warning("Could not load kyr_peaq_context: %s", e)
            return None

    def _help_request_id(self, help_response: Dict[str, Any]) -> Optional[str]:
        hid = help_response.get("id")
        if hid:
            return str(hid)
        nested = help_response.get("helpRequest")
        if isinstance(nested, dict) and nested.get("id"):
            return str(nested.get("id"))
        return None

    def _maybe_obtain_peaq_claim(self, help_response: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not rospy.get_param("~raid_peaq_claim_enabled", True):
            return None
        help_id = self._help_request_id(help_response)
        if not help_id:
            logger.warning("RAID teleop/help JSON has no id (or helpRequest.id); cannot fetch peaq claim")
            return None
        inline = extract_peaq_claim_object(help_response)
        if inline is not None:
            return inline
        return fetch_peaq_claim(
            self.raid_app_url,
            self.robot_id,
            self.teleop_secret,
            help_id,
            timeout_sec=float(rospy.get_param("~raid_peaq_claim_timeout_sec", 10.0)),
            poll_attempts=int(rospy.get_param("~raid_peaq_claim_poll_attempts", 3)),
            poll_delay_sec=float(rospy.get_param("~raid_peaq_claim_poll_delay_sec", 1.0)),
        )

    def _push_peaq_claim_to_dataset(self, claim: Dict[str, Any]) -> None:
        if not claim or not self._set_peaq_claim_proxy:
            return
        try:
            claim_json = json.dumps(claim, ensure_ascii=False)
            wait_sec = float(rospy.get_param("~raid_peaq_dataset_claim_wait_sec", 5.0))
            rospy.wait_for_service(
                "/teleop_fetch/set_peaq_dataset_claim",
                timeout=max(0.5, wait_sec),
            )
            res = self._set_peaq_claim_proxy(claim_json=claim_json, dataset_id="")
            if res.success:
                logger.info("peaq claim merged into dataset metadata")
            else:
                logger.warning("set_peaq_dataset_claim: %s", res.message)
        except (rospy.ROSException, rospy.ServiceException) as e:
            logger.warning("set_peaq_dataset_claim failed: %s", e)

    def close_and_pay(self, session_id: str, reason: str = "completed"):
        """
        Close KYR session and pay the operator (same SOL transfer path as /x402/complete_teleop_payment).
        Prefer calling KYR close from teleop_fetch and then complete_teleop_payment from the same receipt.
        """
        if not self.kyr_close_proxy:
            logger.error("Cannot close and pay: KYR CloseSession proxy unavailable.")
            return

        try:
            rospy.wait_for_service("/kyr/close_session", timeout=5.0)
            res = self.kyr_close_proxy(session_id=session_id, reason=reason)

            if not res.success:
                logger.error("KYR failed to close session: %s", res.message)
                return

            if not self.x402_client:
                logger.warning("x402_client unavailable; post-pay skipped.")
                return

            rate = float(rospy.get_param("~teleop_operator_payment_sol_per_sec", 1e-6))
            ok, msg, _sig = pay_operator_from_receipt_payload(
                self.x402_client, res.receipt_payload, rate
            )
            if ok:
                logger.info("close_and_pay: %s", msg)
            else:
                logger.error("close_and_pay: %s", msg)

        except rospy.ServiceException as e:
            logger.error("KYR close_session service call failed: %s", e)
        except Exception as e:  # pylint: disable=broad-except
            logger.error("Post-pay failed: %s", e)
