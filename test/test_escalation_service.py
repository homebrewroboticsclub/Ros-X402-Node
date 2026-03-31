#!/usr/bin/env python3
"""Unit tests for teleop/help HTTP payload (no roscore)."""

import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.escalation_service import EscalationManager


def _claim_off_get_param(name, default=None):
    if name == "~raid_peaq_claim_enabled":
        return False
    return default


class TestEscalationTeleopHelpPayload(unittest.TestCase):
    @patch("rospy_x402.escalation_service.rospy.get_param", side_effect=_claim_off_get_param)
    @patch("rospy_x402.escalation_service.rospy.ServiceProxy", MagicMock())
    @patch("rospy_x402.escalation_service.rospy.Service", MagicMock())
    @patch("rospy_x402.escalation_service.requests.post")
    def test_help_payload_includes_situation_report(self, mock_post, _mock_get_param):
        mock_resp = MagicMock()
        mock_resp.status_code = 201
        mock_resp.json.return_value = {"id": "help-req-1"}
        mock_post.return_value = mock_resp

        with patch.object(EscalationManager, "_kyr_peaq_context_dict", return_value=None):
            mgr = EscalationManager("http://raid:3000", "robot-uuid", "secret")
            event = {
                "task_id": "t1",
                "error_context": '{"code":1}',
                "situation_report": "Stuck at shelf B2 after three pick attempts.",
            }
            mgr._request_grant_from_raid(event)

        mock_post.assert_called_once()
        kwargs = mock_post.call_args[1]
        self.assertEqual(kwargs["json"]["metadata"]["task_id"], "t1")
        self.assertEqual(kwargs["json"]["metadata"]["error_context"], '{"code":1}')
        self.assertEqual(
            kwargs["json"]["metadata"]["situation_report"],
            "Stuck at shelf B2 after three pick attempts.",
        )

    @patch("rospy_x402.escalation_service.rospy.get_param", side_effect=_claim_off_get_param)
    @patch("rospy_x402.escalation_service.rospy.ServiceProxy", MagicMock())
    @patch("rospy_x402.escalation_service.rospy.Service", MagicMock())
    @patch("rospy_x402.escalation_service.requests.post")
    def test_help_payload_empty_situation_report(self, mock_post, _mock_get_param):
        mock_resp = MagicMock()
        mock_resp.status_code = 201
        mock_resp.json.return_value = {"id": "x"}
        mock_post.return_value = mock_resp

        with patch.object(EscalationManager, "_kyr_peaq_context_dict", return_value=None):
            mgr = EscalationManager("http://raid:3000", "rid", "sec")
            mgr._request_grant_from_raid({"task_id": "t", "error_context": ""})

        body = mock_post.call_args[1]["json"]
        self.assertIn("situation_report", body["metadata"])
        self.assertEqual(body["metadata"]["situation_report"], "")


if __name__ == "__main__":
    unittest.main()
