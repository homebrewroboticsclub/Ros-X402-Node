#!/usr/bin/env python3
"""Tests for RAID GET session-grant polling (no roscore)."""

import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.raid_session_grant_client import (
    _resolve_session_grant_get_url,
    poll_raid_session_grant,
)


class TestResolveSessionGrantUrl(unittest.TestCase):
    def test_default_path(self):
        u = _resolve_session_grant_get_url(
            "http://raid:3000",
            "rid-1",
            "help-uuid",
            {},
        )
        self.assertEqual(
            u,
            "http://raid:3000/api/robots/rid-1/teleop/session-grant?helpRequestId=help-uuid",
        )

    def test_teleop_grant_poll_url_relative(self):
        u = _resolve_session_grant_get_url(
            "http://raid:3000",
            "rid-1",
            "h1",
            {"teleopGrantPollUrl": "/api/robots/rid-1/teleop/session-grant?helpRequestId=h1"},
        )
        self.assertIn("session-grant", u)
        self.assertIn("helpRequestId=h1", u)


class TestPollSessionGrant(unittest.TestCase):
    @patch("rospy_x402.raid_session_grant_client.requests.get")
    def test_grant_not_ready_then_success(self, mock_get):
        ready = MagicMock()
        ready.status_code = 200
        ready.json.return_value = {
            "teleopGrantPayload": '{"session_id":"s","operator_pubkey":"Op111111111111111111111111111111111111111"}',
            "teleopGrantSignature": "sig58",
            "grantSignerPublicKey": "RaidSigner1111111111111111111111111111111",
        }
        waiting = MagicMock()
        waiting.status_code = 404
        waiting.json.return_value = {"error": "grant_not_ready"}
        mock_get.side_effect = [waiting, ready]

        p, s, signer = poll_raid_session_grant(
            "http://raid:3000",
            "rid",
            "sec",
            "hid",
            {},
            timeout_sec=5.0,
            interval_sec=0.01,
        )
        self.assertIn("operator_pubkey", p)
        self.assertEqual(s, "sig58")
        self.assertIn("RaidSigner", signer or "")


if __name__ == "__main__":
    unittest.main()
