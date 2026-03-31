#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.raid_peaq_client import fetch_peaq_claim


class TestRaidPeaqClient(unittest.TestCase):
    def test_fetch_200_returns_claim(self):
        mock_resp = MagicMock()
        mock_resp.status_code = 200
        mock_resp.json.return_value = {"peaq_claim": {"schema_version": 1, "network": "peaq-agung"}}
        session = MagicMock()
        session.get.return_value = mock_resp

        out = fetch_peaq_claim(
            "http://raid/",
            "robot-uuid",
            "sec",
            "help-1",
            session=session,
        )
        self.assertEqual(out["network"], "peaq-agung")
        session.get.assert_called_once()

    @patch("rospy_x402.raid_peaq_client.time.sleep", MagicMock())
    def test_fetch_404_then_200(self):
        r404 = MagicMock()
        r404.status_code = 404
        r200 = MagicMock()
        r200.status_code = 200
        r200.json.return_value = {"peaq_claim": {"ok": True}}
        session = MagicMock()
        session.get.side_effect = [r404, r200]

        out = fetch_peaq_claim(
            "http://raid/",
            "r",
            "s",
            "h",
            poll_attempts=2,
            poll_delay_sec=0.01,
            session=session,
        )
        self.assertTrue(out.get("ok"))
        self.assertEqual(session.get.call_count, 2)


if __name__ == "__main__":
    unittest.main()
