#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.raid_peaq_client import extract_peaq_claim_object, fetch_peaq_claim


class TestExtractPeaqClaim(unittest.TestCase):
    def test_snake_case(self):
        self.assertEqual(
            extract_peaq_claim_object({"peaq_claim": {"x": 1}}),
            {"x": 1},
        )

    def test_camel_case(self):
        self.assertEqual(
            extract_peaq_claim_object({"peaqClaim": {"y": 2}}),
            {"y": 2},
        )

    def test_snake_wins_when_both(self):
        self.assertEqual(
            extract_peaq_claim_object({"peaq_claim": {"a": 1}, "peaqClaim": {"b": 2}}),
            {"a": 1},
        )


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
