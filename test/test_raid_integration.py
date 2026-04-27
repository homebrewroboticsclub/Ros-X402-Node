#!/usr/bin/env python3
"""Unit tests for RAID enroll / allowlist helpers (no ROS)."""

import json
import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.raid_integration import (
    build_enroll_body,
    build_operator_registry_url,
    credentials_from_state,
    enroll_robot,
    load_raid_state,
    normalized_sync_path,
    parse_enroll_credentials,
    parse_operator_sync_body,
    post_robot_enroll,
    save_operator_allowlist,
    save_raid_state,
)


class TestRaidIntegration(unittest.TestCase):
    def test_normalized_sync_path(self):
        self.assertEqual(normalized_sync_path("x"), "/x")
        self.assertEqual(normalized_sync_path("/a/b/"), "/a/b")

    def test_build_operator_registry_url(self):
        u = build_operator_registry_url("192.168.1.5", 18080, "/raid/sync")
        self.assertEqual(u, "http://192.168.1.5:18080/raid/sync")

    def test_build_enroll_body(self):
        b = build_enroll_body("key1", "h", 8080, rosbridge_port=9090, name="r1")
        self.assertEqual(b["enrollmentKey"], "key1")
        self.assertEqual(b["host"], "h")
        self.assertEqual(b["port"], 8080)
        self.assertEqual(b["rosbridgePort"], 9090)
        self.assertEqual(b["name"], "r1")

    def test_credentials_from_state(self):
        self.assertIsNone(credentials_from_state({}))
        c = credentials_from_state({"id": "u", "teleopSecret": "s"})
        self.assertEqual(c, ("u", "s"))

    def test_save_and_load_raid_state(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "state.json")
            save_raid_state(p, "rid", "sec")
            data = load_raid_state(p)
            self.assertIsNotNone(data)
            self.assertEqual(data["id"], "rid")
            self.assertEqual(data["teleopSecret"], "sec")

    def test_parse_enroll_credentials(self):
        rid, sec = parse_enroll_credentials({"id": "a", "teleopSecret": "b"})
        self.assertEqual((rid, sec), ("a", "b"))
        with self.assertRaises(ValueError):
            parse_enroll_credentials({"id": "a"})

    def test_parse_operator_sync_body(self):
        ids = parse_operator_sync_body({"allowedTeleoperatorIds": [" x ", "y"]})
        self.assertEqual(ids, ["x", "y"])
        with self.assertRaises(ValueError):
            parse_operator_sync_body({})

    def test_save_operator_allowlist(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "allow.json")
            save_operator_allowlist(p, ["u1", "u2"])
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.assertEqual(data["allowedTeleoperatorIds"], ["u1", "u2"])

    @patch("rospy_x402.raid_integration.requests.post")
    def test_post_robot_enroll(self, mock_post):
        mock_resp = MagicMock()
        mock_resp.json.return_value = {"id": "i", "teleopSecret": "t"}
        mock_resp.raise_for_status = MagicMock()
        mock_post.return_value = mock_resp

        out = post_robot_enroll(
            "http://raid:3000",
            "fleetsec",
            {"enrollmentKey": "k", "host": "h", "port": 1},
        )
        self.assertEqual(out["id"], "i")
        mock_post.assert_called_once()
        args, kwargs = mock_post.call_args
        self.assertIn("/api/robots/enroll", args[0])
        self.assertEqual(kwargs["headers"]["X-Robot-Fleet-Secret"], "fleetsec")

    @patch("rospy_x402.raid_integration.post_robot_enroll")
    def test_enroll_robot(self, mock_post):
        mock_post.return_value = {"id": "i", "teleopSecret": "t"}
        out = enroll_robot("http://x", "f", "ek", "host", 80)
        self.assertEqual(out["id"], "i")
        body = mock_post.call_args[0][2]
        self.assertEqual(body["enrollmentKey"], "ek")


if __name__ == "__main__":
    unittest.main()
