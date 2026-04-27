#!/usr/bin/env python3
"""Tests for RAID teleop/help → SessionGrant parsing."""

import unittest

from rospy_x402.raid_teleop_grant import (
    canonical_session_grant_json,
    extract_signed_grant_from_raid_help_response,
)


class TestRaidTeleopGrant(unittest.TestCase):
    def test_preferred_string_payload(self):
        data = {
            "id": "h1",
            "teleopGrantPayload": '{"session_id":"s1","robot_id":"r1"}',
            "teleopGrantSignature": "sig58",
        }
        p, s = extract_signed_grant_from_raid_help_response(data)
        self.assertEqual(p, '{"session_id":"s1","robot_id":"r1"}')
        self.assertEqual(s, "sig58")

    def test_alias_keys(self):
        data = {
            "grantPayload": '{"a":1}',
            "grantSignature": "ggsig",
        }
        p, s = extract_signed_grant_from_raid_help_response(data)
        self.assertEqual(p, '{"a":1}')
        self.assertEqual(s, "ggsig")

    def test_nested_help_request(self):
        inner = {
            "teleopGrantPayload": '{"x":true}',
            "teleopGrantSignature": "nested_sig",
        }
        p, s = extract_signed_grant_from_raid_help_response({"helpRequest": inner})
        self.assertEqual(p, '{"x":true}')
        self.assertEqual(s, "nested_sig")

    def test_session_grant_object_canonical(self):
        grant = {
            "session_id": "s",
            "robot_id": "r",
            "operator_pubkey": "op",
            "valid_until_sec": 99,
            "scope_json": "{}",
            "task_id": "t",
        }
        data = {
            "sessionGrant": grant,
            "teleopGrantSignature": "objsig",
        }
        p, s = extract_signed_grant_from_raid_help_response(data)
        self.assertEqual(s, "objsig")
        self.assertEqual(p, canonical_session_grant_json(grant))

    def test_none_when_missing(self):
        self.assertIsNone(extract_signed_grant_from_raid_help_response({}))
        self.assertIsNone(extract_signed_grant_from_raid_help_response({"id": "x"}))


if __name__ == "__main__":
    unittest.main()
