#!/usr/bin/env python3
"""Tests for teleop operator SOL payment helper."""

import unittest
from unittest.mock import MagicMock

from rospy_x402.teleop_operator_payment import pay_operator_from_receipt_payload


class TestTeleopOperatorPayment(unittest.TestCase):
    def test_skip_placeholder_pubkey(self):
        client = MagicMock()
        ok, msg, sig = pay_operator_from_receipt_payload(
            client,
            '{"operator_pubkey":"pending_from_raid","started_at_sec":0,"ended_at_sec":10}',
            1e-6,
        )
        self.assertTrue(ok)
        self.assertIn("skipped", msg.lower())
        self.assertEqual(sig, "")
        client.send_payment.assert_not_called()

    def test_skip_zero_duration(self):
        client = MagicMock()
        ok, msg, sig = pay_operator_from_receipt_payload(
            client,
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111","started_at_sec":5,"ended_at_sec":5}',
            1e-6,
        )
        self.assertTrue(ok)
        self.assertIn("Zero", msg)
        self.assertEqual(sig, "skipped_zero_amount")
        client.send_payment.assert_not_called()

    def test_pays_when_duration_positive(self):
        client = MagicMock()
        client.send_payment.return_value = "txsig123"
        payload = (
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111",'
            '"started_at_sec":0,"ended_at_sec":10}'
        )
        ok, msg, sig = pay_operator_from_receipt_payload(client, payload, 0.001)
        self.assertTrue(ok)
        self.assertEqual(sig, "txsig123")
        client.send_payment.assert_called_once()
        args = client.send_payment.call_args[0]
        self.assertEqual(args[1], 0.01)  # 10 * 0.001 SOL


if __name__ == "__main__":
    unittest.main()
