#!/usr/bin/env python3
"""Tests for teleop operator SOL payment helper."""

import unittest
from unittest.mock import MagicMock

from rospy_x402.teleop_operator_payment import pay_operator_from_receipt_payload


class TestTeleopOperatorPayment(unittest.TestCase):
    def test_skip_placeholder_pubkey(self):
        client = MagicMock()
        ok, msg, sig, amt = pay_operator_from_receipt_payload(
            client,
            '{"operator_pubkey":"pending_from_raid","started_at_sec":0,"ended_at_sec":10}',
            1e-6,
        )
        self.assertTrue(ok)
        self.assertIn("skipped", msg.lower())
        self.assertEqual(sig, "")
        self.assertEqual(amt, 0.0)
        client.send_payment.assert_not_called()

    def test_skip_zero_duration(self):
        client = MagicMock()
        ok, msg, sig, amt = pay_operator_from_receipt_payload(
            client,
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111","started_at_sec":5,"ended_at_sec":5}',
            1e-6,
        )
        self.assertTrue(ok)
        self.assertIn("Zero", msg)
        self.assertEqual(sig, "skipped_zero_amount")
        self.assertEqual(amt, 0.0)
        client.send_payment.assert_not_called()

    def test_pays_when_duration_positive(self):
        client = MagicMock()
        client.send_payment.return_value = "txsig123"
        payload = (
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111",'
            '"started_at_sec":0,"ended_at_sec":10}'
        )
        ok, msg, sig, amt = pay_operator_from_receipt_payload(client, payload, 0.001)
        self.assertTrue(ok)
        self.assertEqual(sig, "txsig123")
        self.assertEqual(amt, 0.01)
        client.send_payment.assert_called_once()
        args = client.send_payment.call_args[0]
        self.assertEqual(args[1], 0.01)  # 10 * 0.001 SOL

    def test_flat_sol_ignores_zero_duration(self):
        client = MagicMock()
        client.send_payment.return_value = "txflat"
        payload = (
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111",'
            '"started_at_sec":9,"ended_at_sec":9}'
        )
        ok, msg, sig, amt = pay_operator_from_receipt_payload(
            client, payload, sol_per_sec=1e-6, flat_sol=0.0005
        )
        self.assertTrue(ok)
        self.assertEqual(sig, "txflat")
        self.assertEqual(amt, 0.0005)
        client.send_payment.assert_called_once_with(
            "SoMeValidBase58Key111111111111111111111111111", 0.0005
        )

    def test_receipt_operator_payment_sol_overrides_flat(self):
        client = MagicMock()
        client.send_payment.return_value = "tx1"
        payload = (
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111",'
            '"operator_payment_sol":0.002,"started_at_sec":0,"ended_at_sec":1}'
        )
        ok, _, _, amt = pay_operator_from_receipt_payload(
            client, payload, sol_per_sec=0.0, flat_sol=0.0005
        )
        self.assertTrue(ok)
        self.assertEqual(amt, 0.002)
        client.send_payment.assert_called_once_with(
            "SoMeValidBase58Key111111111111111111111111111", 0.002
        )

    def test_abnormal_closure_halves_flat_amount(self):
        client = MagicMock()
        client.send_payment.return_value = "txhalf"
        payload = (
            '{"operator_pubkey":"SoMeValidBase58Key111111111111111111111111111",'
            '"closure_reason":"robot_watchdog_timeout",'
            '"started_at_sec":0,"ended_at_sec":0}'
        )
        ok, _, sig, amt = pay_operator_from_receipt_payload(
            client, payload, sol_per_sec=0.0, flat_sol=0.0004, abnormal_payment_fraction=0.5
        )
        self.assertTrue(ok)
        self.assertEqual(sig, "txhalf")
        self.assertAlmostEqual(amt, 0.0002)
        client.send_payment.assert_called_once_with(
            "SoMeValidBase58Key111111111111111111111111111", 0.0002
        )


if __name__ == "__main__":
    unittest.main()
