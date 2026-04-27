"""X402Client.send_payment normalizes RPC signature values for ROS string fields."""

from __future__ import annotations

import unittest
from unittest.mock import MagicMock, patch

from rospy_x402.x402.client import X402Client
from rospy_x402.x402.key_manager import KeyMaterial


class _FakeSignature:
    """Mimics solders.signature.Signature: not a str, but str() is base58 ASCII."""

    def __str__(self) -> str:
        return "5VERv8NMvzbJMEkV8xnrLkEaWRtSz9CosKDYjCJjBRnb"


class TestX402ClientSendPayment(unittest.TestCase):
    def test_send_payment_coerces_non_string_signature(self) -> None:
        km = MagicMock(spec=KeyMaterial)
        km.secret_key_64 = b"\x01" * 64
        km.public_key_b58 = "11111111111111111111111111111112"

        client = X402Client(rpc_endpoint="http://localhost:8899")
        client._key_material = km  # pylint: disable=protected-access

        fake_sender = MagicMock()
        fake_sender.transfer_lamports.return_value = {"result": _FakeSignature()}

        with patch(
            "rospy_x402.x402.client.SolanaTransactionSender",
            return_value=fake_sender,
        ):
            out = client.send_payment("11111111111111111111111111111112", 0.001)

        self.assertIsInstance(out, str)
        self.assertEqual(out, "5VERv8NMvzbJMEkV8xnrLkEaWRtSz9CosKDYjCJjBRnb")
        fake_sender.transfer_lamports.assert_called_once()


if __name__ == "__main__":
    unittest.main()
