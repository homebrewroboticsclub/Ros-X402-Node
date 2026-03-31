import getpass
import logging
import os
from dataclasses import dataclass
from typing import Optional

_logger = logging.getLogger(__name__)

from nacl import exceptions as nacl_exceptions
from nacl.signing import SigningKey

from .encoding import b58decode, b58encode


@dataclass
class KeyMaterial:
    signing_key: SigningKey

    @property
    def public_key_b58(self) -> str:
        return b58encode(bytes(self.signing_key.verify_key))

    @property
    def secret_key_bytes(self) -> bytes:
        return bytes(self.signing_key)

    @property
    def secret_key_64(self) -> bytes:
        verify_key_bytes = bytes(self.signing_key.verify_key)
        return self.secret_key_bytes + verify_key_bytes


class KeyManager:
    def __init__(self) -> None:
        self._key_material: Optional[KeyMaterial] = None

    def load_from_env(self, env_var: str = "SOLANA_PRIVATE_KEY") -> Optional[KeyMaterial]:
        """Load key from environment. Returns None if not set, empty, or invalid base58."""
        secret_b58 = (os.environ.get(env_var) or "").strip()
        if not secret_b58:
            return None
        try:
            key_material = self.load_from_base58(secret_b58)
        except ValueError as exc:
            _logger.warning(
                "Ignoring invalid %s from environment (%s). Unset it, fix base58, or use launch/env.",
                env_var,
                exc,
            )
            return None
        self._key_material = key_material
        return key_material

    def load_from_prompt(self) -> KeyMaterial:
        secret_input = getpass.getpass(
            prompt="Enter Solana private key (base58, kept only in RAM): "
        ).strip()
        if not secret_input:
            raise ValueError("Private key input is empty.")
        return self.load_from_base58(secret_input)

    def load_from_base58(self, secret_b58: str) -> KeyMaterial:
        secret_bytes = b58decode(secret_b58)
        if len(secret_bytes) not in (32, 64):
            raise ValueError(
                "Unsupported private key length. Expected 32 or 64 bytes after base58 decoding."
            )

        seed = secret_bytes[:32]
        try:
            signing_key = SigningKey(seed)
        except nacl_exceptions.CryptoError as exc:
            raise ValueError("Failed to construct signing key from provided secret.") from exc

        key_material = KeyMaterial(signing_key=signing_key)
        self._key_material = key_material
        return key_material

    @property
    def key_material(self) -> KeyMaterial:
        if not self._key_material:
            raise RuntimeError("Private key has not been loaded.")
        return self._key_material

