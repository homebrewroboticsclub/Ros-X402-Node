from typing import Iterable

_B58_ALPHABET = "123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz"
_B58_BASE = len(_B58_ALPHABET)
_B58_INDEXES = {char: index for index, char in enumerate(_B58_ALPHABET)}


def b58encode(data: bytes) -> str:
    if not data:
        return ""

    # Count leading zeros.
    zeros = 0
    for byte in data:
        if byte == 0:
            zeros += 1
        else:
            break

    # Convert to integer.
    num = int.from_bytes(data, "big")

    # Encode to base58.
    encoded = ""
    while num > 0:
        num, remainder = divmod(num, _B58_BASE)
        encoded = _B58_ALPHABET[remainder] + encoded

    return "1" * zeros + encoded


def b58decode(value: str) -> bytes:
    if not value:
        return b""

    num = 0
    for char in value:
        if char not in _B58_INDEXES:
            raise ValueError(f"Invalid base58 character: {char}")
        num = num * _B58_BASE + _B58_INDEXES[char]

    # Convert back to bytes.
    byte_length = (num.bit_length() + 7) // 8
    data = num.to_bytes(byte_length, "big")

    # Add leading zeros.
    leading_zeros = len(list(_leading_zeros(value)))
    return b"\x00" * leading_zeros + data.lstrip(b"\x00") or b"\x00"


def _leading_zeros(value: Iterable[str]) -> Iterable[str]:
    for char in value:
        if char == "1":
            yield char
        else:
            break

