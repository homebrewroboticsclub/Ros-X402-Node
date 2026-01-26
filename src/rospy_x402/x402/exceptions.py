class X402Error(Exception):
    """Base class for x402 protocol errors."""


class PaymentVerificationError(X402Error):
    """Raised when payment verification fails."""


class PaymentTimeoutError(X402Error):
    """Raised when payment is not completed within the allowed window."""


class PaymentSubmissionError(X402Error):
    """Raised when submitting a payment request to Solana fails."""

