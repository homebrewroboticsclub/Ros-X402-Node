from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Dict, Optional
from uuid import uuid4


@dataclass
class PaymentRequest:
    receiver_account: str
    amount: float
    asset_symbol: str = "SOL"
    reference: str = field(default_factory=lambda: uuid4().hex)


@dataclass
class PaymentSession:
    request: PaymentRequest
    created_at: datetime
    expires_at: datetime
    signature: Optional[str] = None
    metadata: Dict[str, str] = field(default_factory=dict)

    @property
    def is_expired(self) -> bool:
        return datetime.utcnow() >= self.expires_at

    @classmethod
    def create(cls, request: PaymentRequest, ttl_seconds: int) -> "PaymentSession":
        created = datetime.utcnow()
        return cls(
            request=request,
            created_at=created,
            expires_at=created + timedelta(seconds=ttl_seconds),
        )

