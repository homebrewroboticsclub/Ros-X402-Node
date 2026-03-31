"""Mirror log schema matches KYR dashboard_events.jsonl."""

import json
import os
import tempfile
import unittest


class TestDashboardEventsLog(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("KYR_HOME")
        os.environ["KYR_HOME"] = self._td.name

    def tearDown(self):
        self._td.cleanup()
        if self._prev is None:
            os.environ.pop("KYR_HOME", None)
        else:
            os.environ["KYR_HOME"] = self._prev

    def test_append_writes_expected_keys(self):
        from rospy_x402.dashboard_events_log import append_dashboard_event

        append_dashboard_event(
            "rospy_x402",
            "help_request_start",
            "test",
            {"task_id": "t1"},
        )
        p = os.path.join(self._td.name, "dashboard_events.jsonl")
        self.assertTrue(os.path.isfile(p))
        with open(p, encoding="utf-8") as f:
            row = json.loads(f.readline())
        self.assertEqual(row["source"], "rospy_x402")
        self.assertEqual(row["kind"], "help_request_start")
        self.assertIn("timestamp_iso", row)
        self.assertEqual(row["metadata"]["task_id"], "t1")


if __name__ == "__main__":
    unittest.main()
