"""Prevent BEEP's boot-time vendor/bridge ordering cycle."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ServiceBootDependencyTests(unittest.TestCase):
    def test_vendor_app_does_not_wait_for_oled_after_multi_user(self):
        # The installed Yahboom OLED unit has After=multi-user.target.
        # Both vendor and bridge are WantedBy=multi-user.target, so waiting
        # for OLED would cause systemd to drop boot jobs to break the cycle.
        vendor = (ROOT / 'systemd/beep-vendor-app.service').read_text()
        bridge = (ROOT / 'systemd/beep-bridge.service').read_text()
        order = [line.split('=', 1)[1].split() for line in vendor.splitlines()
                 if line.startswith('After=')]
        self.assertEqual(len(order), 1)
        self.assertNotIn('yahboom_oled.service', order[0])
        self.assertNotIn('multi-user.target', order[0])
        self.assertIn('network-online.target', order[0])
        self.assertIn('DOGZILLA_SKIP_STARTUP_ACTION=1', vendor)
        self.assertIn('WantedBy=multi-user.target', vendor)
        self.assertIn('WantedBy=multi-user.target', bridge)


if __name__ == '__main__':
    unittest.main()
