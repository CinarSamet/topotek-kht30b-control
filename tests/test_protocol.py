"""Cihaz gerektirmeyen testler: belgedeki ornek paketler, cihazdan kaydedilmis gercek paketler, girdi dogrulama."""
import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import kht30b as k


class Protocol(unittest.TestCase):
    def test_selftest(self):
        self.assertTrue(k.selftest())

    def test_speed_limit(self):
        with self.assertRaises(ValueError):
            k.s8(120)

    def test_bad_mode(self):
        with self.assertRaises(ValueError):
            k._check_mode("x")


if __name__ == "__main__":
    unittest.main()
