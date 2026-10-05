#!/usr/bin/env python3
"""Topotek KHT30B  -  UDP kontrol kutuphanesi + terminal araci   (surum 1.0)

Hem `from kht30b import KHT30B` ile kendi sisteminize gomulur,
hem de terminalden calisir:  python kht30b.py <komut> [argumanlar] [--json] [-v]

Protokol: Topotek UDP&UART Protocol V1.1.3 + GroundStation V4.1.47'den cikarilan komutlar.
Cihaz IP: KHT_IP ortam degiskeni (vars. 192.168.144.108), port 9003; bu program 9004'u dinler
(GroundStation vb. ayni anda acik olmamali).

YONLER (cihazda dogrulandi): yaw + = saga, pitch + = asagi.
MODLAR: 'gyro' (varsayilan) = uzaya/yere gore, stabilize, dron icin; 'enc' = govdeye gore enkoder acisi.
Gyro modunda yaw hareketin %12-17'si eksik kalir; goto_wait / --wait bunu kalan hatayi
ekleyerek otomatik duzeltir (tipik sonuc: yaw <0.15 derece, pitch <0.05 derece).

DOGRULANDI (gercek cihaz):  attitude (gyro+enkoder), goto (enc/gyro, --wait), home, rate, zoom (goruntude;
  `mag` okumasi gercek zoomu izlemiyor, hep 1.0), day/night, motor on/off, res, codec, rec, snap, sd, pip,
  palette, lrf.
GERCEK PAKETLERLE DOGRULANDI (okuma/cozme): watch, temps, zoompos.
CIHAZDA ETKI GORULMEDI (bu modelde desteklenmiyor olabilir): focus, dzoom, mag <x>, rotate (cihaz ayari
  degisiyor ama goruntu donmuyor), gps/uav (lazer hedef koordinati hesaplanmiyor).
BELIRSIZ: bitrate (gonderiliyor, geri okuma 'K0' doner), synctime (geri okuma yok).
Uygulamadaki takip/kalibrasyon komutlari (TRC, DRV, SAC, CHA...) eklenmedi; `raw` ile gonderilebilir.

Kutuphane tek is parcacigi icin yazildi (ayni KHT30B nesnesini birden cok thread'den kullanmayin).
"""
import argparse, json, os, socket, sys, time
from datetime import datetime, timezone

DEV_IP = os.environ.get("KHT_IP", "192.168.144.108")
DEV_PORT = int(os.environ.get("KHT_PORT", "9003"))
LOCAL_PORT = 9004
DEFAULT_MODE = "gyro"
# Yazilim sinirlari (derece). Kaynak: KHT30B satici teknik ozeti (yaw -280..280, pitch -30..120).
# Cihazin gercek mekanik araligi farkliysa buradan degistirin.
YAW_RANGE = (-280.0, 280.0)
PITCH_RANGE = (-30.0, 120.0)


# ------------------------------------------------------------ paket katmani
def crc(body: bytes) -> bytes:
    return b"%02X" % (sum(body) & 0xFF)


def build(dst, rw, ident, data=b"", src="P") -> bytes:
    """#tp/#TP + src + dst + uzunluk + w/r + ID + veri + CRC"""
    if isinstance(data, str):
        data = data.encode("latin1")
    n = len(data)
    if n <= 0xF:
        head, ln = ("#TP" if n == 2 else "#tp"), "%X" % n
    elif n <= 0xFF:
        head, ln = "#tP", "%02X" % n
    else:
        head, ln = "#Tp", "%04X" % n
    body = f"{head}{src}{dst}{ln}{rw}{ident}".encode() + data
    return body + crc(body)


def parse(pkt: bytes):
    """Gelen paketi coz -> dict(src,dst,rw,ident,data,crc_ok) ya da None"""
    i = pkt.find(b"#")
    if i < 0 or len(pkt) < i + 10:
        return None
    p = pkt[i:]
    head = p[:3]
    try:
        if head == b"#tP":
            ln, off = int(p[5:7], 16), 7
        elif head == b"#Tp":
            ln, off = int(p[5:9], 16), 9
        else:
            ln, off = int(p[5:6], 16), 6
    except ValueError:
        return None
    if p[off:off + 3] == b"TMP":  # sicaklik paketi: w/r karakteri yok (cihazin kendi bicimi)
        data = p[off + 3:off + 3 + ln]
        ok = p[off + 3 + ln:off + 5 + ln] == crc(p[:off + 3 + ln])
        return dict(src=chr(p[3]), dst=chr(p[4]), rw="w", ident="TMP", data=data, crc_ok=ok)
    rw, ident = p[off:off + 1].decode("latin1"), p[off + 1:off + 4].decode("latin1")
    data = p[off + 4:off + 4 + ln]
    got = p[off + 4 + ln:off + 6 + ln]
    ok = got == crc(p[:off + 4 + ln])
    return dict(src=chr(p[3]), dst=chr(p[4]), rw=rw, ident=ident, data=data, crc_ok=ok)


def s16(h):  # 4 hex karakter -> isaretli 16 bit
    v = int(h, 16)
    return v - 65536 if v >= 32768 else v


def ang4(deg):  # derece -> 4 hex (0.01 derece, ikili tumleyen)
    v = round(deg * 100)
    if not -32768 <= v <= 32767:
        raise ValueError("aci araligi disinda")
    return "%04X" % (v & 0xFFFF)


def s8(v, lim=99):
    v = int(v)
    if not -lim <= v <= lim:
        raise ValueError(f"deger {-lim}..{lim} araliginda olmali")
    return "%02X" % (v & 0xFF)


def decode_att(d):
    """12 hex karakter (yaw,pitch,roll; 0.01 derece, isaretli) -> derece"""
    d = d.decode() if isinstance(d, bytes) else d
    return dict(yaw=s16(d[0:4]) / 100, pitch=s16(d[4:8]) / 100, roll=s16(d[8:12]) / 100)


def decode_tmp(d):
    """Sicaklik verisi (22 hex): max(x3,y2,T4) min(x3,y2,T4) merkez(T4); sicaklik 0.1 C biriminde."""
    d = d.decode() if isinstance(d, bytes) else d
    h = lambda a, b: int(d[a:b], 16)
    return dict(max_c=h(5, 9) / 10, max_xy=(h(0, 3), h(3, 5)),
                min_c=h(14, 18) / 10, min_xy=(h(9, 12), h(12, 14)), center_c=h(18, 22) / 10)


def _check_mode(mode):
    if mode not in ("gyro", "enc"):
        raise ValueError("mode 'gyro' ya da 'enc' olmali")


# ------------------------------------------------------------ cihaz sinifi
class KHT30B:
    def __init__(self, ip=DEV_IP, port=DEV_PORT, local_port=LOCAL_PORT):
        self.addr = (ip, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.s.bind(("0.0.0.0", local_port))
        except OSError as e:
            self.s.close()
            raise OSError(f"UDP {local_port} portu kullanimda; GroundStation vb. uygulamayi kapatin") from e
        self.s.settimeout(0.2)
        if hasattr(socket, "SIO_UDP_CONNRESET"):  # Windows: ICMP 'port unreachable' recv'de hata firlatmasin
            try:
                self.s.ioctl(socket.SIO_UDP_CONNRESET, False)
            except (OSError, ValueError):
                pass
        self.verbose = False
        self.force = False  # True: aci sinir kontrolunu atla
        self._last_raw, self._last_t = None, 0.0

    def close(self):
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -- alt seviye
    def send(self, dst, rw, ident, data=b"", src="P"):
        pkt = build(dst, rw, ident, data, src)
        if self.verbose:
            print("->", pkt, file=sys.stderr)
        self.s.sendto(pkt, self.addr)

    def recv_all(self, secs):
        """secs boyunca gelen tum paketleri coz ve listele"""
        out, end = [], time.time() + secs
        while time.time() < end:
            try:
                raw, _ = self.s.recvfrom(4096)
            except socket.timeout:
                continue
            except ConnectionResetError:  # Windows: onceki gonderime ICMP hatasi; cihaz yok/erisilemiyor
                time.sleep(0.05)
                continue
            # cihaz her paketi iki kez gonderiyor: ayni paket <50 ms icinde tekrar gelirse at
            now = time.time()
            if raw == self._last_raw and now - self._last_t < 0.05:
                continue
            self._last_raw, self._last_t = raw, now
            if self.verbose:
                print("<-", raw, file=sys.stderr)
            m = parse(raw)
            if m:
                out.append(m)
        return out

    def drain(self):
        self.s.settimeout(0.01)
        try:
            while True:
                self.s.recvfrom(4096)
        except (socket.timeout, ConnectionResetError):
            pass
        self.s.settimeout(0.2)

    def ask(self, dst, ident, data="00", expect=None, timeout=1.5, rw="r", min_len=0):
        """Gonder ve beklenen ID'li (expect: str ya da liste) ilk yaniti bekle (yoksa None)"""
        self.drain()
        self.send(dst, rw, ident, data)
        expect = expect or ident
        expect = (expect,) if isinstance(expect, str) else tuple(expect)
        end = time.time() + timeout
        while time.time() < end:
            for m in self.recv_all(0.2):
                if m["ident"] in expect and len(m["data"]) >= min_len:
                    return m
        return None

    # -- gimbal: okuma
    def attitude(self, kind="gyro", timeout=1.5):
        """kind='gyro' (uzaya gore, stabilize) | 'enc' (govdeye gore, enkoder). Derece dondurur,
        yanit alinamazsa None. Gercek cihazda gyro okumasi GIC ile sorulur, yanit 'GIA' gelir."""
        _check_mode(kind)
        m = None
        for _ in range(3):  # UDP: istek/yanit ara sira kayboluyor, 3 kez dene
            if kind == "gyro":
                m = self.ask("G", "GIC", "00", expect=("GIA", "GIC"), timeout=timeout / 2, min_len=12)
            else:
                m = self.ask("G", "GAC", "00", timeout=timeout / 2, min_len=12)
            if m:
                return decode_att(m["data"])
        return None

    def stream(self, on=True, gyro=True, enc=True):
        """Aktif attitude yayini ac/kapat (GIA / GAA); ~10 Hz paket gelir (recv_all ile okunur)"""
        v = "01" if on else "00"
        if gyro:
            self.send("G", "w", "GIA", v)
        if enc:
            self.send("G", "w", "GAA", v)

    # -- gimbal: hareket
    def stop(self):
        self.send("G", "w", "PTZ", "00")

    def home(self):
        self.send("G", "w", "PTZ", "05")

    def ptz(self, code):
        self.send("G", "w", "PTZ", "%02X" % code)

    def rate(self, yaw, pitch):
        """Hiz modu, der/sn (-99..99). pitch + = asagi. Durdurmak icin stop()."""
        self.send("G", "w", "GSM", s8(yaw) + s8(pitch))

    def goto(self, yaw=None, pitch=None, roll=None, speed=30, mode=DEFAULT_MODE):
        """Konuma git (beklemeden). mode='gyro' uzaya gore (GI*), 'enc' govdeye gore (GA*).
        Aci derece, hiz 1..99 der/sn. yaw + = saga, pitch + = asagi.
        Aralik siniri YAW_RANGE/PITCH_RANGE ile korunur; asmak icin self.force = True / --force."""
        _check_mode(mode)
        if not 1 <= int(speed) <= 99:
            raise ValueError("hiz 1..99 der/sn olmali")
        for name, v, (lo, hi) in (("yaw", yaw, YAW_RANGE), ("pitch", pitch, PITCH_RANGE)):
            if v is not None and not (lo <= v <= hi) and not self.force:
                raise ValueError(f"{name}={v} sinir disi ({lo}..{hi}); zorlamak icin --force")
        p = "GA" if mode == "enc" else "GI"
        sp = "%02X" % int(speed)
        if yaw is not None and pitch is not None and roll is None:
            self.send("G", "w", p + "M", ang4(yaw) + sp + ang4(pitch) + sp)
            return
        if yaw is not None:
            self.send("G", "w", p + "Y", ang4(yaw) + sp)
        if pitch is not None:
            self.send("G", "w", p + "P", ang4(pitch) + sp)
        if roll is not None:
            self.send("G", "w", p + "R", ang4(roll) + sp)

    def goto_wait(self, yaw, pitch, speed=30, mode=DEFAULT_MODE, tol=0.2, timeout=20, retries=3, fine_speed=None):
        """Konuma git, varana kadar bekle; dict dondurur: reached, corrections, yaw, pitch, roll.
        Gimbal durup hedefin tol'den uzakta kaldiysa kalan hatayi komuta ekleyip yeniden dener
        (gyro modunda yaw %12-17 eksik kaliyor). fine_speed: duzeltme hareketlerinin hizi (opsiyonel)."""
        _check_mode(mode)
        cy, cp, tries = yaw, pitch, 0
        self.goto(cy, cp, speed=speed, mode=mode)
        end, a, prev = time.time() + timeout, None, None
        while time.time() < end:
            time.sleep(0.2)
            a = self.attitude(mode, timeout=1.2)
            if not a:
                continue
            ey, ep = yaw - a["yaw"], pitch - a["pitch"]
            if abs(ey) <= tol and abs(ep) <= tol:
                return dict(reached=True, corrections=tries, **a)
            settled = prev and abs(a["yaw"] - prev["yaw"]) < 0.05 and abs(a["pitch"] - prev["pitch"]) < 0.05
            if settled and tries < retries:  # durdu ama hedefte degil -> duzelt
                tries += 1
                cy = max(min(cy + ey, yaw + 10), yaw - 10)
                cp = max(min(cp + ep, pitch + 10), pitch - 10)
                self.goto(cy, cp, speed=fine_speed or speed, mode=mode)
                prev = None
                continue
            prev = a
        return dict(reached=False, corrections=tries, **(a or {}))

    def motor(self, on):
        self.send("G", "w", "GMS", "01" if on else "00")

    # -- lens
    def zoom(self, v):  # 0 dur, 1 uzaklas, 2 yakinlas (hareket icin ardindan 0 ile durdurun)
        self.send("M", "w", "ZMC", "%02X" % v)

    def focus(self, v):  # 0 dur, 1 +, 2 -, 0x10 oto, 0x11 manuel
        self.send("M", "w", "FCC", "%02X" % v)

    def zoom_pos(self):
        m = self.ask("M", "ZOM", min_len=4)
        return s16(m["data"][:4].decode()) if m else None

    def set_mag(self, x):  # optik buyutme (kismi modeller), x: 12.3 gibi
        self.send("M", "w", "MUL", "%04d" % round(x * 10))

    def get_mag(self):
        """Mevcut buyutme (x). Gercek cihazda ZMP okumasi 3 hane (0.1x) dondurur: '142' -> 14.2x"""
        m = self.ask("M", "ZMP", min_len=3)
        try:
            return int(m["data"][:3]) / 10 if m else None
        except ValueError:
            return None

    def temps(self, wait=1.5):
        """Termal sicakliklar (C). Cihaz bunlari kendiliginden surekli yayinlar; sonuncuyu dondurur."""
        last = None
        for m in self.recv_all(wait):
            if m["ident"] == "TMP" and len(m["data"]) >= 22:
                last = m["data"]
        return decode_tmp(last) if last else None

    # -- goruntu / kayit
    def pip(self, v="0A"):       self.send("D", "w", "PIP", v)      # 00..03, 0A sonraki
    def palette(self, v="0A"):   self.send("D", "w", "IMG", v)      # 00..0B, 0A sonraki
    def irc(self, v):            self.send("D", "w", "IRC", v)      # 00 gunduz, 01 gece

    def _feedback(self, ident, wait=2.0):
        """Cihazin kendiliginden gonderdigi geri bildirim paketini bekle (REC/CAP: '11'+8 hex dosya no)"""
        for m in self.recv_all(wait):
            if m["ident"] == ident and len(m["data"]) >= 10:
                return m["data"].decode(errors="replace")
        return None

    def record(self, v="0A"):  # 00 dur, 01 basla, 0A degistir -> geri bildirimi dondurur
        self.drain()
        self.send("D", "w", "REC", v)
        fb = self._feedback("REC")
        if not fb:
            return dict(feedback=None, note="geri bildirim gelmedi (kart takili mi?)")
        return dict(recording=fb[:2] == "11", file_id=int(fb[2:10], 16))

    def snap(self, kind=1):  # 1 gorunur+termal, 2 gorunur, 3 termal, 5 +sicaklik -> dosya no
        self.drain()
        self.send("D", "w", "CAP", "0%d" % kind)
        fb = self._feedback("CAP")
        if not fb:
            return dict(feedback=None, note="geri bildirim gelmedi (kart takili mi?)")
        return dict(ok=fb[:2] == "11", file_id=int(fb[2:10], 16))

    def rotate(self, deg):       self.send("D", "w", "ROT", "00" if deg == 0 else "02")

    def resolution(self, target, res):  # target 0 kayit,1 foto,2 rtsp; res 0 nativ,1 1080p,2 720p,3 480p
        self.send("D", "w", "VID", "%d%d" % (target, res))

    def bitrate_kbps(self, k):
        """stream=0 bitrate, kb/s (50..99999): 'K' + 5 hane ondalik (belge 5.4.1)"""
        if not 50 <= int(k) <= 99999:
            raise ValueError("bitrate 50..99999 kbps olmali")
        self.send("D", "w", "BIT", "K%05d" % int(k))

    def bitrate_mbps(self, m):
        """stream=0 bitrate, Mbps (1..8): x1='0', x2=Mbps (belge 5.4.1); 6 karakterlik veri alanina sifirla tamamlanir"""
        if not 1 <= int(m) <= 8:
            raise ValueError("bitrate 1..8 Mbps olmali")
        self.send("D", "w", "BIT", "0%d0000" % int(m))
    def encoder(self, stream, h265):  self.send("D", "w", "ENT", "%d%d" % (stream, 1 if h265 else 0))
    def dzoom(self, cam, x):     self.send("D", "w", "DZM", "%d0F%03d" % (cam, round(x * 10)))
    def dzoom_step(self, cam, code):  self.send("D", "w", "DZM", "%d%s" % (cam, code))  # 0A+ 0B- 0C 0D 0E dur

    def sd(self):
        """Hafiza karti: dict(free_mb, total_mb); kart yoksa None degerleri"""
        r = self.ask("D", "SDC", "00")
        t = self.ask("D", "SDC", "01")

        def f(m):
            try:
                return int(m["data"][:5], 16) if m and b"N" not in m["data"][:5] else None
            except ValueError:
                return None
        return dict(free_mb=f(r), total_mb=f(t))

    # -- lazer
    def lrf(self, mode="single", wait=3.0):
        """Tek atis olcum; dict(distance_m, [lon, lat]). Gecerli olcum yoksa distance_m=None.
        Hedef koordinati yalnizca cihaza gps()/uav() verisi beslendiyse anlamlidir."""
        self.drain()
        self.send("E", "w", "LRF", {"stop": "00", "enable": "01", "single": "02", "cont": "03"}[mode])
        # Cihaz bosta da surekli 'ERR.---' / '00000.0' / sifir koordinat yayinlar; yalnizca
        # gercek (>0) olcumu al, yoksa durumu bildir.
        res, status = {}, None
        for m in self.recv_all(wait):
            d = m["data"].decode(errors="replace")
            if m["ident"] == "LRF":
                try:
                    v = float(d)
                except ValueError:
                    status = "ERR"
                    continue
                if v > 0:
                    res["distance_m"] = v
                else:
                    status = status or "0"
            elif m["ident"] in ("LON", "LAT") and "distance_m" in res:
                try:
                    if float(d[1:]) != 0:  # sifir koordinat = bosta yayini, atla
                        res[m["ident"].lower()] = d
                except ValueError:
                    pass
        if "distance_m" not in res:
            res = dict(distance_m=None, status="gecerli olcum yok" + (f" ({status})" if status else ""))
        return res

    # -- ucus kontrolcusu verisi (hedef koordinati hesabi icin; cihazda denenmedi)
    def feed_gps(self, heading, lon, lat, alt_m, src="P"):
        """heading: burun yonu (derece), lon/lat: derece, alt_m: metre"""
        data = (int(round(heading * 100)).to_bytes(2, "little", signed=True)
                + int(round(lon * 1e7)).to_bytes(4, "little", signed=True)
                + int(round(lat * 1e7)).to_bytes(4, "little", signed=True)
                + int(round(alt_m * 1000)).to_bytes(4, "little", signed=True))
        self.s.sendto(build("D", "w", "GPS", data, src), self.addr)

    def feed_uav(self, yaw, pitch, roll, speed, track, src="P"):
        """Ucak attitude: yaw/pitch/roll/track derece, speed m/s (pitch + = asagi, roll + = sag)"""
        data = (int(round(yaw * 100)).to_bytes(2, "little", signed=True)
                + int(round(pitch * 100)).to_bytes(2, "little", signed=True)
                + int(round(roll * 100)).to_bytes(2, "little", signed=True)
                + int(round(speed * 100)).to_bytes(2, "little")
                + int(round(track * 100)).to_bytes(2, "little"))
        self.s.sendto(build("D", "w", "UAV", data, src), self.addr)

    def sync_time(self, beijing=False):
        if beijing:  # TIM: YYYYMMDDHHMMSS (belgede 'Beijing time'; yerel saati yollar)
            self.send("D", "w", "TIM", datetime.now().strftime("%Y%m%d%H%M%S"))
        else:
            self.send("D", "w", "UTC", datetime.now(timezone.utc).strftime("%H%M%S%d%m%y"))

    # -- belgeden eklenenler (UDP&UART V1.1.3); cihazda denenmemis olanlar test_doc.py ile denenir
    def read(self, dst, ident, data="00", timeout=1.2):
        """Genel okuma: yanit verisi (str) ya da None"""
        m = self.ask(dst, ident, data, timeout=timeout)
        return m["data"].decode(errors="replace") if m else None

    def yaw_mode(self, mode):
        """Gimbal yaw modu: 'lock' (yon kilidi, 06), 'follow' (govdeyi takip, 07), 'toggle' (08)"""
        self.ptz({"lock": 0x06, "follow": 0x07, "toggle": 0x08}[mode])

    def calibrate(self):
        """Gimbal kalibrasyonu (PTZ 09). Gimbal hareketsiz ve dengeli olmali!"""
        self.ptz(0x09)

    def focus_pos(self):
        m = self.ask("M", "FOC", min_len=4)
        return s16(m["data"][:4].decode()) if m else None

    def set_zoom_focus(self, zoom=None, focus=None):
        """ZFP: zoom/odak konumu ayarla (isaretli 16 bit); None = degistirme. Ardindan oto odak yapilir."""
        z = "NNNN" if zoom is None else "%04X" % (int(zoom) & 0xFFFF)
        f = "NNNN" if focus is None else "%04X" % (int(focus) & 0xFFFF)
        self.send("M", "w", "ZFP", z + f)

    def aperture(self, v):  # 0 dur, 1 azalt, 2 artir (kismi modeller)
        self.send("M", "w", "APC", "%02X" % v)

    def get_mul(self):
        """Optik buyutme okuma (MUL, kismi modeller): x ya da None"""
        d = self.read("M", "MUL")
        try:
            return int(d[:4]) / 10 if d else None
        except ValueError:
            return None

    def dzoom_get(self):
        """DZM okuma: dict(termal_x, gorunur_x) ya da None (veri X0X1 termal, X2X3 gorunur; 0.1x)"""
        d = self.read("D", "DZM")
        try:
            return dict(termal_x=int(d[:2], 16) / 10, gorunur_x=int(d[2:4], 16) / 10) if d and len(d) >= 4 else None
        except ValueError:
            return dict(raw=d)

    def fog(self, mode, intensity=0):  # mode 0 kapali, 1 oto, 2 manuel; siddet 0..255
        self.send("D", "w", "DFG", "%d%02X" % (mode, intensity))

    def temp_range(self, second):  # False: -20..150 C, True: -20..550 C
        self.send("D", "w", "TMR", "01" if second else "00")

    def temp_region(self, x, y, w=0, h=0, wait=1.5):
        """Nokta/bolge sicakligi (kismi modeller). w=h=0: nokta. Ham yanit verisi (son 4 hane = ort. sicaklik, 0.01 C) ya da None"""
        self.drain()
        self.send("D", "r", "TMP", "%03d%03d%02d%02d" % (x, y, w, h))
        for m in self.recv_all(wait):
            if m["ident"] == "TMP" and len(m["data"]) == 14:
                return m["data"].decode(errors="replace")
        return None

    def ate(self, on):
        self.send("M", "w", "ATE", "01" if on else "00")

    def feed_gps_text(self, lon, lat, alt_m, heading):
        """GPS'i metin biciminde ver (LON/LAT/ALT/AZI, NMEA $GPGGA/$GPRMC bicimi) - ikili GPS'e alternatif"""
        def dm(v, deg_digits):
            a = abs(v)
            d = int(a)
            return "%0*d%07.4f" % (deg_digits, d, (a - d) * 60)
        self.send("D", "w", "LON", ("E" if lon >= 0 else "W") + dm(lon, 3))
        self.send("D", "w", "LAT", ("N" if lat >= 0 else "S") + dm(lat, 2))
        self.send("D", "w", "ALT", "%.1f" % alt_m)
        self.send("D", "w", "AZI", "%05.1f" % (heading % 360))


    def raw(self, text):
        """'#TPP' sonrasi metni gonder, CRC otomatik. Orn: D2wMNU01"""
        body = ("#TPP" + text).encode()
        self.s.sendto(body + crc(body), self.addr)


# ------------------------------------------------------------ kendi kendine test (cihaz gerekmez)
def selftest():
    """Belgedeki ornek paketleri ve cihazdan kaydedilmis gercek paketleri dogrular."""
    ok = True

    def chk(name, got, exp):
        nonlocal ok
        good = got == exp
        ok &= good
        print(("OK    " if good else "HATA  ") + name + ("" if good else f"  beklenen={exp!r} gelen={got!r}"))

    # belge ornekleri
    chk("PTZ dur", build("G", "w", "PTZ", "00"), b"#TPPG2wPTZ0065")
    chk("REC", build("D", "w", "REC", "0A", src="U"), b"#TPUD2wREC0A54")
    chk("GSY", build("G", "w", "GSY", "E2", src="U"), b"#TPUG2wGSYE276")
    chk("GAY", build("G", "w", "GAY", "EF0732", src="U"), b"#tpUG6wGAYEF073288")
    chk("GIY", build("G", "w", "GIY", "EF0732", src="U"), b"#tpUG6wGIYEF073290")
    chk("GIC oku", build("G", "r", "GIC", "00", src="U"), b"#TPUG2rGIC003A")
    chk("GAC oku", build("G", "r", "GAC", "00", src="U"), b"#TPUG2rGAC0032")
    chk("ZFP", build("M", "w", "ZFP", "FFB40032", src="U"), b"#tpUM8wZFPFFB400320F")
    chk("MUL", build("M", "w", "MUL", "0123", src="U"), b"#tpUM4wMUL012308")
    chk("DZM", build("D", "w", "DZM", "20F140"), b"#tpPD6wDZM20F14070")
    gps = bytes.fromhex("23 74 70 55 44 45 77 47 50 53 0E 01 18 62 A5 47 80 AC FD 11 F4 01 00 00 45 41")
    chk("GPS ikili", build("D", "w", "GPS", gps[10:24], src="U"), gps)
    uav = bytes.fromhex("23 74 70 50 44 41 77 55 41 56 02 D1 F0 05 F8 F3 BA 22 10 27 30 35")
    chk("UAV ikili", build("D", "w", "UAV", uav[10:20]), uav)
    chk("aci -50", ang4(-50), "EC78")
    chk("aci -43.45", ang4(-43.45), "EF07")
    # cihazdan kaydedilmis gercek paketler
    m = parse(b"#tpGPCrGACF3ADFCD8FEAC30")
    chk("gercek GAC", (m["ident"], m["crc_ok"], decode_att(m["data"])), ("GAC", True, dict(yaw=-31.55, pitch=-8.08, roll=-3.4)))
    m = parse(b"#tpGPCrGIAF3B6FC170000C6")
    chk("gercek gyro (GIA)", (m["ident"], m["crc_ok"], decode_att(m["data"])), ("GIA", True, dict(yaw=-31.46, pitch=-10.01, roll=0.0)))
    m = parse(b"#tPDP16TMP13C7500FC004BE00E700F29E")
    chk("gercek sicaklik", (m["ident"], m["crc_ok"], decode_tmp(m["data"])),
        ("TMP", True, dict(max_c=25.2, max_xy=(316, 117), min_c=23.1, min_xy=(4, 190), center_c=24.2)))
    m = parse(b"#tpMP3rZMP142D7")
    chk("gercek ZMP", (m["ident"], m["data"], m["crc_ok"]), ("ZMP", b"142", True))
    m = parse(b"#tpMP4rZOMFF0F42")
    chk("gercek ZOM", (m["ident"], s16(m["data"].decode()), m["crc_ok"]), ("ZOM", -241, True))
    # girdi dogrulama
    for name, fn in (("hiz>99", lambda: s8(120)), ("bos mod", lambda: _check_mode("x"))):
        try:
            fn(); chk("reddet: " + name, False, True)
        except ValueError:
            chk("reddet: " + name, True, True)
    print("TUMU GECTI" if ok else "HATA VAR")
    return ok


# ------------------------------------------------------------ terminal araci
def out(o, js):
    print(json.dumps(o, ensure_ascii=False) if js else
          " ".join(f"{k}={v}" for k, v in o.items()) if isinstance(o, dict) else o)


def main():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--json", action="store_true", default=argparse.SUPPRESS, help="ciktiyi JSON ver")
    common.add_argument("-v", "--verbose", action="store_true", default=argparse.SUPPRESS,
                        help="giden/gelen ham paketleri goster")
    common.add_argument("--force", action="store_true", default=argparse.SUPPRESS,
                        help="aci sinir kontrolunu atla")
    ap = argparse.ArgumentParser(description=__doc__, parents=[common],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)

    def add(name, h, *args, **kw):
        p = sp.add_parser(name, help=h, parents=[common])
        for a in args:
            p.add_argument(a[0], **a[1])
        return p

    F = lambda **k: dict(type=float, **k)
    I = lambda **k: dict(type=int, **k)
    MODE = ("--mode", dict(choices=["gyro", "enc"], default=DEFAULT_MODE))
    FINE = ("--fine-speed", I(default=None, help="duzeltme hareketlerinin hizi (der/sn)"))
    add("selftest", "cihaz gerektirmeyen kendi kendine test (paket bicimleri, gercek paket ornekleri)")
    add("attitude", "gyro (uzaya gore) ve enkoder (govdeye gore) acilarini oku")
    add("temps", "termal max/min/merkez sicaklik (C)")
    add("watch", "aktif attitude yayinini JSON satirlari olarak akit", ("sec", F(nargs="?", default=10)))
    add("stop", "gimbal hareketini durdur"); add("home", "eve don")
    add("down90", "tek tusla asagi bak")
    add("goto", "konuma git (yaw pitch); yaw + saga, pitch + asagi", ("yaw", F()), ("pitch", F()),
        ("--speed", I(default=30)), MODE, ("--wait", dict(action="store_true", help="varana kadar bekle ve duzelt")),
        ("--tol", F(default=0.2)), FINE)
    add("yaw", "sadece yaw'a git", ("deg", F()), ("--speed", I(default=30)), MODE)
    add("pitch", "sadece pitch'e git", ("deg", F()), ("--speed", I(default=30)), MODE)
    add("rate", "hiz modu (der/sn) belirli sure", ("yaw", I()), ("pitch", I()), ("sec", F(nargs="?", default=1)))
    add("motor", "gimbal motoru", ("state", dict(choices=["on", "off"])))
    add("zoom", "zoom", ("dir", dict(choices=["in", "out"])), ("sec", F(nargs="?", default=1)))
    add("focus", "odak", ("dir", dict(choices=["near", "far", "auto", "manual"])), ("sec", F(nargs="?", default=1)))
    add("zoompos", "zoom konumunu oku"); add("mag", "optik buyutme oku/ayarla", ("x", F(nargs="?")))
    add("dzoom", "dijital zoom: dzoom 0 2.0 | dzoom 0 up/down/hold-up/hold-down/stop | dzoom read", ("cam", {}), ("x", dict(nargs="?", default=None)))
    add("pip", "goruntu modu", ("v", dict(nargs="?", default="0A")))
    add("palette", "termal palet", ("v", dict(nargs="?", default="0A")))
    add("day", "gunduz"); add("night", "gece/termal")
    add("rec", "kayit", ("v", dict(choices=["on", "off", "toggle"], nargs="?", default="toggle")))
    add("snap", "fotograf (1 gorunur+termal,2 gorunur,3 termal,5 +sicaklik)", ("kind", I(nargs="?", default=1)))
    add("rotate", "goruntu cevir", ("deg", dict(choices=["0", "180"])))
    add("res", "cozunurluk (0 kayit,1 foto,2 rtsp / 0 nativ,1 1080,2 720,3 480)", ("target", I()), ("res", I()))
    add("bitrate", "stream=0 bitrate: bitrate 2000 (kbps) | bitrate 3 --mbps (1..8 Mbps) | bitrate read", ("value", {}),
        ("--mbps", dict(action="store_true", help="deger Mbps (1..8)")))
    add("codec", "kodlayici", ("stream", I()), ("codec", dict(choices=["h264", "h265"])))
    add("sd", "hafiza karti kapasitesi")
    add("lrf", "lazer mesafe olc", ("mode", dict(choices=["single", "cont", "stop", "enable"], nargs="?", default="single")),
        ("--wait", F(default=3.0)))
    add("gps", "ucus kontrolcu GPS verisi gonder", ("heading", F()), ("lon", F()), ("lat", F()), ("alt", F()))
    add("uav", "ucak attitude verisi gonder", ("yaw", F()), ("pitch", F()), ("roll", F()), ("speed", F()), ("track", F()))
    add("synctime", "cihaz saatini UTC ile esitle", ("--beijing", dict(action="store_true", help="TIM (yerel saat, YYYYMMDDHHMMSS) bicimi")))
    add("ymode", "gimbal yaw modu: lock (yon kilidi) / follow (govdeyi takip) / toggle", ("mode", dict(choices=["lock", "follow", "toggle"])))
    add("calibrate", "gimbal kalibrasyonu (PTZ 09) - gimbal hareketsiz olmali", ("--yes", dict(action="store_true", help="onay")))
    add("info", "cihazdan tum okunabilen durum/ayarlari oku")
    add("read", "genel okuma: read D ROT", ("dst", dict(choices=list("MDEG"))), ("ident", {}), ("data", dict(nargs="?", default="00")))
    add("focuspos", "odak konumunu oku"); add("zfp", "zoom/odak konumu ayarla (N = degistirme)", ("zoom", {}), ("focus", dict(nargs="?", default="N")))
    add("aperture", "diyafram (kismi modeller)", ("dir", dict(choices=["stop", "less", "more"])))
    add("fog", "elektronik sis giderme", ("mode", dict(choices=["off", "auto", "manual"])), ("intensity", I(nargs="?", default=0)))
    add("trange", "termal olcum araligi: 1 = -20..150C, 2 = -20..550C", ("n", dict(choices=["1", "2"])))
    add("tpoint", "nokta/bolge sicakligi (kismi modeller)", ("x", I()), ("y", I()), ("w", I(nargs="?", default=0)), ("h", I(nargs="?", default=0)))
    add("gpstext", "GPS'i metin biciminde ver (LON/LAT/ALT/AZI)", ("lon", F()), ("lat", F()), ("alt", F()), ("heading", F()))
    add("bench", "konumlandirma dogrulugu testi: n konuma gider, hata istatistigi verir (gimbal HAREKET EDER)",
        ("--n", I(default=6)), ("--tol", F(default=0.2)), ("--speed", I(default=30)), FINE, MODE)
    add("raw", "ham komut (#TPP sonrasi, CRC otomatik)", ("text", {}))
    a = ap.parse_args()

    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    try:
        g = KHT30B()
    except OSError as e:
        sys.exit(f"hata: {e}")
    g.verbose, g.force = getattr(a, "verbose", False), getattr(a, "force", False)
    c, js = a.cmd, getattr(a, "json", False)
    try:
        if c == "attitude":
            out(dict(gyro=g.attitude("gyro"), enc=g.attitude("enc")), js)
        elif c == "temps":
            out(g.temps() or dict(temps=None), js)
        elif c == "watch":
            g.stream(True)
            end = time.time() + a.sec
            try:
                while time.time() < end:
                    for m in g.recv_all(0.2):
                        if m["ident"] in ("GIA", "GIC", "GAC") and len(m["data"]) >= 12:
                            row = dict(t=round(time.time(), 3), kind="enc" if m["ident"] == "GAC" else "gyro")
                            print(json.dumps({**row, **decode_att(m["data"])}), flush=True)
                        elif g.verbose:
                            print(json.dumps(dict(ident=m["ident"], data=m["data"].decode(errors="replace"))), flush=True)
            except BrokenPipeError:  # cikti baska programa (head vb.) yonlendirilip kapandi
                pass
            finally:
                g.stream(False)
        elif c == "stop": g.stop()
        elif c == "home": g.home()
        elif c == "down90": g.ptz(0x0A)
        elif c == "goto":
            if a.wait:
                out(g.goto_wait(a.yaw, a.pitch, a.speed, a.mode, a.tol, fine_speed=a.fine_speed), js)
            else:
                g.goto(a.yaw, a.pitch, speed=a.speed, mode=a.mode)
        elif c == "yaw": g.goto(yaw=a.deg, speed=a.speed, mode=a.mode)
        elif c == "pitch": g.goto(pitch=a.deg, speed=a.speed, mode=a.mode)
        elif c == "rate":
            try:
                g.rate(a.yaw, a.pitch); time.sleep(a.sec)
            finally:
                g.stop()
        elif c == "motor": g.motor(a.state == "on")
        elif c in ("zoom", "focus"):
            start = {"in": 2, "out": 1} if c == "zoom" else {"near": 1, "far": 2}
            fn = g.zoom if c == "zoom" else g.focus
            if a.dir in ("auto", "manual"):
                fn(0x10 if a.dir == "auto" else 0x11)
            else:
                try:
                    fn(start[a.dir]); time.sleep(a.sec)
                finally:
                    fn(0)
        elif c == "zoompos": out(dict(zoom_pos=g.zoom_pos()), js)
        elif c == "mag":
            if a.x is None: out(dict(mag=g.get_mag()), js)
            else: g.set_mag(a.x)
        elif c == "dzoom":
            steps = {"up": "0A", "down": "0B", "hold-up": "0C", "hold-down": "0D", "stop": "0E"}
            if a.cam == "read": out(g.dzoom_get() or dict(dzoom=None), js)
            elif a.x in steps: g.dzoom_step(int(a.cam), steps[a.x])
            elif a.x is None: sys.exit("hata: dzoom <cam> <x | up|down|hold-up|hold-down|stop>  ya da  dzoom read")
            else: g.dzoom(int(a.cam), float(a.x))
        elif c == "pip": g.pip(a.v)
        elif c == "palette": g.palette(a.v)
        elif c == "day": g.irc("00")
        elif c == "night": g.irc("01")
        elif c == "rec": out(g.record({"on": "01", "off": "00", "toggle": "0A"}[a.v]), js)
        elif c == "snap": out(g.snap(a.kind), js)
        elif c == "rotate": g.rotate(int(a.deg))
        elif c == "res": g.resolution(a.target, a.res)
        elif c == "bitrate":
            if a.value == "read": out(dict(bitrate=g.read("D", "BIT")), js)
            elif a.mbps: g.bitrate_mbps(int(a.value))
            else: g.bitrate_kbps(int(a.value))
        elif c == "codec": g.encoder(a.stream, a.codec == "h265")
        elif c == "sd": out(g.sd(), js)
        elif c == "lrf": out(g.lrf(a.mode, a.wait), js)
        elif c == "gps": g.feed_gps(a.heading, a.lon, a.lat, a.alt)
        elif c == "uav": g.feed_uav(a.yaw, a.pitch, a.roll, a.speed, a.track)
        elif c == "synctime": g.sync_time(a.beijing)
        elif c == "ymode": g.yaw_mode(a.mode)
        elif c == "calibrate":
            if not a.yes: sys.exit("hata: kalibrasyon gimbali hareket ettirebilir; emin iseniz --yes ekleyin")
            g.calibrate()
        elif c == "info":
            q = dict(vsn_D=("D", "VSN"), vsn_M=("M", "VSN"), serial_G=("G", "VER"), rot=("D", "ROT"), vid=("D", "VID"),
                     bit=("D", "BIT"), pip=("D", "PIP"), palette=("D", "IMG"), dzm=("D", "DZM"), fog=("D", "DFG"),
                     mul=("M", "MUL"), zom=("M", "ZOM"), foc=("M", "FOC"), motor=("G", "GMS"), gaa=("G", "GAA"),
                     gia=("G", "GIA"), rec=("D", "REC"))
            res = {k: g.read(d, i, timeout=0.8) for k, (d, i) in q.items()}
            res["sd"] = g.sd()
            out(res, js)
        elif c == "read": out(dict(data=g.read(a.dst, a.ident, a.data)), js)
        elif c == "focuspos": out(dict(focus_pos=g.focus_pos()), js)
        elif c == "zfp": g.set_zoom_focus(None if a.zoom == "N" else int(a.zoom), None if a.focus == "N" else int(a.focus))
        elif c == "aperture": g.aperture({"stop": 0, "less": 1, "more": 2}[a.dir])
        elif c == "fog": g.fog({"off": 0, "auto": 1, "manual": 2}[a.mode], a.intensity)
        elif c == "trange": g.temp_range(a.n == "2")
        elif c == "tpoint": out(dict(raw=g.temp_region(a.x, a.y, a.w, a.h)), js)
        elif c == "gpstext": g.feed_gps_text(a.lon, a.lat, a.alt, a.heading)
        elif c == "bench":
            targets = [(10, 10), (-15, 25), (20, 0), (-30, 15), (0, 5), (25, 30)]
            rows = []
            g.home(); time.sleep(3)
            for i in range(a.n):
                y, p = targets[i % len(targets)]
                t0 = time.time()
                r = g.goto_wait(y, p, a.speed, a.mode, a.tol, fine_speed=a.fine_speed)
                dt = time.time() - t0
                time.sleep(1.0)                      # tam otursun, son hatayi bundan olc
                f = g.attitude(a.mode) or {}
                row = dict(target=(y, p), yaw_err=round(f.get("yaw", float("nan")) - y, 2),
                           pitch_err=round(f.get("pitch", float("nan")) - p, 2),
                           corrections=r["corrections"], reached=r["reached"], sec=round(dt, 1))
                rows.append(row); print(json.dumps(row), flush=True)
            g.home()
            ye = [abs(r["yaw_err"]) for r in rows if r["yaw_err"] == r["yaw_err"]]
            pe = [abs(r["pitch_err"]) for r in rows if r["pitch_err"] == r["pitch_err"]]
            out(dict(mode=a.mode, tol=a.tol, n=len(rows),
                     yaw_mean=round(sum(ye) / len(ye), 2) if ye else None, yaw_max=max(ye, default=None),
                     pitch_mean=round(sum(pe) / len(pe), 2) if pe else None, pitch_max=max(pe, default=None),
                     reached=sum(r["reached"] for r in rows), avg_sec=round(sum(r["sec"] for r in rows) / len(rows), 1)), js)
        elif c == "raw": g.raw(a.text)
    except KeyboardInterrupt:
        g.stop()
    except ValueError as e:
        sys.exit(f"hata: {e}")
    finally:
        g.close()


if __name__ == "__main__":
    main()
