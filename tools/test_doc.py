import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
#!/usr/bin/env python3
"""KHT30B - belgeden (UDP&UART V1.1.3) eklenen / yeniden denenen komutlarin cihaz testi.
Kullanim (depo kokunden):   python tools/test_doc.py

Hazirlik: GroundStation kapali, VLC acik (rtsp://192.168.144.108:554/stream=0), gimbalin onu bos, kamera
2 m'den uzak bir cisme bakiyor. Gorsel testlerde size e/h/a sorulur. Sonunda test_doc_report.txt yazilir;
icerigini paylasin. Her test sonunda cihaz eski haline alinir.
"""
import json, sys, time
from kht30b import KHT30B

g = KHT30B()
results = []
NOISE = ("LRF", "LON", "LAT", "TMP", "GAC", "GIA", "GIC")  # bosta surekli gelenler


def user(q):
    while True:
        a = input(f"   {q}  [e=evet / h=hayir / a=atla] > ").strip().lower()
        if a in ("e", "h", "a"):
            return {"e": "evet", "h": "hayir", "a": "atlandi"}[a]


def run(name, action, ask=None, after=None, confirm=None):
    print(f"\n=== {name}")
    r = {"test": name}
    if confirm and input(f"   {confirm}  [Enter=calistir / a=atla] > ").strip().lower() == "a":
        r["sonuc"] = "atlandi"
        results.append(r)
        return
    try:
        r["cikti"] = action()
    except Exception as e:
        r["hata"] = f"{type(e).__name__}: {e}"
    if ask:
        r["kullanici"] = user(ask)
    if after:
        try:
            r["geri_alma"] = after()
        except Exception as e:
            r["geri_alma"] = f"HATA: {e}"
    results.append(r)
    print("   ->", {k: v for k, v in r.items() if k != "test"})


def sniff(secs):
    """secs boyunca gelen, bosta-yayin disindaki paketlerin ident/veri ozeti"""
    seen = {}
    for m in g.recv_all(secs):
        if m["ident"] not in NOISE:
            seen[m["ident"]] = m["data"].decode(errors="replace")
    return seen


def snapshot():
    return dict(zom=g.zoom_pos(), zmp=g.get_mag(), mul=g.get_mul(), foc=g.focus_pos(), dzm=g.dzoom_get())


# ------------------------------------------------------------------ hazirlik
print(__doc__)
input("Hazirsaniz Enter'a basin (iptal: Ctrl+C) ")
g.home(); time.sleep(3)
att = g.attitude("gyro")
if not att:
    sys.exit("HATA: gimbal (G) yanit vermiyor. Gucu kapatip acin, GroundStation kapali olsun, ping 192.168.144.108 deneyin.")
start = snapshot()
start["dzm_raw"] = g.read("D", "DZM")
print("baslangic:", start)

# ------------------------------------------------------------------ 1) tum okumalar
QUERIES = dict(vsn_D=("D", "VSN"), vsn_M=("M", "VSN"), serial_G=("G", "VER"), rot=("D", "ROT"), vid=("D", "VID"),
               bit=("D", "BIT"), pip=("D", "PIP"), palette=("D", "IMG"), dzm=("D", "DZM"), fog=("D", "DFG"),
               mul=("M", "MUL"), zom=("M", "ZOM"), zmp=("M", "ZMP"), foc=("M", "FOC"), motor=("G", "GMS"),
               gaa=("G", "GAA"), gia=("G", "GIA"), rec=("D", "REC"), ate=("M", "ATE"), swh=("M", "SWH"))
run("tum okuma komutlari", lambda: {k: g.read(d, i, timeout=0.8) for k, (d, i) in QUERIES.items()})

# ------------------------------------------------------------------ 2) maksimum optik zoom
def t_zoom_max():
    rows = [("basta", snapshot())]
    g.drain()
    g.zoom(2)
    pushed = {}
    for i in range(6):  # 6 x 2 sn = 12 sn yakinlas, arada ZMP/ZOM yayinlarini yakala
        for m in g.recv_all(2.0):
            if m["ident"] in ("ZMP", "ZOM", "MUL"):
                pushed[m["ident"]] = m["data"].decode(errors="replace")
        rows.append((f"{(i + 1) * 2}sn", dict(zom=g.zoom_pos(), zmp=g.get_mag(), mul=g.get_mul())))
    g.zoom(0)
    time.sleep(0.5)
    rows.append(("son", snapshot()))
    return dict(adimlar=rows, yayinlanan=pushed)


run("zoom max (12 sn yakinlas)", t_zoom_max, ask="Goruntu en yakina (maks zoom) ulasti mi? Net/odakli mi?",
    after=lambda: (g.zoom(1), time.sleep(12), g.zoom(0), time.sleep(0.5), snapshot())[4])

# ------------------------------------------------------------------ 3) dijital zoom (belgedeki tum bicimler)
def t_dzoom():
    out = {}
    def step(label, fn, wait=1.5):
        g.drain(); fn(); time.sleep(wait)
        out[label] = dict(dzm=g.read("D", "DZM"), yayin=sniff(0.3))
    step("0A x4 (adim yakinlas, 0.5x/adim)", lambda: [(g.dzoom_step(0, "0A"), time.sleep(0.4)) for _ in range(4)])
    step("0B x4 (adim uzaklas)", lambda: [(g.dzoom_step(0, "0B"), time.sleep(0.4)) for _ in range(4)])
    step("0C basili tut 1.5sn + 0E dur", lambda: (g.dzoom_step(0, "0C"), time.sleep(1.5), g.dzoom_step(0, "0E")))
    step("0D basili tut 1.5sn + 0E dur", lambda: (g.dzoom_step(0, "0D"), time.sleep(1.5), g.dzoom_step(0, "0E")))
    step("dogrudan '020' (ondalik 2.0x)", lambda: g.send("D", "w", "DZM", "020"))
    step("dogrudan '014' (hex 2.0x)", lambda: g.send("D", "w", "DZM", "014"))
    step("0F: '00F020' (2.0x, belge bicimi)", lambda: g.dzoom(0, 2.0))
    step("0F: '00F100' (10.0x)", lambda: g.dzoom(0, 10.0))
    return out


run("dzoom: tum biciler (DZM okumasi degisiyor mu?)", t_dzoom, ask="Bu sirada goruntu dijital olarak yakinlasip uzaklasti mi?",
    after=lambda: (g.dzoom(0, 1.0), g.send("D", "w", "DZM", "010"), time.sleep(1.5), g.read("D", "DZM"))[3])

# ------------------------------------------------------------------ 4) odak
def t_focus():
    f0 = g.focus_pos()
    g.focus(1); time.sleep(1.2); g.focus(0); time.sleep(0.4); f1 = g.focus_pos()
    g.focus(2); time.sleep(1.2); g.focus(0); time.sleep(0.4); f2 = g.focus_pos()
    g.focus(0x10); time.sleep(1.5); f3 = g.focus_pos()
    return dict(basta=f0, focus_arti=f1, focus_eksi=f2, oto=f3)


run("focus (FOC konum okumasi ile)", t_focus, ask="Odak/netlik degisimini gordunuz mu?")

# ------------------------------------------------------------------ 5) yaw modu (drone icin onemli)
def t_ymode(mode):
    def f():
        g.yaw_mode(mode); time.sleep(1.0)
        return f"{mode} gonderildi; yaw={ (g.attitude('gyro') or {}).get('yaw') }"
    return f


run("yaw modu LOCK (yon kilidi)", t_ymode("lock"),
    ask="Gimbal govdesini (tabani) elinizle yavasca saga cevirin: kamera ayni dunya yonune bakmaya devam etti mi?",
    confirm="Gimbal tabanini elle cevirebilecek misiniz?")
run("yaw modu FOLLOW (govdeyi takip)", t_ymode("follow"),
    ask="Simdi tabani cevirin: kamera tabanla birlikte dondu mu (govdeye gore sabit kaldi mi)?",
    confirm="Gimbal tabanini elle cevirebilecek misiniz?",
    after=lambda: (g.home(), time.sleep(3), "home")[2])

# ------------------------------------------------------------------ 6) GPS: ikili (surekli besleme) ve metin biciminde
def feed_and_lrf(feeder):
    end = time.time() + 3.0
    while time.time() < end:
        feeder(); g.feed_uav(90.0, 0.0, 0.0, 0.0, 90.0); time.sleep(0.2)
    g.drain()
    res = {}
    for _ in range(2):
        feeder()
        res = g.lrf("single", 3.0)
        if res.get("distance_m"):
            break
    return res


run("GPS ikili (surekli besleme) + lazer", lambda: feed_and_lrf(lambda: g.feed_gps(90.0, 29.0, 41.0, 100.0)))
run("GPS metin (LON/LAT/ALT/AZI) + lazer", lambda: feed_and_lrf(lambda: g.feed_gps_text(29.0, 41.0, 100.0, 90.0)))

# ------------------------------------------------------------------ 7) goruntu cevirme (VLC yeniden baglanarak)
def t_rot():
    cur = g.read("D", "ROT")
    g.rotate(0 if cur == "02" else 180); time.sleep(2)
    return dict(onceki=cur, okunan=g.read("D", "ROT"))


run("rotate (VLC'yi kapatip YENIDEN acin)", t_rot, ask="VLC'yi kapatip yeniden actiginizda goruntu ters dondu mu?",
    confirm="Test sirasinda VLC'yi kapatip acacaksiniz",
    after=lambda: (g.rotate(180 if g.read("D", "ROT") == "00" else 0), time.sleep(2), g.read("D", "ROT"))[2])

# ------------------------------------------------------------------ 8) bitrate (iki bicim)
def t_bit():
    out = dict(onceki=g.read("D", "BIT"))
    g.bitrate_kbps(2000); time.sleep(1); out["K02000_sonra"] = g.read("D", "BIT")
    g.bitrate_mbps(3); time.sleep(1); out["3Mbps_6hane_sonra"] = g.read("D", "BIT")
    g.send("D", "w", "BIT", "03"); time.sleep(1); out["03_2hane_sonra"] = g.read("D", "BIT")
    return out


run("bitrate (K02000 ve 3Mbps bicimi)", t_bit,
    after=lambda: (g.bitrate_mbps(4), time.sleep(1), g.read("D", "BIT"))[2])

# ------------------------------------------------------------------ 9) sis giderme, sicaklik araligi, nokta sicakligi
def t_fog():
    out = dict(onceki=g.read("D", "DFG"))
    g.fog(2, 50); time.sleep(1); out["manuel_50"] = g.read("D", "DFG")
    return out


run("elektronik sis giderme (DFG)", t_fog, after=lambda: (g.fog(0, 0), time.sleep(1), g.read("D", "DFG"))[2])
run("sicaklik araligi 2 (-20..550C)", lambda: (g.temp_range(True), time.sleep(1.5), g.temps())[2],
    after=lambda: (g.temp_range(False), time.sleep(1), g.temps())[2])
run("nokta sicakligi (merkez 320,256)", lambda: dict(raw=g.temp_region(320, 256, 0, 0)))

# ------------------------------------------------------------------ 10) saat (Beijing/TIM bicimi)
run("synctime TIM bicimi", lambda: (g.sync_time(beijing=True), "gonderildi (geri okuma yok)")[1])

# ------------------------------------------------------------------ toparlama + rapor
for fn in (g.stop, lambda: g.zoom(0), lambda: g.focus(0), lambda: g.motor(True), lambda: g.irc("00"), g.home):
    try:
        fn()
    except Exception:
        pass
try:
    g.s.close()
except Exception:
    pass

lines = ["KHT30B belge tabanli test raporu", "", f"baslangic: {json.dumps(start, ensure_ascii=False)}", ""]
for i, r in enumerate(results, 1):
    lines.append(f"{i}. {r['test']}")
    for k, v in r.items():
        if k != "test":
            lines.append(f"     {k}: {json.dumps(v, ensure_ascii=False)}")
text = "\n".join(lines)
open("test_doc_report.txt", "w", encoding="utf-8").write(text + "\n")
print("\n" + "=" * 60 + "\n" + text + "\n" + "=" * 60)
print("Rapor test_doc_report.txt dosyasina yazildi. Yukaridaki metni ya da dosyayi paylasin.")
