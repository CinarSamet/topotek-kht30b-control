import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
#!/usr/bin/env python3
"""KHT30B - denenmemis komutlarin cihaz testi.  Kullanim (depo kokunden: python tools/test_unverified.py):

    python test_unverified.py

Hazirlik: GroundStation kapali, VLC'de goruntu acik (rtsp://192.168.144.108:554/stream=0),
gimbalin onu bos, kamera bir cisme (>2 m) bakiyor. Her test: komut gonderilir, mumkunse cihazdan geri
okunarak otomatik dogrulanir, gorsel olanlarda size sorulur. Sonunda test_report.txt yazilir; icerigini paylasin.
Test sonunda cihaz baslangic durumuna geri alinir (zoom, dijital zoom, gece/gunduz, motor, cevirme, cozunurluk...).
"""
import json, sys, time
from kht30b import KHT30B

g = KHT30B()
results = []


def user(q):
    while True:
        a = input(f"   {q}  [e=evet / h=hayir / a=atla] > ").strip().lower()
        if a in ("e", "h", "a"):
            return {"e": "evet", "h": "hayir", "a": "atlandi"}[a]


def run(name, action, verdict=None, ask=None, after=None, confirm=None):
    print(f"\n=== {name}")
    r = {"test": name}
    if confirm:
        if input(f"   {confirm}  [Enter=calistir / a=atla] > ").strip().lower() == "a":
            r["sonuc"] = "atlandi"
            results.append(r)
            return
    try:
        out = action()
        r["cikti"] = out
        if verdict:
            r["otomatik"] = verdict(out)
    except Exception as e:  # bir test digerlerini durdurmasin
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


def rd_g():
    m = g.ask("G", "GMS", "00", timeout=1.2)
    return m["data"].decode(errors="replace") if m else None


def rd(ident, data="00"):  # D adresli okuma, yanit verisi (str) ya da None
    m = g.ask("D", ident, data, timeout=1.2)
    return m["data"].decode(errors="replace") if m else None


# ---------------------------------------------------------------- hazirlik
print(__doc__)
input("Hazirsaniz Enter'a basin (iptal: Ctrl+C) ")
g.home(); time.sleep(3)
start = dict(att=g.attitude("gyro"), mag=g.get_mag(), rot=rd("ROT"), vid=rd("VID", "20"), ent=rd("ENT", "00"),
             bit=rd("BIT"))
print("baslangic durumu:", start)
if not start["att"]:
    sys.exit("HATA: gimbal (G) yanit vermiyor (gyro okunamadi). Gimbalin gucunu kapatip acin, GroundStation kapali olsun, sonra tekrar deneyin.")

# ---------------------------------------------------------------- zoom
def t_zoom():
    m0 = g.get_mag()
    g.zoom(2); time.sleep(1.5); g.zoom(0); time.sleep(0.6); m1 = g.get_mag()
    g.zoom(1); time.sleep(1.5); g.zoom(0); time.sleep(0.6); m2 = g.get_mag()
    return dict(basta=m0, yakinlasinca=m1, uzaklasinca=m2)


run("zoom in / out", t_zoom,
    verdict=lambda o: "CALISTI (buyutme degisti)" if None not in o.values() and len({o["basta"], o["yakinlasinca"], o["uzaklasinca"]}) > 1
    else "buyutme degismedi/okunamadi (gorsel soruya bakin)",
    ask="Zoom goruntude yakinlasip uzaklasti mi?")

# ---------------------------------------------------------------- odak
def t_focus():
    g.focus(1); time.sleep(1.0); g.focus(0); time.sleep(0.5)
    g.focus(2); time.sleep(1.0); g.focus(0); time.sleep(0.5)
    g.focus(0x10)
    return "gonderildi (oda yakin, uzak, oto)"


run("focus yakin / uzak / oto", t_focus, ask="Odak (netlik) degistiginde goruntude fark ettiniz mi?")


# ---------------------------------------------------------------- hiz modu
def t_rate():
    a0 = g.attitude("gyro")
    if not a0:
        raise RuntimeError("gyro okunamadi, gimbal yanit vermiyor")
    g.rate(8, 0); time.sleep(1.5); g.stop(); time.sleep(0.8); a1 = g.attitude("gyro")
    g.rate(0, 8); time.sleep(1.0); g.stop(); time.sleep(0.8); a2 = g.attitude("gyro")
    g.goto_wait(a0["yaw"], a0["pitch"])  # geri don
    return dict(yaw_farki=round(a1["yaw"] - a0["yaw"], 2), pitch_farki=round(a2["pitch"] - a1["pitch"], 2))


run("rate (hiz modu): sag 8 dg/s 1.5 sn, asagi 8 dg/s 1 sn", t_rate,
    verdict=lambda o: ("CALISTI" if o["yaw_farki"] > 5 and o["pitch_farki"] > 3 else "BEKLENEN HAREKET OLMADI"),
    ask="Gimbal once saga sonra asagi dondu mu?")

# ---------------------------------------------------------------- dijital zoom
run("dzoom: ana kamera 2.0x", lambda: (g.dzoom(0, 2.0), time.sleep(1.5), "gonderildi")[2],
    ask="Goruntu 2x dijital yakinlasti mi?", after=lambda: (g.dzoom(0, 1.0), time.sleep(1), "1.0x'e donuldu")[2])


# ---------------------------------------------------------------- optik buyutme ayari
def t_mag():
    prev = g.get_mag()
    g.set_mag(5.0); time.sleep(2.5)
    return dict(onceki=prev, istenen=5.0, sonra=g.get_mag())


run("mag 5.0 (optik buyutme ayari; kismi modellerde var)", t_mag,
    verdict=lambda o: "CALISTI" if o["sonra"] and abs(o["sonra"] - 5.0) < 0.6 else "bu cihazda calismadi/okunamadi",
    after=lambda: (g.set_mag(start["mag"]) if start["mag"] else None, "eski buyutmeye donuldu")[1])

# ---------------------------------------------------------------- gece / gunduz
run("night", lambda: (g.irc("01"), time.sleep(2.5), "gonderildi")[2], ask="Gece moduna gecti mi (goruntu degisti mi)?")
run("day", lambda: (g.irc("00"), time.sleep(2.5), "gonderildi")[2], ask="Gunduz moduna dondu mu?")


# ---------------------------------------------------------------- goruntu cevirme
def t_rotate():
    cur180 = start["rot"] == "02"
    g.rotate(0 if cur180 else 180); time.sleep(2)
    return dict(onceki=start["rot"], okunan=rd("ROT"))


run("rotate (goruntuyu ters cevir)", t_rotate,
    verdict=lambda o: "CALISTI (okuma degisti)" if o["okunan"] and o["okunan"] != o["onceki"] else "okuma degismedi",
    ask="Goruntu ters dondu mu?",
    after=lambda: (g.rotate(180 if start["rot"] == "02" else 0), time.sleep(2), "geri alindi, okunan=" + str(rd("ROT")))[2])

# ---------------------------------------------------------------- motor (gimbal gevser!)
def t_motor():
    g.motor(False); time.sleep(1.0)  # GMS okumasi desteklenmiyor; yalnizca gorsel/elle kontrol
    return "gonderildi (wGMS00)"


run("motor off/on", t_motor,
    confirm="DIKKAT: gimbal motoru kapanir, kamera sarkabilir. Kamerayi hafifce tutun",
    ask="Gimbal gevsedi mi (elle hareket ediyor mu)?",
    after=lambda: (g.motor(True), time.sleep(2), "acildi; okunan=" + str(rd_g()))[2])


# ---------------------------------------------------------------- cozunurluk (RTSP)
def t_res():
    g.resolution(2, 2); time.sleep(1.5)
    return dict(onceki=start["vid"], okunan=rd("VID", "20"))


run("res 2 2 (RTSP 720p)", t_res, confirm="RTSP akisi kesilebilir; VLC'yi yeniden baglamaniz gerekebilir",
    verdict=lambda o: "CALISTI" if o["okunan"] == "22" else "okuma beklenenden farkli",
    after=lambda: (g.resolution(2, int(start["vid"][1]) if start["vid"] and start["vid"][1:2].isdigit() else 1),
                   time.sleep(1.5), "geri alindi, okunan=" + str(rd("VID", "20")))[2])

# ---------------------------------------------------------------- bitrate
def t_bit():
    g.bitrate_kbps(2000); time.sleep(1)
    return dict(onceki=start["bit"], okunan=rd("BIT"))


run("bitrate 2000 kbps", t_bit, verdict=lambda o: f"okunan={o['okunan']} (komut gonderildi; uygulama ayari 2M)")

# ---------------------------------------------------------------- kodlayici
def t_codec():
    h265_now = start["ent"] == "01"
    g.encoder(0, not h265_now); time.sleep(1.5)
    return dict(onceki=start["ent"], okunan=rd("ENT", "00"))


run("codec (stream 0 kodlayiciyi degistir)", t_codec,
    confirm="Video akisi kesilebilir (bitince eski koda doner)",
    verdict=lambda o: "CALISTI (okuma degisti)" if o["okunan"] and o["okunan"] != o["onceki"] else "okuma degismedi",
    after=lambda: (g.encoder(0, start["ent"] == "01"), time.sleep(1.5), "geri alindi, okunan=" + str(rd("ENT", "00")))[2])


# ---------------------------------------------------------------- gps + uav -> lazerle hedef koordinati
def t_gps():
    for _ in range(5):
        g.feed_gps(90.0, 29.0, 41.0, 100.0)
        g.feed_uav(90.0, 0.0, 0.0, 0.0, 90.0)
        time.sleep(0.2)
    return g.lrf("single", 3.5)


def v_gps(o):
    try:
        if o.get("distance_m") is None:
            return "lazer olcumu alinamadi; bir cisme (2+ m) cevirip tekrar deneyin"
        lon, lat = float(o["lon"][1:]), float(o["lat"][1:])
        return "CALISTI (hedef koordinati GPS'ten hesaplandi)" if abs(lon - 29) < 0.5 and abs(lat - 41) < 0.5 \
            else f"koordinat geldi ama beklenen yakinda degil ({o['lon']}, {o['lat']})"
    except Exception:
        return f"koordinat eksik/bicimi farkli: {o}"


run("gps + uav verisi, ardindan lazer (hedef koordinati 29E, 41N civarinda cikmali)", t_gps, verdict=v_gps)

# ---------------------------------------------------------------- saat esitleme
run("synctime", lambda: (g.sync_time(), "gonderildi (otomatik dogrulama yok)")[1])

# ---------------------------------------------------------------- toparlama
for fn in (g.stop, lambda: g.zoom(0), lambda: g.focus(0), lambda: g.motor(True), lambda: g.irc("00"), g.home):
    try:
        fn()
    except Exception:
        pass
try:
    g.s.close()
except Exception:
    pass

# ---------------------------------------------------------------- rapor
lines = ["KHT30B denenmemis komutlar - test raporu", ""]
for i, r in enumerate(results, 1):
    lines.append(f"{i}. {r['test']}")
    for k, v in r.items():
        if k != "test":
            lines.append(f"     {k}: {json.dumps(v, ensure_ascii=False)}")
text = "\n".join(lines)
open("test_report.txt", "w", encoding="utf-8").write(text + "\n")
print("\n" + "=" * 60 + "\n" + text + "\n" + "=" * 60)
print("Rapor test_report.txt dosyasina yazildi. Yukaridaki metni ya da dosyayi paylasin.")
