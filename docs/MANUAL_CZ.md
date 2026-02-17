# Uživatelská příručka — Terminál eMISTR (APT1220)

Tento dokument slouží jako podrobný návod k obsluze, konfiguraci a řešení problémů průmyslového terminálu **APT1220**.

---

## 1. Základní popis

Terminál **eMISTR** (APT1220) je zařízení postavené na platformě ESP32 (OLIMEX ESP32-POE), které slouží k evidenci docházky, výrobních operací a zakázek. Zařízení komunikuje s centrálním serverem pomocí sítě Ethernet (preferováno) nebo WiFi.

### Hlavní vlastnosti
- **Duální RFID čtečky**: Umožňuje připojení dvou čteček (označené jako C a D).
- **Online/Offline režim**: Při výpadku sítě se data ukládají do interní paměti a odešlou se automaticky po obnovení spojení.
- **Odolnost**: Hardware je navržen pro nepřetržitý provoz s automatickým denním restartem.
- **Displej**: 4řádkový LCD displej zobrazuje stav, čas a pokyny pro obsluhu.

---

## 2. Režimy provozu

### 2.1 Pracant (Výchozí)
Terminál slouží pro evidenci práce.
- **Displej (Line 0):** `eMISTR 2025 ETH` (nebo WIFI)
- **Funkce:** Čte čipy zaměstnanců, kódy operací a zakázek.

### 2.2 Dveře (Zámek)
Terminál ovládá dveřní zámek/turniket.
- **Displej (Line 0):** `eMISTR 2025 DVERE`
- **Funkce:** Po přiložení oprávněného čipu pošle signál k otevření dveří.

---

## 3. Ovládání a Signalizace

### Hlavní obrazovka
```
Line 0:  eMISTR 2025   ETH
Line 1: ----*---------------   (běžící hvězdička = systém žije)
Line 2: 17.02.2025  14:30:00   (aktuální čas)
Line 3:       ON-LINE          (stav sítě)
```

### Stavy sítě
- **ON-LINE**: Terminál je připojen k serveru a data se odesílají v reálném čase.
- **OFF-LINE**: Terminál není spanojen se serverem.
  - Zobrazuje: `OFF-LINE Volno: 98%` (zbývající kapacita paměti).
  - Data se ukládají do souboru `aptbuffer.txt`.
  - Po přiložení čipu se zobrazí:
    ```
    *------------------*
    |     ULOZENO      |
    |   DO SOUBORU     |
    *------------------*
    ```
- **ODESILAM SOUBOR...**: Po obnovení spojení terminál automaticky odesílá uložená data na server.

---

## 4. Konfigurace terminálu

Konfigurace se provádí pomocí speciálních RFID karet nebo zadáváním kódů přes emulátor klávesnice/čtečky.

### Vstup do menu
Pro zobrazení aktuálního nastavení přiložte konfigurační kartu s kódem `CONFIG` (nebo `SHOWCONFIG`).
- Displej zobrazí IP adresu, Masku, IP Serveru a Verzi FW.
- Terminál se na chvíli odpojí od serveru pro bezpečné zobrazení.

### Změna nastavení (SET příkazy)
Pro změnu parametru je nutné nejprve načíst příkaz (např. `SET-IP`) a následně zadat hodnotu.

**Postup zadávání hodnoty:**
1. Načtěte kartu `SET-IP` (nebo jiný příkaz).
2. Na displeji se objeví výzva (např. "Nastaveni IP").
3. Zadávejte znaky (čísla/tečky) pomocí čtečky.
   - **Nově (od v1.0.2.2):**
     - Příkaz `BACKSPACE` nebo `DEFAULT`: Smaže poslední zadaný znak (oprava chyby).
     - Příkaz `STORNO`: Okamžitě ukončí zadávání bez uložení a vrátí se na hlavní obrazovku (nedojde k restartu sítě).
     - Příkaz `OK`: Potvrdí zadání a uloží hodnotu.

### Seznam příkazů

| Příkaz | Popis | Akce |
|---|---|---|
| **Síť** | | |
| `SET-IP` | Nastavit statickou IP terminálu | Vyžaduje restart sítě |
| `SET-MASK` | Nastavit masku podsítě | Vyžaduje restart sítě |
| `SET-GATE` | Nastavit bránu (Gateway) | Vyžaduje restart sítě |
| `SET-SERV` | Nastavit IP adresu serveru | Uložení do paměti |
| `SET-DHCP` | Zapnout DHCP (automatická IP) | Restart sítě |
| `SET-STATIC` | Vypnout DHCP (použít statickou) | Restart sítě |
| `SET-ETH` | Přepnout na Ethernet (kabel) | Restart sítě |
| `SET-WIFI` | Přepnout na WiFi | Uložení |
| **WiFi / AP** | | |
| `SET-SSID` | Název WiFi sítě | Restart sítě |
| `SET-PASSW` | Heslo WiFi sítě | Restart sítě |
| `SET-APSSID` | Název sítě pro AP režim | Restart AP |
| `SET-APPASS` | Heslo pro AP režim | Restart AP |
| **Systém** | | |
| `SET-SAVE` | Uložit vše a restartovat terminál | **Restart MCU** |
| `SET-LOAD` | Znovu načíst konfiguraci z paměti | - |
| `DEFAULT` | Obnovit tovární nastavení | - |
| `RESET-BUFFER`| Smazat offline data (použít opatrně!) | - |
| `MEMINFO` | Zobrazit diagnostiku paměti | - |

---

## 5. Řešení problémů

### Terminál je v boot loopu (neustále se restartuje)
- **Příčina**: Může jít o poškozený soubor bufferu nebo chybu v konfiguraci.
- **Řešení**: Zkuste terminál zapnout bez připojené sítě. Pokud naběhne, použijte kartu `RESET-BUFFER` pro smazání poškozených offline dat.

### Terminál čte duplicitní kódy
- **Příčina**: Odrazy signálu nebo příliš citlivé nastavení čtečky.
- **Řešení**: Firmware (od v1.0.2.1) obsahuje pokročilou deduplikaci. Pokud problém přetrvává, zkontrolujte, zda nejsou dvě čtečky příliš blízko sebe.

### Nelze zadat IP adresu (chyba při psaní)
- **Řešení**: Použijte kartu/kód `BACKSPACE` pro smazání posledního znaku. Pokud chcete zadávání zrušit úplně, použijte `STORNO` – terminál se ihned vrátí do základního stavu.

### Offline Buffer se neodesílá
- **Indikace**: Terminál je ON-LINE, ale v paměti jsou stále data.
- **Řešení**: Restartujte terminál (`SET-SAVE` nebo odpojení napájení). Odesílání probíhá automaticky po navázání spojení.

---

## 6. Technické údaje

- **Napájení**: PoE (802.3af) nebo USB-C
- **Pracovní teplota**: 0°C až +50°C
- **Konektivita**: Ethernet 10/100 Mbps, WiFi 2.4 GHz
- **Protokol**: TCP/IP, port 54321 (binární protokol viz technická dokumentace)

---

*Dokumentace platná pro firmware verze 1.0.2.2 a vyšší.*
