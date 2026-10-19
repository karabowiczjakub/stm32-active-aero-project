# Odtwarzanie telemetrii okrążenia na NUCLEO-G474RE

Aktualny nadzór danych w trybie rzeczywistym opisano w [failsafe.md](failsafe.md).
Nie blokuje on odtwarzania bez fizycznych czujników.

## Uruchomienie

1. W `Core/Inc/app_config.h` ustaw `APP_LAP_SIM_ENABLED` na `1`.
   W repozytorium pozostawiono domyślne `0` (normalny sterownik).
2. W STM32CubeIDE odśwież projekt, wykonaj Clean Project i Build Project.
   Katalog `Core` jest już źródłem projektu. Sprawdź, że konsola kompilacji
   obejmuje nowe `lap_sim.c` i `lap_data.c`; nie używaj starej listy źródeł
   z wygenerowanego `Debug/makefile` bez jej odtworzenia przez IDE.
3. Wgraj firmware przez ST-LINK. Zachowaj dotychczasowe zasilanie serwa,
   wspólną masę i połączenie PWM. Samochód ani połączenie OBD nie są potrzebne.
4. Otwórz istniejący port szeregowy COM1/ST-LINK VCP: **460800 baud, 8N1,
   bez kontroli przepływu**. W tym projekcie COM1 korzysta z LPUART1.
5. Poczekaj na `SIM_WAITING USER_BUTTON`. Skrzydło pozostaje w NORMAL.
6. Naciśnij USER, synchronizując start z początkiem klipu. Pojawi się
   `SIM_START`, nagłówek i rekordy `SIM`. Następne naciśnięcia są ignorowane.
7. Po próbce 104500 ms pojawi się `SIM_END,104500`. Stan algorytmu i serwo
   wracają do NORMAL. Kolejny przebieg wymaga RESET mikrokontrolera.

Callback USER pozostał bez zmian: ustawia tylko flagę. Początek zegara jest
ustalany przy obsłudze tej flagi w pętli głównej, poza przerwaniem. Synchronizacja
przyciskiem ma więc rozdzielczość ticka 1 ms oraz opóźnienie obsługi pętli;
nie jest pomiarem dokładnego czasu zbocza EXTI.

## Dane i generowanie

Źródło: `data/telemetry_50hz.csv` i `data/README_telemetria_onboard.md`.
5226 rekordów, od 0 do 104500 ms, krok 20 ms. CSV nie został zmieniony.
Jego SHA256:

```text
e6dec04ff179c46385da9b857e42a40a81687b6e39a94030ae7d01bb0b5c5ecd
```

```bash
python3 -B tools/csv_to_lap_data.py
python3 -B tools/csv_to_lap_data.py --check
```

Pierwsza komenda generuje `Core/Inc/lap_data.h` i `Core/Src/lap_data.c`.
Druga sprawdza ich zgodność bez zapisywania plików. Generator kontroluje kolumny,
ciągłość czasu, skończone wartości, zakres prędkości 0..255 km/h, jednostki,
zakres signed milli-g i znane etykiety jakości. Nie interpoluje wejścia.

Każdy rekord Flash ma 12 B: timestamp `uint32_t`, dwa kanały `int16_t` w milli-g,
prędkość `uint8_t` i flagi jakości. Rozmiar sprawdza `_Static_assert`.
Cała tablica `const` zajmuje **62712 B** w `.rodata`, bez kopii do RAM.
Przy `APP_LAP_SIM_ENABLED=0` tablica i funkcje symulatora nie są kompilowane.

## Jednostki i osie

- Boczne G: dodatni znak oznacza punkt po prawej stronie G-metera filmu.
- Wzdłużne G: dodatni znak oznacza punkt poniżej środka, opisany w źródle jako
  przyspieszanie. Jest dostępny w `LapSim_Diag_t.longitudinal_mg` i logach.
- Przeliczenie: `1000 mg = 1 g`. Generator zaokrągla do najbliższego milli-g.
  Aktualne dane mają już trzy miejsca dziesiętne. Nie obcina wartości do ±2 g;
  zachowane minimum boczne wynosi −2.101 g.
- Są to estymowane przyspieszenia liniowe z obrazu, bez odzyskanej osi pionowej
  i żyroskopów. Nie przypisujemy ich fizycznym osiom LSM6DSO ani układowi ENU.
- Detektor otrzymuje podpisane boczne milli-g. Wartość bezwzględna jest liczona
  dopiero po filtracji, dokładnie jak wcześniej.
- Wzdłużne G nie wpływa na AIR BRAKE. MotionFX otrzymuje wyłącznie rzeczywiste
  dane trójosiowego akcelerometru i żyroskopu, zachowując swoją konfigurację.

Flagi `quality`: bit 0 (`1`) = prędkość interpolowana w źródłowym CSV;
bit 1 (`2`) = G odczytane z rozszerzonego ROI. `0` oznacza oba kanały `read`.
W snapshotach/logach flagi odnoszą się do faktycznie użytej prędkości i bieżącego G.
Jakość jest zachowana informacyjnie; nie dodano nowej polityki odrzucania próbek.
50 Hz próbkowania filmu nie oznacza 50 niezależnych aktualizacji jego nakładki.

## Integracja i czas

- `main.c`: wybór źródła przez jeden przełącznik, obsługa flagi USER,
  `LapSim_Task()` przed rzeczywistym `IMU_Task()`. W symulacji pominięte są
  `CAN_Test_Init()`, przetwarzanie odebranej prędkości i żądania OBD.
  TIM6 nie ustawia flagi żądania OBD. Konfiguracja timerów, FDCAN, I2C i DMA
  pozostała bez zmian. TIM7 nadal zleca rzeczywiste odczyty diagnostyczne IMU.
- `Control_Update_From_Speed()` zachowuje interfejs i pobiera `HAL_GetTick()`
  dla normalnego trybu. Wspólne `Control_Update_From_SpeedAt()` przyjmuje czas
  próbki dla regresji i wszystkich timerów decyzyjnych. Pozostały odczyt ticka
  w `Control_GetDiagSnapshot()` oblicza wyłącznie diagnostyczny czas do końca
  potwierdzenia/podtrzymania; nie zmienia decyzji sterownika.
- `IMU_ProcessLateralSample_mG()` zawiera dotychczasowy filtr `/4`, progi
  200/120 mg oraz potwierdzanie 3/5 próbek. W normalnym trybie jest zasilany
  wybraną osią Y. W symulacji dekoder rzeczywistego DMA nie wywołuje tej funkcji;
  stan detektora może zmieniać tylko telemetria okrążenia.
- Po każdej próbce bocznej zdarzenie z `IMU_TakeCornerDetectedEvent()` uruchamia
  `Control_ForceNormalForCorner()`. `IMU_IsCornerDetected()` zachowuje priorytet
  zakrętu w istniejącym sterowniku. Serwo i PWM nie zostały zmienione.
- `Control_Reset()` i `IMU_ResetCornerDetection()` są dostępne w obu trybach po dodaniu fail-safe.
  Start resetuje historię, potwierdzenia i detektor. Koniec/błąd resetuje również
  wewnętrzny stan skrzydła oraz ustawia rzeczywiste serwo w NORMAL.

Odtwarzacz korzysta z `HAL_GetTick()` i istniejącego wybudzania przez SysTick
oraz `__WFI()`. Nie ma dodatkowych przerwań, aktywnego oczekiwania ani `HAL_Delay()`.
Terminy są liczone od jednego początku, bez narastającego dryfu. Kanały są
obsługiwane chronologicznie; przy jednakowym czasie pierwszeństwo ma boczne G.

Boczne i wzdłużne G: co **20 ms**. Prędkość: co **50 ms**, 2091 aktualizacji.
Dla 50 ms używana jest wartość z CSV w 40 ms, dla 150 ms z 140 ms itd.
Przyszłe próbki nie są odczytywane. Prędkość zachowuje rozdzielczość OBD 1 km/h.
Pole `speed_sample_ms` oznacza termin podania do sterownika, nie czas wiersza CSV.

Regresja ma osiem miejsc historii, okno 300+30 ms i minimalny span 250 ms.
Przy 50 ms jest poprawna najwcześniej od 250 ms; potem typowo używa siedmiu próbek
z 300 ms. Podawanie prędkości co 20 ms dałoby tylko 140 ms historii i stale
niepoprawny trend. Filtr zakrętu zachowuje częstotliwość obsługi 50 Hz,
pomimo skonfigurowanego ODR rzeczywistego sensora 104 Hz.

## Opóźnienie

`APP_LAP_SIM_MAX_LATENESS_MS` wynosi domyślnie **10 ms**. Można go zmienić w tym
samym `app_config.h`; musi pozostać poniżej 20 ms. Dzięki temu nie nadrabiamy
kilku zaległych aktualizacji bocznego G. Dla niewielkiego spóźnienia zachowane są
timestampy i kolejność zdarzeń; regresja i timery nie dostają czasu grupowej obsługi.

Opóźnienie jest sprawdzane przed każdym zdarzeniem. Przy wartości **większej**
od limitu symulator kończy pracę przed podaniem tego zdarzenia, wymusza NORMAL
i emituje `SIM_TIMING_ERROR,t_ms,lag_ms`. Nie nadrabia historycznych decyzji serwa.
Limit jest sprawdzany w pętli, więc nie zastępuje sprzętowego watchdog-a:
podczas długiej blokady NORMAL zostanie ustawiony po odzyskaniu sterowania pętli.

## Logi

W symulacji rozbudowane cykliczne `VEH/IMU/FX` są wyłączone, a tekstowe komunikaty
detektora zakrętu pominięte. Krótkie komunikaty zmian serwa pozostały dostępne.
Normalny tryb zachowuje całą dotychczasową diagnostykę.

```text
SIM_HEADER,t_ms,speed_kmh,lat_g,long_g,rate_kmh_s,rate_valid,corner,wing,lag_ms,quality
SIM,0,215,-0.038,0.181,nan,0,0,NORMAL,0,0
```

`SIM` jest rekordem CSV. Do eksportu zachowaj wiersze zaczynające się od `SIM,`,
a jako nagłówek użyj wiersza `SIM_HEADER` z pierwszą kolumną nazwaną `record`.
Pozostałe komunikaty mają inne prefiksy. Trend `nan` z `rate_valid=0` oznacza brak
poprawnej regresji, również po końcowym resecie sterownika. `lag_ms` opisuje
spóźnienie ostatnio obsłużonego zdarzenia, nie wiek podtrzymywanej próbki.

Rekord jest emitowany nominalnie co 100 ms, dodatkowo przy zmianie stanu skrzydła,
starcie i zakończeniu. Zawiera ostatnie dostępne wejścia; G może być starsze od
czasu logu o mniej niż 20 ms, a prędkość o mniej niż 50 ms przy obsłudze bez opóźnień.

Test pełnego nagrania dał 1060 rekordów SIM i 32 komunikaty zmian serwa.
Najdłuższy SIM miał 54 B z CRLF: **1.172 ms** transmisji przy 460800 8N1.
Test dodatkowo wymusza limit 80 B, czyli 1.736 ms. Cały strumień to około
52 kB na 104.5 s, około 0.5 kB/s przy przepustowości UART 46.08 kB/s.
To czas na przewodzie, bez kosztu formatowania i pracy przerwań. UART i `printf()`
pozostają blokujące; rzeczywisty jitter trzeba zmierzyć na płytce.

## Weryfikacja wykonana w tym środowisku

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
python3 -B tools/csv_to_lap_data.py --check
git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check
```

**17 testów PASS, 0 FAIL.** GCC hosta z `-Wall -Wextra -Werror` i UBSan.
Testy wykonują rzeczywiste `wing_control.c`, `imu.c`, `servo.c`, `obd.c`,
`diag_log.c`, `lap_sim.c` i `lap_data.c`. Pętla i callbacki są pobierane z
`main.c`; zastąpiony jest tylko sprzęt i procedura jego inicjalizacji.

Sprawdzono generowanie wszystkich próbek i jakości, okresy 20/50 ms i podwójne
wywołania pętli, regresję, AIR BRAKE i jego histerezę/minimalną prędkość,
OPEN 55/45 km/h, potwierdzanie i podtrzymanie NORMAL, priorytet zakrętu,
filtr oraz progi 3/5 próbek, przycisk USER i brak restartu, koniec nagrania,
limit 10 ms włącznie, przerwanie przy 11 ms i 200 ms, przepełnienie ticka,
separację fizycznego IMU, rzeczywiste wejścia MotionFX i brak żądań OBD.
Przebiegi z opóźnieniem 7 ms i bez niego mają identyczne decyzje i regresję.

Normalny tryb porównano dodatkowo z kodem zachowanym przed zmianą: cały strumień
diagnostyczny i 1001 kolejnych snapshotów są identyczne. Test zachowuje SHA256
tego przebiegu odniesienia, więc działa również bez tymczasowej kopii źródeł.

Wykonano rzeczywistą kompilację i linkowanie ARM wersji odniesienia i obu trybów:
GNU Tools for STM32 14.3.1, Cortex-M4F, hard float, Debug `-O0`, po 39 źródeł,
z oryginalnym startupem, HAL, BSP, skryptem linkera i biblioteką MotionFX.
Użyto narzędzi z instalacji CubeIDE przez WSL; nie uruchamiano GUI IDE.
Wyniki zapisano w `/tmp`, bez nadpisywania `Debug/`.

| Obraz ARM | Flash | `.data + .bss` | RAM z minimalnym heap/stack |
|---|---:|---:|---:|
| Przed zmianą | 95 344 B | 8 740 B | 10 280 B |
| Normalny, flaga 0 | 95 400 B | 8 740 B | 10 280 B |
| Symulacja, flaga 1 | 153 088 B | 8 232 B | 9 768 B |

Limity linkera: 512 KiB Flash, 128 KiB RAM. Normalny tryb przyrósł o 56 B Flash
przez wydzielenie wspólnych wejść; RAM nie przyrósł. Symulacja zużywa mniej RAM
dzięki pominięciu bufora rozbudowanych logów. `lap_data` ma adres Flash i rozmiar
62712 B. Sprawdzono brak tablicy i funkcji symulatora w symbolach normalnego obrazu
oraz brak funkcji żądań OBD w obrazie symulacji. Nie zmierzono szczytowego użycia stosu.

Kompilacja ma te same sześć ostrzeżeń co kod sprzed zmian: pięć nieużywanych
funkcji pomocniczych IMU/OBD oraz zagnieżdżony początek komentarza w `servo.c`.
Nie dodano nowych ostrzeżeń. Nie poprawiano tych niezwiązanych miejsc.

`diff --check` przeszedł z uznaniem CRLF za zakończenie linii. Zwykła konfiguracja
tego repo zgłasza CR jako trailing whitespace; zachowano istniejące CRLF zamiast
zmieniać zakończenia wszystkich linii lub konfigurację Git.

## Pliki i pozostałe sprawdzenia na płytce

Utworzono:

- `Core/Inc/app_config.h`, `Core/Inc/lap_sim.h`, `Core/Inc/lap_data.h`;
- `Core/Src/lap_sim.c`, `Core/Src/lap_data.c`;
- `tools/csv_to_lap_data.py`;
- `tests/test_lap_sim.py`, `tests/host/main.h`, `tests/host/hardware.c`,
  `tests/host/test_controller.c`;
- `docs/lap_sim.md`.

Zmodyfikowano:

- `Core/Src/main.c`, `Core/Src/wing_control.c`, `Core/Src/imu.c`, `Core/Src/diag_log.c`;
- `Core/Inc/wing_control.h`, `Core/Inc/imu.h`.

Pozostawiono bez zmian: `obd.c/.h`, `servo.c/.h`, `motion_fx_app.c/.h`, HAL,
MotionFX, UART, `syscalls.c`, `stm32g4xx_it.c/.h`, `.ioc`, `.cproject`, linkery
i istniejące pliki wygenerowane w `Debug/`. Nie wykonano commit, push ani merge.

Nie wykonano flashowania ani testu fizycznej płytki. Do sprawdzenia pozostają:
rzeczywisty jitter przy aktywnym MotionFX i logowaniu, synchronizacja przyciskiem,
impulsy PWM NORMAL=1600 / OPEN=1900 / AIR_BRAKE=1100 µs, ruch serwa pod obciążeniem
i brak ramek żądań OBD na CAN. Estymowane G z filmu pozostają wejściem testowym,
a nie walidacją fizycznej dokładności IMU lub aerodynamiki.
