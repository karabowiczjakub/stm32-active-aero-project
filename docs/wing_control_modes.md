# Sterowanie LEGACY i HYBRID

Wybór przez `APP_WING_CONTROL_MODE` w `Core/Inc/app_config.h`:

| Tryb | Polecenie serwa | Równoległe obliczenia |
|---|---|---|
| `WING_CONTROL_LEGACY_ONLY` | LEGACY | MotionFX diagnostycznie |
| `WING_CONTROL_LEGACY_SHADOW_HYBRID` | LEGACY | HYBRID bez wpływu na PWM |
| `WING_CONTROL_HYBRID_ACTIVE` — aktualny domyślny | HYBRID | LEGACY jako punkt odniesienia |

Aktualne źródła mają `APP_LAP_SIM_ENABLED=0`. Aby wykonać porównanie bez zmiany
sterowania serwem przez LEGACY, należy przed kompilacją wybrać
`APP_WING_CONTROL_MODE=WING_CONTROL_LEGACY_SHADOW_HYBRID`. Ustawienie w źródłach
nie potwierdza konfiguracji obrazu znajdującego się we Flash.

Stare nazwy `WING_CONTROL_LEGACY_SHADOW_FX` i `WING_CONTROL_FX_ACTIVE` pozostają
aliasami wartości 1 i 2. Wariant FX został zastąpiony hybrydą.
`APP_ACC_SOURCE` nadal wybiera wyłącznie źródło wspólnego API diagnostycznego.

## Wspólna logika

Oba warianty korzystają wyłącznie z dotychczasowego detektora zakrętu RAW.
Nie zmieniono jego filtracji, progów, histerezy ani potwierdzeń.
Przyspieszenie boczne MotionFX pozostaje diagnostyczne.

Regresja OBD jest wspólna i zawiera wyłącznie nowe odpowiedzi prędkości.
HYBRID jej nie modyfikuje. Automaty mają oddzielne stany i potwierdzenia OPEN.
Zachowano progi prędkości, przejście AIR BRAKE → NORMAL, blokadę OPEN po BRAKE
oraz PWM. Symulator korzysta z LEGACY we wszystkich trybach i zachowuje USER
BUTTON, NORMAL na końcu i SIM_TIMING_ERROR. Zachowano harmonogram pętli głównej oraz WFI.

## Wykrywanie i przejęcie hamowania

1. OBD może samodzielnie uruchomić AIR BRAKE przy dotychczasowym progu wejścia.
2. Uzbrojony detektor FX może uruchomić AIR BRAKE wcześniej po potwierdzeniu
   ujemnego przyspieszenia wzdłużnego. Nadal obowiązują minimum 30 km/h,
   brak zakrętu RAW i poprawne dane OBD, IMU oraz regresji.
3. Faza WAIT_OBD daje maksymalnie 400 ms na przekroczenie progu wejścia OBD.
   Ten sam impuls nie odnawia czasu. Brak potwierdzenia kończy się NORMAL.
4. Po potwierdzeniu faza OBD korzysta z regresji i dotychczasowego progu
   podtrzymania. Zanik wartości FX nie kończy BRAKE.
5. Wyjście wymaga potwierdzenia przekroczenia progu OFF przez 100 ms na nowych
   odpowiedziach OBD. To potwierdzenie końca, nie minimalny czas podtrzymania.
6. Po wykorzystaniu impulsu FX detektor uzbraja się ponownie po powrocie sygnału
   powyżej progu resetowania przez wymagany czas.

Wczesne wejście FX jest blokowane, gdy regresja wskazuje wyraźne przyspieszanie
(dotychczasowy ACCEL_THRESHOLD_X10). Taki impuls zostaje zużyty do ponownego
uzbrojenia. Ogranicza to wyzwolenia po zakończeniu przyspieszania.

## Parametry eksperymentalne

| Parametr | Domyślnie |
|---|---:|
| `MOTIONFX_ATIME` | `6.0f` |
| `MOTIONFX_FILTER_ENABLE` / `MOTIONFX_FILTER_DIVISOR` | `1` / `4` |
| `HYBRID_FX_ON_MG` / `HYBRID_FX_REARM_MG` | `-140` / `-70` |
| `HYBRID_FX_ON_CONFIRM_MS` | `60` |
| `HYBRID_FX_REARM_CONFIRM_MS` | `80` |
| `HYBRID_OBD_CONFIRM_MS` | `400` |
| `HYBRID_OBD_OFF_CONFIRM_MS` | `100` |

To kompromis dobrany w testach modelowych; wyniki przejazdu SHADOW opisano poniżej.
Nie potwierdzono jeszcze fizycznego sterowania HYBRID w samochodzie.
Nie zmieniono osi, ODR ani parametrów fuzji innych niż ATime.

## Ważność i diagnostyka

Brak świeżego OBD lub IMU oraz niepoprawna regresja wymuszają NORMAL.
Zakręt RAW i ograniczenie prędkości przerywają również WAIT_OBD.
Niepoprawny lub przeterminowany FX blokuje wczesne wejście i wycofuje WAIT_OBD.
Nie blokuje samodzielnego działania prawidłowego OBD, także po przejęciu.
To jawna właściwość HYBRID, widoczna w flags_fx, a nie ciche przełączenie trybu.
NaN, Inf, nieciągłe numery próbek oraz timestampy nie potwierdzają wejścia FX.

CTRL_CMP ma 27 pól, m.in. wing_actual, wing_legacy, wing_hybrid, corner_raw,
phase (0=IDLE, 1=WAIT_OBD, 2=OBD), armed, wait_ms i liczniki early/confirmed/rejected.
Pozostają liczniki strat UART. CTRL_EVT path=HYBRID opisuje zmianę położenia;
CTRL_HYB opisuje przejęcie przez OBD, odrzucenie, weto przyspieszania i dostępność FX.
Rekordy VEH, IMU, IMU_FX i okres 100 ms pozostają. UART nadal działa przez DMA.

## Walidacja i dalsze uruchomienie

Testy: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v`.
Sprawdzają rzeczywisty kod C z atrapami HAL i wstrzykiwanymi wynikami MotionFX.
Historyczny wynik implementacji z 3 listopada: 154 testy PASS. LEGACY zachował
cztery ślady po 6001 punktów, w tym przepełnienie ticka. Zbudowano trzy tryby
normalne i trzy symulatora. Warianty testowe wybierają teraz tryb jawnie:
zmiana domyślnej konfiguracji aplikacji nie zmienia testu SHADOW ani porównania
LEGACY z zapisanym śladem. Osobno sprawdzany jest wybór wyjścia domyślnej konfiguracji.

Weryfikacja przed merge z 10 listopada: ponownie 154/154 testy PASS oraz sześć
poprawnych kompilacji i linkowań ARM z rzeczywistą biblioteką ST.
Flash/RAM [B]: LEGACY 107736/22100, SHADOW 109360/22100, HYBRID 109520/22100;
każdy wariant symulatora 160552/21452. Pozostało sześć wcześniejszych ostrzeżeń
w każdym wariancie: pięć nieużywanych funkcji pomocniczych i komentarz w `servo.c`.
W ramach tej weryfikacji nie wykonywano nowych testów sprzętowych ani zapisu Flash.

Badania rzeczywistej biblioteki ST, testy kontrolera na Nucleo z RAM, metryki
oraz ograniczenia opisuje raport poza repozytorium:
`../motionfx_bench_20261102_movbqfg4/hybrid_20261103/REPORT.md` względem katalogu projektu.

Testy stanowiskowe z 3 listopada uruchamiano z RAM, bez zmiany Flash.
Programowanie kolejnego obrazu wymaga osobnej zgody. Po nim należy sprawdzić w PuTTY: 460800, 8N1,
flow control None, wing_mode=LEGACY_SHADOW_HYBRID i wing_actual=wing_legacy.
APP_LAP_SIM_ENABLED musi wynosić 0. Włączyć zapis całej sesji przed resetem.
Sprawdzać rosnący plik, aktualność źródeł, seq oraz uart_drop/uart_err.

Przejazd SHADOW z 3 listopada wykazał cztery potwierdzone wcześniejsze wejścia
HYBRID o 123–383 ms oraz osiem wejść niepotwierdzonych przez OBD w ciągu 400 ms.
Łączny czas BRAKE wyniósł 21,388 s dla LEGACY i 31,626 s dla HYBRID.
Szczegółowy raport i pomiary znajdują się poza repozytorium, w katalogu
`hybrid_road_audit_20261103_57ytjvxt` obok projektu. Są to wyniki porównania SHADOW,
bez potwierdzenia fizycznego sterowania HYBRID w samochodzie.

Znane ograniczenia: symulator nadal wykonuje tylko LEGACY; krótkie pary
`fx_unavailable` / `fx_recovered` po zakręcie wymagają uporządkowania diagnostyki;
`rxovf` zlicza przepełnienia programowej kolejki CAN, a nie sprzętowego FIFO.
Nastawy pozostają eksperymentalne. NORMAL jest stanem awaryjnym demonstratora,
bez gwarancji bezpieczeństwa aerodynamicznego w rzeczywistym samochodzie.
