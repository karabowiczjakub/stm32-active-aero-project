# Równoległy tor przyspieszeń RAW i MotionFX

Stan: 26 października 2026, `feature/motionfx-integration`.

Nowy interfejs nie jest podłączony do sterowania. Detektor zakrętów nadal
otrzymuje dotychczasowe surowe Y, a hamowanie jest oceniane z regresji prędkości
CAN. W symulatorze detektor nadal korzysta z telemetrii przejazdu.

## Konfiguracja

W `Core/Inc/app_config.h`:

| Parametr | Domyślnie | Znaczenie |
|---|---|---|
| `APP_ACC_SOURCE` | `ACC_SOURCE_RAW_IMU` | źródło wyłącznie nowego API |
| `ACC_SOURCE_RAW_IMU` | `0U` | fizyczny akcelerometr |
| `ACC_SOURCE_MOTIONFX` | `1U` | estymacja liniowa MotionFX |
| `MOTIONFX_FILTER_ENABLE` | `1` | filtr obu mapowanych osi MotionFX |
| `MOTIONFX_FILTER_DIVISOR` | `4` | dzielnik filtra pierwszego rzędu, co najmniej 1 |

Ustawienia można nadpisać definicjami kompilatora. Błędne źródło, wartość
włączenia filtra inna niż 0/1 oraz niedodatni dzielnik są odrzucane.
Nie zmieniono limitów świeżości: IMU i MotionFX po 100 ms.

`APP_LAP_SIM_ENABLED` pozostaje domyślnie **1**. Do diagnostyki VEH/IMU/IMU_FX
należy wybrać **0** i ponownie zbudować firmware. Symulator zachowuje swoje
dotychczasowe rekordy SIM, bez dodatkowych rekordów porównawczych.

## API

Deklaracje są w `Core/Inc/imu.h`:

```c
IMU_Acceleration_t acc;
if (IMU_GetAccelerationSnapshot(&acc)) {
    int32_t lateral_mg = acc.acc_lateral_mg;
}
```

`IMU_GetAccelerationSnapshot()` używa `APP_ACC_SOURCE`.
`IMU_GetAccelerationFromSource(source, &acc)` pozwala niezależnie odczytać
oba tory, bez przełączania stanu i bez wykonywania fuzji lub filtra.

Snapshot zawiera `acc_longitudinal_mg`, `acc_lateral_mg`, `acc_vertical_mg`,
`timestamp_ms`, `age_ms`, `sample_id`, `source` i `valid`.
Zwracana wartość odpowiada `valid`: próbka musi być dostępna i aktualna.
Przy braku ważności przyspieszenia w tym API są zerowe; nie oznacza to pomiaru
zerowego przyspieszenia i nie wolno pomijać sprawdzenia ważności.

`sample_id` zwiększa się tylko przy nowej publikacji. Powtarzane odczyty API
nie tworzą próbek. Identyfikator należy porównywać razem ze źródłem; liczniki
RAW i MotionFX są niezależne. Wartość 0 oznacza brak publikacji. Po zawinięciu
licznik pomija 0. Licznik MotionFX zachowuje ciągłość przy ponownej inicjalizacji.
Przy próbkowaniu 50 Hz i logowaniu 10 Hz numer może wzrosnąć o kilka między
wierszami; samo to nie oznacza utraty rekordów UART.

Gdy ważna próbka się zestarzeje, pozostają jej timestamp, identyfikator i wiek,
ale `valid=0`. Przed pierwszą publikacją wiek w API wynosi `UINT32_MAX`,
a w logach jest oznaczany przez `-1`. Brak MotionFX nie powoduje zastępczego
wyboru RAW. Zmiana źródła przez konfigurację wymaga kompilacji i restartu.

## Jednostki i mapowanie

Interfejs używa milli-g: `1000 mg = 1 g`. Konwersja float do mg obcina część
ułamkową w kierunku zera, po sprawdzeniu skończoności i zakresu `int32_t`.

RAW: wzdłużne = fizyczne X, boczne = fizyczne Y, pionowe = fizyczne Z.
Wartości RAW w nowym API są przed filtrem detektora i zawierają wpływ grawitacji.
Dotychczasowy filtr boczny pozostaje widoczny osobno w rekordzie IMU.

MotionFX: funkcja `MotionFxApp_MapVehicleAxes()` zawiera robocze przypisanie:

```c
acc_longitudinal = linear_g[1];
acc_lateral = -linear_g[0];
acc_vertical = linear_g[2];
```

Jest to hipoteza do sprawdzenia w pojeździe. Zachowano konfigurację `"nwu"`,
`MFX_ENGINE_OUTPUT_ENU`, tryb 6X, parametry fuzji i częstotliwość akwizycji.

Lokalny `MotionFX/Inc/motion_fx.h` opisuje `linear_acceleration` i `gravity`
jako wektory w układzie urządzenia. Dokument producenta
[DT0075, sekcja Input/output conventions](https://www.st.com/resource/en/design_tip/dm00358267-motionfx-orientation-estimation--stmicroelectronics.pdf)
rozdziela orientację osi wejściowych od układu odniesienia orientacji wyjściowej.
Samo ustawienie ENU nie potwierdza więc powyższej zamiany osi przyspieszenia.
Trzeba także sprawdzić zgodność fizycznego Z skierowanego w dół z interpretacją
orientacji wejścia. Nie zmieniano konfiguracji biblioteki na podstawie tej wątpliwości.

## Filtr i utrata danych

Filtr pracuje na float w g, oddzielnie dla osi wzdłużnej i bocznej:

```c
filtered += (raw - filtered) / MOTIONFX_FILTER_DIVISOR;
```

Pierwszy poprawny wynik inicjalizuje obie wartości bez narastania od zera.
Aktualizacja odbywa się w `MotionFxApp_Process()`, wyłącznie przy publikacji
nowego poprawnego wyniku. Getter i logger nie przeliczają filtra.
Przy wyłączonym filtrze wartości po filtrze są równe wartościom przed filtrem.
Z niepoprawnej próbki nie powstaje nowy timestamp ani identyfikator.

Po unieważnieniu wyniku lub inicjalizacji pierwszy ponownie poprawny wynik
inicjalizuje filtr od nowa. Zachowano istniejącą kontrolę odstępu próbek:
po odstępie ponad 100 ms pierwsze wywołanie ustala nową podstawę czasu bez
publikacji, a kolejne może opublikować wynik. Pojedynczy błąd nie reinicjalizuje ST.

Nie ma wspólnego stanu filtra RAW/MotionFX, który mógłby przejść pomiędzy źródłami.
Przy przyszłym podłączaniu API do detektora trzeba uwzględnić jego istniejący
filtr: podanie już filtrowanego MotionFX oznaczałoby dwa kolejne filtry.
Można wyłączyć wstępny filtr MotionFX, zachowując istniejący detektor.
Na tym etapie żadnego z tych połączeń nie wykonano.

## Logi przez istniejący UART DMA

Okres pozostaje 100 ms. Nie dodano nowego typu rekordu ani dodatkowego strumienia.

- VEH zachowuje dotychczasowe dane pojazdu i skrzydła. Dodano `acc_src`,
  `acc_valid`, `acc_age_ms`, `acc_sample_ms`, `acc_seq`, `acc_long_mg`, `acc_lat_mg`.
  `acc_src` dotyczy API; sterowanie nadal korzysta ze starego toru RAW.
- IMU zachowuje pomiary XYZ, żyroskop, filtr boczny i błędy DMA/I2C. Dodano
  `sample_ms`, `seq`, `acc_valid`, `longRaw_mg`, `latRaw_mg`, `latFilt_mg`.
  Stare `valid` oznacza otrzymanie próbki; nowe `acc_valid` uwzględnia jej wiek.
- Okresowy FX nazywa się teraz IMU_FX. Zachowano kąty, całkę żyroskopu oraz
  wektory liniowy i grawitacji. Nazwy `linENU_g`/`gravENU_g` zastąpiono przez
  `linFX_g`/`gravFX_g`, bez zmiany samych wartości. Dodano timestamp próbki,
  identyfikator, parametry filtra i pięć mapowanych wartości w mg.

Rekordy mają odpowiednio 32, 21 i 19 pól `nazwa=wartość` poza prefiksem.
Pola z końcówką `_g` pozostają w g, `_mg` są całkowitymi milli-g.
Przy nieważnym FX wartości liczbowe są zerowane do logowania, ale zachowane są
metadane ostatniej poprawnej publikacji. Należy zawsze odczytywać `valid`.

Przykład syntetyczny z testu, nie pomiar z Nucleo:

```text
IMU_FX t_ms=100 valid=1 age_ms=0 yaw_deg=+0.000 pitch_deg=+0.000 roll_deg=+0.000 gyroIntY_deg=-0.140 linFX_g=+0.500,+1.000,+0.250 gravFX_g=+0.000,+0.000,+0.000 sample_ms=100 seq=2 src=MFX filter=1 divisor=4 longRaw_mg=1000 latRaw_mg=-500 longFilt_mg=1000 latFilt_mg=-500 vert_mg=250
```

Bufor formatowania powiększono z 512 do 640 B, miejsce kolejki UART z 544 do
672 B. Kolejka nadal ma 12 miejsc i odrzuca całe wiersze zamiast oczekiwać.
Testy sprawdzają kompletność również dla maksymalnej szerokości pól liczbowych.
Nie zmieniono transportu UART ani jego callbacków. Nie dodano formatowania 64-bit
do printf; wiek jest drukowany jako 32-bit unsigned albo dosłowne `-1`.

## Weryfikacja

Przed zmianami: 95 testów PASS. Po zmianach: **117 PASS, 0 FAIL**, w tym 22 nowe.
Sprawdzono oba źródła, mapowanie i znaki, filtr włączony/wyłączony i dzielnik 2,
nowe próbki, powtarzane odczyty, błędne wyniki, zakres konwersji po zmianie znaku,
starzenie i odzyskanie, inicjalizację, brak MotionFX oraz przepełnienie ticka.
Kontrolowano pełne rekordy przesłane przez rzeczywistą kolejkę C z atrapą UART.

Testy używają rzeczywistych modułów C, UBSan i kontroli float-cast-overflow.
Funkcje fuzji ST są zastąpione kontrolowanymi wynikami. Nie jest to potwierdzenie
fizycznego mapowania ani jakości estymacji biblioteki na mikrokontrolerze.

Normalny firmware i symulator skompilowano i zlinkowano z rzeczywistym HAL, BSP
i `MotionFX_CM4F_wc32_ot_hard.a`, GNU Tools for STM32 14.3.1, hard float, Debug -O0.

| Wariant | Flash BIN | Statyczny RAM | RAM z rezerwą heap/stack |
|---|---:|---:|---:|
| Normalny | 105540 B | 17796 B | 19332 B |
| Symulator | 160068 B | 17168 B | 18708 B |

Pozostało sześć istniejących ostrzeżeń: pięć nieużywanych funkcji oraz stary
komentarz w servo.c. Nie zmieniano tych fragmentów. Zgodność 113 chronionych
plików sprawdzono hashami, w tym sterownika, CAN, serwa, HAL, biblioteki ST,
konfiguracji peryferiów i symulatora. Usunięto tylko zakomentowany test ruchu
serwa z main.c; normalna kolejność zadań i WFI pozostają bez zmian.

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
```

Artefakty kompilacji i logi weryfikacji są w katalogu tymczasowym WSL:
`/tmp/motionfx-integration-fba6ts1g/`. Nie zmieniano Debug, nie flashowano płytki,
nie wykonano commita ani pusha.

## Nucleo i piątkowa jazda

1. Wybrać `APP_LAP_SIM_ENABLED=0`, pozostawiając początkowo RAW jako źródło API.
   Zbudować i samodzielnie wgrać firmware w CubeIDE.
2. W PuTTY ustawić port ST-LINK VCP, 460800, 8N1, flow control None.
   Rejestrować dotychczasowy strumień do pliku tekstowego terminala.
3. Sprawdzić LOG_INFO version=2, IMU init OK, MotionFX diagnostic ready oraz
   cykliczne VEH/IMU/IMU_FX. Brak OBD pozostawia skrzydło w NORMAL i nie blokuje
   diagnostyki IMU/MotionFX.
4. W bezruchu i przy znanych przechyłach sprawdzić osie, znaki i resztkowe liniowe
   przyspieszenie MotionFX. RAW nadal zawiera grawitację.
5. Podczas powtarzalnego przyspieszania i zwalniania sprawdzić oś wzdłużną;
   podczas skrętów w obie strony sprawdzić oś boczną. Zweryfikować, czy podane
   mapowanie daje te same dodatnie zwroty co fizyczne X/Y RAW.
6. Porównać wartości przed i po filtrze oraz opóźnienie zmian. Logowanie 10 Hz
   nie pokazuje wszystkich próbek przetwarzanych nominalnie w 50 Hz.
7. Sprawdzić age_ms, sample_ms i seq: odczyt loggera nie tworzy nowej próbki.
   Przy braku danych ważność ma zniknąć, a timestamp ostatniej publikacji pozostać.
8. Opcjonalnie przebudować z `APP_ACC_SOURCE=ACC_SOURCE_MOTIONFX` oraz z wyłączonym
   filtrem. Zmieni się API/diagnostyka; decyzje skrzydła nadal pozostają na starym torze.
9. Monitorować w debuggerze liczniki pominiętych logów i błędów UART. Czas CPU,
   jitter i prawidłowość fizycznych osi nie zostały zmierzone w tej implementacji.
