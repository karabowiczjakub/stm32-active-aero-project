# Minimalny fail-safe danych OBD i IMU

Kolejny etap — timeout DMA i walidację MotionFX — opisano w [imu_motionfx.md](imu_motionfx.md).
Wyniki 37 testów i rozmiary poniżej dotyczą pierwotnego etapu fail-safe.

## Zakres

Zaimplementowano nadzór aktualności wymaganych źródeł w istniejących modułach.
Nie dodano nowej maszyny stanów. Regresja, progi, histerezy, czasy potwierdzania,
filtr boczny, priorytet zakrętu, PWM i konfiguracja peryferiów pozostały zachowane.
Zmiany dotyczą wyłącznie reakcji na brak danych, niepoprawną regresję i powrót pomiarów.

## Limity

W `Core/Inc/app_config.h` dodano konfigurowalne wartości:

| Definicja | Domyślnie | Znaczenie |
|---|---:|---|
| `APP_OBD_SPEED_MAX_AGE_MS` | 500 ms | maksymalny wiek ostatniej odebranej prędkości i ramki oczekującej w kolejce |
| `APP_IMU_MAX_AGE_MS` | 100 ms | maksymalny wiek ostatniej przetworzonej próbki IMU, liczony od zakończenia odczytu |

Próbka jest aktualna przy wieku równym limitowi; awaria zaczyna się po jego przekroczeniu.
Różnice czasu są liczone na `uint32_t`, z obsługą przepełnienia `HAL_GetTick()`.
Konfiguracja odrzuca limity zerowe i nie mniejsze niż połowa zakresu licznika.

500 ms pozostawia margines wobec okresu zapytań 50 ms, oczekiwania na odpowiedź
oraz ponawiania po nieudanym żądaniu. Przykładowo: nowe żądanie 50 ms po ostatnim
pomiarze, timeout po 200 ms i odpowiedź na ponowienie po kolejnych 200 ms dają
450 ms bez poprawnej próbki. Limit świeżości nie zmienia timeoutu żądania 200 ms.
100 ms odpowiada pięciu nominalnym okresom obsługi IMU po 20 ms.
Są to ustawienia demonstratora; nie zmierzono w tej sesji opóźnień fizycznego ECU.
Należy potwierdzić je logami na stanowisku z docelowym źródłem danych.

## Reakcja i odzyskiwanie

`Control_CheckDataFreshness()` działa cyklicznie w normalnej pętli, przed
`IMU_Task()`, niezależnie od napływu CAN. Jest również wywoływana przed podmianą
ostatniej prędkości w dekoderze oraz przed przyjęciem próbki przez sterownik.
Dzięki temu świeża odpowiedź po długiej przerwie nie ukrywa wcześniejszej utraty danych.

Brak pierwszej próbki jest rozróżniany od przeterminowania wcześniej odebranej próbki.
Wejście w awarię źródła uruchamia istniejący `Control_Reset()`: NORMAL, PWM 1600 µs,
wyczyszczenie historii prędkości, potwierdzania OPEN, podtrzymania po hamowaniu
i diagnostyki regresji. Reset odbywa się przy przejściu od dostępnych źródeł do
awarii, a nie w każdej iteracji. Podczas awarii próbki nie zasilają historii sterowania.

Po odzyskaniu obu źródeł historia jest ponownie zbierana. Niepoprawna regresja
wymusza NORMAL i kasuje potwierdzenia, ale pozostawia historię do dalszego zbierania.
Przy nominalnych odstępach 50 ms regresja staje się poprawna po 250 ms.
OPEN wymaga następnie dotychczasowego potwierdzania 1000 ms; AIR BRAKE może
powstać dopiero z aktualnej, poprawnej regresji spełniającej istniejące warunki.
Nie dodano dodatkowego czasu oczekiwania ani nowych progów hamowania.

Po luce w próbkach IMU większej niż jego limit istniejący reset detektora czyści
filtr i liczniki przed przetworzeniem pierwszej nowej próbki. Wzór filtra, progi
200/120 mg i potwierdzanie 3/5 próbek pozostają takie same. Ciągłe poprawne pomiary
IMU nie są resetowane z powodu awarii OBD.

Kolejka CAN ma jedno dodatkowe pole czasu odbioru ustawiane w callbacku RX.
Dekoder odrzuca ramki starsze niż limit. Zaakceptowane ramki przekazują czas odbioru
zarówno do diagnostyki, jak i do istniejącej regresji. Sposób wysyłania żądań,
przerwania, FIFO i warunki dekodowania odpowiedzi pozostały zachowane.
Nieużywane obecnie publiczne `CAN_Poll_Rx()` korzysta z tego samego dekodera.

## Diagnostyka

Dotychczasowe rekordy VEH, IMU, FX i SIM nie zmieniają formatu.
Dodano osobny komunikat `FAILSAFE flags=N`, tylko po zmianie maski błędów.
Maska jest również dostępna jako `Control_Diag_t.failsafe_flags`.

| Bit / wartość | Znaczenie |
|---|---|
| 1 | brak pierwszej prędkości OBD |
| 2 | przeterminowana prędkość OBD |
| 4 | brak pierwszej próbki IMU |
| 8 | przeterminowane IMU |
| 16 | niepoprawna lub jeszcze niegotowa regresja |
| 0 | oba źródła aktualne i regresja poprawna |

Bity sumują się: 18 oznacza utratę OBD i brak poprawnej regresji, 24 utratę IMU
i brak poprawnej regresji. Samo 16 przy rozruchu albo po odzyskaniu pomiarów jest
normalnym etapem zbierania historii. Istniejące `obd_valid` i `imu.valid` nadal
informują, że próbka została kiedykolwiek odebrana; świeżość pokazuje nowa maska.
Wyzerowanie maski nie oznacza obowiązkowego OPEN: nadal decydują dotychczasowe
warunki, w tym priorytet zakrętu.

## Symulator

Przy `APP_LAP_SIM_ENABLED=1` nadzór fizycznych źródeł jest pomijany.
Nieobecność OBD i LSM6DSO nie blokuje telemetrii okrążenia. Pozostają USER BUTTON,
czas próbki symulatora, regresja, detektor zakrętów, `SIM_TIMING_ERROR` i reset
po zakończeniu. Nie zmieniono `lap_sim.c`, tablicy danych ani MotionFX.
Obsługa rzeczywistego IMU i MotionFX nadal może zajmować czas procesora, tak jak wcześniej.

## Weryfikacja

**37 testów PASS, 0 FAIL:** 20 nowych testów nadzoru oraz 17 wcześniejszych testów.
Testy kompilują aktualne moduły C i rzeczywistą treść pętli `main.c` z atrapami HAL.
Nie zastępują algorytmu sterowania modelem w Pythonie. GCC hosta używa
`-Wall -Wextra -Werror` i UBSan, zachowując dotychczasowe wyłączenia ostrzeżeń
od nieużywanych funkcji i starego komentarza w serwie.

Nowe testy obejmują: brak OBD od startu; utratę OBD w NORMAL, OPEN i AIR BRAKE;
stare ramki oraz granicę wieku; zmienne opóźnienie kolejki i czas regresji;
brak IMU od startu, utratę na wprost i w zakręcie; niepoprawną regresję w obu
aktywnych stanach; przerwy podczas potwierdzania OPEN; odzyskanie obu źródeł,
odbudowę historii i ponowne potwierdzenie OPEN; reset starego detektora zakrętu;
przepełnienie ticka dla obu źródeł, kolejki i odzyskiwania; inne limity 700/140 ms.

Wszystkie 1001 snapshotów poprawnego przebiegu są identyczne z zapisanymi przed
zmianami. Ich SHA256 to
`5f2fc19d98bec7cbfef8806d3671314e927d1d55057ed747d7338d33106ddd4d`.
Dwa wcześniejsze testy dostosowano do zamierzonego nowego kontraktu: dodatkowych
komunikatów fail-safe i dostępności `Control_Reset()` również w normalnym obrazie.
Nie zmieniono oczekiwanych decyzji algorytmu. Test całego okrążenia dodatkowo
potwierdza brak fizycznych próbek IMU/MotionFX i brak żądań OBD.

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
python3 -B tools/csv_to_lap_data.py --check
git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check -- Core tests docs
```

Sprawdzenie CSV i zmienionych plików z zachowaniem CRLF: PASS.
Wykonane zwykłe `git diff --check` całego katalogu zwraca błąd z powodu interpretacji
CRLF oraz wcześniejszych zmian w wygenerowanym `Debug/`. Nie zmieniano zakończeń
linii, konfiguracji Git ani istniejących artefaktów, aby usunąć ten niezwiązany problem.

Skompilowano i zlinkowano cztery obrazy: oba tryby sprzed zmian oraz oba po zmianach,
po 39 źródeł, GNU Tools for STM32 14.3.1, Cortex-M4F, hard float, Debug -O0.
Użyto oryginalnych HAL, startupu, BSP, skryptu linkera i biblioteki MotionFX.

| Obraz | Flash, obraz BIN | Statyczny RAM (.data+.bss) | RAM z rezerwą heap/stack |
|---|---:|---:|---:|
| Normalny przed zmianą | 95 400 B | 8 740 B | 10 280 B |
| Normalny po zmianie | 96 032 B | 8 772 B | 10 312 B |
| Symulator przed zmianą | 153 088 B | 8 232 B | 9 768 B |
| Symulator po zmianie | 153 152 B | 8 264 B | 9 800 B |

Przyrost: normalny +632 B Flash / +32 B RAM; symulator +64 B Flash / +32 B RAM.
Limity linkera: 512 KiB Flash / 128 KiB RAM. Nie zmierzono szczytowego stosu.
Oba warianty mają dokładnie te same sześć ostrzeżeń co obrazy odniesienia:
pięć nieużywanych funkcji pomocniczych oraz istniejący komentarz w `servo.c`.
Brak nowych ostrzeżeń. Ścieżki narzędzi Windows doprecyzowano wyłącznie
w tymczasowym skrypcie budowania dla WSL; nie zmieniono projektu CubeIDE.

Wyniki tej sesji: `/tmp/failsafe-arm-l0hwh5ac/normal/` i
`/tmp/failsafe-arm-l0hwh5ac/sim/`, pliki `board_test_2.elf`, `.bin`, `.map`
i `build.log`; porównanie w `results.json` w katalogu nadrzędnym.

Potwierdzono identyczne hashe 108 chronionych plików, w tym HAL, MotionFX,
serwa, startupu, konfiguracji CubeIDE, `.ioc` i skryptów linkera.
W `main.c` obszary poza USER CODE są identyczne z wersją sprzed zmian.
Nie wykonano commit, push, merge ani flashowania.

## Zmienione pliki

- `Core/Inc/app_config.h`: dwa limity świeżości i kontrola ich zakresu.
- `Core/Inc/wing_control.h`: maska błędów, snapshot i dostęp do nadzoru/resetu.
- `Core/Inc/imu.h`: istniejący reset detektora dostępny w obu trybach.
- `Core/Src/main.c`: cykliczny nadzór w ścieżce rzeczywistej.
- `Core/Src/wing_control.c`: nadzór, reset przy wejściu w awarię, blokada źródeł,
  NORMAL przy niepoprawnej regresji i komunikaty zmian stanu diagnostycznego.
- `Core/Src/obd.c`: timestamp kolejki, odrzucanie starych ramek, przekazanie czasu odbioru.
- `Core/Src/imu.c`: czyszczenie starego filtra/liczników po luce w próbkach.
- `tests/test_lap_sim.py`: możliwość kompilacji nowych prób i zachowanie porównania
  oryginalnych snapshotów przy rozszerzonej diagnostyce.
- `tests/host/test_controller.c`: sprawdzenie pełnego okrążenia bez fizycznych próbek.
- `tests/test_failsafe.py`: nowy zestaw testów hostowych.
- `tests/host/test_failsafe.c`: nowe próby aktualnego sterownika C.
- `docs/lap_sim.md`: odnośnik do aktualnego nadzoru i informacja o dostępności resetów.
- `docs/failsafe.md`: niniejszy raport i instrukcja prób.

Wcześniejsze modyfikacje i pliki wynikowe w `Debug/` nie należą do tej implementacji.

## Próby na fizycznej płytce

Próby wykonywać na stanowisku z emulatorem odpowiedzi ECU i zabezpieczonym mechanizmem.
Obserwować jednocześnie UART 460800 8N1, PWM i faktyczny ruch serwa.

1. Zbudować normalny firmware z `APP_LAP_SIM_ENABLED=0`. Przy działającym IMU
   uruchomić bez odpowiedzi OBD: NORMAL/1600 µs, bit 1, ponawiane żądania i liczone
   timeouty. Osobno sprawdzić uruchomienie bez IMU: bit 4 i brak aktywnych pozycji.
2. Podawać poprawne odpowiedzi PID 0D co około 50 ms. Przy stałych 60 km/h
   oczekiwać poprawnej regresji po około 250 ms i OPEN po kolejnym 1000 ms.
   Zatrzymać odpowiedzi przy działającym IMU. Po przekroczeniu 500 ms od ostatniej
   odebranej prędkości oczekiwać NORMAL/1600 µs i maski 18. Powtórzyć w NORMAL.
3. Uzyskać AIR BRAKE, np. podając 80, 79, 78... km/h co 50 ms, przy nieruchomym
   IMU i prędkości powyżej minimum hamowania. Po zatrzymaniu odpowiedzi oczekiwać
   NORMAL zamiast utrzymywania AIR BRAKE/1100 µs.
4. Przy aktywnym OPEN i ciągłym OBD przerwać odczyty IMU. Po przekroczeniu 100 ms
   oczekiwać NORMAL i maski 24. Powtórzyć po wykryciu zakrętu. Przywrócić poprawne
   próbki: historia ma odbudować się od nowa, a OPEN wymagać pełnego potwierdzenia.
   Odłączenie zasilania IMU może wymagać resetu płytki, ponieważ ponowna konfiguracja
   czujnika nie została dodana. Aktualną obsługę timeoutu DMA opisuje kolejny etap.
5. Przy aktualnych źródłach i OPEN podać serię odpowiedzi prędkości co 20 ms,
   tak aby osiem ostatnich próbek miało rozpiętość poniżej 250 ms. Oczekiwać
   niepoprawnej regresji, NORMAL i bitu 16. Wrócić do odstępów 50 ms i sprawdzić
   ponowne zebranie historii oraz pełne potwierdzanie OPEN.
6. Stare ramki i przepełnienie ticka są powtarzalnie sprawdzane testami hostowymi.
   Opcjonalnie na płytce zatrzymać debugger po zapisaniu ramki w kolejce,
   zwiększyć `uwTick` o ponad 500 ms i wznowić bez nowych odpowiedzi: stara ramka
   nie może odświeżyć OBD. Samo zatrzymanie rdzenia debuggerem może zatrzymać SysTick
   i nie jest wystarczającym testem upływu czasu.
7. Wgrać wariant `APP_LAP_SIM_ENABLED=1` bez OBD i fizycznego IMU. USER uruchamia
   przebieg; po końcu pozostaje NORMAL. Sprawdzić także istniejący
   `SIM_TIMING_ERROR`: przekroczenie 10 ms kończy odtwarzanie, gdy pętla odzyska czas CPU.

## Pozostałe ograniczenia

To programowa reakcja na utratę danych, wykonywana przez działającą pętlę.
Całkowite zatrzymanie CPU lub blokada UART/MotionFX nadal może utrzymywać ostatni PWM.
Watchdog, przebudowa UART i rozbudowane odzyskiwanie I2C pozostają do osobnego etapu.
Abortowanie DMA oraz walidację wyników MotionFX dodano później; ich aktualne
zachowanie opisuje [imu_motionfx.md](imu_motionfx.md).
Timestamp CAN opisuje pobranie ramki przez callback, nie sprzętowy czas jej pojawienia
się na magistrali przy zablokowanych przerwaniach.

Nie ma sprzężenia zwrotnego pozycji skrzydła. Limitów świeżości, opóźnień pętli,
ruchu serwa pod obciążeniem i działania na fizycznej płytce nie zweryfikowano w tej sesji.
NORMAL jest przyjętym stanem awaryjnym demonstratora i nie gwarantuje bezpieczeństwa
aerodynamicznego w rzeczywistym samochodzie.
