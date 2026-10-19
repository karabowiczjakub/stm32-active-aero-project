# Stabilizacja IMU DMA i diagnostyki MotionFX

## Zakres

Zmiany rozszerzają istniejące moduły. Nie zmieniono algorytmu skrzydła, regresji,
progów, histerez, detektora zakrętów, fail-safe OBD/IMU, PWM, konfiguracji
peryferiów ani kodu biblioteki ST. MotionFX pozostaje wyłącznie diagnostyczny.

## Timeout DMA

`APP_IMU_DMA_TIMEOUT_MS` w `app_config.h` wynosi domyślnie **10 ms na etap**.
Każdy odczyt statusu (1 bajt) i danych (12 bajtów) dostaje osobny timestamp startu.
Przy nominalnym zlecaniu pomiarów co 20 ms jest to margines na krótkie transfery,
a zarazem pozwala reagować przed limitem świeżości IMU 100 ms.
Timeout zachodzi przy czasie trwania **co najmniej 10 ms**. Wartość jest
konfigurowalna; zero i wartości co najmniej połowy zakresu ticka są odrzucane.
Nie zmieniono prędkości I2C ani okresów timerów. Rzeczywiste opóźnienia wymagają
jeszcze pomiaru na Nucleo.

`IMU_Task()` sprawdza czas bez oczekiwania i bez `HAL_Delay()`.
Przy pierwszym przekroczeniu limitu danego transferu zwiększa licznik timeoutów,
włącza `imu_dma_recovering`, unieważnia oczekującą próbkę i etap odczytu.
Nie zwalnia samego `imu_dma_busy` i nie uruchamia nowego DMA na aktywnym interfejsie.
Gotowa próbka, która zestarzała się o ponad 100 ms przed jej przetworzeniem,
również jest odrzucana przed detektorem zakrętów i MotionFX.

## Przerwanie i ponowienie

W użytej wersji STM32G4 HAL funkcja nazywa się **`HAL_I2C_Master_Abort_IT()`**,
a nie `HAL_I2C_Abort_IT()`. Obsługuje tryb master oraz memory, używany przez
`HAL_I2C_Mem_Read_DMA()`. Jej sukces oznacza zlecenie STOP i obsługi przerwania,
a nie natychmiastową gotowość sprzętu.

Sprawdzono lokalne implementacje:

- `stm32g4xx_hal_i2c.c`: `HAL_I2C_Master_Abort_IT()`, `I2C_ITMasterCplt()`,
  `I2C_ITError()`, `I2C_TreatErrorCallback()` i `I2C_DMAAbort()`;
- `stm32g4xx_hal_dma.c`: `HAL_DMA_Abort_IT()` i obsługę zakończenia DMA.

Procedura odzyskiwania:

1. Dla zajętego I2C zleca `HAL_I2C_Master_Abort_IT()` i czeka na dalsze obiegi pętli.
2. `HAL_I2C_AbortCpltCallback()` odrzuca pozostałości próbki, ale sam nie dopuszcza
   nowego transferu. Zwykły callback błędu I2C także przełącza moduł w odzyskiwanie.
3. Jeśli I2C jest już READY, lecz DMA pozostało BUSY, używa `HAL_DMA_Abort_IT()`.
   To obsługuje także pozostawiony kanał DMA po błędzie w fazie adresowania.
   W tej wersji HAL przerwanie DMA wykonuje ograniczoną liczbę zapisów rejestrów
   i może synchronicznie wywołać callback; nie ma w nim pętli oczekiwania.
4. Ponowienie jest dozwolone dopiero, gdy HAL I2C jest READY i odblokowany,
   HAL DMA jest READY, sprzętowa flaga BUSY I2C jest skasowana oraz żądania RX DMA
   są wyłączone. Następny odczyt zaczyna się od statusu, nie od porzuconych danych.
5. Odrzucone zlecenie abortowania jest liczone. Kolejna próba następuje najwcześniej
   po 10 ms, zgodnie ze stanem HAL. Podczas `HAL_I2C_STATE_ABORT` nie wysyła się
   kolejnych abortów na trwającą procedurę. Jeżeli sprzęt później rzeczywiście
   przejdzie do gotowości, odczyty mogą wrócić również bez aplikacyjnego callbacku.

Jeśli HAL lub magistrala nie odzyskają gotowości, moduł pozostaje w odzyskiwaniu.
Nie ma fałszywego komunikatu sukcesu, resetu MCU, deinit/init peryferiów ani
przepisywania stanów HAL ręcznie. Dotychczasowy nadzór IMU po 100 ms sam wymusza
NORMAL. Timeout komunikacji nie zawiera drugiej logiki wyboru pozycji skrzydła.

## Callbacki i współbieżność

Start transferu i przejście do odzyskiwania są objęte krótkimi sekcjami z zachowaniem
i odtworzeniem PRIMASK. Nie obejmują przetwarzania MotionFX, UART ani oczekiwania
na sprzęt. Callback, który nadejdzie tuż przed sekcją, kończy aktualizację przed
ponownym sprawdzeniem stanu przez pętlę. Callback oczekujący podczas sekcji zobaczy
już znacznik odzyskiwania.

Callback zakończenia odczytu jest ignorowany, gdy nie ma aktywnego odczytu,
trwa odzyskiwanie, HAL nie potwierdza zakończenia albo czas przekroczył deadline.
Po timeoutcie nie publikuje danych, nie odświeża timestampu próbki i nie uruchamia
następnego etapu. Powtórzony callback jest ignorowany. Spóźniony callback podczas
nowego transferu nie może zakończyć go przy stanie HAL BUSY.
Ponowne użycie bufora jest możliwe dopiero po zakończeniu lub przerwaniu poprzedniego
transferu; HAL przy abortowaniu DMA kasuje jego flagi i wyłącza przerwania kanału.

Timestampy są porównywane przez odejmowanie `uint32_t`. Testy obejmują przepełnienie
`HAL_GetTick()`, ukończenie przed maskowaniem IRQ oraz odtworzenie wcześniejszego PRIMASK.

## Walidacja MotionFX

`APP_MOTIONFX_MAX_AGE_MS` wynosi domyślnie **100 ms**, czyli pięć nominalnych
okresów próbek po 20 ms. Wynik o wieku równym limitowi jest aktualny;
po przekroczeniu limitu API zwraca nieważność, również bez nowego wywołania Process.

Sprawdzane są wszystkie publikowane składowe: yaw, pitch, roll, grawitacja XYZ,
przyspieszenie liniowe XYZ, żyroskop XYZ i jego całka Y. Muszą być skończone oraz
reprezentowalne po istniejącym przeliczeniu na tysięczne w `int32_t`.
Sprawdzany jest także skończony wynik mnożenia przez 1000. Porównanie z granicami
int32 uwzględnia zaokrąglenie `INT32_MAX` we float. Nie dodano progów fizycznych,
kalibracji ani testów nieużywanych pól heading/quaternion.

`MotionFxApp_Process()` odrzuca nieskończone wejścia przed wywołaniem fuzji.
Zachowano dotychczasową kontrolę delta czasu 0 lub ponad 100 ms.
Nowy wynik powstaje w lokalnej strukturze i jest publikowany dopiero po pełnej
walidacji, w tym jego wieku. Błąd ustawia `valid=0`, zachowując timestamp ostatniego
poprawnego wyniku. Również całka żyroskopu jest zatwierdzana dopiero z poprawnym
wynikiem, dzięki czemu błąd liczbowy nie zatruwa kolejnych aktualizacji.

`MotionFxApp_GetLatest()` ponownie sprawdza ważność i wiek. Przy nieważności
zwraca 0 i kopiuje wynik z `valid=0` oraz zachowanym timestampem ostatniej poprawnej
publikacji. Każdy nowy poprawny wynik automatycznie przywraca ważność.
Pojedynczy błąd nie reinicjalizuje biblioteki. Nieudana jawna inicjalizacja
unieważnia poprzedni wynik. Parametry fuzji, osie i tryb 6X pozostają bez zmian.

## Logger

`DiagLog_Task()` niezależnie sprawdza składowe i wiek przed konwersjami.
Niepoprawny wynik jest zerowany i wypisywany w dotychczasowym formacie jako
`FX ... valid=0 age_ms=-1 ...`; żaden NaN, Infinity ani wynik poza zakresem nie
trafia do konwersji całkowitoliczbowej. `MotionFxApp_PrintDebug()` korzysta ze
zweryfikowanego API i pomija nieważne dane.

Do końca istniejącego rekordu IMU dodano tylko:

- `dma_timeout`: liczba transferów, które przekroczyły limit;
- `dma_abort_err`: liczba odrzuconych prób przerwania;
- `dma_recovering`: oczekiwanie na potwierdzoną gotowość interfejsu.

Formaty VEH, FX i SIM pozostają takie same. Nie dodano logowania w każdej iteracji
ani zmiany działania UART. Nieważność MotionFX nie jest przesłanką fail-safe skrzydła.

## Testy

**67 PASS, 0 FAIL:** 30 nowych testów i wszystkie 37 wcześniejszych.
Kompilowany jest rzeczywisty kod C modułów oraz treść pętli aplikacji.
GCC hosta używa `-Wall -Wextra -Werror`, UBSan oraz `float-cast-overflow`;
pozostają wcześniejsze wyłączenia dla starych nieużywanych funkcji i komentarza serwa.

Testy DMA obejmują oba etapy, prawidłowy odczyt, callback błędu, brak callbacku,
granicę timeoutu, udane i nieudane abortowanie, ponowienie po odzyskaniu,
pozostawiony kanał DMA, brak callbacku abortowania, stan sprzętu mimo gotowości HAL,
spóźnione i powtórzone callbacki, wyścig na granicy sekcji krytycznej,
licznik timeoutów, starą gotową próbkę, przepełnienie ticka i reakcję istniejącego fail-safe.

Testy MotionFX wykonują prawdziwą warstwę `motion_fx_app.c`, ale funkcje biblioteki ST
są zastąpione w `tests/host/motion_fx.c` kontrolowanymi wynikami.
Nie są to testy obliczeń prawdziwej biblioteki na mikrokontrolerze.
Sprawdzono NaN i oba znaki Infinity w każdym publikowanym kanale wyjściowym,
wartości skończone poza zakresem loggera, brak zatrucia całki, wiek wyniku,
niepoprawny delta czas, odzyskanie ważności, inicjalizację i przepełnienie ticka.

Logger dostał także przez podmianę samego API 78 niepoprawnych zestawów danych:
13 składowych razy 6 wartości granicznych lub niepoprawnych. Wszystkie rekordy
miały `valid=0`, bez niedozwolonych konwersji i bez NaN/Inf w tekście.
Test nieważnego MotionFX przy zdrowym OBD i IMU pozostawia skrzydło w OPEN.

Zachowano porównanie 1001 snapshotów poprawnego przebiegu sterowania z oryginalnym
SHA256. Testy całego okrążenia obejmują także odłączone IMU oraz niedokończone
abortowanie DMA z aktywnymi zleceniami TIM7. Symulator kończy przebieg w NORMAL,
bez błędu czasu, fizycznego OBD i poprawnych próbek rzeczywistego IMU.
Dotychczasowe USER BUTTON, SIM_TIMING_ERROR i reset końca okrążenia nadal przechodzą testy.

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
python3 -B tools/csv_to_lap_data.py --check
git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check -- Core tests docs
```

## Kompilacja i pamięć

Skompilowano i zlinkowano oba warianty GNU Tools for STM32 14.3.1,
Cortex-M4F, hard float, Debug -O0, po 39 źródeł z rzeczywistą biblioteką MotionFX.
Obrazy odniesienia pochodzą z katalogu roboczego sprzed tego etapu, z istniejącym
już fail-safe; nie z wcześniejszego HEAD. Nie nadpisywano `Debug/`.

| Obraz | Flash BIN | Statyczny RAM | RAM z rezerwą heap/stack |
|---|---:|---:|---:|
| Normalny przed zmianą | 96 032 B | 8 772 B | 10 312 B |
| Normalny po zmianie | 98 152 B | 8 780 B | 10 320 B |
| Symulator przed zmianą | 153 152 B | 8 264 B | 9 800 B |
| Symulator po zmianie | 154 656 B | 8 280 B | 9 816 B |

Przyrost: normalny +2120 B Flash / +8 B RAM; symulator +1504 B Flash / +16 B RAM.
Limity linkera: 512 KiB Flash i 128 KiB RAM. Pozostało dokładnie sześć wcześniejszych
ostrzeżeń, bez nowych: pięć nieużywanych funkcji pomocniczych oraz stary komentarz
w `servo.c`. Kompilator podaje 136 B lokalnego stosu dla MotionFxApp_Process,
bez stosu wywoływanych funkcji i biblioteki ST. Szczytowego stosu nie zmierzono.

Aktualne ELF/BIN/MAP i logi: `/tmp/imu-fx-final-arm-eee_9aba/normal/` oraz
`/tmp/imu-fx-final-arm-eee_9aba/sim/`. Obrazy odniesienia i ich logi są w
`/tmp/imu-fx-arm-6p2wc0ed/baseline_normal/` i `baseline_sim/`.

Sprawdzenie zmienionych źródeł z zachowaniem CRLF i nowych plików: PASS.
Zwykłe `git diff --check` całego katalogu nadal zwraca błąd z powodu CRLF
oraz wcześniej zmodyfikowanych artefaktów `Debug/`; nie poprawiano ich.
Potwierdzono hashe 114 chronionych plików, w tym HAL, biblioteki MotionFX,
serwa, sterowania skrzydłem, OBD, symulatora, `.ioc`, startupu i konfiguracji IDE.
Nie wykonano commit, push, merge ani flashowania.

## Pliki tego etapu

Zmodyfikowane:

- `Core/Inc/app_config.h`
- `Core/Inc/imu.h`
- `Core/Inc/motion_fx_app.h`
- `Core/Src/imu.c`
- `Core/Src/motion_fx_app.c`
- `Core/Src/diag_log.c`
- `tests/host/main.h`
- `tests/host/hardware.c`
- `tests/test_lap_sim.py`
- `docs/failsafe.md`

Nowe:

- `tests/host/motion_fx.c`
- `tests/host/test_imu_fx.c`
- `tests/test_imu_fx.py`
- `docs/imu_motionfx.md`

Wcześniejsze zmiany fail-safe w pozostałych plikach i artefakty Debug nie należą
do tej implementacji.

## Próby na Nucleo

Próby wykonywać na stanowisku, obserwując UART 460800 8N1 oraz rzeczywisty PWM.

1. Z podłączonym IMU sprawdzić regularne próbki i `FX valid=1` po rozruchu.
   `dma_timeout`, `dma_abort_err` i `dma_recovering` powinny pozostać zerowe.
2. Debuggerem tymczasowo zablokować IRQ DMA1_Channel1, pozostawiając SysTick i
   I2C1_EV/ER aktywne. Po 10 ms transfer powinien wejść w odzyskiwanie; licznik
   timeoutów rośnie po kolejnych porzuconych transferach. Po ponad 100 ms bez
   poprawnych próbek dotychczasowy fail-safe powinien ustawić NORMAL.
   Odblokować IRQ i sprawdzić powrót rzeczywistych próbek oraz odbudowę sterowania.
3. Powtórzyć z niedostępnym również IRQ I2C1_EV: oczekiwane oczekiwanie w ABORT,
   `dma_recovering=1` i brak nowego DMA. Po odblokowaniu przerwań i rzeczywistym
   zakończeniu STOP odczyty mogą wrócić. Nie maskować SysTick podczas pomiaru timeoutu.
4. Zatrzymać debugger tuż po `MotionFX_update()`, przed publikacją lokalnego wyniku.
   Wpisać do `mfx_output.rotation[0]` bitowy wzorzec NaN `0x7fc00000`, następnie
   osobno +Inf `0x7f800000` i -Inf `0xff800000`. Oczekiwane `FX valid=0`, zachowany
   timestamp ostatniego dobrego wyniku i bezpieczne zerowe pola logu. Po nowym
   poprawnym wyniku ważność ma wrócić bez ponownej inicjalizacji biblioteki.
5. Na wejściu do `MotionFxApp_GetLatest()` można debuggerem ustawić timestamp
   ostatniego wyniku na `uwTick - 101U`. Oczekiwać nieważności mimo braku nowego
   Process. Przy nadal aktualnym OBD/IMU sama nieważność FX nie zmienia skrzydła.
   Samo zatrzymanie CPU debuggerem może zatrzymać SysTick i nie bada upływu czasu.
6. Uruchomić wariant symulatora bez IMU/OBD. USER ma rozpocząć pełne okrążenie,
   a koniec przywrócić NORMAL. Sprawdzić rzeczywisty jitter z aktywnym logowaniem
   i powrót do pracy po odblokowaniu przerwań.

## Pozostałe ograniczenia

Nie wykonano prób fizycznej płytki ani rzeczywistego pomiaru jittera i czasu sekcji
krytycznych. Testy hostowe modelują HAL, nie zastępują sprawdzenia STOP, flag DMA
i elektrycznego stanu magistrali na Nucleo. Trwale zajęta magistrala, brak przerwań
lub HAL pozostający w ABORT nie są naprawiane resetem peryferium; moduł pozostaje
zablokowany do odzyskania potwierdzonej gotowości. Ewentualny bus-clear lub ponowna
inicjalizacja wymagają osobnego etapu i prób sprzętowych.

Walidacja gwarantuje sprawdzone warunki liczbowe i świeżość diagnostyki, nie fizyczną
poprawność oszacowania orientacji. Nie dodano watchdog-a ani przebudowy UART.
Całkowite zatrzymanie pętli lub samej biblioteki nadal wymaga niezależnego nadzoru.
NORMAL pozostaje stanem awaryjnym demonstratora, bez gwarancji bezpieczeństwa
aerodynamicznego samochodu.
