# Diagnostyka LPUART1 TX DMA

Stan: 21 października 2026, branch `feature/uart-dma`.

## Weryfikacja regeneracji CubeMX

Sprawdzono zmiany względem HEAD i kopii katalogu roboczego wykonanej po
regeneracji, przed implementacją kolejki.

| Element | Wynik |
|---|---|
| `.ioc` | BSP VCP wyłączony, LED i USER BUTTON zachowane, LPUART1 Asynchronous 460800 8N1, PA2/PA3 |
| Kolejność startu | `MX_DMA_Init()` przed `MX_LPUART1_UART_Init()` |
| UART | Jeden uchwyt `hlpuart1`; brak `BspCOMInit` i `BSP_COM_Init()` w aplikacji |
| BSP | `USE_COM_LOG=0`, `USE_BSP_COM_FEATURE=0`; obsługa przycisku i LED nadal działa przez BSP |
| Zegar LPUART1 | PCLK1, nominalnie 170 MHz; bez zmiany zegara systemowego |
| GPIO | PA2 TX i PA3 RX, AF12, push-pull; wygenerowane NOPULL i LOW pozostawiono |
| DMA TX | DMA1 Channel 2, żądanie LPUART1_TX, Memory to Peripheral, Normal, Byte/Byte, Memory Increment, priorytet LOW |
| Powiązanie | `__HAL_LINKDMA(huart, hdmatx, hdma_lpuart1_tx)` w MSP |
| IRQ DMA | `DMA1_Channel2_IRQHandler()` wywołuje `HAL_DMA_IRQHandler(&hdma_lpuart1_tx)` |
| IRQ UART | `LPUART1_IRQHandler()` wywołuje `HAL_UART_IRQHandler(&hlpuart1)` |
| Konfiguracja callbacków | `USE_HAL_UART_REGISTER_CALLBACKS=0`; HAL wywołuje aplikacyjne callbacki |

Oba przerwania są wymagane: w trybie Normal zakończenie DMA wyłącza DMAT
i włącza TCIE. Dopiero obsługa TC w LPUART1 ustawia UART READY i wywołuje
`HAL_UART_TxCpltCallback()`. Sam koniec DMA nie zwalnia kolejki aplikacji.
Wygenerowana nazwa `MX_LPUART1_UART_Init()` i wyłączenie BSP COM usuwają
wcześniejszą równoległą ścieżkę inicjalizacji.

Potwierdzono identyczność funkcji konfiguracji FDCAN, I2C, TIM2, TIM6,
TIM7, GPIO i zegara systemowego oraz callbacków przycisku i timerów względem HEAD.
Moduły sterowania, fail-safe, IMU, MotionFX, serwa i symulatora nie zostały
nadpisane przez CubeMX. Usunięty przez generator `stdio.h` z `main.h` nie był
potrzebny tym modułom: źródła korzystające z printf mają własne include.

Nie poprawiano ponownie konfiguracji CubeMX. Względem otrzymanej regeneracji
do `main.c` dopisano tylko include oraz `UARTLog_Init()` i `UARTLog_Task()`
w istniejących sekcjach USER CODE.

## Droga danych i kolejka

`printf/puts → newlib → _write → składanie wiersza → kolejka → UARTLog_Task → HAL_UART_Transmit_DMA → LPUART1 → ST-LINK VCP → PuTTY`.

Nowe `_write()` jest mocnym symbolem w `uart_log.c` i zastępuje słabą
implementację z `syscalls.c`. Nie zmieniono `syscalls.c` ani BSP.
Sprawdzono mapy obu obrazów: `_write` pochodzi z `uart_log.o`, jest obecne
`HAL_UART_Transmit_DMA`; brak używanego `__io_putchar`, `hcom_uart` i
blokującego `HAL_UART_Transmit` w końcowych symbolach.

`UARTLog_Init()` jest wywołane po inicjalizacji LPUART1, przed pierwszym printf.
Ustawia stdout/stderr na `_IONBF`, dzięki czemu biblioteka nie przetrzymuje
tekstu w dodatkowym buforze stdio. Transport nie korzysta z malloc.

Bufor składa komunikat do `\n`, zachowując wszystkie znaki, w tym `\r`.
To konieczne, ponieważ SIM, detektor zakrętu i funkcje formatowania liczb
budują pojedynczy rekord przez kilka wywołań printf. Komunikatem dla kolejki
jest cały wiersz, a nie pojedynczy fragment przekazany do `_write()`.

- 12 miejsc po 544 B, łącznie 6528 B na gotowe rekordy, wliczając aktywny DMA.
- Osobny bufor składania 544 B.
- `diag_line[512]` może zawierać 511 znaków, a drukowanie dodaje CRLF:
  potrzebne jest do 513 B. 544 B daje mały zapas z zaokrągleniem do 32 B.
- Uruchomienie emituje osiem wierszy w trybie normalnym i dziewięć w trybie
  symulatora, zanim pętla regularnie obsługuje kolejkę. 12 miejsc zapewnia zapas
  także na pierwsze zdarzenia. Test odtwarza rzeczywistą sekcję startową main.c.
- Gotowy rekord jest kopiowany do własnego miejsca. Nadpisanie `diag_line`
  przy przygotowywaniu IMU/FX nie zmienia danych aktywnego DMA.
- Gdy kolejka jest pełna, nowy cały wiersz jest pomijany. Starsze, już przyjęte
  rekordy pozostają w kolejności FIFO. `dropped_count` wzrasta raz na wiersz.
- Wiersz dłuższy niż 544 B jest odrzucany w całości, z pominięciem reszty
  do najbliższego `\n`. Nie jest wysyłany jego początek ani końcówka.
- Niezakończony wiersz czeka na `\n`. Nie dodano automatycznego flushowania
  fragmentów, ponieważ utrudniałoby gwarancję pomijania całych rekordów.

`_write()` zwraca przyjętą długość również przy świadomym pominięciu logu;
utrata jest liczona osobno. Nie zamienia przepełnienia w oczekiwanie ani
trwały błąd stdio. Niepoprawny deskryptor lub argument zwraca -1 z errno.

## Synchronizacja, zakończenie i błędy

W obecnej aplikacji printf jest wykonywany w pętli/startupie, nie w ISR.
Bufor składania ma jednego producenta. Callbacki nie formatują i nie drukują
tekstu. Nie dodano obietnicy reentrant printf z wielu kontekstów.

Przekazanie kompletnego rekordu do kolejki, uruchomienie DMA i zmiany
wspólnego stanu są chronione krótkimi sekcjami krytycznymi. Poprzedni PRIMASK
jest odtwarzany; przerwania nie są bezwarunkowo włączane. Sekcja kopiowania
obejmuje najwyżej jeden wiersz 544 B.

Aktywny rekord pozostaje zajęty do potwierdzonego zakończenia UART.
Callback zakończenia sprawdza właściwy uchwyt, aktywność, brak odzyskiwania,
READY UART/DMA, blokadę HAL, wyłączenie DMAT i TCIE. Powtórzony callback bez
aktywnego transferu oraz spóźniony callback przy nowym BUSY TX nie zwalniają
kolejnego rekordu. Callback jedynie zwalnia miejsce; następny transfer zleca
pętla główna.

`HAL_BUSY` podczas zlecania zachowuje rekord i zwiększa `busy_count`.
Ponowienie jest możliwe w kolejnym obiegu po odzyskaniu gotowości, bez pętli
oczekiwania. Zajętość rozpoznana jeszcze przed wywołaniem HAL nie jest błędem.

Nieudany start lub callback rzeczywistego błędu ustawia odzyskiwanie
i zwiększa `error_count`. Zwykły spóźniony callback błędu z ErrorCode=NONE
jest ignorowany. Aktywny bufor pozostaje chroniony. Pętla zleca
`HAL_UART_AbortTransmit_IT()` i dopuszcza następne DMA dopiero po rzeczywistej
gotowości. Obsłużono także pozostawiony BUSY DMA przy gotowym UART i wyłączonym
DMAT, przez nieblokujące `HAL_DMA_Abort_IT()`.

Zleconego, jeszcze niezakończonego abortowania UART nie ponawia się w każdym
obiegu. Odrzucona próba może być ponowiona. Dopiero po potwierdzeniu gotowości
nieudany aktywny rekord jest usuwany i liczony jako pominięty. Błąd bez aktywnego
transferu nie usuwa niesłanego rekordu. Pozostała kolejka zachowuje kolejność.

Przy błędzie sprzętowym część wadliwego rekordu mogła już zostać wysłana;
nie można cofnąć tych bajtów. Gwarancja całego wiersza dotyczy przepełnienia
i odrzucania za długich komunikatów przed transmisją, nie awarii na przewodzie.

W `main.c` `UARTLog_Task()` jest przed `__WFI()`. Zakończenie UART wybudza
rdzeń. Przy zdarzeniu pomiędzy Task i WFI kolejny SysTick nadal umożliwia
obsługę; program nie oczekuje aktywnie na zwolnienie UART. Nie przeniesiono
printf ani logiki sterowania do przerwań. Wstępne rekordy wychodzą po dojściu
do obsługi kolejki w pętli.

## Diagnostyka bez zmiany formatów

Nie zmieniono tekstu ani wywołań VEH, IMU, FX, SIM, SIM_START, SIM_END,
SIM_TIMING_ERROR, FAILSAFE i SERVO STATE. Nie dodano okresowego dodatkowego rekordu.

`UARTLog_GetDiagSnapshot()` udostępnia liczniki `dropped_count`, `error_count`,
`busy_count` oraz `queued`, `active`, `recovering`. W firmware bez wywołującego
linker może usunąć samą funkcję snapshotu. Liczniki i stan używane przez transport
pozostają dostępne w debuggerze jako zmienne statyczne `uart_log.c`, np.
`'uart_log.c'::dropped_count`, `'uart_log.c'::queued` i
`'uart_log.c'::recovering`. Nie należy wywoływać printf z callbacków diagnostycznych.

## Wyniki weryfikacji

**95 testów PASS, 0 FAIL:** wszystkie wcześniejsze 67 oraz 28 nowych.
Testy kompilują rzeczywiste moduły C, z atrapami HAL. Użyto Werror, UBSan
i float-cast-overflow. Aplikacyjna pętla, callbacki timerów/przycisku i blok
startowy są pobierane z aktualnego main.c.

Nowe próby sprawdzają fragmenty printf, kopię bufora, kolejność i zawijanie
FIFO, brak podwójnego DMA, przepełnienie i ponowne przyjmowanie rekordów,
za długie wiersze, limit 544 B, HAL_BUSY, stany sprzętu, oddzielny koniec DMA
i UART, błędy startu i callbacków, nieudane/pending abortowanie, osierocony
DMA, spóźnione/powtórzone callbacki, inny UART, PRIMASK, callback tuż przed
sekcją krytyczną, argumenty _write oraz przepełnienie ticka modelu transmisji.

Pełne strumienie normalnej pracy i całego przejazdu, wraz ze startupem,
porównano bajt po bajcie z bezpośrednim stdout: zgodne także CRLF, bez
pominiętych rekordów. Osobne próby zatrzymanego UART kończą przejazd w NORMAL
bez SIM_TIMING_ERROR i nie zatrzymują rzeczywistej logiki sterowania.
Pozostały oryginalny wzorzec 1001 snapshotów sterownika oraz wszystkie testy
fail-safe i MotionFX.

Hostowy printf jest przekierowany do rzeczywistego `_write()` przez adapter
vsnprintf. Adapter uwzględnia również `__printf_chk` używany przez glibc
z FORTIFY. To kontrola transportu i formatów z rzeczywistym kodem aplikacji,
nie pomiar newlib ani sprzętowego UART. W testach MotionFX funkcje fuzji ST
są zastąpione kontrolowanymi wynikami; prawdziwa biblioteka jest użyta
w obu kompilacjach ARM.

```bash
python3 -B -m unittest discover -s tests -p 'test_*.py' -v
python3 -B tools/csv_to_lap_data.py --check
git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check -- . ':!Debug/**' ':!docs/wklad_wlasny_promotor.tex'
```

## Kompilacja i pamięć

Skompilowano i zlinkowano oba warianty z oryginalnym HAL, BSP, startupem,
linkerem oraz rzeczywistym MotionFX_CM4F_wc32_ot_hard.a. Narzędzia GNU Tools
for STM32 14.3.1, Cortex-M4F hard float, Debug -O0. Obrazy odniesienia są
kopią dostarczonego kodu po CubeMX, przed dodaniem transportu, nie wcześniejszym HEAD.

| Obraz | Flash BIN | Statyczny RAM (.data + .bss) | RAM z rezerwą heap/stack |
|---|---:|---:|---:|
| Po CubeMX, bez kolejki, normalny | 100476 B | 8856 B | 10396 B |
| TX DMA, normalny | 103052 B | 15976 B | 17516 B |
| Po CubeMX, bez kolejki, symulator | 156980 B | 8348 B | 9884 B |
| TX DMA, symulator | 159556 B | 15468 B | 17004 B |

Przyrost transportu w każdym wariancie: **2576 B Flash i 7120 B statycznego RAM**.
Limity linkera: 512 KiB Flash, 128 KiB RAM. Szczytowego użycia stosu nie zmierzono.

Kompilacje mają sześć dotychczasowych ostrzeżeń: pięć nieużywanych funkcji
IMU/OBD i istniejący komentarz w servo.c. Brak nowych ostrzeżeń.

Końcowe ELF/BIN/MAP, symbole i logi są odseparowane od Debug:
`/tmp/uart-dma-arm-zztiscjj/normal/` oraz `/tmp/uart-dma-arm-zztiscjj/sim/`.
Obrazy odniesienia: `/tmp/uart-dma-arm-qlinxywn/baseline_normal/` i
`baseline_sim/`. Są to katalogi tymczasowe środowiska WSL.
Normalne budowanie w CubeIDE powinno wykryć nowy plik w Core/Src;
nie edytowano generowanych plików makefile w Debug.

## Pliki

W tej implementacji dodano:

- `Core/Inc/uart_log.h`
- `Core/Src/uart_log.c`
- `tests/host/test_uart_log.c`
- `tests/host/uart_capture.c`
- `tests/test_uart_log.py`
- `docs/uart_dma.md`

Rozszerzono:

- `Core/Src/main.c`: wyłącznie trzy dopiski w USER CODE ponad dostarczony CubeMX;
- `tests/host/main.h`, `tests/host/hardware.c`: modele UART/DMA i przerwań;
- `tests/test_lap_sim.py`: kompilacja transportu, pobieranie startupu,
  opcjonalne skierowanie printf do kolejki.

Zmiany `.ioc`, BSP conf, MSP, IRQ, main.h, metadanych i nowe nagłówki LL
pochodzące z wcześniejszej regeneracji CubeMX zostały zachowane. Nie edytowano
HAL/BSP/MotionFX, sterowania, IMU, serwa, symulatora ani jego telemetrii.
`docs/wklad_wlasny_promotor.tex` oraz wcześniejsze artefakty Debug są poza zakresem.
Nie wykonano commit, push, merge ani flashowania.

## Sprawdzenie na Nucleo

1. Odświeżyć projekt w CubeIDE i zbudować wariant normalny. Sprawdzić, czy
   `uart_log.c` jest kompilowany. Po samodzielnym wgraniu otworzyć port ST-LINK
   VCP w PuTTY: 460800, 8 bitów, bez parzystości, 1 stop, flow control None.
   Sprawdzić komplet startupu i rekordy VEH/IMU/FX. Brak danych OBD ma nadal
   powodować dotychczasowy NORMAL, bez zależności od transmisji UART.
2. Przy prawidłowych źródłach obserwować stan i PWM. W debuggerze sprawdzić
   `hlpuart1.hdmatx == &hdma_lpuart1_tx`, `hdma_lpuart1_tx.Instance == DMA1_Channel2`
   oraz liczniki w uart_log.c. Podczas zwykłej pracy oczekiwać stabilnego
   dropped_count=0 i error_count=0, kolejki opróżniającej się między seriami.
3. Krótkimi breakpointami sprawdzić kolejno `_write`, HAL_UART_Transmit_DMA,
   DMA1_Channel2_IRQHandler, LPUART1_IRQHandler i HAL_UART_TxCpltCallback.
   Do pomiaru czasu pętli użyć śladu/analizatora, nie czasu spędzonego na breakpointach.
   Na PA2 można niezależnie sprawdzić transmisję 460800 i ramkę 8N1.
4. Sprawdzić pełne odtwarzanie bez OBD/IMU w wariancie APP_LAP_SIM_ENABLED=1.
   USER uruchamia odtwarzanie, SIM_END powinno wystąpić przy 104500 ms,
   a końcowy stan i PWM to NORMAL/1600 us. Nie powinno być SIM_TIMING_ERROR.
   Po zakończeniu kolejka musi mieć czas na wysłanie ostatnich rekordów.
5. Na stanowisku tymczasowo wyłączyć IRQ DMA1 Channel 2 i LPUART1, pozostawiając
   SysTick oraz FDCAN/I2C i timery aktywne. Po zapełnieniu kolejki dropped_count
   ma rosnąć, ale pętla i sterowanie mają nadal działać. Nie może być kolejnego
   aktywnego DMA na tym samym buforze. Po przywróceniu IRQ oczekiwać zakończenia
   oczekującego transferu, opróżnienia FIFO i wznowienia przyjmowania logów.
6. Opcjonalnie w debuggerze wymusić HAL_UART_ErrorCallback z ustawionym
   HAL_UART_ERROR_DMA dla hlpuart1, następnie wznowić pętlę. To wstrzyknięcie
   błędu aplikacyjnego, nie fizyczny błąd kanału. Oczekiwać error_count,
   abortowania, zwolnienia bufora dopiero po gotowości i dalszej transmisji.
   Jeden uszkodzony rekord może być niepełny na przewodzie i będzie policzony
   jako pominięty. Nie oczekiwać fałszywego sukcesu przy trwale zajętym HAL.

## Pozostałe ograniczenia

Nie wykonano nowych prób fizycznych ani pomiaru jittera, czasu sekcji krytycznych
i szczytowego stosu. DMA usuwa czekanie na wysłanie bajtów; formatowanie printf
i kopiowanie nadal kosztują czas CPU. Maksymalny długotrwały strumień ogranicza
przepustowość 460800 8N1, a nadmiar rekordów będzie pomijany.

Nie dodano osobnego timeoutu UART, watchdog-a ani resetu peryferium. Trwale
brakujący callback/IRQ może zatrzymać diagnostykę i zapełnić kolejkę, ale nie
powoduje blokującego oczekiwania pętli. Inicjalizacja i całkowite zatrzymanie CPU
nadal mają swoje wcześniejsze ograniczenia. Nieważność lub utrata UART nie
wybiera pozycji skrzydła; decyzje pozostają w dotychczasowym sterowniku.
