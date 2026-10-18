# Telemetria z filmu onboard — estymacja z obrazu

Źródło: `Screen_Recording_20261017_135036_YouTube(1).mp4`, nagranie ekranu dostarczone przez użytkownika (104,51 s, ok. 50 kl./s, 1896×1080).

## Pliki
- `telemetry_50hz.csv`: 5226 próbek od t=0 do t=104500 ms z krokiem 20 ms, do przygotowania wejścia symulatora STM32.
- `onboard_telemetria_10Hz.csv`: co piąta próbka (100 ms), do przeglądu i analizy.

## Pola
- `t_ms`: czas względem początku **tego klipu**, niekoniecznie początku okrążenia.
- `speed_kmh`: liczba odczytana z czerwonego, siedmiosegmentowego wskaźnika KPH; 7 nieczytelnych stanów przejściowych interpolowano.
- `g_lateral_est`: oszacowane przeciążenie boczne, dodatnie gdy czerwony punkt jest po PRAWEJ stronie środka wskaźnika.
- `g_longitudinal_est`: oszacowane przeciążenie wzdłużne, dodatnie gdy czerwony punkt jest PONIŻEJ środka (przyspieszanie); ujemne powyżej (hamowanie). Zwrot w osi pojazdu należy sprawdzić.
- `a_lateral_mps2_est`, `a_longitudinal_mps2_est`: pomnożone przez 9.80665 m/s².
- `speed_quality`: `read` lub `interpolated`.
- `g_quality`: `read` lub `extended_roi` (punkt wyszedł poza podstawowy obszar G-metera i został odnaleziony w szerszym obszarze klatki).

## Metoda estymacji
1. Rozpoznanie siedmiu segmentów cyfr w stałym obszarze na klatkach próbkowanych w 50 Hz.
2. Wykrycie centroidu czerwonej kropki G-metera (filtr koloru i komponenty połączone).
3. Przyjęto środek krzyża ok. `(1728.5, 917)` px i promień okręgu oznaczonego `1`: około `69.5 px / 1g`, w rozdzielczości 1896×1080.
4. `g_lateral_est=(dot_x-1728.5)/69.5`, `g_longitudinal_est=(dot_y-917)/69.5`.
5. W fragmentach, gdzie punkt wychodził ponad lub obok zewnętrznej ramki, poszerzono obszar detekcji i odczytano samą kropkę, bez sztucznego ograniczenia do 1.5g.

## Ważne ograniczenia
**To nie są surowe pomiary LSM6DSO ani niezależne dane telemetryczne z ECU.** Są to odczyty z wyświetlacza i odwzorowanie skali graficznej. Nie znamy metody wytworzenia nakładki (filtracja, kompensacja grawitacji, opóźnienia, kalibracja), dlatego składowe G powinny być traktowane jako **przybliżone przyspieszenia liniowe w płaszczyźnie pojazdu**, a nie jako certyfikowana prawda odniesienia.

Wartości o dużej amplitudzie poza skalą zegara (np. ok. 2g) są ekstrapolacją liniowej skali wskaźnika. Wynik ma niepewność wynikającą z rozmiaru punktu, kompresji wideo, kalibracji promienia i pochodzenia samej telemetrii. Dziesiętne miejsca są użyteczne komputerowo, ale **nie wskazują takiej dokładności pomiarowej**. Z nakładki 2D **nie da się odzyskać trzeciej (pionowej) osi ani surowych żyroskopów**.

Próbkowanie filmu w 50 Hz nie daje gwarancji 50 niezależnych aktualizacji telemetrii w sekundzie. Na potrzeby symulatora można podawać próbkowane G co 20 ms, a prędkość co 50 ms (tak jak nominalne OBD), bez udawania fizycznej komunikacji CAN czy DMA.

**MotionFX:** przyszłe `a_lin` może być porównane z tymi oszacowaniami po prawidłowym przeliczeniu osi czujnika na osie pojazdu. W aktualnym algorytmie boczny kanał G można podać do detektora zakrętów, a wzdłużny zachować do późniejszych testów; nie używać go do AIR BRAKE bez świadomej zmiany algorytmu.
