Systemy Wbudowane - Projekt

skład sekcji:
- Rafał Czerwiński
- Krzysztof Karowiec

Opis:

Projekt został wykonany na płytce serii Nucleo-H723ZG.
Mikrokontroler wykorzystuje system operacyjny czasu rzeczywistego FreeRTOS.
Za jego pomocą obsłygiwane są dwa zadania:
- komunikacja z komputerem za pomocą portu szeregowego USART,
- pomiar napięcia z wybranego przez użytkownika kanału przetwornika.

Zaprogramowany układ rozróżnia następujące komendy:

-c - tryb "continous". Zwraca, co określoną jednoskę czasu, pomiar napięcia z aktywnego kanału przetwornika.
-s - rozpoczęcie jednorazowego pomiaru napięcia
-i - zmiana kanału na konkretny numer kanału