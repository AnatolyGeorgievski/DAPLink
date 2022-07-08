# Прошивка контроллера

## Сборка проекта
```
$ make DEVICE=sh29daplink
```
правила сборки проекта, см комментарии в файле Makefile

## Структура проекта
* device/<span>$</span>(DEVICE)/ -- файлы начальной загрузки проекта
* device/<span>$</span>(DEVICE)/board.h -- вся статическая инициализация периферии выносится в этот файл. Описание ног контроллера.
* device/<span>$</span>(DEVICE)/<span>$</span>(CHIPX).ld -- карта памяти для компиляции
* device/<span>$</span>(DEVICE)/startup_<span>$</span>(CHIPX).s -- начальная загрузка 
* device/<span>$</span>(DEVICE)/low_level_init.c -- низкоуровневая инициализация контроллера
* device/<span>$</span>(DEVICE).mak -- дополнительные инструкции для сборки проекта
* HAL/ -- Hardware Abstraction Layer библиотека поддержки аппаратуры
* HAL/CMSIS/include/<span>$</span>(ARCH).h -- описание архитектуры контроллера
* HAL/$(CHIPX)/ -- заголовки и коды для поддержки семейства микроконтроллеров
* r3core/ -- файлы ядра операционной системы R3v2 CMSIS-RTOS
* bin/ -- папка куда помещаются готовые прошивки 
* lst/ -- распечатки скомпилированных кодов
* dap/ -- Референсный код CMSIS-DAP с измерениями. Для функционала программатора
* usbfs/ -- библиотека USB DEVICE FS для микроконтроллеров GigaDevice GD32E10X
* Makefile -- правила сборки проекта

Copyright &copy; 2008-2022 Anatoly Georgievskii
