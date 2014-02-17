# freertos-stm32f103c8-template
=============================
### Project info and IDE

Template for a FreeRTOS project running on a STM32F103C8.
IDE is Eclipse with [GNU ARM Eclipse Plug-ins](http://gnuarmeclipse.livius.net/).

### Folder structure
Download [FreeRTOS](http://www.freertos.org/) and place it in the folder structure shown below. For the portable folder all unused files has to be deleted.
<pre>
workspace
+-- FreeRTOS
|	+-- Source
|		+-- include
|		+-- portable
|			+-- GCC
|			|	+-- ARM_CM3
|			+-- MemMang
|				+-- heap_1.c
+-- freertos-stm32f103c8-template
|	+-- include
|	+-- ldscripts
|	+-- libs
|   |   +-- CMSIS
|   |   +-- FreeRTOS
|   |   +-- StdPeriph
|   |   +-- misc
|   +-- src
</pre>
