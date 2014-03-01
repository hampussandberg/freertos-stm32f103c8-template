# freertos-stm32f103c8-template
=============================
### Project info and IDE

Template for a FreeRTOS project running on a STM32F103C8.
IDE is Eclipse with [GNU ARM Eclipse Plug-ins](http://gnuarmeclipse.livius.net/).

### Folder structure
Download [FreeRTOS](http://www.freertos.org/) and place it in the workspace folder as shown below. The * means the file is linked to the project.
<pre>
workspace
+-- FreeRTOS
|	+-- FreeRTOSVx.x.x
|	|	+-- Source
|	|		+-- All files
+-- freertos-stm32f103c8-template
|	+-- include
|	+-- ldscripts
|	+-- libs
|   |   +-- CMSIS
|   |   +-- FreeRTOS
|	|	|	+-- All .c files *
|	|	|	+-- include
|	|	|	|	+-- All .h files *
|	|	|	+-- portable
|	|	|		+-- GCC
|	|	|		|	+-- ARM_CM3
|	|	|		|		+-- port.c *
|	|	|		|		+-- portmacro.h *
|	|	|		+-- MemMang
|	|	|			+-- heap_1.c *
|	|	+-- StdPeriph
|	|	+-- misc
|	+-- src
</pre>
