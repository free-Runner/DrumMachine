This project was built on Linux.   
To set up in Windows, one would have to manually configure all the libraries.  
To test in Windows, a binary is provided in the build folder.  


Code is mainly placed in main.c & stm32f4xx_it.c  
Audio.h & .c are external libraries used to play mp3.  
stm32f4_discovery_LIS302dl.h & .c provide easy access to accelerometer.  
mp3_data.c has hex representation of mp3 samples. bin2hex is Perl script used for conversion.  

Includes CMSIS library. See this [guide](https://github.com/free-Runner/STM32f4-Discovery-QuickStart) to setup the STM board.


