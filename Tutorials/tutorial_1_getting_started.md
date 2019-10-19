# Getting Started with Cube and System Workbench

This page walks through the process of creating microcontroller projects using the Cube and System Workbench flow.

## Overview of the Tools
### Cube
Cube is graphical tool that helps us initialize our microcontroller peripherals (e.g. ADCs, timers, communication modules). As per our configuration, Cube generates peripheral initialization code that uses ST's hardware abstraction layer (HAL) API. It is a nice convenience, but you'll find that you'll still need access to datasheets for the microcontroller and the development board you're using. For example, although you can configure timer prescalers and preload values in Cube, you'll need to use the microcontroller's datasheet to find which clock domain drives the timer. Similarly, you'll need to look at your development board's datasheet to figure out how the microcontroller pin names map to the development board pin names.

To download Cube:
1. Go to https://www.st.com/en/development-tools/stm32cubemx.html
2. At the bottom of the page click "Get Software". Login or register as needed
3. The download should automatically begin; if not, repeat the process once logged in

### System Workbench
System Workbench is where the programming happens. It is an Eclipse-based IDE, and it comes with all the tools required to program and debug STM32 microcontrollers.

To download System Workbench:
1. Go to http://www.openstm32.org and create an account
2. Go to http://www.openstm32.org/Downloading%2Bthe%2BSystem%2BWorkbench%2Bfor%2BSTM32%2Binstaller
3. Choose the appropriate installer for your system (note: the Windows version is compatible with Windows 10)

### STLink USB Driver
You will need this USB driver to program code into the microcontroller. To download it, follow the link below and click "Get Software" at the bottom.
https://www.st.com/en/development-tools/stsw-link009.html

## Creating Your First Project
This tutorial will demonstrate how to use Cube and System Workbench to create a LED blinking program for a STM32L432KC microcontroller on a Nucleo-L432KC development board.

### Cube: Selecting the Development Board
First, we want to create a new Cube project for the Nucleo-L432KC development board:
1. Open Cube and select New Project
2. In the New Project window, choose the Board Selector tab and type in "nucleo-l432kc" in the search bar
3. The panel on the right will list 1 item, double-click on it and choose "Yes" when the prompt pops up asking if you'd like to initialize all peripherals to their default mode

![Board Selection](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/1-Board-Select.jpg)

This will open up the view in the image below.

![Cube](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/2-Blank-Project.jpg)

### Cube: Project Settings
Next, we want to set up code generation & toolchain settings:
1. In the top panel, select Project -> Settings ...

![Project Settings](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/3-Project-Settings-Location.jpg)

2. In the project tab, choose an appropriate name and directory for the project. Also, change the toolchain to SW4STM32

![Project Settings 1](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/4-Project-Settings-1.jpg)

3. In the code generator tab, check the option "Generate peripheral initialization as a pair of '.c/.h' files per peripheral"

![Project Settings 2](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/5-Project-Settings-2.jpg)

4. Click OK to exit to save current settings and exit the project settings window

### Cube: Peripheral Initialization
At this point, our project is all set up and we can see Cube automatically tells us which pin is connected to the LED. Thus, this example program does not really require peripheral configuration. We can still dive a bit deeper by going to the Configuration tab and opening up the Pin Configuration window. From this window, we can change the GPIO operation frequency, default value (low or high), mode, and label. The label allows you to set an alias for the pin which you can use to make your code more readable.

![Pin Alias](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/6-LED-Pin.jpg)

![GPIO Config](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/7-GPIO-Config.jpg)

### Cube: Code Generation
We can now generate the Eclipse project and peripheral initialization code. To do this, click Project -> Generate Code. Choose the option "Open Project" to open the project in the System Workbench IDE. You may need to create a new Eclipse workspace. It is suggested that you use 1 workspace for all embedded projects. You can import and remove projects from a workspace as needed to keep it organized.

#### Known manual adjustments after generating code

CubeMX does not always behave as expected, and modifications made to the project may be overwritten when re-generating code. Below is a list of known settings that must be changed back manually after re-generating code on a project:

- Compiler optimization settings for a project are by default set to the `-O3` (optimize most) flag by Cube. We are using `-Og` (optimize for debug) for our Debug builds (we currently program the Debug build on the robot) (see [related](https://github.com/utra-robosoccer/soccer-embedded/issues/128#issuecomment-439647194)). This must be manually changed back by going to project `Properties -> C/C++ build -> Settings -> Tool settings -> <MCU ... Compiler> -> Optimization`. Do this for both the MCU GCC and MCU G++ compilers.

![Generate Code](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/8-Generate-Code.jpg)

### System Workbench: a Look at the Files
Upon opening the project in the System Workbench, we can take a look at some key files in the Src and Inc directories:
- `Src/main.c`: contains the `main` function, from which all program execution begins. We notice this function automatically calls some peripheral initialization functions before entering a while loop which user code can go into
- `Src/gpio.c`: implements functions for initializing the GPIO peripherals
- `Inc/stm32l4xx_hal_conf.h`: public interface for the entire microcontroller. This header includes a lot of others, so essentially any file which includes this has access to all the HAL APIs

![System Workbench Project Explorer](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/9-System-Workbench-Project-Explorer.jpg)

### System Workbench: Toggling the LED
Let's open up stm32l4xx_hal_gpio.h. This can be found inside the Drivers folder. Inside the Outline panel in the System Workbench, we can see all the functions declared in this file (Window -> Show View -> Outline). These declarations form the public interface for the GPIO peripherals.

![GPIO APIs](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/10-GPIO-APIs.jpg)

From these declarations, we can see that the function `HAL_GPIO_TogglePin` should work well in our program. We can see this function takes in a GPIO port and GPIO pin as arguments. Going to `Inc/main.h`, we can see that using the LD3 alias in Cube for the LED pin has generated the GPIO port and pin macros `LD3_GPIO_Port` and `LD3_Pin`, respectively.

![LD3 Aliases](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/11-GPIO-decl-in-main.jpg)

We put this together to blink the LED at 1 Hz as follows:

```C
/* USER CODE BEGIN WHILE */
while (1)
{

/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(500);
}
```

### System Workbench: Compiling & Programming
To compile, either press the hammer icon or right-click the project in the Project Explorer and choose the Build Project option.

![Building](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/12-Compiling.jpg)

After the project has built successfully, plug the development board into your computer. Then, load the executable into the microcontroller by pressing the green play button at the top panel, or choosing the Run As -> 1 Ac6 STM32 C/C++ Application option after right-clicking the project.

![Programming](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/13-Running.jpg)

After following these steps, you should see the LED on the microcontroller blinking at 1 Hz.

At this point, it is also worthwhile to mention that a full-featured debugger can be accessed through the System Workbench IDE via the debug button at the top panel, or choosing the Debug As -> 1 Ac6 STM32 C/C++ Application option after right-clicking the project.

![Debugging](https://raw.githubusercontent.com/utra-robosoccer/soccer-embedded/master/Tutorials/Images/tutorial_1/14-Debugging.jpg)