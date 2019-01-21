# Style Guide - C/C++

This document provides guidelines to improve code consistency across files worked on by different people on the team. We write new code largely in C++, but this guide also applies to any code we write in C.

## Inspiration

A few existing well-known C++ guidelines we looked at as a base for this style guide are listed here. Where this style guide does not cover a topic, the following places can be looked at for guidance:

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md)

## Organization

### Directory layout

In general, files that are not project-specific or part of a library should exist in `Common`, under one of the subdirectories:

```
Common
  |-- app       // Application-level code that does not belong to a specific class.
  |-- component // Source files for classes, which should talk to hardware through a mockable interface. Create a new directory to hold source files for any new classes.
  |-- hardware  // Implementation of mockable interfaces, which directly talk to hardware and OS drivers.
  |-- include   // Header files for any of the source files in the Common directory.
  `-- test      // Source files that contain unit tests.
```

### File layout

All new source/header files should be based on the templates in the [Templates](Templates) folder. This keeps the code structure consistent, and ensures the Doxygen fields are filled out.

### Include guards

PROPOSED

Header files must have `#define` guards, of the format `<PROJECT>_<PATH>_<FILE>_H`. This is based on Google's [style guide](https://google.github.io/styleguide/cppguide.html#The__define_Guard). Including the `PROJECT` string in the guard reduces collision with different projects which may include code from each other and have files of the same name. An example guard is as follows:

```C
#ifndef SOCCER_EMBEDDED_COMMON_INCLUDE_OS_INTERFACE_H
#define SOCCER_EMBEDDED_COMMON_INCLUDE_OS_INTERFACE_H

...

#endif  /* SOCCER_EMBEDDED_COMMON_INCLUDE_OS_INTERFACE_H */
```

TODO: update template and example

### Include ordering

We generally follow Google's [style guide](https://google.github.io/styleguide/cppguide.html#Names_and_Order_of_Includes) on order of includes. The corresponding header file for the source file should be included first; then standard C/C++ libraries; then external libraries; then other files specific to the Robot project. Including the files in this order helps avoid dependencies being brought in indirectly, which can cause the linker to behave inconsistently when re-building the project.

An example:

```C++
/**
 * @file shapes/circle.cpp
 */

// The corresponding header file that this source file implements
#include "shapes/circle.h"

// C standard headers that this file directly uses
#include <stdint.h>

// C++ standard headers that this file directly uses
#include <initializer_list>

// External dependencies that are not standard C/C++ headers
#include "lib/coolgraphicslibrary.h"

// Other dependencies that are part of the project
#include "style/colors.h"
#include "style/text.h"
```

### Comments

1. Each file must contain a Doxygen-style comment at the top, following the format in the [Templates](Templates) folder. Header files should define a  Doxygen group, using `@defgroup`, indicating some module of code. Source files belonging to a module should include an `@ingroup` tag.
2. Functions must contain a Doxygen-style comment above them. In header (`.h`) files, function comments should include the Doxygen tags `@brief`, `@param`, `@return`. `@brief` tags should be at most 2 lines long. Function comments in source (`.c`/`.cpp`) files should include an `@details` tag describing any details that would be helpful to a developer reading the code.
3. Code examples embedded within Doxygen comments must either use ticks "`", or @code @endcode to denote code.
4. Before complicated lines or blocks of code, write comments inline with the code to help explain what the code is doing (using either `//` or `/*` syntax).
5. Doxygen-style comments must be started as a C-style comment block with two as such: `/**`. Example:

    ```C++
    /**
    * @brief  Gets the day of the week
    * @return The day of the week
    */
    days_t Date::getDay(){
        return theDay;
    }
    ```

6. All structs and enums must have their own documentation, and their members should usually be documented too. The documentation of a struct or enum should precede the definition of the struct or enum, and should use the @brief tag. For documenting members of a struct or enum, the comment must be added right after the member using the `<` marker after opening the comment block. Example:

    ```C++
    /** @brief Enumerates the names of the week */
    typedef enum{
        MONDAY,    /**< The first day of the week     */
        TUESDAY,   /**< The second day of the week    */
        WEDNESDAY, /**< The third day of the week     */
        THURSDAY,  /**< The fourth day of the week    */
        FRIDAY,    /**< The fifth day of the week     */
        SATURDAY,  /**< The first day of the weekend  */
        SUNDAY     /**< The second day of the weekend */
    }days_e;
    ```

    In this case it would have been okay to omit the member documentation since it does not add anything. However, if the enum was used to enumerate something less obvious, such as states of a controller, this omission would not have been acceptable.

- ([Example C++ folder](Templates/cpp))
- ([Example C folder](Templates/c))

## TODO comments

Whenever you think of something that needs to be done sometime in the future, write a "TODO" comment so that we won't forget about it. Any comment beginning with "TODO" catches Eclipse's attention, so it is easy to track these and come back to them. Also, put your name after the TODO so that if someone sees it in the future they'll know who to ask if they have any questions. Include any other relevant information that can help give context, such as a link to a GitHub issue, or a date or event it needs to be done by.

Example:

```C++
uint8_t foo = -1; // TODO(tyler): Foo should not be assigned a negative value if it is unsigned

```

## Naming

### Naming rules
0. Function and variable names must be descriptive. Common shortenings of words are permitted, as long as they are used consistently (e.g. "pos" for "position", "num" for "number"). Well-known abbreviateions (in the context of this project) may also be used, e.g. "IT" for interrupt, and "DMA" for Direct Memory Access.
1. Never begin the name of anything with an underscore. Such identifiers are generally reserved, and we don't want to risk collision.
2. A name should only be all caps when it is for a constant or macro. The exception is for when the name is very short and rule 0 is satisfied better by having all caps.
3. If you read the name of a bool out loud, it should essentially sound like a claim. For example:

    ```C++
    bool isInitialized = mySpecialFunction();
    if(!isInitialized){
        return;
    }
    ```

4. (**C only**) Function names should be preceded by the name of the module they are associated with followed by an underscore. Example: `Dynamixel_getVoltage` is a function from the Dynamixel module that gets the voltage of an actuator.

### Naming scheme summary

PROPOSED: variables use snake case, functions use lower camel case
PROPOSED: append _ for constructor parameters

This summarizes the formatting applied to names. In the following table, apply the most specific type of name listed.

| Type                                  | Example |
| ------------------------------------- | ------- |
| variable - private member             | `m_num_reads` (prepend "m_" for private members only) |
| variable - parameter to a constructor | `num_reads_` (append "_") |
| variable - common                     | `uart_handle_ptr` |
| function                              | `readBuff` |
| function  - implementation of a class function | `readBuffImpl` (append "Impl") |
| constant                              | `POS_RESOLUTION` |
| class                                 | `CircularDmaBuffer` |
| class - mockable interface            | `UartInterface` ("Interface" is appended) |
| class - hardware implementation of mockable interface | `HalUartInterface` (prepend the name of the main hardware API used, in this case "Hal" for the HAL API) |
| class - mock implementation of mockable interface | `MockUartInterface` (prepend "Mock") |
| enums                                 | `RxParseState` |
| type definition (alias)               | `UARTcmd_t` (the appended "_t" is optional - only to make it obvious it's a type) |
| namespace                             | `some_namespace` |
| file - header                         | `CircularDmaBuffer.h` |
| file - source                         | `CircularDmaBuffer.cpp` (always use .cpp) |
| macro - include guard                 | `SOCCER_EMBEDDED_COMMON_INCLUDE_OS_INTERFACE_H` (see [include guards](#Include-guards)) |
| macro - compiler flag                 | `USE_DEBUG_UART` |

## Formatting

### Line length

Lines should not exceed 80 characters.

Function invocations can be split across several lines, and temporary variables can be introduced instead of chaining methods. Function definition/declaration signatures are treated similarly.

```C++
// Bad
bool success = unnecessarilyLongFunctionNameForNoGoodReason(whatAmIEvenThinking, 42, BIT_MASK, A_FOURTH_ARG, WHOA);

// Good - doesn't exceed 80 characters on any line, and is easier to read
bool success = unnecessarilyLongFunctionNameForNoGoodReason(
    whatAmIEvenThinking,
    42,
    BIT_MASK,
    A_FOURTH_ARG,
    WHOA
);

// Bad
bool unnecessarilyLongFunctionNameForNoGoodReason(const char* what, int num, uint32_t flags, coolEnum target, int& outNum) {
    ...
}

// Good
bool unnecessarilyLongFunctionNameForNoGoodReason(
        const char* what, // 8 space indent
        int num,
        uint32_t flags,
        coolEnum target,
        int& outNum) {
    ... // 4 space indent
}

// Good - alternative
bool unnecessarilyLongFunctionNameForNoGoodReason(const char* what, int num,
                                                  uint32_t flags,
                                                  coolEnum target,
                                                  int& outNum) {
    ...
}
```

- ([Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Function_Declarations_and_Definitions))

### Indentation

Indentations must consist of 4 spaces (no tabs!). We suggest configuring Eclipse to automatically insert 4 spaces when you press the tab key; most other text editors can do this as well.

Regarding curly braces, we follow the `K&R (OTBS)` [style](https://en.wikipedia.org/wiki/Indentation_style#Variant:_1TBS_(OTBS)).

```C++
bool isNegative(int num) {
    if (num < 0) {
        return true;
    } else {
        return false;
    }
}
```

## C/C++ features

### Memory
1. Heap usage is disallowed. Objects may either have static storage duration (i.e. allocated at compile time), or be allocated on the stack

### nullptr and NULL

Use nullptr for pointers in C++, NULL in C:
https://google.github.io/styleguide/cppguide.html#0_and_nullptr/NULL

### Namespaces (C++ only)

1. The using directive must not be used, but using declarations may be used. Using directives introduce all the names from a namespace into the current scope, while using declarations introduce only a specific name.

    ```C++
    using namespace i2c; // Bad (using directive)
    using i2c::MPU6050; // Good (using declaration)
    ```

2. Header files must not have anonymous namespaces
3. All functions and data structures besides main should not reside in the global namespace. Instead, they should be in named or anonymous namespaces


### Object-oriented (C++ only)

1. Constructors must initialize their objects to a valid internal state
2. Constructors must not perform any I/O
3. If initialization of the class is error-prone, consider using an `Init()` [method](https://google.github.io/styleguide/cppguide.html#Doing_Work_in_Constructors)
4. The layout of class and struct members/methods based on visibility must be like so:

    ```C++
    class Example{
    public:
    ...
    protected:
    ...
    private:
    ...
    };

    ```

### Const-qualified function arguments and class methods (C++ only)

1. Any class method which does not modify members of that class should be const-qualified
2. Any function arguments which are not modified by the function should be const-qualified

### Binary data

1. Binary data should be represented using the stdint types (e.g. uint8_t, etc.)

### Constants, macros, and inline functions

1. Macros must only be used to conditionally compile code or conditionally include files (i.e. include guard). On that note, each header file must begin with a unique include guard. Exception: see rule 2
2. (**C only**) The rare exception for which a macro is permitted for data is when you need to initialize something at compile-time that does not allow initialization with a constant
3. (**C++ only**) constexpr must be used in place of macros, and is preferred to const
4. Inline functions must be used in place of macro functions as they are type-safe


### Enums

1. Prefer to use enums wherever it makes sense in terms of enhancing readability
2. (**C++ only**) Prefer to use enum classes over regular enums

    ```C++
    /** @brief Colors of the rainbow */
    enum class Colors{
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        INDIGO,
        VIOLET
    }

    ...
    // Usage
    switch(myColor){
        case Colors::RED:
            doSomething();
            break;
        ...
        default:
            break;
    }
    ```

### Scope qualifiers

1. Never allocate storage for a variable in a header file; this creates many issues that have to do with variables being redefined, etc. Instead, allocate variables in the .c file of the module the header is related to, and declare the variable in the header using the keyword extern. **Note: keep in mind that global variables are generally bad practice, so usage of this technique should be rather limited**.

    Example:

    module.c/.cpp
    ```C++
    ...
    uint8_t importantNumber;
    const SOME_CONST = 42;
    ...
    ```
    module.h
    ```C++
    ...
    extern uint8_t importantNumber;
    extern const SOME_CONST;
    ...
    ```

2. **C only:** If a variable or function can be made static, it should be static. When static variables and functions are defined in the scope of a file, it makes them "private" in the sense that they can only be seen from within the file (or any file which includes it). Having private data and functions is often useful. Example: by making sensitive data static, you protect it from being tampered with from outside the library which is using it, thus reducing the probability that the code will be misused. **C++ only:** same thing as above, but for private class members. **The spirit of these scope rules is to minimize the interfaces you expose across the program**


### Typecasting

1. (**C++ only**) Prefer to use static_cast\<\>() over C-style typecasting. Example:

    ```C++
    char initial = 'T';

    // Good
    uint8_t byte = static_cast<uint8_t>(initial);

    // Not as good
    uint8_t byte = (uint8_t)initial;
    ```

## Errors

1. Where it is possible for anything to go wrong in a function, that function should return status codes

## Unit testing

TODO: link to wiki on mockable interfaces

1. All hardware and OS-related functions must be in wrapper classes which inherit from an interface class (i.e. abstract class) to facilitate mocking. These hardware-facing classes should not have data members, only functions.
2. The test driver for a component must follow a name of the form \<component\>_test.cpp (TODO: move to naming section)

## General Development

### Compiler warnings

1. Any code written by us should be free of compiler warnings. Any exception should be documented

### Dead code

1. Delete unused variables (i.e. if the compiler issues a warning that a variable is unused, you better have a good reason for keeping it around). This does not hold for 3rd party code we are using, it only holds for code we write 
2. If there is code that is now outdated and will never be used, it should be deleted
3. Any conditionally-executed code whose conditions will never be satisfied must be removed

### Code generation

TODO: remove this section once fully migrated away from Cube

1. The configuration of all peripherals must be done from within Cube
2. Pin names should be labelled in Cube once their functionality has been decided
3. All FreeRTOS things that _can_ be configured in Cube _should_ be configured in Cube. Examples of things that cannot be configured in Cube are event groups and queue sets. This rule _can_ be bypassed for other things (e.g. mutexes), as long as there is a sufficiently good reason that the team supports, and the scenario is documented
