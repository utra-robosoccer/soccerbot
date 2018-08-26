/**
  *****************************************************************************
  * @file    template.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup Header
  * @ingroup  TODO -- module name defined in template.c
  * @{
  *****************************************************************************
  */




#ifndef __EXAMPLE_CPP_H__
#define __EXAMPLE_CPP_H__




/********************************* Includes **********************************/
#include <stdint.h>




/********************************* Namespace *********************************/
namespace Namespace {




/********************************* Constants *********************************/




/********************************** Classes **********************************/
/** 
 * @class   Class
 * @brief   Short class description
 * @details Detailed class description
 */
class Class {
public:
    Class();
   ~Class();

    /** 
     * @brief  Member description here
     * @param  a description of the significance of a
     * @param  s description of the significance of s
     * @return The test results
     */
    int testMe(int a, const char* s);

protected:

private:

}; // class Class

} // namespace Namespace

/**
 * @}
 */
/* end - Namespace */

#endif /* __EXAMPLE_CPP_H__ */
