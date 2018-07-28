/**
  *****************************************************************************
  * @file    example_cpp.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file
  *
  * @defgroup TODO -- module name defined in template.c with " (header)" after it
  * @brief    TODO -- brief description of header
  * @ingroup  TODO -- module name defined in template.c
  * @{
  *****************************************************************************
  */




/******************** Define to prevent recursive inclusion ******************/
#ifndef __EXAMPLE_CPP_H__
#define __EXAMPLE_CPP_H__




/********************************* Includes **********************************/
#include <stdint.h>




/********************************** Macros ***********************************/



/********************************* Namespace *********************************/
/**
 *  \addtogroup Namespace
 *  @{
 */
namespace Namespace {
/********************************* Constants *********************************/




/********************************** Classes **********************************/
//!  A test class. 
/*!
  A more elaborate class description.
*/
class Class {

public:
    //! A constructor.
    /**
      A more elaborate description of the constructor.
    */
    Class();
    //! A destructor.
    /**
      A more elaborate description of the destructor.
    */
   ~Class();
    //! A normal member taking two arguments and returning an integer value.
    /**
      \param a an integer argument.
      \param s a constant character pointer.
      \return The test results
    */
    int testMe(int a,const char *s);       

protected:
            
                       
private:

}; //class Class



} //namespace Namespace
/**
 * @}
 */
/**/

#endif /* __EXAMPLE_CPP_H__ */
