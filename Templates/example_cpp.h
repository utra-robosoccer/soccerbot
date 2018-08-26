/**
  *****************************************************************************
  * @file    example_cpp.h
  * @author  TODO -- your name here
  * @brief   TODO -- briefly describe this file (if necessary)
  *****************************************************************************
  */




#ifndef __EXAMPLE_CPP_H__
#define __EXAMPLE_CPP_H__




/********************************* Includes **********************************/
#include <stdint.h>




/********************************* Namespace *********************************/
/**
 * @addtogroup Namespace
 * @ingroup Parent_Module_Name
 *
 * @brief   Brief namespace (module) description
 * @details Detailed namespace (module) description
 *
 * @{
 */
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
