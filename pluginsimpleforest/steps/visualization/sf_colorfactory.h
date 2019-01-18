#ifndef SF_COLORFACTORY_H
#define SF_COLORFACTORY_H

#include <ct_color.h>

class SF_ColorFactory
{
  CT_Color getColor(Color clr);

private:
  SF_ColorFactory();
  enum class Color
  {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    VIOLET,
    CYAN,
    BRIGHT,
    DARK
  };
};

#endif // SF_COLORFACTORY_H
